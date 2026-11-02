# NeuroPhysics — Detailed Technical Architecture

Architecture design · C++ / Raylib / Physics Simulation / Reinforcement Learning / Deep Learning

We're going to approach this as a proper software engineering project, not just attach a Python reinforcement learning script to your physics engine.

The objective is to transform your existing C++/Raylib rigid-body and fluid simulation engine into an AI training and inference platform where reinforcement learning agents learn to interact with your custom physics environment.

We'll first finalize the architecture. Then we'll implement it incrementally, testing each component before moving to the next.

I recommend retaining your existing engine and extending it rather than rewriting it.

# 1. Project vision and final system

![Genesis Physics Simulator 2026](https://images.openai.com/static-rsc-4/nFFzWCszAVNIL5sFAvbtMtgeFupZc8oMpJda56RfNyQ-tqoW5_gmvz3P_WL9O90_QiUXw-q_IvYY6bvxf7KzZH1awcz7b2muYP6CCeD5gnxJd2vS5cGWg2F_n1wxA4voy3Lj4n9-ZbkABmu4Wr9Snh4HzwEqykRMkdydVwUZL_s?purpose=inline)

![robocloud-dashboard.vercel.app](https://robocloud-dashboard.vercel.app/learn/blog/genesis-simulator-robotics?utm_source=chatgpt.com)

# NeuroPhysics

An AI-powered 2D physics and fluid simulation laboratory

Custom physics engine + reinforcement learning + neural network inference + real-time visualization.

The final system should support five capabilities:

1. Simulation: Run your custom rigid-body and fluid physics engine.
2. Training: Train AI agents using reinforcement learning.
3. Learning visualization: Watch agents improve through repeated episodes.
4. Inference: Run trained neural networks directly inside your C++ application.
5. Experimentation: Change gravity, friction, mass, fluid properties and other physical parameters to test the agent's adaptability.

The most important design principle:

C++ owns the physics. Python owns the training. The trained neural network can eventually run inside C++.

This separation will make the project modular, easier to debug, and suitable for future expansion.

# 2. High-level technical architecture

The system will consist of six major layers.

## System architecture

Layer 1 — User Interface

Raylib · Simulation controls · Training dashboard · Debug visualization

Layer 2 — AI Agent Runtime

Policy selection · Observation processing · Action execution · ONNX Runtime

Layer 3 — RL Environment

Environment interface · Observation builder · Reward calculator · Episode manager

Layer 4 — Physics Simulation

PhysicsWorld · RigidBody · Collision system · Joints · Fluid system

Layer 5 — Python Training System

Gymnasium · Stable-Baselines3 · PPO · PyTorch · TensorBoard

Layer 6 — Model and Experiment Storage

Model checkpoints · ONNX exports · Training configurations · Evaluation results

Conceptual architecture. During training, Python communicates with the C++ environment. During deployment, C++ can load the exported model directly.

There are actually two operating modes:

| Mode       | Physics execution       | AI execution       |
| ---------- | ----------------------- | ------------------ |
| Training   | C++ headless simulation | Python / PyTorch   |
| Deployment | C++ / Raylib            | C++ / ONNX Runtime |

This avoids needing Python to run your final interactive application.

# 3. Layer-by-layer technical design

## Layer 1 — Existing physics engine (C++)

This is the foundation. We should not unnecessarily modify your existing physics classes.

Your current architecture already includes concepts such as:

- `RigidBody`
- `PhysicsWorld`
- `Shape`
- `Circle`
- `Box`
- `Vec2`
- `Renderer`

We'll organize the engine into the following responsibilities.

Physics Core

Authoritative simulation state

`PhysicsWorld`

Coordinator

- Owns rigid bodies and simulation systems.
- Advances the simulation.
- Controls gravity and world settings.

  Rigid Body System
- Position, velocity, acceleration.
- Mass, inertia and angular velocity.
- Forces, impulses and constraints.

  Collision System
- Broad-phase detection.
- Narrow-phase detection.
- Contact generation.
- Collision resolution.

  Fluid System
- Particle storage.
- Neighbor searches.
- Density, pressure and viscosity, depending on your fluid algorithm.

  Rendering System
- Raylib drawing.
- Debug visualization.
- AI observation and action overlays.

### Important architectural modification: headless simulation

Currently, your application probably follows a loop resembling:

```
while (!WindowShouldClose())
{
    world.Step();

    BeginDrawing();
    renderer.Draw(world);
    EndDrawing();
}
```

For reinforcement learning, we need to separate the simulation from the window.

The physics engine must support:

```
world.Reset();
world.Step(fixedDeltaTime);
```

without requiring:

- `BeginDrawing()`
- `EndDrawing()`
- `WindowShouldClose()`
- `DrawCircle()`
- Any rendering operations.

The renderer becomes a consumer of the physics state, not a dependency of the physics engine.

Why?

During training, an agent may need millions of simulation steps. Opening a graphics window and rendering every step wastes computational resources.

We'll retain Raylib for human-visible simulation and use headless execution for training.

## Layer 2 — Reinforcement learning environment (C++)

This is the most important new C++ subsystem.

It converts your physics engine into an environment that an AI agent can interact with.

We'll introduce a separate environment abstraction.

### Proposed class architecture

```

class RLEnvironment
{
public:
    virtual ~RLEnvironment() = default;

    virtual std::vector<float> Reset(
        unsigned int seed
    ) = 0;

    virtual RLStepResult Step(
        const std::vector<float>& action
    ) = 0;

    virtual std::vector<float> GetObservation() const = 0;
};

```

The return structure:

```
struct RLStepResult
{
    std::vector<float> observation;

    float reward = 0.0f;

    bool terminated = false;
    bool truncated = false;
};
```

Then we create specialized environments:

```
RLEnvironment
     |
     |-----------------------
     |          |           |
 BallReach   CartPole    RobotArm
   Env         Env         Env
```

Each environment uses the same underlying `PhysicsWorld`, but defines its own task.

### Responsibilities of an RL environment

| Component   | Responsibility                       |
| ----------- | ------------------------------------ |
| Reset       | Initialize the environment           |
| Observation | Extract useful physics data          |
| Action      | Apply the agent's commands           |
| Reward      | Evaluate agent behavior              |
| Termination | Determine task success or failure    |
| Truncation  | Stop an episode after its time limit |

This abstraction is essential. It allows you to introduce new RL tasks without changing the PPO training code.

## Layer 3 — Observation, action and reward system

This layer translates physical simulation into mathematical information the neural network understands.

Let's use the first environment: `BallReachEnv`.

[making a ball pushing game for GA training | by Metta Ong | Medium](https://images.openai.com/static-rsc-4/29QL1pAyRC72_c431TUSQS_CKYF9SChHK6Tf8XtxWMRw7eaS3mE7LejZ1IMDKPtacoBR7ypHnu2Hb7-w05gJMIiXXdUIE4Jv1xU3hffqxtFBPp84rrjZV_A4BgGY8TnmB4Qjgb0mfnqnXrk3wZhJ6sI7GR44IHz5nTcL9sEahMw?purpose=inline)

[medium.com](https://medium.com/%40ongspxm/making-a-ball-pushing-game-for-ga-training-adce07e79f7?utm_source=chatgpt.com)

### A. Observation space

Suppose a ball must reach a target.

The agent receives:

| Observation | Meaning                     |
| ----------- | --------------------------- |
| Ball X      | Current horizontal position |
| Ball Y      | Current vertical position   |
| Velocity X  | Horizontal velocity         |
| Velocity Y  | Vertical velocity           |
| Target X    | Target horizontal position  |
| Target Y    | Target vertical position    |
| Distance X  | Horizontal target distance  |
| Distance Y  | Vertical target distance    |

The observation vector:

\\[ s_t = [x,y,v_x,v_y,x_g,y_g,\Delta x,\Delta y] \\]

This is an eight-dimensional continuous observation space.

We'll normalize these values before giving them to the neural network. For example, positions can be scaled relative to the world dimensions.

### B. Action space

The agent should be able to apply horizontal and vertical forces.

$ a_t=[a_x,a_y] $

Each action is normalized:

$ a_x,a_y \in [-1,1] $

The environment converts the normalized action into forces:

$F_x=a_xF\_{\max}$

$F_y=a_yF\_{\max}$

This makes the action space compatible with PPO's continuous-control policy.

### C. Reward system

The agent needs a reason to move toward the target.

One possible reward:

$r_t = w_p(d\_{t-1}-d_t) -w_v\\|v_t\\|^2 +r\_{\text{success}}$

Where:

- \\(d_t\\) is the current distance to the target.
- The first term rewards progress.
- The second term discourages excessive velocity.
- The success bonus rewards reaching the target.

We'll tune the reward empirically rather than assuming one formula will work for every task.

### D. Episode termination

For example:

- Success: ball reaches the target region.
- Failure: ball leaves the permitted world.
- Truncation: maximum episode length reached.

This gives the agent a clear objective.

## Layer 4 — C++ to Python communication

This is where `pybind11` comes in.

We do not want to reimplement the physics engine in Python.

Instead, Python calls the existing C++ environment.

Python training code

```
observation, info = env.reset()observation, reward, terminated, truncated, info = env.step(action)
```

pybind11 bridge

Converts Python calls and data into C++ calls and values.

C++ environment

```
env.Reset(seed);
env.Step(action);
env.GetObservation();
```

The C++ binding will expose environment creation and operations.

Conceptually:

```
PYBIND11_MODULE(neurophysics, m)
{
    // Expose environment classes and methods.
}
```

The actual binding implementation will come later, once the C++ interface is stable.

### Why pybind11 instead of a REST API?

For this project, pybind11 is preferable because:

- No HTTP server is required.
- No JSON serialization for every simulation step.
- Python can call C++ directly.
- Data conversion overhead is relatively small.
- You can retain a single C++ physics implementation.

We should keep the Python interface relatively coarse-grained: one call to `step()` should advance multiple internal physics substeps, rather than crossing the language boundary for every tiny physics calculation.

## Layer 5 — Python training and deep learning

This is where the actual learning happens.

### Selected algorithm: PPO

Recommended starting algorithm

We'll use Proximal Policy Optimization.

PPO is a good starting point for continuous-control environments because it is relatively stable, widely used, and supported by Stable-Baselines3.

### Neural network architecture

For the initial ball-reaching task, a small multilayer perceptron is sufficient.

## PPO policy network

Input layer

8 observation values

Hidden layer 1

64 neurons · Tanh activation

Hidden layer 2

64 neurons · Tanh activation

Actor

Action distribution

Critic

State value estimate

The actor decides what action to take. The critic estimates how valuable the current state is.

PPO updates both networks based on collected experience.

### Training pipeline

```
Initialize PPO
     |
     v
Get observation
     |
     v
Actor selects action
     |
     v
C++ physics simulation
     |
     v
Receive reward and next state
     |
     v
Store experience
     |
     v
Update Actor and Critic
     |
     v
Repeat
```

### Training infrastructure

| Component              | Tool                 |
| ---------------------- | -------------------- |
| RL algorithm           | PPO                  |
| Framework              | Stable-Baselines3    |
| Neural networks        | PyTorch              |
| Environment interface  | Gymnasium            |
| Training visualization | TensorBoard          |
| Experiment tracking    | CSV / JSON initially |
| Model checkpoints      | Stable-Baselines3    |

One important distinction: PPO training will initially run in Python, while your custom physics simulation will execute in compiled C++.

Your RTX 4060 can accelerate the neural network training, but for these relatively small policies, CPU training may sometimes be faster because environment stepping is likely to dominate the workload.

## Layer 6 — Model deployment and C++ inference

After training, we need to get the trained agent back into your Raylib application.

We will use ONNX as the interchange format and ONNX Runtime for inference.

PyTorch PPO model

Trained policy and neural network weights

ONNX export

Export the policy network, not the training algorithm.

ONNX Runtime in C++

Load policy and calculate actions.

Raylib simulation

AI controls your objects in real time.

The deployed agent's loop:

```
while (!WindowShouldClose())
{
    auto observation = env.GetObservation();

    auto action = policy.Predict(observation);

    env.ApplyAction(action);

    world.Step(fixedDeltaTime);

    renderer.Draw(world);
}
```

This is conceptual code; the actual `Predict()` and environment APIs will be implemented separately.

A subtle but important detail: the deployed policy must use exactly the same observation ordering, normalization and action scaling used during training. Otherwise, the agent may behave incorrectly even when the ONNX model is valid.

# 4. How your existing features fit into NeuroPhysics

We should not immediately give the RL agent control of every part of the simulation.

Instead, use a progression of increasing complexity.

| Existing feature  | Initial RL integration   | Future RL integration              |
| ----------------- | ------------------------ | ---------------------------------- |
| Gravity           | Fixed during training    | Randomized gravity                 |
| Circle            | Controlled ball          | Multi-object manipulation          |
| Box               | Static obstacle          | Dynamic stacking                   |
| Rotation          | Observe angular state    | Control angular velocity           |
| Mouse interaction | Manual environment setup | Human demonstration collection     |
| Fluid simulation  | Initially disabled       | AI swimming and fluid manipulation |
| Collision system  | Physical environment     | Contact-rich control tasks         |

Your existing gravity toggle can become a particularly good demonstration feature: test whether an agent trained under different gravity settings can adapt.

But we should first establish a stable, deterministic baseline.

# 5. Repository and module structure

I recommend this eventual structure.

```

NeuroPhysics/
│
├── Engine/
│   ├── Core/
│   │   ├── Vec2.h
│   │   ├── RigidBody.h
│   │   ├── RigidBody.cpp
│   │   ├── PhysicsWorld.h
│   │   └── PhysicsWorld.cpp
│   │
│   ├── Collision/
│   │   ├── Collision.h
│   │   ├── Collision.cpp
│   │   └── SpatialGrid.h
│   │
│   ├── Shapes/
│   │   ├── Shape.h
│   │   ├── Circle.h
│   │   └── Box.h
│   │
│   ├── Fluid/
│   │   ├── FluidWorld.h
│   │   └── FluidWorld.cpp
│   │
│   └── Rendering/
│       ├── Renderer.h
│       └── Renderer.cpp
│
├── AI/
│   ├── Core/
│   │   ├── RLEnvironment.h
│   │   └── RLStepResult.h
│   │
│   ├── Environments/
│   │   ├── BallReachEnv.h
│   │   ├── BallReachEnv.cpp
│   │   ├── CartPoleEnv.h
│   │   └── CartPoleEnv.cpp
│   │
│   ├── Bindings/
│   │   └── python_bindings.cpp
│   │
│   └── Inference/
│       ├── Policy.h
│       └── ONNXPolicy.cpp
│
├── Python/
│   ├── environments/
│   │   └── gym_env.py
│   │
│   ├── training/
│   │   ├── train_ppo.py
│   │   └── evaluate.py
│   │
│   ├── callbacks/
│   │   └── training_callback.py
│   │
│   └── export/
│       └── export_onnx.py
│
├── Models/
│   ├── checkpoints/
│   └── onnx/
│
├── Experiments/
│   ├── configs/
│   ├── results/
│   └── logs/
│
├── Tests/
│   ├── PhysicsTests.cpp
│   ├── EnvironmentTests.cpp
│   └── ObservationTests.cpp
│
├── CMakeLists.txt
└── main.cpp

```

This is the target structure, not something we need to create all at once.

# 6. Important engineering decisions

Before implementation, I want us to establish these rules.

| Decision            | Architecture choice | Reason                           |
| ------------------- | ------------------- | -------------------------------- |
| Physics ownership   | C++                 | Reuse your existing engine       |
| Training            | Python              | Mature RL ecosystem              |
| RL algorithm        | PPO                 | Good continuous-control baseline |
| Integration         | pybind11            | Low-overhead C++/Python bridge   |
| Simulation          | Fixed timestep      | Consistent physics               |
| Training renderer   | Disabled            | Higher simulation throughput     |
| Inference           | ONNX Runtime        | Run trained policy in C++        |
| Environment design  | Modular classes     | Support multiple tasks           |
| Experiment tracking | TensorBoard         | Monitor learning                 |

### Four technical requirements we must not skip

1\. Deterministic reset

Given the same random seed and actions, the environment should produce repeatable results, within the limits of your floating-point implementation.

2\. Action safety

Every AI action must be clamped and validated before it reaches your physics engine. An agent must not be able to generate unlimited forces.

3\. Simulation timing

RL action frequency and physics timestep should be separate concepts.

For example, the agent may choose an action every 1/30 second while physics advances at 1/120 second. That means four physics steps for every action.

4\. Thread safety

Initially, keep one environment instance per Python worker and avoid sharing mutable `PhysicsWorld` objects across training threads.

When we add parallel environments, each instance must own its own simulation state.

# 7. Implementation order and dependency graph

This is the order I want us to follow. Each phase has a completion condition. We will not move to the next phase until the current one works.

## Development sequence

1. Phase 0 — Audit existing engine
   - Understand your existing classes.
   - Check `PhysicsWorld::Step()`.
   - Check gravity, forces and collision handling.
   - Identify missing physics capabilities.

     Foundation
2. Phase 1 — Headless physics
   - Separate rendering from simulation.
   - Implement reliable world reset.
   - Introduce fixed timestep.
   - Test deterministic execution.

     Completion: run simulation without a Raylib window.
3. Phase 2 — RL environment interface
   - Create `RLEnvironment`.
   - Create `RLStepResult`.
   - Implement `BallReachEnv`.
   - Define observations, actions and rewards.

     Completion: manually control the environment through C++ calls.
4. Phase 3 — Python integration
   - Install Python dependencies.
   - Configure pybind11.
   - Create Python bindings.
   - Build Gymnasium wrapper.

     Completion: control your C++ simulation from Python.
5. Phase 4 — PPO training
   - Configure PyTorch and Stable-Baselines3.
   - Train initial agent.
   - Monitor reward progression.
   - Evaluate against random actions.

     Completion: agent reliably reaches the target.
6. Phase 5 — C++ AI inference
   - Export policy to ONNX.
   - Integrate ONNX Runtime.
   - Execute the policy in Raylib.

     Completion: AI controls the simulation without Python.
7. Phase 6 — Advanced RL environments
   - Implement CartPole.
   - Add joints and constraints if required.
   - Implement robotic arm or walker.

     Completion: multiple environments use the same training pipeline.
8. Phase 7 — Portfolio presentation
   - AI training dashboard.
   - Model selection.
   - Reward graphs.
   - Replay and comparisons.
   - Performance benchmarks.

     Final deliverable

# 8. What we are deliberately postponing

To avoid making this project unnecessarily complicated, we won't initially implement:

- GPU-based reinforcement learning.
- Distributed training.
- Multi-agent reinforcement learning.
- Neural fluid prediction.
- AI-generated simulation scenes.
- Reinforcement learning for every rigid body simultaneously.

These are potential future research extensions.

First, we need one working AI agent inside your own physics engine.

# 9. Development environment

Your existing Windows 11 and RTX 4060 setup is sufficient for this architecture.

The intended development tools are:

| Tool                    | Purpose                      |
| ----------------------- | ---------------------------- |
| Visual Studio / VS Code | C++ development              |
| CMake                   | Build system                 |
| C++17 or C++20          | Physics and environment code |
| Raylib                  | Visualization                |
| Python 3.11             | RL training environment      |
| PyTorch                 | Neural networks              |
| Gymnasium               | RL environment interface     |
| Stable-Baselines3       | PPO implementation           |
| pybind11                | Python/C++ integration       |
| TensorBoard             | Training visualization       |
| ONNX Runtime            | C++ inference                |

We'll choose compatible library versions when setting up the project, rather than installing everything blindly at once.

Useful documentation:

- [pybind11 documentation](https://pybind11.readthedocs.io/en/stable/?utm_source=chatgpt.com)
- [Gymnasium documentation](https://gymnasium.farama.org/?utm_source=chatgpt.com)
- [Stable-Baselines3 documentation](https://stable-baselines3.readthedocs.io/en/master/?utm_source=chatgpt.com)
- [PyTorch documentation](https://pytorch.org/docs/stable/index.html?utm_source=chatgpt.com)
- [ONNX Runtime documentation](https://onnxruntime.ai/docs/?utm_source=chatgpt.com)

# 10. Our first milestone: audit and prepare your existing engine

Next step

Before writing any reinforcement learning code, I want to understand the exact state of your physics engine.

I already have some context about your `RigidBody`, `PhysicsWorld`, `Shape`, `Circle`, `Box`, `Renderer`, and `Vec2` architecture. I also know you've worked on object grabbing, gravity toggling and fluid integration. We'll preserve that work.&#x20;

I don't want to give you code that conflicts with your existing implementation.

Please share these files first:

Files for the initial architecture audit

Select the files you can share. We can inspect the remaining files later.

PhysicsWorld.h

PhysicsWorld.cpp

RigidBody.h

RigidBody.cpp

Vec2.h

main.cpp

Renderer.h / Renderer.cpp

Fluid simulation files (if integrated)

Confirm files

You can upload the files together or share the source code in messages.

What I'll do with them:

1. Map your current class architecture.
2. Identify what can be reused without modification.
3. Identify what needs to change for headless execution.
4. Design the exact RL environment interfaces around your code.
5. Give you the first implementation task with complete code and explanations.

Our working rule throughout this project: one implementation step, one test, one confirmation, then the next step.

We will build NeuroPhysics incrementally, and by the end you'll have a custom C++ physics engine with an actual trained reinforcement learning agent controlling it.