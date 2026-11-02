# 2D Rigid Body Physics Engine: Collisions, Rotation, and Implementation Guide

An exhaustive theoretical and practical reference for 2D rigid body mechanics, contact generation, impulse-based collision resolution, rotational dynamics, and the complete source code architecture of this engine.

---

## Table of Contents
1. [Coordinate System & Mathematical Conventions](#1-coordinate-system--mathematical-conventions)
2. [Rigid Body Dynamics & Kinematics](#2-rigid-body-dynamics--kinematics)
   - 2.1 State Variables
   - 2.2 Moment of Inertia (Circle and Box)
   - 2.3 Numerical Integration (Semi-Implicit Euler)
   - 2.4 Point Velocity & 2D Torque
3. [The Impulse-Based Resolution Framework](#3-the-impulse-based-resolution-framework)
   - 3.1 Contact Physics & Newton's Law of Restitution
   - 3.2 Full Derivation of Linear & Angular Impulse
   - 3.3 Position Correction (Baumgarte / Linear Projection)
4. [Circle vs Circle Collision](#4-circle-vs-circle-collision)
   - 4.1 Detection & Normal Generation
   - 4.2 Impulse Resolution & Zero Torque Principle
   - 4.3 Code Walkthrough
5. [Circle vs Box Collision](#5-circle-vs-box-collision)
   - 5.1 Local Space Coordinate Transformation
   - 5.2 Voronoi Regions & Closest Point on OBB
   - 5.3 Contact Point & Rotational Torque Generation
   - 5.4 Code Walkthrough
6. [Box vs Box Collision (Separating Axis Theorem - SAT)](#6-box-vs-box-collision-separating-axis-theorem---sat)
   - 6.1 The Separating Axis Theorem (SAT)
   - 6.2 Testing 4 Separating Axes
   - 6.3 Contact Manifold & Point Generation
   - 6.4 Dual-Body Rotational Impulse Resolution
   - 6.5 Code Walkthrough
7. [Ground Collision & Multi-Contact Stabilization](#7-ground-collision--multi-contact-stabilization)
   - 7.1 Single-Corner Instability Problem
   - 7.2 Multi-Point Ground Contact Solution
   - 7.3 Resting Contacts & Angular Damping
   - 7.4 Code Walkthrough
8. [Rendering & ASCII Rasterization](#8-rendering--ascii-rasterization)
   - 8.1 Raylib Rotated Box Rendering
   - 8.2 ASCII Terminal Rasterization & Inverse Rotation
9. [Summary Architecture & Verification](#9-summary-architecture--verification)

---

## 1. Coordinate System & Mathematical Conventions

### Screen Space Coordinates
In computer graphics (including Raylib and terminal grids), the Y-axis points **downward**:
```
(0, 0) ───────────────► +X (Right)
  │
  │
  ▼
 +Y (Down)
```

- **Position $\vec{p}$**: $(x, y)$ in pixels.
- **Velocity $\vec{v}$**: $(v_x, v_y) = \frac{d\vec{p}}{dt}$.
- **Angle $\theta$**: Measured in **radians**.
- **Rotation Direction**:
  - In a right-handed system where $+X$ is Right and $+Y$ is Down, the cross-product $+X \times +Y$ points **into the screen** ($+Z$).
  - Therefore, positive angular velocity $\omega > 0$ corresponds to **Clockwise (CW)** rotation.
  - Negative angular velocity $\omega < 0$ corresponds to **Counter-Clockwise (CCW)** rotation.

### 2D Rotation Matrix
Rotating a vector $\vec{v} = (x, y)$ by an angle $\theta$ in screen space:
$$\begin{bmatrix} x' \\ y' \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} x \\ y \end{bmatrix}$$
$$x' = x \cos\theta - y \sin\theta$$
$$y' = x \sin\theta + y \cos\theta$$

For **inverse rotation** (transforming world space into a body's local space rotated by $\theta$), we rotate by $-\theta$:
$$x_{\text{local}} = x \cos\theta + y \sin\theta$$
$$y_{\text{local}} = -x \sin\theta + y \cos\theta$$

---

## 2. Rigid Body Dynamics & Kinematics

### 2.1 State Variables
Each `RigidBody` in the engine maintains the following physical quantities:

| Variable | Symbol | Units | Definition |
| :--- | :--- | :--- | :--- |
| `position` | $\vec{p}$ | pixels | Center of mass position $(x, y)$ |
| `velocity` | $\vec{v}$ | pixels/s | Linear velocity vector $(v_x, v_y)$ |
| `force` | $\vec{F}$ | N (pixel-mass units) | Net linear force accumulator |
| `mass` | $m$ | kg | Mass of the rigid body |
| `invMass` | $1/m$ | $\text{kg}^{-1}$ | Inverse mass ($0$ if static / infinite mass) |
| `angle` | $\theta$ | rad | Orientation angle (radians) |
| `angularVelocity` | $\omega$ | rad/s | Angular speed about the center of mass |
| `torqueAccum` | $\tau$ | $\text{N}\cdot\text{pixels}$ | Net torque accumulator |
| `inertia` | $I$ | $\text{kg}\cdot\text{pixels}^2$ | Moment of inertia about the center of mass |
| `restitution` | $e$ | unitless $[0, 1]$ | Coefficient of restitution (bounciness) |

---

### 2.2 Moment of Inertia (Circle and Box)

The moment of inertia $I$ measures an object's resistance to angular acceleration about its center of mass:
$$I = \iint r^2 \, dm$$

#### 1. Solid Disc (Circle)
For a circle of uniform density with mass $m$ and radius $R$:
$$I = \frac{1}{2} m R^2$$
In `RigidBody.cpp`:
```cpp
inertia = 0.5f * this->mass * r * r;
```

#### 2. Solid Rectangle (Box)
For a box of uniform density with mass $m$, width $W$ (along local X), and height $H$ (along local Y):
$$I = \frac{1}{12} m (W^2 + H^2)$$
In `RigidBody.cpp`:
```cpp
inertia = this->mass * (w * w + h * h) / 12.0f;
```

---

### 2.3 Numerical Integration (Semi-Implicit Euler)

The engine updates the physics state using **Semi-Implicit Euler** (also called Symplectic Euler). This method updates velocity first, then uses the updated velocity to update position, providing superior energy conservation compared to explicit Euler.

#### Linear Update:
$$\vec{a} = \frac{\vec{F}_{\text{accum}}}{m}$$
$$\vec{v}(t + \Delta t) = \vec{v}(t) + \vec{a} \Delta t$$
$$\vec{p}(t + \Delta t) = \vec{p}(t) + \vec{v}(t + \Delta t) \Delta t$$

#### Angular Update:
$$\alpha = \frac{\tau_{\text{accum}}}{I}$$
$$\omega(t + \Delta t) = \omega(t) + \alpha \Delta t$$
$$\theta(t + \Delta t) = \theta(t) + \omega(t + \Delta t) \Delta t$$

#### Code Implementation (`RigidBody::Integrate`):
```cpp
void RigidBody::Integrate(float dt)
{
    if (dt <= 0.0f) return;

    // Linear integration
    Vec2 acceleration = force / mass;
    velocity += acceleration * dt;
    position += velocity * dt;

    // Angular integration
    if (inertia > 0.0001f) {
        const float angularAcceleration = torqueAccum / inertia;
        angularVelocity += angularAcceleration * dt;
    }
    angle += angularVelocity * dt;

    // Clear accumulators for the next frame
    force = Vec2{0.0f, 0.0f};
    torqueAccum = 0.0f;
}
```

---

### 2.4 Point Velocity & 2D Torque

#### Velocity of a Point on a Rotating Rigid Body
Consider a point $P$ on a rigid body with center of mass at $\vec{p}_{\text{cm}}$. The vector from the center of mass to $P$ is $\vec{r} = P - \vec{p}_{\text{cm}} = (r_x, r_y)$.

In 3D with angular velocity vector $\vec{\omega} = (0, 0, \omega)$:
$$\vec{v}_P = \vec{v}_{\text{cm}} + \vec{\omega} \times \vec{r}$$
Expanding the 3D cross-product with $\vec{r} = (r_x, r_y, 0)$:
$$\vec{\omega} \times \vec{r} = \begin{vmatrix} \hat{i} & \hat{j} & \hat{k} \\ 0 & 0 & \omega \\ r_x & r_y & 0 \end{vmatrix} = (-\omega r_y) \hat{i} + (\omega r_x) \hat{j}$$

Therefore, in 2D coordinates:
$$\vec{v}_P = \left( v_x - \omega r_y, \; v_y + \omega r_x \right)$$

#### 2D Torque Definition
When a force $\vec{F} = (F_x, F_y)$ is applied at an offset $\vec{r} = (r_x, r_y)$:
$$\vec{\tau} = \vec{r} \times \vec{F} = r_x F_y - r_y F_x$$
In `RigidBody::ApplyForceAtPoint`:
```cpp
void RigidBody::ApplyForceAtPoint(const Vec2& appliedForce, const Vec2& worldPoint)
{
    ApplyForce(appliedForce);
    const Vec2 r = worldPoint - position;
    const float torque = r.x * appliedForce.y - r.y * appliedForce.x;
    ApplyTorque(torque);
}
```

---

## 3. The Impulse-Based Resolution Framework

Instead of computing forces via stiff springs (penalty methods), the engine uses **instantaneous momentum impulses**.

```
 Body A (Mass mA, Inertia IA)                   Body B (Mass mB, Inertia IB)
   Pos: positionA                                Pos: positionB
          \                                             /
           \                  Contact Point P          /
            \                       ●                 /
             ───► rA = P - posA          rB = P - posB ◄───
                                    ▲
                                    │ normal (A -> B)
```

### 3.1 Contact Physics & Newton's Law of Restitution
Let $\vec{n}$ be the unit normal vector pointing from **Body A to Body B**.
Let $P$ be the contact point in world coordinates.
- $\vec{r}_A = P - \vec{p}_A$ (lever arm for Body A)
- $\vec{r}_B = P - \vec{p}_B$ (lever arm for Body B)

Contact point velocities:
$$\vec{v}_{AP} = \vec{v}_A + (-\omega_A r_{Ay}, \; \omega_A r_{Ax})$$
$$\vec{v}_{BP} = \vec{v}_B + (-\omega_B r_{By}, \; \omega_B r_{Bx})$$

Relative velocity at the contact point:
$$\vec{v}_{\text{rel}} = \vec{v}_{BP} - \vec{v}_{AP}$$

Velocity along the collision normal:
$$v_n = \vec{v}_{\text{rel}} \cdot \vec{n}$$
- If $v_n \ge 0$: The bodies are already separating. **No impulse is required**.
- If $v_n < 0$: The bodies are colliding.

Newton's collision rule states:
$$v_n' = -e \, v_n$$
where $e = \min(e_A, e_B)$ is the restitution coefficient, and $v_n'$ is the post-collision normal relative velocity.

---

### 3.2 Full Derivation of Linear & Angular Impulse

An impulse of magnitude $j \ge 0$ is applied along $\vec{n}$:
- On **Body B**: Impulse $\vec{J}_B = +j \vec{n}$
- On **Body A**: Impulse $\vec{J}_A = -j \vec{n}$ (Newton's 3rd Law)

#### Change in Velocities:
$$\Delta \vec{v}_A = -\frac{j \vec{n}}{m_A}, \qquad \Delta \vec{v}_B = +\frac{j \vec{n}}{m_B}$$
$$\Delta \omega_A = \frac{\vec{r}_A \times (-j \vec{n})}{I_A} = -\frac{j (\vec{r}_A \times \vec{n})}{I_A}$$
$$\Delta \omega_B = \frac{\vec{r}_B \times (+j \vec{n})}{I_B} = +\frac{j (\vec{r}_B \times \vec{n})}{I_B}$$

Let us define the 2D cross-product scalars:
$$u_A = \vec{r}_A \times \vec{n} = r_{Ax} n_y - r_{Ay} n_x$$
$$u_B = \vec{r}_B \times \vec{n} = r_{Bx} n_y - r_{By} n_x$$

Then:
$$\Delta \omega_A = -\frac{j u_A}{I_A}, \qquad \Delta \omega_B = +\frac{j u_B}{I_B}$$

#### Change in Relative Normal Velocity:
$$\Delta v_n = (\Delta \vec{v}_{BP} - \Delta \vec{v}_{AP}) \cdot \vec{n}$$
Expanding each term:
$$\Delta \vec{v}_{BP} \cdot \vec{n} = \frac{j}{m_B} (\vec{n} \cdot \vec{n}) + (\Delta \vec{\omega}_B \times \vec{r}_B) \cdot \vec{n}$$
Notice that $(\Delta \vec{\omega}_B \times \vec{r}_B) \cdot \vec{n} = \Delta \omega_B (\vec{r}_B \times \vec{n}) = \left(\frac{j u_B}{I_B}\right) u_B = j \frac{u_B^2}{I_B}$.

Similarly, for Body A:
$$\Delta \vec{v}_{AP} \cdot \vec{n} = -\frac{j}{m_A} - j \frac{u_A^2}{I_A}$$

Combining both:
$$\Delta v_n = j \left[ \frac{1}{m_A} + \frac{1}{m_B} + \frac{u_A^2}{I_A} + \frac{u_B^2}{I_B} \right]$$

Since we require $\Delta v_n = v_n' - v_n = -(1 + e) v_n$:
$$j \left[ \frac{1}{m_A} + \frac{1}{m_B} + \frac{u_A^2}{I_A} + \frac{u_B^2}{I_B} \right] = -(1 + e) v_n$$

$$\boxed{j = \frac{-(1 + e) v_n}{\frac{1}{m_A} + \frac{1}{m_B} + \frac{u_A^2}{I_A} + \frac{u_B^2}{I_B}}}$$

#### Final Velocity Updates:
$$\vec{v}_A \mathrel{-}= \frac{j \vec{n}}{m_A}$$
$$\vec{v}_B \mathrel{+}= \frac{j \vec{n}}{m_B}$$
$$\omega_A \mathrel{-}= \frac{j u_A}{I_A}$$
$$\omega_B \mathrel{+}= \frac{j u_B}{I_B}$$

---

### 3.3 Position Correction (Baumgarte / Linear Projection)

To prevent bodies from sinking into one another due to numerical integration drift, we project them apart proportionally to their inverse masses:

```cpp
const float slop    = 0.01f;   // Penetration allowance to prevent jitter
const float percent = 0.8f;    // Penetration resolution percentage (80%)

const float correctionMagnitude =
    percent * std::max(penetration - slop, 0.0f) / (invMassA + invMassB);

const Vec2 correction = normal * correctionMagnitude;

position       -= correction * invMassA;
other.position += correction * invMassB;
```

---

## 4. Circle vs Circle Collision

```
      Circle A                       Circle B
      (posA, rA)                     (posB, rB)
         ( ● ) ────── normal ──────► ( ● )
            \                       /
             \                     /
              P (Contact Point)
```

### 4.1 Detection & Normal Generation
Let $\vec{\Delta} = \vec{p}_B - \vec{p}_A$.
Distance squared: $d^2 = \Delta_x^2 + \Delta_y^2$.
Sum of radii: $R = r_A + r_B$.

- If $d^2 \ge R^2$: No collision.
- If $d^2 < R^2$: Overlap occurs.
  $$\text{dist} = \sqrt{d^2}$$
  $$\vec{n} = \frac{\vec{\Delta}}{\text{dist}}, \qquad \text{penetration} = R - \text{dist}$$

### 4.2 Impulse Resolution & Zero Torque Principle
The contact point lies directly on the line connecting the two centers:
$$\vec{r}_A = \vec{n} r_A, \qquad \vec{r}_B = -\vec{n} r_B$$

Because $\vec{r}_A$ is parallel to $\vec{n}$, the cross product vanishes:
$$u_A = \vec{r}_A \times \vec{n} = (r_A \vec{n}) \times \vec{n} = 0$$
$$u_B = \vec{r}_B \times \vec{n} = (-r_B \vec{n}) \times \vec{n} = 0$$

Hence, for frictionless circles, collisions **cannot generate torque**:
$$\Delta \omega_A = 0, \qquad \Delta \omega_B = 0$$

### 4.3 Code Walkthrough (`RigidBody::Collision`)
```cpp
void RigidBody::Collision(RigidBody& other)
{
    Vec2 normal = other.position - position;
    float distanceSq = normal.x * normal.x + normal.y * normal.y;
    float totalRadius = GetRadius() + other.GetRadius();

    if (distanceSq >= totalRadius * totalRadius) return;

    float distance = std::sqrt(distanceSq);
    normal = (distance > 0.0001f) ? (normal / distance) : Vec2(1.0f, 0.0f);

    // Position correction
    const float invMassA = 1.0f / mass;
    const float invMassB = 1.0f / other.mass;
    const float invMassSum = invMassA + invMassB;
    const float penetration = totalRadius - distance;
    const float correctionMagnitude = 0.8f * std::max(penetration - 0.01f, 0.0f) / invMassSum;
    position       -= normal * (correctionMagnitude * invMassA);
    other.position += normal * (correctionMagnitude * invMassB);

    // Relative contact velocity
    const Vec2 rA = normal * GetRadius();
    const Vec2 rB = normal * (-other.GetRadius());
    const Vec2 vA = velocity + Vec2(-angularVelocity * rA.y, angularVelocity * rA.x);
    const Vec2 vB = other.velocity + Vec2(-other.angularVelocity * rB.y, other.angularVelocity * rB.x);

    const Vec2 relativeVelocity = vB - vA;
    const float velocityAlongNormal = relativeVelocity.x * normal.x + relativeVelocity.y * normal.y;
    if (velocityAlongNormal >= 0.0f) return;

    const float uA = rA.x * normal.y - rA.y * normal.x; // = 0
    const float uB = rB.x * normal.y - rB.y * normal.x; // = 0
    const float denominator = invMassSum + (uA * uA * invInertiaA) + (uB * uB * invInertiaB);

    const float r = std::min(restitution, other.restitution);
    const float impulseMagnitude = -(1.0f + r) * velocityAlongNormal / denominator;
    const Vec2 impulse = normal * impulseMagnitude;

    velocity       -= impulse * invMassA;
    other.velocity += impulse * invMassB;
    angularVelocity       -= (uA * impulseMagnitude) * invInertiaA;
    other.angularVelocity += (uB * impulseMagnitude) * invInertiaB;
}
```

---

## 5. Circle vs Box Collision

```
                     Rotated Box (Angle θ)
                   ┌────────────────────────┐
                   │           rB           │
                   │          ┌───►● (Closest Point P)
                   │          │   / 
                   │       posB  / normal
                   │            /
                   └───────────/────────────┘
                              ▼
                           (  ●  ) Circle
```

### 5.1 Local Space Coordinate Transformation
To test a circle against an **Oriented Bounding Box (OBB)** rotated by angle $\theta_B$:
1. Calculate delta in world space: $\vec{\Delta} = \vec{p}_{\text{circle}} - \vec{p}_{\text{box}}$.
2. Transform $\vec{\Delta}$ into the box's local axis frame by rotating by $-\theta_B$:
   $$\text{localCircle}_x = \Delta_x \cos\theta_B + \Delta_y \sin\theta_B$$
   $$\text{localCircle}_y = -\Delta_x \sin\theta_B + \Delta_y \cos\theta_B$$

### 5.2 Voronoi Regions & Closest Point on OBB
In the box's local frame, the box is an axis-aligned box with extents $[-hw, +hw]$ and $[-hh, +hh]$.
The closest point $(cx, cy)$ on the box to the circle is obtained by clamping:
$$cx = \text{clamp}(\text{localCircle}_x, -hw, +hw)$$
$$cy = \text{clamp}(\text{localCircle}_y, -hh, +hh)$$

Local contact normal:
$$\vec{n}_{\text{local}} = (cx - \text{localCircle}_x, \; cy - \text{localCircle}_y)$$

If distance $d \ge R_{\text{circle}}$: No collision.
If the circle's center is entirely inside the box ($d \approx 0$), we project it out along the shortest face axis (left, right, top, or bottom).

### 5.3 Contact Point & Rotational Torque Generation
Rotate the local contact point and normal back to world coordinates:
$$\vec{n} = \begin{pmatrix} n_{\text{local}, x} \cos\theta_B - n_{\text{local}, y} \sin\theta_B \\ n_{\text{local}, x} \sin\theta_B + n_{\text{local}, y} \cos\theta_B \end{pmatrix}$$
$$\vec{r}_B = \begin{pmatrix} cx \cos\theta_B - cy \sin\theta_B \\ cx \sin\theta_B + cy \cos\theta_B \end{pmatrix}$$
$$P = \vec{p}_{\text{box}} + \vec{r}_B$$
$$\vec{r}_A = P - \vec{p}_{\text{circle}}$$

Notice that **$\vec{r}_B$ does not pass through the box's center of mass** unless the impact is perfectly centered. Therefore:
$$u_B = r_{Bx} n_y - r_{By} n_x \neq 0$$
This non-zero lever arm produces **torque**, causing the box to spin upon impact!

### 5.4 Code Walkthrough (`RigidBody::ResolveCircleBox`)
```cpp
void RigidBody::ResolveCircleBox(RigidBody& boxBody)
{
    // 1. Transform Circle center into Box's local coordinate frame
    const Vec2 delta = position - boxBody.position;
    const float cosB = std::cos(boxBody.angle);
    const float sinB = std::sin(boxBody.angle);

    const Vec2 localCircle(
         delta.x * cosB + delta.y * sinB,
        -delta.x * sinB + delta.y * cosB
    );

    // 2. Find closest point on AABB in local space
    const float closestX = std::clamp(localCircle.x, -halfWidth,  halfWidth);
    const float closestY = std::clamp(localCircle.y, -halfHeight, halfHeight);

    Vec2 localNormal(closestX - localCircle.x, closestY - localCircle.y);
    float distSq = localNormal.x * localNormal.x + localNormal.y * localNormal.y;
    if (distSq >= rad * rad) return;

    // 3. World space contact vector rB and collision normal
    const Vec2 normal(
        localNormal.x * cosB - localNormal.y * sinB,
        localNormal.x * sinB + localNormal.y * cosB
    );
    const Vec2 rB(
        closestX * cosB - closestY * sinB,
        closestX * sinB + closestY * cosB
    );
    const Vec2 rA = (boxBody.position + rB) - position;

    // 4. Contact point velocities with angular components
    const Vec2 vA = velocity + Vec2(-angularVelocity * rA.y, angularVelocity * rA.x);
    const Vec2 vB = boxBody.velocity + Vec2(-boxBody.angularVelocity * rB.y, boxBody.angularVelocity * rB.x);
    const Vec2 relativeVelocity = vB - vA;

    const float velocityAlongNormal = relativeVelocity.x * normal.x + relativeVelocity.y * normal.y;
    if (velocityAlongNormal >= 0.0f) return;

    // 5. Compute angular denominators and apply impulses
    const float uA = rA.x * normal.y - rA.y * normal.x;
    const float uB = rB.x * normal.y - rB.y * normal.x;

    const float denominator = invMassSum + (uA * uA * invInertiaA) + (uB * uB * invInertiaB);
    const float impulseMag = -(1.0f + resti) * velocityAlongNormal / denominator;

    velocity         -= normal * (impulseMag * invMassA);
    boxBody.velocity += normal * (impulseMag * invMassB);

    angularVelocity         -= (uA * impulseMag) * invInertiaA;
    boxBody.angularVelocity += (uB * impulseMag) * invInertiaB; // Box spins!
}
```

---

## 6. Box vs Box Collision (Separating Axis Theorem - SAT)

```
        Box A (Angle θA)
        ┌──────────────┐
        │              │       Axis 3 (Normal of B)
        │       ●──────┼──────────────►
        │     rA       │      ▲
        └───────┼──────┘      │
                │ P (Contact) │
             ┌──┴─────────────┴─┐
             │         rB       │
             │       ●          │
             │                  │
             └──────────────────┘
               Box B (Angle θB)
```

### 6.1 The Separating Axis Theorem (SAT)
> **Theorem**: Two convex polygons do not penetrate if and only if there exists a line (axis) onto which their projections do not overlap.

For two 2D boxes (oriented rectangles), any separating axis must be parallel to one of the **edge normals** of the two boxes:
- 2 perpendicular axes from Box A: $\vec{u}_{A1} = (\cos\theta_A, \sin\theta_A)$, $\vec{u}_{A2} = (-\sin\theta_A, \cos\theta_A)$
- 2 perpendicular axes from Box B: $\vec{u}_{B1} = (\cos\theta_B, \sin\theta_B)$, $\vec{u}_{B2} = (-\sin\theta_B, \cos\theta_B)$

There are **4 candidate axes** in total.

---

### 6.2 Testing 4 Separating Axes

For each candidate unit axis $\vec{a}$:
1. **Project Box A's half extents onto $\vec{a}$**:
   $$\text{proj}_A = hw_A |\vec{u}_{A1} \cdot \vec{a}| + hh_A |\vec{u}_{A2} \cdot \vec{a}|$$
   When $\vec{a} = \vec{u}_{A1}$, this simplifies to $\text{proj}_A = hw_A$.
2. **Project Box B's half extents onto $\vec{a}$**:
   $$\text{proj}_B = hw_B |\vec{u}_{B1} \cdot \vec{a}| + hh_B |\vec{u}_{B2} \cdot \vec{a}|$$
3. **Project distance between centers $\vec{\Delta} = \vec{p}_B - \vec{p}_A$**:
   $$\text{dist} = |\vec{\Delta} \cdot \vec{a}|$$
4. **Overlap**:
   $$\text{overlap} = \text{proj}_A + \text{proj}_B - \text{dist}$$
   - If $\text{overlap} \le 0$: We found a separating axis! The boxes do not intersect $\implies$ **return immediately**.
   - If $\text{overlap} > 0$: Record the axis with the **minimum overlap** (Minimum Translation Vector - MTV).

---

### 6.3 Contact Manifold & Point Generation

Once the collision normal $\vec{n}$ is found, we need the **world contact point $P$** to compute lever arms $\vec{r}_A = P - \vec{p}_A$ and $\vec{r}_B = P - \vec{p}_B$.

We compute world coordinates of all 4 corners of Box A and Box B:
$$\vec{C}_{k} = \vec{p} + \begin{pmatrix} c_{k, x} \cos\theta - c_{k, y} \sin\theta \\ c_{k, x} \sin\theta + c_{k, y} \cos\theta \end{pmatrix}$$

We then identify **penetrating vertices**:
1. Transform each corner of A into Box B's local frame. If within $[-hw_B, hw_B]$ and $[-hh_B, hh_B]$, it is a contact point.
2. Transform each corner of B into Box A's local frame. If within $[-hw_A, hw_A]$ and $[-hh_A, hh_A]$, it is a contact point.
3. Average the penetrating points:
   $$P = \frac{1}{N} \sum_{k=1}^N \vec{C}_k$$
   *(If none are strictly inside due to floating-point precision, take the midpoint of the extreme vertices along $\vec{n}$.)*

---

### 6.4 Dual-Body Rotational Impulse Resolution
With contact point $P$:
$$\vec{r}_A = P - \vec{p}_A, \qquad \vec{r}_B = P - \vec{p}_B$$
$$u_A = r_{Ax} n_y - r_{Ay} n_x, \qquad u_B = r_{Bx} n_y - r_{By} n_x$$

Both boxes receive angular impulse:
$$\Delta \omega_A = -\frac{j u_A}{I_A}$$
$$\Delta \omega_B = +\frac{j u_B}{I_B}$$

This ensures **conservation of angular momentum** during impacts.

---

### 6.5 Code Walkthrough (`RigidBody::ResolveBoxBox`)
```cpp
void RigidBody::ResolveBoxBox(RigidBody& other)
{
    // ... SAT loop over axesA[2] and axesB[2] ...
    // Finds bestAxis and minOverlap

    // Ensure normal points from A toward B
    const float d = delta.x * bestAxis.x + delta.y * bestAxis.y;
    const Vec2 normal = (d < 0.0f) ? Vec2(-bestAxis.x, -bestAxis.y) : bestAxis;

    // --- Contact Point Generation ---
    // Test all 4 corners of A against B and vice versa
    std::vector<Vec2> contactPoints;
    for (int i = 0; i < 4; i++) {
        // Transform world corners into opposite body's local frame
        // If inside bounds + epsilon, push to contactPoints
    }

    Vec2 contactPoint = Average(contactPoints);

    // Position correction
    const Vec2 correction = normal * (percent * std::max(minOverlap - slop, 0.0f) / invMassSum);
    position       -= correction * invMassA;
    other.position += correction * invMassB;

    // Velocity resolution with angular terms
    const Vec2 rA = contactPoint - position;
    const Vec2 rB = contactPoint - other.position;

    const Vec2 vA = velocity + Vec2(-angularVelocity * rA.y, angularVelocity * rA.x);
    const Vec2 vB = other.velocity + Vec2(-other.angularVelocity * rB.y, other.angularVelocity * rB.x);
    const Vec2 relativeVelocity = vB - vA;

    const float velocityAlongNormal = relativeVelocity.x * normal.x + relativeVelocity.y * normal.y;
    if (velocityAlongNormal >= 0.0f) return;

    const float uA = rA.x * normal.y - rA.y * normal.x;
    const float uB = rB.x * normal.y - rB.y * normal.x;
    const float denominator = invMassSum + (uA * uA * invInertiaA) + (uB * uB * invInertiaB);

    const float impulseMag = -(1.0f + r) * velocityAlongNormal / denominator;
    const Vec2 impulse = normal * impulseMag;

    velocity       -= impulse * invMassA;
    other.velocity += impulse * invMassB;

    angularVelocity       -= (uA * impulseMag) * invInertiaA; // Box A rotates
    other.angularVelocity += (uB * impulseMag) * invInertiaB; // Box B rotates
}
```

---

## 7. Ground Collision & Multi-Contact Stabilization

```
   Single-Point Bug (Oscillates):           Multi-Point Solution (Stable):
       ┌───────────┐                            ┌───────────┐
       │     ●     │                            │     ●     │
       └─────┬─────┘                            └─────┬─────┘
         r1 ◄┘ (Torque tilts box!)              j1 ▲     ▲ j2 (Equal torques cancel!)
       ══════▲════════════                      ═══╧═════╧═══
            Ground                                  Ground
```

### 7.1 Single-Corner Instability Problem
If a flat-falling box only resolves the single lowest corner:
1. Corner 1 is slightly lower ($0.001$ pixels) $\implies$ Entire upward ground impulse hits Corner 1 ($r_{1x} = +50$).
2. Lever arm $r_{1x}$ exerts huge torque $\implies$ Box violently rotates towards Corner 2.
3. Next frame, Corner 2 hits the ground $\implies$ Impulse hits Corner 2 ($r_{2x} = -50$).
4. Box violently rotates back.
5. **Result: The box jitters and rocks back and forth indefinitely on the ground.**

---

### 7.2 Multi-Point Ground Contact Solution
When a box lands, we test **all 4 corners**:
$$\text{bottom}_k = \text{position}_y + \text{rotated}_{k, y}$$
$$\text{penetration}_k = \text{bottom}_k - \text{groundY}$$

If two corners penetrate, we collect both contacts:
- Ground normal is $\vec{n} = (0, -1)$.
- Lever arm cross product:
  $$\text{crossRN}_k = \vec{r}_k \times \vec{n} = r_{kx} (-1) - r_{ky} (0) = -r_{kx}$$
- Contact velocity at corner $k$:
  $$v_{Pk, y} = v_y + \omega r_{kx}$$

When the box is flat ($r_{1x} = -hw, r_{2x} = +hw$):
- Corner 1 torque: $-(-hw) \cdot j_1 = +hw \cdot j_1$
- Corner 2 torque: $-(+hw) \cdot j_2 = -hw \cdot j_2$
- Since $j_1 \approx j_2$, the torques **sum to zero**:
  $$\Sigma \tau = +hw \cdot j - hw \cdot j = 0$$

The box lands flat and stable without artificial wobbling!

---

### 7.3 Resting Contacts & Angular Damping
- When contact velocity $|v_{Pk, y}| < 15.0$, we clamp restitution $e = 0.0$ to eliminate microscopic bouncing.
- When both corners are grounded and vertical velocity is near zero, angular velocity is damped:
  ```cpp
  if (contacts.size() >= 2 && std::abs(velocity.y) < 20.0f) {
      angularVelocity *= 0.9f;
      if (std::abs(angularVelocity) < 0.01f)
          angularVelocity = 0.0f;
  }
  ```

---

### 7.4 Code Walkthrough (`RigidBody::GroundCollision`)
```cpp
void RigidBody::GroundCollision(float groundY)
{
    if (!shape || mass <= 0.0f) return;

    if (shape->GetType() == ShapeType::Circle)
    {
        const auto& circle = static_cast<const Circle&>(*shape);
        const float r = circle.GetRadius();
        const float penetration = (position.y + r) - groundY;
        if (penetration <= 0.0f || velocity.y <= 0.0f) return;

        position.y -= penetration;
        const float e = (std::abs(velocity.y) < 15.0f) ? 0.0f : std::clamp(restitution, 0.0f, 1.0f);
        const float impulse = (1.0f + e) * velocity.y * mass;
        velocity.y -= impulse / mass;
    }
    else if (shape->GetType() == ShapeType::Box)
    {
        const auto& box = static_cast<const Box&>(*shape);
        const float hw = box.GetWidth() * 0.5f;
        const float hh = box.GetHeight() * 0.5f;
        const float c = std::cos(angle);
        const float s = std::sin(angle);

        const Vec2 corners[4] = { Vec2(-hw, -hh), Vec2(hw, -hh), Vec2(hw, hh), Vec2(-hw, hh) };

        struct ContactPoint { Vec2 r; float penetration; };
        std::vector<ContactPoint> contacts;
        float maxPenetration = 0.0f;

        for (const Vec2& corner : corners)
        {
            const Vec2 rotated(corner.x * c - corner.y * s, corner.x * s + corner.y * c);
            const float bottom = position.y + rotated.y;
            const float p = bottom - groundY;
            if (p > 0.0f) {
                contacts.push_back({ rotated, p });
                maxPenetration = std::max(maxPenetration, p);
            }
        }

        if (contacts.empty()) return;
        position.y -= maxPenetration;

        const float invMass = 1.0f / mass;
        const float invInertia = inertia > 0.0001f ? 1.0f / inertia : 0.0f;
        const float numContacts = static_cast<float>(contacts.size());

        for (const auto& cp : contacts)
        {
            const float contactVy = velocity.y + angularVelocity * cp.r.x;
            if (contactVy <= 0.0f) continue;

            const float crossRN = -cp.r.x;
            const float denominator = (invMass / numContacts) + crossRN * crossRN * invInertia;
            if (denominator <= 0.0f) continue;

            const float e = (std::abs(contactVy) < 15.0f) ? 0.0f : std::clamp(restitution, 0.0f, 1.0f);
            const float impulseMag = (1.0f + e) * contactVy / denominator;

            velocity.y      -= (impulseMag * invMass) / numContacts;
            angularVelocity += crossRN * impulseMag * invInertia;
        }

        if (contacts.size() >= 2 && std::abs(velocity.y) < 20.0f) {
            angularVelocity *= 0.9f;
            if (std::abs(angularVelocity) < 0.01f) angularVelocity = 0.0f;
        }
    }
}
```

---

## 8. Rendering & ASCII Rasterization

### 8.1 Raylib Rotated Box Rendering
In `Renderer.cpp`, Raylib renders rotated rectangles using `DrawRectanglePro`:
```cpp
Rectangle rectangle{
    position.x,
    position.y,
    box.GetWidth(),
    box.GetHeight()
};
Vector2 origin = {
    box.GetWidth() * 0.5f,
    box.GetHeight() * 0.5f
};

const float angleDegrees = body.GetAngle() * (180.0f / PI);

DrawRectanglePro(rectangle, origin, angleDegrees, BLUE);
```
- **Origin**: Setting origin to `(width/2, height/2)` makes the box rotate around its center of mass.
- **Angle Conversion**: Physics engine angles are in radians; Raylib expects degrees: $\text{deg} = \text{rad} \times \frac{180}{\pi}$.

---

### 8.2 ASCII Terminal Rasterization & Inverse Rotation
In `ASCIIrenderer.h`, the terminal grid evaluates whether each cell $(gx, gy)$ falls inside a rotated box:

```cpp
float dx = worldX - pos.x;
float dy = worldY - pos.y;

// Rotate into box local frame by -angle
float lx =  dx * std::cos(-body.GetAngle()) - dy * std::sin(-body.GetAngle());
float ly =  dx * std::sin(-body.GetAngle()) + dy * std::cos(-body.GetAngle());

if (std::abs(lx) <= halfWidth && std::abs(ly) <= halfHeight) {
    grid[gy][gx] = '#';
}
```
This rasterizes the rotating box into ASCII `#` characters in real time without floating-point polygon clipping.

---

## 9. Summary Architecture & Verification

```
                      PhysicsWorld::Step(dt)
                                │
        ┌───────────────────────┴───────────────────────┐
        ▼                                               ▼
1. Apply Gravity & External Forces             2. Collision Resolution
        │                                               │
   body.Integrate(dt)                        Iterate All Pairs (i, j)
   - v += (F / m) * dt                                  │
   - p += v * dt                                        ├── Circle vs Circle: Collision()
   - ω += (τ / I) * dt                                  ├── Circle vs Box:    ResolveCircleBox()
   - θ += ω * dt                                        └── Box vs Box:       ResolveBoxBox() (SAT)
        │                                               │
   body.GroundCollision(groundY)                        ▼
   - Multi-point corner detection             Angular & Linear Impulses
   - Cancels torque on flat surfaces          - j = -(1+e)vn / Denominator
   - Damps resting vibrations                 - Δv = ± j*n / m
                                              - Δω = ± j*(r x n) / I
```

### Key Equations at a Glance:
1. **Torque**: $\tau = r_x F_y - r_y F_x$
2. **Point Velocity**: $\vec{v}_P = (v_x - \omega r_y, \; v_y + \omega r_x)$
3. **Cross Product**: $u = \vec{r} \times \vec{n} = r_x n_y - r_y n_x$
4. **Collision Impulse**:
   $$j = \frac{-(1 + e) v_n}{\frac{1}{m_A} + \frac{1}{m_B} + \frac{u_A^2}{I_A} + \frac{u_B^2}{I_B}}$$
5. **Linear Velocity Update**: $\Delta \vec{v}_A = -\frac{j \vec{n}}{m_A}, \quad \Delta \vec{v}_B = +\frac{j \vec{n}}{m_B}$
6. **Angular Velocity Update**: $\Delta \omega_A = -\frac{j u_A}{I_A}, \quad \Delta \omega_B = +\frac{j u_B}{I_B}$
