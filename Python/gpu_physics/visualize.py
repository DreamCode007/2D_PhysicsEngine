"""
visualize.py
============
Real-time visualization of the GPU physics simulation.

Uses matplotlib's animation to render what the GPU is computing each frame.
Your RTX 3050 runs the physics; matplotlib just draws the result.

Controls:
  SPACE  — toggle simulation pause
  R      — reset with new random bodies
  +/-    — add / remove bodies
  Q      — quit
"""

import sys
import os
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import torch

sys.path.insert(0, os.path.dirname(__file__))
from batch_physics import GPUPhysicsEngine


# ─────────────────────────────────────────────────────────────────────────────
# Simulation configuration
# ─────────────────────────────────────────────────────────────────────────────

WORLD_W    = 1080.0
WORLD_H    = 720.0
GROUND_Y   = 650.0
GRAVITY_Y  = 500.0
TARGET_FPS = 60
DT         = 1.0 / TARGET_FPS

# Starting body count — increase to stress-test
INITIAL_N  = 200

# Colour palette (cycle through bodies)
COLORS = plt.cm.plasma(np.linspace(0.2, 0.9, 32))


# ─────────────────────────────────────────────────────────────────────────────
# Body factory
# ─────────────────────────────────────────────────────────────────────────────

def make_bodies(N: int, seed: int = None):
    rng = np.random.default_rng(seed)
    pos   = rng.uniform([20, 20],    [WORLD_W - 20, GROUND_Y - 100], (N, 2))
    vel   = rng.uniform([-200, -50], [200, 300],                      (N, 2))
    mass  = rng.uniform(10, 60,      N)
    rad   = rng.uniform(5, 18,       N)
    rest  = rng.uniform(0.4, 0.85,   N)
    return pos.tolist(), vel.tolist(), mass.tolist(), rad.tolist(), rest.tolist()


# ─────────────────────────────────────────────────────────────────────────────
# Visualizer
# ─────────────────────────────────────────────────────────────────────────────

class PhysicsVisualizer:
    def __init__(self):
        self.engine = GPUPhysicsEngine(
            gravity_y=GRAVITY_Y,
            ground_y=GROUND_Y,
            world_w=WORLD_W,
        )
        self.N      = INITIAL_N
        self.paused = False
        self.frame  = 0
        self.step_times = []
        self._init_bodies()
        self._setup_figure()

    # ── Init ──────────────────────────────────────────────────────────────────

    def _init_bodies(self, seed=None):
        pos, vel, mass, rad, rest = make_bodies(self.N, seed)
        self.engine.add_bodies(pos, vel, mass, rad, rest)
        self.radii = np.array(rad)

    def _setup_figure(self):
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.fig.patch.set_facecolor('#0d0d0d')
        self.ax.set_facecolor('#0d0d0d')
        self.ax.set_xlim(0, WORLD_W)
        self.ax.set_ylim(WORLD_H, 0)   # Y flipped (screen coords)
        self.ax.set_aspect('equal')
        self.ax.axis('off')

        # Ground line
        self.ax.axhline(GROUND_Y, color='#444444', linewidth=1.5, zorder=0)
        self.ax.text(10, GROUND_Y + 15, 'GROUND', color='#555555',
                     fontsize=8, va='top')

        # Build circle patches (one per body, reused every frame)
        self.circles = []
        for i in range(self.N):
            color = COLORS[i % len(COLORS)]
            c = plt.Circle((0, 0), 1, color=color, zorder=2)
            self.ax.add_patch(c)
            self.circles.append(c)

        # HUD text
        txt_kw = dict(transform=self.ax.transAxes, fontsize=9,
                      fontfamily='monospace', va='top')
        self.txt_fps    = self.ax.text(0.01, 0.99, '', color='#00ff88', **txt_kw)
        self.txt_bodies = self.ax.text(0.01, 0.95, '', color='#88aaff', **txt_kw)
        self.txt_step   = self.ax.text(0.01, 0.91, '', color='#ffaa44', **txt_kw)
        self.txt_device = self.ax.text(0.99, 0.99, '', color='#ff6688',
                                       ha='right', **txt_kw)
        self.txt_hint   = self.ax.text(0.5, 0.01,
                                       'SPACE=pause  R=reset  +/-=bodies  Q=quit',
                                       color='#555555', ha='center',
                                       transform=self.ax.transAxes, fontsize=8)

        # Device label
        dev = self.engine.device
        if dev.type == 'cuda':
            dev_str = f"GPU: {torch.cuda.get_device_name(0)}"
        else:
            dev_str = "CPU mode"
        self.txt_device.set_text(dev_str)

        # Key bindings
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self._last_frame_time = time.perf_counter()

    # ── Frame update ──────────────────────────────────────────────────────────

    def update(self, frame):
        # FPS calculation
        now = time.perf_counter()
        frame_dt = now - self._last_frame_time
        self._last_frame_time = now
        display_fps = 1.0 / max(frame_dt, 1e-6)

        if not self.paused:
            # ── GPU physics step ──
            t0 = time.perf_counter()
            self.engine.step(DT)
            if self.engine.device.type == 'cuda':
                torch.cuda.synchronize()
            step_ms = (time.perf_counter() - t0) * 1000.0

            self.step_times.append(step_ms)
            if len(self.step_times) > 60:
                self.step_times.pop(0)
            avg_step = sum(self.step_times) / len(self.step_times)

            # ── Read positions from GPU ──
            pos = self.engine.get_positions_numpy()   # [N, 2]

            # ── Update circle patches ──
            for i, circle in enumerate(self.circles):
                if i < len(pos):
                    circle.center = (pos[i, 0], pos[i, 1])
                    circle.radius = float(self.radii[i]) if i < len(self.radii) else 8.0

            # ── HUD ──
            self.txt_fps.set_text(f'Render FPS : {display_fps:>6.1f}')
            self.txt_bodies.set_text(f'Bodies     : {self.N:>6}')
            self.txt_step.set_text(f'GPU step   : {avg_step:>6.3f} ms')

        return self.circles + [self.txt_fps, self.txt_bodies,
                               self.txt_step, self.txt_device]

    # ── Key bindings ──────────────────────────────────────────────────────────

    def _rebuild(self):
        """Rebuild circle patches to match new N."""
        for c in self.circles:
            c.remove()
        self.circles.clear()
        self._init_bodies(seed=None)
        self.radii = self.engine.state.radius.cpu().numpy()
        for i in range(self.N):
            color = COLORS[i % len(COLORS)]
            c = plt.Circle((0, 0), 1, color=color, zorder=2)
            self.ax.add_patch(c)
            self.circles.append(c)

    def _on_key(self, event):
        if event.key == ' ':
            self.paused = not self.paused
            status = 'PAUSED' if self.paused else 'RUNNING'
            print(f"Simulation {status}")

        elif event.key == 'r':
            print(f"Reset with N={self.N}")
            self._rebuild()

        elif event.key in ('+', '='):
            self.N = min(self.N + 50, 3000)
            print(f"Bodies → {self.N}")
            self._rebuild()

        elif event.key == '-':
            self.N = max(self.N - 50, 10)
            print(f"Bodies → {self.N}")
            self._rebuild()

        elif event.key == 'q':
            plt.close('all')

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self):
        print(f"\n[Visualizer] Starting with N={self.N} bodies on {self.engine.device}")
        print("Controls: SPACE=pause | R=reset | +/-=add/remove bodies | Q=quit\n")

        self.anim = FuncAnimation(
            self.fig,
            self.update,
            interval=1,         # call as fast as possible
            blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout(pad=0)
        plt.show()


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    viz = PhysicsVisualizer()
    viz.run()

