"""
benchmark_sat.py
================
Benchmarks the Vectorized Separating Axis Theorem (SAT) GPU engine
with a mix of 50% Circles and 50% Rotated Boxes (OBBs).
"""

import torch
import numpy as np
import time
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from batch_sat_physics import GPUSATPhysicsEngine

def benchmark_sat():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print("=" * 65)
    print(f"  GPU SAT Physics Benchmark (Circles + Rotated Boxes)")
    print(f"  Device: {torch.cuda.get_device_name(0) if device == 'cuda' else 'CPU'}")
    print("=" * 65)
    print(f"{'N Bodies':>10} | {'Shape Mix':>16} | {'ms/step':>12} | {'Physics FPS':>14}")
    print("-" * 65)

    counts = [100, 250, 500, 1000, 2000]

    for N in counts:
        rng = np.random.default_rng(42)
        pos = rng.uniform([30, 30], [1050, 500], size=(N, 2)).tolist()
        vel = rng.uniform([-100, 0], [100, 150], size=(N, 2)).tolist()
        ang = rng.uniform(0, 2 * np.pi, size=N).tolist()
        ang_vel = rng.uniform(-5.0, 5.0, size=N).tolist()
        mass = rng.uniform(10, 60, size=N).tolist()

        # 50% circles (0), 50% boxes (1)
        shapes = [0 if i % 2 == 0 else 1 for i in range(N)]
        
        # Half extents: circles [r, r], boxes [w/2, h/2]
        extents = []
        for s in shapes:
            if s == 0:
                r = rng.uniform(6, 14)
                extents.append([r, r])
            else:
                hw = rng.uniform(8, 18)
                hh = rng.uniform(6, 12)
                extents.append([hw, hh])

        rest = rng.uniform(0.5, 0.8, size=N).tolist()

        engine = GPUSATPhysicsEngine(device=device)
        engine.add_bodies(pos, vel, ang, ang_vel, mass, shapes, extents, rest)

        dt = 1.0 / 60.0
        # Warmup
        for _ in range(20):
            engine.step(dt)

        if device == 'cuda':
            torch.cuda.synchronize()

        reps = 100
        t0 = time.perf_counter()
        for _ in range(reps):
            engine.step(dt)
        if device == 'cuda':
            torch.cuda.synchronize()

        ms = (time.perf_counter() - t0) / reps * 1000.0
        fps = 1000.0 / ms

        mix_str = f"{N//2} C + {N - N//2} B"
        print(f"{N:>10} | {mix_str:>16} | {ms:>12.3f} | {fps:>14.1f}")

    print("=" * 65)

if __name__ == '__main__':
    benchmark_sat()

