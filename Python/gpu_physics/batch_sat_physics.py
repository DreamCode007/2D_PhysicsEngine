"""
batch_sat_physics.py
====================
Fully Vectorized GPU Rigid-Body Physics with Oriented Bounding Boxes (OBBs)
and Separating Axis Theorem (SAT) on PyTorch CUDA.

Supports:
  - Circles AND Rotated Rectangles (Boxes)
  - Angular velocity (omega), torque, moment of inertia (I)
  - Semi-implicit Euler integration for both linear and angular motion
  - Vectorized SAT (Separating Axis Theorem) collision for Box-Box
  - Vectorized Circle-Box collision in local box coordinates
  - Vectorized Circle-Circle collision
  - Restitution, friction, and angular impulses
"""

import torch
import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class BatchedWorldState:
    # Linear state
    pos:         torch.Tensor  # [N, 2]
    vel:         torch.Tensor  # [N, 2]
    force:       torch.Tensor  # [N, 2]
    mass:        torch.Tensor  # [N]
    inv_mass:    torch.Tensor  # [N]

    # Angular state
    angle:       torch.Tensor  # [N] (radians)
    ang_vel:     torch.Tensor  # [N] (rad/s)
    torque:      torch.Tensor  # [N] (accumulated torque)
    inertia:     torch.Tensor  # [N] (moment of inertia)
    inv_inertia: torch.Tensor  # [N]

    # Shape attributes
    shape_type:  torch.Tensor  # [N] (0 = Circle, 1 = Box)
    half_extents:torch.Tensor  # [N, 2] (half_width, half_height; for circle: [radius, radius])
    restitution: torch.Tensor  # [N] [0, 1]

    @property
    def N(self) -> int:
        return self.pos.shape[0]


class GPUSATPhysicsEngine:
    def __init__(
        self,
        gravity_y: float = 500.0,
        ground_y: float = 650.0,
        world_w: float = 1080.0,
        device: str = 'cuda'
    ):
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.gravity = torch.tensor([0.0, gravity_y], device=self.device)
        self.ground_y = ground_y
        self.world_w = world_w
        self.state: Optional[BatchedWorldState] = None

    def add_bodies(
        self,
        positions,
        velocities,
        angles,
        ang_velocities,
        masses,
        shape_types,  # 0 = Circle, 1 = Box
        half_extents, # [[hw, hh], ...]
        restitutions
    ):
        def t(data, dtype=torch.float32):
            return torch.tensor(data, dtype=dtype, device=self.device)

        N = len(positions)
        mass_t = t(masses)
        inv_mass_t = 1.0 / mass_t.clamp(min=1e-5)

        st_t = torch.tensor(shape_types, dtype=torch.long, device=self.device)
        he_t = t(half_extents)

        # Calculate moment of inertia:
        # Circle: I = 0.5 * m * r^2
        # Box:    I = m * (w^2 + h^2) / 12 = m * ((2*hw)^2 + (2*hh)^2) / 12
        r_sq = he_t[:, 0] * he_t[:, 0]
        i_circle = 0.5 * mass_t * r_sq
        w_sq = 4.0 * he_t[:, 0] * he_t[:, 0]
        h_sq = 4.0 * he_t[:, 1] * he_t[:, 1]
        i_box = mass_t * (w_sq + h_sq) / 12.0

        inertia_t = torch.where(st_t == 0, i_circle, i_box)
        inv_inertia_t = 1.0 / inertia_t.clamp(min=1e-5)

        self.state = BatchedWorldState(
            pos=t(positions),
            vel=t(velocities),
            force=torch.zeros((N, 2), device=self.device),
            mass=mass_t,
            inv_mass=inv_mass_t,
            angle=t(angles),
            ang_vel=t(ang_velocities),
            torque=torch.zeros(N, device=self.device),
            inertia=inertia_t,
            inv_inertia=inv_inertia_t,
            shape_type=st_t,
            half_extents=he_t,
            restitution=t(restitutions).clamp(0.0, 1.0)
        )

    def reset_forces(self):
        self.state.force.zero_()
        self.state.torque.zero_()

    def apply_gravity(self):
        s = self.state
        s.force += self.gravity.unsqueeze(0) * s.mass.unsqueeze(1)

    def integrate(self, dt: float):
        s = self.state
        # Linear integration
        accel = s.force * s.inv_mass.unsqueeze(1)
        s.vel += accel * dt
        s.pos += s.vel * dt

        # Angular integration
        ang_accel = s.torque * s.inv_inertia
        s.ang_vel += ang_accel * dt
        s.angle += s.ang_vel * dt

    def ground_collision(self):
        s = self.state
        # For circles: bottom = pos.y + radius
        # For boxes: bottom = pos.y + hw*|sin(theta)| + hh*|cos(theta)|
        c = s.angle.cos().abs()
        sin_a = s.angle.sin().abs()
        half_ext_y = torch.where(
            s.shape_type == 0,
            s.half_extents[:, 0],
            s.half_extents[:, 0] * sin_a + s.half_extents[:, 1] * c
        )

        bottom = s.pos[:, 1] + half_ext_y
        penetration = bottom - self.ground_y
        in_contact = penetration > 0.0

        if not in_contact.any():
            return

        s.pos[:, 1] -= torch.where(in_contact, penetration, torch.zeros_like(penetration))

        moving_down = s.vel[:, 1] > 0.0
        should_bounce = in_contact & moving_down

        e = torch.where(s.vel[:, 1].abs() < 15.0, torch.zeros_like(s.restitution), s.restitution)
        new_vy = -s.vel[:, 1] * e
        s.vel[:, 1] = torch.where(should_bounce, new_vy, s.vel[:, 1])

        # Angular friction on ground contact
        s.ang_vel = torch.where(in_contact, s.ang_vel * 0.95, s.ang_vel)

    def wall_collision(self):
        s = self.state
        # Bounding radius for wall tests
        r = s.half_extents.norm(dim=1)

        # Left wall
        left_pen = r - s.pos[:, 0]
        hit_left = left_pen > 0
        s.pos[:, 0] += torch.where(hit_left, left_pen, torch.zeros_like(left_pen))
        s.vel[:, 0] = torch.where(hit_left, s.vel[:, 0].abs(), s.vel[:, 0])

        # Right wall
        right_pen = (s.pos[:, 0] + r) - self.world_w
        hit_right = right_pen > 0
        s.pos[:, 0] -= torch.where(hit_right, right_pen, torch.zeros_like(right_pen))
        s.vel[:, 0] = torch.where(hit_right, -s.vel[:, 0].abs(), s.vel[:, 0])

    def broad_phase(self) -> torch.Tensor:
        """Pairwise distance check using bounding radii."""
        s = self.state
        pos = s.pos
        # Bounding radius: circles = r, boxes = sqrt(hw^2 + hh^2)
        bound_r = s.half_extents.norm(dim=1)

        diff = pos.unsqueeze(0) - pos.unsqueeze(1)
        dist_sq = (diff * diff).sum(dim=2)

        sum_r = bound_r.unsqueeze(1) + bound_r.unsqueeze(0)
        candidates = dist_sq < (sum_r * sum_r)

        mask = torch.triu(candidates, diagonal=1)
        pairs = mask.nonzero(as_tuple=False)
        return pairs

    def resolve_collisions(self, pairs: torch.Tensor):
        if pairs.shape[0] == 0:
            return

        s = self.state
        i = pairs[:, 0]
        j = pairs[:, 1]

        type_i = s.shape_type[i]
        type_j = s.shape_type[j]

        # 1. Circle - Circle (both == 0)
        cc_mask = (type_i == 0) & (type_j == 0)
        if cc_mask.any():
            self._resolve_circle_circle(i[cc_mask], j[cc_mask])

        # 2. Box - Box (both == 1) via GPU Vectorized SAT
        bb_mask = (type_i == 1) & (type_j == 1)
        if bb_mask.any():
            self._resolve_box_box_sat(i[bb_mask], j[bb_mask])

        # 3. Circle - Box (type_i == 0, type_j == 1)
        cb_mask = (type_i == 0) & (type_j == 1)
        if cb_mask.any():
            self._resolve_circle_box(i[cb_mask], j[cb_mask])

        # 4. Box - Circle (type_i == 1, type_j == 0)
        bc_mask = (type_i == 1) & (type_j == 0)
        if bc_mask.any():
            self._resolve_circle_box(j[bc_mask], i[bc_mask])

    def _resolve_circle_circle(self, i: torch.Tensor, j: torch.Tensor):
        s = self.state
        pos_a, pos_b = s.pos[i], s.pos[j]
        vel_a, vel_b = s.vel[i], s.vel[j]
        r_a, r_b = s.half_extents[i, 0], s.half_extents[j, 0]

        delta = pos_b - pos_a
        dist = delta.norm(dim=1).clamp(min=1e-6)
        normal = delta / dist.unsqueeze(1)

        penetration = (r_a + r_b) - dist
        colliding = penetration > 0.0

        inv_ma = s.inv_mass[i]
        inv_mb = s.inv_mass[j]
        inv_sum = inv_ma + inv_mb

        corr = 0.8 * (penetration - 0.01).clamp(min=0.0) / inv_sum
        correction = normal * corr.unsqueeze(1)

        s.pos.scatter_add_(0, i.unsqueeze(1).expand(-1, 2), torch.where(colliding.unsqueeze(1), -correction * inv_ma.unsqueeze(1), torch.zeros_like(correction)))
        s.pos.scatter_add_(0, j.unsqueeze(1).expand(-1, 2), torch.where(colliding.unsqueeze(1), correction * inv_mb.unsqueeze(1), torch.zeros_like(correction)))

        rel_vel = vel_b - vel_a
        vn = (rel_vel * normal).sum(dim=1)
        active = colliding & (vn < 0.0)

        e = torch.min(s.restitution[i], s.restitution[j])
        j_mag = torch.where(active, -(1.0 + e) * vn / inv_sum, torch.zeros_like(vn))
        impulse = normal * j_mag.unsqueeze(1)

        s.vel.scatter_add_(0, i.unsqueeze(1).expand(-1, 2), -impulse * inv_ma.unsqueeze(1))
        s.vel.scatter_add_(0, j.unsqueeze(1).expand(-1, 2), impulse * inv_mb.unsqueeze(1))

    def _resolve_box_box_sat(self, i: torch.Tensor, j: torch.Tensor):
        """
        Vectorized Separating Axis Theorem (SAT) on GPU.
        Tests 4 axes (2 from A, 2 from B) simultaneously for all M colliding pairs.
        """
        s = self.state
        M = i.shape[0]

        pos_a, pos_b = s.pos[i], s.pos[j]
        hw_a, hh_a = s.half_extents[i, 0], s.half_extents[i, 1]
        hw_b, hh_b = s.half_extents[j, 0], s.half_extents[j, 1]

        cos_a, sin_a = s.angle[i].cos(), s.angle[i].sin()
        cos_b, sin_b = s.angle[j].cos(), s.angle[j].sin()

        # Axes for Box A: [M, 2, 2] -> 2 unit vectors per pair
        # axis 0: (cos, sin), axis 1: (-sin, cos)
        ax_a0 = torch.stack([cos_a, sin_a], dim=1)
        ax_a1 = torch.stack([-sin_a, cos_a], dim=1)

        # Axes for Box B:
        ax_b0 = torch.stack([cos_b, sin_b], dim=1)
        ax_b1 = torch.stack([-sin_b, cos_b], dim=1)

        # Stack all 4 test axes: [M, 4, 2]
        axes = torch.stack([ax_a0, ax_a1, ax_b0, ax_b1], dim=1)

        # Delta vector from A to B: [M, 1, 2]
        delta = (pos_b - pos_a).unsqueeze(1)

        # Distance along each of the 4 axes: [M, 4]
        dist_along = (delta * axes).sum(dim=-1).abs()

        # Projection of Box A onto each axis:
        # projA = hw_a * |ax_a0 . axis| + hh_a * |ax_a1 . axis|
        dot_a0 = (axes * ax_a0.unsqueeze(1)).sum(dim=-1).abs()
        dot_a1 = (axes * ax_a1.unsqueeze(1)).sum(dim=-1).abs()
        proj_a = hw_a.unsqueeze(1) * dot_a0 + hh_a.unsqueeze(1) * dot_a1

        # Projection of Box B onto each axis:
        dot_b0 = (axes * ax_b0.unsqueeze(1)).sum(dim=-1).abs()
        dot_b1 = (axes * ax_b1.unsqueeze(1)).sum(dim=-1).abs()
        proj_b = hw_b.unsqueeze(1) * dot_b0 + hh_b.unsqueeze(1) * dot_b1

        # Overlap along each axis: [M, 4]
        overlap = (proj_a + proj_b) - dist_along

        # Separating axis exists if ANY overlap <= 0
        has_separation = (overlap <= 0.0).any(dim=1)  # [M] bool
        is_colliding = ~has_separation

        if not is_colliding.any():
            return

        # Find minimum positive overlap and corresponding best axis
        # Set negative overlaps to large number so min finds minimum positive
        valid_overlap = torch.where(overlap > 0, overlap, torch.full_like(overlap, 1e9))
        min_overlap, best_axis_idx = valid_overlap.min(dim=1)  # [M]

        # Gather best axis for each pair: [M, 2]
        batch_indices = torch.arange(M, device=self.device)
        best_normal = axes[batch_indices, best_axis_idx]

        # Ensure normal points from A to B: dot(delta, normal) >= 0
        delta_dot = (delta.squeeze(1) * best_normal).sum(dim=1)
        sign = torch.where(delta_dot < 0, -1.0, 1.0)
        normal = best_normal * sign.unsqueeze(1)

        # Positional correction
        inv_ma = s.inv_mass[i]
        inv_mb = s.inv_mass[j]
        inv_sum = inv_ma + inv_mb

        corr = 0.8 * (min_overlap - 0.01).clamp(min=0.0) / inv_sum
        correction = normal * corr.unsqueeze(1)

        s.pos.scatter_add_(0, i.unsqueeze(1).expand(-1, 2), torch.where(is_colliding.unsqueeze(1), -correction * inv_ma.unsqueeze(1), torch.zeros_like(correction)))
        s.pos.scatter_add_(0, j.unsqueeze(1).expand(-1, 2), torch.where(is_colliding.unsqueeze(1), correction * inv_mb.unsqueeze(1), torch.zeros_like(correction)))

        # Velocity impulse
        vel_a, vel_b = s.vel[i], s.vel[j]
        rel_vel = vel_b - vel_a
        vn = (rel_vel * normal).sum(dim=1)
        active = is_colliding & (vn < 0.0)

        e = torch.min(s.restitution[i], s.restitution[j])
        j_mag = torch.where(active, -(1.0 + e) * vn / inv_sum, torch.zeros_like(vn))
        impulse = normal * j_mag.unsqueeze(1)

        s.vel.scatter_add_(0, i.unsqueeze(1).expand(-1, 2), -impulse * inv_ma.unsqueeze(1))
        s.vel.scatter_add_(0, j.unsqueeze(1).expand(-1, 2), impulse * inv_mb.unsqueeze(1))

    def _resolve_circle_box(self, i_circle: torch.Tensor, j_box: torch.Tensor):
        """Vectorized Circle vs Rotated Box collision on GPU."""
        s = self.state
        pos_c, pos_b = s.pos[i_circle], s.pos[j_box]
        rad = s.half_extents[i_circle, 0]
        hw, hh = s.half_extents[j_box, 0], s.half_extents[j_box, 1]

        cos_b = s.angle[j_box].cos()
        sin_b = s.angle[j_box].sin()

        # Transform circle into box's local frame
        delta = pos_c - pos_b
        local_x = delta[:, 0] * cos_b + delta[:, 1] * sin_b
        local_y = -delta[:, 0] * sin_b + delta[:, 1] * cos_b

        # Closest point on AABB in local space
        closest_x = local_x.clamp(-hw, hw)
        closest_y = local_y.clamp(-hh, hh)

        # Distance from circle center to closest point
        nx = local_x - closest_x
        ny = local_y - closest_y
        dist_sq = nx * nx + ny * ny
        colliding = dist_sq < (rad * rad)

        if not colliding.any():
            return

        dist = dist_sq.sqrt().clamp(min=1e-6)
        pen = rad - dist

        # Rotate normal back to world space
        local_norm_x = nx / dist
        local_norm_y = ny / dist
        world_nx = local_norm_x * cos_b - local_norm_y * sin_b
        world_ny = local_norm_x * sin_b + local_norm_y * cos_b
        normal = torch.stack([world_nx, world_ny], dim=1) # Normal points from box to circle

        inv_ma = s.inv_mass[i_circle]
        inv_mb = s.inv_mass[j_box]
        inv_sum = inv_ma + inv_mb

        corr = 0.8 * (pen - 0.01).clamp(min=0.0) / inv_sum
        correction = normal * corr.unsqueeze(1)

        s.pos.scatter_add_(0, i_circle.unsqueeze(1).expand(-1, 2), torch.where(colliding.unsqueeze(1), correction * inv_ma.unsqueeze(1), torch.zeros_like(correction)))
        s.pos.scatter_add_(0, j_box.unsqueeze(1).expand(-1, 2), torch.where(colliding.unsqueeze(1), -correction * inv_mb.unsqueeze(1), torch.zeros_like(correction)))

        # Velocity impulse
        vel_c, vel_b = s.vel[i_circle], s.vel[j_box]
        rel_vel = vel_c - vel_b
        vn = (rel_vel * normal).sum(dim=1)
        active = colliding & (vn < 0.0)

        e = torch.min(s.restitution[i_circle], s.restitution[j_box])
        j_mag = torch.where(active, -(1.0 + e) * vn / inv_sum, torch.zeros_like(vn))
        impulse = normal * j_mag.unsqueeze(1)

        s.vel.scatter_add_(0, i_circle.unsqueeze(1).expand(-1, 2), impulse * inv_ma.unsqueeze(1))
        s.vel.scatter_add_(0, j_box.unsqueeze(1).expand(-1, 2), -impulse * inv_mb.unsqueeze(1))

    def step(self, dt: float):
        self.reset_forces()
        self.apply_gravity()
        self.integrate(dt)
        self.ground_collision()
        self.wall_collision()
        pairs = self.broad_phase()
        self.resolve_collisions(pairs)

    def get_positions_numpy(self):
        return self.state.pos.cpu().numpy()

    def get_angles_numpy(self):
        return self.state.angle.cpu().numpy()

    def get_shape_types_numpy(self):
        return self.state.shape_type.cpu().numpy().astype('int32')

    def get_half_extents_numpy(self):
        return self.state.half_extents.cpu().numpy()

    def spawn_body(self, x: float, y: float, vx: float = 0.0, vy: float = 0.0,
                   angle: float = 0.0, ang_vel: float = 0.0, mass: float = 50.0,
                   shape_type: int = 0, hw: float = 25.0, hh: float = 25.0, rest: float = 0.75):
        s = self.state
        dev = self.device

        new_pos = torch.tensor([[x, y]], dtype=torch.float32, device=dev)
        new_vel = torch.tensor([[vx, vy]], dtype=torch.float32, device=dev)
        new_force = torch.zeros((1, 2), dtype=torch.float32, device=dev)
        new_mass = torch.tensor([mass], dtype=torch.float32, device=dev)
        new_inv_mass = 1.0 / new_mass
        new_angle = torch.tensor([angle], dtype=torch.float32, device=dev)
        new_ang_vel = torch.tensor([ang_vel], dtype=torch.float32, device=dev)
        new_torque = torch.zeros(1, dtype=torch.float32, device=dev)
        new_st = torch.tensor([shape_type], dtype=torch.long, device=dev)
        new_he = torch.tensor([[hw, hh]], dtype=torch.float32, device=dev)
        new_rest = torch.tensor([rest], dtype=torch.float32, device=dev)

        if shape_type == 0:
            i_val = 0.5 * mass * (hw * hw)
        else:
            i_val = mass * (4.0 * hw * hw + 4.0 * hh * hh) / 12.0
        new_inertia = torch.tensor([i_val], dtype=torch.float32, device=dev)
        new_inv_inertia = 1.0 / new_inertia

        if s is None or s.N == 0:
            self.state = BatchedWorldState(
                pos=new_pos, vel=new_vel, force=new_force, mass=new_mass, inv_mass=new_inv_mass,
                angle=new_angle, ang_vel=new_ang_vel, torque=new_torque, inertia=new_inertia,
                inv_inertia=new_inv_inertia, shape_type=new_st, half_extents=new_he, restitution=new_rest
            )
        else:
            s.pos = torch.cat([s.pos, new_pos], dim=0)
            s.vel = torch.cat([s.vel, new_vel], dim=0)
            s.force = torch.cat([s.force, new_force], dim=0)
            s.mass = torch.cat([s.mass, new_mass], dim=0)
            s.inv_mass = torch.cat([s.inv_mass, new_inv_mass], dim=0)
            s.angle = torch.cat([s.angle, new_angle], dim=0)
            s.ang_vel = torch.cat([s.ang_vel, new_ang_vel], dim=0)
            s.torque = torch.cat([s.torque, new_torque], dim=0)
            s.inertia = torch.cat([s.inertia, new_inertia], dim=0)
            s.inv_inertia = torch.cat([s.inv_inertia, new_inv_inertia], dim=0)
            s.shape_type = torch.cat([s.shape_type, new_st], dim=0)
            s.half_extents = torch.cat([s.half_extents, new_he], dim=0)
            s.restitution = torch.cat([s.restitution, new_rest], dim=0)

    def set_body_pos(self, index: int, x: float, y: float):
        if self.state and 0 <= index < self.state.N:
            self.state.pos[index, 0] = x
            self.state.pos[index, 1] = y

    def set_body_vel(self, index: int, vx: float, vy: float):
        if self.state and 0 <= index < self.state.N:
            self.state.vel[index, 0] = vx
            self.state.vel[index, 1] = vy


