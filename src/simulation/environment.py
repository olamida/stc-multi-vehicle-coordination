"""
Simulation environment: world, obstacles, metrics, and scenario builders.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from src.vehicles.vehicle import Vehicle, VehicleState
from src.coordination.decentralized import VEHICLE_COLORS


@dataclass
class Obstacle:
    """Axis-aligned rectangular static obstacle."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            self.xmin - margin <= x <= self.xmax + margin
            and self.ymin - margin <= y <= self.ymax + margin
        )

    def intersects_ray(
        self, origin: np.ndarray, direction: np.ndarray, max_range: float
    ) -> float:
        """Ray vs AABB slab method; returns hit distance or max_range."""
        inv = np.zeros(2)
        safe = np.abs(direction) >= 1e-12
        inv[safe] = 1.0 / direction[safe]
        inv[~safe] = 1e12
        t1 = (self.xmin - origin[0]) * inv[0]
        t2 = (self.xmax - origin[0]) * inv[0]
        t3 = (self.ymin - origin[1]) * inv[1]
        t4 = (self.ymax - origin[1]) * inv[1]
        tmin = max(min(t1, t2), min(t3, t4))
        tmax = min(max(t1, t2), max(t3, t4))
        if tmax < 0 or tmin > tmax:
            return max_range
        hit = tmin if tmin > 0 else tmax
        if 0.0 < hit <= max_range:
            return float(hit)
        return max_range

    @property
    def rect(self) -> Tuple[float, float, float, float]:
        return (self.xmin, self.ymin, self.xmax, self.ymax)


@dataclass
class SimulationMetrics:
    """Aggregate performance metrics for a simulation run."""

    n_vehicles: int = 0
    success_count: int = 0
    success_rate: float = 0.0
    average_travel_time: float = 0.0
    total_travel_time: float = 0.0
    conflicts_resolved: int = 0
    collisions: int = 0
    sim_time: float = 0.0
    mode: str = "stc"

    def finalize(self, vehicles: Sequence[Vehicle], conflicts: int, sim_time: float) -> None:
        self.n_vehicles = len(vehicles)
        self.success_count = sum(1 for v in vehicles if v.reached_goal)
        self.success_rate = self.success_count / max(1, self.n_vehicles)
        times = [v.travel_time for v in vehicles if v.reached_goal]
        self.average_travel_time = float(np.mean(times)) if times else float("nan")
        self.total_travel_time = float(sum(v.travel_time for v in vehicles))
        self.conflicts_resolved = conflicts
        self.collisions = int(sum(v.collision_count for v in vehicles) // 2)  # pairwise
        self.sim_time = sim_time

    def as_dict(self) -> dict:
        return {
            "mode": self.mode,
            "n_vehicles": self.n_vehicles,
            "success_count": self.success_count,
            "success_rate": round(self.success_rate, 3),
            "average_travel_time": round(self.average_travel_time, 3)
            if self.average_travel_time == self.average_travel_time
            else None,
            "total_travel_time": round(self.total_travel_time, 3),
            "conflicts_resolved": self.conflicts_resolved,
            "collisions": self.collisions,
            "sim_time": round(self.sim_time, 3),
        }

    def __str__(self) -> str:
        d = self.as_dict()
        lines = ["=== Simulation Metrics ==="]
        for k, v in d.items():
            lines.append(f"  {k}: {v}")
        return "\n".join(lines)


@dataclass
class Environment:
    """
    2D world containing obstacles and vehicles.

    Supports single-vehicle baseline mode and multi-vehicle STC mode.
    """

    world_size: float = 60.0
    obstacles: List[Obstacle] = field(default_factory=list)
    vehicles: List[Vehicle] = field(default_factory=list)
    dt: float = 0.1
    time: float = 0.0
    collision_distance: float = 1.6

    def add_obstacle(self, xmin: float, ymin: float, xmax: float, ymax: float) -> None:
        self.obstacles.append(Obstacle(xmin, ymin, xmax, ymax))

    def add_vehicle(self, vehicle: Vehicle) -> None:
        self.vehicles.append(vehicle)

    def check_collisions(self) -> int:
        """Detect vehicle-vehicle and vehicle-obstacle collisions; return new count."""
        new_hits = 0
        n = len(self.vehicles)
        for i in range(n):
            vi = self.vehicles[i]
            if vi.reached_goal:
                continue
            pi = vi.state.position()
            for obs in self.obstacles:
                if obs.contains(pi[0], pi[1], margin=vi.safety_radius * 0.6):
                    vi.collision_count += 1
                    new_hits += 1
            for j in range(i + 1, n):
                vj = self.vehicles[j]
                if vj.reached_goal:
                    continue
                d = float(np.linalg.norm(pi - vj.state.position()))
                if d < self.collision_distance:
                    vi.collision_count += 1
                    vj.collision_count += 1
                    new_hits += 1
        return new_hits

    def all_done(self) -> bool:
        return all(v.reached_goal for v in self.vehicles)

    def clamp_vehicles_to_world(self) -> None:
        m = 1.0
        for v in self.vehicles:
            v.state.x = float(np.clip(v.state.x, m, self.world_size - m))
            v.state.y = float(np.clip(v.state.y, m, self.world_size - m))


def _heading_toward(start: np.ndarray, goal: np.ndarray) -> float:
    d = goal - start
    return float(np.arctan2(d[1], d[0]))


def build_single_vehicle_scenario(world_size: float = 60.0) -> Environment:
    """Baseline scenario: one vehicle, obstacles, goal-reaching with LiDAR."""
    env = Environment(world_size=world_size)
    env.add_obstacle(18, 15, 26, 28)
    env.add_obstacle(32, 30, 42, 38)
    env.add_obstacle(10, 40, 18, 48)
    start = np.array([8.0, 8.0])
    goal = np.array([52.0, 52.0])
    v = Vehicle(
        vehicle_id=0,
        state=VehicleState(x=start[0], y=start[1], theta=_heading_toward(start, goal), v=0.0),
        goal=goal,
        color=VEHICLE_COLORS[0],
    )
    env.add_vehicle(v)
    return env


def build_multi_vehicle_scenario(
    n_vehicles: int = 6,
    world_size: float = 60.0,
    seed: int = 42,
) -> Environment:
    """
    Multi-vehicle lane / intersection scenario for 4-8 agents.

    Each vehicle travels along a thin horizontal or vertical lane corridor.
    Perpendicular lanes cross at small right-angle junctions, the classic
    spatio-temporal-corridor problem: corridors overlap only in narrow,
    localised regions that STC resolves by time-slicing or lateral offset.
    """
    n_vehicles = int(np.clip(n_vehicles, 4, 8))
    rng = np.random.default_rng(seed)
    env = Environment(world_size=world_size)

    # Obstacles sit in the corners, clear of every lane, enriching the scene
    # and exercising LiDAR without blocking the corridors.
    env.add_obstacle(world_size - 8.0, world_size - 8.0, world_size - 2.0, world_size - 2.0)
    env.add_obstacle(2.0, 2.0, 8.0, 8.0)
    env.add_obstacle(2.0, world_size - 8.0, 8.0, world_size - 2.0)
    env.add_obstacle(world_size - 8.0, 2.0, world_size - 2.0, 8.0)

    # Thin lanes: horizontal (move along x) and vertical (move along y).
    # Parallel lanes are spaced 10 m apart so 5 m-wide strips never touch;
    # only perpendicular lanes form cross junctions.
    h_lanes = [
        (8.0, 12.0, world_size - 8.0, 12.0),   # right
        (world_size - 8.0, 22.0, 8.0, 22.0),   # left
        (8.0, 32.0, world_size - 8.0, 32.0),   # right
        (world_size - 8.0, 42.0, 8.0, 42.0),   # left
    ]
    v_lanes = [
        (12.0, 8.0, 12.0, world_size - 8.0),   # up
        (22.0, world_size - 8.0, 22.0, 8.0),   # down
        (32.0, 8.0, 32.0, world_size - 8.0),   # up
        (42.0, world_size - 8.0, 42.0, 8.0),   # down
    ]
    lanes = h_lanes + v_lanes

    for i in range(n_vehicles):
        x0, y0, x1, y1 = lanes[i % len(lanes)]
        start = np.array([x0, y0], dtype=float)
        goal = np.array([x1, y1], dtype=float)
        jitter = rng.uniform(-0.8, 0.8, size=2)
        start = start + jitter
        goal = goal + jitter
        v = Vehicle(
            vehicle_id=i,
            state=VehicleState(
                x=float(start[0]),
                y=float(start[1]),
                theta=_heading_toward(start, goal),
                v=0.0,
            ),
            goal=goal,
            color=VEHICLE_COLORS[i % len(VEHICLE_COLORS)],
        )
        env.add_vehicle(v)
    return env
