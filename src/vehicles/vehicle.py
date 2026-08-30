"""
Vehicle model with kinematic unicycle dynamics and ray-casting LiDAR sensing.

Provides the single-vehicle foundation used by both the baseline navigator
and the multi-agent STC coordinator.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class VehicleState:
    """Kinematic state of a ground vehicle in the plane."""

    x: float
    y: float
    theta: float  # heading (rad), 0 = +x
    v: float = 0.0  # forward speed (m/s)

    def position(self) -> np.ndarray:
        """Return (x, y) as a NumPy array."""
        return np.array([self.x, self.y], dtype=float)

    def copy(self) -> "VehicleState":
        return VehicleState(self.x, self.y, self.theta, self.v)


@dataclass
class LidarScan:
    """Result of a single ray-casting sweep."""

    angles: np.ndarray  # relative angles (rad)
    ranges: np.ndarray  # distances (m); max_range if free
    hit_points: np.ndarray  # (N, 2) world-frame endpoints


def _ray_circle_distance(
    origin: np.ndarray,
    direction: np.ndarray,
    center: np.ndarray,
    radius: float,
    max_range: float,
) -> float:
    """Smallest positive intersection distance of a ray with a circle, or max_range."""
    oc = origin - center
    b = 2.0 * float(np.dot(direction, oc))
    c = float(np.dot(oc, oc)) - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return max_range
    sqrt_disc = np.sqrt(disc)
    t0 = (-b - sqrt_disc) * 0.5
    t1 = (-b + sqrt_disc) * 0.5
    for t in (t0, t1):
        if 0.0 < t <= max_range:
            return float(t)
    return max_range


@dataclass
class Vehicle:
    """
    Rectangular unicycle vehicle with onboard LiDAR.

    Dynamics (discrete unicycle):
        x'     = x + v * cos(theta) * dt
        y'     = y + v * sin(theta) * dt
        theta' = theta + omega * dt
        v'     = clip(v + a * dt, 0, v_max)
    """

    vehicle_id: int
    state: VehicleState
    goal: np.ndarray
    color: Tuple[int, int, int] = (60, 120, 220)
    length: float = 1.6
    width: float = 1.0
    v_max: float = 4.0
    a_max: float = 2.5
    omega_max: float = 1.8
    lidar_rays: int = 36
    lidar_max_range: float = 12.0
    lidar_fov: float = 2.0 * np.pi
    safety_radius: float = 0.9
    trajectory: List[Tuple[float, float]] = field(default_factory=list)
    reached_goal: bool = False
    travel_time: float = 0.0
    collision_count: int = 0
    corridor_bounds: Optional[Tuple[float, float, float, float]] = None

    def __post_init__(self) -> None:
        self.goal = np.asarray(self.goal, dtype=float)
        self.record_pose()

    def sense_lidar(
        self,
        obstacles: Sequence[object],
        other_vehicles: Optional[Sequence["Vehicle"]] = None,
    ) -> LidarScan:
        """Ray-cast LiDAR against static obstacles and peer vehicle discs."""
        n = self.lidar_rays
        if self.lidar_fov < 2 * np.pi - 1e-6:
            half = self.lidar_fov * 0.5
            rel_angles = np.linspace(-half, half, n, endpoint=False)
        else:
            rel_angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
        world_angles = rel_angles + self.state.theta
        origin = self.state.position()
        ranges = np.full(n, self.lidar_max_range, dtype=float)
        hits = np.zeros((n, 2), dtype=float)

        for i, ang in enumerate(world_angles):
            direction = np.array([np.cos(ang), np.sin(ang)], dtype=float)
            d_min = self.lidar_max_range
            for obs in obstacles:
                d = obs.intersects_ray(origin, direction, self.lidar_max_range)
                if d < d_min:
                    d_min = d
            if other_vehicles:
                for peer in other_vehicles:
                    if peer.vehicle_id == self.vehicle_id:
                        continue
                    d = _ray_circle_distance(
                        origin, direction, peer.state.position(),
                        peer.safety_radius, self.lidar_max_range,
                    )
                    if d < d_min:
                        d_min = d
            ranges[i] = d_min
            hits[i] = origin + direction * d_min
        return LidarScan(angles=rel_angles, ranges=ranges, hit_points=hits)

    def step(self, a: float, omega: float, dt: float) -> None:
        """Integrate unicycle dynamics one step with control limits."""
        if self.reached_goal:
            self.state.v = 0.0
            return
        a = float(np.clip(a, -self.a_max, self.a_max))
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))
        v = float(np.clip(self.state.v + a * dt, 0.0, self.v_max))
        theta = self.state.theta + omega * dt
        theta = (theta + np.pi) % (2.0 * np.pi) - np.pi
        x = self.state.x + v * np.cos(theta) * dt
        y = self.state.y + v * np.sin(theta) * dt
        if self.corridor_bounds is not None:
            xmin, ymin, xmax, ymax = self.corridor_bounds
            margin = self.safety_radius * 0.5
            x = float(np.clip(x, xmin + margin, xmax - margin))
            y = float(np.clip(y, ymin + margin, ymax - margin))
        self.state = VehicleState(x=x, y=y, theta=theta, v=v)
        self.travel_time += dt
        self.record_pose()
        self._check_goal()

    def record_pose(self) -> None:
        self.trajectory.append((float(self.state.x), float(self.state.y)))

    def _check_goal(self, tol: float = 1.2) -> None:
        if np.linalg.norm(self.state.position() - self.goal) <= tol:
            self.reached_goal = True
            self.state.v = 0.0

    def distance_to_goal(self) -> float:
        return float(np.linalg.norm(self.state.position() - self.goal))

    def corners(self) -> np.ndarray:
        """Oriented rectangle corners in world frame, shape (4, 2)."""
        c, s = np.cos(self.state.theta), np.sin(self.state.theta)
        local = np.array([
            [self.length * 0.5, self.width * 0.5],
            [self.length * 0.5, -self.width * 0.5],
            [-self.length * 0.5, -self.width * 0.5],
            [-self.length * 0.5, self.width * 0.5],
        ], dtype=float)
        rot = np.array([[c, -s], [s, c]], dtype=float)
        return local @ rot.T + self.state.position()
