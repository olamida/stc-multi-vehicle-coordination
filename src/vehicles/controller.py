"""
Low-level navigation controllers.

* ``VehicleController`` – reactive goal-seeking with LiDAR obstacle avoidance
  (single-vehicle baseline / master's-level foundation).
* Corridor-aware variant keeps the vehicle inside an assigned STC slice.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from .vehicle import LidarScan, Vehicle


class VehicleController:
    """
    Potential-field style unicycle controller.

    Attractive force pulls toward the goal (or a corridor-constrained waypoint).
    Repulsive force is derived from short LiDAR returns. Optional corridor walls
    add soft lateral repulsion so the agent stays inside its STC.
    """

    def __init__(
        self,
        k_attr: float = 1.4,
        k_rep: float = 2.8,
        k_corridor: float = 3.5,
        influence_radius: float = 5.0,
        goal_slow_radius: float = 4.0,
        desired_speed: float = 3.2,
    ) -> None:
        self.k_attr = k_attr
        self.k_rep = k_rep
        self.k_corridor = k_corridor
        self.influence_radius = influence_radius
        self.goal_slow_radius = goal_slow_radius
        self.desired_speed = desired_speed

    def compute_control(
        self,
        vehicle: Vehicle,
        scan: LidarScan,
        waypoint: Optional[np.ndarray] = None,
    ) -> Tuple[float, float]:
        """
        Compute linear acceleration ``a`` and angular rate ``omega``.

        Parameters
        ----------
        vehicle :
            Ego vehicle.
        scan :
            Latest LiDAR measurement.
        waypoint :
            Optional intermediate target (defaults to vehicle.goal).
        """
        if vehicle.reached_goal:
            return 0.0, 0.0

        target = np.asarray(waypoint if waypoint is not None else vehicle.goal, dtype=float)
        pos = vehicle.state.position()
        to_goal = target - pos
        dist = float(np.linalg.norm(to_goal)) + 1e-9
        attr = self.k_attr * (to_goal / dist)

        # LiDAR repulsion (inverse-distance along hit rays inside influence radius)
        rep = np.zeros(2, dtype=float)
        for i, r in enumerate(scan.ranges):
            if r >= self.influence_radius:
                continue
            # Direction from hit back toward vehicle (repulsion)
            hit = scan.hit_points[i]
            away = pos - hit
            d = float(np.linalg.norm(away)) + 1e-6
            strength = self.k_rep * (1.0 / d - 1.0 / self.influence_radius) / (d * d)
            rep += strength * (away / d)

        # Soft corridor boundary repulsion
        if vehicle.corridor_bounds is not None:
            xmin, ymin, xmax, ymax = vehicle.corridor_bounds
            margin = vehicle.safety_radius + 0.4
            cx = pos[0]
            cy = pos[1]
            if cx - xmin < margin:
                rep[0] += self.k_corridor * (margin - (cx - xmin))
            if xmax - cx < margin:
                rep[0] -= self.k_corridor * (margin - (xmax - cx))
            if cy - ymin < margin:
                rep[1] += self.k_corridor * (margin - (cy - ymin))
            if ymax - cy < margin:
                rep[1] -= self.k_corridor * (margin - (ymax - cy))

        force = attr + rep
        force_norm = float(np.linalg.norm(force)) + 1e-9
        desired_heading = float(np.arctan2(force[1], force[0]))

        heading_err = _angle_diff(desired_heading, vehicle.state.theta)
        omega = float(np.clip(2.2 * heading_err, -vehicle.omega_max, vehicle.omega_max))

        # Speed profile: slow near goal, and when turning hard / obstacles close
        speed_scale = min(1.0, dist / self.goal_slow_radius)
        min_range = float(np.min(scan.ranges)) if len(scan.ranges) else vehicle.lidar_max_range
        if min_range < 2.5:
            speed_scale *= max(0.15, min_range / 2.5)
        # Reduce speed when heading error is large
        speed_scale *= max(0.25, 1.0 - 0.6 * abs(heading_err) / np.pi)

        v_des = self.desired_speed * speed_scale
        # Cap by force magnitude so free-space aggression stays bounded
        v_des = min(v_des, 0.5 + 1.5 * min(force_norm, 2.0))
        v_des = min(v_des, vehicle.v_max)

        a = float(np.clip(1.8 * (v_des - vehicle.state.v), -vehicle.a_max, vehicle.a_max))
        return a, omega

    def corridor_waypoint(self, vehicle: Vehicle, look_ahead: float = 6.0) -> np.ndarray:
        """
        Project a look-ahead waypoint toward the goal, clamped inside the corridor.
        """
        pos = vehicle.state.position()
        goal = vehicle.goal
        direction = goal - pos
        dist = float(np.linalg.norm(direction))
        if dist < 1e-6:
            return goal.copy()
        step = min(look_ahead, dist)
        wp = pos + (direction / dist) * step

        if vehicle.corridor_bounds is not None:
            xmin, ymin, xmax, ymax = vehicle.corridor_bounds
            m = vehicle.safety_radius
            wp = np.array(
                [
                    float(np.clip(wp[0], xmin + m, xmax - m)),
                    float(np.clip(wp[1], ymin + m, ymax - m)),
                ],
                dtype=float,
            )
        return wp


def _angle_diff(a: float, b: float) -> float:
    """Signed difference a - b wrapped to [-pi, pi]."""
    d = (a - b + np.pi) % (2.0 * np.pi) - np.pi
    return float(d)
