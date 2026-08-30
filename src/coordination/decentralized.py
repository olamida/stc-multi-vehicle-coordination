"""
Decentralized multi-vehicle coordination via shared corridor requests.

Each vehicle broadcasts its intended goal and a corridor request. Peers
observe the shared message bus and the STC generator allocates non-overlapping
spatio-temporal corridors. Vehicles plan trajectories inside their assigned STC.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from src.corridor.corridor import CorridorRequest, SpatioTemporalCorridor
from src.corridor.stc_generator import STCGenerator
from src.vehicles.controller import VehicleController
from src.vehicles.vehicle import Vehicle


VEHICLE_COLORS: List[Tuple[int, int, int]] = [
    (66, 135, 245),
    (245, 130, 66),
    (66, 200, 120),
    (200, 80, 180),
    (240, 200, 50),
    (50, 200, 200),
    (170, 120, 255),
    (240, 90, 110),
]


@dataclass
class SharedMessage:
    """Broadcast message exchanged among vehicles (goals + corridor intent)."""

    vehicle_id: int
    position: np.ndarray
    goal: np.ndarray
    t: float
    priority: float
    corridor_request: Optional[CorridorRequest] = None
    status: str = "moving"  # moving | waiting | done


@dataclass
class DecentralizedCoordinator:
    """
    Lightweight decentralized coordinator.

    A single process hosts the shared bus while each agent only uses locally
    visible messages and its own STC for motion planning.
    """

    world_size: float = 60.0
    replan_interval: float = 3.0
    corridor_width: float = 5.0
    generator: STCGenerator = field(default_factory=STCGenerator)
    controller: VehicleController = field(default_factory=VehicleController)
    corridors: Dict[int, SpatioTemporalCorridor] = field(default_factory=dict)
    message_bus: List[SharedMessage] = field(default_factory=list)
    last_replan_t: float = -1e9
    total_conflicts_resolved: int = 0

    def __post_init__(self) -> None:
        self.generator = STCGenerator(world_size=self.world_size)
        self.controller = VehicleController()

    def assign_initial_corridors(self, vehicles: Sequence[Vehicle], t: float = 0.0) -> None:
        """Build initial non-overlapping STCs from each vehicle start/goal."""
        self.generator.reset_counters()
        requests = [self._make_request(v, t) for v in vehicles]
        allocated = self.generator.generate(requests)
        self.total_conflicts_resolved += self.generator.conflicts_resolved
        self.corridors.clear()
        for stc in allocated:
            self.corridors[stc.vehicle_id] = stc
            for v in vehicles:
                if v.vehicle_id == stc.vehicle_id:
                    v.corridor_bounds = stc.bounds
                    v.color = VEHICLE_COLORS[v.vehicle_id % len(VEHICLE_COLORS)]
                    break
        self.last_replan_t = t
        self._publish_all(vehicles, t)

    def _make_request(self, vehicle: Vehicle, t: float) -> CorridorRequest:
        dist = vehicle.distance_to_goal()
        priority = 1.0 / (dist + 1.0) + 0.05 * vehicle.state.v
        return CorridorRequest(
            vehicle_id=vehicle.vehicle_id,
            start=vehicle.state.position(),
            goal=vehicle.goal,
            t_start=t,
            t_end=t + dist / 2.5 + 4.0,
            preferred_width=self.corridor_width,
            priority=priority,
        )

    def _publish_all(self, vehicles: Sequence[Vehicle], t: float) -> None:
        self.message_bus.clear()
        for v in vehicles:
            req = self._make_request(v, t)
            status = "done" if v.reached_goal else "moving"
            if v.vehicle_id in self.corridors:
                stc = self.corridors[v.vehicle_id]
                if t < stc.t_start and not v.reached_goal:
                    status = "waiting"
            self.message_bus.append(
                SharedMessage(
                    vehicle_id=v.vehicle_id,
                    position=v.state.position(),
                    goal=v.goal.copy(),
                    t=t,
                    priority=req.priority,
                    corridor_request=req,
                    status=status,
                )
            )

    def maybe_replan(self, vehicles: Sequence[Vehicle], t: float) -> bool:
        """Periodically renegotiate corridors from shared intents."""
        if t - self.last_replan_t < self.replan_interval:
            return False
        active = [v for v in vehicles if not v.reached_goal]
        if not active:
            return False
        self.generator.reset_counters()
        requests = [self._make_request(v, t) for v in active]
        allocated = self.generator.generate(requests)
        self.total_conflicts_resolved += self.generator.conflicts_resolved
        for stc in allocated:
            self.corridors[stc.vehicle_id] = stc
            for v in active:
                if v.vehicle_id == stc.vehicle_id:
                    v.corridor_bounds = stc.bounds
                    break
        self.last_replan_t = t
        self._publish_all(vehicles, t)
        return True

    def step_vehicle(
        self,
        vehicle: Vehicle,
        obstacles: Sequence[object],
        peers: Sequence[Vehicle],
        t: float,
        dt: float,
    ) -> None:
        """Sense, plan inside STC, and integrate one control step."""
        if vehicle.reached_goal:
            return
        stc = self.corridors.get(vehicle.vehicle_id)
        if stc is not None:
            vehicle.corridor_bounds = stc.bounds
            if t < stc.t_start:
                vehicle.state.v = max(0.0, vehicle.state.v - vehicle.a_max * dt)
                vehicle.record_pose()
                vehicle.travel_time += dt
                return
        scan = vehicle.sense_lidar(obstacles, other_vehicles=peers)
        waypoint = self.controller.corridor_waypoint(vehicle)
        a, omega = self.controller.compute_control(vehicle, scan, waypoint=waypoint)
        vehicle.step(a, omega, dt)

    def step_all(
        self,
        vehicles: Sequence[Vehicle],
        obstacles: Sequence[object],
        t: float,
        dt: float,
    ) -> None:
        """One coordinated simulation tick for every vehicle."""
        self.maybe_replan(vehicles, t)
        snapshot = list(vehicles)
        for v in vehicles:
            self.step_vehicle(v, obstacles, snapshot, t, dt)
        self._publish_all(vehicles, t + dt)
