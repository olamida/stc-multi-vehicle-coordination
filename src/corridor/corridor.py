"""
Spatio-Temporal Corridor (STC) data structures.

An STC is a rectangular prism in (x, y, t): an axis-aligned spatial box that is
exclusively reserved for one vehicle over a closed time interval. Non-overlapping
STCs guarantee collision-free motion when each agent plans strictly inside its
own corridor.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class CorridorRequest:
    """
    Lightweight message a vehicle broadcasts when asking for right-of-way space.

    Used by the decentralized coordinator to negotiate non-overlapping STCs.
    """

    vehicle_id: int
    start: np.ndarray  # (x, y)
    goal: np.ndarray  # (x, y)
    t_start: float
    t_end: float
    preferred_width: float = 4.0
    priority: float = 0.0  # higher wins; often 1/distance or arrival urgency

    def __post_init__(self) -> None:
        self.start = np.asarray(self.start, dtype=float)
        self.goal = np.asarray(self.goal, dtype=float)


@dataclass
class SpatioTemporalCorridor:
    """
    Axis-aligned spatio-temporal corridor.

    Spatial extent : [xmin, xmax] × [ymin, ymax]
    Temporal extent: [t_start, t_end]
    """

    corridor_id: int
    vehicle_id: int
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    t_start: float
    t_end: float
    color: Tuple[int, int, int] = (100, 180, 255)
    waypoints: List[np.ndarray] = field(default_factory=list)

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    @property
    def width(self) -> float:
        return self.xmax - self.xmin

    @property
    def height(self) -> float:
        return self.ymax - self.ymin

    @property
    def center(self) -> np.ndarray:
        return np.array([(self.xmin + self.xmax) * 0.5, (self.ymin + self.ymax) * 0.5], dtype=float)

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        return (self.xmin, self.ymin, self.xmax, self.ymax)

    def contains_point(self, x: float, y: float, t: Optional[float] = None, margin: float = 0.0) -> bool:
        """Return True if (x, y[, t]) lies inside the corridor."""
        if not (self.xmin + margin <= x <= self.xmax - margin and self.ymin + margin <= y <= self.ymax - margin):
            return False
        if t is not None and not (self.t_start <= t <= self.t_end):
            return False
        return True

    def active_at(self, t: float) -> bool:
        return self.t_start <= t <= self.t_end

    def spatial_overlap(self, other: "SpatioTemporalCorridor", margin: float = 0.0) -> bool:
        """True if the spatial rectangles overlap (ignoring time)."""
        return not (
            self.xmax - margin <= other.xmin + margin
            or other.xmax - margin <= self.xmin + margin
            or self.ymax - margin <= other.ymin + margin
            or other.ymax - margin <= self.ymin + margin
        )

    def temporal_overlap(self, other: "SpatioTemporalCorridor") -> bool:
        return not (self.t_end < other.t_start or other.t_end < self.t_start)

    def conflicts_with(self, other: "SpatioTemporalCorridor", margin: float = 0.3) -> bool:
        """Conflict iff both space and time overlap for different vehicles."""
        if self.vehicle_id == other.vehicle_id:
            return False
        return self.spatial_overlap(other, margin=margin) and self.temporal_overlap(other)

    def inflate(self, dx: float, dy: float) -> "SpatioTemporalCorridor":
        """Return a copy expanded by dx/dy on each side."""
        return SpatioTemporalCorridor(
            corridor_id=self.corridor_id,
            vehicle_id=self.vehicle_id,
            xmin=self.xmin - dx,
            ymin=self.ymin - dy,
            xmax=self.xmax + dx,
            ymax=self.ymax + dy,
            t_start=self.t_start,
            t_end=self.t_end,
            color=self.color,
            waypoints=list(self.waypoints),
        )

    def clamp_to_world(self, world_size: float) -> None:
        self.xmin = float(np.clip(self.xmin, 0.0, world_size))
        self.ymin = float(np.clip(self.ymin, 0.0, world_size))
        self.xmax = float(np.clip(self.xmax, 0.0, world_size))
        self.ymax = float(np.clip(self.ymax, 0.0, world_size))

    def as_dict(self) -> dict:
        return {
            "corridor_id": self.corridor_id,
            "vehicle_id": self.vehicle_id,
            "bounds": self.bounds,
            "t_start": self.t_start,
            "t_end": self.t_end,
        }

    def __repr__(self) -> str:
        return (
            f"STC(id={self.corridor_id}, v={self.vehicle_id}, "
            f"xy=[{self.xmin:.1f},{self.ymin:.1f}]-[{self.xmax:.1f},{self.ymax:.1f}], "
            f"t=[{self.t_start:.1f},{self.t_end:.1f}])"
        )
