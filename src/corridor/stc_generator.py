"""
Spatio-Temporal Corridor generator.

Builds rectangular corridors along start-goal segments and resolves pairwise
spatial-temporal overlaps by shrinking, shifting, or time-slicing corridors
according to vehicle priority.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from .corridor import CorridorRequest, SpatioTemporalCorridor


CORRIDOR_PALETTE: List[Tuple[int, int, int]] = [
    (80, 160, 255),
    (255, 140, 90),
    (120, 220, 140),
    (220, 120, 200),
    (255, 210, 80),
    (100, 220, 220),
    (200, 160, 255),
    (255, 120, 140),
]


class STCGenerator:
    """
    Generate and repair non-overlapping STCs for a set of corridor requests.

    Algorithm (greedy, decentralized-friendly):
      1. Sort requests by descending priority (then by id for determinism).
      2. For each request, build a tube corridor around the start-goal segment.
      3. Against already-accepted corridors, detect space x time conflicts.
      4. Resolve by (a) lateral shift, (b) width shrink, or (c) time delay.
      5. Emit the accepted STC list and a conflict-resolution counter.
    """

    def __init__(
        self,
        world_size: float = 60.0,
        default_width: float = 5.0,
        min_width: float = 2.8,
        time_padding: float = 2.0,
        spatial_margin: float = 0.4,
        nominal_speed: float = 3.0,
    ) -> None:
        self.world_size = world_size
        self.default_width = default_width
        self.min_width = min_width
        self.time_padding = time_padding
        self.spatial_margin = spatial_margin
        self.nominal_speed = nominal_speed
        self._next_id = 0
        self.conflicts_resolved: int = 0

    def reset_counters(self) -> None:
        self._next_id = 0
        self.conflicts_resolved = 0

    def generate(
        self,
        requests: Sequence[CorridorRequest],
        existing: Optional[Sequence[SpatioTemporalCorridor]] = None,
    ) -> List[SpatioTemporalCorridor]:
        """Allocate STCs for requests without conflicting with existing."""
        accepted: List[SpatioTemporalCorridor] = list(existing) if existing else []
        ordered = sorted(requests, key=lambda r: (-r.priority, r.vehicle_id))
        new_corridors: List[SpatioTemporalCorridor] = []
        for req in ordered:
            stc = self._build_tube(req)
            stc = self._resolve_against(stc, accepted)
            stc.clamp_to_world(self.world_size)
            accepted.append(stc)
            new_corridors.append(stc)
        return new_corridors

    def regenerate_for_vehicle(
        self,
        request: CorridorRequest,
        others: Sequence[SpatioTemporalCorridor],
    ) -> SpatioTemporalCorridor:
        """Re-plan a single vehicle's corridor given peer STCs."""
        stc = self._build_tube(request)
        stc = self._resolve_against(stc, list(others))
        stc.clamp_to_world(self.world_size)
        return stc

    def _alloc_id(self) -> int:
        cid = self._next_id
        self._next_id += 1
        return cid

    def _build_tube(self, req: CorridorRequest) -> SpatioTemporalCorridor:
        """Axis-aligned bounding tube around start-goal with preferred width."""
        start, goal = req.start, req.goal
        width = max(req.preferred_width, self.min_width)
        pad = width * 0.5
        xmin = float(min(start[0], goal[0]) - pad)
        xmax = float(max(start[0], goal[0]) + pad)
        ymin = float(min(start[1], goal[1]) - pad)
        ymax = float(max(start[1], goal[1]) + pad)
        if xmax - xmin < width:
            cx = 0.5 * (xmin + xmax)
            xmin, xmax = cx - width * 0.5, cx + width * 0.5
        if ymax - ymin < width:
            cy = 0.5 * (ymin + ymax)
            ymin, ymax = cy - width * 0.5, cy + width * 0.5
        dist = float(np.linalg.norm(goal - start))
        travel = dist / max(self.nominal_speed, 0.5)
        t0 = float(req.t_start)
        t1 = float(max(req.t_end, t0 + travel + self.time_padding))
        color = CORRIDOR_PALETTE[req.vehicle_id % len(CORRIDOR_PALETTE)]
        waypoints = [start.copy(), 0.5 * (start + goal), goal.copy()]
        return SpatioTemporalCorridor(
            corridor_id=self._alloc_id(),
            vehicle_id=req.vehicle_id,
            xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax,
            t_start=t0, t_end=t1, color=color, waypoints=waypoints,
        )

    def _resolve_against(
        self,
        stc: SpatioTemporalCorridor,
        others: List[SpatioTemporalCorridor],
    ) -> SpatioTemporalCorridor:
        """Iteratively repair stc until it no longer conflicts with others."""
        max_iters = 12
        for _ in range(max_iters):
            conflict = None
            for other in others:
                if stc.conflicts_with(other, margin=self.spatial_margin):
                    conflict = other
                    break
            if conflict is None:
                return stc
            self.conflicts_resolved += 1
            stc = self._repair(stc, conflict)
        latest_end = stc.t_start
        for other in others:
            if stc.spatial_overlap(other, margin=self.spatial_margin):
                latest_end = max(latest_end, other.t_end + 0.5)
        duration = stc.t_end - stc.t_start
        stc.t_start = latest_end
        stc.t_end = latest_end + duration
        self.conflicts_resolved += 1
        return stc

    def _repair(
        self,
        stc: SpatioTemporalCorridor,
        other: SpatioTemporalCorridor,
    ) -> SpatioTemporalCorridor:
        """Try spatial shift, then shrink, then time delay."""
        sc, oc = stc.center, other.center
        delta = sc - oc
        if float(np.linalg.norm(delta)) < 1e-6:
            delta = np.array([1.0, 0.0])
        direction = delta / (float(np.linalg.norm(delta)) + 1e-9)
        shift = direction * 2.2
        shifted = SpatioTemporalCorridor(
            corridor_id=stc.corridor_id, vehicle_id=stc.vehicle_id,
            xmin=stc.xmin + shift[0], ymin=stc.ymin + shift[1],
            xmax=stc.xmax + shift[0], ymax=stc.ymax + shift[1],
            t_start=stc.t_start, t_end=stc.t_end,
            color=stc.color, waypoints=list(stc.waypoints),
        )
        shifted.clamp_to_world(self.world_size)
        if not shifted.conflicts_with(other, margin=self.spatial_margin):
            return shifted

        cx, cy = stc.center
        new_w = max(self.min_width, stc.width * 0.75)
        new_h = max(self.min_width, stc.height * 0.75)
        shrunk = SpatioTemporalCorridor(
            corridor_id=stc.corridor_id, vehicle_id=stc.vehicle_id,
            xmin=cx - new_w * 0.5, ymin=cy - new_h * 0.5,
            xmax=cx + new_w * 0.5, ymax=cy + new_h * 0.5,
            t_start=stc.t_start, t_end=stc.t_end,
            color=stc.color, waypoints=list(stc.waypoints),
        )
        shrunk.clamp_to_world(self.world_size)
        if not shrunk.conflicts_with(other, margin=self.spatial_margin):
            return shrunk

        duration = stc.t_end - stc.t_start
        return SpatioTemporalCorridor(
            corridor_id=stc.corridor_id, vehicle_id=stc.vehicle_id,
            xmin=stc.xmin, ymin=stc.ymin, xmax=stc.xmax, ymax=stc.ymax,
            t_start=other.t_end + 0.4, t_end=other.t_end + 0.4 + duration,
            color=stc.color, waypoints=list(stc.waypoints),
        )
