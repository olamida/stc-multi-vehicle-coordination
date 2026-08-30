"""
Pygame real-time visualizer with optional GIF export via Pillow.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import pygame
    from pygame import gfxdraw
except ImportError as exc:  # pragma: no cover
    raise ImportError("pygame is required: pip install pygame") from exc

from src.corridor.corridor import SpatioTemporalCorridor
from src.simulation.environment import Environment, SimulationMetrics
from src.vehicles.vehicle import Vehicle


class Visualizer:
    """
    Professional 2D top-down renderer.

    - Vehicles: filled oriented rectangles + ID label + heading notch
    - Corridors: semi-transparent axis-aligned rectangles
    - Trajectories: polylines in vehicle color
    - Goals: hollow diamonds
    - LiDAR (optional): faint rays for ego / all agents
    - HUD: time, mode, metrics
    - GIF: frame buffer saved with Pillow at the end
    """

    def __init__(
        self,
        world_size: float = 60.0,
        window_size: int = 800,
        caption: str = "STC Multi-Agent",
        show_lidar: bool = False,
        record_gif: bool = False,
        gif_path: Optional[str] = None,
        gif_scale: float = 0.5,
        gif_every_n: int = 2,
        target_fps: int = 30,
    ) -> None:
        self.world_size = world_size
        self.window_size = window_size
        self.scale = window_size / world_size
        self.show_lidar = show_lidar
        self.record_gif = record_gif
        self.gif_path = gif_path
        self.gif_scale = gif_scale
        self.gif_every_n = max(1, gif_every_n)
        self.target_fps = target_fps
        self._frames: List["Image.Image"] = []
        self._frame_i = 0

        pygame.init()
        pygame.display.set_caption(caption)
        self.screen = pygame.display.set_mode((window_size, window_size + 56))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 16)
        self.font_sm = pygame.font.SysFont("consolas", 13)
        self.font_lg = pygame.font.SysFont("consolas", 18, bold=True)
        self.running = True
        self.paused = False

        # Colors
        self.bg = (18, 22, 30)
        self.grid = (32, 38, 50)
        self.panel = (24, 28, 38)
        self.text = (220, 225, 235)
        self.muted = (140, 150, 170)
        self.obstacle_fill = (55, 62, 78)
        self.obstacle_edge = (90, 100, 120)

    # ------------------------------------------------------------------
    # Coordinate transforms (world y-up -> screen y-down)
    # ------------------------------------------------------------------
    def w2s(self, x: float, y: float) -> Tuple[int, int]:
        sx = int(x * self.scale)
        sy = int((self.world_size - y) * self.scale)
        return sx, sy

    def w2s_arr(self, pts: np.ndarray) -> List[Tuple[int, int]]:
        return [self.w2s(float(p[0]), float(p[1])) for p in pts]

    # ------------------------------------------------------------------
    # Drawing primitives
    # ------------------------------------------------------------------
    def _draw_grid(self) -> None:
        step = 5.0
        for g in np.arange(0, self.world_size + 1e-6, step):
            c = self.grid
            pygame.draw.line(self.screen, c, self.w2s(g, 0), self.w2s(g, self.world_size), 1)
            pygame.draw.line(self.screen, c, self.w2s(0, g), self.w2s(self.world_size, g), 1)

    def _draw_obstacles(self, env: Environment) -> None:
        for obs in env.obstacles:
            x0, y0 = self.w2s(obs.xmin, obs.ymax)
            x1, y1 = self.w2s(obs.xmax, obs.ymin)
            rect = pygame.Rect(x0, y0, x1 - x0, y1 - y0)
            pygame.draw.rect(self.screen, self.obstacle_fill, rect)
            pygame.draw.rect(self.screen, self.obstacle_edge, rect, 2)

    def _draw_corridor(self, stc: SpatioTemporalCorridor, t: float) -> None:
        active = stc.active_at(t)
        alpha = 70 if active else 28
        x0, y0 = self.w2s(stc.xmin, stc.ymax)
        x1, y1 = self.w2s(stc.xmax, stc.ymin)
        w, h = max(1, x1 - x0), max(1, y1 - y0)
        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        r, g, b = stc.color
        surf.fill((r, g, b, alpha))
        self.screen.blit(surf, (x0, y0))
        border = 2 if active else 1
        pygame.draw.rect(self.screen, (*stc.color, ), pygame.Rect(x0, y0, w, h), border)

    def _draw_trajectory(self, vehicle: Vehicle) -> None:
        traj = vehicle.trajectory
        if len(traj) < 2:
            return
        # downsample for performance
        step = max(1, len(traj) // 200)
        pts = [self.w2s(x, y) for x, y in traj[::step]]
        if len(pts) >= 2:
            pygame.draw.lines(self.screen, vehicle.color, False, pts, 2)

    def _draw_goal(self, vehicle: Vehicle) -> None:
        gx, gy = self.w2s(float(vehicle.goal[0]), float(vehicle.goal[1]))
        s = 7
        diamond = [(gx, gy - s), (gx + s, gy), (gx, gy + s), (gx - s, gy)]
        pygame.draw.polygon(self.screen, vehicle.color, diamond, 2)

    def _draw_vehicle(self, vehicle: Vehicle) -> None:
        corners = vehicle.corners()
        pts = self.w2s_arr(corners)
        pygame.draw.polygon(self.screen, vehicle.color, pts)
        # darker edge
        edge = tuple(max(0, c - 40) for c in vehicle.color)
        pygame.draw.polygon(self.screen, edge, pts, 2)
        # heading notch at front midpoint
        front = 0.5 * (corners[0] + corners[1])
        cx, cy = self.w2s(float(vehicle.state.x), float(vehicle.state.y))
        fx, fy = self.w2s(float(front[0]), float(front[1]))
        pygame.draw.circle(self.screen, (255, 255, 255), (fx, fy), 3)
        # ID
        label = self.font_sm.render(str(vehicle.vehicle_id), True, (15, 15, 20))
        self.screen.blit(label, (cx - 4, cy - 6))
        if vehicle.reached_goal:
            pygame.draw.circle(self.screen, (80, 255, 120), (cx, cy), int(1.2 * self.scale), 2)

    def _draw_lidar(self, vehicle: Vehicle, env: Environment) -> None:
        peers = [v for v in env.vehicles if v.vehicle_id != vehicle.vehicle_id]
        scan = vehicle.sense_lidar(env.obstacles, peers)
        ox, oy = self.w2s(vehicle.state.x, vehicle.state.y)
        for hit in scan.hit_points:
            hx, hy = self.w2s(float(hit[0]), float(hit[1]))
            pygame.draw.aaline(self.screen, (60, 80, 70), (ox, oy), (hx, hy))

    def _draw_hud(
        self,
        env: Environment,
        mode: str,
        metrics: Optional[SimulationMetrics] = None,
        extra: str = "",
    ) -> None:
        panel = pygame.Rect(0, self.window_size, self.window_size, 56)
        pygame.draw.rect(self.screen, self.panel, panel)
        pygame.draw.line(
            self.screen, (50, 58, 75),
            (0, self.window_size), (self.window_size, self.window_size), 1,
        )
        done = sum(1 for v in env.vehicles if v.reached_goal)
        n = len(env.vehicles)
        line1 = f"t={env.time:6.1f}s   mode={mode}   goals={done}/{n}"
        if metrics is not None:
            line1 += f"   collisions={metrics.collisions}   conflicts={metrics.conflicts_resolved}"
        txt1 = self.font.render(line1, True, self.text)
        self.screen.blit(txt1, (12, self.window_size + 8))
        help_txt = "SPACE pause | L lidar | G gif-dump | ESC quit"
        if extra:
            help_txt = extra + "  |  " + help_txt
        txt2 = self.font_sm.render(help_txt, True, self.muted)
        self.screen.blit(txt2, (12, self.window_size + 32))

    # ------------------------------------------------------------------
    # Frame lifecycle
    # ------------------------------------------------------------------
    def handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_l:
                    self.show_lidar = not self.show_lidar
                elif event.key == pygame.K_g:
                    self.record_gif = True

    def render(
        self,
        env: Environment,
        corridors: Optional[Dict[int, SpatioTemporalCorridor]] = None,
        mode: str = "stc",
        metrics: Optional[SimulationMetrics] = None,
        extra: str = "",
    ) -> None:
        self.handle_events()
        self.screen.fill(self.bg)
        self._draw_grid()
        self._draw_obstacles(env)

        if corridors:
            for stc in corridors.values():
                self._draw_corridor(stc, env.time)

        for v in env.vehicles:
            self._draw_trajectory(v)
            self._draw_goal(v)

        if self.show_lidar:
            for v in env.vehicles:
                if not v.reached_goal:
                    self._draw_lidar(v, env)

        for v in env.vehicles:
            self._draw_vehicle(v)

        self._draw_hud(env, mode, metrics, extra=extra)
        pygame.display.flip()
        self.clock.tick(self.target_fps)

        if self.record_gif:
            self._capture_frame()

    def _capture_frame(self) -> None:
        self._frame_i += 1
        if self._frame_i % self.gif_every_n != 0:
            return
        try:
            from PIL import Image
        except ImportError:
            return
        raw = pygame.image.tostring(self.screen, "RGB")
        w, h = self.screen.get_size()
        img = Image.frombytes("RGB", (w, h), raw)
        if self.gif_scale != 1.0:
            nw = max(1, int(w * self.gif_scale))
            nh = max(1, int(h * self.gif_scale))
            img = img.resize((nw, nh), Image.BILINEAR)
        self._frames.append(img)

    def save_gif(self, path: Optional[str] = None) -> Optional[str]:
        path = path or self.gif_path
        if not path or not self._frames:
            return None
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        try:
            from PIL import Image
        except ImportError:
            print("Pillow not installed; cannot save GIF.")
            return None
        duration = int(1000 * self.gif_every_n / max(self.target_fps, 1))
        self._frames[0].save(
            path,
            save_all=True,
            append_images=self._frames[1:],
            duration=duration,
            loop=0,
            optimize=True,
        )
        print(f"GIF saved: {path} ({len(self._frames)} frames)")
        return path

    def close(self) -> None:
        pygame.quit()
