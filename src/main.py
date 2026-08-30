"""
STC Multi-Agent entry point.

Usage
-----
    python -m src.main
    python -m src.main --mode stc --agents 6
    python -m src.main --mode baseline --gif
    python -m src.main --mode stc --agents 8 --headless --gif
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Optional

# Ensure project root is on sys.path when run as python -m src.main
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from src.coordination.decentralized import DecentralizedCoordinator
from src.simulation.environment import (
    Environment,
    SimulationMetrics,
    build_multi_vehicle_scenario,
    build_single_vehicle_scenario,
)
from src.vehicles.controller import VehicleController


def parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Decentralized Multi-Vehicle Coordination with Spatio-Temporal Corridors",
    )
    p.add_argument(
        "--mode", choices=("stc", "baseline"), default="stc",
        help="stc = multi-agent STC coordination; baseline = single-vehicle LiDAR nav",
    )
    p.add_argument("--agents", type=int, default=6, help="Number of vehicles (4-8) for stc mode")
    p.add_argument("--world-size", type=float, default=60.0)
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--max-time", type=float, default=90.0, help="Simulation time limit (s)")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--headless", action="store_true", help="No Pygame window (faster batch)")
    p.add_argument("--gif", action="store_true", help="Record animation GIF to results/")
    p.add_argument("--gif-path", type=str, default=None)
    p.add_argument("--show-lidar", action="store_true")
    p.add_argument("--window", type=int, default=800)
    p.add_argument("--results-dir", type=str, default=None)
    return p.parse_args(argv)


def run_baseline(args: argparse.Namespace) -> SimulationMetrics:
    """Single-vehicle foundation: LiDAR sensing + potential-field navigation."""
    env = build_single_vehicle_scenario(world_size=args.world_size)
    env.dt = args.dt
    controller = VehicleController()
    metrics = SimulationMetrics(mode="baseline")

    viz = None
    if not args.headless or args.gif:
        from src.simulation.visualizer import Visualizer
        gif_path = args.gif_path or os.path.join(
            args.results_dir or _default_results(), "baseline.gif"
        )
        viz = Visualizer(
            world_size=args.world_size,
            window_size=args.window,
            caption="STC Multi-Agent | Baseline (Single Vehicle)",
            show_lidar=args.show_lidar or True,
            record_gif=args.gif,
            gif_path=gif_path,
        )

    while env.time < args.max_time:
        if viz and not viz.running:
            break
        if viz and viz.paused:
            viz.render(env, corridors=None, mode="baseline", metrics=metrics)
            continue

        v = env.vehicles[0]
        if not v.reached_goal:
            scan = v.sense_lidar(env.obstacles)
            a, omega = controller.compute_control(v, scan)
            v.step(a, omega, env.dt)
        env.check_collisions()
        env.clamp_vehicles_to_world()
        env.time += env.dt

        if viz:
            metrics.collisions = sum(v.collision_count for v in env.vehicles)
            viz.render(env, corridors=None, mode="baseline", metrics=metrics,
                       extra="Single-vehicle LiDAR baseline")

        if env.all_done():
            break

    metrics.finalize(env.vehicles, conflicts=0, sim_time=env.time)
    metrics.mode = "baseline"
    if viz:
        if args.gif:
            viz.save_gif()
        viz.close()
    return metrics


def run_stc(args: argparse.Namespace) -> SimulationMetrics:
    """Multi-vehicle decentralized STC coordination."""
    n = int(max(4, min(8, args.agents)))
    env = build_multi_vehicle_scenario(
        n_vehicles=n, world_size=args.world_size, seed=args.seed,
    )
    env.dt = args.dt
    coord = DecentralizedCoordinator(world_size=args.world_size)
    coord.assign_initial_corridors(env.vehicles, t=0.0)

    metrics = SimulationMetrics(mode="stc")
    viz = None
    if not args.headless or args.gif:
        from src.simulation.visualizer import Visualizer
        gif_path = args.gif_path or os.path.join(
            args.results_dir or _default_results(), f"stc_{n}agents.gif"
        )
        viz = Visualizer(
            world_size=args.world_size,
            window_size=args.window,
            caption=f"STC Multi-Agent | {n} Vehicles",
            show_lidar=args.show_lidar,
            record_gif=args.gif,
            gif_path=gif_path,
        )

    while env.time < args.max_time:
        if viz and not viz.running:
            break
        if viz and viz.paused:
            metrics.conflicts_resolved = coord.total_conflicts_resolved
            viz.render(env, corridors=coord.corridors, mode="stc", metrics=metrics)
            continue

        coord.step_all(env.vehicles, env.obstacles, env.time, env.dt)
        env.check_collisions()
        env.clamp_vehicles_to_world()
        env.time += env.dt

        metrics.conflicts_resolved = coord.total_conflicts_resolved
        metrics.collisions = int(sum(v.collision_count for v in env.vehicles) // 2)

        if viz:
            viz.render(env, corridors=coord.corridors, mode="stc", metrics=metrics,
                       extra=f"agents={n}")

        if env.all_done():
            break

    metrics.finalize(env.vehicles, conflicts=coord.total_conflicts_resolved, sim_time=env.time)
    metrics.mode = "stc"
    if viz:
        if args.gif:
            viz.save_gif()
        viz.close()
    return metrics


def _default_results() -> str:
    path = os.path.join(_ROOT, "results")
    os.makedirs(path, exist_ok=True)
    return path


def save_metrics(metrics: SimulationMetrics, results_dir: str, tag: str) -> str:
    os.makedirs(results_dir, exist_ok=True)
    path = os.path.join(results_dir, f"metrics_{tag}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(metrics.as_dict(), f, indent=2)
    print(f"Metrics saved: {path}")
    return path


def main(argv: Optional[list] = None) -> int:
    args = parse_args(argv)
    results_dir = args.results_dir or _default_results()
    args.results_dir = results_dir

    print("=" * 60)
    print("  STC Multi-Agent  |  Decentralized Corridor Coordination")
    print("=" * 60)
    print(f"  mode={args.mode}  agents={args.agents if args.mode == 'stc' else 1}")
    print(f"  max_time={args.max_time}s  dt={args.dt}  headless={args.headless}")
    print("=" * 60)

    if args.mode == "baseline":
        metrics = run_baseline(args)
        tag = "baseline"
    else:
        metrics = run_stc(args)
        tag = f"stc_{max(4, min(8, args.agents))}agents"

    print()
    print(metrics)
    save_metrics(metrics, results_dir, tag)
    return 0 if metrics.success_rate >= 0.5 else 1


if __name__ == "__main__":
    raise SystemExit(main())
