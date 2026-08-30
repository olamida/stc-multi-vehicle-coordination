"""
Batch experiment runner for STC multi-agent scenarios.

Runs baseline + multi-agent configs (4/6/8 vehicles), writes JSON summaries
and optional comparison plots under results/.
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any, Dict, List

import numpy as np

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from src.main import run_baseline, run_stc, parse_args, save_metrics


def _ns(**kwargs):
    """Build an argparse-like namespace from defaults + overrides."""
    args = parse_args([])
    for k, v in kwargs.items():
        setattr(args, k, v)
    args.results_dir = args.results_dir or os.path.join(_ROOT, "results")
    os.makedirs(args.results_dir, exist_ok=True)
    return args


def run_all(headless: bool = True, gif: bool = False, max_time: float = 90.0) -> List[Dict[str, Any]]:
    results: List[Dict[str, Any]] = []

    print("\n>>> Baseline (single vehicle)")
    m = run_baseline(_ns(mode="baseline", headless=headless, gif=gif, max_time=max_time,
                         show_lidar=False))
    save_metrics(m, os.path.join(_ROOT, "results"), "baseline")
    results.append(m.as_dict())
    print(m)

    for n in (4, 6, 8):
        print(f"\n>>> STC multi-agent  n={n}")
        m = run_stc(_ns(mode="stc", agents=n, headless=headless, gif=gif,
                        max_time=max_time, seed=42 + n))
        save_metrics(m, os.path.join(_ROOT, "results"), f"stc_{n}agents")
        results.append(m.as_dict())
        print(m)

    summary_path = os.path.join(_ROOT, "results", "experiment_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\nSummary written to {summary_path}")

    _plot_comparison(results)
    return results


def _plot_comparison(results: List[Dict[str, Any]]) -> None:
    """Bar charts for success rate and travel time using Matplotlib."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping plots")
        return

    labels = []
    success = []
    travel = []
    collisions = []
    conflicts = []
    for r in results:
        if r["mode"] == "baseline":
            labels.append("baseline")
        else:
            labels.append(f"stc-{r['n_vehicles']}")
        success.append(r["success_rate"] * 100.0)
        t = r["average_travel_time"]
        travel.append(t if t is not None else 0.0)
        collisions.append(r["collisions"])
        conflicts.append(r["conflicts_resolved"])

    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    fig.suptitle("STC Multi-Agent Experiment Comparison", fontsize=13, fontweight="bold")
    colors = plt.cm.viridis(np.linspace(0.25, 0.85, len(labels)))

    axes[0, 0].bar(labels, success, color=colors)
    axes[0, 0].set_ylabel("Success rate (%)")
    axes[0, 0].set_ylim(0, 105)
    axes[0, 0].set_title("Goal reach success")

    axes[0, 1].bar(labels, travel, color=colors)
    axes[0, 1].set_ylabel("Avg travel time (s)")
    axes[0, 1].set_title("Average travel time")

    axes[1, 0].bar(labels, collisions, color=colors)
    axes[1, 0].set_ylabel("Collisions")
    axes[1, 0].set_title("Collision count")

    axes[1, 1].bar(labels, conflicts, color=colors)
    axes[1, 1].set_ylabel("Conflicts resolved")
    axes[1, 1].set_title("STC conflict resolutions")

    for ax in axes.ravel():
        ax.tick_params(axis="x", rotation=20)
        ax.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    out = os.path.join(_ROOT, "results", "comparison.png")
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"Comparison plot saved: {out}")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Run STC batch experiments")
    ap.add_argument("--show", action="store_true", help="Show Pygame windows")
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--max-time", type=float, default=90.0)
    cli = ap.parse_args()
    run_all(headless=not cli.show, gif=cli.gif, max_time=cli.max_time)
