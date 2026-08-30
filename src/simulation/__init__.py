"""Simulation environment and visualization."""

from .environment import Environment, Obstacle, SimulationMetrics

__all__ = ["Environment", "Obstacle", "SimulationMetrics"]


def __getattr__(name: str):
    """Lazily import the pygame-backed Visualizer so headless use only needs numpy."""
    if name == "Visualizer":
        from .visualizer import Visualizer  # deferred: requires pygame

        return Visualizer
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
