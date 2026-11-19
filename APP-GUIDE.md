# APP-GUIDE — stc-multi-vehicle-coordination

**Question:** Can 4–8 ground vehicles cross a shared workspace safely with no central path planner?

**One-line summary:** Research prototype using greedy priority-based Spatio-Temporal Corridor allocation with a Shift → Shrink → Delay repair cascade.

This guide records what the application/project has already done and what is still
to be done. It is the session-to-session entry point: read this first, then
`USER-GUIDE.md` to run it, `SYSTEM-DOCUMENTATION.md` to change it, and
`PENDING-TASKS.md` for the live work ledger.

## Work already executed

- Implemented a single-vehicle LiDAR baseline and a multi-agent STC layer on top of it.
- Implemented the Greedy Priority-based STC Allocation algorithm: sort by priority, build tubes, then repair conflicts with an ordered Shift → Shrink → Delay cascade.
- Built a lightweight message bus with SharedMessage, periodic replanning for active agents and snapshot-based stepping.
- Added a Pygame visualizer with pause/LiDAR/GIF controls and headless mode for batch runs.
- Ran a baseline + 4/6/8-vehicle experiment sweep and wrote metrics JSON and comparison plots under `results/`.
- Published the repository on GitHub with MIT license and a development guide; added four demo GIFs.

## Work to be executed (next)

- Replace the simulated message bus with a real distributed consensus/network implementation including loss and latency.
- Replace axis-aligned rectangles and the single-tube plan with piecewise SE(2) corridors.
- Replace greedy allocation with an optimization-based (MIP/QP) allocator and compare.
- Add dynamic obstacles, pedestrians and runtime safety verification (barrier certificates / reachability).
- Bridge the controller to ROS 2 and physical differential-drive robots.

## High-level flow

1. Configure a seeded scenario / task set / signal.
2. Run the simulator or experiment script in this repository.
3. Metrics are written to `results/`; figures to `figures/`.
4. The animation in `results/stc_8agents.gif` is rendered from the same simulator state.

## Commands at a glance

| Purpose | Command |
|---|---|
| Install | `pip install -r requirements.txt` |
| Verify | `python -m src.main --mode stc --agents 6 --headless` |
| Experiments | `python experiments/run_scenarios.py` |
| Animation | `python -m src.main --mode stc --agents 8 --gif` |

## Where this connects

Earlier, ground-vehicle-only ancestor of `stc-corridors`; formal verification is explored in `stc-formal`.

## Honesty notes

All numbers quoted in this repository come from files under `results/`.
Simulation-only results are not hardware measurements, and the documented
limitations in `README.md` are part of the deliverable.
