# STC Multi-Agent

**Decentralized Multi-Vehicle Coordination using Spatio-Temporal Corridors (2D)**

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

A clean, modular research prototype that coordinates **4–8 ground vehicles** in a shared 2D workspace by allocating **non-overlapping Spatio-Temporal Corridors (STCs)**—rectangular regions in *(x, y, t)*. Each agent plans and executes its trajectory **only inside its assigned corridor**, enabling simple decentralized conflict resolution without a central trajectory optimizer.

The project also ships a **single-vehicle baseline** (ray-casting LiDAR, reactive obstacle avoidance, goal-reaching navigation) that documents the master’s-level foundation this multi-agent layer builds on.

---

## Features

| Capability | Description |
|---|---|
| Single-vehicle baseline | Unicycle dynamics, 360° ray-cast LiDAR, potential-field navigation |
| Spatio-Temporal Corridors | Axis-aligned boxes in space × time, exclusive per vehicle |
| Decentralized coordination | Agents broadcast goals + corridor requests; greedy priority repair |
| Conflict resolution | Lateral shift → width shrink → time delay |
| Multi-agent scale | Configurable **4 to 8** vehicles |
| Real-time visualization | Pygame top-down view (vehicles, corridors, trajectories, HUD) |
| GIF export | Optional animation capture via Pillow |
| Metrics | Success rate, avg travel time, conflicts resolved, collisions |
| Batch experiments | `experiments/run_scenarios.py` + Matplotlib comparison plots |

---

## Architecture

```
                    ┌─────────────────────────────────────────┐
                    │              src/main.py                │
                    │     CLI  (baseline | stc modes)         │
                    └───────────────┬─────────────────────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              ▼                     ▼                     ▼
   ┌──────────────────┐  ┌────────────────────┐  ┌──────────────────┐
   │   simulation/    │  │   coordination/    │  │    corridor/     │
   │ environment.py   │  │ decentralized.py   │  │  corridor.py     │
   │ visualizer.py    │  │  shared message    │  │  stc_generator   │
   └────────┬─────────┘  │  bus + replan      │  └────────┬─────────┘
            │            └─────────┬──────────┘           │
            │                      │                      │
            │                      ▼                      │
            │            ┌────────────────────┐           │
            └───────────►│     vehicles/      │◄──────────┘
                         │  vehicle.py        │
                         │  controller.py     │
                         └────────────────────┘
```

**Data flow (STC mode)**

1. **Environment** spawns N vehicles on a ring with opposite goals (crossing traffic) and static AABB obstacles.
2. Each vehicle publishes a **CorridorRequest** `(start, goal, t_window, priority)` on the shared bus.
3. **STCGenerator** sorts by priority, builds tube corridors around start→goal, and repairs overlaps (shift / shrink / delay).
4. **DecentralizedCoordinator** assigns corridor bounds to each vehicle and periodically replans.
5. **VehicleController** tracks a corridor-clamped waypoint with LiDAR repulsion (and soft corridor-wall forces).
6. **Visualizer** renders corridors (semi-transparent), vehicles (oriented rectangles + ID), trajectories, and HUD metrics; optionally dumps a GIF.

---

## Project structure

```
stc-multi-agent/
├── README.md
├── requirements.txt
├── assets/                 # static media (optional screenshots)
├── src/
│   ├── vehicles/
│   │   ├── vehicle.py      # unicycle model + ray-cast LiDAR
│   │   └── controller.py   # potential-field / corridor-aware control
│   ├── corridor/
│   │   ├── corridor.py     # STC + CorridorRequest dataclasses
│   │   └── stc_generator.py
│   ├── coordination/
│   │   └── decentralized.py
│   ├── simulation/
│   │   ├── environment.py  # world, obstacles, metrics, scenarios
│   │   └── visualizer.py   # Pygame + GIF
│   └── main.py             # CLI entry: python -m src.main
├── experiments/
│   └── run_scenarios.py    # batch 4/6/8 + baseline comparison
└── results/                # metrics JSON, GIFs, plots
```

---

## Installation

```bash
# clone / enter project
cd stc-multi-agent

# recommended: virtual environment
python -m venv .venv

# Windows
.venv\Scripts\activate
# Linux / macOS
# source .venv/bin/activate

pip install -r requirements.txt
```

**Requirements:** Python **3.10+**, NumPy, Matplotlib, Pygame, Pillow (GIF only).

---

## Usage

### Multi-agent STC (default)

```bash
python -m src.main
python -m src.main --mode stc --agents 6
python -m src.main --mode stc --agents 8 --gif
```

### Single-vehicle baseline

```bash
python -m src.main --mode baseline
python -m src.main --mode baseline --show-lidar --gif
```

### Headless (CI / batch)

```bash
python -m src.main --mode stc --agents 6 --headless
python experiments/run_scenarios.py
python experiments/run_scenarios.py --gif
```

### CLI flags

| Flag | Default | Description |
|------|---------|-------------|
| `--mode` | `stc` | `stc` or `baseline` |
| `--agents` | `6` | Vehicles in STC mode (clamped to 4–8) |
| `--max-time` | `90` | Simulation horizon (seconds) |
| `--dt` | `0.1` | Integration step |
| `--seed` | `42` | Scenario RNG seed |
| `--headless` | off | No window |
| `--gif` | off | Record GIF under `results/` |
| `--show-lidar` | off | Draw LiDAR rays |
| `--window` | `800` | Viewport size (px) |

### Keyboard (interactive window)

| Key | Action |
|-----|--------|
| `SPACE` | Pause / resume |
| `L` | Toggle LiDAR overlay |
| `G` | Start GIF capture |
| `ESC` | Quit |

---

## Method summary

### Spatio-Temporal Corridor

An STC is an axis-aligned prism:

```
C_i = [x_min, x_max] × [y_min, y_max] × [t_start, t_end]
```

Two corridors **conflict** iff they overlap in **both** space and time and belong to different vehicles. The generator guarantees a conflict-free set by greedy priority allocation:

1. **Tube construction** — inflate the start–goal AABB by half preferred width; set duration from nominal speed + padding.
2. **Shift** — push the lower-priority corridor away from the conflict center.
3. **Shrink** — reduce width/height toward center (floor at `min_width`).
4. **Time delay** — open the corridor only after the blocker frees the space.

### Decentralized protocol (simulated)

- Agents broadcast `SharedMessage`: pose, goal, priority, corridor intent.
- Coordinator (stand-in for local broadcast + consensus) regenerates STCs every `replan_interval` seconds for agents still en route.
- Motion is **corridor-constrained**: control waypoints are clamped to bounds; soft wall repulsion keeps the body inside; agents **wait** if `t < t_start`.

### Single-vehicle baseline

Classic sensing → avoidance → goal stack:

- Ray-cast LiDAR vs AABB obstacles (and peer discs in multi-agent mode).
- Attractive force to goal + inverse-distance repulsive force from short returns.
- Unicycle tracking of the resultant desired heading with speed scaling near goals/obstacles.

---

## Results

After a run, metrics are printed and written to `results/metrics_*.json`:

```json
{
  "mode": "stc",
  "n_vehicles": 6,
  "success_count": 6,
  "success_rate": 1.0,
  "average_travel_time": 28.4,
  "conflicts_resolved": 11,
  "collisions": 0,
  "sim_time": 31.2
}
```

Batch comparison (`python experiments/run_scenarios.py`) also writes:

- `results/experiment_summary.json`
- `results/comparison.png` — success rate, travel time, collisions, conflicts

**Typical observations (seed-dependent):**

- Baseline reliably reaches the far corner while skirting box obstacles via LiDAR.
- STC mode resolves crossing conflicts primarily by spatial separation; residual overlaps are time-sliced.
- Success rate stays high for 4–6 agents; 8 agents may need longer horizons or narrower corridors as density grows.
- Collisions remain near zero when agents respect corridor bounds and wait windows.

> Re-run experiments on your machine to populate `results/` with GIFs and plots for reports.

---

## Performance metrics

| Metric | Definition |
|--------|------------|
| **Success rate** | Fraction of vehicles that reach their goal within `max_time` |
| **Average travel time** | Mean time-to-goal over successful agents |
| **Conflicts resolved** | Number of STC repair operations (shift/shrink/delay) |
| **Collisions** | Pairwise vehicle contacts (+ obstacle penetrations) counted during the run |

---

## Future work

- **3D / SE(2) corridors** with orientation-aware footprints and non-rectangular shapes
- **Optimization-based STC** (MIP / QP) instead of greedy repair for tighter packs
- **Communication models** — range limits, dropouts, delay, consensus protocols
- **Dynamic obstacles** and moving pedestrians inside the same STC framework
- **Learning-based priority / width** adaptation (RL or IL on top of the corridor layer)
- **ROS 2 bridge** for deployment on physical differential-drive robots
- **Formal safety** — barrier certificates or reachable-set validation inside corridors

---

## Citation

If you use this codebase in academic work, please cite it as a software prototype for decentralized multi-vehicle STC coordination (2D).

---

## License

MIT — free to use, modify, and distribute with attribution.
#   s t c - m u l t i - v e h i c l e - c o o r d i n a t i o n  
 