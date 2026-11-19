# USER-GUIDE — stc-multi-vehicle-coordination

End-user manual. You do not need to edit code to follow this guide.

## 1. What you can do with it

Research prototype using greedy priority-based Spatio-Temporal Corridor allocation with a Shift → Shrink → Delay repair cascade.

The research question is: *Can 4–8 ground vehicles cross a shared workspace safely with no central path planner?*

## 2. Install

```bash
git clone https://github.com/olamida/stc-multi-vehicle-coordination.git
cd stc-multi-vehicle-coordination
pip install -r requirements.txt
```

Python 3.10+ is assumed. On Windows use `.venv\Scripts\activate` instead of
`source .venv/bin/activate` if you create a virtual environment.

## 3. Watch the animation

Open `results/stc_8agents.gif` directly on GitHub or in any image viewer; GitHub plays
GIFs inline. The animation shows: Eight vehicles negotiating perpendicular lanes; translucent rectangles are their reserved corridors.

To regenerate it from source:

```bash
python -m src.main --mode stc --agents 8 --gif
```

## 4. Verify the installation

```bash
python -m src.main --mode stc --agents 6 --headless
```

A passing run prints a test summary with no failures. If a C compiler is
mentioned as missing, install a system C compiler (gcc on Linux, Xcode command
line tools on macOS, MinGW/MSVC on Windows) and rerun.

## 5. Run the full experiments

```bash
python experiments/run_scenarios.py
```

Outputs appear in `results/` (CSV/JSON/Markdown tables) and `figures/`
(PNG/GIF). Re-running with the shipped seeds should reproduce the shipped
numbers; small differences can occur with different library versions.

## 6. How to read the outputs

- Success rate 100% and zero collisions in every reported configuration — `results/experiment_summary.json`.
- Conflicts resolved grow with fleet density: 0 (4 agents), 11 (6 agents), 21 (8 agents) — `results/experiment_summary.json`.
- Average travel time: baseline 28.0 s, 4 agents 17.2 s, 6 agents 22.9 s, 8 agents 24.9 s — `results/experiment_summary.json`.

## 7. Known limitations (please read)

- Axis-aligned rectangles only; diagonal/curved paths inflate wastefully.
- One bounding tube per vehicle per replan; no piecewise corridor sequence.
- Decentralization is simulated: the message bus lives in one coordinator object in one process.
- Static obstacles only; no pedestrians or dynamic obstacles.
- Greedy allocation is heuristic and seed-dependent, not optimal; no formal safety proof.

## 8. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| `ModuleNotFoundError` | Running from the wrong directory or missing `PYTHONPATH=src` | Run from the repository root and use the exact command in section 4. |
| Build fails with "gcc not found" | No system C compiler | Install gcc/Xcode CLT/MinGW; the pure-Python reference is only for the animation. |
| Animation script cannot find sibling repo | `stc-corridors` not beside `stc-formal` | Clone both repositories side by side, or install `stc` into the environment. |
| Numbers differ from README | Different seeds or library versions | Use the command in section 5 and compare against `results/` in the repo. |

## 9. Getting help

Open an issue with the exact command, your OS/Python version and the last 20
lines of output. Please do not attach private or employer data.
