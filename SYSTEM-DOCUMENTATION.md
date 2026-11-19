# SYSTEM-DOCUMENTATION — stc-multi-vehicle-coordination

Deep technical / developer documentation. For running instructions see
`USER-GUIDE.md`; for the work ledger see `PENDING-TASKS.md`.

## 1. Purpose and scope

Research prototype using greedy priority-based Spatio-Temporal Corridor allocation with a Shift → Shrink → Delay repair cascade.

Research question: **Can 4–8 ground vehicles cross a shared workspace safely with no central path planner?**

This is a simulation study. No hardware measurements, external datasets or
proprietary data are involved. The repository is intentionally small enough to
be read end-to-end by a reviewer in one sitting.

## 2. Architecture

| Component | Responsibility |
|---|---|
| `src/vehicles/` | Unicycle dynamics, ray-cast LiDAR, footprint, potential-field + corridor-aware controller. |
| `src/corridor/` | SpatioTemporalCorridor, CorridorRequest and the greedy allocation/repair logic. |
| `src/coordination/` | Message bus, SharedMessage, periodic replanning and per-tick stepping. |
| `src/simulation/` | World, obstacles, collision metrics, scenario builders and Pygame renderer. |
| `experiments/run_scenarios.py` | Baseline and 4/6/8-agent batch comparison. |

Dependencies flow one way: configuration/scenario → model/algorithm →
experiment runner → results → figures/animation. The simulator does not read
its own results; results are an output only.

## 3. Data flow

1. A seeded scenario, task set, signal or topology is constructed.
2. The core model advances in discrete steps (ticks or control steps).
3. Metrics are accumulated independently of the decision logic where possible
   (for example, a ground-truth collision checker rather than the agents' beliefs).
4. `experiments/` writes machine-readable results under `results/`.
5. `make_figures.py` and `make_animation.py` read those results or re-run the
   seeded simulator to produce visual artefacts.

## 4. Algorithms / numerical methods

See `README.md` for the high-level description and, where present,
`docs/DESIGN.md` or `DEVELOPMENT_GUIDE.md` for the full derivation. The
implementation is the authoritative reference: start at the module table in
section 2 and follow the entry point used by `python -m src.main --mode stc --agents 6 --headless`.

Reproducibility rules:
- Every stochastic component is driven by an explicit seed.
- Experiment parameters are recorded in the result rows.
- A change to a result number requires regenerating the CSV/JSON, not editing prose.

## 5. Results and evidence

- Success rate 100% and zero collisions in every reported configuration — `results/experiment_summary.json`.
- Conflicts resolved grow with fleet density: 0 (4 agents), 11 (6 agents), 21 (8 agents) — `results/experiment_summary.json`.
- Average travel time: baseline 28.0 s, 4 agents 17.2 s, 6 agents 22.9 s, 8 agents 24.9 s — `results/experiment_summary.json`.

## 6. Animation pipeline

`python -m src.main --mode stc --agents 8 --gif` renders `results/stc_8agents.gif`. It uses the project's own simulator
(or an explicitly documented pure-Python reference of the same controller) and
Matplotlib's Pillow writer. Frames are subsampled to keep the GIF small; the
underlying trajectory is not resampled or smoothed. Eight vehicles negotiating perpendicular lanes; translucent rectangles are their reserved corridors.

## 7. Build, test and CI

```bash
pip install -r requirements.txt
python -m src.main --mode stc --agents 6 --headless
python experiments/run_scenarios.py
python -m src.main --mode stc --agents 8 --gif
```

CI (`.github/workflows/ci.yml`) runs on push and pull request. Jobs install the
documented dependencies, set `PYTHONPATH=src` where required, and run the test
suite. Repositories that call C code install a system compiler in CI; footprint
measurement with cross-compilers is deliberately optional.

## 8. Extension points

- Add a new scenario/policy by extending the corresponding factory or registry
  and a seeded test, then re-run the full experiment suite.
- Add a new metric by writing it into the result row and updating
  `experiments/summarize.py` and the README table together.
- Add a new animation by recording the relevant simulator state (prefer an
  optional `record` flag over duplicating the stepping logic) and saving via
  `FuncAnimation`/`PillowWriter` in `experiments/`.

## 9. Known limitations

- Axis-aligned rectangles only; diagonal/curved paths inflate wastefully.
- One bounding tube per vehicle per replan; no piecewise corridor sequence.
- Decentralization is simulated: the message bus lives in one coordinator object in one process.
- Static obstacles only; no pedestrians or dynamic obstacles.
- Greedy allocation is heuristic and seed-dependent, not optimal; no formal safety proof.

## 10. Relation to the rest of the portfolio

Earlier, ground-vehicle-only ancestor of `stc-corridors`; formal verification is explored in `stc-formal`.

## 11. Release metadata

`CITATION.cff` carries title, author (Yusuf Ndanusa Yahaya), version 0.1.0,
release date, repository URL and MIT license. No DOI or ORCID is claimed.
