# AV0 Report

## Work completed

- inspected the full tracked source tree, example configurations, motor data, technical PDF, and supplied `.ork` project;
- mapped entry points, frames, equations, aerodynamics, propulsion, atmosphere, mass properties, actuator, sensors/noise, UI, plotting, configuration, dependencies, and manual tests;
- inspected the Dhruva Python SITL adapter and identified the reusable interface surface;
- captured three deterministic representative simulations with copied inputs and hashes;
- captured aerodynamic coefficient/CP snapshots for each case;
- confirmed byte-identical traces and aero snapshots on repeat execution;
- documented numerical/software risks, target architecture, validation strategy, and AV1-AV10 gates.

## Files changed

Only AV0 evidence and documentation were added:

- `docs/aerovector_optimization/AV0_REPOSITORY_AUDIT.md`
- `docs/aerovector_optimization/BASELINE_BEHAVIOR.md`
- `docs/aerovector_optimization/NUMERICAL_RISKS.md`
- `docs/aerovector_optimization/ARCHITECTURE_PLAN.md`
- `docs/aerovector_optimization/VALIDATION_PLAN.md`
- `docs/aerovector_optimization/IMPLEMENTATION_ROADMAP.md`
- `docs/aerovector_optimization/phases/AV0_REPORT.md`
- `tests/baselines/README.md`
- `tests/baselines/capture_av0_baselines.py`
- generated fixtures under `tests/baselines/{legacy_tvc,legacy_active_fins,dhruva_dual_motor_sitl}/`
- `tests/baselines/manifest.json`

No existing physics, solver, UI, configuration, SITL, or example source file was modified by AV0.

## Model/equation changes

None.

The capture harness replaces Tkinter widget reads with headless value stubs but calls the existing configuration, physics, controller, servo, and simulation loops. This is test plumbing, not a model change.

## Assumptions

- the current dirty working tree is the intended baseline state;
- Python 3.11.9 and the existing `.venv` are representative of the current development environment;
- a fixed seed is appropriate for deterministic AV0 evidence even though the normal application does not expose one;
- the legacy playback sampling cadence is sufficient for a first regression trace, while future solver tests must capture every requested canonical output sample;
- the technical PDF is the primary statement of the original model's intended equations and validity.

## References inspected

- `AeroVECTOR-Technical-Documentation.pdf`, all 37 pages, especially equations of motion, simulation procedure, body/fin aerodynamics, actuator, wind, PID, and sensor-noise sections;
- source modules listed in the repository audit;
- `README.md`;
- `Gaintuningork.ork` XML and saved simulation data;
- Dhruva Python SITL adapter and associated example configurations.

## Tests and checks run

- Python compile check: passed for `AeroVECTOR.py`, `src`, and the AV0 harness;
- automated test discovery: unavailable because `pytest` is not installed;
- legacy TVC baseline: completed;
- legacy active-fin baseline: completed;
- Dhruva dual-motor Python SITL baseline: completed;
- deterministic repeat: all three `trace.csv` and `aerodynamics.json` files were byte-identical;
- input config and motor copy hashes: matched their sources.

## Numerical results

| Case | Step | Runtime class | Apogee event | Final state summary |
|---|---:|---|---|---|
| Legacy TVC | 0.002 s | about 3 s on audit host | 117.802919 m at 5.818 s | ground crossing at 15.31 s, -18.155800 m/s vertical |
| Legacy active fins | 0.005 s | about 3 s | 31.540968 m at 4.88 s, later contradicted by re-ascent | duration 10 s at 61.558814 m, +17.648200 m/s vertical |
| Dhruva dual motor SITL | 0.002 s | about 2 s | 70.007322 m at 3.568 s; motor 2 ignited same time | ground crossing at 8.30 s, -24.291870 m/s vertical |

Runtime is observational and varies by host/load; it is not a golden equality value.

## Regressions

No existing code was changed, so no code regression was introduced. The audit exposed a baseline event-semantic defect: the active-fin apogee detector can fire before a later higher ascent.

## Performance

No profiling or optimization was performed. Baseline wall times are recorded only to guide later profiling. The largest obvious structural costs are per-step Python/global orchestration, repeated interpolation/object work, always-populated playback arrays, and coupling of plotting/SITL collection to the simulation loop; these must be measured in AV10 before optimization.

## Limitations

- no authoritative Dhruva S1/Run Archive schema exists in this checkout;
- no source automated tests or dependency lock exist;
- AV0 did not run RocketPy, MATLAB, or OpenRocket executables;
- external saved OpenRocket results were inspected but not treated as equivalent validation cases;
- the serial hardware SITL path was not run because AV0 excludes hardware interaction;
- normal application runs remain nondeterministic unless the RNG is externally controlled;
- baseline plots cover altitude and vertical velocity; full standardized reporting belongs to later stages.

## Recommended next stage

Proceed to AV1 only after review of this report and explicit approval. AV1 should implement canonical typed configuration, units, provenance, unknown-value handling, deterministic serialization/hashing, and a loss-aware legacy adapter. It should make no physics or solver changes.

## AV0 status

Complete. Stop before AV1.
