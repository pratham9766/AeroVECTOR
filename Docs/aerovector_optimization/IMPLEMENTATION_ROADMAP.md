# AeroVECTOR Optimization Roadmap

Every stage uses the AV names below. No stage begins automatically.

## AV0 - Repository audit and baseline capture

Completed in this execution.

Deliverables: architecture/model audit, deterministic baseline fixtures, numerical/software risk register, architecture plan, validation plan, and AV1-AV10 roadmap.

Gate: three representative cases captured with exact inputs and hashes; deterministic replay confirmed; no physics or solver refactor.

## AV1 - Canonical vehicle/configuration model

Scope:

- typed canonical models for vehicle, mass, aero, propulsion, environment, initial conditions, and simulation;
- SI core units with explicit source units;
- uncertainty, source, provenance, notes, and unknown-value representation;
- deterministic serialization and configuration hash;
- loss-aware adapter from the legacy text format;
- schema/version and validation errors.

Do not alter physics or integration.

Gate:

- legacy examples round-trip through the adapter without unexplained information loss;
- unknown/invalid fields never become silent defaults;
- identical canonical inputs hash identically;
- AV0 trajectories and aero snapshots remain within frozen tolerances.

## AV2 - OpenRocket `.ork` importer

Scope:

- safe ZIP/XML reader;
- component hierarchy and source-path preview;
- geometry, mass/material, motor, recovery, launch/environment, and saved-simulation extraction;
- unit normalization, conflicts, unsupported-field retention, importer provenance;
- explicit user-confirmation boundary before canonical mapping.

Gate:

- malicious/pathological archive tests pass;
- `Gaintuningork.ork` preview is complete and reproducible;
- every mapped value retains original XML path/value/unit;
- no unavailable value is inferred.

## AV3 - NumPy scientific-core refactor

Scope:

- explicit NumPy state vectors, forces, moments, and transforms;
- pure or controlled-state physics functions;
- typed shape/unit/frame documentation;
- separation from GUI, plots, VPython, serial, and dynamic module loading;
- exact legacy adapter retained.

Gate:

- AV0 regression passes;
- coefficient grids match or every difference has an approved change record;
- two simulations run independently in one process;
- import has no GUI/browser side effects.

## AV4 - Solver architecture and SciPy integration

Scope:

- common solver/result/event interface;
- preserved legacy trapezoid;
- fixed-step RK4;
- SciPy RK45, DOP853, and LSODA;
- structured status, warnings, evaluation counts, tolerance/step metadata;
- explicit scheduling for controller, actuator, and sensors.

Gate:

- analytical cases pass with predeclared tolerances;
- solver metadata is complete;
- legacy solver still reproduces AV0;
- event ordering is deterministic.

## AV5 - Numerical verification and regression suite

Scope:

- unit, analytical, convergence, regression, and event tests;
- timestep sweeps and solver comparisons;
- coefficient/propulsion/atmosphere fixtures;
- CI and dependency lock;
- formal numerical reports.

Gate:

- fundamental analytical suite passes;
- convergence is demonstrated or limitations are explicit;
- false-apogee/re-ascent cases are covered;
- no unexplained baseline delta.

## AV6 - RocketPy comparison integration

Scope:

- isolated canonical-config-to-RocketPy adapter;
- compatible result mapping;
- atmosphere, propulsion, mass, trajectory, event, and dispersion comparisons;
- standardized metrics and assumption-difference reports.

Gate:

- RocketPy is optional;
- comparisons reject incompatible frames/definitions;
- provider version and exact inputs are recorded;
- discrepancies are quantified, not tuned away.

## AV7 - MATLAB/OpenRocket/AeroVECTOR validation workflows

Scope:

- versioned JSON/CSV and optional MAT interchange;
- MATLAB reference import;
- OpenRocket saved-result extraction/alignment;
- common residual and report framework;
- external-tool fixture management.

Gate:

- all exchanges round-trip metadata;
- compatible quantities and common intervals are explicit;
- reports contain metrics, residuals, assumptions, and limitations;
- AeroVECTOR has no runtime MATLAB dependency.

## AV8 - Monte Carlo and sensitivity analysis

Scope:

- evidence-backed uncertainty models;
- seeded sampling and replay;
- percentiles, histograms, scatter plots, and sensitivity method;
- convergence/adequacy checks for sampling;
- measurement-priority recommendations.

Gate:

- deterministic AV5 suite is stable;
- every varied parameter has nominal, distribution/uncertainty, unit, and source;
- seeds and sample matrices are archived;
- uncertainty results do not claim model validation.

## AV9 - Dhruva platform integration

Scope:

- acquire and version the authoritative Dhruva S1 schema;
- map canonical AeroVECTOR results to provider-neutral results;
- connect provenance, jobs, evidence, reports, and Run Archive;
- supply S2 2-D and S3 3-D playback channels where representable;
- retain the current Dhruva Python SITL adapter as a separate test adapter.

Gate:

- schema contract tests pass against real Dhruva examples;
- no second major UI is built;
- unsupported 2-D-to-3-D fields are explicit, not fabricated;
- archive records reproduce the originating run.

## AV10 - Final regression, performance, and release

Scope:

- profile solver, interpolation, plotting, memory, and provider costs;
- optimize measured bottlenecks without obscuring equations;
- full cross-platform/solver/provider regression;
- release notes, migration guide, versioned schemas, and limitations;
- reproducible release artifacts.

Gate:

- AV1-AV9 gates are green or carry approved limitations;
- performance changes have before/after measurements;
- final deterministic and external validation reports are archived;
- release identifies supported model domains and non-goals.

## Cross-stage quality gate

No stage advances unless existing tests remain green, new tests pass, numerical warnings are resolved or documented, regressions are explained, and provenance survives all transformations. Performance optimization begins only after profiling. Monte Carlo begins only after deterministic validation.
