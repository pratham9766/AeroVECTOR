# AeroVECTOR Validation Plan

## Validation hierarchy

```text
analytical/manufactured reference
  -> optimized AeroVECTOR
  -> RocketPy compatible quantities
  -> MATLAB reference/interchange
  -> OpenRocket compatible quantities
  -> measured data when available
```

Agreement between tools is evidence, not proof. Every comparison must first establish compatible configuration, definitions, units, frames, initial conditions, sampling, and event semantics.

## Gate 0: provenance and reproducibility

Before a numerical comparison is accepted, retain:

- canonical configuration snapshot and hash;
- source file hashes and importer versions;
- code revision/model versions;
- solver and all settings;
- random seed and generator;
- provider name/version;
- warnings and termination status;
- coordinate-frame and unit declaration.

The AV0 fixture manifest is the initial regression dataset.

## Gate 1: unit tests

Required focused tests:

- body/global rotation matrices, inverses, signs, and round trips;
- ISA values at layer bases, boundaries, and selected references;
- constant atmosphere and imported profile interpolation;
- thrust validation, interpolation, bounds, impulse, and preservation of source samples;
- mass/CG/inertia endpoint and interpolation behavior;
- fin geometry, aspect ratio, aerodynamic-center, and interference utilities;
- actuator discretization, quantization, sample delay, and limits;
- configuration parsing, unknown values, unit conversion, hashes, and provenance;
- OpenRocket safe ZIP/XML parsing and unsupported-field retention;
- validation metrics and time alignment.

## Gate 2: analytical and manufactured dynamics

Build minimal cases with aerodynamics and discrete controls disabled as appropriate:

| Case | Reference |
|---|---|
| Constant acceleration | `x = x0 + v0*t + 0.5*a*t^2`, `v = v0 + a*t` |
| Free fall | constant gravity solution |
| Constant body/global force | Newtonian translation with known frame |
| Constant thrust, constant mass | closed-form one-axis motion |
| Zero-drag vertical coast | ballistic solution and apogee time |
| Constant angular acceleration | `theta` and `Q` quadratic/linear reference |
| Constant torque/inertia | angular Newtonian reference |
| Rotation-only transform | norm preservation and known quadrants |
| Table interpolation | exact knots, midpoints, and bounds policy |

Use explicit absolute and relative tolerances derived from solver order and step. A higher-level validation report must show failed if any fundamental test fails.

## Gate 3: legacy regression

For the three AV0 cases, compare:

- state histories at aligned times;
- aerodynamic snapshots;
- final state;
- primary/secondary burnout;
- apogee detection and altitude;
- stage-two ignition;
- parachute and termination state.

Do not update golden data automatically. Any accepted change requires old result, new result, reason, reference, and reviewer-visible tolerance impact.

## Gate 4: solver convergence

Run fixed-step sweeps at 0.02, 0.01, 0.005, and 0.0025 s, adding 0.00125 s where the asymptotic regime is unclear. For each case report:

- altitude, vertical speed, downrange, pitch, and pitch rate norms;
- apogee altitude/time and event-time error;
- final-state differences;
- maximum Mach/dynamic pressure where available;
- runtime and derivative evaluations.

Compare RK4, RK45, DOP853, and LSODA against a tight DOP853 or analytically known reference. Fixed-step results should demonstrate an observed order consistent with their implementation away from discontinuities. Event errors must be reported separately from state errors.

## Gate 5: aerodynamic regression and evidence

Preserve coefficient grids over compatible domains:

- angle of attack;
- Mach below the supported limit;
- Reynolds number;
- pitch rate;
- actuator angle;
- CG position where it affects moments.

Report `CN`, `CA`, `CM`, CP, body contributions, passive/control fin contributions, and drag breakdown. The technical documentation's acknowledged fin-drag uncertainty must appear in limitations. No aerodynamic refactor is accepted solely because it approaches another simulator.

## Gate 6: external providers

### OpenRocket

Use `Gaintuningork.ork` first as an importer and event/result fixture. Compare only mapped values:

- component geometry and source paths;
- mass/CG/inertia when definitions and motor state match;
- reference area and selected stability/aerodynamic quantities;
- altitude, speed, apogee, apogee time, and flight duration under matched conditions.

The file's saved Simulation 1 reports 24.501 m apogee, 2.626 s time to apogee, and 4.972 s flight time. These are source values to preserve, not automatic AeroVECTOR acceptance targets.

### RocketPy

Build an adapter from canonical configuration. Keep RocketPy internals isolated. Compare atmosphere, propulsion, mass variation, position, speed, vertical velocity, apogee, apogee time, and duration where both models support equivalent assumptions.

### MATLAB

Export versioned JSON/CSV configuration and results. Import MATLAB reference outputs through the same comparison schema. Use MATLAB for independent ODE solutions, sweeps, filtering, and residual analysis; never make runtime AeroVECTOR depend on MATLAB.

## Metrics

For aligned compatible channels compute:

- RMSE;
- MAE;
- signed bias;
- maximum absolute error;
- normalized error when the normalization is meaningful and stated;
- event-time difference;
- raw and time-aligned residual series.

Never hide interpolation error. Record the alignment method, extrapolation policy, common interval, and excluded samples.

## Engineering reports

Standard plots should include altitude, velocity, acceleration, trajectory, pitch/rate, thrust, drag, CG, convergence, residuals, and cross-provider overlays. Every figure footer or sidecar metadata must identify configuration hash, model version, provider, solver/settings, units, frames, and source.

## Acceptance policy

A stage passes only when:

- all lower gates pass or have an approved, explicit limitation;
- new warnings are explained;
- results are reproducible;
- incompatible comparisons are excluded rather than coerced;
- tolerances are justified before viewing the candidate result;
- no baseline is rewritten to make a failure disappear.
