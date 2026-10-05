# AeroVECTOR Target Architecture Plan

This plan describes the intended boundaries. AV0 did not implement them.

## Design constraints

- keep scientific code independent of Tkinter, VPython, serial I/O, and Dhruva UI code;
- keep the legacy simulator callable until regression comparison is complete;
- make units, frames, provenance, solver settings, warnings, and versions explicit;
- represent unknown engineering values as unknown, never as convenient defaults;
- support deterministic headless execution as the primary scientific interface;
- treat RocketPy, MATLAB, and OpenRocket as independent providers, not dependencies of the core equations;
- do not add hardware actuation or autonomous flight-command capability.

## Proposed package boundaries

```text
aerovector/
  config/          typed canonical configuration, units, provenance, hashing
  importers/       legacy text and OpenRocket adapters
  physics/
    dynamics/      state definition, frames, force/moment assembly
    aerodynamics/  preserved legacy aero adapter, later validated models
    propulsion/    immutable thrust/mass-flow data and interpolation
    environment/   atmosphere and wind interfaces
    mass/          mass, CG, inertia providers
    actuators/     optional simulated actuator response
    sensors/       optional sampled/noisy observation providers
  solvers/         common interface, legacy trapezoid, RK4, solve_ivp adapters
  events/          launch, burnout, apogee, stage, recovery, ground
  results/         canonical samples, events, diagnostics, metadata
  validation/      alignment, metrics, comparison providers, reports
  providers/       RocketPy, MATLAB interchange, OpenRocket results, Dhruva S1
  reporting/       Matplotlib engineering plots
  legacy/          thin adapters around current modules during migration
```

## Canonical configuration

AV1 should define immutable or validation-controlled models for:

- `VehicleGeometry`;
- `MassProperties`;
- `AerodynamicConfig`;
- `PropulsionConfig`;
- `EnvironmentConfig`;
- `InitialConditions`;
- `SimulationConfig`.

Each scalar engineering field should be able to carry `value`, `unit`, `uncertainty`, `source`, `provenance`, and `notes`. A missing value is represented explicitly. Derived values must identify their derivation and input sources.

The canonical model must also define coordinate frames and signs. At minimum:

- global vertical/up and downrange axes;
- body longitudinal and transverse axes;
- pitch sign and zero definition;
- AGL versus MSL altitude;
- nose-tip datum for axial positions;
- SI normalization at the core boundary.

## Simulation request and result

A headless request should contain a canonical configuration snapshot, solver choice/settings, seed, selected model versions, and output sampling policy. It should produce:

- immutable time-indexed state arrays;
- forces, moments, coefficients, mass/CG/inertia, atmosphere, and actuator channels;
- structured events with detection method and uncertainty;
- termination status;
- warnings with severity and time/range context;
- exact configuration/input hashes;
- code revision and provider versions;
- runtime and solver diagnostics.

The result should be serializable to JSON metadata plus columnar CSV/Parquet-style samples. Dhruva S1 mapping belongs at this result boundary, not inside the ODE function.

## Solver contract

The solver interface should accept a pure derivative function, initial state, interval, event functions, and settings. It should return samples, dense-output access where available, event roots, evaluation counts, and status.

Implementations planned:

- `LegacyTrapezoidSolver`: exact current update order for regression only;
- `RK4FixedStepSolver`: simple transparent reference;
- SciPy `RK45`, `DOP853`, and `LSODA` adapters through `solve_ivp`.

The actuator and sampled controller may require hybrid/discrete scheduling. Their updates must be explicit events or clocked components rather than hidden module timers.

## Model interfaces

Suggested conceptual contracts:

```text
Environment.at(position, time) -> atmospheric/wind state
Propulsion.at(time, events) -> thrust, mass flow, metadata
MassModel.at(time, events) -> mass, CG, inertia
Aerodynamics.evaluate(vehicle_state, environment, config) -> coefficients/forces/moments
Actuator.step(command, state, dt) -> actuator state/output
Dynamics.derivative(time, state, context) -> state derivative
EventDetector.evaluate(time, state, context) -> scalar/root condition
```

These interfaces should operate on NumPy arrays with documented shapes and SI units. They should not import GUI or provider modules.

## OpenRocket import boundary

The `.ork` importer should produce an `ImportPreview`, not a ready-to-run configuration. The preview contains:

- archive/XML version and source hash;
- component tree with stable XML paths and IDs;
- original text/value/unit and normalized SI value;
- supported mapping candidates;
- conflicts, missing requirements, unsupported components, and assumptions;
- importer version.

Only an explicit confirmation step creates/updates canonical configuration. ZIP entry paths, decompressed sizes, XML entity handling, and nesting depth require safety limits.

## Dhruva boundary

AV9 should map canonical AeroVECTOR results to the versioned Dhruva S1 schema. The repository currently contains only a control-oriented SITL adapter, not the platform result contract. Before implementation, obtain and fixture the authoritative S1 schema plus examples of Run Archive, evidence/provenance, S2 playback, and S3 playback records.

The mapping should be provider-neutral: AeroVECTOR and RocketPy results both enter the same canonical comparison/archive layer. No Dhruva UI logic should be added to the scientific core.

## Migration sequence

1. Freeze current behavior with AV0 fixtures.
2. Introduce canonical models and legacy adapters without changing numerical results.
3. Add `.ork` preview/import with no solver dependency.
4. Extract pure NumPy state/physics functions behind the exact legacy path.
5. Add solver interface and independent solvers.
6. Establish analytical, convergence, and regression gates.
7. Add external providers and validation reports.
8. Add uncertainty workflows only after deterministic gates pass.
9. Add Dhruva result mapping after schema acquisition.

## Architectural acceptance criteria

- importing the numerical core creates no GUI, browser, plot, or serial side effect;
- two runs can execute independently in the same process;
- every public array has a documented shape, dtype, unit, and frame;
- invalid and unknown values are distinguishable;
- run metadata fully reconstructs deterministic inputs;
- the legacy fixtures remain comparable until an intentional change is approved;
- external providers can be disabled without affecting core imports.
