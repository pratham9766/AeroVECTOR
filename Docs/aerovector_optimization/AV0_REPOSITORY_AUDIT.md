# AV0 Repository Audit

Status: complete for the repository state inspected on 2026-10-03.

Audit revision: `276bf18a7594cd77ba25f24de354d5e637630064` on branch `multimotor-codebase`.

The checkout was already dirty before AV0. In particular, the two Dhruva example configuration files and several cache/environment paths were modified or untracked. AV0 preserved those changes. The captured baselines therefore describe the working tree, and each copied input is protected by a SHA-256 hash.

## Executive finding

AeroVECTOR is a legacy desktop-oriented, two-dimensional rocket-flight simulator and controller tuner. It models vertical translation, downrange translation, and pitch. The physical model is substantial, but simulation state is held in module globals and the numerical loop is coupled to Tkinter, VPython, plotting, configuration parsing, control, and SITL. The current integrator is a fixed-step trapezoidal scheme implemented through mutable scalar history objects. No SciPy ODE solver is used.

The repository has useful foundations for the optimization track: documented equations, NumPy transforms, SciPy interpolation, an ISA implementation, detailed fin aerodynamics, thrust-curve input, mass/CG/inertia evolution, a Python SITL contract, and a Dhruva-specific adapter. It lacks a typed canonical configuration, a headless public simulation API, dependency metadata, automated source tests, explicit units in data structures, numerical convergence evidence, and a provider-neutral result schema.

## Repository structure

| Area | Current contents | Role |
|---|---|---|
| `AeroVECTOR.py` | Tkinter application bootstrap | Primary user entry point; changes CWD and starts the GUI |
| `src/simulation/main_simulation.py` | Dynamics loop, global state, fixed-step integration, events, SITL loops, plotting, VPython playback | Physics orchestration, numerical integration, visualization, and I/O are coupled here |
| `src/aerodynamics/rocket_functions.py` | `Rocket`, body aerodynamics, drag, CP, propulsion, mass/CG/inertia interpolation | Main vehicle/aerodynamics model |
| `src/aerodynamics/fin_aerodynamics.py` | Low/moderate/ultra-low aspect-ratio fin aerodynamics | Empirical and semi-empirical fin model |
| `src/aerodynamics/fin_physical_properties.py` | Fin geometry and derived properties | Geometry preprocessing and validation warnings |
| `src/aerodynamics/flight_conditions.py` | Atmosphere-backed Mach/Reynolds conditions | Aerodynamic operating point |
| `src/isacalc/` and `src/ISA_calculator.py` | Layered International Standard Atmosphere | Temperature, pressure, density, sound speed, viscosity |
| `src/simulation/servo_lib.py` | SG90 state-space response and Tustin discretization | Optional actuator-response model |
| `src/control.py` | PID/torque-compensated internal controller | Controller model coupled back to simulation globals |
| `src/files.py` | Positional text configuration, motor CSV reading, plot export, SITL loading | Data/configuration layer, but imports GUI code |
| `src/gui/` | Tkinter tabs, canvas drawing, file operations | Configuration editor and desktop UI |
| `src/python_sitl_functions.py` | Time, sensor, command, and plot-variable bridge | Minimal Python SITL interface |
| `3 - Examples/` | Passive, TVC, active-fin, landing, Arduino SITL, Python SITL, Dhruva adapter | Representative configurations and scripts |
| `Motors/` | CSV and one ENG thrust data file | Propulsion inputs with inconsistent header/metadata formats |
| `Gaintuningork.ork` | OpenRocket ZIP/XML project | External configuration and validation opportunity |
| `AeroVECTOR-Technical-Documentation.pdf` | 37-page model document | Primary equation/assumption reference |
| `2 - Tests/` | Interactive plotting/engineering scripts | Manual exploratory checks, not automated assertions |
| `tests/` | Only cached historical test bytecode before AV0 | No runnable test source was present |

## Entry points and execution paths

The supported desktop entry point is `AeroVECTOR.py`. It initializes eight Tkinter tabs and calls `mainloop()`. A simulation starts through the Run Simulation button, which calls `main_simulation.run_simulation()` and then `run_3d()`.

`run_simulation()` resets module globals, reads values from GUI widget objects, updates the shared `Rocket`, `Controller`, and `Servo` instances, selects one of three loops, and finally plots:

- `run_sim_local()` for the internal controller;
- `run_sim_sitl()` for serial/Arduino SITL;
- `run_sim_python_sitl()` for a dynamically loaded Python `SITLProgram`.

There was no headless application API. AV0 therefore added an observation-only harness under `tests/baselines/` that supplies value-returning GUI stubs while calling the existing setup and run methods unchanged.

## Current physical model

### State and coordinate frames

The simulated degrees of freedom are global vertical position, global downrange position, and pitch. `theta = 0` means the rocket body axis points vertically upward. Body/local axes are longitudinal `X` and transverse `Z`; global array element 0 is vertical and element 1 is horizontal/downrange.

The documented and implemented transforms are:

```text
body -> global: [[ cos(theta),  sin(theta)],
                 [-sin(theta),  cos(theta)]]

global -> body: transpose(body -> global)
```

Angles inside the model are radians; many GUI and SITL boundaries use degrees. Positions such as CG, CP, and motor mount are measured from the nose tip.

### Equations of motion

For normal flight, the implementation evaluates body-axis forces and pitch moment:

```text
Fx = thrust*cos(motor_angle) - q*S*CA + m*gx
Fz = thrust*sin(motor_angle) + m*gz + q*S*CN
My = thrust*sin(motor_angle)*(xt - xcg) + q*S*d*CMcg
```

It then computes `U_ddot = Fx/m`, `W_ddot = Fz/m`, and `Q_dot = My/Iy`. The documented vector-derivative terms are disabled (`v_d = 0`). Instead, each step transforms the current global velocity into the new body frame, adds the trapezoidally integrated local acceleration increments, and transforms velocity back to global coordinates.

Gravity is a constant `9.8 m/s^2`, transformed into the body frame. The launch rod suppresses lateral and pitch acceleration until global vertical altitude exceeds the vertical component of the configured rod length. Pad hold keeps all accelerations zero until thrust exceeds weight.

The parachute path is a separate simplified global-axis model. Effective area is recalculated to target a fixed 4 m/s terminal descent at the current mass/density; pitch receives an artificial restoring acceleration `-4Q - 6theta`.

### Aerodynamic model

Body normal force uses a modified Extended Barrowman treatment plus Galejs body lift. The reference area is the maximum vehicle cross-section. Components are conical/frustum segments, with optional ogive integration. Component angle of attack includes pitch-rate-induced tangential velocity. CP is a force-weighted location; moment coefficient is `CN * (CP - CG) / diameter`.

Body axial force includes skin friction, pressure drag, and base drag, with empirical compressibility correction. The code aborts the flight when Mach reaches 0.9; the technical document states the intended range is below about Mach 0.6.

Fin aerodynamics supports trapezoidal stabilization and control fins, attached or detached. It uses Diederich lift-slope relations, wind-tunnel interpolation for ultra-low aspect ratios, Reynolds-dependent limits, empirical post-stall curves, fin-body interference, and an angle-dependent aerodynamic-center location. The technical document explicitly reports material uncertainty: fin drag is typically underestimated by about 25%, with larger possible zero-lift-drag error for some aspect ratios.

### Propulsion and mass properties

Motor CSVs are parsed permissively: lines whose first two comma-separated fields convert to floats are accepted; other lines are skipped. A leading `(0, 0)` point is always inserted, even when already present. Thrust uses `numpy.interp`, is bounded by explicit burn-time checks, and is floored to `0.001 N` rather than reaching exact zero.

Mass, pitch inertia, and CG are interpolated linearly from liftoff to first burnout. Optional second-stage values are interpolated linearly from first-burnout values after second ignition. No mass-flow curve is represented independently of thrust.

### Atmosphere and environment

The bundled ISA model returns temperature, pressure, density, sound speed, and Sutherland viscosity by altitude layer. The equations of motion directly call this implementation through the aerodynamic `Rocket`; there is no interchangeable environment interface. Wind is a constant global-horizontal component plus a Gaussian sample updated every 0.1 seconds. The random generator is unseeded in normal application use.

### Actuator and sensor representation

The servo is a nonlinear second-order SG90 model with `K` and `J` interpolated against requested displacement. It is discretized with a Tustin transformation at the current simulator step, then applies sample delay, angle quantization, gearing, and a configurable load compensation factor.

Serial and Python SITL expose pitch rate, two accelerometer channels, altitude, GNSS position, and GNSS velocity. Optional sensor noise is zero-bias Gaussian noise with per-sensor standard deviations and sample times. There is no drift, bias instability, correlation, saturation, quantization model beyond rounding, or deterministic seed in the normal UI workflow.

## Configuration system and duplication

The save format is a positional text file split by `###=#`. `SaveFile` knows ordered parameter-name arrays, GUI tabs know the same ordering, and `update_all_parameters()` repeats index-to-variable mapping with compatibility branches for older lengths. Defaults are silently substituted on missing or invalid values. Units exist mainly in labels and comments, not the serialized data model.

Configuration is duplicated across:

- `SaveFile.parameter_names`, `conf_*_names`, and creation defaults;
- Tkinter widget ordering and conversion logic;
- `main_simulation.update_all_parameters()` indexes and fallback defaults;
- example text files;
- controller and servo constructor defaults;
- top-level simulation globals;
- the Dhruva SITL adapter's own gains, limits, and state thresholds.

This is the primary AV1 problem. Migration must preserve unknown values and original source/provenance rather than normalizing by silent defaults.

## Dhruva integration inventory

The only Dhruva-named interface in this checkout is `DHRUVA_AeroVECTOR_SITL.py`. It adapts AeroVECTOR's Python SITL contract to a Dhruva-like state machine, altitude/velocity filter, and pitch PID. It explicitly notes that AeroVECTOR cannot represent yaw, magnetometer data, the second TVC axis, or hardware-specific drivers.

Reusable pieces are limited but useful:

- time API: `millis()` and `micros()`;
- sensor packet: gyro, two accelerations, altitude, GNSS position/velocity;
- command packet: one servo angle, parachute flag, optional ignition flag;
- ten generic plotted variables;
- a `SITLProgram` lifecycle of initialization hook, `void_setup()`, and `void_loop()`;
- the adapter's named flight states and logged transitions.

No Dhruva S1 provider-neutral result schema, run archive client, job/evidence API, or S2/S3 playback schema is present in this repository. AV9 therefore requires either importing those contracts from the actual Dhruva platform repository or receiving their versioned schema definitions. The current SITL module should be treated as an adapter example, not the S1 integration contract.

## OpenRocket compatibility

There is no `.ork` reader in the code. `Gaintuningork.ork` is a valid ZIP containing `rocket.ork`, XML version `1.10`, created by OpenRocket `24.12`. It contains one stage, nose cone, body tube/motor mount, parachute, two mass components, inner tube, three launch lugs, a GATI-E30 motor reference, and two saved simulations.

Saved Simulation 1 uses OpenRocket's RK4 simulator and Barrowman calculator with a 0.05 s configured step. It records 24.501 m apogee at 2.626 s and 4.972 s flight time. Simulation 2 is aborted at 0.85 s. The XML also contains detailed time-series values and events.

Import is feasible with the standard ZIP and XML libraries, but mapping is not one-to-one. OpenRocket's component hierarchy, relative positioning, shape vocabulary, computed mass properties, recovery configuration, and 3-D flight definitions exceed AeroVECTOR's current positional body-point and two-fin-set format. The importer must retain unmapped fields, namespaces/version, component IDs/paths, original units, normalized values, and conflicts. It must reject unsafe ZIP paths and resource exhaustion.

## Testing and dependencies

There is no `requirements.txt`, `pyproject.toml`, lockfile, CI configuration, or runnable automated test suite. `pytest` is not installed. `2 - Tests/` contains interactive engineering scripts and plots without assertions. Historical `tests/*.pyc` files exist without their source and cannot serve as maintainable tests.

Direct runtime dependencies are Tkinter, NumPy, SciPy, Pandas, Matplotlib, VPython, and optional PySerial. The working `.venv` used Python 3.11.9 with NumPy 2.4.6, SciPy 1.17.1, Pandas 3.0.6, Matplotlib 3.11.2, VPython 7.6.5, and PySerial 3.5.

AV0 verification performed:

- all current Python source and the AV0 harness compiled successfully;
- test discovery could not run because `pytest` is absent;
- three representative simulations completed;
- repeated capture produced byte-identical traces and aerodynamic snapshots with seed `20261003`.

## Separation assessment

| Concern | Current location | Separation quality |
|---|---|---|
| Physics | `main_simulation.py`, `rocket_functions.py`, fin modules | Mixed with events, controller, GUI-derived globals, and visualization buffers |
| Numerical integration | `IntegrableVariable` plus `simulation()` | Embedded in global mutable state; no solver interface |
| Data/configuration | `files.py`, GUI widgets, example text files | Positional, duplicated, silently defaulted |
| Visualization | `main_simulation.py`, `gui_functions.py`, VPython | Reads physics globals directly and shares buffers with solver loop |
| Validation | manual scripts, technical PDF, newly captured AV0 fixtures | No unified comparison model or automated metrics |

## Known TODOs and warnings

The only explicit source-level known bug says VPython force arrows can point incorrectly. Experimental switches for body discretization, dynamic-pressure damping scaling, and per-component/fin Reynolds handling are hard-coded off. The README also warns that landing aerodynamics are not reliable. The code emits warnings for invalid dimensions, CG inside an ogive, zero fin thickness, transition aspect ratio, and stalled fins, but warnings are process-global flags rather than structured per-run evidence.

## AV0 conclusion

AV1 should introduce a typed, unit-explicit, provenance-preserving canonical configuration and adapters from the legacy text format only. It should not change equations, aerodynamic outputs, event definitions, or the integrator. The AV0 fixtures are now the regression boundary for that work.
