# AV0 Numerical Risks

Risks are ordered by potential to invalidate scientific conclusions, not by implementation difficulty.

## Critical risks

### No convergence evidence

The fixed-step trapezoidal integrator is the only flight solver, and no timestep sweep or independent solution is checked. A visually plausible plot can therefore contain material integration error. The configured examples span 0.001 to 0.005 s, but equivalence is asserted only in comments.

Required control: AV5 convergence sweeps at 0.02, 0.01, 0.005, and 0.0025 s, with a finer reference where stable. Compare state histories, final state, maxima, and event times.

### State integration is not a standard coupled ODE step

Acceleration and pitch are integrated through separate mutable history objects. Pitch is advanced before translational acceleration is rotated; velocity is transformed using the new pitch, while force evaluation used the previous state. Local acceleration increments are then added to a transformed global velocity. This operator ordering may be first-order in important coupled terms even though each scalar integration uses a trapezoid.

Required control: express the exact current update as the legacy reference solver, build a single explicit state derivative for new solvers, and compare on analytical and manufactured cases before claiming equivalence.

### Event detection can false-trigger

Apogee is declared on one vertical-velocity sign change after launch plus 0.5 s. AV0 captured a false apogee in the active-fin case: the event occurred at 31.54 m, but the run later reached 61.56 m and was climbing. The same event can trigger second-stage ignition.

Required control: distinguish a preserved legacy event from a validated event detector; test persistence, interpolation of root time, hysteresis, powered-flight conditions, and re-ascent behavior.

### Random behavior is unseeded in the application

Wind gust and sensor noise use the process-global Python RNG. Normal runs do not record or accept a seed, so equivalent configurations are not reproducible.

Required control: inject a per-run generator and record its algorithm and seed. Preserve the legacy generator path for regression.

## High risks

### Silent defaults and partial parsing

Configuration conversion catches type errors and inserts defaults. Unknown or malformed engineering values can silently become plausible numbers. The motor reader skips malformed lines without diagnostics and adds a duplicate origin when a curve already starts at zero.

Required control: typed validation with explicit missing/invalid states, provenance, units, and warnings. Never modify the source curve.

### Non-zero thrust floor

`get_thrust()` returns at least 0.001 N, including outside burns. This prevents exact coast/free-fall conditions and can affect event logic or analytical verification at tight tolerances.

Required control: preserve as a documented legacy behavior in regression; remove or justify only through an explicit model change record.

### Mass properties tied only to burn time

Mass, inertia, and CG vary linearly with time over each burn, independent of actual thrust or a mass-flow curve. Irregular thrust profiles therefore do not imply corresponding propellant depletion. Stage transitions assume hand-entered endpoint values.

Required control: canonical propulsion data must distinguish thrust and mass-flow provenance; retain linear-time legacy mode until a validated alternative is available.

### Constant gravity and ambiguous altitude domains

Gravity is fixed at 9.8 m/s², while atmosphere receives `position_global[0] + launch_altitude`. Ground logic uses relative altitude. Negative or out-of-range atmospheric altitude behavior is not explicitly validated.

Required control: define AGL versus MSL types and test boundaries. Keep gravity as a configurable model rather than hiding it in equations.

### CP and force-application divisions

CP is calculated as moment divided by total normal coefficient, and force application as moment divided by normal force. Near-zero denominators can create very large, NaN, or infinite values. Only the plotted force-application location is saturated; the underlying CP path is not guarded by a declared numerical policy.

Required control: specify singular-state behavior and add zero/symmetric-AoA tests.

### Mach handling is stateful and incomplete

At Mach >= 0.9, `is_supersonic` is latched true, while `self.mach` is not updated in that branch before `beta = sqrt(1-mach²)`. Documentation recommends use only below Mach 0.6. A run aborts after a step rather than preventing unsupported coefficient evaluation.

Required control: explicit validity-domain checks before coefficient calculation, structured warning/termination, and boundary tests.

### Atmosphere layer and interpolation boundaries

Atmosphere and SciPy interpolation calls have mixed policies: some clamp, some extrapolate, and some raise. The policies are not surfaced in run metadata.

Required control: standardize explicit bounds behavior and test every table endpoint and out-of-domain request.

## Medium risks

### Time scheduling by threshold comparisons

Controller, servo, sensors, wind, plot sampling, and animation sampling use independent `t >= previous + sample*0.999` checks. Non-integer ratios to the integration step cause jitter and phase drift. Python SITL's `micros()` truncates floating time to an integer.

### Controller derivative at startup

The controller computes `(error - last_error)/(t - t_prev)` with both times initially zero. NumPy scalar propagation can produce warnings or infinities that are later saturated rather than handled explicitly.

### Variable-step serial SITL differs from local solver

Serial SITL sets integration step from wall-clock elapsed time. Results depend on operating-system scheduling and load, and cannot be directly compared with fixed-step runs without recording every realized step.

### Angle wrapping mutates only part of state history

Pitch is wrapped to [-pi, pi], after which `new_f(theta)` shifts history again. This can produce discontinuities or duplicated history updates around the wrap boundary.

### Parachute model encodes its target result

Effective area is derived every step from the target 4 m/s descent rate, current mass, and density. It is not a physical canopy model and cannot validate descent performance. The rotational restoring law is likewise artificial.

### Global mutable objects contaminate isolation

The `Rocket`, fin instances, controller, servo, warning flags, and arrays are module-global. Repeated simulations rely on manual resets and may leak state. Parallel execution is unsafe.

### Aero model validity is broader in code than evidence

The technical documentation acknowledges approximate high-angle extensions, underestimated fin drag, simplified stall, and rounded fin-edge assumptions. These uncertainties are not reported with results.

## Software and observability risks

- exceptions are often swallowed or converted to defaults;
- warnings are printed and stored globally, not attached to run results;
- termination reason is not structured;
- no configuration/version/input hash is stored by normal runs;
- GUI labels include inconsistent units and spelling;
- plot export depends on user-selected signals rather than a canonical result;
- relative paths depend on process CWD;
- dynamic SITL modules execute arbitrary Python by design and are not safe input parsers;
- `.ork` ZIP/XML parsing is absent;
- dependencies are unpinned and no CI exists.

## Risk treatment rule

AV1-AV5 must not silently fix these behaviors. First encode the old result and limitation, then add a separately named validated behavior with evidence and migration notes. A numerical difference is neither a regression nor an improvement until its cause and reference are documented.
