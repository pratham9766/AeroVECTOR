# AV0 Baseline Behavior

## Purpose

These baselines preserve what the current working tree does before physics or solver refactoring. They are regression evidence, not a claim of physical correctness.

Capture code and artifacts are in `tests/baselines/`. Every case includes the exact copied configuration, copied motor inputs, SHA-256 hashes, a JSON summary, a CSV trace, four aerodynamic operating-point snapshots, and an engineering PNG.

Common capture metadata:

- Git revision: `276bf18a7594cd77ba25f24de354d5e637630064`
- Python: 3.11.9
- random seed: `20261003`
- solver: existing fixed-step trapezoidal implementation
- trace cadence: legacy VPython playback cadence, approximately 0.005 s
- deterministic replay: confirmed by a second capture with byte-identical traces and aerodynamic snapshots

The working tree contained pre-existing edits to Dhruva example configuration. Each fixture hashes and copies the actual input used, so this ambiguity is explicit.

## Representative cases

| Case | Model path | Step | Samples | Key events | Final condition |
|---|---|---:|---:|---|---|
| `legacy_tvc` | internal controller, TVC, Estes F15 | 0.002 s | 2552 | burnout 3.45 s; apogee 117.802919 m at 5.818 s | ground crossing at 15.31 s; altitude -0.561892 m; vertical speed -18.155800 m/s |
| `legacy_active_fins` | internal controller, active fins, AeroTech G12 | 0.005 s | 2001 | detector reports apogee 31.540968 m at 4.88 s | duration limit 10.0 s; altitude 61.558814 m; vertical speed +17.648200 m/s |
| `dhruva_dual_motor_sitl` | Python SITL, Dhruva adapter, two motors | 0.002 s | 1384 | first burnout 0.51 s; reported apogee 70.007322 m and motor-2 ignition at 3.568 s | ground crossing at 8.30 s; altitude -0.557065 m; vertical speed -24.291870 m/s |

The active-fin result is an important baseline anomaly. A vertical-velocity sign change at 4.88 s is treated as permanent apogee, yet the vehicle later reaches a higher altitude and is climbing at the run limit. The current detector has no persistence, hysteresis, maximum-altitude confirmation, or motor-state condition. Because the dual-motor logic can ignite on this event, AV5 must test event semantics independently from trajectory convergence.

## Final state snapshots

### Legacy TVC

```json
{
  "time_s": 15.31,
  "altitude_m": -0.5618917243581126,
  "downrange_m": -7.8114601051616495,
  "vertical_velocity_m_s": -18.155799554049405,
  "horizontal_velocity_m_s": -2.870675634595808,
  "pitch_rad": 2.583472956484387,
  "pitch_rate_rad_s": -0.21490014904294683,
  "mass_kg": 0.6,
  "xcg_m": 0.51,
  "pitch_inertia_kg_m2": 0.0601
}
```

### Legacy active fins

```json
{
  "time_s": 10.0,
  "altitude_m": 61.55881382472646,
  "downrange_m": -264.1948284702565,
  "vertical_velocity_m_s": 17.648199828155164,
  "horizontal_velocity_m_s": -43.20140370177025,
  "pitch_rad": 1.0564037302782485,
  "pitch_rate_rad_s": -0.03453916316761883,
  "mass_kg": 1.0,
  "xcg_m": 0.55,
  "pitch_inertia_kg_m2": 0.0662
}
```

### Dhruva dual motor SITL

```json
{
  "time_s": 8.3,
  "altitude_m": -0.5570651849547282,
  "downrange_m": 41.47344758795732,
  "vertical_velocity_m_s": -24.29187031033632,
  "horizontal_velocity_m_s": 4.968019691525375,
  "pitch_rad": -0.3737332971527701,
  "pitch_rate_rad_s": -20.179288571694062,
  "mass_kg": 0.771,
  "xcg_m": 0.4,
  "pitch_inertia_kg_m2": 0.021
}
```

## Captured artifacts

The manifest is `tests/baselines/manifest.json`. Per-case directories contain:

- `input_config.txt`: exact legacy input;
- `primary_motor.csv` and, where applicable, `secondary_motor.csv`;
- `summary.json`: hashes, solver settings, runtime, events, and final state;
- `trace.csv`: time, position, velocity, pitch, AoA, actuator, thrust, and CG;
- `aerodynamics.json`: coefficient/CP snapshots at zero and five-degree nominal AoA, with pitch-rate and actuator variations;
- `trajectory.png`: altitude and vertical velocity against time.

The harness runs headlessly by replacing only GUI value access. It invokes the existing `SaveFile`, `Rocket`, `Controller`, `Servo`, `run_sim_local()`, and `run_sim_python_sitl()` code paths. It does not duplicate the dynamics equations or integration algorithm.

## Termination behavior

The local and Python-SITL loops stop on the first of:

- parachute landing below -0.1 m;
- ground crossing below -0.55 m;
- SITL parachute command;
- configured duration;
- Mach at or above the model's hard abort threshold;
- for serial SITL only, an additional ten-burn-time condition.

These conditions are loop-specific. Termination reason is printed, not returned as structured data. Baseline summaries infer completion from state and printed behavior; AV3/AV4 should introduce a typed termination record without changing these semantics until regression comparison is complete.

## Reproduction

From the repository root, with the existing environment:

```powershell
.\.venv\Scripts\python.exe .\tests\baselines\capture_av0_baselines.py
```

A deterministic regression should compare exact input hashes first, then compare events and numerical arrays within a declared tolerance. Runtime must be tracked but not used as a deterministic equality condition.
