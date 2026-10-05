# Optimized Dhruva Dual-Motor Profile

## Run

From PowerShell:

```powershell
cd "D:\RESOURCES\AeroVector-Simulator\TVC"
.\.venv\Scripts\python.exe .\AeroVECTOR.py
```

In the File tab, open:

```text
3 - Examples\Example Python SITL\TVC_Itr01_AeroVECTOR_OPTIMIZED.txt
```

Then open **Run Simulation** and press **Run Simulation**. The optimized SITL module locks the validated gains, so the SITL Gains tab does not need manual editing for this profile.

## Tuned values

| Setting | Original | Optimized |
|---|---:|---:|
| Motor 2 trigger | Distance after apogee | Time after motor-1 burnout |
| Trigger delay | 0 m drop | 0.00 s |
| Launch rod length | 0 m | 1.00 m |
| Passive fins | Disabled | Enabled, attached |
| Fin root | local-looking `0, 0.079629` | absolute `0.570371, 0.079629` m |
| Fin tip | local-looking `0.04, 0.04` | absolute `0.610371, 0.04` m |
| Fin span | 0.08 m | 0.10 m |
| SITL Kp | 0.05 | 0.02 |
| SITL Ki | 0.04 | 0.00 |
| SITL Kd | 0.01 | 0.04 |
| Derivative filter | 0.90 | 0.50 |
| Descent parachute adapter | Disabled | Enabled |

## Deterministic simulation result

Seed: `20261003`; wind 3 m/s with 0.2 m/s gust standard deviation; sensor noise disabled.

| Metric | Original | Optimized |
|---|---:|---:|
| Pitch at motor-2 ignition | 155.45° | 0.49° |
| Ascent pitch RMS | 90.73° | 7.48° |
| Maximum ascent pitch magnitude | 179.56° | 18.22° |
| Actuator saturation before recovery | 84.1% | 0% |
| Apogee | 68.08 m | 206.11 m |
| Motor-2 ignition | 3.516 s | 0.510 s |
| Apogee time | 3.516 s | 6.750 s |
| Recovery | Ground impact | Parachute command at 7.092 s |

The legacy simulator stops when the Python SITL module commands parachute deployment, so this run verifies deployment but does not model the complete canopy descent or landing speed.

## Physical verification required

This is a simulation-specific optimum, not automatic flight authorization. Before hardware use, verify that the actual rocket has the modeled 1 m effective rail, the fin positions and 0.10 m exposed span, adequate static margin across both mass states, reliable immediate staging at motor-1 burnout, and a separately validated recovery system. Re-run with measured inertia, CG, actuator dynamics, wind envelopes, and motor ignition delay.
