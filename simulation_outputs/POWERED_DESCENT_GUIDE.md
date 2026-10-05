# Dhruva TVC-Powered Descent Profile

## Run

From PowerShell:

```powershell
cd "D:\RESOURCES\AeroVector-Simulator\TVC"
.\.venv\Scripts\python.exe .\AeroVECTOR.py
```

Close any AeroVECTOR window that was already running before this profile was updated; Python does not hot-reload the simulation module in an existing process.

In the File tab, open:

```text
3 - Examples\Example Python SITL\TVC_Itr01_AeroVECTOR_POWERED_DESCENT.txt
```

Then open **Run Simulation** and press **Run Simulation**. The profile locks its validated ascent and descent gains, so values left in the SITL Gains tab do not replace them.

## Profile behavior

- Motor 1 performs the ascent.
- Motor 2 (`upr wala csv.csv`) is reserved for descent braking.
- Motor 2 ignites after a 5.0 m drop from detected apogee.
- Descent TVC remains active during the complete motor-2 burn.
- The recovery command is issued after motor-2 burnout.
- The inherited near-apogee landing detector is inhibited until the powered burn occurs.

## Locked settings

| Setting | Value |
|---|---:|
| Motor-2 trigger | Distance After Apogee |
| Trigger value | 5.0 m |
| Ascent Kp / Ki / Kd | 0.02 / 0.00 / 0.04 |
| Ascent derivative filter | 0.50 |
| Descent Kp / Ki / Kd | 0.40 / 0.00 / 0.10 |
| Descent derivative filter | 0.50 |
| TVC limit | +/-12 deg |
| Launch rod | 1.0 m |
| 3-D playback slow motion | 1 (normal speed) |
| Passive fins | Enabled and attached |
| Stabilization-fin span | 0.10 m |

## Deterministic simulation result

Seed: `20261003`; wind 3 m/s with 0.2 m/s gust standard deviation; sensor noise disabled.

| Event / metric | Result |
|---|---:|
| Motor-1 burnout | 0.510 s |
| Apogee | 114.33 m at 5.042 s |
| Motor-2 ignition | 6.070 s, about 109.28 m |
| Vertical speed at ignition | -9.23 m/s |
| Pitch at ignition | about 72.9 deg |
| Maximum pitch during burn | about 82.3 deg |
| Vertical speed at motor-2 burnout | about +0.76 m/s |
| Pitch at motor-2 burnout | about 9.3 deg |
| Recovery command | 6.974 s, about 104.64 m |
| Ground contact | 33.508 s |
| Ground-contact vertical speed | -4.00 m/s |
| Ground-contact pitch | effectively 0 deg (upright) |
| Ground-contact horizontal speed | -0.06 m/s |

After the recovery command, the simulator now continues with its canopy drag and upright-restoring model until ground contact. The complete run therefore verifies the powered braking pulse, canopy descent, and upright simulated touchdown.

## Important flight limitation

This is an **experimental simulation profile, not a flight-ready landing solution**. The supplied second motor is fixed-thrust and cannot throttle or shut down. The rocket therefore falls tail-first only briefly, reaches a large tilt before ignition, and finishes the burn with a small upward rebound. The final upright descent is provided by the simulator's idealized parachute restoring-torque model, not by TVC after motor burnout. `Auto Parachute on Tilt` is disabled because the simulator's current abort threshold equals the 12 deg TVC actuator limit; enabling it would cancel motor 2 before this simulated maneuver.

For a physically robust powered landing, add coast-phase attitude authority and use a throttleable/restartable descent motor or a motor curve selected for the required braking impulse. Validate in 3-D with measured CG, inertia, actuator bandwidth, ignition delay, wind envelopes, and a hardware-in-the-loop recovery system before any flight use.
