# Dhruva Low-Altitude TVC Landing Profile

## Run

```powershell
cd "D:\RESOURCES\AeroVector-Simulator\TVC"
.\.venv\Scripts\python.exe .\AeroVECTOR.py
```

Load:

```text
3 - Examples\Example Python SITL\TVC_Itr01_AeroVECTOR_TVC_LANDING.txt
```

Then press **Run Simulation**. This is a separate profile; it does not deploy a parachute.

## Flight sequence

1. The reduced ascent motor raises the rocket to approximately 17.6 m.
2. A bounded coast attitude system—with deadband, limited authority, and disturbance torque—works to keep it upright after motor-1 burnout.
3. The landing predictor continuously calculates stopping distance from vertical velocity, motor-2 mean thrust, mass, and gravity.
4. Motor 2 ignites near 6.2 m while descending at approximately 14.7 m/s.
5. TVC controls pitch during the complete landing burn.
6. The simulator evaluates vertical speed, horizontal speed, pitch, and pitch rate at ground contact.

## Validated result

Deterministic seed `20261003`, wind 5 m/s, gust standard deviation 0.5 m/s, sensor noise disabled.

| Metric | Result |
|---|---:|
| Apogee | 17.75 m |
| Apogee time | 2.222 s |
| Landing-motor ignition altitude | 6.19 m |
| Landing-motor ignition time | 3.772 s |
| Ignition vertical velocity | about -14.67 m/s |
| Touchdown time | 4.776 s |
| Touchdown vertical velocity | -1.97 m/s |
| Touchdown horizontal velocity | 1.76 m/s |
| Touchdown pitch | 1.34 deg |
| Touchdown pitch rate | -1.16 deg/s |
| Maximum descent pitch | about 2.49 deg |
| Descent pitch RMS | about 1.60 deg |
| Maximum descent actuator deflection | about 2.05 deg |
| Parachute deployed | No |
| Result | TVC LANDING SUCCESS |

Success limits are 2.0 m/s vertical speed, 2.0 m/s horizontal speed, 3 deg pitch, and 5 deg/s pitch rate.

## Important physical limitation

The ascent motor file `niche wali csv TVC landing.csv` is a derived 41% thrust version created to study a 15-20 m flight. It is not a measured commercial motor curve. Motor 2 remains the supplied `upr wala csv.csv` curve.

The coast attitude system still represents separate hardware such as RCS or a reaction wheel. It is not TVC: an unlit motor cannot generate TVC torque. Unlike the earlier ideal hold, the model now has a 2 deg deadband, bounded angular acceleration, stronger deterministic disturbance torque, 0.40 deg motor misalignment, 5 m/s wind, and 0.5 m/s gusts. TVC visibly corrects the sustained wobble during the landing burn. The vehicle coasts briefly between motor-2 burnout and contact. Hardware use requires a real low-impulse ascent motor, a real coast attitude system, landing legs rated for the simulated velocities, measured ignition delay, and 3-D validation.
