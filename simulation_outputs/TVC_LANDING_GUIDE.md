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
2. A separate idealized coast attitude hold keeps it upright after motor-1 burnout.
3. The landing predictor continuously calculates stopping distance from vertical velocity, motor-2 mean thrust, mass, and gravity.
4. Motor 2 ignites near 6.2 m while descending at approximately 14.7 m/s.
5. TVC controls pitch during the complete landing burn.
6. The simulator evaluates vertical speed, horizontal speed, pitch, and pitch rate at ground contact.

## Validated result

Deterministic seed `20261003`, wind 3 m/s, gust standard deviation 0.2 m/s, sensor noise disabled.

| Metric | Result |
|---|---:|
| Apogee | 17.63 m |
| Apogee time | 2.208 s |
| Landing-motor ignition altitude | 6.16 m |
| Landing-motor ignition time | 3.750 s |
| Ignition vertical velocity | about -14.66 m/s |
| Touchdown time | 4.740 s |
| Touchdown vertical velocity | -1.84 m/s |
| Touchdown horizontal velocity | 1.14 m/s |
| Touchdown pitch | -0.73 deg |
| Touchdown pitch rate | -3.14 deg/s |
| Parachute deployed | No |
| Result | TVC LANDING SUCCESS |

Success limits are 2.0 m/s vertical speed, 2.0 m/s horizontal speed, 3 deg pitch, and 5 deg/s pitch rate.

## Important physical limitation

The ascent motor file `niche wali csv TVC landing.csv` is a derived 41% thrust version created to study a 15-20 m flight. It is not a measured commercial motor curve. Motor 2 remains the supplied `upr wala csv.csv` curve.

The coast attitude hold is also idealized and represents separate hardware such as RCS or a reaction wheel. It is not TVC: an unlit motor cannot generate TVC torque. TVC controls attitude during the landing burn, and the vehicle coasts for roughly 0.18 s between motor-2 burnout and contact. Hardware use requires a real low-impulse ascent motor, a real coast attitude system, landing legs rated for the simulated velocities, measured ignition delay, and 3-D validation.
