# Units and Coordinate Frames

## Canonical SI units

| Quantity | Canonical unit | Accepted legacy unit |
|---|---|---|
| Length, CG, altitude, fin dimensions | `m` | `m` |
| Surface roughness | `m` | `um` |
| Mass | `kg` | `kg` |
| Pitch inertia | `kg*m^2` | `kg*m^2` |
| Linear velocity | `m/s` | `m/s` |
| Linear acceleration/noise SD | `m/s^2` | `g0` |
| Force | `N` | `N` |
| Angle | `rad` | `deg` |
| Angular rate/noise SD | `rad/s` | `deg/s` |
| Time/sample period | `s` | `s` |
| Ratios/gains/toggles | `1` or no unit | legacy dimensionless value |

Registered conversions are deterministic scalar conversions: `deg`/`rad`, `deg/s`/`rad/s`, `um`/`m`, `mm`/`m`, and standard gravity `g0 = 9.80665 m/s^2`. Unregistered units or incompatible conversions raise `UnitError`.

The legacy adapter converts to SI on import and converts back to the original legacy conventions on regeneration. This is a representation change only; the downstream simulator continues to receive its historical positional values and units.

## Frames and signs

AV1 records the existing conventions without transforming runtime values:

| Convention | Definition |
|---|---|
| Global frame | 2D inertial launch frame used by the legacy simulator |
| Global +x | Horizontal direction used by the legacy equations |
| Global +z | Upward; altitude increases upward |
| Body frame | 2D vehicle-fixed pitch plane |
| Body axial datum | Nose-based legacy axial coordinate |
| Body +axial | From nose toward tail |
| Pitch sign | Existing simulator/controller sign convention, unchanged |
| Actuator sign | Existing servo/TVC sign convention, unchanged |
| `launch_altitude_msl` | Launch-site elevation above mean sea level |
| `initial_altitude_agl` | Initial displacement above launch point |

No handedness, force/moment sign, aerodynamic angle, gravity direction, or altitude reference was changed in AV1. A future phase must introduce an explicit transform and regression evidence before changing any convention.
