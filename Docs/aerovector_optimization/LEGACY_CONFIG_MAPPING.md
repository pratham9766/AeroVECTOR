# Legacy Configuration Mapping

Positions below are zero-based because they correspond directly to the established Python lists. Every supported legacy position is mapped. Canonical values use SI; the adapter regenerates the legacy units shown.

## Parameters section

| Legacy position | Legacy name | Canonical field | Unit | Validation | Notes |
|---:|---|---|---|---|---|
| 0 | Motor | `propulsion.primary_motor_file` | text | Required file name | File existence checked when motor directory is known |
| 1 | Motor 2 | `propulsion.secondary_motor.motor_file` | text | Required when trigger enabled | May be `None`/disabled; legacy upgrade default |
| 2 | Motor 2 Trigger Mode | `propulsion.secondary_motor.trigger_mode` | enum | Supported trigger enum | Legacy upgrade default |
| 3 | Motor 2 Trigger Value | `propulsion.secondary_motor.trigger_value` | `s`, `m`, or `1` | Finite | Unit selected by trigger mode; legacy upgrade default |
| 4 | Mass Liftoff | `mass_properties.mass_liftoff` | `kg` | Positive | — |
| 5 | Mass Burnout | `mass_properties.mass_burnout` | `kg` | Positive | — |
| 6 | Mass Burnout 2 | `mass_properties.mass_burnout_secondary` | `kg` | Positive | Legacy upgrade copies burnout |
| 7 | Iy Liftoff | `mass_properties.pitch_inertia_liftoff` | `kg*m^2` | Positive | — |
| 8 | Iy Burnout | `mass_properties.pitch_inertia_burnout` | `kg*m^2` | Positive | — |
| 9 | Iy Burnout 2 | `mass_properties.pitch_inertia_burnout_secondary` | `kg*m^2` | Positive | Legacy upgrade copies burnout |
| 10 | Xcg Liftoff | `mass_properties.cg_liftoff` | `m` | Nonnegative | Nose-datum coordinate |
| 11 | Xcg Burnout | `mass_properties.cg_burnout` | `m` | Nonnegative | Nose-datum coordinate |
| 12 | Xcg Burnout 2 | `mass_properties.cg_burnout_secondary` | `m` | Nonnegative | Legacy upgrade copies burnout |
| 13 | Xt | `actuator.motor_mount_position` | `m` | Nonnegative | Nose-datum coordinate |
| 14 | Servo Resolution/definition | `actuator.servo_resolution` | `deg` → `rad` | Nonnegative | — |
| 15 | Max Actuator Angle | `actuator.maximum_actuator_angle` | `deg` → `rad` | Nonnegative | — |
| 16 | Actuator Reduction | `actuator.actuator_reduction` | `1` | Positive | — |
| 17 | Initial Misalignment | `actuator.initial_misalignment` | `deg` → `rad` | Finite | Signed value retained |
| 18 | Servo Compensation | `actuator.servo_compensation` | `1` | Nonnegative | — |
| 19 | Wind | `environment.mean_wind_speed` | `m/s` | Finite | Signed legacy value preserved |
| 20 | Wind Gust | `environment.wind_gust_standard_deviation` | `m/s` | Nonnegative | — |
| 21 | Launch Rod Length | `launch.launch_rod_length` | `m` | Nonnegative | — |
| 22 | Launch Rod Angle | `launch.launch_rod_angle` | `deg` → `rad` | Finite | Signed value retained |
| 23 | Motor Misalignment | `actuator.motor_misalignment` | `deg` → `rad` | Finite | Signed value retained |
| 24 | Rocket Roughness | `aerodynamics.body_roughness` | `um` → `m` | Nonnegative | — |
| 25 | Stabilization Fin Roughness | `aerodynamics.stabilization_fin_roughness` | `um` → `m` | Nonnegative | — |
| 26 | Control Fin Roughness | `aerodynamics.control_fin_roughness` | `um` → `m` | Nonnegative | — |

The old 21-position layout is upgraded by inserting positions 1–3 and copying positions 5→6, 8→9, and 11→12. Inserted/copied values are marked `DEFAULTED_LEGACY`.

## 3D display section

| Legacy position | Legacy name | Canonical field | Unit | Validation | Notes |
|---:|---|---|---|---|---|
| 0 | Toggle 3D | `display.enable_3d` | boolean | `True`/`False` | — |
| 1 | Camera Shake Toggle | `display.camera_shake` | boolean | `True`/`False` | — |
| 2 | Hide Forces | `display.hide_forces` | boolean | `True`/`False` | — |
| 3 | Variable FOV | `display.variable_fov` | boolean | `True`/`False` | — |
| 4 | Hide cg | `display.hide_cg` | boolean | `True`/`False` | — |
| 5 | Camera Type | `display.camera_type` | enum | Existing camera modes only | — |
| 6 | Slow mo | `display.slow_motion_factor` | `1` | Positive | — |
| 7 | Force Scale | `display.force_scale` | `m/N` | Nonnegative | Display-only scale |
| 8 | FOV | `display.field_of_view` | `1` | Positive | Legacy display value |

## Controller/simulation section

| Legacy position | Legacy name | Canonical field | Unit | Validation | Notes |
|---:|---|---|---|---|---|
| 0 | Torque Controller | `simulation.torque_controller` | boolean | `True`/`False` | Existing mode |
| 1 | Anti Windup | `simulation.anti_windup` | boolean | `True`/`False` | Existing mode |
| 2 | Auto Parachute on Tilt | `recovery.auto_parachute_on_tilt` | boolean | `True`/`False` | Old layout inserts `True`; `DEFAULTED_LEGACY` |
| 3 | Input Type | `simulation.input_mode` | enum | Supported mode | `Step [º]`, `Ramp [º/s]`, or `Up` |
| 4 | Kp | `simulation.kp` | `1` | Finite | — |
| 5 | Ki | `simulation.ki` | `1` | Finite | — |
| 6 | Kd | `simulation.kd` | `1` | Finite | — |
| 7 | K All | `simulation.overall_gain` | `1` | Finite | — |
| 8 | K Damping | `simulation.damping_gain` | `1` | Finite | — |
| 9 | Reference Thrust | `simulation.reference_thrust` | `N` | Nonnegative | — |
| 10 | Input | `simulation.input_value` | `deg`/`deg/s` → `rad`/`rad/s` | Finite | Unit follows input mode |
| 11 | Input time | `simulation.input_time` | `s` | Nonnegative | — |
| 12 | Launch Time | `simulation.launch_time` | `s` | Nonnegative | — |
| 13 | Servo Sample Time | `simulation.servo_sample_time` | `s` | Nonnegative | Zero legacy behavior retained |
| 14 | Controller Sample Time | `simulation.controller_sample_time` | `s` | Nonnegative | Zero legacy behavior retained |
| 15 | Maximum Sim Duration | `simulation.maximum_duration` | `s` | Positive | — |
| 16 | Sim Delta T | `simulation.integration_step` | `s` | Positive | — |
| 17 | Export T | `simulation.export_step` | `s` | Positive | — |
| 18 | Launch Altitude | `launch.launch_altitude_msl` | `m` | Finite | MSL reference |
| 19 | Initial Altitude | `launch.initial_altitude_agl` | `m` | Finite | AGL reference |
| 20 | Initial Vertical Velocity | `launch.initial_vertical_velocity` | `m/s` | Finite | Existing sign convention |
| 21 | Initial Horizontal Velocity | `launch.initial_horizontal_velocity` | `m/s` | Finite | Existing sign convention |
| 22 | Initial Pitch Angle | `launch.initial_pitch_angle` | `deg` → `rad` | Finite | Existing sign convention |
| 23 | Initial Pitch Rate | `launch.initial_pitch_rate` | `deg/s` → `rad/s` | Finite | Existing sign convention |

## SITL/sensor section

| Legacy position | Legacy name | Canonical field | Unit | Validation | Notes |
|---:|---|---|---|---|---|
| 0 | Activate SITL | `sensors.sitl_active` | boolean | `True`/`False` | Existing behavior |
| 1 | Use Sensor Noise | `sensors.noise_enabled` | boolean | `True`/`False` | Existing behavior |
| 2 | Python SITL | `sensors.python_sitl` | boolean | `True`/`False` | Existing behavior |
| 3 | File | `sensors.python_module` | text | String | Empty allowed for non-Python modes |
| 4 | Port | `sensors.serial_port` | text | String | Empty allowed for Python mode |
| 5 | Baudrate | `sensors.baudrate` | `1` | Positive integer | — |
| 6 | Gyroscope SD | `sensors.gyro_noise_sd` | `deg/s` → `rad/s` | Nonnegative | — |
| 7 | Accelerometer SD | `sensors.accelerometer_noise_sd` | `g0` → `m/s^2` | Nonnegative | Standard gravity conversion |
| 8 | Altimeter SD | `sensors.altimeter_noise_sd` | `m` | Nonnegative | — |
| 9 | GNSS Pos SD | `sensors.gnss_position_noise_sd` | `m` | Nonnegative | — |
| 10 | GNSS Vel SD | `sensors.gnss_velocity_noise_sd` | `m/s` | Nonnegative | — |
| 11 | Gyroscope ST | `sensors.gyro_sample_time` | `s` | Positive | — |
| 12 | Accelerometer ST | `sensors.accelerometer_sample_time` | `s` | Positive | — |
| 13 | Altimeter ST | `sensors.altimeter_sample_time` | `s` | Positive | — |
| 14 | GNSS ST | `sensors.gnss_sample_time` | `s` | Positive | — |

## Plot section

| Legacy position | Legacy name | Canonical field | Unit | Validation | Notes |
|---:|---|---|---|---|---|
| 0 | First Plot | `display.plot_channels[0]` | text | Non-empty | Existing labels, including `Off` |
| 1 | Second Plot | `display.plot_channels[1]` | text | Non-empty | Existing labels, including `Off` |
| 2 | Third Plot | `display.plot_channels[2]` | text | Non-empty | Existing labels, including `Off` |
| 3 | Fourth Plot | `display.plot_channels[3]` | text | Non-empty | Existing labels, including `Off` |
| 4 | Fifth Plot | `display.plot_channels[4]` | text | Non-empty | Existing labels, including `Off` |
| 5 | Sixth Plot | `display.plot_channels[5]` | text | Non-empty | Existing labels, including `Off` |
| 6 | Seventh Plot | `display.plot_channels[6]` | text | Non-empty | Existing labels, including `Off` |
| 7 | Eighth Plot | `display.plot_channels[7]` | text | Non-empty | Existing labels, including `Off` |
| 8 | Ninth Plot | `display.plot_channels[8]` | text | Non-empty | Existing labels, including `Off` |
| 9 | Tenth Plot | `display.plot_channels[9]` | text | Non-empty | Existing labels, including `Off` |

Plotting behavior is unchanged.

## Geometry section

| Legacy position | Legacy name | Canonical field | Unit | Validation | Notes |
|---:|---|---|---|---|---|
| 0 | Ogive nose toggle | `vehicle_geometry.ogive_nose` | boolean | `True`/`False` | — |
| 1 | Stabilization fins enabled | `vehicle_geometry.stabilization_fins_enabled` | boolean | `True`/`False` | — |
| 2 | Stabilization fins attached | `vehicle_geometry.stabilization_fins_attached` | boolean | `True`/`False` | — |
| 3 | Control fins enabled | `vehicle_geometry.control_fins_enabled` | boolean | `True`/`False` | — |
| 4 | Control fins attached | `vehicle_geometry.control_fins_attached` | boolean | `True`/`False` | — |
| 5…before `Fins_s` | Body `axial_position,diameter` pairs | `vehicle_geometry.body_stations[]` | `m,m` | Both nonnegative | Variable-length body list |
| marker | `Fins_s` | Structural delimiter | none | Required | Starts stabilization-fin block |
| marker + 1 | Stabilization root pair | `vehicle_geometry.stabilization_fin.root_*` | `m,m` | Both nonnegative | Leading position, chord |
| marker + 2 | Stabilization tip pair | `vehicle_geometry.stabilization_fin.tip_*` | `m,m` | Both nonnegative | Leading position, chord |
| marker + 3 | Stabilization span | `vehicle_geometry.stabilization_fin.span` | `m` | Nonnegative | — |
| marker + 4 | Stabilization thickness | `vehicle_geometry.stabilization_fin.thickness` | `m` | Nonnegative | — |
| marker | `Fins_c` | Structural delimiter | none | Required | Starts control-fin block |
| marker + 1 | Control root pair | `vehicle_geometry.control_fin.root_*` | `m,m` | Both nonnegative | Leading position, chord |
| marker + 2 | Control tip pair | `vehicle_geometry.control_fin.tip_*` | `m,m` | Both nonnegative | Leading position, chord |
| marker + 3 | Control span | `vehicle_geometry.control_fin.span` | `m` | Nonnegative | — |
| marker + 4 | Control thickness | `vehicle_geometry.control_fin.thickness` | `m` | Nonnegative | — |

Extra fixed-section positions and extra fin-block entries are retained in `unsupported_data`. Missing `Fins_s` or `Fins_c` is a structural error because their absence makes subsequent positional interpretation ambiguous.
