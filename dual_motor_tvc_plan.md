# Dual-Motor TVC Simulation — Analysis & Implementation Plan

## 1. Codebase Architecture (What Exists Today)

### File Map
```
AeroVECTOR.py                   ← Entry point (Tkinter GUI)
src/
  simulation/main_simulation.py  ← Simulation engine, integration loop, 3D playback
  aerodynamics/rocket_functions.py ← Rocket class: aero, mass, motor
  gui/gui_setup.py               ← All GUI tabs
  gui/gui_functions.py           ← GUI helper widgets
  files.py                       ← Save/load, motor CSV reading
  control.py                     ← PID controller
  python_sitl_functions.py       ← Python SITL interface
  simulation/servo_lib.py        ← Servo model
  ISA_calculator.py              ← Atmosphere (ISA)
  aerodynamics/fin_aerodynamics.py
Motors/                          ← CSV thrust curves (time, thrust)
3 - Examples/                    ← Python SITL examples (parachute, landing, DHRUVA)
```

### How the Single-Motor Simulation Works (Key Data Flow)

```
1. gui_setup.py reads the GUI → passes one motor filename to files.py
2. files.read_motor_data(name) → loads [t_list, thrust_list] into SaveFile
3. rocket.set_motor(data) → stores as rocket.motor[0]=t, rocket.motor[1]=thrust
4. t_burnout = rocket.burnout_time() = motor[0][-1]

Per timestep in main_simulation.simulation():
  thrust = rocket.get_thrust(t, t_launch)        # np.interp on the single curve
  m, Iy, xcg = rocket.get_mass_parameters(t, t_launch)  # linear interp liftoff→burnout
  x_force  = thrust * cos(motor_angle) - drag + gravity_x
  z_force  = thrust * sin(motor_angle) + gravity_z + aero_normal
  Q_moment = thrust * sin(motor_angle) * (xt - xcg) + aero_moment
```

### What the Mass Model Does
`get_mass()`, `get_Iy()`, `get_xcg()` all linearly interpolate between two endpoints:
- t=0 (liftoff): `m_liftoff`, `Iy_liftoff`, `xcg_liftoff`
- t=t_burnout: `m_burnout`, `Iy_burnout`, `xcg_burnout`

This is fine for one motor but needs a 3-phase model for two.

---

## 2. What Changes Are Needed (Complete List)

### A. Motor 2 — Trigger Condition
Motor 2 can ignite on one of two conditions (both worth supporting):
- **Altitude-based**: ignite when `position_global[0]` reaches a user-set altitude below apogee (e.g., "start 50 m before apogee").
- **Time-based**: ignite at a fixed time offset after motor 1 burns out.

The user said "some distance from apogee," so altitude-based is primary. Apogee detection requires tracking when vertical velocity (`v_glob[0]`) crosses zero (starts going negative).

### B. Changes to `rocket_functions.py` — `Rocket` class

**Add second motor storage:**
```python
self.motor2 = [[], []]
self.t_burnout2 = 0
self.motor2_active = False
self.t_launch2 = None        # set at ignition time
self.m2_prop = 0             # propellant mass of motor 2
```

**Add `set_motor2(data, prop_mass)`** — mirrors `set_motor()`.

**Replace `get_thrust()` to sum both motors:**
```python
def get_thrust(self, t, t_launch):
    thrust1 = np.interp(t - t_launch, self.motor[0], self.motor[1])
    thrust2 = 0.0
    if self.motor2_active and self.t_launch2 is not None:
        dt = t - self.t_launch2
        if 0 <= dt <= self.t_burnout2:
            thrust2 = np.interp(dt, self.motor2[0], self.motor2[1])
    self.thrust = max(thrust1 + thrust2, 0.001)
    return float(self.thrust)
```

**Replace mass model — 3-phase linear interpolation:**

| Phase | Start | End | Description |
|-------|-------|-----|-------------|
| Phase 1 | t=t_launch | t=t_burnout1 | Motor 1 burns; mass drops from liftoff to burnout1 |
| Phase 2 | t=t_burnout1 | t=t_launch2 | Coasting; mass = m_burnout1 (no propellant flow) |
| Phase 3 | t=t_launch2 | t=t_burnout2 | Motor 2 burns; mass drops by m2_prop |

```python
def get_mass(self, t, t_launch):
    dt1 = t - t_launch
    m = np.interp(dt1, [0, self.t_burnout], [self.m_liftoff, self.m_burnout])
    if self.motor2_active and self.t_launch2 is not None:
        dt2 = t - self.t_launch2
        if dt2 > 0:
            m2_start = self.m_burnout
            m2_end   = self.m_burnout - self.m2_prop
            m -= np.interp(dt2, [0, self.t_burnout2], [0, self.m2_prop])
            m = max(m, m2_end)
    return float(m)
```

`get_Iy()` and `get_xcg()` need the same 3-phase treatment. The CG will shift when motor 2 burns since propellant is consumed from a different location (motor 2 is at the base, same as motor 1, but their propellant masses are separate).

**Add `activate_motor2(t_now)`:**
```python
def activate_motor2(self, t_now):
    if not self.motor2_active:
        self.motor2_active = True
        self.t_launch2 = t_now
        print(f"Motor 2 ignited at t={t_now:.3f}s")
```

**Add `burnout_time_total()`** for simulation end logic:
```python
def burnout_time_total(self):
    if self.motor2_active and self.t_launch2 is not None:
        return self.t_launch2 + self.t_burnout2
    return self.t_burnout
```

---

### C. Changes to `main_simulation.py`

**New global parameters:**
```python
motor2_trigger_type = "altitude"   # "altitude" or "time"
motor2_trigger_value = 0.0         # metres below apogee OR seconds after burnout1
motor2_ignited = False
apogee_detected = False
apogee_altitude = 0.0
v_glob_prev = [0, 0]               # for apogee detection
```

**In `update_all_parameters()`** — read Motor 2 settings from GUI and load its CSV:
```python
# Motor 2
motor2_name = conf_controller[<new_index_motor2_name>]
motor2_trigger_type  = conf_controller[<new_index_trigger_type>]
motor2_trigger_value = conf_controller[<new_index_trigger_value>]
motor2_prop_mass     = conf_controller[<new_index_prop_mass>]

gui.savefile.read_motor2_data(motor2_name)          # new method in files.py
rocket.set_motor2(gui.savefile.get_motor2_data(),   # new method in Rocket
                  motor2_prop_mass)
```

**In `update_parameters()`** — add apogee detection + motor 2 ignition logic:
```python
# Apogee detection
global apogee_detected, apogee_altitude, v_glob_prev, motor2_ignited

if not apogee_detected and v_glob[0] < 0 and v_glob_prev[0] >= 0 and t > t_launch + 0.5:
    apogee_detected = True
    apogee_altitude = position_global[0]
    print(f"Apogee detected at {apogee_altitude:.1f} m, t={t:.2f}s")

# Motor 2 ignition check
if not motor2_ignited:
    if motor2_trigger_type == "altitude" and apogee_detected:
        drop = apogee_altitude - position_global[0]
        if drop >= motor2_trigger_value:
            rocket.activate_motor2(t)
            motor2_ignited = True
    elif motor2_trigger_type == "time":
        if t >= t_launch + rocket.t_burnout + motor2_trigger_value:
            rocket.activate_motor2(t)
            motor2_ignited = True

v_glob_prev = list(v_glob)
```

**In `simulation()`** — the physics equations are unchanged. `thrust` already comes from `rocket.get_thrust()` which will now sum both motors automatically. The TVC deflection (`motor_angle`) steers the net thrust vector — this models a single TVC mount that gimbles both motors together (which is the simplest and most common case for a co-axial two-motor stack).

**In `reset_variables()`** — reset the new flags:
```python
motor2_ignited = False
apogee_detected = False
apogee_altitude = 0.0
v_glob_prev = [0, 0]
```

**Plot updates** — add motor 2 burnout line to `plot_plots()`:
```python
if rocket.motor2_active and rocket.t_launch2 is not None:
    plt.axvline(x=rocket.t_launch2 + rocket.t_burnout2, color="blue", linewidth=1,
                linestyle="--", label="Motor 2 burnout")
    plt.axvline(x=rocket.t_launch2, color="green", linewidth=1,
                linestyle="--", label="Motor 2 ignition")
```

---

### D. Changes to `files.py`

Add `read_motor2_data()` and `get_motor2_data()` — exact clones of the existing motor methods but storing into `self.t_mot2` and `self.thrust_mot2`. The CSV format is unchanged (same `time, thrust` structure as existing files in `Motors/`).

---

### E. Changes to `gui_setup.py` and `gui_functions.py`

**In the Parameters tab** — add:
- Dropdown (combobox) for Motor 2 CSV file (same list as Motor 1 dropdown).
- Dropdown for trigger type: `["altitude_below_apogee", "time_after_burnout1"]`.
- Entry field for trigger value (metres or seconds).
- Entry field for Motor 2 propellant mass (kg) — needed for the mass model.
- Entry fields for Motor 2 mass/Iy/CG parameters at ignition and burnout:
  - `m2_at_ignition`, `m2_at_burnout` (or just `m2_prop_mass` if the main mass model handles it).
  - `Iy2_liftoff`, `Iy2_burnout` (moment of inertia contribution changes as motor 2 burns).
  - `xcg2_shift_liftoff`, `xcg2_shift_burnout` (how CG shifts during motor 2 burn).

For the **minimum viable version**, you only need:
1. Motor 2 CSV filename
2. Trigger type + value
3. Motor 2 propellant mass

The Iy and CG shifts during motor 2 can be approximated by extending the `xcg_burnout` interpolation linearly — this is acceptable for initial simulations.

**In `files.py` — savefile** — add the new fields to `parameter_names`, default values in `create_file()`, and read/write in `read_file()` / `save_all_configurations()`.

---

### F. Motor 2 CSV File Format

Same as existing files — no changes needed:
```
Time(s),Thrust(N)
0,0
0.03,69.16
...
0.51,0
```

Place it in the `Motors/` folder like any other motor. The GUI dropdown will pick it up automatically.

---

## 3. TVC Behaviour After Motor 2 Ignites

This is determined entirely by the thrust curve, mass model, and the existing physics — **no new physics code is needed**. Here is what will happen:

| Scenario | What the simulation shows |
|----------|--------------------------|
| Motor 2 thrust > aerodynamic drag + gravity component | Rocket decelerates less or re-accelerates |
| Motor 2 CG shift is significant | Moment arm `(xt - xcg)` changes → TVC authority changes |
| Motor 2 has a high thrust spike | Large moment → PID/servo may saturate briefly |
| Rocket is already pitched off-axis at ignition | Motor 2 thrust acts along current body axis, producing lateral force |
| TVC is active during motor 2 burn | Controller steers using combined thrust from both motors |

The controller (`control.py`) uses `thrust` as a gain-scheduling input in `control_theta()`. With two motors active, the total thrust is automatically higher, so the TVC produces larger moments — the PID gains may need retuning for the second burn phase.

---

## 4. Implementation Order (Step-by-Step)

### Step 1 — Motor 2 CSV
Create or copy a motor CSV for the second motor into `Motors/`. The format is identical to `our_tvc_motor_neeche_wala.csv`.

### Step 2 — `rocket_functions.py`
- Add `motor2` storage and `m2_prop` attribute.
- Add `set_motor2()`, `activate_motor2()`.
- Modify `get_thrust()` to sum both motors.
- Modify `get_mass()`, `get_Iy()`, `get_xcg()` to handle 3-phase interpolation.
- Add `burnout_time_total()`.

### Step 3 — `files.py`
- Add `read_motor2_data()` and `get_motor2_data()`.
- Extend savefile parameter list with 4 new fields: motor2 name, trigger type, trigger value, motor2 prop mass.

### Step 4 — `main_simulation.py`
- Add new globals: `motor2_trigger_type`, `motor2_trigger_value`, `motor2_ignited`, `apogee_detected`, `apogee_altitude`, `v_glob_prev`.
- Extend `update_all_parameters()` to read and load motor 2.
- Add apogee detection + ignition logic in `update_parameters()`.
- Extend `reset_variables()`.
- Add ignition/burnout vertical lines to `plot_plots()`.

### Step 5 — `gui_setup.py` / `gui_functions.py`
- Add Motor 2 controls to the Parameters tab.
- Ensure new fields are wired to the savefile read/write path.

### Step 6 — Test
- Motor 1 only: verify existing behaviour unchanged.
- Motor 2 triggers at altitude: check console output for "Motor 2 ignited" message and plot thrust vs time.
- Motor 2 with TVC: verify controller keeps rocket pitched toward setpoint during second burn.

---

## 5. What Does NOT Change

- The aerodynamics (`calculate_aero_coef`) — unchanged, it sees the rocket as one body.
- The PID controller structure — unchanged (it just receives a different total thrust).
- The integration loop (trapezoidal) — unchanged.
- The 3D viewer — no second motor flame/trail, but the rocket body and TVC mount animation remain correct since they depend on `thrust`, not on which motor produced it. (Optionally: add a second red cone for motor 2's visual.)
- The SITL interface — unchanged; Python SITL programs can trigger motor 2 via `Sim.getSimData()` altitude and call `Sim.sendCommand()` as usual.
- Motor file format — unchanged.

---

## 6. Assumptions to Document for the User

1. **Single TVC mount**: both motors are gimballed together by the same servo. If your rocket has motor 2 on a separate independent mount, you need a second `actuator_angle` variable and a second servo model.
2. **Co-axial motors**: both motors push along the same body axis. If motor 2 is offset (e.g., side-mounted), add a moment arm offset to the `Q_moment` calculation.
3. **Instantaneous ignition**: the simulation fires motor 2 the moment the trigger condition is met, with no ignition delay. Add a delay variable (`motor2_ignition_delay`) if needed.
4. **CG model**: the 3-phase linear interpolation is an approximation. If you have a more detailed CG vs time curve, replace `np.interp` with a full interpolation array.
5. **Subsonic only**: the existing Mach < 0.9 limit still applies across both burn phases.
