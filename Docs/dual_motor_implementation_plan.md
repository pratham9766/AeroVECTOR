# Dual-Motor TVC Simulation Architecture & Implementation Plan

## 1. Overview & System Objectives

The Dual-Motor TVC Simulation enables full aerodynamic and TVC modeling of a multi-stage or dual-pulse solid rocket vehicle in AeroVECTOR.
- **Motor 1 (Booster)**: Ignites at liftoff ($t = t_{\text{launch}}$) to propel the rocket through the initial ascent phase.
- **Coast Phase**: Unpowered ballistic coast where mass, moment of inertia ($I_y$), and center of gravity ($x_{cg}$) remain fixed at Motor 1 burnout values while aerodynamic drag and gravity act on the airframe.
- **Motor 2 (Sustainer / Apogee Pulse)**: Ignites based on a configurable trigger condition:
  1. `Altitude Threshold [m]` ($h \ge h_{\text{trigger}}$ during ascent)
  2. `Distance After Apogee [m]` ($h_{\text{apogee}} - h \ge \Delta h_{\text{drop}}$ during descent)
  3. `Time After Burnout 1 [s]` ($t \ge t_{\text{launch}} + t_{\text{burnout1}} + \Delta t$)
  4. `Time After Launch [s]` ($t \ge t_{\text{launch}} + \Delta t$)
- **TVC Control & Multi-Motor Physics**:
  - Independent thrust curve interpolation for both motors: $T_{\text{net}}(t) = T_1(t - t_{\text{launch1}}) + T_2(t - t_{\text{launch2}})$.
  - 3-phase mass, CG, and inertia depletion.
  - Active TVC gimbal torque calculation $M_{\text{TVC}} = T_{\text{net}} \cdot \sin(\delta) \cdot (x_t - x_{cg})$.

---

## 2. Flight Dynamics & Multi-Phase Depletion

### Mass, CG ($x_{cg}$), and Inertia ($I_y$) Transitions

| Flight Phase | Time Interval | Mass $m(t)$ | Inertia $I_y(t)$ | Center of Gravity $x_{cg}(t)$ |
| :--- | :--- | :--- | :--- | :--- |
| **Phase 1: Motor 1 Burn** | $t_{\text{launch}} \le t \le t_{\text{launch}} + t_{\text{burnout1}}$ | Linear interp: $m_{\text{liftoff}} \to m_{\text{burnout1}}$ | Linear interp: $I_{y,\text{liftoff}} \to I_{y,\text{burnout1}}$ | Linear interp: $x_{cg,\text{liftoff}} \to x_{cg,\text{burnout1}}$ |
| **Phase 2: Coast Phase** | $t_{\text{burnout1}} < t < t_{\text{launch2}}$ | Constant: $m_{\text{burnout1}}$ | Constant: $I_{y,\text{burnout1}}$ | Constant: $x_{cg,\text{burnout1}}$ |
| **Phase 3: Motor 2 Burn** | $t_{\text{launch2}} \le t \le t_{\text{launch2}} + t_{\text{burnout2}}$ | Linear interp: $m_{\text{burnout1}} \to m_{\text{burnout2}}$ | Linear interp: $I_{y,\text{burnout1}} \to I_{y,\text{burnout2}}$ | Linear interp: $x_{cg,\text{burnout1}} \to x_{cg,\text{burnout2}}$ |
| **Phase 4: Final Coast / Descent** | $t > t_{\text{launch2}} + t_{\text{burnout2}}$ | Constant: $m_{\text{burnout2}}$ | Constant: $I_{y,\text{burnout2}}$ | Constant: $x_{cg,\text{burnout2}}$ |

---

## 3. Implementation Blueprint

### A. Rocket Class (`src/aerodynamics/rocket_functions.py`)
- Second motor storage: `self.motor2 = [[], []]`, `self.t_burnout2`, `self.motor2_active`, `self.t_launch2`, `self.m_burnout2`, `self.Iy_burnout2`, `self.xcg_burnout2`.
- `set_motor2(data, m_burnout2, Iy_burnout2, xcg_burnout2)`
- `activate_motor2(t_now)`
- `get_thrust(t, t_launch)` summing instantaneous Motor 1 and Motor 2 curves.
- `get_mass(t, t_launch)`, `get_Iy(t, t_launch)`, `get_xcg(t, t_launch)` executing the 4-phase depletion schedule.
- `burnout_time_total()` computing dynamic total simulation active thrust window.

### B. Storage & SaveFile Engine (`src/files.py`)
- Read and parse `Motor 2` CSV from `Motors/` directory via `read_motor2_data(name)`.
- Support selection of `None` / `Disabled` for single-motor backward compatibility.
- Synchronize parameter lists and save/load serialization for all dual-motor fields.

### C. Simulation Engine (`src/simulation/main_simulation.py`)
- Apogee detection based on zero-crossing vertical velocity: $v_{\text{glob}}[0] < 0$ after launch.
- Continuous trigger check for Motor 2 ignition during `update_parameters()`.
- Dynamic plot markers for Motor 1 ignition/burnout, apogee, and Motor 2 ignition/burnout.
- 3D visualizer plume support across both active motor burn windows.

### D. Graphical User Interface (`src/gui/gui_setup.py`)
- Parameters Tab extension with Motor 2 selection combobox, Trigger Mode combobox, and entry parameters for Motor 2 mass/inertia/CG at burnout and trigger values.
