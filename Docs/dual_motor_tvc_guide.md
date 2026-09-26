# Dual-Motor TVC Simulation: User Guide & Best Practices

## 1. Summary of Changes Done in the Codebase

The codebase has been upgraded on branch `multimotor-codebase` from single-motor simulation to a full **multi-motor / dual-burn TVC simulation platform**.

### Summary of Modified Files

| File | Changes Made |
| :--- | :--- |
| **`src/aerodynamics/rocket_functions.py`** | Added `Rocket.set_motor2()`, `Rocket.activate_motor2()`, `Rocket.burnout_time_total()`. Replaced `get_thrust()` to sum independent thrust curves $T_1(t - t_{\text{launch1}}) + T_2(t - t_{\text{launch2}})$. Implemented a continuous 4-phase mass, moment of inertia ($I_y$), and center of gravity ($x_{cg}$) depletion engine. |
| **`src/files.py`** | Added `get_motor2_names()`, `SaveFile.read_motor2_data()`, `SaveFile.get_motor2_data()`. Extended save file schema with Motor 2 parameters while maintaining backward compatibility with legacy 21-parameter saves. |
| **`src/gui/gui_setup.py`** | Updated the **Parameters** tab to add Motor 2 selection combobox, Trigger Mode selector, and entry fields for Motor 2 burnout mass, $I_y$, $x_{cg}$, and trigger value. |
| **`src/simulation/main_simulation.py`** | Added apogee detection ($v_{\text{glob}}[0]$ zero-crossing) and 4 ignition trigger modes. Added multi-motor event markers to diagnostic plots (Motor 1 Burnout, Apogee, Motor 2 Ignition, Motor 2 Burnout). Updated 3D animation to render exhaust plume during both active motor burn windows. |
| **`3 - Examples/Example Python SITL/SITL Modules/DHRUVA_AeroVECTOR_SITL.py`** | Added `STATE_SECOND_BURN` state to DHRUVA's flight state machine, allowing active TVC stabilization during both ascent and secondary apogee/descent burns. |

---

## 2. How to Use the Dual-Motor TVC Feature

### A. Selecting Motors
1. Place any standard OpenMotor/ThrustCurve `.csv` files into the `Motors/` directory.
2. In the **Parameters** tab in AeroVECTOR:
   * **Motor 1**: Select your booster / ascent motor (e.g. `our_tvc_motor_neeche_wala.csv` or `Estes_D12.csv`).
   * **Motor 2**: Select your sustainer / secondary motor from the dropdown (or select `None` to run in single-motor mode).

### B. Configuring Trigger Modes

| Trigger Mode | Description | Example Trigger Value |
| :--- | :--- | :--- |
| **`Disabled`** / **`None`** | Motor 2 will not ignite. Vehicle operates as a single-motor rocket. | `0` |
| **`Distance After Apogee [m]`** | Apogee is automatically detected when vertical velocity crosses zero ($v_z < 0$). Motor 2 ignites once the rocket falls by the specified distance $\Delta h$. | `10.0` (ignites $10\text{ m}$ below apogee) |
| **`Altitude Threshold [m]`** | Motor 2 ignites as soon as the rocket climbs through this altitude threshold during ascent. | `45.0` (ignites when $h \ge 45\text{ m}$) |
| **`Time After Burnout 1 [s]`** | Motor 2 ignites after a specified coast delay following Motor 1 burnout ($t_{\text{ignite2}} = t_{\text{launch}} + t_{\text{burnout1}} + \Delta t$). | `1.5` (ignites $1.5\text{ s}$ after Motor 1 burns out) |
| **`Time After Launch [s]`** | Motor 2 ignites at an absolute mission elapsed time from liftoff ($t_{\text{ignite2}} = t_{\text{launch}} + \Delta t$). | `6.0` (ignites $6.0\text{ s}$ after liftoff) |

### C. Setting Mass & Inertia Properties

For realistic 3-phase physics, fill in the three mass stages in the Parameters tab:
1. **Liftoff State** ($t = t_{\text{launch}}$):
   * `Mass Liftoff [kg]`: Total rocket mass with both motors fully fueled.
   * `Iy Liftoff [kg*m^2]`: Pitch moment of inertia with both motors fully fueled.
   * `Xcg Liftoff [m]`: Center of gravity measured from nose tip with both motors fully fueled.
2. **Motor 1 Burnout State** ($t = t_{\text{burnout1}}$):
   * `Mass Burnout [kg]`: Rocket mass after Motor 1 propellant is spent (Motor 2 still unburnt).
   * `Iy Burnout [kg*m^2]`: Moment of inertia after Motor 1 propellant is spent.
   * `Xcg Burnout [m]`: Center of gravity after Motor 1 propellant is spent.
3. **Motor 2 Burnout State** ($t = t_{\text{burnout2}}$):
   * `Mass Burnout 2 [kg]`: Final dry mass after both motors have burned out.
   * `Iy Burnout 2 [kg*m^2]`: Final moment of inertia after both motors have burned out.
   * `Xcg Burnout 2 [m]`: Final center of gravity after both motors have burned out.

---

## 3. How to Simulate Better 2-Motor TVC

### 1. Static Margin & Dynamic Stability ($CP - CG$)
* As Motor 1 burns, propellant mass at the tail decreases, shifting $x_{cg}$ forward. This increases the static stability margin $(x_{cp} - x_{cg})/d$.
* When Motor 2 ignites during low dynamic pressure ($q = \frac{1}{2}\rho v^2$) near apogee, aerodynamic restoring moments $q S d C_{m\alpha}$ are minimal. 
* **Key takeaway**: Near apogee, almost 100% of attitude control is provided by the TVC gimbal moment $M_{\text{TVC}} = T_2 \cdot \sin(\delta) \cdot (x_t - x_{cg})$. Ensure your TVC actuator limits ($\pm 10^\circ \text{ to } \pm 12^\circ$) and servo speed are sufficient before firing.

### 2. PID Gain Scheduling Across Different Thrust Levels
* The control authority changes with total instantaneous thrust:
  $$\ddot{\theta} = \frac{T(t) \cdot \sin(\delta) \cdot (x_t - x_{cg}) + q S d C_{m\alpha}}{I_y(t)}$$
* If Motor 2 has significantly lower thrust than Motor 1 (e.g. low-thrust sustainer), you may need higher $K_p$ or larger gimbal deflections to produce the same angular acceleration.
* If Motor 2 has a high-thrust spike, decrease derivative gain $K_d$ filtering to avoid servo chatter.

### 3. Avoiding Apogee Inversion During Descent Firing
* If simulating a **powered descent / landing burn** (`Distance After Apogee`), the rocket's tail may be pointing downward while velocity is downward (high angle of attack $\alpha \approx 180^\circ$).
* When Motor 2 fires, the thrust decelerates the vehicle. Ensure the setpoint controller is set to hold vertical pitch ($\theta = 0^\circ$) so the gimbal actively corrects tilt rather than flipping the airframe.

### 4. Running Python SITL with DHRUVA
* To test real-time flight software against the dual-motor model:
  1. Open the **SITL** tab in AeroVECTOR.
  2. Enable `Activate SITL = True` and `Python SITL = True`.
  3. Select `DHRUVA_AeroVECTOR_SITL`.
  4. Run the simulation. The state machine will automatically transition from `ON_PAD` $\to$ `ASCENT` $\to$ `APOGEE` $\to$ `SECOND_BURN` $\to$ `DESCENT` $\to$ `LANDED`.
