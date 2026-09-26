# Parachute Implementation Plan
## Auto-deploy on Motor 2 tilt-over condition — physics + 3D animation

---

## 1. What Is Already Done (Do Not Redo)

The previous session already made these changes. Verify each before you start:

| File | What was done |
|------|--------------|
| `src/gui/gui_setup.py` | `"Parachute Area [m^2] = "` added as last entry in `names_entry` list (line ~232) |
| `src/files.py` | `"Parachute Area = "` added to `parameter_names` (line ~210); `"0.5"` added as last default in `create_file()` (line ~459) |
| `src/simulation/main_simulation.py` | Module-level globals declared: `PARACHUTE_CD = 1.5`, `parachute_area = 0.5`, `parachute_deployed = False`, `parachute_deploy_time = 0.0` (lines 76–79) |
| `src/simulation/main_simulation.py` | `parachute_3d = [False]` declared with other 3D lists (line ~231) |
| `src/simulation/main_simulation.py` | `parachute_area, parachute_deployed, parachute_deploy_time` in `global` line of `update_all_parameters()` (line ~297) |
| `src/simulation/main_simulation.py` | `parachute_area = _get_param(21, 0.5)` in the old-format (21-param) branch (line ~339) |
| `src/simulation/main_simulation.py` | `parachute_area = _get_param(27, 0.5)` in the new-format (27-param) else branch (line ~365) |
| `src/simulation/main_simulation.py` | `parachute_deployed = False`, `parachute_deploy_time = 0.0` in `reset_variables()` (lines ~623–625) |
| `src/simulation/main_simulation.py` | `parachute_3d = [False]` reset in `reset_variables()` (line ~550) |

---

## 2. What Still Needs to Be Done

Everything below is **not yet implemented**. Do it in this exact order.

---

### STEP 1 — Parachute deployment trigger in `update_parameters()`

**File:** `src/simulation/main_simulation.py`

**Where:** Inside `update_parameters()`, immediately before the existing `if should_ignite:` block (which calls `rocket.activate_motor2(t)`). The full trigger block currently ends:

```python
        if should_ignite:
            if rocket.activate_motor2(t):
                motor2_ignited = True
```

**Replace that entire `if should_ignite:` block** with this:

```python
        if should_ignite:
            # === PARACHUTE SAFETY CHECK ===
            # Before igniting motor 2, check if the rocket is tilted
            # beyond what TVC can correct. If so, deploy parachute instead.
            global parachute_deployed, parachute_deploy_time
            tilt_angle = abs(theta)   # theta=0 means perfectly vertical
            if not parachute_deployed and tilt_angle > Actuator_max:
                parachute_deployed = True
                parachute_deploy_time = t
                print(f"PARACHUTE DEPLOYED at t={t:.3f}s  |  tilt={tilt_angle*RAD2DEG:.1f}°  >  TVC_max={Actuator_max*RAD2DEG:.1f}°")
                # Do NOT ignite motor 2
            else:
                if rocket.activate_motor2(t):
                    motor2_ignited = True
```

**Why `abs(theta)` and `Actuator_max`:**
- `theta` is the pitch angle in radians from vertical. Zero = straight up.
- `Actuator_max` is already read from the GUI in radians (line ~329 or ~355).
- If the rocket is tilted more than the TVC can deflect the nozzle, motor 2 thrust cannot be steered to correct it, so deployment is the safe option.
- Using `Actuator_max` as the threshold means this automatically updates if the user changes the max actuator angle in the GUI — no hardcoding needed.

---

### STEP 2 — Parachute physics inside `simulation()`

**File:** `src/simulation/main_simulation.py`

**Where:** Inside `simulation()`, after `update_parameters()` is called and before the physics `if/else` block. The current structure is:

```python
    actuator_angle = (servo_current_angle/Actuator_reduction) + u_initial_offset
    update_parameters()
    ...
    v_d = 0

    if rocket.is_in_the_pad(position_global[0]) and thrust < m*g:
        accx = 0
        ...
    else:
        ...
        accQ = (Q_moment/Iy) * launchrod_lock
```

**Replace the entire `if rocket.is_in_the_pad ... else: ... accQ = ...` block** with:

```python
    v_d = 0

    # === PARACHUTE PHYSICS OVERRIDE ===
    # When parachute is deployed, ignore all rocket aerodynamics/thrust.
    # Model: pure 1D vertical drag + gravity, clamped to terminal velocity.
    # Horizontal velocity bleeds off via lateral drag on the same chute area.
    global parachute_deployed, parachute_area, PARACHUTE_CD
    if parachute_deployed:
        rho_chute = rocket.rho          # ISA density already computed in update_parameters
        # Vertical: gravity down, drag up (opposing vertical velocity)
        v_vertical = v_glob[0]          # positive = upward
        drag_vertical = (0.5 * rho_chute * PARACHUTE_CD * parachute_area
                         * v_vertical * abs(v_vertical))
        # Horizontal: drag opposing lateral drift
        v_lateral = v_glob[1]           # positive = rightward
        drag_lateral = (0.5 * rho_chute * PARACHUTE_CD * parachute_area
                         * v_lateral * abs(v_lateral))
        accx = (-g - drag_vertical / m)   # local X = global vertical (theta=0)
        accz = (-drag_lateral / m)        # lateral deceleration
        accQ = 0.0                        # no pitch moment — chute stabilises attitude
        force_app_point = 0
        normal_force = 0
        # Lock attitude upright (chute hangs straight down)
        # We do not zero theta instantly — let it slowly return to vertical
        # by zeroing angular rate. accQ=0 means it coasts at current rate
        # but gravity will pull CG down and attitude will naturalise.
    elif rocket.is_in_the_pad(position_global[0]) and thrust < m*g:
        accx = 0
        accz = 0
        accQ = 0
        force_app_point = 0
        normal_force = 0
    else:
        launchrod_global_coor = loc2glob(launchrod_lenght, 0, launchrod_angle)
        if position_global[0] <= launchrod_global_coor[0]:
            launchrod_lock = 0
        else:
            launchrod_lock = 1
        if rocket.use_fins_control is False:
            motor_angle = actuator_angle + motor_offset
        else:
            motor_angle = motor_offset
            fin_force = q * S * rocket.fin_cn[1]
        x_force = thrust * np.cos(motor_angle) - q*S*ca + m*g_loc[0]
        z_force = thrust * np.sin(motor_angle) + m*g_loc[1] + q*S*cn
        Q_moment = (thrust * np.sin(motor_angle) * (xt-xcg) + S*q*d*cm_xcg)
        accx = x_force/m - W*Q*v_d
        accz = (z_force/m + U*Q*v_d) * launchrod_lock
        accQ = (Q_moment/Iy) * launchrod_lock
        normal_force = z_force - m*g_loc[1]
        force_app_point = Q_moment / normal_force + xcg
        force_app_point = saturate_plot_xa_force_app(force_app_point)
```

**Physics notes:**
- `drag = 0.5 * rho * Cd * A * v * |v|` — the `v * |v|` form preserves the sign (drag always opposes velocity direction) without an `if` statement.
- `accx` maps to the global vertical axis because the existing code uses `glob2loc/loc2glob` transformations. When the parachute is deployed, we work directly in global coordinates — `accx` is vertical (up/down), `accz` is lateral.
- `m` is already updated by `update_parameters()` so it correctly reflects the current rocket + casing mass.
- `rocket.rho` is already set by the ISA calculator inside `calculate_aero_coef()` which runs in `update_parameters()`.

---

### STEP 3 — Record `parachute_3d` in the 3D data buffer inside `simulation()`

**File:** `src/simulation/main_simulation.py`

**Where:** Inside `simulation()`, in the `if t >= t_timer_3d + 0.00499:` block at the end of the function. Currently that block ends with:

```python
        setpoint_3d.append(setpoint)
        t_timer_3d = t
```

**Add one line before `t_timer_3d = t`:**

```python
        setpoint_3d.append(setpoint)
        parachute_3d.append(parachute_deployed)   # ADD THIS LINE
        t_timer_3d = t
```

This stamps every 3D frame with whether the parachute was deployed at that moment, so the 3D replay can show/hide the canopy correctly.

---

### STEP 4 — Add parachute deployment marker to `plot_plots()`

**File:** `src/simulation/main_simulation.py`

**Where:** In `plot_plots()`, after the existing Motor 2 vertical lines block. Currently the function ends the first plot section with:

```python
    if rocket.motor2_active and rocket.t_launch2 is not None:
        plt.axvline(x=rocket.t_launch2, color="green", ...)
        plt.axvline(x=rocket.t_launch2 + rocket.t_burnout2, color="blue", ...)
```

**After both of those `if` blocks (for both Figure 1 and Figure 2), add:**

```python
    if parachute_deployed:
        plt.axvline(x=parachute_deploy_time, color="orange", linewidth=2,
                    linestyle="-.", label="Parachute Deployed")
```

Do this twice — once under the Figure 1 block, once under the Figure 2 block, matching the pattern of the existing Motor 2 lines.

---

### STEP 5 — Create the 3D parachute object in `run_3d()`

**File:** `src/simulation/main_simulation.py`

**Where:** Inside `run_3d()`, after the `cg_ball` and `velocity_arrow` objects are created and before the `"""buttons & Sliders"""` comment block. The landmark is:

```python
        cg_ball.visilbe = not hide_cg    # (note: this has a typo in the original — keep it)

        """Activate for the velocity arrow to vary with time"""
        variable_length_velocity_arrow = False
        ...
        velocity_arrow = vp.arrow(...)
```

**After `velocity_arrow = vp.arrow(...)`, add:**

```python
        # === PARACHUTE 3D OBJECT ===
        # A flat disc (cylinder with tiny height) above the rocket nose.
        # Visible only when parachute_3d[i] is True during playback.
        chute_radius = np.sqrt(parachute_area / np.pi)   # physical radius from area
        chute_visual_scale = max(chute_radius, d * 3)     # at least 3x body diameter so it's visible
        parachute_canopy = vp.cylinder(
            pos=vp.vector(dim_x_floor/2,
                          L_total + chute_visual_scale * 1.5,   # above the nose tip
                          dim_z_floor/2),
            axis=vp.vector(0, 0.05, 0),          # very flat disc
            radius=chute_visual_scale,
            color=vp.color.orange,
            opacity=0.75,
            visible=False
        )
        # Thin cord connecting nose to canopy centre
        parachute_cord = vp.cylinder(
            pos=vp.vector(dim_x_floor/2,
                          L_total,                           # starts at nose tip
                          dim_z_floor/2),
            axis=vp.vector(0, chute_visual_scale * 1.5, 0),
            radius=d * 0.03,
            color=vp.color.gray(0.3),
            visible=False
        )
```

**Note on `parachute_area`:** `parachute_area` is a module-level global already read from the GUI. At the time `run_3d()` executes it already has its correct value, so `np.sqrt(parachute_area / np.pi)` gives the physical canopy radius.

---

### STEP 6 — Move and show/hide the parachute inside `run_3d_graphics(i, j)`

**File:** `src/simulation/main_simulation.py`

**Where:** Inside `run_3d_graphics(i, j)` (the inner function of `run_3d()`), at the very end, just before the `if hide_forces is True:` block.

**Add:**

```python
            # === PARACHUTE 3D UPDATE ===
            chute_is_open = parachute_3d[i] if i < len(parachute_3d) else False
            parachute_canopy.visible = chute_is_open
            parachute_cord.visible = chute_is_open
            if chute_is_open:
                # The canopy tracks the rocket nose position.
                # rocket_3d.pos is the centroid of the compound body.
                # The nose tip is L_total/2 above the centroid (in local Y),
                # but the centroid moves with the rocket so we just use
                # the current rocket_3d.pos.y (already updated above).
                nose_y = rocket_3d.pos.y + (L_total/2 - xcg_3d[i])    # approximate nose in world Y
                nose_x = rocket_3d.pos.x
                parachute_canopy.pos = vp.vector(nose_x,
                                                 nose_y + chute_visual_scale * 1.5,
                                                 dim_z_floor/2)
                parachute_cord.pos = vp.vector(nose_x, nose_y, dim_z_floor/2)
```

**Why this works:** After deployment, `theta` coasts at whatever angle it was, but `position_3d[i]` continues to be updated by the parachute physics in Step 2, so `rocket_3d.pos` already moves correctly. We just need to pin the canopy above the nose at each frame.

---

### STEP 7 — `run_sim_local()` — add parachute break condition

**File:** `src/simulation/main_simulation.py`

**Where:** In `run_sim_local()`, after the existing `if position_global[0] < -0.55:` landing/crash check. Currently:

```python
        if position_global[0] < -0.55:
            ...
            break
        if t >= sim_duration:
            ...
```

**Add between those two checks:**

```python
        if parachute_deployed and position_global[0] < -0.1:
            progress_bar.update(t, t, 0)
            print("\nParachute Landing — simulation complete.")
            break
```

This prevents the simulation running forever after parachute descent reaches ground — it catches the rocket at a slightly higher threshold (`-0.1 m`) than the crash check (`-0.55 m`) so you get a clean "soft landing" message.

Do the same inside `run_sim_python_sitl()` immediately before its `if parachute == 1:` check (line ~1365).

---

### STEP 8 — Declare `parachute_canopy` and `parachute_cord` in `run_3d_graphics` closure scope

**Why this is needed:** `parachute_canopy` and `parachute_cord` are created in the `run_3d()` body but referenced in the nested `run_3d_graphics(i, j)` function. Python closures capture outer variables by reference, so as long as they are assigned **before** `run_3d_graphics` is defined (or before it is called), they are automatically in scope. No `nonlocal` keyword is needed because you are only reading and mutating the objects (`.visible`, `.pos`) — not reassigning the variable names themselves.

To be safe, make sure `parachute_canopy` and `parachute_cord` are created in Step 5 **before** the `"""buttons & Sliders"""` block, which is before `run_3d_graphics` is defined. The existing structure already follows this pattern (e.g. `cg_ball` is created before `run_3d_graphics` and used freely inside it).

Also add `chute_visual_scale` as a local variable at the same point so the `run_3d_graphics` closure can read it.

---

## 3. Complete Execution Order Summary

```
Step 1  update_parameters()     — tilt check → deploy or ignite
Step 2  simulation()            — parachute physics branch (replaces normal forces)
Step 3  simulation()            — record parachute_3d per 3D frame
Step 4  plot_plots()            — orange vertical line at deploy time
Step 5  run_3d()                — create canopy + cord VPython objects
Step 6  run_3d_graphics(i,j)    — move + show/hide canopy per frame
Step 7  run_sim_local()         — break on parachute landing
Step 8  (no code change)        — closure scope note; verify ordering
```

---

## 4. Variable Cross-Reference

| Variable | Declared | Set | Read |
|----------|----------|-----|------|
| `PARACHUTE_CD` | module level (line 76) | fixed = 1.5 | Step 2 |
| `parachute_area` | module level (line 77) | `update_all_parameters()` from GUI | Step 2, Step 5 |
| `parachute_deployed` | module level (line 78) | Step 1 (trigger), `reset_variables()` | Step 2, Step 4, Step 7 |
| `parachute_deploy_time` | module level (line 79) | Step 1 (trigger), `reset_variables()` | Step 4 |
| `parachute_3d` | module level (line 231) | Step 3 per frame, `reset_variables()` | Step 6 |
| `Actuator_max` | `update_all_parameters()` | from GUI (rad) | Step 1 (comparison) |
| `theta` | `simulation()` integrator | continuous | Step 1 (tilt check) |
| `rocket.rho` | `calculate_aero_coef()` | ISA model | Step 2 |
| `m` | `update_parameters()` | `get_mass_parameters()` | Step 2 |

---

## 5. Behaviour Summary

| Condition at Motor 2 trigger time | Result |
|-----------------------------------|--------|
| `abs(theta) <= Actuator_max` | Motor 2 ignites normally. No parachute. |
| `abs(theta) > Actuator_max` | Motor 2 is **not** ignited. Parachute deploys immediately. |
| Parachute deployed, descending | Physics: `a_vertical = -g - (drag/m)`, converging to ~4 m/s terminal. |
| Rocket reaches ground with chute | "Parachute Landing" message. Simulation ends cleanly. |
| 3D animation | Orange disc + grey cord appear above nose at deploy frame, track rocket to ground. |
| Plot | Orange dash-dot vertical line at `parachute_deploy_time`. |

---

## 6. Things You Do NOT Need to Change

- `rocket_functions.py` — no changes. The parachute is entirely a sim-layer concern.
- `control.py` — the PID controller still runs after deployment but `u_servos` has no effect because `simulation()` ignores the actuator angle entirely when `parachute_deployed` is True.
- `files.py` and `gui_setup.py` — already complete from the previous session.
- `servo_lib.py` — unchanged.
- SITL interface — the Arduino SITL already has `parachute == 1` as a break condition (line ~1256); you don't need to touch it.
