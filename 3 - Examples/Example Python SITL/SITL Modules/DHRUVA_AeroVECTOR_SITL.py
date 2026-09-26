"""
DHRUVA -> AeroVECTOR Python SITL
--------------------------------

This is the AeroVECTOR-specific SITL port of DHRUVA. It keeps the
important DHRUVA flight-software logic:

- ON_PAD / ASCENT / APOGEE / DESCENT / LANDED state machine
- altitude/vertical-velocity Kalman filter
- pitch PID
- derivative filtering
- conditional integral anti-windup
- +/- 12 deg controller output
- TVC disabled outside ASCENT

AeroVECTOR is a 2-D pitch simulation, so its Python SITL interface
provides one gyro, two accelerometer axes and one pitch actuator.
Therefore the DHRUVA yaw loop, magnetometer, second TVC actuator and
hardware-specific sensor drivers cannot be represented directly.

Mapping:
    BMP388 altitude       -> AeroVECTOR alt
    BNO085 gyro           -> AeroVECTOR gyro
    BNO085 pitch          -> integrated AeroVECTOR gyro
    MPU/BNO acceleration  -> AeroVECTOR accx / accz
    GNSS                  -> available but not required by DHRUVA
    pitch TVC servo       -> AeroVECTOR servo command
    yaw TVC servo         -> not available in AeroVECTOR 2-D model
    parachute             -> optional adapter command

Put this file in:

3 - Examples/Example Python SITL/SITL Modules/

Then set the AeroVECTOR configuration:

    Activate SITL = True
    Python SITL = True
    File = DHRUVA_AeroVECTOR_SITL

"""

from src import python_sitl_functions as Sim
import numpy as np


class SITLProgram:

    def __init__(self):
        # ----------------------------
        # DHRUVA servo limits
        # ----------------------------
        self.servo_home_deg = 0.0
        self.servo_range_deg = 12.0

        # ----------------------------
        # DHRUVA flight-state constants
        # ----------------------------
        self.ASCENT_ALTITUDE_THRESHOLD = 3.0
        self.ASCENT_REQUIRED_READINGS = 5

        self.APOGEE_REQUIRED_READINGS = 5

        self.LANDED_ALTITUDE_BAND = 1.0
        self.LANDED_REQUIRED_READINGS = 20

        # ----------------------------
        # DHRUVA PID gains
        # ----------------------------
        self.pitch_kp = 2.0
        self.pitch_ki = 2.4
        # self.pitch_kd = 5.2
        self.pitch_kd = 0.0
        self.derivative_filter = 0.90

        self.pitch_setpoint_deg = 0.0

        self.output_min = -12.0
        self.output_max = 12.0

        # ----------------------------
        self.STATE_BOOT = 0
        self.STATE_ON_PAD = 1
        self.STATE_ASCENT = 2
        self.STATE_APOGEE = 3
        self.STATE_DESCENT = 4
        self.STATE_LANDED = 5
        self.STATE_SECOND_BURN = 6

        self.flight_state = self.STATE_BOOT

        # ----------------------------
        # Kalman state
        # ----------------------------
        self.kalman_altitude = 0.0
        self.kalman_velocity = 0.0

        self.P00 = 2.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 2.0

        # ----------------------------
        # PID state
        # ----------------------------
        self.integral = 0.0
        self.previous_error = 0.0
        self.filtered_derivative = 0.0

        self.pitch_p = 0.0
        self.pitch_i = 0.0
        self.pitch_d = 0.0
        self.pitch_output = 0.0

        # ----------------------------
        # State-machine counters
        # ----------------------------
        self.ascent_counter = 0
        self.apogee_counter = 0
        self.landed_counter = 0

        self.previous_altitude = 0.0
        self.landing_reference_altitude = 0.0

        # ----------------------------
        # Timing
        # ----------------------------
        self.initialized = False
        self.t_prev = None

        # ----------------------------
        # Sensor values
        # ----------------------------
        self.gyro = 0.0
        self.accx = 0.0
        self.accz = 0.0
        self.alt = 0.0

        self.pos_gnss = 0.0
        self.vel_gnss = 0.0

        self.pitch_deg = 0.0

        # Optional AeroVECTOR parachute adapter.
        # Set False if you want to observe DHRUVA state transitions
        # without deploying AeroVECTOR's simulated parachute.
        self.deploy_parachute_on_descent = False

        print(
            f"DHRUVA TEST CONFIG: Kp={self.pitch_kp}, "
            f"Ki={self.pitch_ki}, Kd={self.pitch_kd}, "
            f"Parachute={self.deploy_parachute_on_descent}"
        )

        # ----------------------------
        # Logging / plots
        # ----------------------------
        self.last_state = self.STATE_BOOT

    # AeroVECTOR's Python SITL loader explicitly calls this method
    # before void_setup(), so all runtime variables that correspond
    # to the original DHRUVA global/static initialization live here.
    def everything_that_is_outside_functions(self):
        # Kept as an explicit initialization hook to match
        # AeroVECTOR's SITLProgram contract.
        pass

    # ============================================================
    # DHRUVA KALMAN FILTER
    # ============================================================

    def kalman_begin(self, initial_altitude):
        self.kalman_altitude = initial_altitude
        self.kalman_velocity = 0.0

        self.P00 = 2.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 2.0

    def kalman_update(self, measured_altitude, dt):

        if dt <= 0:
            return

        dt = min(dt, 0.2)

        # Prediction
        self.kalman_altitude += self.kalman_velocity * dt

        q_altitude = 0.02
        q_velocity = 1.0

        self.P00 = (
            self.P00
            + dt * (self.P10 + self.P01)
            + dt * dt * self.P11
            + q_altitude * dt
        )

        self.P01 = self.P01 + dt * self.P11
        self.P10 = self.P01
        self.P11 = self.P11 + q_velocity * dt

        # Measurement update
        measurement_noise = 0.5

        innovation = measured_altitude - self.kalman_altitude
        S = self.P00 + measurement_noise

        if S <= 0:
            return

        K0 = self.P00 / S
        K1 = self.P10 / S

        self.kalman_altitude += K0 * innovation
        self.kalman_velocity += K1 * innovation

        oldP00 = self.P00
        oldP01 = self.P01

        self.P00 = (1.0 - K0) * oldP00
        self.P01 = (1.0 - K0) * oldP01
        self.P10 = self.P01
        self.P11 = self.P11 - K1 * oldP01

    # ============================================================
    # DHRUVA PID
    # ============================================================

    def pid_reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.filtered_derivative = 0.0

        self.pitch_p = 0.0
        self.pitch_i = 0.0
        self.pitch_d = 0.0
        self.pitch_output = 0.0

    def pid_update(self, setpoint, measured, dt):

        if dt <= 0:
            return self.pitch_output

        error = setpoint - measured

        self.pitch_p = self.pitch_kp * error

        raw_derivative = (
            error - self.previous_error
        ) / dt

        self.filtered_derivative = (
            self.derivative_filter * self.filtered_derivative
            + (1.0 - self.derivative_filter) * raw_derivative
        )

        self.pitch_d = self.pitch_kd * self.filtered_derivative

        candidate_integral = (
            self.integral + error * dt
        )

        candidate_i = self.pitch_ki * candidate_integral

        candidate_output = (
            self.pitch_p
            + candidate_i
            + self.pitch_d
        )

        saturating_high = (
            candidate_output > self.output_max
            and error > 0
        )

        saturating_low = (
            candidate_output < self.output_min
            and error < 0
        )

        if not saturating_high and not saturating_low:
            self.integral = candidate_integral

        self.pitch_i = self.pitch_ki * self.integral

        raw_output = (
            self.pitch_p
            + self.pitch_i
            + self.pitch_d
        )

        self.pitch_output = np.clip(
            raw_output,
            self.output_min,
            self.output_max
        )

        self.previous_error = error

        return self.pitch_output

    # ============================================================
    # FLIGHT STATE MACHINE
    # ============================================================

    def update_flight_state(self, altitude):

        previous_state = self.flight_state

        if self.flight_state == self.STATE_BOOT:

            self.flight_state = self.STATE_ON_PAD

            self.ascent_counter = 0
            self.apogee_counter = 0
            self.landed_counter = 0

            self.previous_altitude = altitude
            self.landing_reference_altitude = altitude

        elif self.flight_state == self.STATE_ON_PAD:

            if altitude > self.ASCENT_ALTITUDE_THRESHOLD:
                self.ascent_counter += 1
            else:
                self.ascent_counter = 0

            if self.ascent_counter >= self.ASCENT_REQUIRED_READINGS:
                self.flight_state = self.STATE_ASCENT
                self.apogee_counter = 0

        elif self.flight_state == self.STATE_ASCENT:

            if altitude < self.previous_altitude:
                self.apogee_counter += 1
            else:
                self.apogee_counter = 0

            if self.apogee_counter >= self.APOGEE_REQUIRED_READINGS:

                self.flight_state = self.STATE_APOGEE

                self.landing_reference_altitude = altitude
                self.landed_counter = 0

        elif self.flight_state in (self.STATE_APOGEE, self.STATE_DESCENT):
            if self.accx > 1.3 and altitude > 5.0:
                self.flight_state = self.STATE_SECOND_BURN
                self.landed_counter = 0
            elif self.flight_state == self.STATE_APOGEE:
                self.flight_state = self.STATE_DESCENT
            elif self.flight_state == self.STATE_DESCENT:
                if abs(
                    altitude - self.landing_reference_altitude
                ) < self.LANDED_ALTITUDE_BAND:
                    self.landed_counter += 1
                else:
                    self.landed_counter = 0
                    self.landing_reference_altitude = altitude

                if self.landed_counter >= self.LANDED_REQUIRED_READINGS:
                    self.flight_state = self.STATE_LANDED
                    self.pid_reset()

        elif self.flight_state == self.STATE_SECOND_BURN:
            if self.accx < 0.9 and altitude < self.previous_altitude:
                self.flight_state = self.STATE_DESCENT
                self.landing_reference_altitude = altitude
                self.landed_counter = 0

        elif self.flight_state == self.STATE_LANDED:

            self.pid_reset()

        self.previous_altitude = altitude

        if previous_state != self.flight_state:
            print(
                "DHRUVA STATE:",
                self.state_name(previous_state),
                "->",
                self.state_name(self.flight_state)
            )

    def state_name(self, state):

        names = {
            self.STATE_BOOT: "BOOT",
            self.STATE_ON_PAD: "ON_PAD",
            self.STATE_ASCENT: "ASCENT",
            self.STATE_APOGEE: "APOGEE",
            self.STATE_DESCENT: "DESCENT",
            self.STATE_LANDED: "LANDED",
            self.STATE_SECOND_BURN: "SECOND_BURN",
        }

        return names[state]

    # ============================================================
    # DHRUVA TVC
    # ============================================================

    def update_tvc(self, dt):

        # TVC is active during ASCENT and SECOND_BURN.
        if self.flight_state not in (self.STATE_ASCENT, self.STATE_SECOND_BURN):

            self.pid_reset()

            return 0.0

        pitch_output = self.pid_update(
            self.pitch_setpoint_deg,
            self.pitch_deg,
            dt
        )

        return float(
            np.clip(
                pitch_output,
                self.output_min,
                self.output_max
            )
        )

    # ============================================================
    # AEROVECTOR SETUP
    # ============================================================

    def void_setup(self):
        # Nothing hardware-specific is required.

        self.t_prev = Sim.micros() / 1_000_000.0

        self.gyro, self.accx, self.accz, self.alt, \
            self.pos_gnss, self.vel_gnss = Sim.getSimData()

        self.kalman_begin(self.alt)

        self.previous_altitude = self.alt
        self.landing_reference_altitude = self.alt

        self.initialized = True

        print("DHRUVA SITL initialized")
        print("Initial altitude:", self.alt)

    # ============================================================
    # AEROVECTOR LOOP
    # ============================================================

    def void_loop(self):

        if not self.initialized:
            self.void_setup()

        t = Sim.micros() / 1_000_000.0

        if self.t_prev is None:
            self.t_prev = t
            return

        dt = t - self.t_prev

        if dt <= 0:
            return

        # Prevent a huge timestep from corrupting the controller
        # if the simulator is paused or delayed.
        dt = min(dt, 0.2)

        self.t_prev = t

        # --------------------------------------------------------
        # Read AeroVECTOR simulated sensors
        # --------------------------------------------------------

        self.gyro, self.accx, self.accz, self.alt, \
            self.pos_gnss, self.vel_gnss = Sim.getSimData()

        # AeroVECTOR example provides gyro in deg/s and
        # accelerometer in g. Convert to DHRUVA-style units.
        gyro_rad_s = self.gyro * np.pi / 180.0

        accx_ms2 = self.accx * 9.8
        accz_ms2 = self.accz * 9.8

        # --------------------------------------------------------
        # Attitude estimate
        # --------------------------------------------------------
        #
        # AeroVECTOR's Python SITL is a 2-D pitch simulator.
        # Therefore integrate the simulated pitch gyro.
        #
        self.pitch_deg += gyro_rad_s * dt * 180.0 / np.pi

        # --------------------------------------------------------
        # Kalman altitude
        # --------------------------------------------------------

        self.kalman_update(self.alt, dt)

        # --------------------------------------------------------
        # State machine
        # --------------------------------------------------------

        self.update_flight_state(self.alt)

        # --------------------------------------------------------
        # TVC
        # --------------------------------------------------------

        servo = self.update_tvc(dt)

        # --------------------------------------------------------
        # Parachute adapter
        # --------------------------------------------------------
        #
        # DHRUVA's original code does not directly command a
        # parachute. AeroVECTOR does require a parachute command
        # for a recovery simulation, so this is explicitly kept
        # as an adapter-level option.
        #
        parachute = 0

        if (
            self.deploy_parachute_on_descent
            and self.flight_state in (
                self.STATE_DESCENT,
                self.STATE_LANDED
            )
        ):
            parachute = 1

        # --------------------------------------------------------
        # Send command to AeroVECTOR
        # --------------------------------------------------------

        Sim.sendCommand(
            float(servo),
            int(parachute)
        )

        # --------------------------------------------------------
        # Plots
        # --------------------------------------------------------

        Sim.plot_variable(
            self.pitch_deg,
            1
        )

        Sim.plot_variable(
            self.kalman_altitude,
            2
        )

        Sim.plot_variable(
            self.pitch_output,
            3
        )

        Sim.plot_variable(
            self.kalman_velocity,
            4
        )

        Sim.plot_variable(
            float(self.flight_state),
            5
        )
