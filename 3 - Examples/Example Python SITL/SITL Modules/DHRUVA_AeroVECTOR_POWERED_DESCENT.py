"""Dhruva profile that reserves motor 2 for TVC-controlled descent braking."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
from src import python_sitl_functions as Sim


_SOURCE = Path(__file__).with_name("DHRUVA_AeroVECTOR_SITL.py")
_SPEC = importlib.util.spec_from_file_location("_dhruva_aerovector_descent_base", _SOURCE)
if _SPEC is None or _SPEC.loader is None:
    raise ImportError(f"Could not load Dhruva SITL base module from {_SOURCE}")
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)


class SITLProgram(_BASE.SITLProgram):
    def __init__(self):
        super().__init__()
        # Lock the validated profile so a stale GUI gains tab cannot silently
        # replace the powered-descent controller values.
        self.use_profile_gains = True
        self.ascent_pitch_kp = 0.02
        self.ascent_pitch_ki = 0.0
        self.ascent_pitch_kd = 0.04
        self.ascent_derivative_filter = 0.50
        self.descent_pitch_kp = 0.40
        self.descent_pitch_ki = 0.0
        self.descent_pitch_kd = 0.10
        self.descent_derivative_filter = 0.50
        self.deploy_parachute_on_descent = False
        self.deploy_recovery_after_burn = True
        self.powered_burn_seen = False
        self.recovery_ready = False
        print("DHRUVA POWERED DESCENT PROFILE: motor 2 reserved for braking")

    def update_flight_state(self, altitude):
        previous_state = self.flight_state
        normal_landed_readings = self.LANDED_REQUIRED_READINGS
        if not self.powered_burn_seen:
            self.LANDED_REQUIRED_READINGS = 1_000_000_000
        try:
            super().update_flight_state(altitude)
        finally:
            self.LANDED_REQUIRED_READINGS = normal_landed_readings
        # The base firmware's stationary-on-ground test is intentionally
        # simple, but near apogee it can mistake a slowly changing altitude
        # for landing.  A powered-descent profile cannot enter LANDED before
        # the reserved motor has fired.
        if self.flight_state == self.STATE_LANDED and not self.powered_burn_seen:
            self.flight_state = self.STATE_DESCENT
            self.landed_counter = 0
            self.landing_reference_altitude = altitude
        if (
            self.flight_state == self.STATE_ASCENT
            and self.kalman_velocity < -0.5
            and altitude > 5.0
        ):
            self.flight_state = self.STATE_DESCENT
            self.landing_reference_altitude = altitude
            self.landed_counter = 0
            self.pid_reset()
            print("DHRUVA STATE: ASCENT -> DESCENT (Kalman vertical velocity)")
        if self.flight_state == self.STATE_SECOND_BURN:
            self.powered_burn_seen = True
        if (
            self.powered_burn_seen
            and previous_state == self.STATE_SECOND_BURN
            and self.flight_state == self.STATE_DESCENT
        ):
            self.recovery_ready = True

    def update_tvc(self, dt):
        if self.flight_state == self.STATE_ASCENT:
            gains = (
                self.ascent_pitch_kp,
                self.ascent_pitch_ki,
                self.ascent_pitch_kd,
                self.ascent_derivative_filter,
            )
        elif self.flight_state in (self.STATE_DESCENT, self.STATE_SECOND_BURN):
            gains = (
                self.descent_pitch_kp,
                self.descent_pitch_ki,
                self.descent_pitch_kd,
                self.descent_derivative_filter,
            )
        else:
            self.pid_reset()
            return 0.0
        return float(
            np.clip(
                self.pid_update(self.pitch_setpoint_deg, self.pitch_deg, dt, *gains),
                self.output_min,
                self.output_max,
            )
        )

    def void_loop(self):
        super().void_loop()
        if self.recovery_ready and self.deploy_recovery_after_burn:
            Sim.sendCommand(0.0, 1)
