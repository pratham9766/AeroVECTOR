"""Optimized Dhruva profile wrapper for the tuned dual-motor scenario.

Controller gains are still supplied by AeroVECTOR's SITL Gains tab. This
wrapper enables the existing descent parachute adapter while leaving the
original reference module unchanged.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path


_SOURCE = Path(__file__).with_name("DHRUVA_AeroVECTOR_SITL.py")
_SPEC = importlib.util.spec_from_file_location("_dhruva_aerovector_base", _SOURCE)
if _SPEC is None or _SPEC.loader is None:
    raise ImportError(f"Could not load Dhruva SITL base module from {_SOURCE}")
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)


class SITLProgram(_BASE.SITLProgram):
    def __init__(self):
        super().__init__()
        # Tell AeroVECTOR to use this module's validated profile rather than
        # replacing it with the global SITL Gains tab values.
        self.use_profile_gains = True
        self.ascent_pitch_kp = 0.02
        self.ascent_pitch_ki = 0.0
        self.ascent_pitch_kd = 0.04
        self.ascent_derivative_filter = 0.50
        self.descent_pitch_kp = 0.02
        self.descent_pitch_ki = 0.0
        self.descent_pitch_kd = 0.04
        self.descent_derivative_filter = 0.50
        self.deploy_parachute_on_descent = True
        print(
            "DHRUVA OPTIMIZED PROFILE: locked tuned gains and descent "
            "parachute adapter enabled"
        )

    def update_flight_state(self, altitude):
        """Retain the base state machine, with sample-hold-safe descent entry."""
        super().update_flight_state(altitude)
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
