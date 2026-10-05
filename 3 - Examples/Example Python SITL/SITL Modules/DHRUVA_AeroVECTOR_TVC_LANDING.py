"""Experimental low-altitude, fixed-motor TVC landing profile.

The bounded coast hold represents imperfect hardware such as RCS or a reaction
wheel. TVC attitude control is used during the landing-motor burn itself.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path


_SOURCE = Path(__file__).with_name("DHRUVA_AeroVECTOR_POWERED_DESCENT.py")
_SPEC = importlib.util.spec_from_file_location("_dhruva_powered_descent_base", _SOURCE)
if _SPEC is None or _SPEC.loader is None:
    raise ImportError(f"Could not load powered-descent base module from {_SOURCE}")
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)


class SITLProgram(_BASE.SITLProgram):
    def __init__(self):
        super().__init__()
        self.tvc_landing_mode = True
        self.deploy_recovery_after_burn = False
        self.deploy_parachute_on_descent = False

        # TVC gains used while the fixed landing motor produces thrust.
        self.descent_pitch_kp = 0.42
        self.descent_pitch_ki = 0.0
        self.descent_pitch_kd = 0.12
        self.descent_derivative_filter = 0.50

        # Imperfect, separately bounded coast-phase attitude system. This is
        # not TVC; an unlit rocket motor cannot generate TVC torque. Deadband,
        # limited authority and deterministic disturbance make the landing
        # controller visibly work instead of holding an ideal zero attitude.
        self.ideal_coast_attitude_hold = True
        self.coast_hold_kp = 8.0
        self.coast_hold_kd = 4.0
        self.coast_hold_deadband_deg = 2.0
        self.coast_hold_max_accel_rad_s2 = 2.0
        self.coast_disturbance_rad_s2 = 0.40
        self.coast_disturbance_hz = 1.35

        self.touchdown_max_vertical_speed_m_s = 2.0
        self.touchdown_max_horizontal_speed_m_s = 2.0
        self.touchdown_max_pitch_deg = 3.0
        self.touchdown_max_pitch_rate_deg_s = 5.0

        print("DHRUVA TVC LANDING PROFILE: parachute disabled")
