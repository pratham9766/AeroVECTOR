"""Tune reduced ascent impulse and fixed-motor TVC landing timing."""

from __future__ import annotations

from contextlib import redirect_stdout
import importlib.util
import io
import itertools
import json
import math
from pathlib import Path
import random
import sys


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "3 - Examples" / "Example Python SITL" / "TVC_Itr01_AeroVECTOR_TVC_LANDING.txt"
HARNESS = ROOT / "tests" / "baselines" / "capture_av0_baselines.py"
SEED = 20261003


def main() -> int:
    spec = importlib.util.spec_from_file_location("capture_av0", HARNESS)
    harness = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(harness)
    sys.path.insert(0, str(ROOT))
    harness._install_headless_backend()

    from src.gui import gui_setup as gui
    from src.simulation import main_simulation as sim

    _, parameters, conf_3d, conf_controller, conf_sitl, rocket_dim = (
        harness._configure_legacy_gui(CONFIG)
    )
    # Always tune from the user's original ascent curve.  The saved final
    # configuration points at the already-scaled derived curve.
    parameters[0] = "niche wali csv.csv"
    gui.param_file_tab.values[0] = "niche wali csv.csv"
    gui.param_file_tab.combobox[0] = harness._Value("niche wali csv.csv")
    candidates = list(itertools.product(
        (0.380, 0.385, 0.390, 0.395, 0.400, 0.405, 0.410),
        (0.0,),
    ))
    results = []
    for index, (ascent_scale, landing_margin) in enumerate(candidates, 1):
        candidate_parameters = list(parameters)
        candidate_parameters[3] = landing_margin
        random.seed(SEED)
        sim.random.seed(SEED)
        sim.reset_variables()
        with redirect_stdout(io.StringIO()):
            sim.update_all_parameters(
                candidate_parameters, conf_3d, conf_controller, conf_sitl, rocket_dim
            )
            sim.rocket.motor[1] = [value * ascent_scale for value in sim.rocket.motor[1]]
            sim.run_sim_python_sitl()

        count = min(len(sim.t_3d), len(sim.position_3d), len(sim.v_glob_3d), len(sim.theta_3d))
        ignition_time = sim.rocket.t_launch2
        ignition_altitude = None
        ignition_velocity = None
        if ignition_time is not None and count:
            ignition_index = min(range(count), key=lambda i: abs(sim.t_3d[i] - ignition_time))
            ignition_altitude = sim.position_3d[ignition_index][0]
            ignition_velocity = sim.v_glob_3d[ignition_index][0]
        touchdown_speed = sim.touchdown_vertical_velocity
        touchdown_pitch_deg = (
            sim.touchdown_pitch * sim.RAD2DEG if sim.touchdown_pitch is not None else None
        )
        touchdown_rate_deg_s = (
            sim.touchdown_pitch_rate * sim.RAD2DEG if sim.touchdown_pitch_rate is not None else None
        )
        score = (
            15.0 * abs(sim.apogee_altitude - 18.0)
            + 50.0 * abs(touchdown_speed if touchdown_speed is not None else 20.0)
            + 10.0 * abs(touchdown_pitch_deg if touchdown_pitch_deg is not None else 30.0)
            + (0.0 if sim.touchdown_success else 500.0)
        )
        result = {
            "ascent_thrust_scale": ascent_scale,
            "landing_margin_m": landing_margin,
            "score": score,
            "apogee_m": sim.apogee_altitude,
            "apogee_time_s": sim.apogee_time,
            "motor2_ignition_time_s": ignition_time,
            "motor2_ignition_altitude_m": ignition_altitude,
            "motor2_ignition_vertical_velocity_m_s": ignition_velocity,
            "touchdown_detected": sim.touchdown_detected,
            "touchdown_success": sim.touchdown_success,
            "touchdown_time_s": sim.touchdown_time,
            "touchdown_vertical_velocity_m_s": touchdown_speed,
            "touchdown_pitch_deg": touchdown_pitch_deg,
            "touchdown_pitch_rate_deg_s": touchdown_rate_deg_s,
            "parachute_deployed": sim.parachute_deployed,
            "final_time_s": sim.t,
            "final_altitude_m": sim.position_global[0],
            "final_vertical_velocity_m_s": sim.v_glob[0],
        }
        results.append(result)
        print(
            f"{index}/{len(candidates)} scale={ascent_scale:.2f} margin={landing_margin:.1f} "
            f"apogee={sim.apogee_altitude:.2f}m ignition={ignition_altitude}m "
            f"touchdown_v={touchdown_speed} success={sim.touchdown_success}",
            flush=True,
        )

    results.sort(key=lambda item: float(item["score"]))
    output = ROOT / "simulation_outputs" / "dhruva_tvc_landing_sweep.json"
    output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results[:8], indent=2))
    print(f"Saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
