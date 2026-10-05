"""Deterministic motor-2 descent-trigger and TVC gain optimizer."""

from __future__ import annotations

from contextlib import redirect_stdout
import csv
import importlib.util
import io
import itertools
import json
import math
from pathlib import Path
import random
import sys


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "3 - Examples" / "Example Python SITL" / "TVC_Itr01_AeroVECTOR_POWERED_DESCENT.txt"
HARNESS = ROOT / "tests" / "baselines" / "capture_av0_baselines.py"
SEED = 20261003


def wrapped_degrees(value: float) -> float:
    return math.degrees(math.atan2(math.sin(value), math.cos(value)))


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
    candidates = list(
        itertools.product(
            (4.6, 4.7, 4.8, 4.9, 5.0),
            (0.20, 0.25, 0.30, 0.35),
            (0.10,),
        )
    )
    results = []
    for index, (drop_m, descent_kp, descent_kd) in enumerate(candidates, 1):
        candidate_parameters = list(parameters)
        candidate_parameters[3] = drop_m
        gui.sitl_gains_tab = harness._Tab(
            [0.02, 0.0, 0.04, 0.50, descent_kp, 0.0, descent_kd, 0.50]
        )
        random.seed(SEED)
        sim.random.seed(SEED)
        sim.reset_variables()
        with redirect_stdout(io.StringIO()):
            sim.update_all_parameters(
                candidate_parameters, conf_3d, conf_controller, conf_sitl, rocket_dim
            )
            sim.run_sim_python_sitl()

        count = min(
            len(sim.t_3d), len(sim.theta_3d), len(sim.servo_3d),
            len(sim.position_3d), len(sim.v_glob_3d),
        )
        times = sim.t_3d[:count]
        ignition_time = sim.rocket.t_launch2
        if ignition_time is None:
            results.append({"drop_m": drop_m, "score": 1e9, "motor2_ignited": False})
            continue
        burnout_time = ignition_time + sim.rocket.t_burnout2
        ignition_index = min(range(count), key=lambda i: abs(times[i] - ignition_time))
        burnout_index = min(range(count), key=lambda i: abs(times[i] - burnout_time))
        burn_indices = list(range(ignition_index, burnout_index + 1))
        angles = [wrapped_degrees(sim.theta_3d[i]) for i in burn_indices]
        servos = [abs(math.degrees(sim.servo_3d[i])) for i in burn_indices]
        velocity_ignition = sim.v_glob_3d[ignition_index][0]
        velocity_burnout = sim.v_glob_3d[burnout_index][0]
        pitch_ignition = wrapped_degrees(sim.theta_3d[ignition_index])
        pitch_burnout = wrapped_degrees(sim.theta_3d[burnout_index])
        saturation = sum(value >= 11.9 for value in servos) / max(len(servos), 1)
        max_pitch = max((abs(value) for value in angles), default=180.0)
        # Target a small downward velocity at burnout. Penalize unsafe attitude,
        # actuator saturation, and upward rebound.
        score = (
            8.0 * abs(velocity_burnout + 1.0)
            + 2.0 * abs(pitch_ignition)
            + 3.0 * max_pitch
            + 100.0 * saturation
            + 8.0 * max(velocity_burnout, 0.0)
        )
        result = {
            "drop_m": drop_m,
            "descent_kp": descent_kp,
            "descent_ki": 0.0,
            "descent_kd": descent_kd,
            "score": score,
            "motor2_ignited": bool(sim.motor2_ignited),
            "ignition_time_s": ignition_time,
            "burnout_time_s": burnout_time,
            "ignition_altitude_m": sim.position_3d[ignition_index][0],
            "burnout_altitude_m": sim.position_3d[burnout_index][0],
            "ignition_vertical_velocity_m_s": velocity_ignition,
            "burnout_vertical_velocity_m_s": velocity_burnout,
            "braking_delta_v_m_s": velocity_burnout - velocity_ignition,
            "ignition_pitch_deg": pitch_ignition,
            "burnout_pitch_deg": pitch_burnout,
            "burn_max_pitch_abs_deg": max_pitch,
            "burn_actuator_saturation_fraction": saturation,
            "run_end_time_s": sim.t,
            "run_end_altitude_m": sim.position_global[0],
        }
        results.append(result)
        print(
            f"{index}/{len(candidates)} drop={drop_m}m "
            f"kp={descent_kp} kd={descent_kd} "
            f"v0={velocity_ignition:.2f} v1={velocity_burnout:.2f}m/s "
            f"pitch0={pitch_ignition:.1f} max={max_pitch:.1f}deg score={score:.1f}",
            flush=True,
        )

    results.sort(key=lambda item: float(item["score"]))
    output = ROOT / "simulation_outputs" / "dhruva_powered_descent_gain_sweep.json"
    output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2))
    print(f"Saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
