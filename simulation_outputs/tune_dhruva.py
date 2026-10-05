"""Deterministic gain sweep for the supplied Dhruva dual-motor scenario."""

from __future__ import annotations

from contextlib import redirect_stdout
import copy
import importlib.util
import io
import itertools
import json
import math
from pathlib import Path
import random
import sys


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "3 - Examples" / "Example Python SITL" / "TVC_Itr01_AeroVECTOR.txt"
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
            (1.0,),
            (0.10,),
            (0.0,),
            (0.02,),
            (0.04,),
            (0.20, 0.35, 0.50, 0.65, 0.80),
        )
    )
    results: list[dict[str, float | bool]] = []
    for index, (launch_rod_length, fin_span, stage_delay, kp, kd, derivative_filter) in enumerate(candidates, 1):
        gains = [kp, 0.0, kd, derivative_filter, kp, 0.0, kd, derivative_filter]
        gui.sitl_gains_tab = harness._Tab(gains)
        candidate_parameters = list(parameters)
        candidate_parameters[21] = launch_rod_length
        candidate_parameters[2] = "Time After Burnout 1 [s]"
        candidate_parameters[3] = stage_delay
        candidate_rocket_dim = copy.deepcopy(rocket_dim)
        candidate_rocket_dim[1] = True
        candidate_rocket_dim[2] = True
        candidate_rocket_dim[6] = [
            [0.570371, 0.079629],
            [0.610371, 0.04],
            fin_span,
            0.005,
        ]
        gui.draw_rocket_tab = harness._Tab(candidate_rocket_dim)
        random.seed(SEED)
        sim.random.seed(SEED)
        sim.reset_variables()
        captured = io.StringIO()
        with redirect_stdout(captured):
            sim.update_all_parameters(
                candidate_parameters, conf_3d, conf_controller, conf_sitl,
                candidate_rocket_dim,
            )
            sim.run_sim_python_sitl()

        count = min(
            len(sim.t_3d), len(sim.theta_3d), len(sim.servo_3d),
            len(sim.position_3d), len(sim.v_glob_3d),
        )
        times = sim.t_3d[:count]
        angles = [wrapped_degrees(value) for value in sim.theta_3d[:count]]
        servos = [abs(math.degrees(value)) for value in sim.servo_3d[:count]]
        cutoff = sim.apogee_time if sim.apogee_detected else (times[-1] if times else 0.0)
        ascent_indices = [i for i, value in enumerate(times) if value <= cutoff]
        if not ascent_indices:
            ascent_indices = list(range(count))
        ascent_angles = [angles[i] for i in ascent_indices]
        ignition_index = min(
            range(count),
            key=lambda i: abs(times[i] - (sim.rocket.t_launch2 or cutoff)),
        )
        rms_pitch = math.sqrt(
            sum(value * value for value in ascent_angles) / max(len(ascent_angles), 1)
        )
        max_pitch = max((abs(value) for value in ascent_angles), default=180.0)
        saturation = sum(value >= 11.9 for value in servos) / max(count, 1)
        ignition_pitch = abs(angles[ignition_index]) if count else 180.0
        previous_index = max(0, ignition_index - 1)
        ignition_rate = abs(
            (angles[ignition_index] - angles[previous_index])
            / max(times[ignition_index] - times[previous_index], 1e-9)
        )
        max_altitude = max((value[0] for value in sim.position_3d[:count]), default=0.0)
        score = (
            2.0 * ignition_pitch
            + 10.0 * rms_pitch
            + 5.0 * max_pitch
            + 100.0 * saturation
            - 0.10 * max_altitude
        )
        result = {
            "kp": kp,
            "ki": 0.0,
            "kd": kd,
            "filter": derivative_filter,
            "launch_rod_length_m": launch_rod_length,
            "passive_fins_enabled": True,
            "passive_fin_span_m": fin_span,
            "motor2_trigger_mode": "Time After Burnout 1 [s]",
            "motor2_stage_delay_s": stage_delay,
            "score": score,
            "ignition_pitch_abs_deg": ignition_pitch,
            "ascent_pitch_rms_deg": rms_pitch,
            "ascent_pitch_max_abs_deg": max_pitch,
            "actuator_saturation_fraction": saturation,
            "max_altitude_m": max_altitude,
            "apogee_time_s": float(cutoff),
            "motor2_ignited": bool(sim.motor2_ignited),
            "impact_time_s": float(sim.t),
            "impact_downrange_m": float(sim.position_global[1]),
            "impact_vertical_velocity_m_s": float(sim.v_glob[0]),
            "ignition_rate_abs_deg_s": ignition_rate,
        }
        results.append(result)
        if index % 5 == 0:
            best = min(results, key=lambda item: float(item["score"]))
            print(
                f"{index}/{len(candidates)} best kp={best['kp']} kd={best['kd']} "
                f"rod={best['launch_rod_length_m']}m score={best['score']:.3f} "
                f"pitch@M2={best['ignition_pitch_abs_deg']:.3f}deg",
                flush=True,
            )

    results.sort(key=lambda item: float(item["score"]))
    output = ROOT / "simulation_outputs" / "dhruva_filter_refinement.json"
    output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results[:10], indent=2))
    print(f"Saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
