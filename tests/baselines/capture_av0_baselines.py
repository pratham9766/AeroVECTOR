"""Capture deterministic AV0 regression evidence from the legacy simulator.

This is an observation harness only.  It does not replace or modify any
equation, interpolation, controller, actuator, or integration routine in
``src``.  The small GUI stubs supply the configuration values that the legacy
simulation currently reads from Tkinter widgets.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import random
import shutil
import subprocess
import sys
import time
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
BASELINE_ROOT = Path(__file__).resolve().parent
SEED = 20261003

CASES = {
    "legacy_tvc": REPOSITORY_ROOT / "3 - Examples" / "Others" / "Example Rocket TVC.txt",
    "legacy_active_fins": REPOSITORY_ROOT / "3 - Examples" / "Others" / "Example Rocket Active Fins.txt",
    "dhruva_dual_motor_sitl": REPOSITORY_ROOT / "3 - Examples" / "Example Python SITL" / "TVC_Itr01_AeroVECTOR.txt",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json_value(value: Any) -> Any:
    if hasattr(value, "item"):
        value = value.item()
    if isinstance(value, float):
        if value != value:
            return "NaN"
        if value == float("inf"):
            return "Infinity"
        if value == float("-inf"):
            return "-Infinity"
    return value


def _git_revision() -> str | None:
    try:
        return subprocess.check_output(
            [
                "git",
                "-c",
                f"safe.directory={REPOSITORY_ROOT.as_posix()}",
                "-C",
                str(REPOSITORY_ROOT),
                "rev-parse",
                "HEAD",
            ],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


class _Value:
    def __init__(self, value: Any):
        self.value = value

    def get(self) -> Any:
        return self.value


class _Tab:
    def __init__(self, values: list[Any]):
        self.values = values

    def get_configuration_destringed(self) -> list[Any]:
        return self.values


def _destring(values: list[Any]) -> list[Any]:
    converted: list[Any] = []
    for value in values:
        if value == "True":
            converted.append(True)
        elif value == "False":
            converted.append(False)
        else:
            try:
                number = float(value)
                converted.append(int(number) if number > 9000 and number.is_integer() else number)
            except (TypeError, ValueError):
                converted.append(value)
    return converted


def _number_pair(value: str) -> list[float]:
    return [float(part.strip()) for part in value.split(",")]


def _parse_rocket_dim(values: list[Any]) -> list[Any]:
    fin_stabilization = values.index("Fins_s")
    fin_control = values.index("Fins_c")
    flags = [value == "True" for value in values[:5]]
    body = [_number_pair(value) for value in values[5:fin_stabilization]]

    def fin_block(block: list[str]) -> list[Any]:
        root = _number_pair(block[0])
        tip = _number_pair(block[1])
        return [root, tip, float(block[2]), float(block[3])]

    return flags + [body, fin_block(values[fin_stabilization + 1:fin_control]), fin_block(values[fin_control + 1:])]


def _install_headless_backend() -> None:
    # main_simulation imports pyplot and then requests TkAgg.  AV0 needs the
    # numerical implementation, not a Tk window, so retain Agg during import.
    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib

    matplotlib.use("Agg")
    original_use = matplotlib.use
    matplotlib.use = lambda *args, **kwargs: None
    try:
        from src.simulation import main_simulation  # noqa: F401
    finally:
        matplotlib.use = original_use


def _configure_legacy_gui(config_path: Path):
    from src.gui import gui_setup as gui

    savefile = gui.savefile
    savefile.update_path(config_path.as_posix())
    savefile.read_file()

    parameters = _destring(savefile.get_parameters())
    conf_3d = _destring(savefile.get_conf_3d())
    conf_controller = _destring(savefile.get_conf_controller())
    conf_sitl = _destring(savefile.get_conf_sitl())
    conf_plots = _destring(savefile.get_conf_plots())
    rocket_dim = _parse_rocket_dim(savefile.get_rocket_dim())

    gui.param_file_tab = _Tab(parameters)
    gui.param_file_tab.combobox = [_Value(parameters[0]), _Value(parameters[1])]
    gui.draw_rocket_tab = _Tab(rocket_dim)
    gui.conf_3d_tab = _Tab(conf_3d)
    gui.sim_setup_tab = _Tab(conf_controller)
    gui.conf_sitl_tab = _Tab(conf_sitl)
    gui.run_sim_tab = _Tab(conf_plots)
    gui.sitl_gains_tab = _Tab([0.05, 0.04, 0.01, 0.90, 0.05, 0.04, 0.01, 0.90])

    return savefile, parameters, conf_3d, conf_controller, conf_sitl, rocket_dim


def _write_trace(sim: Any, path: Path) -> None:
    count = min(
        len(sim.t_3d),
        len(sim.position_3d),
        len(sim.v_glob_3d),
        len(sim.theta_3d),
        len(sim.aoa_3d),
        len(sim.servo_3d),
        len(sim.thrust_3d),
        len(sim.xcg_3d),
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "altitude_m",
                "downrange_m",
                "vertical_velocity_m_s",
                "horizontal_velocity_m_s",
                "pitch_rad",
                "aoa_rad",
                "actuator_angle_rad",
                "thrust_n",
                "xcg_m",
            ]
        )
        for index in range(count):
            writer.writerow(
                [
                    sim.t_3d[index],
                    sim.position_3d[index][0],
                    sim.position_3d[index][1],
                    sim.v_glob_3d[index][0],
                    sim.v_glob_3d[index][1],
                    sim.theta_3d[index],
                    sim.aoa_3d[index],
                    sim.servo_3d[index],
                    sim.thrust_3d[index],
                    sim.xcg_3d[index],
                ]
            )


def _write_plot(sim: Any, path: Path, title: str) -> None:
    import matplotlib.pyplot as plt

    count = min(len(sim.t_3d), len(sim.position_3d), len(sim.v_glob_3d))
    times = sim.t_3d[:count]
    altitude = [value[0] for value in sim.position_3d[:count]]
    velocity = [value[0] for value in sim.v_glob_3d[:count]]
    figure, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(times, altitude)
    axes[0].set_ylabel("Altitude [m]")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(times, velocity)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Vertical velocity [m/s]")
    axes[1].grid(True, alpha=0.3)
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def _capture_aerodynamics(sim: Any, path: Path) -> list[dict[str, Any]]:
    import math

    snapshots: list[dict[str, Any]] = []
    for angle_deg, pitch_rate_rad_s, actuator_deg in (
        (0.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        (5.0, 0.25, 0.0),
        (5.0, 0.25, 5.0),
    ):
        velocity = 30.0
        lateral = velocity * math.tan(math.radians(angle_deg))
        cn, cm_xcg, ca, cp = sim.rocket.calculate_aero_coef(
            [velocity, lateral],
            pitch_rate_rad_s,
            0.0,
            math.radians(actuator_deg),
        )
        snapshots.append(
            {
                "input": {
                    "body_axial_velocity_m_s": velocity,
                    "body_lateral_velocity_m_s": lateral,
                    "nominal_aoa_deg": angle_deg,
                    "pitch_rate_rad_s": pitch_rate_rad_s,
                    "altitude_m_msl": 0.0,
                    "actuator_angle_deg": actuator_deg,
                },
                "output": {
                    "cn": _json_value(cn),
                    "cm_about_cg": _json_value(cm_xcg),
                    "ca": _json_value(ca),
                    "cp_from_nose_m": _json_value(cp),
                    "mach": _json_value(sim.rocket.mach),
                    "reynolds": _json_value(sim.rocket.reynolds),
                    "rho_kg_m3": _json_value(sim.rocket.rho),
                    "passive_fin_cn": [_json_value(value) for value in sim.rocket.fin_cn],
                    "fin_ca": [_json_value(value) for value in sim.rocket.fin_ca],
                },
            }
        )
    with path.open("w", encoding="utf-8") as stream:
        json.dump(snapshots, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return snapshots


def capture_case(name: str, config_path: Path) -> dict[str, Any]:
    from src.simulation import main_simulation as sim

    case_dir = BASELINE_ROOT / name
    case_dir.mkdir(parents=True, exist_ok=True)
    copied_config = case_dir / "input_config.txt"
    shutil.copy2(config_path, copied_config)

    random.seed(SEED)
    sim.random.seed(SEED)
    sim.reset_variables()
    savefile, parameters, conf_3d, conf_controller, conf_sitl, rocket_dim = _configure_legacy_gui(config_path)

    propulsion_inputs: list[dict[str, str]] = []
    for index, role in ((0, "primary"), (1, "secondary")):
        motor_name = str(parameters[index])
        if motor_name in ("", "None", "Disabled"):
            continue
        motor_path = REPOSITORY_ROOT / "Motors" / motor_name
        copied_motor = case_dir / f"{role}_motor{motor_path.suffix}"
        shutil.copy2(motor_path, copied_motor)
        propulsion_inputs.append(
            {
                "role": role,
                "source": str(motor_path.relative_to(REPOSITORY_ROOT)),
                "source_sha256": _sha256(motor_path),
                "copy": copied_motor.name,
                "copy_sha256": _sha256(copied_motor),
            }
        )

    start = time.perf_counter()
    status = "completed"
    error = None
    aero_snapshots: list[dict[str, Any]] = []
    try:
        sim.update_all_parameters(
            parameters,
            conf_3d,
            conf_controller,
            conf_sitl,
            rocket_dim,
        )
        aero_snapshots = _capture_aerodynamics(sim, case_dir / "aerodynamics.json")
        if sim.Activate_SITL and sim.enable_python_sitl:
            sim.run_sim_python_sitl()
        elif sim.Activate_SITL:
            status = "not_run_hardware_sitl"
        else:
            sim.run_sim_local()
    except Exception as exc:  # Preserve failures as baseline evidence.
        status = "failed"
        error = f"{type(exc).__name__}: {exc}"
    runtime = time.perf_counter() - start

    _write_trace(sim, case_dir / "trace.csv")
    _write_plot(sim, case_dir / "trajectory.png", f"AV0 baseline: {name}")

    summary = {
        "schema_version": 1,
        "track": "AV0",
        "case": name,
        "status": status,
        "error": error,
        "seed": SEED,
        "source_config": str(config_path.relative_to(REPOSITORY_ROOT)),
        "source_config_sha256": _sha256(config_path),
        "copied_config_sha256": _sha256(copied_config),
        "propulsion_inputs": propulsion_inputs,
        "git_revision": _git_revision(),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "solver": {
            "name": "legacy fixed-step trapezoidal",
            "sim_delta_t_s": _json_value(getattr(sim, "T", None)),
            "controller_sample_time_s": _json_value(getattr(sim, "T_Program", None)),
            "servo_sample_time_s": _json_value(getattr(sim, "Ts", None)),
        },
        "runtime_seconds": runtime,
        "sample_count": len(sim.t_3d),
        "aerodynamic_snapshot_count": len(aero_snapshots),
        "final_state": {
            "time_s": _json_value(getattr(sim, "t", None)),
            "altitude_m": _json_value(sim.position_global[0]),
            "downrange_m": _json_value(sim.position_global[1]),
            "vertical_velocity_m_s": _json_value(sim.v_glob[0]),
            "horizontal_velocity_m_s": _json_value(sim.v_glob[1]),
            "pitch_rad": _json_value(getattr(sim, "theta", None)),
            "pitch_rate_rad_s": _json_value(getattr(sim, "Q", None)),
            "mass_kg": _json_value(getattr(sim, "m", None)),
            "xcg_m": _json_value(getattr(sim, "xcg", None)),
            "pitch_inertia_kg_m2": _json_value(getattr(sim, "Iy", None)),
        },
        "events": {
            "primary_burnout_time_s": _json_value(getattr(sim, "t_launch", 0.0) + sim.rocket.t_burnout),
            "apogee_detected": bool(getattr(sim, "apogee_detected", False)),
            "apogee_time_s": _json_value(getattr(sim, "apogee_time", None)) if getattr(sim, "apogee_detected", False) else None,
            "apogee_altitude_m": _json_value(getattr(sim, "apogee_altitude", None)) if getattr(sim, "apogee_detected", False) else None,
            "motor2_ignited": bool(getattr(sim, "motor2_ignited", False)),
            "motor2_ignition_time_s": _json_value(sim.rocket.t_launch2),
            "parachute_deployed": bool(getattr(sim, "parachute_deployed", False)),
            "parachute_deploy_time_s": _json_value(getattr(sim, "parachute_deploy_time", None)),
        },
        "artifacts": {
            "configuration": "input_config.txt",
            "trace": "trace.csv",
            "plot": "trajectory.png",
            "aerodynamics": "aerodynamics.json",
        },
        "notes": [
            "Dynamics, aerodynamics, propulsion, actuator, controller, and integrator are the unmodified legacy implementations.",
            "Tkinter widgets are replaced only by value-returning stubs so the run is headless.",
            "trace.csv uses the legacy 3-D playback sampling cadence (approximately 0.005 s), not every integration step.",
        ],
    }
    with (case_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return summary


def main() -> int:
    os.chdir(REPOSITORY_ROOT)
    sys.path.insert(0, str(REPOSITORY_ROOT))
    _install_headless_backend()
    summaries = [capture_case(name, path) for name, path in CASES.items()]
    manifest = {
        "schema_version": 1,
        "track": "AV0",
        "seed": SEED,
        "git_revision": _git_revision(),
        "cases": summaries,
    }
    with (BASELINE_ROOT / "manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return 0 if all(item["status"] == "completed" for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
