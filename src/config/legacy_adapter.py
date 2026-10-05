"""Loss-aware adapter for AeroVECTOR's positional legacy text format."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Any, Callable

from src.config.models import (
    ActuatorConfig,
    AerodynamicConfig,
    CanonicalConfig,
    DisplayConfig,
    EngineeringValue,
    EnvironmentConfig,
    FinGeometry,
    GeometryPoint,
    LaunchConfig,
    MassProperties,
    PropulsionConfig,
    RecoveryConfig,
    SecondaryMotorConfig,
    SensorConfig,
    SimulationConfig,
    ValidationIssue,
    ValidationState,
    VehicleGeometry,
)
from src.config.units import UnitError, convert_value


class LegacyConfigError(ValueError):
    pass


@dataclass(frozen=True)
class LegacySections:
    parameters: tuple[str, ...]
    display: tuple[str, ...]
    controller: tuple[str, ...]
    sitl: tuple[str, ...]
    plots: tuple[str, ...]
    geometry: tuple[str, ...]
    raw_lines: tuple[str, ...] = ()


PARAMETER_NAMES = (
    "Motor", "Motor 2", "Motor 2 Trigger Mode", "Motor 2 Trigger Value",
    "Mass Liftoff", "Mass Burnout", "Mass Burnout 2", "Iy Liftoff",
    "Iy Burnout", "Iy Burnout 2", "Xcg Liftoff", "Xcg Burnout",
    "Xcg Burnout 2", "Xt", "Servo Resolution", "Max Actuator Angle",
    "Actuator Reduction", "Initial Misalignment", "Servo Compensation",
    "Wind", "Wind Gust", "Launch Rod Length", "Launch Rod Angle",
    "Motor Misalignment", "Rocket Roughness", "Stabilization Fin Roughness",
    "Control Fin Roughness",
)

DISPLAY_NAMES = (
    "Toggle 3D", "Camera Shake Toggle", "Hide Forces", "Variable FOV",
    "Hide cg", "Camera Type", "Slow mo", "Force Scale", "FOV",
)

CONTROLLER_NAMES = (
    "Torque Controller", "Anti Windup", "Auto Parachute on Tilt", "Input Type",
    "Kp", "Ki", "Kd", "K All", "K Damping", "Reference Thrust", "Input",
    "Input time", "Launch Time", "Servo Sample Time", "Controller Sample Time",
    "Maximum Sim Duration", "Sim Delta T", "Export T", "Launch Altitude",
    "Initial Altitude", "Initial Vertical Velocity", "Initial Horizontal Velocity",
    "Initial Pitch Angle", "Initial Pitch Rate",
)

SITL_NAMES = (
    "Activate SITL", "Use Sensor Noise", "Python SITL", "File", "Port",
    "Baudrate", "Gyroscope SD", "Accelerometer SD", "Altimeter SD",
    "GNSS Pos SD", "GNSS Vel SD", "Gyroscope ST", "Accelerometer ST",
    "Altimeter ST", "GNSS ST",
)

PLOT_NAMES = tuple(f"Plot {index}" for index in range(1, 11))

TRIGGER_MODES = frozenset(
    {
        "Disabled",
        "None",
        "",
        "Distance After Apogee [m]",
        "Altitude Threshold [m]",
        "Time After Burnout 1 [s]",
        "Time After Launch [s]",
    "TVC Landing Burn",
    }
)
INPUT_MODES = frozenset({"Step [º]", "Ramp [º/s]", "Up"})
CAMERA_TYPES = frozenset({"Fixed", "Follow", "Follow Far", "Side View"})


def _format_number(value: float | int) -> str:
    if isinstance(value, int) or float(value).is_integer():
        return str(int(value))
    return format(float(value), ".15g")


class LegacyConfigAdapter:
    """Parse and regenerate legacy-compatible positional section values."""

    def __init__(self, motor_directory: str | Path | None = None):
        self.motor_directory = Path(motor_directory) if motor_directory is not None else None
        self._issues: list[ValidationIssue] = []
        self._unsupported: list[dict[str, Any]] = []
        self._comments: list[str] = []
        self._value_state = ValidationState.IMPORTED

    @staticmethod
    def parse_sections(text: str) -> LegacySections:
        raw_lines = tuple(text.splitlines(keepends=True))
        sections: list[list[str]] = [[]]
        for raw_line in text.splitlines():
            if raw_line.strip() == "###=#":
                sections.append([])
                continue
            if raw_line.lstrip().startswith("#"):
                # Comments are metadata, not positional values. They remain in
                # raw_lines and are copied to CanonicalConfig.comments.
                continue
            try:
                value = raw_line.split("=")[1].strip()
            except IndexError:
                value = raw_line.split("=")[0].strip()
            sections[-1].append(value)
        if len(sections) < 6:
            raise LegacyConfigError(
                f"Expected six legacy sections separated by five '###=#' lines; received {len(sections)} sections"
            )
        return LegacySections(
            parameters=tuple(sections[0]),
            display=tuple(sections[1]),
            controller=tuple(sections[2]),
            sitl=tuple(sections[3]),
            plots=tuple(sections[4]),
            geometry=tuple(sections[5]),
            raw_lines=raw_lines,
        )

    def parse_file(self, path: str | Path) -> CanonicalConfig:
        source_path = Path(path)
        text = source_path.read_text(encoding="utf-8")
        created_at = datetime.fromtimestamp(source_path.stat().st_mtime, timezone.utc).isoformat()
        return self.from_sections(
            self.parse_sections(text),
            source=str(source_path),
            created_at=created_at,
        )

    def parse_text(self, text: str, *, source: str | None = None) -> CanonicalConfig:
        return self.from_sections(self.parse_sections(text), source=source)

    @staticmethod
    def _upgrade_sections(sections: LegacySections) -> tuple[LegacySections, set[int], set[int]]:
        parameters = list(sections.parameters)
        controller = list(sections.controller)
        defaulted_parameters: set[int] = set()
        defaulted_controller: set[int] = set()
        if len(parameters) == 21:
            old = parameters
            parameters = [
                old[0], "None", "Disabled", "0", old[1], old[2], old[2],
                old[3], old[4], old[4], old[5], old[6], old[6],
            ] + old[7:21]
            defaulted_parameters.update({1, 2, 3, 6, 9, 12})
        if len(controller) == 23:
            controller = [controller[0], controller[1], "True"] + controller[2:]
            defaulted_controller.add(2)
        return (
            LegacySections(
                tuple(parameters), sections.display, tuple(controller), sections.sitl,
                sections.plots, sections.geometry, sections.raw_lines,
            ),
            defaulted_parameters,
            defaulted_controller,
        )

    def _source(self, section: str, index: int, name: str) -> str:
        return f"legacy:{section}[{index}] {name}"

    def _issue(
        self,
        field: str,
        received: Any,
        condition: str,
        source: str | None,
        correction: str,
        *,
        severity: str = "error",
    ) -> None:
        self._issues.append(
            ValidationIssue(field, received, condition, source, correction, severity)
        )

    def _numeric(
        self,
        raw: Any,
        *,
        field: str,
        source: str,
        unit: str | None,
        original_unit: str | None = None,
        condition: Callable[[float], bool] = lambda value: True,
        expected: str = "a finite numeric value",
        correction: str = "Provide a finite numeric value in the documented unit.",
        defaulted: bool = False,
    ) -> EngineeringValue[float]:
        state = ValidationState.DEFAULTED_LEGACY if defaulted else self._value_state
        if raw is None or str(raw).strip() == "":
            self._issue(field, raw, expected, source, correction)
            return EngineeringValue(None, unit, source, validation_state=ValidationState.MISSING, original_value=raw, original_unit=original_unit)
        try:
            original = float(raw)
            if not math.isfinite(original):
                raise ValueError
            value = convert_value(original, original_unit, unit) if original_unit != unit else original
            if not condition(value):
                raise ValueError
        except (ValueError, TypeError, UnitError):
            self._issue(field, raw, expected, source, correction)
            return EngineeringValue(None, unit, source, validation_state=ValidationState.INVALID, original_value=raw, original_unit=original_unit)
        conversion = None
        if original_unit != unit:
            conversion = {"from_unit": original_unit, "to_unit": unit, "formula": "registered scalar conversion"}
        return EngineeringValue(
            float(value), unit, source, validation_state=state,
            original_value=raw, original_unit=original_unit, conversion=conversion,
            provenance={"legacy_default_substitution": defaulted} if defaulted else None,
        )

    def _integer(self, raw: Any, *, field: str, source: str, positive: bool = False) -> EngineeringValue[int]:
        numeric = self._numeric(
            raw, field=field, source=source, unit="1", original_unit="1",
            condition=(lambda value: value > 0 and value.is_integer()) if positive else (lambda value: value.is_integer()),
            expected="a positive integer" if positive else "an integer",
            correction="Provide an integer value.",
        )
        return EngineeringValue(
            int(numeric.value) if numeric.value is not None else None,
            numeric.unit, numeric.source, numeric.uncertainty, numeric.provenance,
            numeric.notes, numeric.validation_state, numeric.original_value,
            numeric.original_unit, numeric.conversion,
        )

    def _boolean(self, raw: Any, *, field: str, source: str, defaulted: bool = False) -> EngineeringValue[bool]:
        if raw in (True, "True", "true", "1", 1):
            value = True
        elif raw in (False, "False", "false", "0", 0):
            value = False
        else:
            self._issue(field, raw, "one of True or False", source, "Use 'True' or 'False'.")
            return EngineeringValue(None, None, source, validation_state=ValidationState.MISSING if raw in (None, "") else ValidationState.INVALID, original_value=raw)
        return EngineeringValue(
            value, None, source,
            validation_state=ValidationState.DEFAULTED_LEGACY if defaulted else self._value_state,
            original_value=raw,
            provenance={"legacy_default_substitution": True} if defaulted else None,
        )

    def _string(
        self,
        raw: Any,
        *,
        field: str,
        source: str,
        allowed: frozenset[str] | None = None,
        allow_empty: bool = False,
        defaulted: bool = False,
    ) -> EngineeringValue[str]:
        if raw is None or (str(raw) == "" and not allow_empty):
            self._issue(field, raw, "a non-empty string", source, "Provide a supported value.")
            return EngineeringValue(None, None, source, validation_state=ValidationState.MISSING, original_value=raw)
        value = str(raw)
        if allowed is not None and value not in allowed:
            self._issue(field, raw, f"one of {sorted(allowed)!r}", source, "Choose one of the supported modes.")
            return EngineeringValue(None, None, source, validation_state=ValidationState.INVALID, original_value=raw)
        return EngineeringValue(
            value, None, source,
            validation_state=ValidationState.DEFAULTED_LEGACY if defaulted else self._value_state,
            original_value=raw,
            provenance={"legacy_default_substitution": True} if defaulted else None,
        )

    @staticmethod
    def _at(values: tuple[str, ...], index: int) -> str | None:
        return values[index] if index < len(values) else None

    def _pair(self, raw: Any, *, field: str, source: str) -> tuple[EngineeringValue[float], EngineeringValue[float]]:
        parts = str(raw).split(",") if raw is not None else []
        first = parts[0].strip() if len(parts) > 0 else None
        second = parts[1].strip() if len(parts) > 1 else None
        return (
            self._numeric(first, field=f"{field}.position", source=source, unit="m", original_unit="m", condition=lambda value: value >= 0, expected="a nonnegative axial position in m"),
            self._numeric(second, field=f"{field}.size", source=source, unit="m", original_unit="m", condition=lambda value: value >= 0, expected="a nonnegative dimension in m"),
        )

    def _fin(self, block: list[str | None], *, prefix: str, base_index: int) -> FinGeometry:
        root_position, root_chord = self._pair(block[0], field=f"{prefix}.root", source=self._source("geometry", base_index, "fin root"))
        tip_position, tip_chord = self._pair(block[1], field=f"{prefix}.tip", source=self._source("geometry", base_index + 1, "fin tip"))
        span = self._numeric(block[2], field=f"{prefix}.span", source=self._source("geometry", base_index + 2, "fin span"), unit="m", original_unit="m", condition=lambda value: value >= 0, expected="a nonnegative fin span in m")
        thickness = self._numeric(block[3], field=f"{prefix}.thickness", source=self._source("geometry", base_index + 3, "fin thickness"), unit="m", original_unit="m", condition=lambda value: value >= 0, expected="a nonnegative fin thickness in m")
        return FinGeometry(root_position, root_chord, tip_position, tip_chord, span, thickness)

    def from_sections(
        self,
        sections: LegacySections,
        *,
        source: str | None = None,
        created_at: str | None = None,
        value_state: ValidationState = ValidationState.IMPORTED,
    ) -> CanonicalConfig:
        self._issues = []
        self._unsupported = []
        self._value_state = value_state
        self._comments = [line.rstrip("\r\n") for line in sections.raw_lines if line.lstrip().startswith("#") and line.strip() != "###=#"]
        sections, defaulted_parameters, defaulted_controller = self._upgrade_sections(sections)
        p, d, c, s, plots, geometry = (
            sections.parameters, sections.display, sections.controller,
            sections.sitl, sections.plots, sections.geometry,
        )

        expected_lengths = {"parameters": 27, "display": 9, "controller": 24, "sitl": 15, "plots": 10}
        actual = {"parameters": p, "display": d, "controller": c, "sitl": s, "plots": plots}
        for section_name, expected_length in expected_lengths.items():
            values = actual[section_name]
            if len(values) > expected_length:
                for index, value in enumerate(values[expected_length:], start=expected_length):
                    self._unsupported.append({"section": section_name, "position": index, "raw_value": value, "reason": "extra legacy field"})

        def num(values: tuple[str, ...], index: int, field: str, unit: str | None, original_unit: str | None = None, condition: Callable[[float], bool] = lambda value: True, expected: str = "a finite numeric value", correction: str = "Provide a valid value in the documented unit.", defaulted_set: set[int] | None = None) -> EngineeringValue[float]:
            names = PARAMETER_NAMES if values is p else CONTROLLER_NAMES if values is c else DISPLAY_NAMES if values is d else SITL_NAMES
            name = names[index] if index < len(names) else field
            return self._numeric(self._at(values, index), field=field, source=self._source("parameters" if values is p else "controller" if values is c else "display" if values is d else "sitl", index, name), unit=unit, original_unit=unit if original_unit is None else original_unit, condition=condition, expected=expected, correction=correction, defaulted=defaulted_set is not None and index in defaulted_set)

        def boolean(values: tuple[str, ...], index: int, field: str, defaulted_set: set[int] | None = None) -> EngineeringValue[bool]:
            names = CONTROLLER_NAMES if values is c else DISPLAY_NAMES if values is d else SITL_NAMES
            return self._boolean(self._at(values, index), field=field, source=self._source("controller" if values is c else "display" if values is d else "sitl", index, names[index]), defaulted=defaulted_set is not None and index in defaulted_set)

        def string(values: tuple[str, ...], index: int, field: str, *, allowed: frozenset[str] | None = None, allow_empty: bool = False, defaulted_set: set[int] | None = None) -> EngineeringValue[str]:
            names = PARAMETER_NAMES if values is p else CONTROLLER_NAMES if values is c else DISPLAY_NAMES if values is d else SITL_NAMES
            return self._string(self._at(values, index), field=field, source=self._source("parameters" if values is p else "controller" if values is c else "display" if values is d else "sitl", index, names[index]), allowed=allowed, allow_empty=allow_empty, defaulted=defaulted_set is not None and index in defaulted_set)

        primary_motor = string(p, 0, "propulsion.primary_motor_file")
        secondary_file = string(p, 1, "propulsion.secondary_motor.motor_file", allow_empty=True, defaulted_set=defaulted_parameters)
        trigger_mode = string(p, 2, "propulsion.secondary_motor.trigger_mode", allowed=TRIGGER_MODES, allow_empty=True, defaulted_set=defaulted_parameters)
        trigger_unit = "s" if trigger_mode.value in {"Time After Burnout 1 [s]", "Time After Launch [s]"} else "m" if trigger_mode.value in {"Distance After Apogee [m]", "Altitude Threshold [m]", "TVC Landing Burn"} else "1"
        trigger_value = num(p, 3, "propulsion.secondary_motor.trigger_value", trigger_unit, trigger_unit, defaulted_set=defaulted_parameters)

        if self.motor_directory is not None:
            for field_name, motor in (("propulsion.primary_motor_file", primary_motor), ("propulsion.secondary_motor.motor_file", secondary_file)):
                if motor.value not in (None, "", "None", "Disabled") and not (self.motor_directory / motor.value).is_file():
                    self._issue(field_name, motor.value, "an existing motor data file", motor.source, f"Add {motor.value!r} to {str(self.motor_directory)!r} or choose an existing motor file.")
        if trigger_mode.value not in (None, "", "None", "Disabled") and secondary_file.value in (None, "", "None", "Disabled"):
            self._issue("propulsion.secondary_motor", secondary_file.value, "a motor file when a trigger is enabled", secondary_file.source, "Select a secondary motor or disable the trigger.")

        mass = MassProperties(
            num(p, 4, "mass_properties.mass_liftoff", "kg", condition=lambda value: value > 0, expected="a positive mass in kg", defaulted_set=defaulted_parameters),
            num(p, 5, "mass_properties.mass_burnout", "kg", condition=lambda value: value > 0, expected="a positive mass in kg", defaulted_set=defaulted_parameters),
            num(p, 6, "mass_properties.mass_burnout_secondary", "kg", condition=lambda value: value > 0, expected="a positive mass in kg", defaulted_set=defaulted_parameters),
            num(p, 7, "mass_properties.pitch_inertia_liftoff", "kg*m^2", condition=lambda value: value > 0, expected="a positive pitch inertia in kg*m^2", defaulted_set=defaulted_parameters),
            num(p, 8, "mass_properties.pitch_inertia_burnout", "kg*m^2", condition=lambda value: value > 0, expected="a positive pitch inertia in kg*m^2", defaulted_set=defaulted_parameters),
            num(p, 9, "mass_properties.pitch_inertia_burnout_secondary", "kg*m^2", condition=lambda value: value > 0, expected="a positive pitch inertia in kg*m^2", defaulted_set=defaulted_parameters),
            num(p, 10, "mass_properties.cg_liftoff", "m", condition=lambda value: value >= 0, expected="a nonnegative CG position in m", defaulted_set=defaulted_parameters),
            num(p, 11, "mass_properties.cg_burnout", "m", condition=lambda value: value >= 0, expected="a nonnegative CG position in m", defaulted_set=defaulted_parameters),
            num(p, 12, "mass_properties.cg_burnout_secondary", "m", condition=lambda value: value >= 0, expected="a nonnegative CG position in m", defaulted_set=defaulted_parameters),
        )

        actuator = ActuatorConfig(
            num(p, 13, "actuator.motor_mount_position", "m", condition=lambda value: value >= 0, expected="a nonnegative axial position in m"),
            num(p, 14, "actuator.servo_resolution", "rad", "deg", condition=lambda value: value >= 0, expected="a nonnegative angle"),
            num(p, 15, "actuator.maximum_actuator_angle", "rad", "deg", condition=lambda value: value >= 0, expected="a nonnegative angle"),
            num(p, 16, "actuator.actuator_reduction", "1", condition=lambda value: value > 0, expected="a positive reduction ratio"),
            num(p, 17, "actuator.initial_misalignment", "rad", "deg"),
            num(p, 18, "actuator.servo_compensation", "1", condition=lambda value: value >= 0, expected="a nonnegative compensation factor"),
            num(p, 23, "actuator.motor_misalignment", "rad", "deg"),
        )

        environment = EnvironmentConfig(
            num(p, 19, "environment.mean_wind_speed", "m/s"),
            num(p, 20, "environment.wind_gust_standard_deviation", "m/s", condition=lambda value: value >= 0, expected="a nonnegative standard deviation in m/s"),
        )
        aerodynamics = AerodynamicConfig(
            num(p, 24, "aerodynamics.body_roughness", "m", "um", condition=lambda value: value >= 0, expected="a nonnegative roughness"),
            num(p, 25, "aerodynamics.stabilization_fin_roughness", "m", "um", condition=lambda value: value >= 0, expected="a nonnegative roughness"),
            num(p, 26, "aerodynamics.control_fin_roughness", "m", "um", condition=lambda value: value >= 0, expected="a nonnegative roughness"),
        )

        geometry_values = list(geometry)
        flags = [self._boolean(self._at(geometry, index), field=f"vehicle_geometry.{name}", source=self._source("geometry", index, name)) for index, name in enumerate(("ogive_nose", "stabilization_fins_enabled", "stabilization_fins_attached", "control_fins_enabled", "control_fins_attached"))]
        try:
            stabilization_marker = geometry_values.index("Fins_s")
            control_marker = geometry_values.index("Fins_c")
        except ValueError as exc:
            raise LegacyConfigError("Geometry section must contain Fins_s and Fins_c markers") from exc
        body_points = tuple(
            GeometryPoint(*self._pair(raw, field=f"vehicle_geometry.body_stations[{index}]", source=self._source("geometry", index + 5, "body station")))
            for index, raw in enumerate(geometry_values[5:stabilization_marker])
        )
        stabilization_block = geometry_values[stabilization_marker + 1:control_marker]
        control_block = geometry_values[control_marker + 1:]
        while len(stabilization_block) < 4:
            stabilization_block.append(None)
        while len(control_block) < 4:
            control_block.append(None)
        vehicle = VehicleGeometry(
            flags[0], flags[1], flags[2], flags[3], flags[4], body_points,
            self._fin(stabilization_block[:4], prefix="vehicle_geometry.stabilization_fin", base_index=stabilization_marker + 1),
            self._fin(control_block[:4], prefix="vehicle_geometry.control_fin", base_index=control_marker + 1),
        )
        if len(stabilization_block) > 4 or len(control_block) > 4:
            self._unsupported.append({"section": "geometry", "raw_value": geometry_values, "reason": "extra fin geometry values"})

        input_mode = string(c, 3, "simulation.input_mode", allowed=INPUT_MODES)
        input_unit = "rad/s" if input_mode.value == "Ramp [º/s]" else "rad"
        simulation = SimulationConfig(
            boolean(c, 0, "simulation.torque_controller"),
            boolean(c, 1, "simulation.anti_windup"),
            input_mode,
            num(c, 4, "simulation.kp", "1"), num(c, 5, "simulation.ki", "1"),
            num(c, 6, "simulation.kd", "1"), num(c, 7, "simulation.overall_gain", "1"),
            num(c, 8, "simulation.damping_gain", "1"),
            num(c, 9, "simulation.reference_thrust", "N", condition=lambda value: value >= 0, expected="a nonnegative force in N"),
            num(c, 10, "simulation.input_value", input_unit, "deg/s" if input_unit == "rad/s" else "deg"),
            num(c, 11, "simulation.input_time", "s", condition=lambda value: value >= 0, expected="a nonnegative timestamp in s"),
            num(c, 12, "simulation.launch_time", "s", condition=lambda value: value >= 0, expected="a nonnegative timestamp in s"),
            num(c, 13, "simulation.servo_sample_time", "s", condition=lambda value: value >= 0, expected="a nonnegative sample time in s"),
            num(c, 14, "simulation.controller_sample_time", "s", condition=lambda value: value >= 0, expected="a nonnegative sample time in s"),
            num(c, 15, "simulation.maximum_duration", "s", condition=lambda value: value > 0, expected="a positive duration in s"),
            num(c, 16, "simulation.integration_step", "s", condition=lambda value: value > 0, expected="a positive integration step in s", correction="Use a finite integration step greater than zero."),
            num(c, 17, "simulation.export_step", "s", condition=lambda value: value > 0, expected="a positive export step in s"),
        )
        recovery = RecoveryConfig(boolean(c, 2, "recovery.auto_parachute_on_tilt", defaulted_controller))
        launch = LaunchConfig(
            num(p, 21, "launch.launch_rod_length", "m", condition=lambda value: value >= 0, expected="a nonnegative rod length in m"),
            num(p, 22, "launch.launch_rod_angle", "rad", "deg"),
            num(c, 18, "launch.launch_altitude_msl", "m"),
            num(c, 19, "launch.initial_altitude_agl", "m"),
            num(c, 20, "launch.initial_vertical_velocity", "m/s"),
            num(c, 21, "launch.initial_horizontal_velocity", "m/s"),
            num(c, 22, "launch.initial_pitch_angle", "rad", "deg"),
            num(c, 23, "launch.initial_pitch_rate", "rad/s", "deg/s"),
        )
        display = DisplayConfig(
            boolean(d, 0, "display.enable_3d"), boolean(d, 1, "display.camera_shake"),
            boolean(d, 2, "display.hide_forces"), boolean(d, 3, "display.variable_fov"),
            boolean(d, 4, "display.hide_cg"),
            string(d, 5, "display.camera_type", allowed=CAMERA_TYPES),
            num(d, 6, "display.slow_motion_factor", "1", condition=lambda value: value > 0, expected="a positive slow-motion factor"),
            num(d, 7, "display.force_scale", "m/N", condition=lambda value: value >= 0, expected="a nonnegative force display scale"),
            num(d, 8, "display.field_of_view", "1", condition=lambda value: value > 0, expected="a positive field of view"),
            tuple(self._string(self._at(plots, index), field=f"display.plot_channels[{index}]", source=self._source("plots", index, PLOT_NAMES[index])) for index in range(10)),
        )
        sensors = SensorConfig(
            boolean(s, 0, "sensors.sitl_active"), boolean(s, 1, "sensors.noise_enabled"),
            boolean(s, 2, "sensors.python_sitl"),
            string(s, 3, "sensors.python_module", allow_empty=True),
            string(s, 4, "sensors.serial_port", allow_empty=True),
            self._integer(self._at(s, 5), field="sensors.baudrate", source=self._source("sitl", 5, SITL_NAMES[5]), positive=True),
            num(s, 6, "sensors.gyro_noise_sd", "rad/s", "deg/s", condition=lambda value: value >= 0, expected="a nonnegative standard deviation"),
            num(s, 7, "sensors.accelerometer_noise_sd", "m/s^2", "g0", condition=lambda value: value >= 0, expected="a nonnegative standard deviation"),
            num(s, 8, "sensors.altimeter_noise_sd", "m", condition=lambda value: value >= 0, expected="a nonnegative standard deviation"),
            num(s, 9, "sensors.gnss_position_noise_sd", "m", condition=lambda value: value >= 0, expected="a nonnegative standard deviation"),
            num(s, 10, "sensors.gnss_velocity_noise_sd", "m/s", condition=lambda value: value >= 0, expected="a nonnegative standard deviation"),
            num(s, 11, "sensors.gyro_sample_time", "s", condition=lambda value: value > 0, expected="a positive sample time in s"),
            num(s, 12, "sensors.accelerometer_sample_time", "s", condition=lambda value: value > 0, expected="a positive sample time in s"),
            num(s, 13, "sensors.altimeter_sample_time", "s", condition=lambda value: value > 0, expected="a positive sample time in s"),
            num(s, 14, "sensors.gnss_sample_time", "s", condition=lambda value: value > 0, expected="a positive sample time in s"),
        )

        return CanonicalConfig(
            vehicle, mass, aerodynamics,
            PropulsionConfig(primary_motor, SecondaryMotorConfig(secondary_file, trigger_mode, trigger_value)),
            environment, launch, simulation, display, recovery, actuator, sensors,
            created_at=created_at, source=source,
            validation_issues=tuple(self._issues), unsupported_data=tuple(self._unsupported),
            comments=tuple(self._comments),
        )

    @staticmethod
    def _legacy_value(value: EngineeringValue[Any], *, unit: str | None = None) -> str:
        if value.validation_state in {ValidationState.INVALID, ValidationState.MISSING} and value.original_value is not None:
            return str(value.original_value)
        if value.value is None:
            return ""
        if isinstance(value.value, bool):
            return "True" if value.value else "False"
        if isinstance(value.value, str):
            return value.value
        target_unit = unit if unit is not None else value.original_unit if value.original_unit is not None else value.unit
        numeric = convert_value(float(value.value), value.unit, target_unit) if value.unit != target_unit else float(value.value)
        return _format_number(numeric)

    @classmethod
    def to_legacy_sections(cls, config: CanonicalConfig) -> LegacySections:
        v = cls._legacy_value
        m, a, e, launch, sim, display, recovery, actuator, sensors = (
            config.mass_properties, config.aerodynamics, config.environment,
            config.launch, config.simulation, config.display, config.recovery,
            config.actuator, config.sensors,
        )
        secondary = config.propulsion.secondary_motor
        parameters = (
            v(config.propulsion.primary_motor_file), v(secondary.motor_file), v(secondary.trigger_mode), v(secondary.trigger_value),
            v(m.mass_liftoff), v(m.mass_burnout), v(m.mass_burnout_secondary),
            v(m.pitch_inertia_liftoff), v(m.pitch_inertia_burnout), v(m.pitch_inertia_burnout_secondary),
            v(m.cg_liftoff), v(m.cg_burnout), v(m.cg_burnout_secondary),
            v(actuator.motor_mount_position), v(actuator.servo_resolution, unit="deg"),
            v(actuator.maximum_actuator_angle, unit="deg"), v(actuator.actuator_reduction),
            v(actuator.initial_misalignment, unit="deg"), v(actuator.servo_compensation),
            v(e.mean_wind_speed), v(e.wind_gust_standard_deviation), v(launch.launch_rod_length),
            v(launch.launch_rod_angle, unit="deg"), v(actuator.motor_misalignment, unit="deg"),
            v(a.body_roughness, unit="um"), v(a.stabilization_fin_roughness, unit="um"),
            v(a.control_fin_roughness, unit="um"),
        )
        controller = (
            v(sim.torque_controller), v(sim.anti_windup), v(recovery.auto_parachute_on_tilt),
            v(sim.input_mode), v(sim.kp), v(sim.ki), v(sim.kd), v(sim.overall_gain),
            v(sim.damping_gain), v(sim.reference_thrust),
            v(sim.input_value, unit="deg/s" if sim.input_mode.value == "Ramp [º/s]" else "deg"),
            v(sim.input_time), v(sim.launch_time), v(sim.servo_sample_time),
            v(sim.controller_sample_time), v(sim.maximum_duration), v(sim.integration_step),
            v(sim.export_step), v(launch.launch_altitude_msl), v(launch.initial_altitude_agl),
            v(launch.initial_vertical_velocity), v(launch.initial_horizontal_velocity),
            v(launch.initial_pitch_angle, unit="deg"), v(launch.initial_pitch_rate, unit="deg/s"),
        )
        display_values = (
            v(display.enable_3d), v(display.camera_shake), v(display.hide_forces),
            v(display.variable_fov), v(display.hide_cg), v(display.camera_type),
            v(display.slow_motion_factor), v(display.force_scale), v(display.field_of_view),
        )
        sitl = (
            v(sensors.sitl_active), v(sensors.noise_enabled), v(sensors.python_sitl),
            v(sensors.python_module), v(sensors.serial_port), v(sensors.baudrate),
            v(sensors.gyro_noise_sd, unit="deg/s"), v(sensors.accelerometer_noise_sd, unit="g0"),
            v(sensors.altimeter_noise_sd), v(sensors.gnss_position_noise_sd),
            v(sensors.gnss_velocity_noise_sd), v(sensors.gyro_sample_time),
            v(sensors.accelerometer_sample_time), v(sensors.altimeter_sample_time),
            v(sensors.gnss_sample_time),
        )
        geometry_model = config.vehicle_geometry
        geometry: list[str] = [
            v(geometry_model.ogive_nose), v(geometry_model.stabilization_fins_enabled),
            v(geometry_model.stabilization_fins_attached), v(geometry_model.control_fins_enabled),
            v(geometry_model.control_fins_attached),
        ]
        for point in geometry_model.body_stations:
            geometry.append(f"{v(point.axial_position)},{v(point.diameter)}")

        def append_fin(marker: str, fin: FinGeometry) -> None:
            geometry.extend(
                [
                    marker,
                    f"{v(fin.root_leading_position)}, {v(fin.root_chord)}",
                    f"{v(fin.tip_leading_position)}, {v(fin.tip_chord)}",
                    v(fin.span), v(fin.thickness),
                ]
            )

        append_fin("Fins_s", geometry_model.stabilization_fin)
        append_fin("Fins_c", geometry_model.control_fin)
        return LegacySections(
            parameters, display_values, controller, sitl,
            tuple(v(channel) for channel in display.plot_channels), tuple(geometry),
        )

    def canonicalize_sections(self, sections: LegacySections, *, source: str | None = None) -> tuple[CanonicalConfig, LegacySections]:
        config = self.from_sections(
            sections,
            source=source,
            value_state=ValidationState.USER_SUPPLIED,
        )
        return config, self.to_legacy_sections(config)
