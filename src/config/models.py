"""Typed AV1 canonical configuration models.

The models describe configuration only. They do not implement or alter any
physics, integration, event, controller, actuator, sensor, or plotting logic.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass
from enum import Enum
import hashlib
import json
from typing import Any, Generic, TypeVar

from src.config.frames import FRAME_CONVENTIONS


SCHEMA_VERSION = "1.0.0"
IMPORTER_VERSION = "aerovector-legacy-text/1.0.0"
T = TypeVar("T")


class ValidationState(str, Enum):
    MISSING = "MISSING"
    INVALID = "INVALID"
    DEFAULTED_LEGACY = "DEFAULTED_LEGACY"
    USER_SUPPLIED = "USER_SUPPLIED"
    IMPORTED = "IMPORTED"


@dataclass(frozen=True)
class ValidationIssue:
    field: str
    received_value: Any
    expected_condition: str
    source: str | None
    recommended_correction: str
    severity: str = "error"


@dataclass(frozen=True)
class EngineeringValue(Generic[T]):
    value: T | None
    unit: str | None
    source: str | None
    uncertainty: Any = None
    provenance: dict[str, Any] | None = None
    notes: str | None = None
    validation_state: ValidationState = ValidationState.USER_SUPPLIED
    original_value: Any = None
    original_unit: str | None = None
    conversion: dict[str, Any] | None = None


@dataclass(frozen=True)
class GeometryPoint:
    axial_position: EngineeringValue[float]
    diameter: EngineeringValue[float]


@dataclass(frozen=True)
class FinGeometry:
    root_leading_position: EngineeringValue[float]
    root_chord: EngineeringValue[float]
    tip_leading_position: EngineeringValue[float]
    tip_chord: EngineeringValue[float]
    span: EngineeringValue[float]
    thickness: EngineeringValue[float]


@dataclass(frozen=True)
class VehicleGeometry:
    ogive_nose: EngineeringValue[bool]
    stabilization_fins_enabled: EngineeringValue[bool]
    stabilization_fins_attached: EngineeringValue[bool]
    control_fins_enabled: EngineeringValue[bool]
    control_fins_attached: EngineeringValue[bool]
    body_stations: tuple[GeometryPoint, ...]
    stabilization_fin: FinGeometry
    control_fin: FinGeometry


@dataclass(frozen=True)
class MassProperties:
    mass_liftoff: EngineeringValue[float]
    mass_burnout: EngineeringValue[float]
    mass_burnout_secondary: EngineeringValue[float]
    pitch_inertia_liftoff: EngineeringValue[float]
    pitch_inertia_burnout: EngineeringValue[float]
    pitch_inertia_burnout_secondary: EngineeringValue[float]
    cg_liftoff: EngineeringValue[float]
    cg_burnout: EngineeringValue[float]
    cg_burnout_secondary: EngineeringValue[float]


@dataclass(frozen=True)
class AerodynamicConfig:
    body_roughness: EngineeringValue[float]
    stabilization_fin_roughness: EngineeringValue[float]
    control_fin_roughness: EngineeringValue[float]


@dataclass(frozen=True)
class SecondaryMotorConfig:
    motor_file: EngineeringValue[str]
    trigger_mode: EngineeringValue[str]
    trigger_value: EngineeringValue[float]


@dataclass(frozen=True)
class PropulsionConfig:
    primary_motor_file: EngineeringValue[str]
    secondary_motor: SecondaryMotorConfig


@dataclass(frozen=True)
class EnvironmentConfig:
    mean_wind_speed: EngineeringValue[float]
    wind_gust_standard_deviation: EngineeringValue[float]


@dataclass(frozen=True)
class LaunchConfig:
    launch_rod_length: EngineeringValue[float]
    launch_rod_angle: EngineeringValue[float]
    launch_altitude_msl: EngineeringValue[float]
    initial_altitude_agl: EngineeringValue[float]
    initial_vertical_velocity: EngineeringValue[float]
    initial_horizontal_velocity: EngineeringValue[float]
    initial_pitch_angle: EngineeringValue[float]
    initial_pitch_rate: EngineeringValue[float]


@dataclass(frozen=True)
class SimulationConfig:
    torque_controller: EngineeringValue[bool]
    anti_windup: EngineeringValue[bool]
    input_mode: EngineeringValue[str]
    kp: EngineeringValue[float]
    ki: EngineeringValue[float]
    kd: EngineeringValue[float]
    overall_gain: EngineeringValue[float]
    damping_gain: EngineeringValue[float]
    reference_thrust: EngineeringValue[float]
    input_value: EngineeringValue[float]
    input_time: EngineeringValue[float]
    launch_time: EngineeringValue[float]
    servo_sample_time: EngineeringValue[float]
    controller_sample_time: EngineeringValue[float]
    maximum_duration: EngineeringValue[float]
    integration_step: EngineeringValue[float]
    export_step: EngineeringValue[float]


@dataclass(frozen=True)
class DisplayConfig:
    enable_3d: EngineeringValue[bool]
    camera_shake: EngineeringValue[bool]
    hide_forces: EngineeringValue[bool]
    variable_fov: EngineeringValue[bool]
    hide_cg: EngineeringValue[bool]
    camera_type: EngineeringValue[str]
    slow_motion_factor: EngineeringValue[float]
    force_scale: EngineeringValue[float]
    field_of_view: EngineeringValue[float]
    plot_channels: tuple[EngineeringValue[str], ...]


@dataclass(frozen=True)
class RecoveryConfig:
    auto_parachute_on_tilt: EngineeringValue[bool]


@dataclass(frozen=True)
class ActuatorConfig:
    motor_mount_position: EngineeringValue[float]
    servo_resolution: EngineeringValue[float]
    maximum_actuator_angle: EngineeringValue[float]
    actuator_reduction: EngineeringValue[float]
    initial_misalignment: EngineeringValue[float]
    servo_compensation: EngineeringValue[float]
    motor_misalignment: EngineeringValue[float]


@dataclass(frozen=True)
class SensorConfig:
    sitl_active: EngineeringValue[bool]
    noise_enabled: EngineeringValue[bool]
    python_sitl: EngineeringValue[bool]
    python_module: EngineeringValue[str]
    serial_port: EngineeringValue[str]
    baudrate: EngineeringValue[int]
    gyro_noise_sd: EngineeringValue[float]
    accelerometer_noise_sd: EngineeringValue[float]
    altimeter_noise_sd: EngineeringValue[float]
    gnss_position_noise_sd: EngineeringValue[float]
    gnss_velocity_noise_sd: EngineeringValue[float]
    gyro_sample_time: EngineeringValue[float]
    accelerometer_sample_time: EngineeringValue[float]
    altimeter_sample_time: EngineeringValue[float]
    gnss_sample_time: EngineeringValue[float]


def _plain(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {item.name: _plain(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, tuple):
        return [_plain(item) for item in value]
    if isinstance(value, list):
        return [_plain(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))}
    return value


def _strip_nonsemantic(value: Any) -> Any:
    if isinstance(value, dict):
        stripped = {
            key: _strip_nonsemantic(item)
            for key, item in value.items()
            if key not in {
                "config_hash",
                "created_at",
                "source",
                "importer_version",
                "provenance",
                "notes",
                "original_value",
                "original_unit",
                "conversion",
                "validation_issues",
                "comments",
            }
        }
        # Origin state does not change a valid engineering value's meaning.
        # Preserve MISSING/INVALID because those states are semantically distinct.
        if stripped.get("validation_state") in {
            ValidationState.DEFAULTED_LEGACY.value,
            ValidationState.IMPORTED.value,
            ValidationState.USER_SUPPLIED.value,
        }:
            stripped["validation_state"] = "VALID"
        return stripped
    if isinstance(value, list):
        return [_strip_nonsemantic(item) for item in value]
    return value


@dataclass(frozen=True)
class CanonicalConfig:
    vehicle_geometry: VehicleGeometry
    mass_properties: MassProperties
    aerodynamics: AerodynamicConfig
    propulsion: PropulsionConfig
    environment: EnvironmentConfig
    launch: LaunchConfig
    simulation: SimulationConfig
    display: DisplayConfig
    recovery: RecoveryConfig
    actuator: ActuatorConfig
    sensors: SensorConfig
    schema_version: str = SCHEMA_VERSION
    config_hash: str | None = None
    created_at: str | None = None
    source: str | None = None
    importer_version: str = IMPORTER_VERSION
    frame_conventions: dict[str, str] = field(default_factory=lambda: dict(FRAME_CONVENTIONS))
    validation_issues: tuple[ValidationIssue, ...] = ()
    unsupported_data: tuple[dict[str, Any], ...] = ()
    comments: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        payload = _plain(self)
        payload["config_hash"] = self.compute_hash()
        return payload

    def semantic_dict(self) -> dict[str, Any]:
        return _strip_nonsemantic(_plain(self))

    def to_json(self) -> str:
        return json.dumps(
            self.to_dict(),
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )

    def semantic_json(self) -> str:
        return json.dumps(
            self.semantic_dict(),
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )

    def compute_hash(self) -> str:
        return hashlib.sha256(self.semantic_json().encode("utf-8")).hexdigest()

    @property
    def is_valid(self) -> bool:
        return not any(issue.severity == "error" for issue in self.validation_issues)
