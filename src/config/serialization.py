"""Deterministic canonical configuration serialization helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from src.config.migrations import migrate_payload
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


def serialize_config(config: CanonicalConfig, *, pretty: bool = False) -> str:
    if not pretty:
        return config.to_json()
    return json.dumps(
        config.to_dict(),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        indent=2,
    ) + "\n"


def write_config(config: CanonicalConfig, path: str | Path, *, pretty: bool = True) -> None:
    Path(path).write_text(serialize_config(config, pretty=pretty), encoding="utf-8")


def deserialize_payload(serialized: str) -> dict[str, Any]:
    """Read and minimally verify a serialized payload.

    Typed schema migration is intentionally separate: AV1 supports schema
    1.0.0 and exposes the payload for a future registered migration chain.
    """
    payload = json.loads(serialized)
    if not isinstance(payload, dict):
        raise ValueError("Canonical configuration must be a JSON object")
    if "schema_version" not in payload:
        raise ValueError("Canonical configuration is missing schema_version")
    if "config_hash" not in payload:
        raise ValueError("Canonical configuration is missing config_hash")
    return payload


def _engineering_value(payload: dict[str, Any]) -> EngineeringValue[Any]:
    data = dict(payload)
    data["validation_state"] = ValidationState(data["validation_state"])
    return EngineeringValue(**data)


def _fin(payload: dict[str, Any]) -> FinGeometry:
    return FinGeometry(**{key: _engineering_value(value) for key, value in payload.items()})


def deserialize_config(serialized: str) -> CanonicalConfig:
    """Deserialize schema 1.0.0 JSON into the typed canonical model."""
    payload = migrate_payload(deserialize_payload(serialized))

    vehicle_payload = payload["vehicle_geometry"]
    vehicle = VehicleGeometry(
        ogive_nose=_engineering_value(vehicle_payload["ogive_nose"]),
        stabilization_fins_enabled=_engineering_value(vehicle_payload["stabilization_fins_enabled"]),
        stabilization_fins_attached=_engineering_value(vehicle_payload["stabilization_fins_attached"]),
        control_fins_enabled=_engineering_value(vehicle_payload["control_fins_enabled"]),
        control_fins_attached=_engineering_value(vehicle_payload["control_fins_attached"]),
        body_stations=tuple(
            GeometryPoint(
                _engineering_value(point["axial_position"]),
                _engineering_value(point["diameter"]),
            )
            for point in vehicle_payload["body_stations"]
        ),
        stabilization_fin=_fin(vehicle_payload["stabilization_fin"]),
        control_fin=_fin(vehicle_payload["control_fin"]),
    )

    def values(model, key: str):
        return model(**{name: _engineering_value(value) for name, value in payload[key].items()})

    secondary_payload = payload["propulsion"]["secondary_motor"]
    propulsion = PropulsionConfig(
        primary_motor_file=_engineering_value(payload["propulsion"]["primary_motor_file"]),
        secondary_motor=SecondaryMotorConfig(
            **{key: _engineering_value(value) for key, value in secondary_payload.items()}
        ),
    )
    display_payload = payload["display"]
    display = DisplayConfig(
        **{
            key: tuple(_engineering_value(item) for item in value)
            if key == "plot_channels"
            else _engineering_value(value)
            for key, value in display_payload.items()
        }
    )
    issues = tuple(ValidationIssue(**issue) for issue in payload.get("validation_issues", []))
    config = CanonicalConfig(
        vehicle_geometry=vehicle,
        mass_properties=values(MassProperties, "mass_properties"),
        aerodynamics=values(AerodynamicConfig, "aerodynamics"),
        propulsion=propulsion,
        environment=values(EnvironmentConfig, "environment"),
        launch=values(LaunchConfig, "launch"),
        simulation=values(SimulationConfig, "simulation"),
        display=display,
        recovery=values(RecoveryConfig, "recovery"),
        actuator=values(ActuatorConfig, "actuator"),
        sensors=values(SensorConfig, "sensors"),
        schema_version=payload["schema_version"],
        config_hash=payload.get("config_hash"),
        created_at=payload.get("created_at"),
        source=payload.get("source"),
        importer_version=payload.get("importer_version", ""),
        frame_conventions=dict(payload.get("frame_conventions", {})),
        validation_issues=issues,
        unsupported_data=tuple(payload.get("unsupported_data", [])),
        comments=tuple(payload.get("comments", [])),
    )
    expected_hash = payload.get("config_hash")
    if expected_hash != config.compute_hash():
        raise ValueError(
            f"Canonical configuration hash mismatch: stored={expected_hash!r}, "
            f"computed={config.compute_hash()!r}"
        )
    return config
