"""AeroVECTOR canonical configuration public API."""

from src.config.legacy_adapter import LegacyConfigAdapter, LegacyConfigError, LegacySections
from src.config.migrations import MIGRATIONS, migrate_payload, register_migration
from src.config.models import (
    ActuatorConfig,
    AerodynamicConfig,
    CanonicalConfig,
    DisplayConfig,
    EngineeringValue,
    EnvironmentConfig,
    LaunchConfig,
    MassProperties,
    PropulsionConfig,
    RecoveryConfig,
    SCHEMA_VERSION,
    SecondaryMotorConfig,
    SensorConfig,
    SimulationConfig,
    ValidationIssue,
    ValidationState,
    VehicleGeometry,
)
from src.config.serialization import deserialize_config, serialize_config, write_config
from src.config.units import UnitError, convert_value, validate_unit

__all__ = [
    "ActuatorConfig",
    "AerodynamicConfig",
    "CanonicalConfig",
    "DisplayConfig",
    "EngineeringValue",
    "EnvironmentConfig",
    "LaunchConfig",
    "LegacyConfigAdapter",
    "LegacyConfigError",
    "LegacySections",
    "MIGRATIONS",
    "MassProperties",
    "PropulsionConfig",
    "RecoveryConfig",
    "SCHEMA_VERSION",
    "SecondaryMotorConfig",
    "SensorConfig",
    "SimulationConfig",
    "ValidationIssue",
    "ValidationState",
    "VehicleGeometry",
    "UnitError",
    "convert_value",
    "deserialize_config",
    "migrate_payload",
    "register_migration",
    "serialize_config",
    "validate_unit",
    "write_config",
]
