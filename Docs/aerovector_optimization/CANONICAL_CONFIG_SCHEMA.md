# Canonical Configuration Schema

AV1 defines schema `1.0.0` in `src/config`. It is a configuration boundary only: it does not contain equations, integration, event detection, controller execution, SITL transport, or plotting behavior.

## Root model

`CanonicalConfig` owns these typed sections:

| Section | Model | Purpose |
|---|---|---|
| `vehicle_geometry` | `VehicleGeometry` | Body stations, fin geometry, and legacy geometry flags |
| `mass_properties` | `MassProperties` | Liftoff/burnout mass, pitch inertia, and CG states |
| `aerodynamics` | `AerodynamicConfig` | Surface roughness inputs |
| `propulsion` | `PropulsionConfig` | Primary motor plus `SecondaryMotorConfig` |
| `environment` | `EnvironmentConfig` | Mean wind and gust standard deviation |
| `launch` | `LaunchConfig` | Launch rail and initial state |
| `simulation` | `SimulationConfig` | Controller parameters, timing, and integration/export settings |
| `display` | `DisplayConfig` | Existing 3D and plot selections |
| `recovery` | `RecoveryConfig` | Existing automatic parachute option |
| `actuator` | `ActuatorConfig` | Existing servo and motor-mount configuration only |
| `sensors` | `SensorConfig` | Existing SITL and sensor configuration only |

The root also records `schema_version`, computed `config_hash`, source/import metadata, frame conventions, validation issues, unsupported legacy data, and comments.

## Engineering values

Every engineering input is an `EngineeringValue[T]` with:

| Property | Meaning |
|---|---|
| `value` | Parsed canonical value, or `null` when missing/invalid |
| `unit` | Explicit canonical unit; `null` for booleans/enums/text |
| `source` | Source section, zero-based legacy position, and legacy label |
| `uncertainty` | Reserved explicit uncertainty; no uncertainty is invented |
| `provenance` | Structured import/default information |
| `notes` | Optional engineering note |
| `validation_state` | `MISSING`, `INVALID`, `DEFAULTED_LEGACY`, `USER_SUPPLIED`, or `IMPORTED` |
| `original_value` | Loss-aware raw legacy value |
| `original_unit` | Unit used by the legacy field |
| `conversion` | Explicit conversion metadata when canonicalization changes units |

`DEFAULTED_LEGACY` is used only for the compatibility fields inserted when a 21-parameter/23-controller legacy file is upgraded. Missing and invalid values never silently become zeros.

## Validation contract

Each `ValidationIssue` reports:

- canonical field path;
- received value;
- expected condition;
- source location;
- recommended correction;
- severity.

Validation covers section shape, required values, finite numerics, positive/nonnegative ranges, supported enums and booleans, known units/conversions, secondary-motor completeness, motor-file references when a motor directory is provided, and required geometry markers.

Unknown trailing fields are retained in `unsupported_data`; they are not discarded or interpreted as known fields. Invalid raw values are retained for loss-aware regeneration.

## Serialization and hashing

`serialize_config()` emits UTF-8 JSON with sorted keys, no insignificant whitespace, and no NaN/Infinity. `deserialize_config()` restores the typed model and rejects a stored/computed hash mismatch.

The SHA-256 hash is calculated from canonical semantic JSON. It includes typed values, canonical units, missing/invalid state, schema version, frames, and unsupported data. It excludes timestamps, source paths, importer metadata, comments, diagnostics, raw spellings, and conversion/provenance notes. Valid origin states (`IMPORTED`, `USER_SUPPLIED`, `DEFAULTED_LEGACY`) normalize to `VALID`, so semantically identical configurations hash identically regardless of origin.

## Versioning and migration

The schema follows semantic versioning. `migrations.py` provides a deterministic linear registry:

1. register one next migration for a source version;
2. copy the input payload before migration;
3. require each step to set its declared target version;
4. detect cycles and missing steps;
5. deserialize and verify the resulting current-schema hash.

AV1 registers no historical canonical migrations because `1.0.0` is the first canonical schema. Unknown versions fail explicitly rather than being guessed.

## Compatibility boundary

`src/files.py:SaveFile` parses through `LegacyConfigAdapter`, stores the typed configuration and validation issues, then regenerates the established positional lists consumed by the GUI and simulator. Existing downstream interfaces are unchanged. Saving routes the current positional GUI values through the same canonical validation/normalization boundary before writing.
