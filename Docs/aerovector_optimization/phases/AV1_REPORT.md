# AV1 Report — Canonical Configuration Model

Status: **complete**

Scope: canonical configuration only
Schema: `1.0.0`

## Delivered

- Typed immutable models for vehicle geometry, mass properties, aerodynamics, propulsion/secondary motor, environment, launch, simulation, display, recovery, actuator, and sensors.
- `EngineeringValue` metadata for value, unit, source, uncertainty, provenance, notes, validation state, original value/unit, and conversion.
- Explicit SI normalization and documented legacy frame/sign conventions.
- Loss-aware adapter for all six legacy text sections, including the historical 21→27 parameter and 23→24 controller upgrades.
- Structured validation issues with field, received value, condition, source, correction, and severity.
- Deterministic JSON, semantic SHA-256 hashes, typed deserialization, tamper checks, and a versioned migration registry.
- `SaveFile` compatibility bridge. GUI and simulator consumers retain their existing positional-list interfaces.
- Complete schema, positional mapping, units, frames, and migration documentation.

No physics, aerodynamics, propulsion behavior, event logic, integrator, controller execution, actuator model, sensor model, SITL protocol, or plotting implementation was changed.

## Validation evidence

The test suite uses the standard-library `unittest` runner because pytest is not present in the project environment. Command:

```text
.venv\Scripts\python.exe -m unittest discover -s tests -p 'test_*.py' -v
```

Result: **15 tests passed**. Coverage includes known-file parsing, comment preservation, typed SI values, malformed section structure, missing vs invalid values, rich diagnostics, missing file references, extra-field preservation, legacy defaults, user-supplied provenance, supported and rejected units, round-trip conversions, deterministic source-independent hashes, typed JSON round-trip, tamper rejection, unknown schema rejection, GUI-list compatibility, and frozen numerical outputs.

## Exact baseline comparison

All three AV0 scenarios were rerun after `SaveFile` began routing configuration through the canonical model. The numerical artifacts remained byte-for-byte identical:

| Scenario | Artifact | AV0 SHA-256 | AV1 rerun | Result |
|---|---|---|---|---|
| Dhruva dual-motor SITL | `aerodynamics.json` | `a559115c717f1ce528009a0ce36c41c1a9380d84e5981ab8cb0971f962cc1323` | same | exact |
| Dhruva dual-motor SITL | `trace.csv` | `6fb72dd956b975a6aef881c69ebb2a6b4ff88bee1f894b83191364596c4e53bd` | same | exact |
| Legacy active fins | `aerodynamics.json` | `def7c97dbc9bd1d7903b8459a8af5cefd95262e3d1bd74b5b69988072d4cc6a8` | same | exact |
| Legacy active fins | `trace.csv` | `e51c281f76aec188a8acdf9e69ef5d302925c5067e4b120895406119cbfa61cc` | same | exact |
| Legacy TVC | `aerodynamics.json` | `29ee037ff1e905bd1f3e67ba8f5d879c5be3be2f69ba416cd0dbed0133d90b29` | same | exact |
| Legacy TVC | `trace.csv` | `64bc3e3156a0d006199fa3dfb068bcf5f3d0f335d2b9f62b7e124b4e6d7eb0f3` | same | exact |

The active-fin run still reports the frozen known defect `KNOWN_DEFECT_EVENT_APOGEE_001`: apogee is declared near 31.54 m at 4.88 s after a temporary vertical-velocity sign change, although the later trajectory reaches about 61.56 m while climbing. AV1 intentionally does not repair or mask it.

## Compatibility and limitations

- The canonical model is authoritative at load/save boundaries, while established runtime code continues to receive legacy lists. Direct migration of runtime consumers belongs to a later phase.
- Unknown trailing values are preserved as unsupported data in canonical JSON, but the legacy writer has no stable label syntax for reinserting unknown positions into arbitrary future formats.
- Comments are retained in canonical metadata; the current legacy writer continues using its established template labels rather than preserving arbitrary input formatting.
- Interactive rendering was not altered. Automated `SaveFile` bridge checks confirm every GUI-consumed list has its expected shape.

## AV2 readiness

AV2 can consume `CanonicalConfig` through `SaveFile.get_canonical_config()` and can use semantic hashes for reproducibility. Any migration of simulator internals should be incremental and must retain the six exact baseline locks. AV1 stops here as required.
