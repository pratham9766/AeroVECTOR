"""AV1 canonical configuration unit and compatibility tests."""

from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path
import unittest

from src import files
from src.config import (
    LegacyConfigAdapter,
    LegacyConfigError,
    LegacySections,
    ValidationState,
    UnitError,
    convert_value,
    deserialize_config,
    migrate_payload,
    serialize_config,
    validate_unit,
)


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "3 - Examples" / "Others" / "Example Rocket TVC.txt"
MOTORS = ROOT / "Motors"


class CanonicalConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.adapter = LegacyConfigAdapter(MOTORS)
        self.text = EXAMPLE.read_text(encoding="utf-8")

    def test_known_legacy_file_parses_into_typed_si_model(self) -> None:
        config = self.adapter.parse_file(EXAMPLE)
        self.assertTrue(config.is_valid)
        self.assertEqual(config.mass_properties.mass_liftoff.value, 0.650)
        self.assertEqual(config.mass_properties.mass_liftoff.unit, "kg")
        self.assertAlmostEqual(config.actuator.maximum_actuator_angle.value, math.radians(10))
        self.assertEqual(config.actuator.maximum_actuator_angle.unit, "rad")
        self.assertAlmostEqual(config.aerodynamics.body_roughness.value, 60e-6)

    def test_old_layout_is_upgraded_with_explicit_default_states(self) -> None:
        config = self.adapter.parse_text(self.text)
        self.assertEqual(
            config.propulsion.secondary_motor.motor_file.validation_state,
            ValidationState.DEFAULTED_LEGACY,
        )
        self.assertEqual(
            config.recovery.auto_parachute_on_tilt.validation_state,
            ValidationState.DEFAULTED_LEGACY,
        )
        legacy = self.adapter.to_legacy_sections(config)
        self.assertEqual(len(legacy.parameters), 27)
        self.assertEqual(len(legacy.controller), 24)

    def test_malformed_section_structure_is_rejected(self) -> None:
        with self.assertRaisesRegex(LegacyConfigError, "Expected six legacy sections"):
            self.adapter.parse_text("Motor = sample.csv\n###=#\nFalse\n")

    def test_missing_and_invalid_values_are_distinguished_and_reported(self) -> None:
        missing = self.adapter.parse_text(self.text.replace("Mass Liftoff = 0.650", "Mass Liftoff = "))
        value = missing.mass_properties.mass_liftoff
        self.assertEqual(value.validation_state, ValidationState.MISSING)
        issue = next(i for i in missing.validation_issues if i.field == "mass_properties.mass_liftoff")
        self.assertIn("positive mass", issue.expected_condition)
        self.assertTrue(issue.source.startswith("legacy:parameters[4]"))
        self.assertTrue(issue.recommended_correction)

        invalid = self.adapter.parse_text(self.text.replace("Mass Liftoff = 0.650", "Mass Liftoff = banana"))
        value = invalid.mass_properties.mass_liftoff
        self.assertEqual(value.validation_state, ValidationState.INVALID)
        self.assertEqual(value.original_value, "banana")
        self.assertEqual(self.adapter.to_legacy_sections(invalid).parameters[4], "banana")

    def test_extra_fields_are_preserved_as_unsupported_data(self) -> None:
        sections = self.adapter.parse_sections(self.text)
        changed = replace(sections, display=sections.display + ("future-display-value",))
        config = self.adapter.from_sections(changed)
        self.assertIn(
            {"section": "display", "position": 9, "raw_value": "future-display-value", "reason": "extra legacy field"},
            config.unsupported_data,
        )

    def test_comments_are_preserved_without_shifting_positions(self) -> None:
        commented = "# configuration note\n" + self.text
        config = self.adapter.parse_text(commented)
        self.assertEqual(config.comments, ("# configuration note",))
        self.assertEqual(config.mass_properties.mass_liftoff.value, 0.650)

    def test_missing_motor_file_reference_is_reported(self) -> None:
        config = self.adapter.parse_text(
            self.text.replace("Motor = Estes_F15.csv", "Motor = definitely_missing.csv")
        )
        issue = next(i for i in config.validation_issues if i.field == "propulsion.primary_motor_file")
        self.assertIn("existing motor data file", issue.expected_condition)

    def test_loss_aware_legacy_round_trip_preserves_supported_values(self) -> None:
        first = self.adapter.parse_text(self.text)
        legacy = self.adapter.to_legacy_sections(first)
        second = self.adapter.from_sections(legacy)
        self.assertEqual(first.compute_hash(), second.compute_hash())
        self.assertEqual(legacy, self.adapter.to_legacy_sections(second))

    def test_gui_compatibility_path_marks_values_user_supplied(self) -> None:
        sections = self.adapter.parse_sections(self.text)
        config, _ = self.adapter.canonicalize_sections(sections, source="legacy-gui")
        self.assertEqual(
            config.mass_properties.mass_liftoff.validation_state,
            ValidationState.USER_SUPPLIED,
        )

    def test_deterministic_serialization_hash_and_source_independence(self) -> None:
        first = self.adapter.parse_text(self.text, source="first.txt")
        second = self.adapter.parse_text(self.text, source="second.txt")
        repeated = self.adapter.parse_text(self.text, source="first.txt")
        self.assertEqual(first.compute_hash(), second.compute_hash())
        self.assertEqual(first.semantic_json(), second.semantic_json())
        self.assertEqual(serialize_config(first), serialize_config(repeated))
        self.assertEqual(first.compute_hash(), first.to_dict()["config_hash"])

    def test_typed_json_round_trip_and_tamper_detection(self) -> None:
        config = self.adapter.parse_text(self.text)
        serialized = serialize_config(config)
        restored = deserialize_config(serialized)
        self.assertEqual(serialized, serialize_config(restored))
        payload = json.loads(serialized)
        payload["mass_properties"]["mass_liftoff"]["value"] = 999
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            deserialize_config(json.dumps(payload))

    def test_schema_migration_registry_rejects_unknown_version(self) -> None:
        with self.assertRaisesRegex(ValueError, "No migration registered"):
            migrate_payload({"schema_version": "0.9.0"})

    def test_unit_validation_and_conversions(self) -> None:
        validate_unit("rad")
        with self.assertRaises(UnitError):
            validate_unit("furlong/fortnight")
        self.assertAlmostEqual(convert_value(180, "deg", "rad"), math.pi)
        self.assertAlmostEqual(convert_value(1, "g0", "m/s^2"), 9.80665)
        self.assertAlmostEqual(convert_value(convert_value(3.5, "deg/s", "rad/s"), "rad/s", "deg/s"), 3.5)

    def test_savefile_compatibility_bridge_keeps_legacy_gui_lists(self) -> None:
        save = files.SaveFile()
        save.update_path(str(EXAMPLE))
        save.read_file()
        self.assertIsNotNone(save.get_canonical_config())
        self.assertEqual(len(save.get_parameters()), 27)
        self.assertEqual(len(save.get_conf_3d()), 9)
        self.assertEqual(len(save.get_conf_controller()), 24)
        self.assertEqual(len(save.get_conf_sitl()), 15)
        self.assertEqual(len(save.get_conf_plots()), 10)
        self.assertGreater(len(save.get_rocket_dim()), 5)


if __name__ == "__main__":
    unittest.main()
