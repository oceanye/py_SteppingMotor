import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from motor_control.state_store import StateStore, StateStoreError


class StateStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.store = StateStore(self.root)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_missing_files_return_none(self):
        self.assertIsNone(self.store.load_axis_config())
        self.assertIsNone(self.store.load_calibration())
        self.assertIsNone(self.store.load_foc_tune())
        self.assertIsNone(self.store.load_gear_tune())
        self.assertIsNone(self.store.load_coordinated_bindings())

    def test_legacy_axis_config_round_trip_keeps_schema(self):
        expected = {
            "mode": ["linear", "rotary"],
            "pulse_per_rev": [200.0, 400.0],
            "gear_ratio": [1.0, 36.0],
            "lead_mm": [1.0, 2.0],
        }
        self.store.save_axis_config(expected)
        self.assertEqual(self.store.load_axis_config(), expected)
        raw = json.loads(self.store.paths.axis_config.read_text(encoding="utf-8"))
        self.assertEqual(set(raw), set(expected))

    def test_legacy_calibration_round_trip_keeps_axis_keys(self):
        expected = {
            "0": {"min": None, "max": 10.0, "position": 2.5, "trusted": True},
            "29": {"min": -90.0, "max": 90.0, "position": 0.0, "trusted": False},
        }
        self.store.save_calibration(expected)
        self.assertEqual(self.store.load_calibration(), expected)

    def test_invalid_json_is_reported(self):
        self.store.paths.foc_tune.write_text("{broken", encoding="utf-8")
        with self.assertRaises(StateStoreError):
            self.store.load_foc_tune()

    def test_coordinated_bindings_round_trip(self):
        expected = {
            "schema_version": 1,
            "revision": 2,
            "bindings": {
                "Mup1": {"kind": "stepper", "axis": 0},
                "Mr1": {"kind": "stepper", "axis": 2},
                "Mup2": None,
                "Mr2": {"kind": "stepper", "axis": 3},
            },
        }
        self.store.save_coordinated_bindings(expected)
        self.assertEqual(self.store.load_coordinated_bindings(), expected)

    def test_non_object_top_level_is_rejected(self):
        self.store.paths.gear_tune.write_text("[]", encoding="utf-8")
        with self.assertRaises(StateStoreError):
            self.store.load_gear_tune()

    def test_failed_replace_preserves_previous_file(self):
        previous = {"mode": ["linear"]}
        self.store.save_axis_config(previous)
        with mock.patch("motor_control.state_store.os.replace", side_effect=OSError("busy")):
            with self.assertRaises(StateStoreError):
                self.store.save_axis_config({"mode": ["rotary"]})
        self.assertEqual(self.store.load_axis_config(), previous)
        self.assertEqual(list(self.root.glob("*.tmp")), [])


if __name__ == "__main__":
    unittest.main()
