"""Distributed geometry defaults; no hardware or local calibration writes."""
import json
import math
import os
from pathlib import Path
import sys
import unittest
from dataclasses import replace
from unittest.mock import Mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_gait_linkage as linkage
from motor_control.gait_avoidance import LEGACY, TWO_MODE
from motor_control.gait_planner import (
    GaitParams, default_gait_params, effective_geometry, parse_gait_params,
)
from motor_control.gait_simulation import GaitSimulation
from motor_control.state_store import StateStoreError


class DistributedGaitDefaultsTests(unittest.TestCase):
    def test_fresh_default_is_190_two_mode_without_calibration(self):
        params = default_gait_params()
        self.assertEqual(params.trajectory_mode, TWO_MODE)
        self.assertEqual(params.geometry.arm_length_mm, 190)
        self.assertEqual(params.geometry.d_mm, math.sqrt(3)*190)
        self.assertFalse(params.calibration_confirmed)
        for name in ("mr1_zero_deg", "mr2_zero_deg", "calibration_fingerprint",
                     "mr1_zero_signature", "mr2_zero_signature"):
            self.assertIsNone(getattr(params, name))

    def test_distributed_sample_matches_runtime_defaults(self):
        path = Path(__file__).resolve().parents[1] / "config/gait_params.example.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(parse_gait_params(document), default_gait_params())
        self.assertEqual(set(document), {"schema", "trajectory_mode", "geometry"})

    def test_partial_two_mode_uses_190_but_legacy_semantics_stay_unchanged(self):
        partial = {"schema": GaitParams.SCHEMA, "trajectory_mode": TWO_MODE}
        self.assertEqual(parse_gait_params(partial), default_gait_params())
        old = parse_gait_params({"schema": GaitParams.SCHEMA})
        self.assertEqual(old.trajectory_mode, LEGACY)
        self.assertEqual(old.geometry.arm_length_mm, 40)
        self.assertEqual(old, GaitParams())

    def test_explicit_saved_dimensions_and_calibration_are_not_migrated(self):
        app = linkage.mechanism()
        saved = replace(app.gait_params, trajectory_mode=TWO_MODE,
                        mr1_zero_deg=12.3, mr2_zero_deg=-4.5)
        app.state_store = Mock()
        app.state_store.load_gait_params.return_value = saved.as_document()
        app._startup_warnings = []
        bindings, profiles = app.control_bindings, list(app.axis_profiles)
        app._load_gait_params()
        self.assertEqual(app.gait_params, replace(saved, calibration_confirmed=False))
        self.assertEqual(app.gait_params.geometry.arm_length_mm, 40)
        self.assertIs(app.control_bindings, bindings)
        self.assertEqual(app.axis_profiles, profiles)
        app.state_store.save_gait_params.assert_not_called()
        self.assertEqual(app.commands, [])

    def test_missing_unreadable_and_invalid_files_fall_back_without_writing(self):
        for value in (None, StateStoreError("unreadable"),
                      {"schema": GaitParams.SCHEMA, "geometry": {"arm_length_mm": -1}},
                      {"schema": GaitParams.SCHEMA, "geometry": {"arm_length_mm": None}}):
            with self.subTest(value=value):
                app = linkage.mechanism()
                app.state_store = Mock()
                if isinstance(value, Exception):
                    app.state_store.load_gait_params.side_effect = value
                else:
                    app.state_store.load_gait_params.return_value = value
                app._startup_warnings = []
                app._load_gait_params()
                self.assertEqual(app.gait_params, default_gait_params())
                self.assertEqual(bool(app._startup_warnings), value is not None)
                app.state_store.save_gait_params.assert_not_called()
                self.assertEqual(app.commands, [])

    def test_distributed_defaults_pass_four_actions_without_real_calibration(self):
        params = default_gait_params()
        for side in ("left", "right"):
            for arc in (60, -60):
                with self.subTest(side=side, arc=arc):
                    sim = GaitSimulation(params)
                    report = sim.begin(side, arc)
                    self.assertTrue(report.feasible, report.message)
                    self.assertGreater(report.min_margin_mm, 0)
                    while sim.active:
                        sim.advance()
                    self.assertEqual(sim.completed_steps, 1)
                    self.assertAlmostEqual(effective_geometry(sim.params).d_mm,
                                           math.sqrt(3)*190)
                    self.assertFalse(sim.params.calibration_confirmed)


if __name__ == "__main__":
    unittest.main()
