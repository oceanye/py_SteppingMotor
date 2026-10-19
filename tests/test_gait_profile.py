"""Explicit field-profile migration: local files only, no serial/hardware."""
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from dataclasses import asdict, replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_gait_linkage import mechanism
from motor_control.gait_avoidance import TWO_MODE
from motor_control.gait_executor import GaitExecutorError
from motor_control.gait_planner import (
    GaitParams, apply_190_two_mode_profile, default_gait_params, parse_gait_params,
)
from motor_control.gait_simulation import GaitSimulation
from motor_control.state_store import StateStore, StateStoreError


class ProfileMigrationTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.app = mechanism()
        self.app._is_serial_connected = lambda: False
        self.app.state_store = StateStore(self.root)
        self.app.state_store.save_gait_params(self.app.gait_params.as_document())
        self.original = self.app.state_store.paths.gait_params.read_bytes()
        self.app.gait_widgets = {"twin": {
            "simulation": SimpleNamespace(active=False), "candidate": object()}}

    def test_remote_style_profile_preserves_site_settings_but_clears_calibration(self):
        old = replace(
            self.app.gait_params, mr1_sign=-1, mr2_sign=-1, lift_mm=70,
            initial_placement="red_left", beam_reference_deg=0,
            geometry=replace(self.app.gait_params.geometry, arm_radius_mm=.2,
                             node_radius_mm=.2, safety_margin_mm=.1))
        new = apply_190_two_mode_profile(old)
        self.assertEqual(new.trajectory_mode, TWO_MODE)
        self.assertEqual(new.geometry.arm_length_mm, 190)
        self.assertEqual(new.geometry.d_mm, math.sqrt(3)*190)
        self.assertEqual(new.geometry.arm_radius_mm, .2)
        self.assertEqual(new.geometry.node_radius_mm, .2)
        self.assertEqual(new.geometry.safety_margin_mm, .1)
        self.assertEqual((new.mr1_sign, new.mr2_sign), (-1, -1))
        self.assertEqual(new.initial_supports, old.initial_supports)
        for field in ("lift_mm", "lift_speed_mm_s", "swing_speed_deg_s", "phase_gain",
                      "settle_speed_mm_s", "rotation_limit_deg", "beam_reference_deg",
                      "mup1_lift_sign", "mup2_lift_sign"):
            self.assertEqual(getattr(new, field), getattr(old, field))
        self.assertFalse(new.calibration_confirmed)
        for field in ("mr1_zero_deg", "mr2_zero_deg", "calibration_fingerprint",
                      "mr1_zero_signature", "mr2_zero_signature"):
            self.assertIsNone(getattr(new, field))
        self.assertTrue(old.calibration_confirmed)  # Immutable source unchanged.

    def test_custom_map_start_is_preserved(self):
        old = replace(default_gait_params(), initial_pad_pair=("邻座(0,-1)", "邻座(0,-2)"),
                      beam_reference_deg=60)
        new = apply_190_two_mode_profile(old)
        self.assertEqual(new.initial_pad_pair, old.initial_pad_pair)
        self.assertEqual(new.beam_reference_deg, 60)
        self.assertEqual(new, old)

    def test_backup_publish_reload_and_no_motor_or_binding_changes(self):
        app = self.app
        bindings = app.control_bindings
        profiles = list(app.axis_profiles)
        axis_states = [asdict(axis) for axis in app.axis_runtime]
        app._gait_needs_recovery = True
        backup = app._gait_apply_190_two_mode()
        self.assertEqual(backup.read_bytes(), self.original)
        self.assertEqual(parse_gait_params(app.state_store.load_gait_params()), app.gait_params)
        self.assertTrue(app._gait_needs_recovery)  # Never clear a recovery latch.
        self.assertEqual(app._gait_world_epoch, 1)
        self.assertEqual(app._gait_supports, app.gait_params.initial_supports)
        self.assertNotIn("simulation", app.gait_widgets["twin"])
        self.assertNotIn("candidate", app.gait_widgets["twin"])
        self.assertIsNone(app._gait_run)
        self.assertIs(app.control_bindings, bindings)
        self.assertEqual(app.axis_profiles, profiles)
        self.assertEqual([asdict(axis) for axis in app.axis_runtime], axis_states)
        self.assertEqual(app.commands, [])
        app._startup_warnings = []
        app._load_gait_params()
        self.assertEqual(app._startup_warnings, [])
        self.assertEqual(app.gait_params.geometry.arm_length_mm, 190)
        self.assertFalse(app.gait_params.calibration_confirmed)

    def test_startup_flags_legacy_but_never_silently_rewrites_it(self):
        self.app._startup_warnings = []
        self.app._load_gait_params()
        self.assertIn("应用190 mm两模态", self.app._startup_warnings[0])
        self.assertEqual(self.app.state_store.paths.gait_params.read_bytes(), self.original)

    def test_connection_busy_simulation_and_estop_refuse_without_writes(self):
        app = self.app
        mutations = (
            {"_is_serial_connected": lambda: True},
            {"_gait_owned": {0: object()}},
            {"_estop_unconfirmed": True},
            {"_disconnecting": True},
            {"gait_widgets": {"twin": {"start_busy": True}}},
            {"gait_widgets": {"twin": {"simulation": SimpleNamespace(active=True)}}},
        )
        old = app.gait_params
        for fields in mutations:
            with self.subTest(fields=tuple(fields)), patch.multiple(app, create=True, **fields):
                with self.assertRaises(GaitExecutorError):
                    app._gait_apply_190_two_mode()
            self.assertIs(app.gait_params, old)
            self.assertEqual(app.state_store.paths.gait_params.read_bytes(), self.original)
        self.assertEqual(list(self.root.glob("*.backup-*")), [])
        self.assertEqual(app.commands, [])

    def test_backup_failure_aborts_before_save_or_publication(self):
        app = self.app
        old = app.gait_params
        with patch.object(app.state_store, "backup_gait_params", side_effect=StateStoreError("disk")):
            with self.assertRaises(StateStoreError):
                app._gait_apply_190_two_mode()
        self.assertIs(app.gait_params, old)
        self.assertEqual(app.state_store.paths.gait_params.read_bytes(), self.original)
        self.assertIn("candidate", app.gait_widgets["twin"])

    def test_save_failure_preserves_file_memory_and_recoverable_backup(self):
        app = self.app
        old = app.gait_params
        with patch("motor_control.state_store.os.replace", side_effect=OSError("busy")):
            with self.assertRaises(StateStoreError):
                app._gait_apply_190_two_mode()
        self.assertIs(app.gait_params, old)
        self.assertEqual(app.state_store.paths.gait_params.read_bytes(), self.original)
        self.assertEqual(next(self.root.glob("*.backup-*")).read_bytes(), self.original)
        self.assertIn("candidate", app.gait_widgets["twin"])

    def test_declining_confirmation_is_read_only(self):
        with patch("motor_control.desktop_app.messagebox.askokcancel", return_value=False):
            self.app._gait_apply_190_two_mode_clicked()
        self.assertEqual(self.app.state_store.paths.gait_params.read_bytes(), self.original)
        self.assertEqual(list(self.root.glob("*.backup-*")), [])

    def test_confirmed_callback_refreshes_fields_and_records_backup(self):
        app = self.app
        app.gait_report_var = Mock()
        with patch("motor_control.desktop_app.messagebox.askokcancel", return_value=True), \
                patch("motor_control.desktop_app.load_gait_fields") as load:
            app._gait_apply_190_two_mode_clicked()
        load.assert_called_once_with(app)
        self.assertIn(".gait_params.json.backup-", app._logs[-1])
        self.assertEqual(app.commands, [])

    def test_migration_requires_fresh_confirmation_before_real_execution(self):
        self.app._gait_apply_190_two_mode()
        self.app._is_serial_connected = lambda: True
        with self.assertRaisesRegex(GaitExecutorError, "尚未确认"):
            self.app._gait_begin_run("left", arc_deg=60)
        self.assertEqual(self.app.commands, [])

    def test_four_actions_for_both_mirror_starts_with_standard_envelopes(self):
        for placement, beta in (("red_right", 180), ("red_left", 0)):
            params = apply_190_two_mode_profile(replace(
                GaitParams(), initial_placement=placement, beam_reference_deg=beta))
            modes = set()
            for side in ("left", "right"):
                for arc in (60, -60):
                    with self.subTest(placement=placement, side=side, arc=arc):
                        sim = GaitSimulation(params)
                        report = sim.begin(side, arc)
                        self.assertTrue(report.feasible, report.message)
                        self.assertGreater(report.min_margin_mm, 0)
                        modes.add(report.modality)
                        while sim.active:
                            sim.advance()
                        self.assertEqual(sim.completed_steps, 1)
            self.assertEqual(len(modes), 2)


if __name__ == "__main__":
    unittest.main()
