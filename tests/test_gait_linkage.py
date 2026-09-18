"""Offline end-to-end: planner -> executor -> real GUI pulse bookkeeping.

No serial port, Tk window, network server or physical motor is opened.
"""
import math
import unittest
from dataclasses import replace
from unittest.mock import patch

from test_desktop_axis_integration import _headless_app
from motor_control import AxisProfile, BindingSet, MODE_ROTARY
from motor_control.desktop_app import StepperGUI
from motor_control.gait_executor import GaitExecutorError
from motor_control.gait_planner import (
    GaitGeometry, GaitParams, angular_targets, plan_gait_stages,
    plan_swing_trajectory, smoothstep5,
)
from motor_control.protocol import build_sync_command, parse_line, reply_matcher_for
from motor_control.serial_session import RequestTimeout


def mechanism(limit=720.0):
    app = _headless_app()
    app.control_bindings = BindingSet.suggested()
    for a in (2, 3):
        app.axis_profiles[a] = AxisProfile(mode=MODE_ROTARY, pulse_per_rev=1600, gear_ratio=1)
    app._gait_owned = {}
    app._gait_run = None
    app._gait_supports, app._gait_beta_deg = ("A", "B"), 180.0
    app._gait_needs_recovery = False
    app._closing = False
    app._is_serial_connected = lambda: True
    app._control_worker_cancelled = lambda: False
    app._post_ui = lambda action: action()
    app._refresh_gait_ui = lambda *args: None
    app._gait_stop_axes = lambda axes: app.stopped.extend(axes)
    app.stopped, app.commands = [], []
    app.gait_params = GaitParams(lift_mm=35, rotation_limit_deg=limit,
                                mr1_zero_deg=0, mr2_zero_deg=0,
                                calibration_confirmed=True,
                                calibration_fingerprint=app._gait_hardware_fingerprint())
    app.gait_params = replace(app.gait_params,
                             mr1_zero_signature=app._gait_zero_signature("Mr1", app.gait_params),
                             mr2_zero_signature=app._gait_zero_signature("Mr2", app.gait_params))

    def controller(command, **kwargs):
        app.commands.append(command)
        if kwargs.get("guard") is not None and not kwargs["guard"]():
            return ""
        if command == "SYNC,S":
            return "OK,SYNC,V1,6"
        t = command.split(",")
        if t[0] == "ENA":
            return f"OK,ENA,{t[1]},1"
        if t[0] == "SYNC":
            # Feed terminals BEFORE ACK to exercise pre-write pending accounting.
            for field in (1, 3):
                a = int(t[field])
                n = abs(int(t[field+1]))
                app._on_step_done(a, n, n)
            return f"OK,SYNC,{t[1]},{t[3]}"
        if t[0] == "MOVE":
            a, n = int(t[1]), int(t[2])
            app._on_step_done(a, n, n)
            return f"ACK,{a}"
        return "ERR:unexpected command"

    app._send_and_read = controller
    return app


class AngularLawTests(unittest.TestCase):
    def test_every_sample_uses_world_and_joint_reference_frames(self):
        for side in ("left", "right"):
            p = GaitParams(lift_mm=35)
            report = plan_swing_trajectory(p, side=side)
            self.assertTrue(report.feasible, report.message)
            for sample in report.samples:
                self.assertAlmostEqual(sample.psi_deg-30, 2*sample.phi_deg)
                self.assertAlmostEqual(sample.swing_q_delta_deg, 3*sample.phi_deg)
                self.assertAlmostEqual(sample.support_q_delta_deg, sample.phi_deg)
            self.assertAlmostEqual(report.samples[-1].beta_deg, 120 if side == "left" else 60)

    def test_linkage_does_not_depend_on_mm_geometry_or_lift(self):
        for d, lift, segments in ((80, 20, 1), (220, 35, 12), (500, 80, 200)):
            p = GaitParams(geometry=GaitGeometry(d_mm=d), lift_mm=lift, swing_segments=segments)
            s4 = next(s for s in plan_gait_stages(p, side="left") if s.stage_id == "S4")
            self.assertEqual([m.delta for m in s4.move_groups[0]], [180, 60])
            self.assertEqual(angular_targets(p, 30), (60, 90, 30))
            self.assertAlmostEqual(s4.duration_s, 18.75)

    def test_tightly_adjacent_hexagons_include_neighbors_and_clearance(self):
        p = GaitParams(geometry=GaitGeometry(d_mm=math.sqrt(3)*40), lift_mm=35)
        report = plan_swing_trajectory(p, side="left")
        self.assertGreater(len(report.hexagons), 3)
        self.assertTrue(report.feasible, report.message)
        # Sampling cannot waive a collision hidden between endpoints.
        sparse = plan_swing_trajectory(replace(p, feasibility_samples=1), side="left")
        self.assertFalse(sparse.feasible)

    def test_low_or_unknown_structure_clearance_blocks_plan(self):
        p = GaitParams(lift_mm=35, geometry=GaitGeometry(beam_height_mm=13))
        self.assertFalse(plan_swing_trajectory(p, side="left").feasible)
        p = GaitParams(lift_mm=35, geometry=GaitGeometry(body_drop_mm=30))
        self.assertFalse(plan_swing_trajectory(p, side="left").feasible)

    def test_gain_is_calibratable_but_must_land_on_low_nodes(self):
        for invalid in (0, 1, 3, float("nan")):
            with self.assertRaises(ValueError):
                GaitParams(phase_gain=invalid).validated()
        self.assertEqual(angular_targets(GaitParams(phase_gain=4), 60), (240, 300, 60))


class PhysicalExecutionBridgeTests(unittest.TestCase):
    def complete(self, app, side):
        run, report = app._gait_begin_run(side)
        self.assertTrue(report.feasible)
        with patch("motor_control.desktop_app.messagebox.askokcancel", return_value=True):
            while run.current_stage() is not None:
                if run.current_stage().is_motion_stage:
                    self.assertTrue(run.execute_current_stage())
                else:
                    app._gait_stage_confirmed()
        self.assertEqual(run.state, "done")
        self.assertEqual(app._gait_owned, {})
        return run

    def test_real_bridge_completes_left_then_right_and_updates_world_beta(self):
        app = mechanism()
        self.complete(app, "left")
        self.assertEqual(app._gait_supports, ("C", "B"))
        self.assertAlmostEqual(app._gait_beta_deg, 119.925)
        self.complete(app, "right")
        self.assertEqual(app._gait_supports, ("C", "A"))
        self.assertAlmostEqual(app._gait_beta_deg, 59.85)
        sync = [c for c in app.commands if c.startswith("SYNC,") and c != "SYNC,S"]
        self.assertEqual(len(sync), 2)
        self.assertFalse(any(c.startswith("MOVE,2,") or c.startswith("MOVE,3,") for c in app.commands))
        for a in (0, 1):
            self.assertEqual(app.axis_runtime[a].position_steps, 0)
        self.assertAlmostEqual(app._gait_angle_snapshot()["psi_delta_deg"], 119.925)

    def test_reverse_side_at_initial_stance_cannot_miss_target_pad(self):
        app = mechanism()
        with self.assertRaisesRegex(GaitExecutorError, "交换"):
            app._gait_begin_run("right")
        self.assertFalse(any(c.startswith("MOVE,") for c in app.commands))

    def test_infeasible_clearance_warns_but_no_longer_blocks(self):
        # 2026-09-18 按用户要求：碰撞/避障校验不再一票否决，
        # 改为日志+弹窗提示；执行器照常就绪。
        app = mechanism()
        app.gait_params = replace(
            app.gait_params,
            geometry=GaitGeometry(beam_height_mm=13),
            calibration_fingerprint=app._gait_hardware_fingerprint())
        app.gait_params = replace(app.gait_params,
                                  mr1_zero_signature=app._gait_zero_signature("Mr1", app.gait_params),
                                  mr2_zero_signature=app._gait_zero_signature("Mr2", app.gait_params))
        with patch("motor_control.desktop_app.messagebox.showwarning") as warn:
            run, report = app._gait_begin_run("left")
        self.assertFalse(report.feasible)
        self.assertIsNotNone(run)
        warn.assert_called_once()
        self.assertTrue(any("仅提示，不拦截" in m for m in app._logs))

    def test_legacy_firmware_rejected_before_lifting(self):
        app = mechanism()
        app._send_and_read = lambda *_args, **_kw: "ERR:unknown command"
        with self.assertRaisesRegex(GaitExecutorError, "固件"):
            app._gait_begin_run("left")
        self.assertEqual(app._gait_owned, {})

    def test_cable_limit_checked_before_any_second_lift(self):
        app = mechanism(limit=180)
        self.complete(app, "left")
        n = len(app.commands)
        with self.assertRaisesRegex(GaitExecutorError, "窗口"):
            app._gait_begin_run("right")
        self.assertFalse(any(c.startswith("MOVE,") for c in app.commands[n:]))

    def test_rebinding_invalidates_saved_calibration(self):
        app = mechanism()
        app.axis_profiles[2] = replace(app.axis_profiles[2], pulse_per_rev=3200)
        with self.assertRaisesRegex(GaitExecutorError, "标定"):
            app._gait_begin_run("left")

    def test_reconfirming_checkbox_cannot_resurrect_old_zero_after_ppr_change(self):
        app = mechanism()
        app.axis_profiles[2] = replace(app.axis_profiles[2], pulse_per_rev=3200)
        app.gait_params = replace(app.gait_params,
                                  calibration_fingerprint=app._gait_hardware_fingerprint())
        with self.assertRaisesRegex(GaitExecutorError, "记零"):
            app._gait_begin_run("left")

    def test_direction_flip_requires_new_zero_even_when_fingerprint_matches(self):
        app = mechanism()
        app.gait_params = replace(app.gait_params, mr1_sign=-1)
        with self.assertRaisesRegex(GaitExecutorError, "记零"):
            app._gait_begin_run("left")

    def test_too_slow_long_duration_rejected_before_lift(self):
        app = mechanism()
        app.gait_params = replace(app.gait_params, swing_speed_deg_s=0.3)
        with self.assertRaisesRegex(GaitExecutorError, "时间"):
            app._gait_begin_run("left")
        self.assertFalse(any(c.startswith("MOVE,") for c in app.commands))

    def test_all_four_axes_remain_reserved_between_confirmation_stages(self):
        app = mechanism()
        app._gait_begin_run("left")
        self.assertEqual(set(app._gait_owned), {0, 1, 2, 3})
        self.assertFalse(app._send_mm(3, 1, 1, 1))
        with self.assertRaises(GaitExecutorError):
            app._gait_begin_run("left")

    def test_incomplete_or_uncounted_done_cannot_advance_to_landing(self):
        for fault in ("partial", "wrong_total", "uncounted"):
            with self.subTest(fault=fault):
                app = mechanism()
                run, _ = app._gait_begin_run("left")
                run.advance_confirm(); run.advance_confirm()
                self.assertTrue(run.execute_current_stage())
                before_sync = len(app.commands)
                original = app._on_step_done

                def corrupt_done(axis, executed=None, total=None):
                    if axis == 2:
                        if fault == "partial":
                            executed -= 1
                        elif fault == "wrong_total":
                            total += 1
                        else:
                            executed = total = None
                    original(axis, executed, total)

                app._on_step_done = corrupt_done
                self.assertFalse(run.execute_current_stage())
                self.assertFalse(app.axis_runtime[2].position_trusted)
                self.assertEqual(set(app.stopped), {0, 1, 2, 3})
                self.assertEqual(run.state, "aborted")
                self.assertFalse(any(c.startswith("MOVE,")
                                     for c in app.commands[before_sync:]))

    def test_lost_sync_ack_invalidates_both_positions_and_cannot_report_success(self):
        app = mechanism()
        run, _ = app._gait_begin_run("left")
        run.advance_confirm(); run.advance_confirm()
        self.assertTrue(run.execute_current_stage())
        def lost_ack(*_args, **_kw):
            raise RequestTimeout("SYNC", 1.0)
        app._send_and_read = lost_ack
        self.assertFalse(run.execute_current_stage())
        self.assertFalse(app.axis_runtime[2].position_trusted)
        self.assertFalse(app.axis_runtime[3].position_trusted)
        self.assertEqual(set(app.stopped), {0, 1, 2, 3})


class SyncProtocolTests(unittest.TestCase):
    def test_command_and_ack_match_both_axes_and_ignore_late_ena_reply(self):
        command = build_sync_command(2, 800, 3, -267, 18750000)
        matcher = reply_matcher_for(command)
        self.assertTrue(matcher(parse_line("OK,SYNC,2,3")))
        self.assertFalse(matcher(parse_line("OK,SYNC,3,2")))
        self.assertFalse(matcher(parse_line("OK,ENA,2,1")))
        self.assertFalse(reply_matcher_for("ENA,2,1")(parse_line("OK,ENA,2,0")))

    def test_zero_duplicate_remote_and_peak_rate_are_rejected(self):
        for args in ((2,0,3,0,10000), (2,800,2,267,1000000),
                     (6,800,3,267,1000000), (2,800,3,267,1000)):
            with self.assertRaises(ValueError):
                build_sync_command(*args)


if __name__ == "__main__":
    unittest.main()
