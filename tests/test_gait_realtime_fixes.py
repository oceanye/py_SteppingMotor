"""Regression evidence for all eight follow-up findings. No serial hardware."""
import math
import time
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

from motor_control import desktop_app
from motor_control.gait_avoidance import leg_clearance, avoidance_path
from motor_control.gait_curve import CurveTimeline
from motor_control.gait_executor import GaitExecutor
from motor_control.gait_planner import default_gait_params, plan_gait_stages, effective_geometry
from motor_control.gait_simulation import GaitSimulation
from motor_control.gait_twin import TwinHistory, build_twin_snapshot, reference_displaced, ROLES
from motor_control.gait_x42s_plan import certify_x42s_stages
from motor_control.ui.gait_curve import curve_payload
from motor_control.ui import gait_twin as twin
from test_gait_executor import FakeHost
from test_gait_twin import x42s_axis
from test_x42s_pc import _x42s_gait_app
from test_x42s_runtime import issue_move


class FakeClock:
    now = 100.

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(.001, seconds)


class ArrivalEvidenceTests(unittest.TestCase):
    def test_wrong_endpoint_old_sample_and_moving_flag_never_done(self):
        for measured, speed, age in ((0, 0, 5), (215, 10, 5), (215, 0, 800)):
            with self.subTest(measured=measured, speed=speed, age=age):
                clock = FakeClock()
                with patch.object(desktop_app.time, "monotonic", clock.monotonic), \
                        patch.object(desktop_app.time, "sleep", clock.sleep):
                    app = _x42s_gait_app()
                    issue_move(app)
                    app._send_and_read = lambda *_a, **_k: (
                        f"X42S,2,S,1,1,1,none,215,{measured},0,{speed},0,25,{age},0,127")
                    self.assertEqual(app._gait_wait_terminal(102, .7), "TIMEOUT")

    def test_unsolicited_reached_does_not_confirm_any_command(self):
        app = _x42s_gait_app()
        self.assertEqual(app._gait_wait_terminal(102, 1), "ABORTED")
        self.assertEqual(app.commands, [])

    def test_post_ack_actual_endpoint_succeeds_unknown_idle_speed_allowed(self):
        clock = FakeClock()
        with patch.object(desktop_app.time, "monotonic", clock.monotonic), \
                patch.object(desktop_app.time, "sleep", clock.sleep):
            app = _x42s_gait_app()
            issue_move(app)
            self.assertEqual(app._gait_wait_terminal(102, 1), "DONE")
            self.assertFalse(app._gait_x42s_moves)

    def test_observed_overshoot_aborts_not_done(self):
        clock = FakeClock()
        with patch.object(desktop_app.time, "monotonic", clock.monotonic), \
                patch.object(desktop_app.time, "sleep", clock.sleep):
            app = _x42s_gait_app()
            issue_move(app)
            app._send_and_read = lambda *_a, **_k: "X42S,2,S,1,1,0,none,215,220,0,1,0,25,5,0,127"
            self.assertEqual(app._gait_wait_terminal(102, 1), "ABORTED")

    def test_owner_replacement_cancels_old_command(self):
        app = _x42s_gait_app()
        issue_move(app)
        app._move_reservation[102] = object()
        self.assertEqual(app._gait_wait_terminal(102, .1), "CANCELLED")

    def test_pair_prevalidates_second_axis_before_any_move(self):
        app = _x42s_gait_app()
        app._gait_begin_run("left", arc_deg=-60)
        app._x42s_states["Mr2"] = replace(app._x42s_states["Mr2"], reached=False)
        self.assertEqual(app._gait_send_synchronized([(102, 1, 1), (104, 1, 1)], 1), "failed")
        self.assertFalse(any(",MOVE," in c for c in app.commands))


class ProgressEnvelopeTests(unittest.TestCase):
    def test_four_actions_entire_independent_progress_rectangles_clear(self):
        params = default_gait_params()
        geometry = effective_geometry(params)
        for side, arc in (("left", 60), ("left", -60), ("right", 60), ("right", -60)):
            sim = GaitSimulation(params)
            report = sim.preview(side, arc)
            stages = plan_gait_stages(params, side=side, arc_deg=arc, route=report.route)
            certified = certify_x42s_stages(stages, params, report, .5)
            s4 = next(s for s in certified if s.stage_id == "S4")
            self.assertEqual(s4.execution_profile, "x42s_bounded_progress")
            self.assertGreater(s4.certified_margin_mm, 0)
            self.assertEqual(s4.sync_endpoints[-1], next(s for s in stages if s.stage_id == "S4").sync_endpoints[-1])
            pivot = next(p.center for p in report.hexagons if p.name == report.route[2])
            start = (0., 0.)
            signs = (params.mr1_sign, params.mr2_sign) if side == "left" else (params.mr2_sign, params.mr1_sign)
            for endpoint in s4.sync_endpoints:
                for first in (0., .5, 1.):
                    for second in (0., .5, 1.):
                        qs, qp = [sign*(a+(b-a)*f) for a, b, f, sign in zip(start, endpoint, (first, second), signs)]
                        theta = math.radians(report.route[3]-qp)
                        center = (pivot[0]+geometry.d_mm*math.cos(theta), pivot[1]-geometry.d_mm*math.sin(theta))
                        margin = leg_clearance(center, 30+qs-qp, geometry, report.hexagons)[0]
                        self.assertGreaterEqual(margin, s4.certified_margin_mm+.5-1e-8)
                start = endpoint

    def test_no_certificate_when_error_budget_exceeds_available_clearance(self):
        params = default_gait_params()
        report = GaitSimulation(params).preview("left", -60)
        stages = plan_gait_stages(params, side="left", arc_deg=-60, route=report.route)
        with self.assertRaisesRegex(ValueError, "不可行"):
            certify_x42s_stages(stages, params, report, 500)

    def test_real_host_full_four_actions_with_fake_closed_loop_transport(self):
        # Exercise actual host, actual executor and actual unit conversion;
        # only serial transport and the clock are fake. Never opens a port.
        for side, arc in (("left", 60), ("left", -60), ("right", 60), ("right", -60)):
            clock = FakeClock()
            with self.subTest(side=side, arc=arc), \
                    patch.object(desktop_app.time, "monotonic", clock.monotonic), \
                    patch.object(desktop_app.time, "sleep", clock.sleep):
                app = _x42s_gait_app()
                params = default_gait_params()
                app.gait_params = replace(app.gait_params, geometry=params.geometry,
                                         trajectory_mode=params.trajectory_mode, phase_gain=params.phase_gain)
                run, _ = app._gait_begin_run(side, arc_deg=arc)
                self.assertIsNotNone(run.twin_report)
                while run.current_stage():
                    # Emulate the existing background idle-driver poll.
                    for role, state in list(app._x42s_states.items()):
                        app._x42s_states[role] = replace(state, last_update_monotonic=clock.now-.005)
                    if run.current_stage().is_motion_stage:
                        self.assertTrue(run.execute_current_stage(), run.last_error)
                    else:
                        self.assertTrue(run.advance_confirm())
                self.assertEqual(run.state, "done")
                self.assertFalse(app._gait_x42s_moves)
                self.assertTrue(any(",MOVE," in c for c in app.commands))
                for role, state in list(app._x42s_states.items()):
                    app._x42s_states[role] = replace(state, last_update_monotonic=clock.now-.005)
                old_supports = app._gait_supports
                fresh = app._x42s_states["Mr1"]
                app._x42s_states["Mr1"] = replace(fresh, last_update_monotonic=clock.now-2)
                app._gait_finish_if_done()
                self.assertEqual(app._gait_supports, old_supports)
                self.assertTrue(app._gait_owned)
                app._x42s_states["Mr1"] = fresh
                app._gait_finish_if_done()
                self.assertTrue(run.ledger_finalized)
                beta = app._gait_beta_deg
                app._gait_finish_if_done()
                self.assertEqual(app._gait_beta_deg, beta)  # Idempotent refresh.
                self.assertFalse(app._gait_owned)


class ClearanceAndDisplayTests(unittest.TestCase):
    def test_positive_tip_distance_does_not_hide_whole_leg_collision(self):
        params = default_gait_params()
        phi = -20
        spin = avoidance_path(180, -60).angles(abs(phi)/60)[0]
        pose = {"feet": {"left": dict(center=(math.cos(math.radians(180-phi)),
                                               -math.sin(math.radians(180-phi))), psi_deg=30+spin-20),
                         "right": dict(center=(0, 0), psi_deg=30)}}
        self.assertGreater(min(twin.tip_surface_distances(pose, params)), 40)
        worst = min(twin.leg_surface_details(pose, params), key=lambda d: d["gap_mm"])
        self.assertLess(worst["gap_mm"], -1)
        self.assertAlmostEqual(worst["margin_mm"], worst["gap_mm"]-2)
        self.assertEqual(worst["leg"], 1)
        self.assertIn("(0,-1)", worst["rod"])

    def test_plan_and_feedback_share_same_whole_leg_metric(self):
        params = default_gait_params()
        sim = GaitSimulation(params)
        report = sim.preview("left", -60)
        series = twin.plan_leg_series(report, params)
        geometry = effective_geometry(params)
        for sample, (_, values) in zip(report.samples, series):
            margin = leg_clearance(sample.center, sample.psi_deg, geometry, report.hexagons)[0]
            self.assertAlmostEqual(min(values[:3])-geometry.safety_margin_mm, margin)

    def test_bad_sample_and_quantum_jitter_are_not_independent_motion(self):
        axes = [x42s_axis(r, position=0., gear_ratio=21.5, lead_mm=8.) for r in ROLES]
        final = dict.fromkeys(ROLES, 0.)
        axes[1].update(position_trusted=False, stale=True, position=None)
        self.assertFalse(reference_displaced(axes, final))
        axes[1].update(position_trusted=True, stale=False, position=.1/21.5)
        self.assertFalse(reference_displaced(axes, final))
        axes[1]["position"] = 1.
        self.assertTrue(reference_displaced(axes, final))
        axes[1].update(position=0., command_pending=True)
        self.assertTrue(reference_displaced(axes, final))

    def test_real_snapshot_recovers_after_gap_and_manual_command_invalidates(self):
        app = _x42s_gait_app()
        app._hardware_estop_active = app._estop_in_progress = app._estop_unconfirmed = False
        app.pico_node_health = {}
        run, _ = app._gait_begin_run("left", arc_deg=-60)
        run.twin_final_positions = dict.fromkeys(ROLES, 0.)
        run.state = "done"
        app._gait_release_ownership()
        self.assertEqual(app._gait_twin_snapshot()["state"], "estimated")
        state = app._x42s_states["Mr1"]
        app._x42s_states["Mr1"] = replace(state, last_update_monotonic=time.monotonic()-2)
        self.assertEqual(app._gait_twin_snapshot()["state"], "untrusted")
        app._x42s_states["Mr1"] = replace(state, measured_position=.1, last_update_monotonic=time.monotonic())
        self.assertEqual(app._gait_twin_snapshot()["state"], "estimated")
        self.assertFalse(run.twin_manual_invalid)
        # Invalidation is command-based, so jogging back between polls cannot
        # silently restore a formerly grounded support assumption.
        app._send_manual_x42s("X42S,2,MOVE,100,100", "Mr1", 2, 100, 100)
        self.assertTrue(run.twin_manual_invalid)

    def test_incoherent_samples_freeze_pose_but_preserve_individual_measurements(self):
        axes = [x42s_axis(r) for r in ROLES]
        axes[1]["last_update_age_ms"] = 1400
        snapshot = build_twin_snapshot(default_gait_params(), axes, calibration_valid=True)
        self.assertTrue(snapshot["axes_measured"])
        self.assertFalse(snapshot["measured"])
        self.assertIsNone(snapshot["pose"])
        self.assertEqual(snapshot["sample_skew_ms"], 1395)
        self.assertEqual(snapshot["axes"]["Mr1"]["position"], 90.)

    def test_clear_display_preserves_completed_count_without_double_count(self):
        history = TwinHistory()
        pose = GaitSimulation(default_gait_params()).frame.pose
        snapshot = dict(world_key="one", reference_key=1, state="estimated", pose=pose, step_done=True)
        history.observe(snapshot)
        snapshot["reference_key"] = 2
        history.observe(snapshot)
        history.clear(snapshot, preserve_steps=True)
        history.observe(snapshot)
        self.assertEqual(history.completed_steps, 2)
        self.assertEqual(history.current_step, 2)
        snapshot["reference_key"] = 3
        history.observe(snapshot)
        self.assertEqual(history.completed_steps, 3)
        snapshot["world_key"] = "new"
        history.observe(snapshot)
        self.assertEqual(history.completed_steps, 1)

    def test_statistics_never_mix_future_plan_with_feedback(self):
        params = default_gait_params()
        app = SimpleNamespace(gait_params=params, gait_widgets={})
        view = dict(mode_var=SimpleNamespace(get=lambda: "live"), snapshot=dict(pose={}),
                    live_curve_timeline=CurveTimeline(), live_curve_reference=dict(
                        offset_s=0., segment=1, current_s=5., key="run", feedback_valid=True,
                        feedback=[(5., (20.,)*6, 0)], series=[(0., (40.,)*6), (10., (-3.,)*6)]))
        with patch.object(twin, "_mode", return_value="live"):
            stats = curve_payload(app, view)["statistics"]
            self.assertEqual(stats["current"], 20.)
            self.assertEqual(stats["executed_min"], 20.)
            self.assertEqual(stats["reference_min"], -3.)
            view["live_curve_reference"]["feedback_valid"] = False
            self.assertIsNone(curve_payload(app, view)["statistics"]["current"])

    def test_executor_progress_survives_read_only_refresh(self):
        params = default_gait_params()
        stages = plan_gait_stages(params, side="left")
        host = FakeHost()
        host.send_synchronized_endpoint = lambda moves, duration, endpoints: host.send_synchronized(moves, duration)
        run = GaitExecutor(host, params, stages, side="left")
        while run.current_stage().stage_id != "S4":
            run.execute_current_stage() if run.current_stage().is_motion_stage else run.advance_confirm()
        progress = []
        self.assertTrue(run.execute_current_stage(lambda *_: progress.append(run.describe()["progress_text"])))
        self.assertTrue(progress)
        self.assertEqual(run.describe()["progress_text"], progress[-1])


if __name__ == "__main__":
    unittest.main()
