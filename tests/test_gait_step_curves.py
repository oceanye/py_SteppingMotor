"""Fixed-scale sliding curves and action candidates, offline only."""
import os
import sys
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_gait_linkage as linkage
import test_gait_ui as ui
from motor_control.gait_curve import EncoderCurveBuffer, clip_series, rolling_window, time_at_phi
from motor_control.gait_planner import default_gait_params, plan_swing_trajectory
from motor_control.gait_simulation import GaitSimulation
from motor_control.gait_twin_timing import plan_sample_times
from motor_control.ui import gait_twin as twin
from motor_control.ui.gait_curve import curve_payload


class StepCurveTests(unittest.TestCase):
    def test_window_has_fixed_span_and_clip_preserves_gaps(self):
        self.assertEqual(rolling_window(0), (0, 90))
        self.assertEqual(rolling_window(75), (0, 90))
        self.assertEqual(rolling_window(120), (30, 120))
        data = [(0, (0,)*6, 1), (100, (100,)*6, 1)]
        self.assertEqual(clip_series(data, 30, 90), [(30, (30.,)*6, 1), (90, (90.,)*6, 1)])
        self.assertEqual(clip_series([data[0], (100, (100,)*6, 2)], 30, 90), [])

    def test_signed_model_progress_clock_not_elapsed_wall_time(self):
        for side in ("left", "right"):
            for arc in (-60, 60):
                report = plan_swing_trajectory(default_gait_params(), side=side, arc_deg=arc)
                times = plan_sample_times(report, default_gait_params())
                for index in (0, len(times)//2, len(times)-1):
                    self.assertAlmostEqual(time_at_phi(report, times, report.samples[index].phi_deg), times[index])
                self.assertEqual(time_at_phi(report, times, arc*2), times[-1])

    def test_simulation_retains_curves_and_world_path_across_steps(self):
        sim = GaitSimulation(default_gait_params())
        for step in range(2):
            previous = list(sim.curve)
            sim.begin("left", 60)
            self.assertEqual(list(sim.curve), previous)
            self.assertEqual(sim.step_started_s, sim.elapsed_s)
            while sim.active:
                sim.advance()
                sim.curve.append((sim.curve_time_s, [50]*6, sim.segment))
            self.assertEqual(len(sim.history.landings), step+1)
        self.assertEqual({p[2] for p in sim.curve}, {1, 2})
        self.assertEqual([p[0] for p in sim.curve], sorted(p[0] for p in sim.curve))

    def test_encoder_boundary_never_accepts_command_data_or_old_steps(self):
        buffer = EncoderCurveBuffer()
        buffer.bind("step1")
        for source, key, time, values in (("commanded_pulses", "step1", 0, [1]*6),
                                          ("encoder", "old", 0, [1]*6),
                                          ("encoder", "step1", float("nan"), [1]*6),
                                          ("encoder", "step1", 0, [1]*5)):
            with self.assertRaises(ValueError):
                buffer.append(key, time, values, source=source)
        self.assertFalse(buffer.samples)
        buffer.append("step1", 1, [1]*6, source="encoder")
        with self.assertRaises(ValueError):
            buffer.append("step1", .5, [1]*6, source="encoder")
        buffer.bind("step2")
        self.assertFalse(buffer.samples)


@unittest.skipUnless(ui.HAS_TK, "real Tk unavailable")
class StepCurveUITests(unittest.TestCase):
    def setUp(self):
        ui.GaitLayoutTests.setUp(self)
        from motor_control.ui.gait_tab import load_gait_fields
        self.app.gait_params = default_gait_params()
        load_gait_fields(self.app)
        self.root.geometry("1480x820")
        self.root.update()
        self.view = self.app.gait_widgets["twin"]
        self.app._save_gait_params = Mock(side_effect=AssertionError("display must not save"))
        self.app._send_and_read = Mock(side_effect=AssertionError("display must not send"))

    def make_live_run(self, side="left", arc=60):
        params = self.app.gait_params
        report = plan_swing_trajectory(params, side=side, arc_deg=arc)
        run = SimpleNamespace(params=params, twin_report=report, twin_psis=(30., 30.))
        self.app._gait_run = run
        self.app._gait_owned = {2: "owned"}
        self.snapshot = {"reference_key": id(run), "world_key": "origin", "state": "estimated",
            "axes": {}, "message": "pulse estimate", "pose": {"phi_deg": 0, "beta_deg": 180,
            "feet": {"left": {"center": (-1, 0), "psi_deg": 30, "z_mm": 0},
                     "right": {"center": (0, 0), "psi_deg": 30, "z_mm": 0}}}}
        self.app._gait_twin_snapshot = lambda: self.snapshot
        twin.refresh_twin_panel(self.app)
        return run

    def test_four_choices_make_dashed_landing_without_save_motion_or_preview_button(self):
        from motor_control.ui.gait_tab import _gait_mode_selected
        self.assertNotIn("play_btn", self.app.gait_widgets)
        self.assertEqual([str(c.cget("value")) for c in self.view["mode_controls"]], ["sim", "live"])
        params = self.app.gait_params
        for selection in ("L+", "L-", "R+", "R-"):
            _gait_mode_selected(self.app, selection)
            self.assertTrue(self.view["canvas"].find_withtag("candidate_ghost"))
            self.assertEqual(self.view["candidate"]["report"].side, "left" if selection[0] == "L" else "right")
            self.assertEqual(self.view["mode_var"].get(), "sim")
            self.assertIs(self.app.gait_params, params)
        self.app._save_gait_params.assert_not_called()
        self.app._send_and_read.assert_not_called()
        self.assertFalse(self.view.get("simulation"))
        self.app.gait_field_vars["arm_length_mm"].set("bad")
        _gait_mode_selected(self.app, "L+")
        self.assertNotIn("candidate", self.view)
        self.assertFalse(self.view["canvas"].find_withtag("candidate_ghost"))

    def test_x42s_feedback_curves_are_modelled_and_gap_safe_not_fake_measurements(self):
        run = self.make_live_run()
        self.snapshot.update(source="x42s_measured", measured=True,
                             axes={r: {"position_trusted": True, "stale": False}
                                   for r in ("Mup1", "Mr1", "Mup2", "Mr2")})
        twin.refresh_twin_panel(self.app)
        self.snapshot["pose"]["phi_deg"] = 10
        twin.refresh_twin_panel(self.app)
        payload = curve_payload(self.app, self.view)
        self.assertEqual(len(payload["feedback"]), 2)
        self.assertFalse(payload["measured"])
        self.assertIn("X42S反馈推算", payload["status"])
        self.assertTrue(self.view["curve_canvas"].find_withtag("curve_feedback"))
        ref = self.view["live_curve_reference"]
        saved = list(ref["feedback"])
        self.snapshot["axes"]["Mr1"]["stale"] = True
        twin.refresh_twin_panel(self.app)
        self.assertEqual(list(ref["feedback"]), saved)
        self.snapshot["axes"]["Mr1"]["stale"] = False
        pose = self.snapshot["pose"]
        self.snapshot.update(pose=None, measured=False, state="stale")
        twin.refresh_twin_panel(self.app)
        self.assertEqual(list(ref["feedback"]), saved)
        self.snapshot.update(pose=pose, measured=True, state="estimated")
        pose["phi_deg"] = 20
        twin.refresh_twin_panel(self.app)
        self.assertNotEqual(ref["feedback"][-1][2], saved[-1][2])
        # Next run archives feedback without connecting across a missing sample.
        self.app._gait_run = SimpleNamespace(params=run.params, twin_report=run.twin_report,
                                             twin_psis=(30., 30.))
        self.snapshot["reference_key"] = id(self.app._gait_run)
        pose["phi_deg"] = 0
        twin.refresh_twin_panel(self.app)
        self.assertTrue(self.view["live_curve_timeline"].feedback)
        twin.clear_twin_history(self.app)
        self.assertFalse(self.view["live_curve_timeline"].feedback)
        self.assertFalse(self.view["live_curve_reference"]["feedback"])

    def test_candidate_after_two_simulated_steps_uses_new_stance_without_accumulating(self):
        from motor_control.ui.gait_tab import _gait_mode_selected
        sim = GaitSimulation(self.app.gait_params)
        for _ in range(2):
            sim.begin("left", 60)
            while sim.active:
                sim.advance()
        self.view["simulation"] = sim
        self.view["mode_var"].set("sim")
        before = (sim.supports, sim.psis, list(sim.history.points))
        _gait_mode_selected(self.app, "R-")
        # 2026-10-22（二版）：弧向换回原始绑定（"R-"=模型 -60）
        self.assertEqual(self.view["candidate"]["report"].route, sim.route("right", -60))
        self.assertEqual((sim.supports, sim.psis, list(sim.history.points)), before)
        self.assertEqual(sim.completed_steps, 2)

    def test_fixed_axis_includes_zero_even_empty_and_does_not_scale_on_collision(self):
        canvas = self.view["curve_canvas"]
        def axis():
            twin._draw_curve(self.app, self.view)
            return [(canvas.itemcget(i, "text"), canvas.coords(i)) for i in canvas.find_withtag("curve_y_tick")]
        before = axis()
        self.assertEqual([v[0] for v in before], ["-20", "0", "50", "100", "150", "200"])
        sim = GaitSimulation(self.app.gait_params)
        self.view["simulation"] = sim
        self.view["mode_var"].set("sim")
        for values in ((50, 80), (-3, 185), (-30, 250)):
            sim.curve.clear()
            sim.curve.extend(((0, [values[0]]*6, 0), (1, [values[1]]*6, 0)))
            self.assertEqual(axis(), before)
        self.assertTrue(canvas.find_withtag("curve_overflow"))
        self.assertTrue(canvas.find_withtag("curve_negative"))

    def test_live_keeps_only_actual_run_grey_reference_not_old_simulation_or_fake_encoder(self):
        from motor_control.ui.gait_tab import _gait_mode_selected
        sim = GaitSimulation(self.app.gait_params)
        sim.curve.extend(((0, [-999]*6, 0), (1, [-999]*6, 0)))
        self.view["simulation"] = sim
        _gait_mode_selected(self.app, "R-")
        run = self.make_live_run("left", 60)
        reference = self.view["live_curve_reference"]
        self.assertIs(reference["report"], run.twin_report)
        self.assertFalse(self.view["encoder_curve"].samples)
        self.assertNotIn("candidate", self.view)
        self.assertFalse(self.view["canvas"].find_withtag("candidate_ghost"))
        c = self.view["curve_canvas"]
        self.assertTrue(c.find_withtag("curve_reference"))
        self.assertFalse(c.find_withtag("curve_measured"))
        self.assertFalse(c.find_withtag("curve_model"))
        self.assertTrue(all(v > -999 for _, vals, _ in self.view["curve_display"]["data"] for v in vals))
        self.assertTrue(self.view["curve_live"])  # Command telemetry exists, but is never measured data.
        self.assertTrue(all(str(b["state"]) == "disabled" for b in self.app.gait_widgets["action_choices"]))
        # A new step replaces the active reference; only reached history is archived.
        encoder = self.view["encoder_curve"]
        encoder.append(encoder.step_key, 0, [2]*6, source="encoder")
        encoder.append(encoder.step_key, 1, [-1]*6, source="encoder")
        twin.draw_twin(self.app)
        self.assertTrue(c.find_withtag("curve_measured"))
        self.assertIn("彩色=编码器", self.view["curve_display"]["status"])
        second = self.make_live_run("right", -60)
        self.assertIs(self.view["live_curve_reference"]["report"], second.twin_report)
        self.assertIsNot(self.view["live_curve_reference"], reference)
        self.assertFalse(encoder.samples)
        self.assertEqual(len(self.view["curve_live"]), 1)

    def test_reference_clock_follows_phi_and_freezes_on_gap_both_windows(self):
        from motor_control.ui.gait_twin_window import open_twin_window
        run = self.make_live_run("left", -60)
        open_twin_window(self.app)
        detail = self.view["large_window"]
        detail["window"].attributes("-alpha", 0)
        self.root.update()
        times = plan_sample_times(run.twin_report, run.params)
        index = len(times)//2
        self.snapshot["pose"]["phi_deg"] = run.twin_report.samples[index].phi_deg
        twin.refresh_twin_panel(self.app)
        current = self.view["curve_display"]["current"]
        self.assertAlmostEqual(current, times[index])
        self.assertEqual(detail["curve_display"]["current"], current)
        self.snapshot.update(pose=None, state="stale")
        twin.refresh_twin_panel(self.app)
        self.assertEqual(self.view["curve_display"]["current"], current)
        self.assertIn("冻结", self.view["curve_display"]["status"])
        self.assertIs(detail["live_curve_reference"], self.view["live_curve_reference"])
        self.app._gait_run = None
        self.app._gait_owned = {}
        self.snapshot["reference_key"] = None
        twin.refresh_twin_panel(self.app)
        self.assertNotIn("live_curve_reference", detail)
        self.assertFalse(detail["curve_canvas"].find_withtag("curve_reference"))

    def test_lift_bars_use_fixed_lift_scale_during_simulation(self):
        # 2026-10-20：柱状图曾用当前帧两腿 z 的最大值自归一化——任何
        # 非零 z 都显示成满格（S2 一开场就顶格、S6 双降全程满格直到突
        # 降 0）。固定标尺（lift_mm）后 S2 中段柱形应介于 0 与满格之间。
        from motor_control.gait_simulation import GaitSimulation
        sim = GaitSimulation(self.app.gait_params)
        sim.begin("left", 60.0)
        self.view["simulation"] = sim
        self.view["mode_var"].set("sim")
        n = 0
        while not sim.frame.stage.startswith("S2 ") or n < 6:
            sim.advance()
            n += 1
            if not sim.active or n > 60:
                self.fail("未推进到 S2 中段")
        twin.draw_twin(self.app)
        c = self.view["canvas"]
        item = c.find_withtag("lift_left")
        self.assertTrue(item, "模拟播放中应绘制左腿柱状图")
        y1, y2 = c.coords(item[0])[1], c.coords(item[0])[3]
        length = abs(y2 - y1)
        z = sim.frame.pose["feet"]["left"]["z_mm"]
        lift = self.app.gait_params.lift_mm
        self.assertGreater(z, 0)
        self.assertLess(z, lift, "应停在 S2 中段而非完成全程")
        self.assertGreater(length, 1.0, "中段柱形应可见（非零）")
        self.assertLess(length, 27.0, "中段柱形不应满格")
        self.assertLess(abs(length / 28.0 - z / lift), 0.1)


if __name__ == "__main__":
    unittest.main()
