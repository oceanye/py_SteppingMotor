"""Cross-step windows and two-step map layers; never connects to hardware."""
import os
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_gait_step_curves as step_tests
import test_gait_ui as ui
from motor_control.gait_curve import CurveTimeline, clip_series, rolling_window
from motor_control.gait_planner import default_gait_params
from motor_control.gait_simulation import GaitSimulation
from motor_control.gait_twin import TwinHistory
from motor_control.ui import gait_twin as twin
from motor_control.ui.gait_curve import curve_payload
from motor_control.ui.gait_paths import FOOT_COLORS, PAST_COLORS


def finish_step(sim, side="left", arc=60):
    sim.begin(side, arc)
    while sim.active:
        sim.advance()
        point = (sim.curve_time_s, (50.,)*6, sim.segment)
        if not sim.curve or sim.curve[-1] != point:
            sim.curve.append(point)


class RollingDisplayTests(unittest.TestCase):
    def test_archive_keeps_reached_part_offsets_and_gaps(self):
        timeline = CurveTimeline()
        timeline.step = 1
        ref = dict(series=[(0, [10]*6), (60, [70]*6), (120, [130]*6)], current_s=45)
        timeline.archive(ref, [(0, (2,)*6, 0), (60, (8,)*6, 0)])
        self.assertEqual([p[0] for p in timeline.history], [0, 45])
        self.assertEqual(timeline.history[-1][1], (55.,)*6)
        self.assertEqual(timeline.measured[-1], (45, (6.5,)*6, (1, 0)))
        timeline.step = 2
        ref["current_s"] = 75
        timeline.archive(ref)
        self.assertEqual(timeline.elapsed_s, 120)
        visible = clip_series(list(timeline.history), *rolling_window(120))
        self.assertEqual((visible[0][0], visible[-1][0]), (30, 120))
        self.assertEqual({p[2] for p in visible}, {1, 2})
        timeline.clear()
        self.assertFalse(timeline.history)
        self.assertFalse(timeline.measured)
        self.assertEqual(timeline.elapsed_s, 120)

    def test_clear_reference_cannot_reintroduce_earlier_samples(self):
        timeline = CurveTimeline()
        timeline.archive(dict(series=[(0, [1]*6), (80, [2]*6)],
                              retain_from_s=40, current_s=60))
        self.assertEqual([p[0] for p in timeline.history], [40, 60])
        self.assertEqual(timeline.elapsed_s, 60)

    def test_simulation_window_really_slides_across_steps_and_cancel_restores(self):
        sim = GaitSimulation(replace(default_gait_params(), swing_speed_deg_s=2))
        finish_step(sim)
        finish_step(sim)
        current = sim.curve_time_s
        start, end = rolling_window(current)
        self.assertGreater(start, 0)
        self.assertEqual(end-start, 90)
        self.assertEqual({p[2] for p in clip_series(list(sim.curve), start, end)}, {1, 2})
        before = (sim.supports, sim.psis, list(sim.curve), list(sim.history.points))
        sim.begin("left", 60)
        while sim.curve_time_s == current:
            sim.advance()
        sim.curve.append((sim.curve_time_s, (50.,)*6, sim.segment))
        sim.cancel()
        self.assertEqual(sim.curve_time_s, current)
        self.assertEqual((sim.supports, sim.psis, list(sim.curve), list(sim.history.points)), before)
        self.assertEqual(sim.history.current_step, 2)
        sim.clear_path()
        self.assertEqual(sim.curve_time_s, current)
        self.assertEqual(sim.supports, before[0])
        self.assertFalse(sim.curve)

    def test_telemetry_gap_does_not_age_a_step(self):
        history = TwinHistory()
        snapshot = dict(state="estimated", world_key="world")
        def sample(step, x):
            snapshot.update(reference_key=step, pose={"feet": {
                "left": {"center": (x, 0), "psi_deg": 30},
                "right": {"center": (0, 0), "psi_deg": 30}}})
            history.observe(snapshot)
        for step in (1, 2, 3):
            sample(step, 0)
            sample(step, .1)
        history.observe(dict(snapshot, pose=None, state="stale"))
        sample(3, .2)
        sample(3, .3)
        recent = history.recent_segments()
        self.assertEqual([age for age, _ in recent], [1, 0, 0])
        self.assertEqual([history.segment_steps[chunk[0][0]] for _, chunk in recent], [2, 3, 3])
        self.assertEqual(len(history.segments()), 4)  # Old ledger retained, not displayed.


@unittest.skipUnless(ui.HAS_TK, "real Tk unavailable")
class RollingDisplayUITests(unittest.TestCase):
    def setUp(self):
        step_tests.StepCurveUITests.setUp(self)

    make_live_run = step_tests.StepCurveUITests.make_live_run

    def test_map_only_two_steps_pale_dark_and_active_dashed_target(self):
        sim = GaitSimulation(self.app.gait_params)
        for _ in range(3):
            finish_step(sim)
        self.view["simulation"] = sim
        self.view["mode_var"].set("sim")
        twin.draw_twin(self.app)
        c = self.view["canvas"]
        self.assertFalse(c.find_withtag("path_step_1"))
        for step, palette in ((2, PAST_COLORS), (3, FOOT_COLORS)):
            items = c.find_withtag(f"path_step_{step}")
            self.assertTrue(items)
            for item in items:
                self.assertIn(c.itemcget(item, "fill"), palette.values())
                self.assertFalse(c.itemcget(item, "dash"))
        self.assertEqual(len(sim.history.landings), 3)
        sim.begin("left", 60)
        sim.advance()
        twin.draw_twin(self.app)
        self.assertFalse(c.find_withtag("path_step_2"))
        self.assertTrue(c.find_withtag("path_previous"))
        targets = [i for i in c.find_withtag("forecast_ghost") if c.type(i) == "line"]
        self.assertTrue(targets)
        for item in targets:
            self.assertEqual(c.itemcget(item, "fill"), FOOT_COLORS["left"])
            self.assertTrue(c.itemcget(item, "dash"))
        sim.cancel()
        twin.draw_twin(self.app)
        self.assertFalse(c.find_withtag("forecast_ghost"))
        self.assertTrue(c.find_withtag("path_step_2"))
        self.assertTrue(c.find_withtag("path_step_3"))
        self.app._send_and_read.assert_not_called()

    def test_candidate_preserves_rolling_history_and_matches_side_colour(self):
        from motor_control.ui.gait_tab import _gait_mode_selected
        sim = GaitSimulation(self.app.gait_params)
        for _ in range(5):
            finish_step(sim)
        self.view["simulation"] = sim
        self.view["mode_var"].set("sim")
        before = curve_payload(self.app, self.view)
        self.assertGreater(before["start"], 0)
        for action, side in (("L+", "left"), ("R-", "right")):
            _gait_mode_selected(self.app, action)
            after = curve_payload(self.app, self.view)
            for key in ("data", "current", "start", "end"):
                self.assertEqual(after[key], before[key])
            c = self.view["canvas"]
            lines = [i for i in c.find_withtag("candidate_ghost") if c.type(i) == "line"]
            self.assertTrue(lines)
            for item in lines:
                self.assertEqual(c.itemcget(item, "fill"), FOOT_COLORS[side])
                self.assertTrue(c.itemcget(item, "dash"))
        self.assertEqual(sim.completed_steps, 5)
        self.app._save_gait_params.assert_not_called()
        self.app._send_and_read.assert_not_called()

    def test_live_cross_step_timeline_shared_by_large_window_and_clear(self):
        from motor_control.ui.gait_twin_window import open_twin_window
        run = self.make_live_run()
        open_twin_window(self.app)
        detail = self.view["large_window"]
        detail["window"].attributes("-alpha", 0)
        self.root.update()
        self.snapshot["pose"]["phi_deg"] = run.twin_report.samples[-1].phi_deg
        twin.refresh_twin_panel(self.app)
        previous_end = self.view["curve_display"]["current"]
        self.assertGreater(previous_end, 0)
        second = self.make_live_run("right", -60)
        self.assertAlmostEqual(self.view["curve_display"]["current"], previous_end)
        self.assertEqual({p[2] for p in self.view["curve_display"]["data"]}, {1, 2})
        self.assertIs(detail["live_curve_timeline"], self.view["live_curve_timeline"])
        self.assertEqual(detail["curve_display"], self.view["curve_display"])
        self.snapshot["pose"]["phi_deg"] = second.twin_report.samples[-1].phi_deg/2
        twin.refresh_twin_panel(self.app)
        current = self.view["curve_display"]["current"]
        twin.clear_twin_history(self.app)
        self.assertAlmostEqual(self.view["curve_display"]["current"], current)
        self.assertTrue(all(p[0] >= current for p in self.view["curve_display"]["data"]))
        self.make_live_run()
        self.assertAlmostEqual(self.view["curve_display"]["current"], current)
        self.assertTrue(all(p[0] >= current for p in self.view["curve_display"]["data"]))
        self.assertFalse(self.view["curve_display"]["measured"])

    def test_live_partial_step_archives_no_future_and_new_world_resets(self):
        run = self.make_live_run()
        self.snapshot["pose"]["phi_deg"] = run.twin_report.samples[-1].phi_deg/2
        twin.refresh_twin_panel(self.app)
        current = self.view["curve_display"]["current"]
        self.make_live_run("right", -60)
        archived = list(self.view["live_curve_timeline"].history)
        self.assertTrue(archived)
        self.assertAlmostEqual(max(p[0] for p in archived), current)
        self.snapshot["world_key"] = "new origin"
        twin.refresh_twin_panel(self.app)
        self.assertEqual(self.view["curve_display"]["current"], 0)
        self.assertFalse(self.view["live_curve_timeline"].history)

    def test_live_six_steps_scroll_and_never_mix_virtual_data(self):
        sim = GaitSimulation(self.app.gait_params)
        sim.curve.extend([(0, (-999.,)*6, 1), (150, (-999.,)*6, 1)])
        self.view["simulation"] = sim
        previous = 0.
        for step in range(6):
            run = self.make_live_run("left", 60)
            self.assertAlmostEqual(self.view["curve_display"]["current"], previous)
            self.snapshot["pose"]["phi_deg"] = run.twin_report.samples[-1].phi_deg
            twin.refresh_twin_panel(self.app)
            previous = self.view["curve_display"]["current"]
        payload = curve_payload(self.app, self.view)
        self.assertGreater(payload["start"], 0)
        self.assertAlmostEqual(payload["end"]-payload["start"], 90)
        self.assertGreater(len({p[2] for p in payload["data"]}), 1)
        self.assertTrue(all(payload["start"] <= t <= payload["end"] for t, _, _ in payload["data"]))
        self.assertTrue(all(v > -999 for _, vals, _ in payload["data"] for v in vals))
        self.assertFalse(payload["measured"])
        self.assertTrue(payload["grey"])
        self.app._send_and_read.assert_not_called()

    def test_active_forecast_uses_start_phase_and_disappears_at_endpoint(self):
        for side, arc in (("left", 60), ("left", -60), ("right", 60), ("right", -60)):
            sim = GaitSimulation(self.app.gait_params)
            sim.begin(side, arc)
            sim.advance()
            self.view["simulation"] = sim
            self.view["mode_var"].set("sim")
            twin.draw_twin(self.app)
            c = self.view["canvas"]
            lines = [i for i in c.find_withtag("forecast_ghost") if c.type(i) == "line"]
            self.assertTrue(lines)
            for line in lines:
                self.assertEqual(c.itemcget(line, "fill"), FOOT_COLORS[side])
                self.assertTrue(c.itemcget(line, "dash"))
            while sim.active and abs(sim.frame.pose["phi_deg"]) < 60:
                sim.advance()
            twin.draw_twin(self.app)
            self.assertFalse(c.find_withtag("forecast_ghost"))
            sim.cancel()


if __name__ == "__main__":
    unittest.main()
