"""Offline display regressions; real Tk when available, never opens a port."""
import math
import os
import sys
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_gait_linkage as linkage  # Install headless import surfaces if needed.
import test_gait_ui as ui
from motor_control.gait_avoidance import TWO_MODE, avoidance_path
from motor_control.gait_planner import GaitParams, plan_gait_stages, plan_swing_trajectory, smoothstep5
from motor_control.gait_twin_timing import inverse_progress, plan_sample_times
from motor_control.ui import gait_twin as twin


class TwinTimingTests(unittest.TestCase):
    def test_inverse_quintic(self):
        for t in (0, .001, .1, .25, .5, .75, .9, .999, 1):
            self.assertAlmostEqual(inverse_progress(smoothstep5(t)), t, places=7)

    def test_four_actions_use_actual_route_and_signed_segment_clocks(self):
        app = linkage.mechanism()
        for side in ("left", "right"):
            for arc in (60, -60):
                for sign in (1, -1):
                    with self.subTest(side=side, arc=arc, sign=sign):
                        params = replace(app.gait_params, trajectory_mode=TWO_MODE,
                                         mr1_sign=sign, mr2_sign=sign)
                        route = app._gait_landing_route(side, params, arc_deg=arc)
                        report = plan_swing_trajectory(params, side=side, route=route, arc_deg=arc)
                        stage = next(s for s in plan_gait_stages(
                            params, side=side, route=route, arc_deg=arc) if s.stage_id == "S4")
                        with patch("motor_control.gait_twin_timing.plan_gait_stages",
                                   wraps=plan_gait_stages) as planner:
                            times = plan_sample_times(report, params)
                        planner.assert_called_once()
                        self.assertEqual(planner.call_args.kwargs["route"], route)
                        self.assertEqual(times[0], 0)
                        self.assertAlmostEqual(times[-1], stage.duration_s, places=7)
                        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
                        path = avoidance_path(route[3], arc)
                        for i, knot in enumerate(path.knots[1:]):
                            index = min(range(len(times)), key=lambda j: abs(report.samples[j].phi_deg-knot[1]))
                            self.assertAlmostEqual(times[index], sum(stage.group_durations[:i+1]), places=6)
                        # Reports sample angle, not time: recover a sample's angle
                        # from the segment's quintic clock (interior, not a knot).
                        i = len(times)//3
                        elapsed, start = 0, 0
                        for duration, end in zip(stage.group_durations, path.knots[1:]):
                            if times[i] <= elapsed+duration:
                                u = (times[i]-elapsed)/duration
                                phi = start+(end[1]-start)*smoothstep5(u)
                                self.assertAlmostEqual(phi, report.samples[i].phi_deg, places=6)
                                break
                            elapsed += duration
                            start = end[1]
        self.assertEqual(app.commands, [])

    def test_legacy_clock_matches_existing_smoothstep_samples(self):
        params = GaitParams()
        report = plan_swing_trajectory(params, side="left", arc_deg=-60)
        times = plan_sample_times(report, params)
        for sample, t in zip(report.samples, times):
            self.assertAlmostEqual(t/times[-1], sample.s, places=7)

    def test_whole_map_fits_between_hud_and_lift_bars(self):
        xs, ys = zip(*twin.twin_pad_window().values())
        bounds = min(xs)-.6, max(xs)+.6, min(ys)-.6, max(ys)+.6
        for width, height in ((340, 340), (600, 260), (1080, 620)):
            project, _ = twin.twin_projection(width, height, *bounds)
            for cx, cy in twin.twin_pad_window().values():
                for i in range(6):
                    a = math.radians(30+60*i)
                    x, y = project((cx+math.cos(a)/math.sqrt(3), cy+math.sin(a)/math.sqrt(3)))
                    self.assertGreaterEqual(x, 12)
                    self.assertLessEqual(x, width-12)
                    self.assertGreaterEqual(y, 76)
                    self.assertLessEqual(y, height-74)


@unittest.skipUnless(ui.HAS_TK, "real Tk unavailable")
class TwinWindowTests(unittest.TestCase):
    def setUp(self):
        ui.GaitLayoutTests.setUp(self)
        self.root.geometry("1024x620")
        self.root.update()
        self.view = self.app.gait_widgets["twin"]
        self.canvas = self.view["canvas"]
        self.pose = {"phi_deg": 0, "beta_deg": 180, "route_pads": {},
                     "feet": {"left": {"center": (-1, 0), "psi_deg": 30, "z_mm": 0},
                              "right": {"center": (0, 0), "psi_deg": 30, "z_mm": 0}}}
        self.snapshot = {"pose": self.pose, "axes": {}, "state": "estimated", "message": "估算",
                         "reference_key": "step1", "world_key": "world1"}
        self.app._gait_twin_snapshot = lambda: self.snapshot
        twin.refresh_twin_panel(self.app)
        self.addCleanup(lambda: self.assertFalse(self.errors))

    def prepare_plan(self):
        self.app.gait_params = replace(self.app.gait_params, trajectory_mode=TWO_MODE)
        self.app._gait_last_report = plan_swing_trajectory(self.app.gait_params, side="left")
        self.view["mode_var"].set("plan")

    def test_plan_without_report_never_displays_live_pose_or_history(self):
        for report in (None, SimpleNamespace(samples=())):
            self.app._gait_last_report = report
            self.view["mode_var"].set("plan")
            twin.draw_twin(self.app)
            self.assertTrue(self.canvas.find_withtag("plan_placeholder"))
            self.assertFalse(self.canvas.find_withtag("live_left"))
            self.assertFalse(self.canvas.find_withtag("history_left"))
            self.assertIn("暂无有效计划", self.view["coordinates"].cget("text"))

    def test_hide_paths_also_hides_plan_tracks_but_not_body(self):
        self.prepare_plan()
        self.view["show_path"].set(False)
        twin.draw_twin(self.app)
        self.assertFalse(self.canvas.find_withtag("plan_center"))
        self.assertFalse(self.canvas.find_withtag("plan_track_0"))
        self.assertTrue(self.canvas.find_withtag("plan_ghost"))
        self.assertTrue(self.canvas.find_withtag("plan_support"))

    def test_new_origin_cannot_display_last_world_frozen_pose(self):
        self.snapshot.update(pose=None, world_key="world2", state="uncalibrated")
        twin.refresh_twin_panel(self.app)
        self.assertIsNone(self.view["last_pose"])
        self.assertFalse(self.canvas.find_withtag("live_left"))
        self.assertTrue(self.canvas.find_withtag("twin_placeholder"))

    def test_owned_run_forces_live_and_stops_preview_timer(self):
        from motor_control.ui.gait_tab import play_preview_animation, _preview_anim_tick
        self.prepare_plan()
        play_preview_animation(self.app)
        anim = self.app.gait_widgets["preview_anim"]
        self.assertIsNotNone(anim["job"])
        self.app._gait_owned = {2: "test"}
        _preview_anim_tick(self.app)
        self.assertFalse(anim["playing"])
        self.assertIsNone(anim["job"])
        twin.draw_twin(self.app)
        self.assertTrue(self.canvas.find_withtag("live_left"))
        self.assertFalse(self.canvas.find_withtag("plan_center"))
        twin.refresh_twin_panel(self.app)
        self.assertEqual(self.view["mode_var"].get(), "live")
        self.assertTrue(all(str(c.cget("state")) == "disabled" for c in self.view["mode_controls"]))

    def test_disconnect_same_reference_breaks_curve_and_clear_resets_clock(self):
        twin._clear_live_curve(self.view)
        with patch.object(twin.time, "monotonic", side_effect=(10, 11, 20, 21, 30)):
            twin.refresh_twin_panel(self.app)
            twin.refresh_twin_panel(self.app)
            self.snapshot.update(pose=None, state="stale")
            twin.refresh_twin_panel(self.app)
            self.snapshot.update(pose=self.pose, state="estimated")
            twin.refresh_twin_panel(self.app)
            twin.refresh_twin_panel(self.app)
            segments = twin._curve_segments(self.view["curve_live"], 0)
            self.assertEqual([len(s) for s in segments], [2, 2])
            twin.clear_twin_history(self.app)
            self.assertIsNone(self.view["curve_t0"])
            twin.refresh_twin_panel(self.app)
            self.assertEqual(self.view["curve_live"][0][0], 0)

    def test_new_world_or_geometry_drops_old_curve_points(self):
        for change in ("world", "geometry"):
            self.view["curve_live"].append((-999, [-99]*6, 999))
            if change == "world":
                self.snapshot["world_key"] = "world2"
            else:
                self.app.gait_params = replace(self.app.gait_params, geometry=replace(
                    self.app.gait_params.geometry, node_radius_mm=7))
            twin.refresh_twin_panel(self.app)
            self.assertEqual(len(self.view["curve_live"]), 1)
            self.assertEqual(self.view["curve_live"][0][0], 0)

    def test_rolling_minimum_ignores_old_offscreen_collision(self):
        self.view["curve_live"].clear()
        self.view["curve_live"].extend(((0, [-100]*6, 0), (100, [12]*6, 0), (101, [15]*6, 0)))
        twin._draw_curve(self.app, self.view)
        c = self.view["curve_canvas"]
        self.assertEqual(c.itemcget(c.find_withtag("curve_best")[0], "text"), "窗口最低 12.00mm")
        self.assertTrue(c.find_withtag("curve_delta"))

    def test_plan_checks_six_tips_and_updates_coordinates(self):
        self.prepare_plan()
        with patch.object(twin, "_tip_surface", side_effect=(20, 21, 22, 1, 2, 3)) as surface:
            twin.draw_twin(self.app)
        self.assertEqual(surface.call_count, 6)
        text = self.canvas.itemcget(self.canvas.find_withtag("hud_nearest")[0], "text")
        self.assertIn("1.00mm", text)
        self.assertIn("非整腿净间隙", text)
        self.assertIn("评估位置（非实机）", self.view["coordinates"].cget("text"))

    def test_plan_cache_rebuilds_only_when_report_or_parameters_change(self):
        self.prepare_plan()
        with patch.object(twin, "plan_tip_series", wraps=twin.plan_tip_series) as compute:
            twin.draw_twin(self.app)
            twin.draw_twin(self.app)
            self.assertEqual(compute.call_count, 1)
            self.app.gait_params = replace(self.app.gait_params, swing_speed_deg_s=9)
            twin.draw_twin(self.app)
            self.assertEqual(compute.call_count, 2)

    def test_static_map_retained_until_view_changes_no_item_leak(self):
        ids = self.canvas.find_withtag("map_hex")
        count = len(self.canvas.find_all())
        for _ in range(8):
            twin.draw_twin(self.app)
            self.assertEqual(self.canvas.find_withtag("map_hex"), ids)
            self.assertEqual(len(self.canvas.find_all()), count)
        self.app.gait_widgets["twin_view"]["zoom"] = 1.5
        twin.draw_twin(self.app)
        self.assertNotEqual(self.canvas.find_withtag("map_hex"), ids)
        self.assertEqual(len(self.canvas.find_withtag("map_hex")), 25)

    def test_large_window_shares_data_resizes_picks_and_closes_without_control(self):
        from motor_control.ui.gait_twin_window import open_twin_window, refresh_twin_window
        from motor_control.gait_map import map_label, map_pads
        calls = []
        self.app._gait_abort_clicked = lambda: calls.append("abort")
        params = self.app.gait_params
        before = list(self.view["history"].points)
        open_twin_window(self.app)
        detail = self.view["large_window"]
        window = detail["window"]
        window.attributes("-alpha", 0)
        open_twin_window(self.app)
        self.assertIs(self.view["large_window"]["window"], window)
        for size in ("640x500", "1100x820"):
            window.geometry(size)
            self.root.update()
            self.assertEqual(len(detail["canvas"].find_withtag("map_hex")), 25)
            self.assertIs(detail["history"], self.view["history"])
            self.assertIs(detail["curve_live"], self.view["curve_live"])
        detail["view_state"].update(zoom=1.4, pan_x=12, pan_y=-8)
        refresh_twin_window(self.app, self.view)
        self.assertEqual(self.app.gait_widgets["twin_view"]["zoom"], 1)
        self.view["select_side"].set("left")
        x, y = detail["project"](map_pads()["C"])
        twin._map_click(self.app, SimpleNamespace(x=x, y=y), detail)
        self.assertEqual(self.view["start_left"].get(), map_label("C"))
        self.assertIs(self.app.gait_params, params)
        self.assertEqual(list(self.view["history"].points), before)
        window.tk.call(window.protocol("WM_DELETE_WINDOW"))
        self.root.update()
        self.assertNotIn("large_window", self.view)
        self.assertEqual(calls, [])
        twin.refresh_twin_panel(self.app)


if __name__ == "__main__":
    unittest.main()
