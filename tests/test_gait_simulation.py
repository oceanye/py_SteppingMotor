"""Virtual gait transactions and real Tk integration; never open a motor port."""
import os
import sys
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_gait_linkage as linkage
import test_gait_ui as ui
from motor_control.gait_avoidance import TWO_MODE
from motor_control.gait_map import map_label, map_pads, start_pair_reference
from motor_control.gait_planner import GaitGeometry, GaitParams
from motor_control.gait_simulation import GaitSimulation
from motor_control.ui import gait_simulation as sim_ui, gait_twin as twin


# Deliberately thin fixture for both certified modes; NOT a hardware preset.
PARAMS = GaitParams(trajectory_mode=TWO_MODE, geometry=GaitGeometry(
    arm_length_mm=60, hub_radius_mm=3, arm_radius_mm=1, node_radius_mm=1, safety_margin_mm=.5))


def complete(sim):
    while sim.active:
        sim.advance()


class SimulationTests(unittest.TestCase):
    def test_four_actions_match_real_route_and_commit_only_at_end(self):
        controller = linkage.mechanism()
        for side in ("left", "right"):
            for arc in (60, -60):
                with self.subTest(side=side, arc=arc):
                    sim = GaitSimulation(PARAMS)
                    report = sim.preview(side, arc)
                    self.assertEqual(report.route, controller._gait_landing_route(side, PARAMS, arc_deg=arc))
                    self.assertEqual(sim.completed_steps, 0)
                    self.assertEqual(sim.supports, ("A", "B"))
                    sim.begin(side, arc)
                    times = [f.time_s for f in sim.frames]
                    self.assertTrue(all(b >= a for a, b in zip(times, times[1:])))
                    for _ in range(len(sim.frames)-1):
                        self.assertFalse(sim.advance())
                        self.assertEqual(sim.supports, ("A", "B"))
                    self.assertTrue(sim.advance())
                    self.assertFalse(sim.advance())
                    self.assertEqual(sim.completed_steps, 1)
                    target, pivot = report.route[1:3]
                    self.assertEqual(sim.supports, (target, pivot) if side == "left" else (pivot, target))
                    self.assertAlmostEqual(sim.beta, 180-arc)
                    self.assertTrue(all(foot["z_mm"] == 0 for foot in sim.frame.pose["feet"].values()))
        self.assertEqual(controller.commands, [])

    def test_three_steps_accumulate_pose_world_psi_and_separate_path_segments(self):
        sim = GaitSimulation(PARAMS)
        previous = sim.supports
        for n in range(3):
            report = sim.begin("left", 60)
            self.assertEqual(report.route[0], previous[0])
            first = sim.frames[0].pose["feet"]["left"]
            self.assertAlmostEqual(first["psi_deg"], sim.psis[0])
            self.assertEqual(first["center"], sim.frame.pose["feet"]["left"]["center"])
            complete(sim)
            self.assertEqual(sim.completed_steps, n+1)
            previous = sim.supports
        self.assertEqual(len(sim.history.landings), 3)
        self.assertEqual(len(sim.history.segments()), 3)
        self.assertEqual(len(sim.history.points), len(set(sim.history.points)))

    def test_linear_stages_include_retract_land_and_zero_final_height(self):
        sim = GaitSimulation(PARAMS)
        sim.begin("left", 60)
        phases = {f.stage.split()[0]: f for f in sim.frames}
        self.assertTrue({"S0", "S1", "S2", "S2B", "S4", "S5", "S5B", "S6", "S7"} <= set(phases))
        heights = lambda phase: tuple(phases[phase].pose["feet"][s]["z_mm"] for s in ("left", "right"))
        self.assertEqual(heights("S2"), (PARAMS.lift_mm, PARAMS.lift_mm))
        self.assertEqual(heights("S2B"), (0, PARAMS.lift_mm))
        self.assertEqual(heights("S5B"), (PARAMS.lift_mm, PARAMS.lift_mm))
        self.assertEqual(heights("S7"), (0, 0))

    def test_next_preview_preserves_continuous_claw_indices_after_rotation(self):
        sim = GaitSimulation(PARAMS)
        sim.begin("left", 60)
        complete(sim)
        for side in ("left", "right"):
            report = sim.preview(side, 60)
            series = twin.plan_tip_series(report, sim.params, sim.psis)
            distances = twin.tip_surface_distances(sim.frame.pose, sim.params)
            for actual, expected in zip(series[0][1], distances):
                self.assertAlmostEqual(actual, expected, places=7)

    def test_clear_path_retains_stance_count_and_clock_cancel_does_not_commit(self):
        sim = GaitSimulation(PARAMS)
        sim.begin("left", 60)
        complete(sim)
        saved = (sim.supports, sim.beta, sim.psis, sim.elapsed_s, sim.completed_steps)
        sim.clear_path()
        self.assertFalse(sim.history.points)
        self.assertFalse(sim.history.landings)
        self.assertEqual((sim.supports, sim.beta, sim.psis, sim.elapsed_s, sim.completed_steps), saved)
        sim.begin("right", 60)
        for _ in range(110):
            sim.advance()
        sim.paused = True
        frame = sim.frame
        self.assertFalse(sim.advance())
        self.assertIs(frame, sim.frame)
        sim.cancel()
        self.assertFalse(sim.active)
        self.assertFalse(sim.history.points)
        self.assertEqual((sim.supports, sim.beta, sim.psis, sim.elapsed_s, sim.completed_steps), saved)

    def test_unsafe_model_and_out_of_map_never_start_or_change_stance(self):
        sim = GaitSimulation(GaitParams(trajectory_mode=TWO_MODE))
        with self.assertRaisesRegex(ValueError, "校验未通过"):
            sim.begin("left", 60)
        self.assertFalse(sim.active)
        self.assertEqual(sim.completed_steps, 0)
        self.assertEqual(sim.supports, ("A", "B"))
        # Select an adjacent boundary pair and a direction leading off the map.
        found = False
        for a in map_pads():
            for b in map_pads():
                try:
                    start_pair_reference((a, b))
                except ValueError:
                    continue
                edge = GaitSimulation(PARAMS, (a, b))
                for side in ("left", "right"):
                    for arc in (60, -60):
                        try:
                            edge.route(side, arc)
                        except ValueError:
                            with self.assertRaisesRegex(ValueError, "5×5"):
                                edge.begin(side, arc)
                            self.assertEqual(edge.supports, (a, b))
                            self.assertFalse(edge.active)
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if found:
                break
        self.assertTrue(found)


@unittest.skipUnless(ui.HAS_TK, "real Tk unavailable")
class SimulationUITests(unittest.TestCase):
    def setUp(self):
        ui.GaitLayoutTests.setUp(self)
        from motor_control.ui.gait_tab import load_gait_fields
        self.app.gait_params = PARAMS
        load_gait_fields(self.app)
        self.app._gait_supports, self.app._gait_beta_deg = ("A", "B"), 180
        self.app._save_gait_params = Mock(side_effect=AssertionError("Simulation must not save physical config"))
        self.app.commands = []
        self.root.geometry("1024x620")
        self.root.update()
        self.view = self.app.gait_widgets["twin"]

    def finish_ui_step(self):
        # Advance actual callback without wall-clock waiting, cancelling only
        # its scheduled timer. This exercises rendering/curves/commit together.
        while self.view["simulation"].active:
            job = self.view.pop("simulation_job", None)
            if job:
                self.root.after_cancel(job)
            sim_ui._tick(self.app)

    def test_cumulative_two_steps_preview_and_large_window_without_physical_writes(self):
        from motor_control.ui.gait_twin_window import open_twin_window
        original = self.app.gait_params
        sim_ui.toggle_simulation(self.app)
        self.finish_ui_step()
        sim = self.view["simulation"]
        first = sim.supports
        self.assertEqual(first, ("C", "B"))
        self.assertEqual(sim.completed_steps, 1)
        # Single preview computes a candidate at the new virtual stance only.
        linkage.StepperGUI._gait_run_dry_run(self.app, interactive=False)
        self.assertEqual(self.app._gait_last_report.route[0], "C")
        self.assertEqual(sim.supports, first)
        self.assertEqual(sim.completed_steps, 1)
        sim_ui.toggle_simulation(self.app)
        self.finish_ui_step()
        self.assertEqual(sim.completed_steps, 2)
        self.assertNotEqual(sim.supports, first)
        self.assertEqual(self.app._gait_supports, ("A", "B"))
        self.assertEqual(self.app._gait_beta_deg, 180)
        self.assertIs(self.app.gait_params, original)
        self.app._save_gait_params.assert_not_called()
        self.assertEqual(self.app.commands, [])
        c = self.view["canvas"]
        self.assertTrue(c.find_withtag("sim_left"))
        self.assertTrue(c.find_withtag("sim_history_left"))
        self.assertFalse(c.find_withtag("live_left"))
        self.assertFalse(self.view["curve_live"])
        self.assertTrue(sim.curve)
        open_twin_window(self.app)
        detail = self.view["large_window"]
        detail["window"].attributes("-alpha", 0)
        self.root.update()
        self.assertIs(detail["simulation"], sim)
        self.assertTrue(detail["canvas"].find_withtag("sim_left"))

    def test_pause_cancel_clear_and_reset_have_different_effects(self):
        sim_ui.toggle_simulation(self.app)
        sim = self.view["simulation"]
        sim_ui.toggle_simulation(self.app)
        self.assertTrue(sim.paused)
        self.assertNotIn("simulation_job", self.view)
        sim_ui.toggle_simulation(self.app)
        self.finish_ui_step()
        stance = sim.supports
        twin.clear_twin_history(self.app)
        self.assertEqual(sim.supports, stance)
        self.assertEqual(sim.completed_steps, 1)
        self.assertFalse(sim.history.points)
        self.assertFalse(sim.curve)
        sim_ui.toggle_simulation(self.app)
        sim_ui.discard_simulation_step(self.app)
        self.assertEqual(sim.completed_steps, 1)
        self.assertEqual(sim.supports, stance)
        self.view["start_left"].set(map_label("C"))
        self.view["start_right"].set(map_label("A"))
        new = sim_ui.reset_simulation(self.app)
        self.assertEqual(new.supports, ("C", "A"))
        self.assertEqual(new.completed_steps, 0)
        self.assertEqual(self.app.gait_params.initial_supports, ("A", "B"))

    def test_preview_playback_replans_after_virtual_stance_changes(self):
        from motor_control.ui.gait_tab import stop_preview_animation
        self.app._gait_run_dry_run = lambda **kw: linkage.StepperGUI._gait_run_dry_run(self.app, **kw)
        sim_ui.toggle_simulation(self.app)
        self.finish_ui_step()
        sim_ui.preview_from_simulation(self.app)
        old_report = self.app._gait_last_report
        sim_ui.toggle_simulation(self.app)
        self.finish_ui_step()
        sim = self.view["simulation"]
        linkage.StepperGUI._gait_play_preview_clicked(self.app)
        self.assertIsNot(self.app._gait_last_report, old_report)
        self.assertEqual(self.app._gait_last_report.route[0], sim.supports[0])
        self.assertEqual(sim.completed_steps, 2)
        stop_preview_animation(self.app)
        self.app._save_gait_params.assert_not_called()

    def test_invalid_parameters_busy_and_mode_switch_stop_scheduled_simulation(self):
        sim_ui.toggle_simulation(self.app)
        sim = self.view["simulation"]
        self.app.gait_field_vars["arm_length_mm"].set("not-a-number")
        self.assertFalse(sim.active)  # invalidation cancels incomplete step
        sim_ui.toggle_simulation(self.app)
        self.assertFalse(sim.active)
        self.assertEqual(sim.completed_steps, 0)
        from motor_control.ui.gait_tab import load_gait_fields
        load_gait_fields(self.app)
        sim_ui.toggle_simulation(self.app)
        self.view["mode_var"].set("live")
        job = self.view.pop("simulation_job", None)
        if job:
            self.root.after_cancel(job)
        sim_ui._tick(self.app)
        self.assertTrue(sim.paused)
        self.assertNotIn("simulation_job", self.view)
        self.app._gait_owned = {2: "test"}
        twin.refresh_twin_panel(self.app)
        self.assertFalse(sim.active)
        self.assertEqual(self.view["mode_var"].get(), "live")

    def test_unsafe_simulation_is_blocked_but_single_preview_still_available(self):
        from motor_control.ui.gait_tab import load_gait_fields
        self.app.gait_params = GaitParams(trajectory_mode=TWO_MODE)
        load_gait_fields(self.app)
        sim_ui.toggle_simulation(self.app)
        sim = self.view["simulation"]
        self.assertFalse(sim.active)
        self.assertEqual(sim.completed_steps, 0)
        self.assertIn("校验未通过", self.app.gait_widgets["sim_status"].cget("text"))
        self.assertTrue(sim_ui.preview_from_simulation(self.app))
        self.assertFalse(self.app._gait_last_report.feasible)

    def test_focus_time_strings_and_keyboard_navigation_after_recent_click(self):
        viewport = self.app.gait_widgets["run_viewport"]
        button = self.app.gait_widgets["advance"]
        viewport.canvas.yview_moveto(0)
        before = viewport.canvas.yview()
        with patch("motor_control.ui.scrollable.time.monotonic", return_value=100):
            for timestamp in ("??", "", None, 123, "123"):
                viewport._mark_pointer_focus(SimpleNamespace(widget=button, time=timestamp))
                viewport._reveal_focus(SimpleNamespace(widget=button, time="??"))
                self.assertEqual(viewport.canvas.yview(), before)
            viewport._keyboard_focus(None)
            viewport._reveal_focus(SimpleNamespace(widget=button))
            self.assertNotEqual(viewport.canvas.yview(), before)


if __name__ == "__main__":
    unittest.main()
