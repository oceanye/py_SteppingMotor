"""Display-only 60-degree map rotation; no hardware or coordinate writes."""
import math
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_gait_step_curves as step_tests
import test_gait_ui as ui
from test_gait_rolling_display import finish_step
from motor_control.gait_map import map_label, map_pads
from motor_control.gait_simulation import GaitSimulation
from motor_control.ui import gait_twin as twin
from motor_control.ui.common import canvas_view_zoom, reset_canvas_view
from motor_control.ui.gait_curve import curve_payload


class RotationProjectionTests(unittest.TestCase):
    def test_sixty_is_counterclockwise_similarity_not_world_coordinate_change(self):
        project, scale = twin.twin_projection(600, 500, -3, 3, -2, 2, rotation_deg=60)
        origin = project((0, 0))
        x, y = project((1, 0))
        self.assertAlmostEqual(x-origin[0], scale/2)
        self.assertAlmostEqual(y-origin[1], -scale*math.sqrt(3)/2)
        a, b = (-1.2, .6), (.9, -.8)
        self.assertAlmostEqual(math.dist(project(a), project(b)), scale*math.dist(a, b))
        default, _ = twin.twin_projection(600, 500, -3, 3, -2, 2)
        zero, _ = twin.twin_projection(600, 500, -3, 3, -2, 2, rotation_deg=0)
        self.assertEqual(default(a), zero(a))

    def test_all_twenty_five_hexagons_fit_after_rotation(self):
        pads = twin.twin_pad_window()
        xs, ys = zip(*pads.values())
        bounds = min(xs)-.6, max(xs)+.6, min(ys)-.6, max(ys)+.6
        for width, height in ((340, 340), (600, 260), (1080, 620)):
            project, _ = twin.twin_projection(width, height, *bounds, rotation_deg=60)
            for cx, cy in pads.values():
                for i in range(6):
                    angle = math.radians(30+60*i)
                    x, y = project((cx+math.cos(angle)/math.sqrt(3), cy+math.sin(angle)/math.sqrt(3)))
                    self.assertGreaterEqual(x, 12)
                    self.assertLessEqual(x, width-12)
                    self.assertGreaterEqual(y, 76)
                    self.assertLessEqual(y, height-74)

    def test_pointer_zoom_is_fixed_at_rotated_world_point(self):
        state = dict(zoom=1., pan_x=14., pan_y=-8., active=True)
        canvas = SimpleNamespace(winfo_width=lambda: 800, winfo_height=lambda: 600)
        def project():
            return twin.twin_projection(800, 600, -3, 3, -2, 2,
                                       state["zoom"], state["pan_x"], state["pan_y"], rotation_deg=60)[0]
        point = (.5, -.7)
        x, y = project()(point)
        canvas_view_zoom(state, SimpleNamespace(widget=canvas, x=x, y=y, delta=120), lambda: None)
        self.assertAlmostEqual(project()(point)[0], x)
        self.assertAlmostEqual(project()(point)[1], y)


@unittest.skipUnless(ui.HAS_TK, "real Tk unavailable")
class RotationUITests(unittest.TestCase):
    def setUp(self):
        step_tests.StepCurveUITests.setUp(self)

    make_live_run = step_tests.StepCurveUITests.make_live_run

    def test_toggle_rotates_map_body_paths_and_candidate_without_changing_data(self):
        from motor_control.ui.gait_tab import _gait_mode_selected
        sim = GaitSimulation(self.app.gait_params)
        finish_step(sim)
        finish_step(sim)
        self.view["simulation"] = sim
        self.view["mode_var"].set("sim")
        _gait_mode_selected(self.app, "L+")
        canvas = self.view["canvas"]
        tags = ("map_hex", "sim_left", "sim_history_left", "candidate_ghost")
        original = {tag: [canvas.coords(i) for i in canvas.find_withtag(tag)] for tag in tags}
        state = (sim.supports, sim.psis, list(sim.history.points), sim.curve_time_s)
        curve = curve_payload(self.app, self.view)
        candidate = self.view["candidate"]
        params = self.app.gait_params
        self.view["rotate_map_control"].invoke()
        self.assertTrue(self.view["rotate_map_var"].get())
        for tag in tags:
            self.assertNotEqual([canvas.coords(i) for i in canvas.find_withtag(tag)], original[tag], tag)
        foot = sim.frame.pose["feet"]["left"]
        current = canvas.coords(canvas.find_withtag("sim_left")[0])
        self.assertEqual(current[:2], list(self.view["project"](foot["center"])))
        report = candidate["report"]
        d = twin.lattice_d_mm(sim.params)
        target = tuple(v/d for v in report.samples[-1].center)
        ghost = canvas.coords(canvas.find_withtag("candidate_ghost")[0])
        self.assertEqual(ghost[:2], list(self.view["project"](target)))
        self.assertEqual((sim.supports, sim.psis, list(sim.history.points), sim.curve_time_s), state)
        self.assertEqual(curve_payload(self.app, self.view), curve)
        self.assertIs(self.app.gait_params, params)
        self.assertIs(self.view["candidate"], candidate)
        self.view["rotate_map_control"].invoke()
        for tag in tags:
            self.assertEqual([canvas.coords(i) for i in canvas.find_withtag(tag)], original[tag], tag)
        self.assertEqual(len(canvas.find_withtag("map_hex")), 25)
        self.app._save_gait_params.assert_not_called()
        self.app._send_and_read.assert_not_called()

    def test_rotated_zoomed_map_click_still_selects_same_world_pad(self):
        self.view["rotate_map_control"].invoke()
        self.app.gait_widgets["twin_view"].update(zoom=1.5, pan_x=14, pan_y=-8)
        twin.draw_twin(self.app)
        params = self.app.gait_params
        self.view["select_side"].set("left")
        canvas = self.view["canvas"]
        x, y = self.view["project"](map_pads()["C"])
        canvas.event_generate("<Button-1>", x=round(x), y=round(y))
        self.root.update()
        self.assertEqual(self.view["start_left"].get(), map_label("C"))
        self.assertIs(self.app.gait_params, params)
        self.app._save_gait_params.assert_not_called()
        self.app._send_and_read.assert_not_called()

    def test_large_window_shares_rotation_but_keeps_own_zoom_and_pan(self):
        from motor_control.ui.gait_twin_window import open_twin_window
        self.view["rotate_map_control"].invoke()
        open_twin_window(self.app)
        detail = self.view["large_window"]
        detail["window"].attributes("-alpha", 0)
        self.root.update()
        self.assertIs(detail["rotate_map_var"], self.view["rotate_map_var"])
        self.assertTrue(detail["rotate_map_var"].get())
        detail["view_state"].update(zoom=1.4, pan_x=12, pan_y=-8)
        twin.draw_twin(self.app)
        self.view["select_side"].set("right")
        x, y = detail["project"](map_pads()["C"])
        twin._map_click(self.app, SimpleNamespace(x=x, y=y), detail)
        self.assertEqual(self.view["start_right"].get(), map_label("C"))
        for _ in range(4):
            detail["rotate_map_control"].invoke()
            for view in (self.view, detail):
                self.assertEqual(len(view["canvas"].find_withtag("map_hex")), 25)
                self.assertEqual(len(view["canvas"].find_withtag("map_orientation")), 1)
        self.assertEqual(self.app.gait_widgets["twin_view"]["zoom"], 1.)
        self.assertEqual(detail["view_state"]["zoom"], 1.4)
        reset_canvas_view(detail["view_state"])
        twin.draw_twin(self.app)
        self.assertTrue(detail["rotate_map_var"].get())  # Fit doesn't change orientation.
        detail["window"].tk.call(detail["window"].protocol("WM_DELETE_WINDOW"))
        self.assertTrue(self.view["rotate_map_var"].get())

    def test_rotation_during_live_run_never_changes_pose_or_commands(self):
        self.make_live_run()
        snapshot = self.view["snapshot"]
        pose = snapshot["pose"]
        points = list(self.view["history"].points)
        curve = curve_payload(self.app, self.view)
        params = self.app.gait_params
        for _ in range(4):
            self.view["rotate_map_control"].invoke()
            self.assertIs(self.view["snapshot"], snapshot)
            self.assertIs(self.view["snapshot"]["pose"], pose)
            self.assertEqual(list(self.view["history"].points), points)
            self.assertEqual(curve_payload(self.app, self.view), curve)
            self.assertIs(self.app.gait_params, params)
        self.app._send_and_read.assert_not_called()
        self.app._save_gait_params.assert_not_called()


if __name__ == "__main__":
    unittest.main()
