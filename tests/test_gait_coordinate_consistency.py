"""Independent mirrored-world, automatic-mode and command lifecycle regressions."""
import math
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

from motor_control.gait_coordinates import action_key, action_values, rod_side_for_pair
from motor_control.gait_avoidance import avoidance_path, HIGH
from motor_control.gait_map import map_pads
from motor_control.gait_planner import (default_gait_params, effective_geometry, gait_pads,
                                       point_segment_distance, RIGHT_SWING)
from motor_control.gait_simulation import GaitSimulation
from motor_control.gait_x42s_wait import wait_moves
from motor_control.x42s_motion import MoveExpectation
from motor_control.gait_executor import GaitExecutorError
from motor_control.ui import gait_twin as twin
from motor_control.ui.gait_tab import PLACEMENT_LABELS, _gait_mode_selected
from test_gait_realtime_fixes import FakeClock
from test_x42s_pc import _x42s_gait_app


class CoordinateConsistencyTests(unittest.TestCase):
    def test_actual_swept_rod_selects_mode_on_every_map_route(self):
        p = default_gait_params()
        g = effective_geometry(p)
        checked = 0
        for left, a in map_pads().items():
            for right, b in map_pads().items():
                if not math.isclose(math.dist(a, b), 1):
                    continue
                sim = GaitSimulation(p, (left, right))
                for key in ("L+", "L-", "R+", "R-"):
                    side, arc = action_values(key)
                    try:
                        route = sim.route(side, arc)
                    except ValueError:
                        continue
                    pads = gait_pads(p, route[2])
                    pivot = pads[route[2]].center
                    # Geometric oracle: midpoint beam covers a high rod iff HIGH.
                    # No reuse of midpoint phase/mod120 classification arithmetic.
                    ang = math.radians(route[3]-arc/2)
                    end = (pivot[0]+g.d_mm*math.cos(ang), pivot[1]-g.d_mm*math.sin(ang))
                    distance = min(point_segment_distance(rod, pivot, end)
                                   for pad in pads.values() for rod in pad.high_nodes(g.arm_length_mm))
                    self.assertEqual(avoidance_path(route[3], arc).modality == HIGH, distance < 1e-6)
                    checked += 1
        self.assertGreater(checked, 300)

    def test_ui_rods_and_tips_are_identical_to_planner_world(self):
        p = default_gait_params()
        g = effective_geometry(p)
        pads = gait_pads(p)
        for name, j, x, y in twin.high_rods_mm(p):
            if name in pads:
                self.assertLess(math.dist((x, y), pads[name].high_nodes(g.arm_length_mm)[j]), 1e-9)
        for name in ("A", "B", "C"):
            center = tuple(v/g.d_mm for v in pads[name].center)
            for psi in (-90, 30, 150, 270):
                tips = twin.foot_tips_mm(center, psi, p)
                for k, tip in enumerate(tips):
                    self.assertLess(math.dist(tip, pads[name].node_center(psi, k, g.arm_length_mm)), 1e-9)

    def test_placement_handedness_is_geometry_not_legacy_key(self):
        self.assertEqual(rod_side_for_pair(("A", "B")), "左")
        self.assertEqual(rod_side_for_pair(("B", "A")), "右")
        self.assertIn("左侧", PLACEMENT_LABELS["red_right"])
        self.assertIn("右侧", PLACEMENT_LABELS["red_left"])

    def test_action_roundtrip_and_busy_revert(self):
        class Var:
            def __init__(self, value): self.value = value
            def get(self): return self.value
            def set(self, value): self.value = value
        for key in ("L+", "L-", "R+", "R-"):
            side, arc = action_values(key)
            self.assertEqual(action_key(side, arc), key)
            app = SimpleNamespace(gait_widgets={}, _gait_owned={1: object()},
                                  gait_side_var=Var(side), gait_arc_var=Var(str(arc)), gait_mode_var=Var(""))
            _gait_mode_selected(app, "L-" if key == "L+" else "L+")
            self.assertEqual(app.gait_mode_var.get(), key)

    def test_default_right_route_bearing_matches_centers(self):
        pads = gait_pads(default_gait_params())
        start, _target, pivot, angle = RIGHT_SWING
        vector = tuple(x-y for x, y in zip(pads[start].center, pads[pivot].center))
        measured = math.degrees(math.atan2(-vector[1], vector[0])) % 360
        self.assertAlmostEqual(measured, angle)

    def test_thin_fixture_dimensions_are_visible_not_silently_changed(self):
        p = default_gait_params()
        p = replace(p, geometry=replace(p.geometry, arm_radius_mm=.2, node_radius_mm=.2, safety_margin_mm=.1))
        report = GaitSimulation(p).preview("left", -60)
        self.assertIn("临时调试值", report.message)
        self.assertEqual(p.geometry.arm_radius_mm, .2)


class GroupWaitTests(unittest.TestCase):
    def fixture(self):
        app = _x42s_gait_app()
        token = object()
        app._gait_owned = {102: token, 104: token}
        for a in (102, 104): app._move_reservation[a] = token
        app._gait_x42s_moves = {a: MoveExpectation(token, 215., 100., origin_motor_deg=0.) for a in (102,104)}
        return app

    def test_second_axis_failure_polled_before_first_axis_completes(self):
        clock = FakeClock()
        with patch("time.monotonic", clock.monotonic), patch("time.sleep", clock.sleep):
            app = self.fixture()
            calls = []
            def reply(command, **_):
                calls.append(command)
                if command.endswith(",4"):
                    return "X42S,4,S,0,0,0,none,0,0,0,0,0,25,5,0,127"
                return "X42S,2,S,1,1,0,none,215,10,0,1,0,25,5,0,127"
            app._send_and_read = reply
            self.assertEqual(wait_moves(app, (102,104), 30), "ABORTED")
            self.assertLess(clock.now-100, 1)
            self.assertEqual(calls[:2], ["X42S,S,2", "X42S,S,4"])
            self.assertEqual(len(app._gait_x42s_moves), 2)

    def test_automatic_restore_cannot_send_uncertified_reverse_moves(self):
        app = _x42s_gait_app()
        app._send_manual_x42s = Mock(side_effect=AssertionError("unsafe MOVE"))
        with self.assertRaises(GaitExecutorError):
            app._gait_auto_restore_run({"Mr1": -1})
        app._send_manual_x42s.assert_not_called()

    def test_stop_requires_exact_ack_and_latches_failed_parking(self):
        for reply in (None, "OK,X42S,2,EN,0", "OK,X42S,2,STOP_EXTRA", "OK,X42S,4,STOP"):
            app = _x42s_gait_app()
            app._send_and_read = lambda *a, **k: reply
            app._send_x42s_stop_command(2)
            self.assertTrue(app._estop_unconfirmed)

    def test_inactive_owned_support_fault_prevents_motion_and_completion(self):
        clock = FakeClock()
        with patch("time.monotonic", clock.monotonic), patch("time.sleep", clock.sleep):
            app = self.fixture()
            token = app._gait_owned[102]
            app._gait_owned[103] = app._move_reservation[103] = token
            app._x42s_states["Mup2"] = replace(app._x42s_states["Mup2"], fault="stall")
            self.assertEqual(wait_moves(app, (102,104), 30), "ABORTED")
            self.assertEqual(app.commands, [])
            app._gait_x42s_moves.clear()
            self.assertEqual(app._gait_send_relative(102, 1, 1), "failed")
            self.assertFalse(any(",MOVE," in c for c in app.commands))


class MirroredCanvasTests(unittest.TestCase):
    def test_default_selection_and_rendered_legs_match_the_planner(self):
        from scripts.gui_smoke import isolated_gui
        with isolated_gui() as (app, errors):
            from motor_control.ui.gait_simulation import reset_simulation
            reset_simulation(app)
            self.assertEqual((app.gait_side_var.get(), float(app.gait_arc_var.get())),
                             action_values(app.gait_mode_var.get()))
            for choice in app.gait_widgets["action_choices"]:
                choice.invoke()
                self.assertEqual((app.gait_side_var.get(), float(app.gait_arc_var.get())),
                                 action_values(app.gait_mode_var.get()))
            view = app.gait_widgets["twin"]
            sim = view["simulation"]
            twin.draw_twin(app)
            canvas, project = view["canvas"], view["project"]
            for side in ("left", "right"):
                foot = sim.frame.pose["feet"][side]
                lines = canvas.find_withtag("sim_"+side)
                self.assertEqual(len(lines), 3)
                d = twin.lattice_d_mm(sim.params)
                tips = twin.foot_tips_mm(foot["center"], foot["psi_deg"], sim.params)
                for item, tip in zip(lines, tips):
                    self.assertLess(math.dist(canvas.coords(item)[-2:], project(tuple(v/d for v in tip))), 1e-6)
            self.assertEqual(errors, [])
