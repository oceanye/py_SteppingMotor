"""Two-mode legs-only sweep and real desktop execution bridge; no hardware IO."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # 裸名互导（discover/单跑都可用）
from dataclasses import replace
from unittest.mock import patch

from motor_control.gait_avoidance import TWO_MODE, LEGACY, LOW, HIGH, avoidance_path
from motor_control.gait_planner import (
    GaitParams, GaitGeometry, plan_swing_trajectory, plan_gait_stages,
    parse_gait_params, effective_geometry, point_segment_distance, gait_pads,
)
from motor_control.gait_executor import GaitExecutorError
from motor_control.gait_map import map_pads
from motor_control.gait_simulation import GaitSimulation
from motor_control.protocol import parse_line
from test_gait_twin import linked_app

MODES = (("left", 60, LOW, (180, 60)), ("right", -60, LOW, (-180, -60)),
         ("left", -60, HIGH, (60, -60)), ("right", 60, HIGH, (-60, 60)))


def params():
    # Synthetic thin-leg fixture, never a calibration or a machine recommendation.
    return GaitParams(trajectory_mode=TWO_MODE, geometry=GaitGeometry(
        d_mm=math.sqrt(3)*40, arm_length_mm=40, arm_radius_mm=.2,
        node_radius_mm=.2, safety_margin_mm=.1), rotation_limit_deg=720)


def app_fixture():
    app = linked_app()
    app.gait_params = replace(app.gait_params, trajectory_mode=TWO_MODE,
                              geometry=params().geometry)
    return app


def complete(app, side, arc):
    run, report = app._gait_begin_run(side, arc_deg=arc)
    with patch("motor_control.desktop_app.messagebox.askokcancel", return_value=True):
        while run.current_stage() is not None:
            if run.current_stage().is_motion_stage:
                if not run.execute_current_stage():
                    raise AssertionError(run.last_error)
            else:
                app._gait_stage_confirmed()
    return run, report


class TwoModeGeometryTests(unittest.TestCase):
    def test_all_map_routes_classify_by_beam_swept_sector_not_button(self):
        # Independent geometric oracle: a high rod's center is inside the
        # signed sector swept by the beam segment. Do not reuse the classifier's
        # midpoint/mod120 formula to construct the expected result.
        p = params()
        g = effective_geometry(p)
        centers = map_pads()
        checked = 0
        modes_by_action = {}
        for left, a in centers.items():
            for right, b in centers.items():
                if not math.isclose(math.dist(a, b), 1.0):
                    continue
                sim = GaitSimulation(p, (left, right))
                for side in ("left", "right"):
                    for arc in (60, -60):
                        try:
                            route = sim.route(side, arc)
                        except ValueError:  # A map-edge action has no landing.
                            continue
                        pads = gait_pads(p, route[2])
                        pivot = pads[route[2]].center
                        crosses = False
                        for pad in pads.values():
                            for rod in pad.high_nodes(g.arm_length_mm):
                                radius = math.dist(pivot, rod)
                                if not 0 < radius < g.d_mm:
                                    continue
                                angle = math.degrees(math.atan2(
                                    rod[1]-pivot[1], rod[0]-pivot[0]))
                                swept = ((route[3]-angle) if arc > 0 else
                                         (angle-route[3])) % 360
                                crosses |= 1e-6 < swept < abs(arc)-1e-6
                        actual = avoidance_path(route[3], arc).modality
                        with self.subTest(supports=sim.supports, side=side, arc=arc):
                            self.assertEqual(actual, HIGH if crosses else LOW)
                        modes_by_action.setdefault((side, arc), set()).add(actual)
                        checked += 1
        self.assertGreater(checked, 300)
        self.assertEqual(len(modes_by_action), 4)
        self.assertTrue(all(modes == {LOW, HIGH} for modes in modes_by_action.values()))

    def test_mirrored_initial_placements_swap_all_four_modalities(self):
        for side, arc, family, _ in MODES:
            for placement in ("red_right", "red_left"):
                sim = GaitSimulation(replace(params(), initial_placement=placement,
                                             beam_reference_deg=0 if placement == "red_left" else 180))
                report = sim.preview(side, arc)
                expected = family if placement == "red_right" else (HIGH if family == LOW else LOW)
                with self.subTest(side=side, arc=arc, placement=placement):
                    self.assertEqual(report.modality, expected)
                    self.assertIn("自动 HIGH" if expected == HIGH else "自动 LOW", report.message)

    def test_continuous_simulation_reclassifies_and_return_keeps_route_modality(self):
        sim = GaitSimulation(params())
        first = sim.begin("left", 60)
        self.assertEqual(first.modality, LOW)
        while sim.active:
            sim.advance()
        # Same button is now HIGH; reversing the just-completed path stays LOW.
        self.assertEqual(sim.preview("left", 60).modality, HIGH)
        self.assertEqual(sim.preview("left", -60).modality, LOW)
        self.assertEqual(sim.preview("right", 60).modality, LOW)
        self.assertEqual(sim.preview("right", -60).modality, HIGH)
        second = sim.begin("right", -60)
        self.assertEqual(second.modality, HIGH)
        while sim.active:
            sim.advance()
        self.assertEqual(sim.preview("right", 60).modality, HIGH)
        self.assertEqual(sim.preview("left", 60).modality, LOW)

    def test_four_initial_modes_and_current_stance_classification(self):
        for side, arc, family, end in MODES:
            bearing = 180 if side == "left" else 360
            path = avoidance_path(bearing, arc)
            self.assertEqual(path.modality, family)
            self.assertEqual(path.angles(1)[1:], end)
        # After left clockwise: right clockwise is LOW, unlike initial R+ HIGH.
        self.assertEqual(avoidance_path(300, 60).modality, LOW)
        self.assertEqual(avoidance_path(120, 60).modality, HIGH)
        for bearing, arc in ((181, 60), (180, 120), (180, 0), (math.nan, 60)):
            with self.assertRaises(ValueError):
                avoidance_path(bearing, arc)

    def test_net_clearance_all_four_and_endpoint_landing(self):
        app = app_fixture()
        for side, arc, family, end in MODES:
            route = app._gait_landing_route(side, params(), arc_deg=arc)
            report = plan_swing_trajectory(params(), side=side, route=route, arc_deg=arc)
            self.assertTrue(report.feasible, report.message)
            self.assertEqual(report.modality, family)
            self.assertAlmostEqual((report.samples[-1].psi_deg-30) % 120, 0)
            self.assertTrue(report.worst_rod)
            self.assertIn(report.worst_leg, (1, 2, 3))

    def test_executed_polyline_is_the_preview_and_collision_path(self):
        app = app_fixture()
        for side, arc, _, _ in MODES:
            route = app._gait_landing_route(side, params(), arc_deg=arc)
            path = avoidance_path(route[3], arc)
            report = plan_swing_trajectory(params(), side=side, route=route, arc_deg=arc)
            stage = next(s for s in plan_gait_stages(params(), side=side, route=route,
                         arc_deg=arc, swing_psi_start_deg=30) if s.stage_id == "S4")
            self.assertEqual(len(stage.move_groups), len(path.knots)-1)
            self.assertAlmostEqual(sum(stage.group_durations), stage.duration_s)
            for sample in report.samples:
                spin, q, phi = path.angles(sample.s)
                self.assertAlmostEqual(sample.psi_deg-30, spin)
                self.assertAlmostEqual(sample.swing_q_delta_deg, q)
                self.assertAlmostEqual(sample.phi_deg, phi)
            for i, knot in enumerate(path.knots[1:]):
                self.assertEqual(stage.sync_endpoints[i], (knot[1]+knot[2], knot[1]))

    def test_no_lift_or_legacy_gain_can_waive_leg_sweep(self):
        app = app_fixture()
        p = params()
        route = app._gait_landing_route("left", p, arc_deg=-60)
        reference = plan_swing_trajectory(p, side="left", route=route, arc_deg=-60)
        changed = plan_swing_trajectory(replace(p, lift_mm=9999, phase_gain=10),
                                       side="left", route=route, arc_deg=-60)
        self.assertEqual(reference.min_margin_mm, changed.min_margin_mm)
        thick = replace(p, geometry=replace(p.geometry, arm_radius_mm=8, node_radius_mm=8))
        self.assertFalse(plan_swing_trajectory(thick, side="left", route=route,
                                              arc_deg=-60).feasible)

    def test_old_configuration_is_not_silently_migrated(self):
        document = params().as_document()
        self.assertEqual(parse_gait_params(document).trajectory_mode, TWO_MODE)
        del document["trajectory_mode"]
        self.assertEqual(parse_gait_params(document).trajectory_mode, LEGACY)
        document["trajectory_mode"] = "bad"
        with self.assertRaises(ValueError):
            parse_gait_params(document)

    def test_mirrored_sweeps_match(self):
        app = app_fixture()
        for a, b in ((("left", 60), ("right", -60)), (("left", -60), ("right", 60))):
            reports = [plan_swing_trajectory(params(), side=s, arc_deg=arc,
                route=app._gait_landing_route(s, params(), arc_deg=arc)) for s, arc in (a, b)]
            for x, y in zip(reports[0].samples, reports[1].samples):
                self.assertAlmostEqual(x.margin_mm, y.margin_mm, places=7)

    def test_obstacles_follow_a_distant_pivot(self):
        pads = gait_pads(params(), "邻座(7,-5)")
        self.assertIn("邻座(7,-5)", pads)
        self.assertIn("邻座(9,-7)", pads)
        self.assertIn("邻座(5,-3)", pads)

    def test_continuous_bound_is_below_independent_mid_edge_measurements(self):
        p = params()
        route = ("A", "邻座(0,-1)", "B", 180.0)
        report = plan_swing_trajectory(p, side="left", route=route, arc_deg=-60)
        g = effective_geometry(p)
        rods = [node for pad in report.hexagons for node in pad.high_nodes(g.arm_length_mm)]
        for a, b in zip(report.samples, report.samples[1:]):
            phi, psi = (a.phi_deg+b.phi_deg)/2, (a.psi_deg+b.psi_deg)/2
            theta = math.radians(180-phi)
            center = (g.d_mm*math.cos(theta), g.d_mm*math.sin(theta))
            for arm in range(3):
                t = math.radians(psi+120*arm)
                tip = (center[0]+g.arm_length_mm*math.cos(t), center[1]+g.arm_length_mm*math.sin(t))
                net = min(point_segment_distance(node, center, tip) for node in rods)
                net -= g.arm_radius_mm+g.node_radius_mm+g.safety_margin_mm
                self.assertGreaterEqual(net+1e-8, report.min_margin_mm)


class TwoModeExecutionTests(unittest.TestCase):
    def test_execution_reclassifies_after_completed_step_like_simulation(self):
        app = app_fixture()
        sim = GaitSimulation(app.gait_params)
        for side, arc, expected in (("left", 60, LOW), ("right", -60, HIGH)):
            simulated = sim.begin(side, arc)
            run, executed = complete(app, side, arc)
            self.assertEqual(run.state, "done")
            self.assertEqual((simulated.modality, executed.modality), (expected, expected))
            self.assertEqual(simulated.route[:3], executed.route[:3])
            while sim.active:
                sim.advance()
            self.assertEqual(sim.supports, tuple(app._gait_supports))

    def test_non_neighbor_arc_refused_before_controller_access(self):
        app = app_fixture()
        with self.assertRaisesRegex(GaitExecutorError, "相邻支座"):
            app._gait_begin_run("left", arc_deg=120)
        self.assertFalse(app.commands)

    def test_four_actions_issue_real_sync_and_cumulative_counts(self):
        for side, arc, family, end in MODES:
            app = app_fixture()
            run, report = complete(app, side, arc)
            self.assertEqual(run.state, "done")
            cmds = [c for c in app.commands if c.startswith("SYNC,") and c != "SYNC,S"]
            self.assertEqual(len(cmds), 1 if family == LOW else 60)
            swing, support = (2, 3) if side == "left" else (3, 2)
            for a, delta in ((swing, end[0]), (support, end[1])):
                expected = math.copysign(app.axis_profiles[a].command_steps_from_units(abs(delta)), delta)
                self.assertEqual(app.axis_runtime[a].position_steps, expected)
            self.assertAlmostEqual(app._gait_angle_snapshot()["phi_deg"],
                                   app.axis_profiles[support].units_from_steps(app.axis_runtime[support].position_steps))
            self.assertEqual(app._gait_twin_snapshot()["state"], "estimated")
            if family == HIGH:
                wire = [int(c.split(",")[2]) for c in cmds]
                self.assertTrue(any(n > 0 for n in wire) and any(n < 0 for n in wire))

    def test_neighbor_pivot_and_reclassification_survive_next_step(self):
        app = app_fixture()
        complete(app, "left", -60)
        run, report = complete(app, "right", -60)
        self.assertIn("邻座", run.route[2])
        self.assertIsNotNone(app._gait_twin_snapshot()["pose"])

    def test_failed_clearance_blocks_before_any_move(self):
        app = app_fixture()
        app.gait_params = replace(app.gait_params, geometry=GaitGeometry())
        with self.assertRaisesRegex(GaitExecutorError, "避让未通过"):
            app._gait_begin_run("left", arc_deg=-60)
        self.assertFalse(any(c.startswith(("MOVE,", "SYNC,2,")) for c in app.commands))
        self.assertFalse(app._gait_owned)

    def test_abort_segment_stops_remaining_path(self):
        app = app_fixture()
        original = app._send_and_read
        count = 0
        def controller(command, **kwargs):
            nonlocal count
            if command.startswith("SYNC,") and command != "SYNC,S":
                count += 1
                if count == 4:
                    t = command.split(",")
                    for i in (1, 3):
                        app._on_step_aborted(int(t[i]), 0, abs(int(t[i+1])))
                    return f"OK,SYNC,{t[1]},{t[3]}"
            return original(command, **kwargs)
        app._send_and_read = controller
        run, _ = app._gait_begin_run("left", arc_deg=-60)
        with patch("motor_control.desktop_app.messagebox.askokcancel", return_value=True):
            while run.current_stage().stage_id != "S4":
                if run.current_stage().is_motion_stage:
                    self.assertTrue(run.execute_current_stage())
                else:
                    app._gait_stage_confirmed()
        self.assertFalse(run.execute_current_stage())
        self.assertEqual(count, 4)
        self.assertEqual(run.state, "aborted")
        self.assertTrue(app.stopped)

    def test_zero_axis_sync_terminal_is_counted_not_skipped(self):
        app = app_fixture()
        app._gait_begin_run("left", arc_deg=-60)
        self.assertEqual(app._gait_send_synchronized([(2, 0, .001), (3, -1, 6)], 1,
                                                    cumulative_targets=(0, -1)), "sent")
        self.assertEqual(app.axis_motion_telemetry[2].last_result, "DONE")
        self.assertEqual(app.axis_runtime[2].position_steps, 0)
        self.assertEqual(parse_line("STEP,2,DONE,0,0").requested_steps, 0)

    def test_intermediate_extrema_checked_before_lift_not_only_final_60(self):
        app = app_fixture()
        app.axis_runtime[2].max_steps = app.axis_profiles[2].command_steps_from_units(70)
        with self.assertRaisesRegex(GaitExecutorError, "超过已标定行程"):
            app._gait_begin_run("left", arc_deg=-60)
        self.assertFalse(any(c.startswith("MOVE,") for c in app.commands))

    def test_quantization_margin_can_refuse_mathematical_path(self):
        app = app_fixture()
        for axis in (2, 3):
            app.axis_profiles[axis] = replace(app.axis_profiles[axis], pulse_per_rev=200)
        p = replace(app.gait_params, calibration_fingerprint=app._gait_hardware_fingerprint())
        app.gait_params = replace(p, mr1_zero_signature=app._gait_zero_signature("Mr1", p),
                                   mr2_zero_signature=app._gait_zero_signature("Mr2", p))
        with self.assertRaisesRegex(GaitExecutorError, "误差包络"):
            app._gait_begin_run("left", arc_deg=-60)

    def test_signed_installation_and_neighbor_step_keep_world_phase(self):
        app = app_fixture()
        p = replace(app.gait_params, mr1_sign=-1, mr2_sign=-1)
        app.gait_params = replace(p, mr1_zero_signature=app._gait_zero_signature("Mr1", p),
                                   mr2_zero_signature=app._gait_zero_signature("Mr2", p))
        complete(app, "left", -60)
        self.assertLess(app.axis_runtime[2].position_steps, 0)
        self.assertGreater(app.axis_runtime[3].position_steps, 0)
        self.assertLess(app._gait_twin_snapshot()["pose"]["phi_deg"], 0)

    def test_intermediate_progress_uses_whole_s4_origin(self):
        app = app_fixture()
        app._gait_begin_run("left", arc_deg=-60)
        # First real counted segment completes immediately in fake firmware.
        app._gait_send_synchronized([(2, -1, 6), (3, -1, 6)], 1,
                                    cumulative_targets=(-1, -1))
        origin = dict(app._gait_run.rotation_start)
        def ack(command, **kwargs):
            t = command.split(",")
            return f"OK,SYNC,{t[1]},{t[3]}"
        app._send_and_read = ack
        app._gait_send_synchronized([(2, -1, 6), (3, -1, 6)], 1,
                                    cumulative_targets=(-2, -2))
        self.assertEqual(origin, app._gait_run.rotation_start)
        for axis in (2, 3):
            total = abs(app._pending_step[axis])
            app._on_step_progress(axis, total//2, total)
        snapshot = app._gait_angle_snapshot()
        self.assertLess(snapshot["phi_deg"], -1)
        self.assertGreater(snapshot["phi_deg"], -2.1)

    def test_lost_ack_invalidates_both_axes_and_refuses_next_segment(self):
        from motor_control.serial_session import RequestTimeout
        app = app_fixture()
        app._gait_begin_run("left", arc_deg=-60)
        def timeout(*args, **kwargs):
            raise RequestTimeout("offline injected ACK loss")
        app._send_and_read = timeout
        self.assertEqual(app._gait_send_synchronized([(2, -1, 6), (3, -1, 6)], 1,
                         cumulative_targets=(-1, -1)), "failed")
        self.assertFalse(app.axis_runtime[2].position_trusted)
        self.assertFalse(app.axis_runtime[3].position_trusted)
        self.assertEqual(app._gait_send_synchronized([(2, -1, 6), (3, -1, 6)], 1,
                         cumulative_targets=(-2, -2)), "failed")


if __name__ == "__main__":
    unittest.main()
