import math
import unittest

from motor_control.gait_planner import (
    HIGH_NODE_PHASE_DEG,
    LOW_NODE_PHASE_DEG,
    SUPPORT_JOINT_DELTA_DEG,
    SWING_ARC_DEG,
    SWING_JOINT_DELTA_DEG,
    SWING_SPIN_DEG,
    DryRunReport,
    GaitGeometry,
    GaitParams,
    GaitStage,
    HexPad,
    RoleMove,
    arm_segment,
    clearance_margin_mm,
    default_hex_pads,
    node_under_hub,
    parse_gait_params,
    plan_gait_stages,
    plan_swing_trajectory,
    point_segment_distance,
    smoothstep5,
    swing_center,
    swing_phase_correction_deg,
)


class SmoothstepTests(unittest.TestCase):
    def test_endpoints_and_midpoint(self):
        self.assertEqual(smoothstep5(0.0), 0.0)
        self.assertEqual(smoothstep5(1.0), 1.0)
        self.assertAlmostEqual(smoothstep5(0.5), 0.5)

    def test_monotonic_with_zero_end_slopes(self):
        previous = -1.0
        for index in range(21):
            s = index / 20.0
            value = smoothstep5(s)
            self.assertGreaterEqual(value, previous)
            previous = value
        h = 1e-3
        start_slope = (smoothstep5(h) - smoothstep5(0.0)) / h
        end_slope = (smoothstep5(1.0) - smoothstep5(1.0 - h)) / h
        self.assertLess(abs(start_slope), 1e-3)
        self.assertLess(abs(end_slope), 1e-3)

    def test_input_is_clamped(self):
        self.assertEqual(smoothstep5(-0.5), 0.0)
        self.assertEqual(smoothstep5(1.5), 1.0)


class GeometryTests(unittest.TestCase):
    def test_default_pads_form_equilateral_triangle(self):
        geometry = GaitGeometry().validated()
        pads = default_hex_pads(geometry)
        ab = math.dist(pads["A"].center, pads["B"].center)
        bc = math.dist(pads["B"].center, pads["C"].center)
        ac = math.dist(pads["A"].center, pads["C"].center)
        self.assertAlmostEqual(ab, geometry.d_mm)
        self.assertAlmostEqual(bc, geometry.d_mm)
        self.assertAlmostEqual(ac, geometry.d_mm)

    def test_swing_arc_endpoints_hit_hexagon_centers(self):
        geometry = GaitGeometry().validated()
        pads = default_hex_pads(geometry)
        # 左摆动：从 A(180°) 绕 B 到 C(120°)
        start = swing_center(pads["B"], geometry.d_mm, 0.0, 180.0)
        end = swing_center(pads["B"], geometry.d_mm, SWING_ARC_DEG, 180.0)
        self.assertAlmostEqual(math.dist(start, pads["A"].center), 0.0, places=6)
        self.assertAlmostEqual(math.dist(end, pads["C"].center), 0.0, places=6)
        # 右摆动：从 B(300°) 绕 C 到 A(240°)
        r_start = swing_center(pads["C"], geometry.d_mm, 0.0, 300.0)
        r_end = swing_center(pads["C"], geometry.d_mm, SWING_ARC_DEG, 300.0)
        self.assertAlmostEqual(math.dist(r_start, pads["B"].center), 0.0, places=6)
        self.assertAlmostEqual(math.dist(r_end, pads["A"].center), 0.0, places=6)

    def test_landing_phase_matches_low_nodes(self):
        geometry = GaitGeometry().validated()
        pads = default_hex_pads(geometry)
        ring = geometry.arm_length_mm
        for pad in pads.values():
            low = pad.low_nodes(ring)
            for phase in (LOW_NODE_PHASE_DEG,
                          LOW_NODE_PHASE_DEG + SWING_SPIN_DEG):
                arms = [
                    _add_scale(pad.center, phase + 120.0 * k, ring)
                    for k in range(3)
                ]
                for arm in arms:
                    self.assertTrue(
                        any(math.dist(arm, node) < 1e-6 for node in low)
                    )

    def test_high_nodes_interleave_low_nodes(self):
        geometry = GaitGeometry().validated()
        pad = default_hex_pads(geometry)["A"]
        ring = geometry.arm_length_mm
        for low in pad.low_nodes(ring):
            for high in pad.high_nodes(ring):
                self.assertGreater(math.dist(low, high), 1.0)


def _add_scale(center, angle_deg, radius):
    rad = math.radians(angle_deg)
    return (center[0] + radius * math.cos(rad),
            center[1] + radius * math.sin(rad))


class PointSegmentTests(unittest.TestCase):
    def test_distances_against_manual_values(self):
        seg = ((0.0, 0.0), (10.0, 0.0))
        self.assertAlmostEqual(point_segment_distance((5.0, 3.0), *seg), 3.0)
        self.assertAlmostEqual(point_segment_distance((-4.0, 0.0), *seg), 4.0)
        self.assertAlmostEqual(point_segment_distance((15.0, 2.0), *seg), math.hypot(5, 2))
        self.assertAlmostEqual(point_segment_distance((3.0, 0.0), *seg), 0.0)


class TrajectoryTests(unittest.TestCase):
    def test_endpoint_angles_follow_handoff_document(self):
        report = plan_swing_trajectory(GaitParams().validated(), side="left")
        first, last = report.samples[0], report.samples[-1]
        self.assertAlmostEqual(first.phi_deg, 0.0)
        self.assertAlmostEqual(last.phi_deg, SWING_ARC_DEG)
        self.assertAlmostEqual(first.psi_deg, LOW_NODE_PHASE_DEG)
        self.assertAlmostEqual(last.psi_deg, LOW_NODE_PHASE_DEG + 120.0)
        self.assertAlmostEqual(first.beta_deg, 180.0)
        self.assertAlmostEqual(last.beta_deg, 120.0)

    def test_default_placeholder_geometry_is_feasible(self):
        for side in ("left", "right"):
            report = plan_swing_trajectory(GaitParams().validated(), side=side)
            self.assertTrue(
                report.feasible,
                f"{side}: {report.message}",
            )
            self.assertGreater(report.min_margin_mm, 0.0)

    def test_oversized_nodes_make_trajectory_infeasible(self):
        params = GaitParams(
            geometry=GaitGeometry(node_radius_mm=60.0)
        ).validated()
        report = plan_swing_trajectory(params, side="left")
        self.assertFalse(report.feasible)
        self.assertLessEqual(report.min_margin_mm, 0.0)
        self.assertIn("不可行", report.message)

    def test_mid_swing_keeps_high_node_between_arms(self):
        # 文档 §3：φ=30° 时 ψ=90°，爪臂 90/210/330，高点落在爪臂之间。
        report = plan_swing_trajectory(GaitParams().validated(), side="left")
        mid = min(report.samples, key=lambda s: abs(s.phi_deg - 30.0))
        self.assertAlmostEqual(mid.psi_deg, 90.0, delta=1.5)
        self.assertGreater(mid.margin_mm, 0.0)

    def test_hub_passovers_are_reported_honestly(self):
        # 起步/收尾段各有一个高节点从壳体正上方越过：跳过检查但必须上报。
        report = plan_swing_trajectory(GaitParams().validated(), side="left")
        self.assertEqual(report.hub_passover_nodes,
                         ("A·90°高节点", "C·210°高节点"))
        self.assertIn("垂向间隙", report.message)
        self.assertIn("A·90°高节点", report.as_summary()["message"])
        mirror = plan_swing_trajectory(GaitParams().validated(), side="right")
        self.assertEqual(mirror.hub_passover_nodes,
                         ("B·210°高节点", "A·330°高节点"))


class StagePlanTests(unittest.TestCase):
    def test_left_plan_roles_and_totals(self):
        params = GaitParams(
            geometry=GaitGeometry(d_mm=140.0), swing_segments=10,
            mr1_sign=1, mr2_sign=-1,
        ).validated()
        stages = plan_gait_stages(params, side="left", swing_psi_start_deg=30.0)
        by_id = {stage.stage_id: stage for stage in stages}
        self.assertEqual(
            [s for s in stages if s.stage_id != "S3"],
            [by_id[key] for key in ("S0", "S1", "S2", "S4", "S5", "S6", "S7")],
        )
        # 左摆动：摆动 Mr1 +180°（sign=+1），支撑 Mr2 −60°（sign=−1）
        swing_total = support_total = 0.0
        s4 = by_id["S4"]
        self.assertEqual(len(s4.move_groups), 10)
        for group in s4.move_groups:
            self.assertEqual(len(group), 2)
            for move in group:
                if move.role == "Mr1":
                    swing_total += move.delta
                else:
                    support_total += move.delta
                    self.assertEqual(move.role, "Mr2")
        self.assertAlmostEqual(swing_total, 180.0, places=6)
        self.assertAlmostEqual(support_total, -60.0, places=6)
        # 抬起/落足方向相反
        s2 = next(m for m in by_id["S2"].move_groups[0] if m.role == "Mup1")
        s6 = next(m for m in by_id["S6"].move_groups[0] if m.role == "Mup1")
        self.assertAlmostEqual(s2.delta, params.lift_mm)
        self.assertAlmostEqual(s6.delta, -params.lift_mm)
        self.assertAlmostEqual(s6.speed, params.settle_speed_mm_s)

    def test_right_plan_swaps_roles(self):
        params = GaitParams(swing_segments=4).validated()
        stages = plan_gait_stages(params, side="right", swing_psi_start_deg=30.0)
        s4 = next(stage for stage in stages if stage.stage_id == "S4")
        roles = {move.role for group in s4.move_groups for move in group}
        self.assertEqual(roles, {"Mr1", "Mr2"})
        swing_total = sum(
            move.delta for group in s4.move_groups for move in group
            if move.role == "Mr2"
        )
        self.assertAlmostEqual(swing_total, SWING_JOINT_DELTA_DEG, places=6)
        lift_roles = {
            move.role
            for stage in stages
            for group in stage.move_groups
            for move in group
            if move.role.startswith("Mup")
        }
        self.assertEqual(lift_roles, {"Mup2"})

    def test_phase_adjustment_uses_shortest_path(self):
        params = GaitParams(mr1_sign=1, mr1_zero_deg=0.0).validated()
        stages = plan_gait_stages(params, side="left", swing_psi_start_deg=120.0)
        s3 = next(stage for stage in stages if stage.stage_id == "S3")
        move = s3.move_groups[0][0]
        # ψ 当前 120°，回 30° 需 Δψ=-90°，sign=+1 → 轴 -90°
        self.assertAlmostEqual(move.delta, -90.0)

    def test_speeds_keep_swing_and_support_time_synchronised(self):
        params = GaitParams().validated()
        stages = plan_gait_stages(params, side="left", swing_psi_start_deg=30.0)
        s4 = next(stage for stage in stages if stage.stage_id == "S4")
        group = s4.move_groups[0]
        durations = {abs(move.delta) / move.speed for move in group}
        self.assertLessEqual(max(durations) - min(durations), 1e-9)

    def test_confirm_only_stages_carry_no_moves(self):
        stages = plan_gait_stages(GaitParams().validated(), side="left")
        for stage_id in ("S0", "S1", "S5", "S7"):
            stage = next(s for s in stages if s.stage_id == stage_id)
            self.assertFalse(stage.is_motion_stage)
            self.assertTrue(stage.confirm_text)


class PhaseCorrectionTests(unittest.TestCase):
    def test_correction_from_calibrated_zero(self):
        params = GaitParams(mr1_sign=1, mr1_zero_deg=12.5).validated()
        self.assertAlmostEqual(
            swing_phase_correction_deg(params, "Mr1", 12.5), 0.0
        )
        self.assertAlmostEqual(
            swing_phase_correction_deg(params, "Mr1", 102.5), -90.0
        )
        flipped = GaitParams(mr1_sign=-1, mr1_zero_deg=0.0).validated()
        self.assertAlmostEqual(
            swing_phase_correction_deg(flipped, "Mr1", -60.0), -60.0
        )

    def test_missing_zero_raises(self):
        params = GaitParams().validated()
        with self.assertRaises(ValueError):
            swing_phase_correction_deg(params, "Mr2", 0.0)


class ParamsDocumentTests(unittest.TestCase):
    def test_round_trip_preserves_all_fields(self):
        params = GaitParams(
            geometry=GaitGeometry(d_mm=155.0, arm_length_mm=45.0),
            swing_segments=9,
            lift_mm=8.5,
            swing_speed_deg_s=4.0,
            settle_speed_mm_s=0.8,
            mr2_sign=-1,
            mup1_lift_sign=-1,
            mr1_zero_deg=-13.25,
        ).validated()
        restored = parse_gait_params(params.as_document())
        self.assertEqual(restored, params)

    def test_defaults_used_for_missing_fields(self):
        restored = parse_gait_params({"schema": GaitParams.SCHEMA})
        self.assertEqual(restored, GaitParams())

    def test_invalid_documents_are_rejected(self):
        with self.assertRaises(ValueError):
            parse_gait_params(None if False else {"schema": "other-v0"})
        with self.assertRaises(ValueError):
            parse_gait_params({
                "schema": GaitParams.SCHEMA,
                "geometry": {"d_mm": -3.0},
            })
        with self.assertRaises(ValueError):
            parse_gait_params({"schema": GaitParams.SCHEMA, "mr1_sign": 2})
        with self.assertRaises(ValueError):
            parse_gait_params({"schema": GaitParams.SCHEMA, "lift_mm": 0.0})
        with self.assertRaises(ValueError):
            GaitParams(
                geometry=GaitGeometry(hub_radius_mm=50.0,
                                      arm_length_mm=40.0)
            ).validated()


class RoleMoveContractTests(unittest.TestCase):
    def test_role_move_and_stage_are_plain_data(self):
        stage = GaitStage(
            stage_id="X", title="t", confirm_text="c",
            move_groups=((RoleMove("Mr1", 15.0, 5.0),),),
        )
        self.assertTrue(stage.is_motion_stage)
        self.assertEqual(stage.move_groups[0][0].role, "Mr1")


class MarginTests(unittest.TestCase):
    def test_landing_margin_uses_chord_clearance(self):
        # 落点位形：节点相对爪臂方位 60°，垂足在臂段内 → 原始距离 R·sin60°。
        geometry = GaitGeometry(d_mm=140.0).validated()
        center = (0.0, 0.0)
        margin = clearance_margin_mm(center, LOW_NODE_PHASE_DEG, geometry,
                                     [HexPad("H", center)])
        expected = (geometry.arm_length_mm * math.sin(math.radians(60.0))
                    - geometry.arm_radius_mm - geometry.node_radius_mm
                    - geometry.safety_margin_mm)
        self.assertAlmostEqual(margin, expected, places=6)

    def test_extra_hexagons_are_checked(self):
        geometry = GaitGeometry(d_mm=140.0).validated()
        far = clearance_margin_mm((0.0, 0.0), 30.0, geometry,
                                  [HexPad("H", (0.0, 0.0))])
        blocked = HexPad("X", (30.0, 0.0))  # 高节点直接压在爪臂路径上
        near = clearance_margin_mm((0.0, 0.0), 30.0, geometry,
                                   [HexPad("H", (0.0, 0.0)), blocked])
        self.assertLess(near, far)
        self.assertLess(near, 0.0)

    def test_nodes_under_hub_are_excluded_from_arm_check(self):
        # 节点落在壳体正下方（(15,0)，距中心 ≤ hub+node 半径）时即使
        # 正对爪臂方向也不计水平碰撞——它由抬升壳体越过，属垂向间隙问题。
        geometry = GaitGeometry().validated()
        base = [HexPad("H", (0.0, 0.0))]
        with_under_hub = base + [
            HexPad("X", (55.0, 0.0), orientation_deg=90.0)  # 高节点→(15,0)
        ]
        without = clearance_margin_mm((0.0, 0.0), 30.0, geometry, base)
        with_node = clearance_margin_mm((0.0, 0.0), 30.0, geometry,
                                        with_under_hub)
        self.assertAlmostEqual(without, with_node)
        self.assertTrue(
            node_under_hub((15.0, 0.0), (0.0, 0.0), geometry)
        )
        self.assertFalse(
            node_under_hub((25.0, 0.0), (0.0, 0.0), geometry)
        )

    def test_arm_segment_spans_hub_edge_to_tip(self):
        geometry = GaitGeometry().validated()
        start, end = arm_segment((0.0, 0.0), 90.0, 0, geometry)
        self.assertAlmostEqual(start[0], 0.0)
        self.assertAlmostEqual(start[1], geometry.hub_radius_mm)
        self.assertAlmostEqual(end[1], geometry.arm_length_mm)


class ReportTests(unittest.TestCase):
    def test_summary_is_json_friendly(self):
        report = plan_swing_trajectory(GaitParams().validated(), side="left")
        summary = report.as_summary()
        self.assertIsInstance(summary["feasible"], bool)
        self.assertIsInstance(summary["min_margin_mm"], float)
        self.assertGreaterEqual(summary["sample_count"], 2)
        self.assertTrue(summary["message"])


if __name__ == "__main__":
    unittest.main()
