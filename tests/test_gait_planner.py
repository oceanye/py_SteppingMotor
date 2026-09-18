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
    unwrap_swing_joint_delta,
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

    def test_unmeasured_geometry_does_not_hide_vertical_collision(self):
        for side in ("left", "right"):
            report = plan_swing_trajectory(GaitParams().validated(), side=side)
            self.assertFalse(report.feasible)
            safe = plan_swing_trajectory(GaitParams(lift_mm=35.0), side=side)
            self.assertTrue(safe.feasible, safe.message)
            self.assertGreater(safe.min_margin_mm, 0.0)

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
        self.assertEqual(len(s4.move_groups), 1)
        self.assertTrue(s4.synchronized)
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
        # 右摆动 Mr2，θ=0 解绕小步 +60°
        self.assertAlmostEqual(swing_total, 180.0, places=6)
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
        # ψ 当前 120°，与 30° 基准的等效最短差按 120° 周期是 +30°
        #（120+30=150≡30 mod 120），sign=+1 → 轴 +30°
        self.assertAlmostEqual(move.delta, 30.0)

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
        # 102.5 → ψ=120°：与 30° 基准差 90°，按 120° 等效周期取 +30°
        self.assertAlmostEqual(
            swing_phase_correction_deg(params, "Mr1", 102.5), 30.0
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
                                     [HexPad("H", center)], include_hub=False)
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

    def test_nodes_under_hub_are_never_silently_excluded(self):
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
        self.assertLess(with_node, without)
        self.assertLess(with_node, 0)
        self.assertGreater(clearance_margin_mm((0.0, 0.0), 30.0, geometry,
                                              with_under_hub, lift_mm=35.0), 0)
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


class UnwrapSwingDeltaTests(unittest.TestCase):
    """解绕小步选择：Δq=60+120k 中取累计角贴窗中央的等效值。"""

    def test_neutral_theta_uses_small_positive_step(self):
        self.assertAlmostEqual(unwrap_swing_joint_delta(0.0, 180.0), 60.0)

    def test_positive_theta_pulls_back_with_negative_step(self):
        # θ=60：+60→120 或 −60→0，取 |落点| 最小 → −60
        self.assertAlmostEqual(unwrap_swing_joint_delta(60.0, 180.0), -60.0)
        # θ=120：−60→60（+60→180 但随后支撑 +60 越窗）
        self.assertAlmostEqual(unwrap_swing_joint_delta(120.0, 180.0), -60.0)

    def test_negative_theta_prefers_most_centred_landing(self):
        # θ=−90：+60→−30 比 +180→90 更贴中央 → 选 +60
        self.assertAlmostEqual(unwrap_swing_joint_delta(-90.0, 180.0), 60.0)
        # θ=−60：+60→0 最居中
        self.assertAlmostEqual(unwrap_swing_joint_delta(-60.0, 180.0), 60.0)

    def test_window_invariant_across_range(self):
        for theta10 in range(-180, 181, 10):
            theta = float(theta10)
            delta = unwrap_swing_joint_delta(theta, 180.0)
            # 摆动后与随后支撑 +60° 后都必须在 ±180° 内
            self.assertLessEqual(abs(theta + delta), 180.0 + 1e-9)
            self.assertLessEqual(abs(theta + delta + 60.0), 180.0 + 1e-9)
            # 位形等效：Δq ≡ 60 (mod 120)
            self.assertAlmostEqual((delta - 60.0) % 120.0, 0.0, places=9)

    def test_slightly_out_of_window_theta_is_pulled_back(self):
        # θ=190：一步拉回窗内（Δq=−180 → 落 10°），不拒绝
        self.assertAlmostEqual(unwrap_swing_joint_delta(190.0, 180.0), -180.0)
        self.assertAlmostEqual(unwrap_swing_joint_delta(-190.0, 180.0), 180.0)

    def test_far_out_theta_beyond_two_turns_raises(self):
        with self.assertRaises(ValueError):
            unwrap_swing_joint_delta(1000.0, 180.0)
        with self.assertRaises(ValueError):
            unwrap_swing_joint_delta(-1000.0, 180.0)

    def test_limit_validation_bounds(self):
        with self.assertRaises(ValueError):
            GaitParams(rotation_limit_deg=120.0).validated()
        with self.assertRaises(ValueError):
            GaitParams(rotation_limit_deg=721.0).validated()
        # 边界合法
        GaitParams(rotation_limit_deg=120.0001).validated()
        GaitParams(rotation_limit_deg=720.0).validated()


class UnwrapGaitSequenceTests(unittest.TestCase):
    """左右轮换连续摆动：两侧累计角始终徘徊在解绕窗口内。"""

    def _advance(self, params, side, thetas):
        psi = LOW_NODE_PHASE_DEG + thetas[side]
        stages = plan_gait_stages(
            params, side=side, swing_psi_start_deg=psi)
        roles = ("Mr1", "Mr2") if side == "left" else ("Mr2", "Mr1")
        for stage in stages:
            for group in stage.move_groups:
                for move in group:
                    owner = "left" if move.role in ("Mr1", "Mup1") else "right"
                    thetas[owner] += (
                        move.delta if move.role.startswith("Mr") else 0.0)
        return thetas

    def test_planning_never_shortcuts_angular_path_to_fit_cable_window(self):
        params = GaitParams(swing_segments=3).validated()
        for theta in (-180, -60, 0, 60, 180):
            stages = plan_gait_stages(params, side="left", swing_psi_start_deg=30+theta)
            s4 = next(s for s in stages if s.stage_id == "S4")
            self.assertAlmostEqual(s4.move_groups[0][0].delta, 180)
            self.assertAlmostEqual(s4.move_groups[0][1].delta, 60)

    def test_dry_run_trajectory_follows_actual_joint_delta(self):
        params = GaitParams().validated()
        with self.assertRaises(ValueError):
            plan_swing_trajectory(params, side="left", swing_joint_delta_deg=60.0)
        # Δq=180 → Δψ=120：ψ 从 30 走到 150
        report = plan_swing_trajectory(params, side="left")
        self.assertAlmostEqual(report.samples[0].psi_deg, 30.0)
        self.assertAlmostEqual(report.samples[-1].psi_deg, 150.0)
        with self.assertRaises(ValueError):
            plan_swing_trajectory(params, side="left", swing_joint_delta_deg=-60.0)


if __name__ == "__main__":
    unittest.main()
