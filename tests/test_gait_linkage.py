"""Offline end-to-end: planner -> executor -> real GUI pulse bookkeeping.

No serial port, Tk window, network server or physical motor is opened.
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # 裸名互导（discover/单跑都可用）
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

from test_desktop_axis_integration import _headless_app
from motor_control import AxisProfile, BindingSet, MODE_ROTARY
from motor_control.desktop_app import StepperGUI
from motor_control.gait_executor import GaitExecutorError
from motor_control.gait_planner import (
    GaitGeometry, GaitParams, RoleMove, angular_targets, default_hex_pads,
    plan_gait_stages, plan_swing_trajectory, smoothstep5,
    LEFT_SWING, RIGHT_SWING,
)
from motor_control.protocol import build_sync_command, parse_line, reply_matcher_for
from motor_control.serial_session import RequestTimeout


def mechanism(limit=720.0):
    app = _headless_app()
    app.control_bindings = BindingSet.suggested()
    for a in (2, 3):
        app.axis_profiles[a] = AxisProfile(mode=MODE_ROTARY, pulse_per_rev=1600, gear_ratio=1)
    app._gait_owned = {}
    app._gait_run = None
    app._gait_supports, app._gait_beta_deg = ("A", "B"), 180.0
    app._gait_needs_recovery = False
    app._closing = False
    app._is_serial_connected = lambda: True
    app._control_worker_cancelled = lambda: False
    app._post_ui = lambda action: action()
    app._refresh_gait_ui = lambda *args: None
    app._gait_stop_axes = lambda axes: app.stopped.extend(axes)
    app.stopped, app.commands = [], []
    app.gait_params = GaitParams(lift_mm=35, rotation_limit_deg=limit,
                                phase_gain=2.0,   # k=2 旧联动；k=-2 见下方专项
                                mr1_zero_deg=0, mr2_zero_deg=0,
                                calibration_confirmed=True,
                                calibration_fingerprint=app._gait_hardware_fingerprint())
    app.gait_params = replace(app.gait_params,
                             mr1_zero_signature=app._gait_zero_signature("Mr1", app.gait_params),
                             mr2_zero_signature=app._gait_zero_signature("Mr2", app.gait_params))

    def controller(command, **kwargs):
        app.commands.append(command)
        if kwargs.get("guard") is not None and not kwargs["guard"]():
            return ""
        if command == "SYNC,S":
            return "OK,SYNC,V1,6"
        t = command.split(",")
        if t[0] == "ENA":
            return f"OK,ENA,{t[1]},1"
        if t[0] == "SYNC":
            # Feed terminals BEFORE ACK to exercise pre-write pending accounting.
            for field in (1, 3):
                a = int(t[field])
                n = abs(int(t[field+1]))
                app._on_step_done(a, n, n)
            return f"OK,SYNC,{t[1]},{t[3]}"
        if t[0] == "MOVE":
            a, n = int(t[1]), int(t[2])
            app._on_step_done(a, n, n)
            return f"ACK,{a}"
        return "ERR:unexpected command"

    app._send_and_read = controller
    return app


class LandingSequenceTests(unittest.TestCase):
    def test_landing_settles_swing_leg_before_dual_descent(self):
        # 2026-09-24 应用户实机反馈重构落脚：单腿降顶部重心偏移、硬顶
        # 卡顿。S5B 先伸移位腿踩实新支座（顶部由站立腿保持高度，落地
        # 纠偏在此触发），S6 才两轴同步降回原标高；直线轴净位移为零。
        p = GaitParams(lift_mm=35, phase_gain=2.0)
        stages = plan_gait_stages(p, side="left")
        order = [s.stage_id for s in stages]
        self.assertLess(order.index("S5"), order.index("S5B"))
        self.assertLess(order.index("S5B"), order.index("S6"))
        by_id = {s.stage_id: s for s in stages}
        # 左移：移位腿 = Mup1；S2B 收起 -lift、S5B 伸出 +lift 抵消。
        self.assertEqual(
            by_id["S5B"].move_groups,
            ((RoleMove("Mup1", 35.0, p.settle_speed_mm_s),),))
        self.assertEqual(
            by_id["S6"].move_groups,
            ((RoleMove("Mup1", -35.0, p.settle_speed_mm_s),
              RoleMove("Mup2", -35.0, p.settle_speed_mm_s)),))
        net = {}
        for stage in stages:
            for group in stage.move_groups:
                for m in group:
                    if m.role.startswith("Mup"):
                        net[m.role] = net.get(m.role, 0.0) + m.delta
        self.assertEqual(net, {"Mup1": 0.0, "Mup2": 0.0})


class AngularLawTests(unittest.TestCase):
    def test_every_sample_uses_world_and_joint_reference_frames(self):
        # k=2 旧联动（handoff 文档）：Δψ=2φ，Δq摆=3φ
        for side in ("left", "right"):
            p = GaitParams(lift_mm=35, phase_gain=2.0)
            report = plan_swing_trajectory(p, side=side)
            self.assertTrue(report.feasible, report.message)
            for sample in report.samples:
                self.assertAlmostEqual(sample.psi_deg-30, 2*sample.phi_deg)
                self.assertAlmostEqual(sample.swing_q_delta_deg, 3*sample.phi_deg)
                self.assertAlmostEqual(sample.support_q_delta_deg, sample.phi_deg)
            self.assertAlmostEqual(report.samples[-1].beta_deg, 120 if side == "left" else 60)

    def test_default_negative_gain_swings_backwards(self):
        # 2026-09-21 默认 k=-2：Δψ=-2φ，Δq摆=(k+1)φ=-φ，支撑仍 +φ
        for side in ("left", "right"):
            p = GaitParams(lift_mm=35)
            report = plan_swing_trajectory(p, side=side)
            self.assertTrue(report.feasible, report.message)
            for sample in report.samples:
                self.assertAlmostEqual(sample.psi_deg-30, -2*sample.phi_deg)
                self.assertAlmostEqual(sample.swing_q_delta_deg, -1*sample.phi_deg)
                self.assertAlmostEqual(sample.support_q_delta_deg, sample.phi_deg)

    def test_linkage_does_not_depend_on_mm_geometry_or_lift(self):
        for d, lift, segments in ((80, 20, 1), (220, 35, 12), (500, 80, 200)):
            p = GaitParams(geometry=GaitGeometry(d_mm=d), lift_mm=lift,
                           swing_segments=segments, phase_gain=2.0)
            s4 = next(s for s in plan_gait_stages(p, side="left") if s.stage_id == "S4")
            self.assertEqual([m.delta for m in s4.move_groups[0]], [180, 60])
            self.assertEqual(angular_targets(p, 30), (60, 90, 30))
            self.assertAlmostEqual(s4.duration_s, 18.75)

    def test_tightly_adjacent_hexagons_include_neighbors_and_clearance(self):
        p = GaitParams(geometry=GaitGeometry(d_mm=math.sqrt(3)*40), lift_mm=35)
        report = plan_swing_trajectory(p, side="left")
        self.assertGreater(len(report.hexagons), 3)
        self.assertTrue(report.feasible, report.message)
        # Sampling cannot waive a collision hidden between endpoints.
        sparse = plan_swing_trajectory(replace(p, feasibility_samples=1), side="left")
        self.assertFalse(sparse.feasible)

    def test_low_or_unknown_structure_clearance_blocks_plan(self):
        p = GaitParams(lift_mm=35, geometry=GaitGeometry(beam_height_mm=13))
        self.assertFalse(plan_swing_trajectory(p, side="left").feasible)
        p = GaitParams(lift_mm=35, geometry=GaitGeometry(body_drop_mm=30))
        self.assertFalse(plan_swing_trajectory(p, side="left").feasible)

    def test_gain_is_calibratable_but_must_land_on_low_nodes(self):
        for invalid in (0, 1, 3, -1, -3, float("nan")):
            with self.assertRaises(ValueError):
                GaitParams(phase_gain=invalid).validated()
        self.assertEqual(angular_targets(GaitParams(phase_gain=4), 60), (240, 300, 60))
        self.assertEqual(angular_targets(GaitParams(phase_gain=-2), 60), (-120, -60, 60))


class PhysicalExecutionBridgeTests(unittest.TestCase):
    def complete(self, app, side):
        run, report = app._gait_begin_run(side)
        self.assertTrue(report.feasible)
        with patch("motor_control.desktop_app.messagebox.askokcancel", return_value=True):
            while run.current_stage() is not None:
                if run.current_stage().is_motion_stage:
                    self.assertTrue(run.execute_current_stage())
                else:
                    app._gait_stage_confirmed()
        self.assertEqual(run.state, "done")
        self.assertEqual(app._gait_owned, {})
        return run

    def test_abort_after_completed_run_does_not_require_recovery(self):
        # 2026-09-24 走完一步(done)后随手点【中止】是无效操作：站位/
        # 横梁角已更新、四轴已释放，不得触发"须人工重建基准"的惩罚，
        # 下一步换位可直接开始（用户实机连续行走的正确姿势）。
        app = mechanism()
        self.complete(app, "left")
        self.assertEqual(app._gait_run.state, "done")
        app._gait_abort_run()
        self.assertFalse(app._gait_needs_recovery)
        self.assertEqual(app._gait_supports, ("C", "B"))
        run, _ = app._gait_begin_run("right")
        self.assertIsNotNone(run)

    def test_mirror_placement_completes_left_swing_on_high_modality(self):
        # 红杆左摆法(左足B右足A、β₀0°)下"左顺移"恰是红杆右摆法"右顺移"
        # 的镜像：落点同为邻座(0,-1) 绕A、模态 HIGH；完整执行链路走通、
        # 账本随之更新。（2026-09-28 校正：左B右A=红杆在左手边。）
        from motor_control.gait_avoidance import TWO_MODE, HIGH
        app = mechanism()
        app.gait_params = replace(
            app.gait_params, trajectory_mode=TWO_MODE,
            geometry=GaitGeometry(d_mm=math.sqrt(3)*40, arm_length_mm=40,
                                  arm_radius_mm=.2, node_radius_mm=.2,
                                  safety_margin_mm=.1),
            initial_placement="red_left", beam_reference_deg=0.0,
            calibration_fingerprint=app._gait_hardware_fingerprint())
        app.gait_params = replace(app.gait_params,
                                  mr1_zero_signature=app._gait_zero_signature("Mr1", app.gait_params),
                                  mr2_zero_signature=app._gait_zero_signature("Mr2", app.gait_params))
        app._gait_supports, app._gait_beta_deg = ("B", "A"), 0.0
        route = app._gait_landing_route("left", app.gait_params)
        self.assertEqual(route, ("B", "邻座(0,-1)", "A", 0.0))
        report = plan_swing_trajectory(app.gait_params, side="left", route=route)
        self.assertEqual(report.modality, HIGH)
        self.assertTrue(report.feasible, report.message)
        self.complete(app, "left")
        self.assertEqual(app._gait_supports, ("邻座(0,-1)", "A"))
        self.assertAlmostEqual(app._gait_beta_deg, -60.075, places=4)

    def test_real_bridge_completes_left_then_right_and_updates_world_beta(self):
        app = mechanism()
        self.complete(app, "left")
        self.assertEqual(app._gait_supports, ("C", "B"))
        self.assertAlmostEqual(app._gait_beta_deg, 119.925)
        self.complete(app, "right")
        self.assertEqual(app._gait_supports, ("C", "A"))
        self.assertAlmostEqual(app._gait_beta_deg, 59.85)
        sync = [c for c in app.commands if c.startswith("SYNC,") and c != "SYNC,S"]
        self.assertEqual(len(sync), 2)
        self.assertFalse(any(c.startswith("MOVE,2,") or c.startswith("MOVE,3,") for c in app.commands))
        for a in (0, 1):
            self.assertEqual(app.axis_runtime[a].position_steps, 0)
        self.assertAlmostEqual(app._gait_angle_snapshot()["psi_delta_deg"], 119.925)

    def test_reverse_side_at_initial_stance_lands_on_adjacent_pad(self):
        # 2026-09-21 按用户要求放开区域限制：初始站位 (A,B) 的右侧换位
        # 顺向 60° 几何落点是邻座(0,-1)（而非 C），照常建立执行器。
        app = mechanism()
        run, report = app._gait_begin_run("right")
        start, target, pivot, bearing = run.route
        self.assertEqual((start, pivot), ("B", "A"))
        self.assertEqual(target, "邻座(0,-1)")
        self.assertTrue(report.feasible, report.message)
        self.assertTrue(any("邻座(0,-1)" in c for c in app._logs))

    def test_begin_run_honors_reverse_direction_selection(self):
        # 2026-09-22 实机移动与预览的四种换位方式一致：方向选择逆向时
        # 路由落到镜像邻座、S4 公转/自转与顺向全部相反。
        app = mechanism()
        app.gait_arc_var = SimpleNamespace(get=lambda: "-60.0")
        run, report = app._gait_begin_run("left")
        self.assertEqual(run.route, ("A", "邻座(0,-1)", "B", 180.0))
        self.assertEqual(report.route[1], "邻座(0,-1)")
        self.assertTrue(report.feasible, report.message)
        s4 = next(s for s in run.stages if s.stage_id == "S4")
        _spin, swing_total, support_total = angular_targets(app.gait_params, -60.0)
        self.assertEqual(
            [m.delta for m in s4.move_groups[0]],
            [app.gait_params.mr1_sign * swing_total,
             app.gait_params.mr2_sign * support_total])
        self.assertLess(app.gait_params.mr2_sign * support_total, 0.0)
        fwd_s4 = next(s for s in plan_gait_stages(
            app.gait_params, side="left", swing_psi_start_deg=30.0)
            if s.stage_id == "S4")
        self.assertAlmostEqual(fwd_s4.duration_s, s4.duration_s)
        self.assertTrue(any("逆向" in line for line in app._logs))

    def test_invalid_direction_selection_blocks_begin_run(self):
        app = mechanism()
        app.gait_arc_var = SimpleNamespace(get=lambda: "abc")
        with self.assertRaisesRegex(GaitExecutorError, "方向"):
            app._gait_begin_run("left")

    def test_begin_run_explicit_arc_overrides_ui_selection(self):
        # 2026-09-22 四个执行按钮各携带方向：显式传入优先于界面选择，
        # "看顺向、走逆向"式误操作不可能发生。
        app = mechanism()
        app.gait_arc_var = SimpleNamespace(get=lambda: "60.0")
        run, report = app._gait_begin_run("left", arc_deg=-60.0)
        self.assertEqual(run.arc_deg, -60.0)
        self.assertEqual(run.route, ("A", "邻座(0,-1)", "B", 180.0))
        self.assertEqual(report.route[1], "邻座(0,-1)")

    def test_gait_mode_radio_syncs_side_arc_and_preruns(self):
        # 四个换位方式 radio（左顺/左逆/右顺/右逆）切换时同步
        # side/arc 变量并静默重跑预览。
        from motor_control.ui.gait_tab import _gait_mode_selected

        class _Var:
            def __init__(self):
                self.value = None

            def set(self, v):
                self.value = v

            def get(self):
                return self.value

        calls = []
        app = SimpleNamespace(
            gait_side_var=_Var(), gait_arc_var=_Var(),
            _gait_run_dry_run=lambda interactive=True: calls.append(interactive),
        )
        for value, side, arc in (("L+", "left", "60.0"), ("L-", "left", "-60.0"),
                                 ("R+", "right", "60.0"), ("R-", "right", "-60.0")):
            _gait_mode_selected(app, value)
            self.assertEqual((app.gait_side_var.value, app.gait_arc_var.value),
                             (side, arc))
        self.assertEqual(calls, [False] * 4)

    def test_play_preview_reruns_dry_run_when_selection_changed(self):
        # 2026-09-22 修复：切换左/右或顺/逆后直接点【▶ 模拟动作】必须
        # 按当前选择重新干跑，不能重播旧报告（interactive=False 静默）。
        calls = []

        class _Var:
            def __init__(self, value):
                self._value = value

            def get(self):
                return self._value

        app = SimpleNamespace(
            gait_widgets={},
            gait_side_var=_Var("right"),
            gait_arc_var=_Var("-60.0"),
            _gait_last_report=object(),
            _gait_last_report_key=("left", 60.0),
            _gait_run_dry_run=lambda interactive=True: calls.append(interactive),
        )
        with patch("motor_control.desktop_app.play_preview_animation") as play:
            StepperGUI._gait_play_preview_clicked(app)
        self.assertEqual(calls, [False])
        play.assert_called_once_with(app)

        calls.clear()
        app._gait_last_report_key = ("right", -60.0)
        with patch("motor_control.desktop_app.play_preview_animation") as play:
            StepperGUI._gait_play_preview_clicked(app)
        self.assertEqual(calls, [])
        play.assert_called_once_with(app)

    def test_no_pad_at_sixty_degree_landing_still_blocks(self):
        # 顺向落点真的没有支座时仍要拦截（站位/横梁角不实的保护）。
        app = mechanism()
        app._gait_beta_deg = 90.0                   # 横梁角与 A/B 站位矛盾
        with self.assertRaisesRegex(GaitExecutorError, "横梁角"):
            app._gait_begin_run("right")

    def test_negative_gain_left_then_right_unwind_cable_to_zero(self):
        # k=-2 全链路：摆动电机反向 60°。左右各换位一次后，两侧旋转轴
        # 累计角都回到零位（对比 k=2 的 +180/+60，线缆几乎不缠绕）。
        app = mechanism()
        app.gait_params = replace(app.gait_params, phase_gain=-2.0)
        self.complete(app, "left")
        self.assertEqual(app._gait_supports, ("C", "B"))
        self.complete(app, "right")
        self.assertEqual(app._gait_supports, ("C", "A"))
        for axis in (2, 3):
            units = app.axis_profiles[axis].units_from_steps(
                app.axis_runtime[axis].position_steps)
            self.assertAlmostEqual(units, 0.0, places=6)

    def test_infeasible_clearance_warns_but_no_longer_blocks(self):
        # 2026-09-18 按用户要求：碰撞/避障校验不再一票否决，
        # 改为日志+弹窗提示；执行器照常就绪。
        app = mechanism()
        app.gait_params = replace(
            app.gait_params,
            geometry=GaitGeometry(beam_height_mm=13),
            calibration_fingerprint=app._gait_hardware_fingerprint())
        app.gait_params = replace(app.gait_params,
                                  mr1_zero_signature=app._gait_zero_signature("Mr1", app.gait_params),
                                  mr2_zero_signature=app._gait_zero_signature("Mr2", app.gait_params))
        with patch("motor_control.desktop_app.messagebox.showwarning") as warn:
            run, report = app._gait_begin_run("left")
        self.assertFalse(report.feasible)
        self.assertIsNotNone(run)
        warn.assert_called_once()
        self.assertTrue(any("仅提示，不拦截" in m for m in app._logs))

    def test_legacy_firmware_rejected_before_lifting(self):
        app = mechanism()
        app._send_and_read = lambda *_args, **_kw: "ERR:unknown command"
        with self.assertRaisesRegex(GaitExecutorError, "固件"):
            app._gait_begin_run("left")
        self.assertEqual(app._gait_owned, {})

    def test_cable_limit_checked_before_any_second_lift(self):
        app = mechanism(limit=180)
        self.complete(app, "left")
        n = len(app.commands)
        with self.assertRaisesRegex(GaitExecutorError, "窗口"):
            app._gait_begin_run("right")
        self.assertFalse(any(c.startswith("MOVE,") for c in app.commands[n:]))

    def test_rebinding_invalidates_saved_calibration(self):
        app = mechanism()
        app.axis_profiles[2] = replace(app.axis_profiles[2], pulse_per_rev=3200)
        with self.assertRaisesRegex(GaitExecutorError, "标定"):
            app._gait_begin_run("left")

    def test_reconfirming_checkbox_cannot_resurrect_old_zero_after_ppr_change(self):
        app = mechanism()
        app.axis_profiles[2] = replace(app.axis_profiles[2], pulse_per_rev=3200)
        app.gait_params = replace(app.gait_params,
                                  calibration_fingerprint=app._gait_hardware_fingerprint())
        with self.assertRaisesRegex(GaitExecutorError, "记零"):
            app._gait_begin_run("left")

    def test_direction_flip_requires_new_zero_even_when_fingerprint_matches(self):
        app = mechanism()
        app.gait_params = replace(app.gait_params, mr1_sign=-1)
        with self.assertRaisesRegex(GaitExecutorError, "记零"):
            app._gait_begin_run("left")

    def test_too_slow_long_duration_rejected_before_lift(self):
        app = mechanism()
        app.gait_params = replace(app.gait_params, swing_speed_deg_s=0.3)
        with self.assertRaisesRegex(GaitExecutorError, "时间"):
            app._gait_begin_run("left")
        self.assertFalse(any(c.startswith("MOVE,") for c in app.commands))

    def test_all_four_axes_remain_reserved_between_confirmation_stages(self):
        app = mechanism()
        app._gait_begin_run("left")
        self.assertEqual(set(app._gait_owned), {0, 1, 2, 3})
        self.assertFalse(app._send_mm(3, 1, 1, 1))
        with self.assertRaises(GaitExecutorError):
            app._gait_begin_run("left")

    def test_incomplete_or_uncounted_done_cannot_advance_to_landing(self):
        for fault in ("partial", "wrong_total", "uncounted"):
            with self.subTest(fault=fault):
                app = mechanism()
                run, _ = app._gait_begin_run("left")
                run.advance_confirm(); run.advance_confirm()
                self.assertTrue(run.execute_current_stage())   # S2 两轴抬升
                self.assertTrue(run.execute_current_stage())   # S2B 收移位腿
                before_sync = len(app.commands)
                original = app._on_step_done

                def corrupt_done(axis, executed=None, total=None):
                    if axis == 2:
                        if fault == "partial":
                            executed -= 1
                        elif fault == "wrong_total":
                            total += 1
                        else:
                            executed = total = None
                    original(axis, executed, total)

                app._on_step_done = corrupt_done
                self.assertFalse(run.execute_current_stage())
                self.assertFalse(app.axis_runtime[2].position_trusted)
                self.assertEqual(set(app.stopped), {0, 1, 2, 3})
                self.assertEqual(run.state, "aborted")
                self.assertFalse(any(c.startswith("MOVE,")
                                     for c in app.commands[before_sync:]))

    def test_lost_sync_ack_invalidates_both_positions_and_cannot_report_success(self):
        app = mechanism()
        run, _ = app._gait_begin_run("left")
        run.advance_confirm(); run.advance_confirm()
        self.assertTrue(run.execute_current_stage())   # S2 两轴抬升
        self.assertTrue(run.execute_current_stage())   # S2B 收移位腿
        def lost_ack(*_args, **_kw):
            raise RequestTimeout("SYNC", 1.0)
        app._send_and_read = lost_ack
        self.assertFalse(run.execute_current_stage())
        self.assertFalse(app.axis_runtime[2].position_trusted)
        self.assertFalse(app.axis_runtime[3].position_trusted)
        self.assertEqual(set(app.stopped), {0, 1, 2, 3})


class InitialPlacementTests(unittest.TestCase):
    """2026-09-24 红杆在横梁左/右两种初始摆放（互为镜像）的账本与模态。"""

    def test_placement_derives_supports_beam_and_roundtrips(self):
        from motor_control.gait_planner import parse_gait_params
        # 2026-09-28 校正后：red_left = 左足B右足A、β₀0°（红杆在左手边）
        p = GaitParams(initial_placement="red_left",
                       beam_reference_deg=0.0).validated()
        self.assertEqual(p.initial_supports, ("B", "A"))
        self.assertEqual(p.initial_beam_deg, 0.0)
        doc = p.as_document()
        self.assertEqual(doc["initial_placement"], "red_left")
        self.assertEqual(parse_gait_params(doc).initial_placement, "red_left")
        # 旧文档缺键时按β₀查配对（0°=红杆左摆法）；摆放与β₀不一致拒绝
        legacy_doc = {k: v for k, v in doc.items() if k != "initial_placement"}
        self.assertEqual(parse_gait_params(legacy_doc).initial_placement, "red_left")
        for beam in (180.0, 90.0):
            with self.assertRaises(ValueError):
                GaitParams(initial_placement="red_left",
                           beam_reference_deg=beam).validated()
        # 原有默认摆法（左足A右足B、β₀180）现在叫 red_right，仍为默认
        self.assertEqual(GaitParams().initial_placement, "red_right")
        self.assertEqual(GaitParams().initial_supports, ("A", "B"))


class SyncProtocolTests(unittest.TestCase):
    def test_command_and_ack_match_both_axes_and_ignore_late_ena_reply(self):
        command = build_sync_command(2, 800, 3, -267, 18750000)
        matcher = reply_matcher_for(command)
        self.assertTrue(matcher(parse_line("OK,SYNC,2,3")))
        self.assertFalse(matcher(parse_line("OK,SYNC,3,2")))
        self.assertFalse(matcher(parse_line("OK,ENA,2,1")))
        self.assertFalse(reply_matcher_for("ENA,2,1")(parse_line("OK,ENA,2,0")))

    def test_zero_duplicate_remote_and_peak_rate_are_rejected(self):
        for args in ((2,0,3,0,10000), (2,800,2,267,1000000),
                     (6,800,3,267,1000000), (2,800,3,267,1000)):
            with self.assertRaises(ValueError):
                build_sync_command(*args)


class LandingRouteTests(unittest.TestCase):
    """几何落点 route：干跑预览与实机执行必须共用同一几何。"""

    def test_default_routes_recorded_in_report(self):
        # 不传 route 时按固定 A→C / B→A 示意路由干跑，报告记录在案。
        base = dict(lift_mm=35, phase_gain=2.0)
        self.assertEqual(
            plan_swing_trajectory(GaitParams(**base), side="left").route,
            LEFT_SWING)
        self.assertEqual(
            plan_swing_trajectory(GaitParams(**base), side="right").route,
            RIGHT_SWING)

    def test_custom_route_roundtrip_keeps_full_clearance_set(self):
        # 传入几何落点 route：报告原样记录；碰撞校验仍覆盖含邻座的
        # 全部支座（route 只影响视图聚焦，不影响安全校验范围）。
        params = GaitParams(lift_mm=35, phase_gain=2.0)
        route = ("B", "邻座(0,-1)", "A", 0.0)
        report = plan_swing_trajectory(params, side="right", route=route)
        self.assertEqual(report.route, route)
        self.assertTrue(any(h.name == "邻座(0,-1)" for h in report.hexagons))
        self.assertGreater(len(report.hexagons), 3)

    def test_landing_route_matches_geometry_for_both_sides(self):
        app = mechanism()
        self.assertEqual(app._gait_landing_route("left", app.gait_params),
                         ("A", "C", "B", 180.0))
        # 右侧初始站位 (A,B)：bearing=β+180=360（保持坐标系原值不取模）
        self.assertEqual(app._gait_landing_route("right", app.gait_params),
                         ("B", "邻座(0,-1)", "A", 360.0))

    def test_landing_route_returns_none_when_no_pad(self):
        # 顺向落点没有支座（站位/横梁角矛盾）：预览回退固定路由，
        # 实机执行在 begin_run 的横梁角校验/落点检查处拦截。
        app = mechanism()
        app._gait_beta_deg = 90.0
        self.assertIsNone(app._gait_landing_route("right", app.gait_params))

    def test_landing_route_reverse_direction_matches_other_pad(self):
        # 2026-09-22 顺/逆双向预览：同一站位的逆向(-60°)落点是另一
        # 个邻座——四种换位方式（左/右×顺/逆）各有几何落点。
        app = mechanism()
        self.assertEqual(
            app._gait_landing_route("left", app.gait_params, arc_deg=-60.0),
            ("A", "邻座(0,-1)", "B", 180.0))
        self.assertEqual(
            app._gait_landing_route("right", app.gait_params, arc_deg=-60.0),
            ("B", "C", "A", 360.0))

    def test_reverse_arc_trajectory_swings_the_other_way(self):
        # 逆向弧：φ 0→-60、ψ=30+k·(-60)=150（≡30 mod 120 三爪对称）、
        # 摆动中心终点落在支点 240° 方位、距离 d 的邻座上。
        params = GaitParams(lift_mm=35, phase_gain=-2.0)
        route = ("A", "邻座(0,-1)", "B", 180.0)
        report = plan_swing_trajectory(
            params, side="left", route=route, arc_deg=-60.0)
        self.assertEqual(report.side, "left")
        self.assertEqual(report.route, route)
        self.assertAlmostEqual(report.samples[0].phi_deg, 0.0)
        self.assertAlmostEqual(report.samples[-1].phi_deg, -60.0)
        self.assertAlmostEqual(report.samples[-1].psi_deg, 150.0)
        end = report.samples[-1].center
        self.assertAlmostEqual(math.degrees(math.atan2(end[1], end[0])) % 360.0,
                               240.0, places=3)
        self.assertAlmostEqual(math.hypot(end[0], end[1]),
                               params.geometry.d_mm, places=3)

    def test_arc_deg_rejects_zero_and_out_of_range(self):
        params = GaitParams(lift_mm=35, phase_gain=2.0)
        for bad in (0.0, 400.0, -400.0, float("nan")):
            with self.assertRaises(ValueError):
                plan_swing_trajectory(params, side="left", arc_deg=bad)


if __name__ == "__main__":
    unittest.main()
