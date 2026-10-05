"""Real Tk layout checks using an isolated page; no controller or serial port.

Run with a Python installation containing Tk, e.g. C:\\Python313\\python.exe.
The embedded project Python can skip these checks when Tk is unavailable.
"""
import unittest
import math
import os
import sys
from dataclasses import replace
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # 裸名互导（discover/单跑都可用）

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = hasattr(tk, "Tk")
except ImportError:
    HAS_TK = False

from motor_control.gait_planner import (
    GaitGeometry,
    GaitParams,
    HexPad,
    SwingSample,
    plan_swing_trajectory,
)


class TipTrackGeometryTests(unittest.TestCase):
    """爪端轨迹折线与爪-红杆最近表面距离的纯几何（无需 Tk）。"""

    GEOMETRY = GaitGeometry(arm_length_mm=10.0, hub_radius_mm=2.0)

    def test_tip_track_points_follow_center_and_psi(self):
        from motor_control.ui.gait_tab import tip_track_points
        samples = (
            SwingSample(s=0.0, phi_deg=0.0, psi_deg=30.0, beta_deg=0.0,
                        center=(0.0, 0.0), margin_mm=None),
            SwingSample(s=1.0, phi_deg=60.0, psi_deg=150.0, beta_deg=120.0,
                        center=(5.0, 0.0), margin_mm=None),
        )
        track = tip_track_points(samples, self.GEOMETRY, 1)   # 腿2 = ψ+120°
        self.assertAlmostEqual(track[0][0], -10.0 * math.cos(math.radians(30)))
        self.assertAlmostEqual(track[0][1], 10.0 * math.sin(math.radians(30)))
        # 终点 ψ+120°=270°：(5,0)+10·dir(270°)
        self.assertAlmostEqual(track[1][0], 5.0, places=9)
        self.assertAlmostEqual(track[1][1], -10.0, places=9)

    def test_tip_rod_clearance_reports_nearest_rod_surface(self):
        from motor_control.ui.gait_tab import tip_rod_clearance, tip_track_points
        samples = (
            SwingSample(s=0.0, phi_deg=0.0, psi_deg=30.0, beta_deg=0.0,
                        center=(0.0, 0.0), margin_mm=None),
            # ψ=90° 时腿1 爪端 (0,10) 正压 B 座 90° 高杆 (0,10)：
            # 杆半径 5（Ø10 红杆）→ 表面距离 −5，必须选中这一对
            SwingSample(s=1.0, phi_deg=60.0, psi_deg=90.0, beta_deg=120.0,
                        center=(0.0, 0.0), margin_mm=None),
        )
        tracks = tuple(tip_track_points(samples, self.GEOMETRY, k)
                       for k in range(3))
        stats = tip_rod_clearance(tracks, (HexPad("B", (0.0, 0.0)),),
                                  self.GEOMETRY)
        surface, leg, index, label, tip, rod = stats
        self.assertEqual((leg, index), (0, 1))
        self.assertEqual(label, "B·90°高杆")
        self.assertAlmostEqual(surface, -5.0, places=9)
        self.assertAlmostEqual(math.dist(tip, rod), 0.0, places=9)

    def test_tip_surface_never_tighter_than_leg_capsule_margin(self):
        # 爪端点是整段腿胶囊的一端：爪-杆表面距离 ≥ 干跑净间隙+腿半径+δ
        from motor_control.gait_avoidance import TWO_MODE
        from motor_control.ui.gait_tab import tip_rod_clearance, tip_track_points
        params = GaitParams(
            trajectory_mode=TWO_MODE,
            geometry=GaitGeometry(d_mm=104.0, arm_length_mm=60.0))
        report = plan_swing_trajectory(params, side="left")
        tracks = tuple(tip_track_points(report.samples, params.geometry, k)
                       for k in range(3))
        stats = tip_rod_clearance(tracks, report.hexagons, params.geometry)
        self.assertIsNotNone(stats)
        self.assertGreaterEqual(
            stats[0],
            report.min_margin_mm + params.geometry.arm_radius_mm
            + params.geometry.safety_margin_mm - 1e-6)


class PreviewViewTransformTests(unittest.TestCase):
    """态势图投影的滚轮缩放/中键平移复合变换（无需真实画布）。

    2026-09-30 预览并入态势图后，视图变换的真源是 ui.gait_twin 的
    twin_projection（固定 5×5 包围盒 + zoom/pan 以画布中心复合）。
    """

    WIDTH, HEIGHT = 800, 600

    def setUp(self):
        self.view = {"zoom": 1.0, "pan_x": 0.0, "pan_y": 0.0}
        self.canvas_stub = SimpleNamespace(
            winfo_width=lambda: self.WIDTH,
            winfo_height=lambda: self.HEIGHT)

    def _to_canvas(self):
        from motor_control.ui.gait_twin import twin_projection
        project, _scale = twin_projection(
            self.WIDTH, self.HEIGHT, -3.0, 3.0, -3.0, 3.0,
            self.view["zoom"], self.view["pan_x"], self.view["pan_y"])
        return project

    def test_zoom_requires_activation_and_keeps_pointer_point_fixed(self):
        from motor_control.ui.common import canvas_view_zoom
        world = (0.3, -1.2)
        # 未左键选中画面：滚轮不缩放，返回 None 让页面滚动绑定继续处理
        idle = canvas_view_zoom(self.view, SimpleNamespace(
            widget=self.canvas_stub, x=50, y=50, delta=120), lambda: None)
        self.assertIsNone(idle)
        self.assertEqual(self.view["zoom"], 1.0)
        self.view["active"] = True
        mx, my = self._to_canvas()(world)
        for delta in (120, 120, -120):
            result = canvas_view_zoom(self.view, SimpleNamespace(
                widget=self.canvas_stub, x=mx, y=my, delta=delta),
                lambda: None)
            self.assertEqual(result, "break")   # 激活后阻断页面滚动
        self.assertGreater(self.view["zoom"], 1.0)
        nx, ny = self._to_canvas()(world)
        self.assertAlmostEqual(nx, mx, delta=0.2)
        self.assertAlmostEqual(ny, my, delta=0.2)

    def test_pan_drag_shifts_view_and_reset_restores(self):
        from motor_control.ui.common import (
            canvas_view_pan_move, canvas_view_pan_start, reset_canvas_view)
        world = (1.1, 0.4)
        x0, y0 = self._to_canvas()(world)
        canvas_view_pan_start(self.view, SimpleNamespace(x=100, y=100))
        canvas_view_pan_move(self.view, SimpleNamespace(x=140, y=75), lambda: None)
        x1, y1 = self._to_canvas()(world)
        self.assertAlmostEqual(x1 - x0, 40.0, places=6)
        self.assertAlmostEqual(y1 - y0, -25.0, places=6)
        reset_canvas_view(self.view)
        x2, y2 = self._to_canvas()(world)
        self.assertAlmostEqual(x2, x0, places=6)
        self.assertAlmostEqual(y2, y0, places=6)

    def test_twin_pad_window_resolves_exactly_5_by_5(self):
        from motor_control.gait_twin import pad_center
        from motor_control.ui.gait_twin import twin_pad_window
        pads = twin_pad_window()
        self.assertTrue(set("ABC") <= set(pads))
        self.assertEqual(len(pads), 25)
        xs = [x for x, _y in pads.values()]
        self.assertGreaterEqual(max(xs) - min(xs), 4.0)
        for name, xy in pads.items():
            self.assertEqual(pad_center(name), xy)


class TwinCurveGeometryTests(unittest.TestCase):
    """爪端-高杆距离的数值口径：邻域杆表 / live 6 爪 / 计划序列（无需 Tk）。"""

    # d_mm 用占位默认 220（≠√3×60）：两模态下该字段被 planner 忽略，
    # 态势图换算也必须走同一有效几何口径（2026-09-30 现场"腿很短、
    # 支点不踩绿点"的根因就是误用了 d_mm 字段）。
    PARAMS = GaitParams(
        trajectory_mode="two_mode_v1",
        geometry=GaitGeometry(d_mm=220.0, arm_length_mm=60.0,
                              node_radius_mm=5.0))

    def test_lattice_d_ignores_placeholder_d_in_two_mode(self):
        from motor_control.gait_avoidance import LEGACY
        from motor_control.ui.gait_twin import lattice_d_mm
        self.assertAlmostEqual(lattice_d_mm(self.PARAMS), math.sqrt(3) * 60.0)
        legacy = replace(self.PARAMS, trajectory_mode=LEGACY)
        self.assertEqual(lattice_d_mm(legacy), 220.0)

    def test_legs_and_rods_are_invariant_to_placeholder_d(self):
        # 同一机构（臂长 60）无论 d_mm 填 220 还是 √3×60：杆表、6 爪
        # 距离、计划序列必须逐点一致——口径只认有效几何。
        from motor_control.ui.gait_twin import (
            high_rods_mm, plan_tip_series, tip_surface_distances)
        twin = replace(self.PARAMS, geometry=replace(
            self.PARAMS.geometry, d_mm=math.sqrt(3) * 60.0))
        self.assertEqual(high_rods_mm(self.PARAMS), high_rods_mm(twin))
        pose = {"feet": {"left": {"center": (0.0, 0.0), "psi_deg": 90.0},
                         "right": {"center": (1.0, 0.0), "psi_deg": 30.0}}}
        self.assertEqual(tip_surface_distances(pose, self.PARAMS),
                         tip_surface_distances(pose, twin))
        report_a = plan_swing_trajectory(self.PARAMS, side="left")
        report_b = plan_swing_trajectory(twin, side="left")
        self.assertEqual(plan_tip_series(report_a, self.PARAMS),
                         plan_tip_series(report_b, twin))

    def test_neighborhood_extends_one_ring_beyond_map(self):
        # "最近足/曲线"的杆表不因 5×5 画面裁剪：边缘支座旁的地图外高杆
        # 同样参与最近距离计算（避障认证范围本来就是完整邻域）。
        from motor_control.gait_map import map_pads
        from motor_control.ui.gait_twin import high_rods_mm, neighborhood_pads
        pads = neighborhood_pads()
        self.assertTrue(set(map_pads()) <= set(pads))
        self.assertGreater(len(pads), 25)
        self.assertEqual(len(high_rods_mm(self.PARAMS)), 3 * len(pads))

    def test_tip_surface_distances_follow_rod_radius_convention(self):
        from motor_control.ui.gait_twin import tip_surface_distances

        def pose(psi):
            return {"feet": {"left": {"center": (0.0, 0.0), "psi_deg": psi},
                             "right": {"center": (1.0, 0.0), "psi_deg": psi}}}

        aligned = tip_surface_distances(pose(90.0), self.PARAMS)
        self.assertEqual(len(aligned), 6)
        # ψ=90°：摆动足爪端正压支座 90° 高杆轴 → 表面距离 = −杆半径
        self.assertAlmostEqual(min(aligned), -5.0, places=6)
        self.assertTrue(all(v >= -5.0 - 1e-9 for v in aligned))
        # 偏转后爪不再压杆轴：最近距离离开 −杆半径
        self.assertGreater(min(tip_surface_distances(pose(45.0), self.PARAMS)),
                           min(aligned) + 1e-6)

    def test_plan_tip_series_support_constant_and_swing_varies(self):
        from motor_control.ui.gait_twin import plan_tip_series
        report = plan_swing_trajectory(self.PARAMS, side="left")
        series = plan_tip_series(report, self.PARAMS)
        self.assertEqual(len(series), len(report.samples))
        self.assertGreaterEqual(len(series), 2)
        times = [t for t, _vals in series]
        self.assertEqual(times[0], 0.0)
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
        self.assertGreater(times[-1], 0.0)   # 时间轴映射到 S4 时长
        # 左足摆动：槽 0..2（左）变化、槽 3..5（右支撑足）为常数线
        supports = {tuple(v[3:6]) for _t, v in series}
        self.assertEqual(len(supports), 1)
        swings = {tuple(v[0:3]) for _t, v in series}
        self.assertGreater(len(swings), 1)
        # 支撑爪踩支点低节点：到任何高杆表面的距离为正
        self.assertTrue(all(v > 0.0 for v in next(iter(supports))))

    def test_set_view_mode_is_safe_without_gui(self):
        # desktop_app 的三处模式钩子（开始执行/干跑/重置）在 headless
        # 测试宿主下必须静默跳过，不能 AttributeError。
        from motor_control.desktop_app import StepperGUI
        StepperGUI._gait_set_view_mode(SimpleNamespace(), "live")
        StepperGUI._gait_set_view_mode(SimpleNamespace(gait_widgets={}), "plan")

    def test_twin_progress_refresh_throttles_and_skips_non_gait(self):
        # 脉冲帧驱动的态势图节流刷新：非步态运动不刷；步态中刷一次后
        # 0.1s 内的后续帧被节流跳过（防高频进度帧刷爆 Tk）。
        from motor_control.desktop_app import StepperGUI
        calls = []
        app = SimpleNamespace(_closing=False, _gait_owned={}, _gait_run=None,
                              _refresh_gait_ui=lambda: calls.append(1))
        StepperGUI._twin_progress_refresh(app)
        self.assertEqual(calls, [])
        app._gait_owned = {0: object()}
        StepperGUI._twin_progress_refresh(app)
        StepperGUI._twin_progress_refresh(app)
        self.assertEqual(calls, [1])


@unittest.skipUnless(HAS_TK, "real Tk unavailable")
class GaitLayoutTests(unittest.TestCase):
    def setUp(self):
        from motor_control.ui.gait_tab import build_gait_tab
        self.root = tk.Tk()
        self.root.withdraw()
        self.root.attributes("-alpha", 0)
        self.root.tk.call("tk", "scaling", 2.0 if "high_dpi" in self._testMethodName else 4/3)
        self.addCleanup(self.root.destroy)
        self.errors = []
        self.root.report_callback_exception = lambda *error: self.errors.append(error)
        self.addCleanup(lambda: self.assertFalse(self.errors, "Unhandled Tk callback exception"))
        self.app = SimpleNamespace(root=self.root, gait_params=GaitParams(),
                                   _closing=False, _gait_run=None,
                                   _gait_angle_snapshot=lambda: None,
                                   _gait_hardware_fingerprint=lambda: "test")
        self.app.log = lambda *_args, **_kwargs: None
        for name in ("_gait_record_zero_clicked", "_gait_save_params_clicked",
                     "_gait_reload_params_clicked", "_gait_reestablish_baseline",
                     "_gait_run_dry_run", "_gait_start_run_clicked",
                     "_gait_play_preview_clicked", "_gait_stage_confirmed",
                     "_gait_auto_clicked", "_gait_abort_clicked",
                     "_gait_reset_run"):
            setattr(self.app, name, lambda *args: None)
        self.page = ttk.Frame(self.root)
        self.page.pack(fill="both", expand=True)
        build_gait_tab(self.app, self.page)
        from motor_control.ui.gait_tab import stop_preview_animation
        from motor_control.ui.gait_simulation import cancel_simulation
        self.addCleanup(lambda: stop_preview_animation(self.app))
        self.addCleanup(lambda: cancel_simulation(self.app))
        self.root.deiconify()

    def test_parameter_entries_do_not_extend_past_scroll_viewport(self):
        for size in ("720x380", "1024x480", "1280x620"):
            with self.subTest(size=size):
                self.root.geometry(size)
                self.root.update()
                def walk(widget):
                    yield widget
                    for child in widget.winfo_children():
                        yield from walk(child)
                for entry in (w for w in walk(self.page) if isinstance(w, ttk.Entry)):
                    if not entry.winfo_viewable():
                        continue
                    ancestor = entry.master
                    while ancestor is not None and not isinstance(ancestor, tk.Canvas):
                        ancestor = ancestor.master
                    if ancestor is not None:
                        self.assertLessEqual(
                            entry.winfo_rootx() + entry.winfo_width(),
                            ancestor.winfo_rootx() + ancestor.winfo_width(),
                            f"entry clipped at {size}: {entry}")
                self.assertFalse(self.errors)

    def test_tabs_and_keyboard_keep_every_parameter_reachable_at_high_dpi(self):
        self.root.geometry("720x380")
        self.root.update()
        book = self.app.gait_widgets["params_book"]
        for page in self.app.gait_widgets["param_pages"].values():
            book.select(page)
            self.root.update()
            for widget in page.content.winfo_children():
                if not isinstance(widget, (ttk.Entry, ttk.Combobox, ttk.Button)):
                    continue
                # The same handler is invoked when the operator tabs to a field.
                widget.event_generate("<FocusIn>")
                self.root.update()
                for axis, dimension in (("x", "width"), ("y", "height")):
                    pos = getattr(widget, f"winfo_root{axis}")()
                    viewport_pos = getattr(page.canvas, f"winfo_root{axis}")()
                    extent = getattr(widget, f"winfo_{dimension}")()
                    viewport_extent = getattr(page.canvas, f"winfo_{dimension}")()
                    self.assertGreaterEqual(pos, viewport_pos - 1)
                    self.assertLessEqual(pos + extent, viewport_pos + viewport_extent + 1)
        # Motor-stop control is outside scrolling content.
        abort = self.app.gait_widgets["abort"]
        self.assertTrue(abort.winfo_viewable())
        self.assertLess(abort.winfo_rooty() + abort.winfo_height(),
                        self.root.winfo_rooty() + self.root.winfo_height())
        self.assertFalse(self.errors)

    def test_editing_run_angles_preserves_reference_geometry_and_zero_values(self):
        from motor_control.ui.gait_tab import collect_gait_params, load_gait_fields
        base = replace(self.app.gait_params,
                       geometry=replace(self.app.gait_params.geometry, d_mm=math.sqrt(3)*60,
                                        arm_length_mm=60, safety_margin_mm=3,
                                        arm_radius_mm=0.123456789),
                       mr1_zero_deg=12.3, mr2_zero_deg=-4.5,
                       mr1_zero_signature="left", mr2_zero_signature="right")
        self.app.gait_params = base
        load_gait_fields(self.app)
        self.app.gait_field_vars["phase_gain"].set("4")
        saved = collect_gait_params(self.app, base)
        self.assertEqual(saved.geometry, base.geometry)
        self.assertEqual(saved.phase_gain, 4)
        self.assertEqual(saved.mr1_zero_deg, 12.3)
        self.assertEqual(saved.mr2_zero_deg, -4.5)
        self.assertEqual(saved.mr1_zero_signature, "left")
        self.assertEqual(saved.mr2_zero_signature, "right")

    def test_changing_trajectory_mode_invalidates_calibration_and_cached_preview(self):
        from motor_control.gait_avoidance import TWO_MODE
        from motor_control.ui.gait_tab import collect_gait_params
        self.app._gait_last_report = object()
        self.app.gait_calibrated_var.set(True)
        self.app.gait_trajectory_var.set(TWO_MODE)
        self.assertFalse(self.app.gait_calibrated_var.get())
        self.assertIsNone(self.app._gait_last_report)
        self.assertEqual(collect_gait_params(self.app, self.app.gait_params).trajectory_mode, TWO_MODE)

    def test_modalities_are_read_only_and_legacy_strategy_is_advanced(self):
        from motor_control.gait_avoidance import TWO_MODE, LEGACY
        widgets = self.app.gait_widgets
        run = widgets["param_pages"]["run"].content
        geometry = widgets["param_pages"]["geometry"].content
        self.assertIsInstance(widgets["trajectory_status"], ttk.Label)
        self.assertIs(widgets["trajectory_status"].master, run)
        self.assertFalse(any(isinstance(w, ttk.Combobox) for w in run.winfo_children()))
        self.assertIs(widgets["trajectory_strategy"].master, geometry)
        self.assertEqual(tuple(widgets["trajectory_strategy"]["values"]), (TWO_MODE, LEGACY))
        self.app.gait_trajectory_var.set(TWO_MODE)
        self.assertIn("后台自动判定", widgets["trajectory_status"]["text"])
        self.app.gait_trajectory_var.set(LEGACY)
        self.assertIn("未启用自动避杆", widgets["trajectory_status"]["text"])

    def test_projected_edge_updates_readonly_spacing_without_rounding_model(self):
        from motor_control.gait_avoidance import TWO_MODE
        from motor_control.gait_planner import effective_geometry, parse_gait_params
        from motor_control.ui.gait_tab import collect_gait_params
        self.app.gait_trajectory_var.set(TWO_MODE)
        self.app._gait_last_report = object()
        self.app.gait_calibrated_var.set(True)
        self.app.gait_field_vars["arm_length_mm"].set("190")
        entry = self.app.gait_widgets["spacing_entry"]
        self.assertEqual(str(entry["state"]), "readonly")
        self.assertEqual(entry.get(), "329.089653")
        self.assertEqual(self.app.gait_field_vars["d_mm"].get(), "220.0")
        collected = collect_gait_params(self.app, self.app.gait_params)
        self.assertEqual(collected.geometry.arm_length_mm, 190)
        self.assertEqual(collected.geometry.d_mm, 220)  # legacy preserved
        self.assertEqual(effective_geometry(collected).d_mm, math.sqrt(3)*190)
        self.assertNotEqual(effective_geometry(collected).d_mm, float(entry.get()))
        self.assertEqual(parse_gait_params(collected.as_document()), collected)
        self.assertFalse(self.app.gait_calibrated_var.get())
        self.assertIsNone(self.app._gait_last_report)

    def test_spacing_strategy_switch_preserves_legacy_draft(self):
        from motor_control.gait_avoidance import TWO_MODE, LEGACY
        from motor_control.ui.gait_tab import collect_gait_params
        self.app.gait_field_vars["arm_length_mm"].set("190")
        self.app.gait_field_vars["d_mm"].set("350.123456789")
        entry = self.app.gait_widgets["spacing_entry"]
        self.app.gait_trajectory_var.set(TWO_MODE)
        self.assertEqual(entry.get(), "329.089653")
        self.app.gait_trajectory_var.set(LEGACY)
        self.assertEqual(str(entry["state"]), "normal")
        self.assertEqual(entry.get(), "350.123456789")
        self.assertEqual(collect_gait_params(self.app, self.app.gait_params).geometry.d_mm,
                         350.123456789)
        self.app.gait_field_vars["d_mm"].set("220")
        with self.assertRaisesRegex(ValueError, "中心距"):
            collect_gait_params(self.app, self.app.gait_params)
        self.app.gait_trajectory_var.set(TWO_MODE)
        collect_gait_params(self.app, self.app.gait_params)  # no stale-d rejection

    def test_invalid_projection_input_never_reuses_old_spacing(self):
        from motor_control.gait_avoidance import TWO_MODE, LEGACY
        from motor_control.ui.gait_tab import collect_gait_params
        self.app.gait_trajectory_var.set(TWO_MODE)
        for value in ("", "-", "0", "-1", "nan", "inf", "1.7e308"):
            with self.subTest(value=value):
                self.app.gait_field_vars["arm_length_mm"].set(value)
                self.assertEqual(self.app.gait_widgets["spacing_entry"].get(), "—")
                with self.assertRaises(ValueError):
                    collect_gait_params(self.app, self.app.gait_params)
        self.app.gait_field_vars["arm_length_mm"].set("190")
        self.app.gait_field_vars["d_mm"].set("invalid legacy draft")
        collect_gait_params(self.app, self.app.gait_params)  # ignored in two-mode
        self.assertEqual(self.app.gait_widgets["spacing_entry"].get(), "329.089653")
        self.app.gait_trajectory_var.set(LEGACY)
        with self.assertRaises(ValueError):
            collect_gait_params(self.app, self.app.gait_params)

    def test_changing_initial_placement_resets_ledger_and_zero(self):
        # 2026-09-24 初始摆放（红杆在横梁左/右）= 坐标基准：切换即账本
        # 回新原点、零位作废、标定失效；β₀ 字段联动为派生值。
        from motor_control.ui.gait_tab import PLACEMENT_LABELS
        self.app._gait_supports = ("C", "B")          # 模拟已走一步
        self.app._gait_beta_deg = 119.925
        self.app.gait_calibrated_var.set(True)
        self.app.gait_params = replace(self.app.gait_params,
                                       mr1_zero_deg=1.5, mr2_zero_deg=-0.5)
        # 2026-09-28 校正后：红杆在左侧 = 左足B·右足A、β₀0°（镜像摆法）
        self.app.gait_placement_var.set(PLACEMENT_LABELS["red_left"])
        self.assertFalse(self.app.gait_calibrated_var.get())
        self.assertEqual(self.app._gait_supports, ("B", "A"))
        self.assertAlmostEqual(self.app._gait_beta_deg, 0.0)
        self.assertIsNone(self.app.gait_params.mr1_zero_deg)
        self.assertIsNone(self.app.gait_params.mr2_zero_deg)
        self.assertEqual(self.app.gait_field_vars["beam_reference_deg"].get(), "0.0")
        self.assertEqual(self.app.gait_params.initial_placement, "red_left")
        self.assertFalse(self.errors)

    def test_pointer_click_focus_does_not_scroll_run_viewport(self):
        # 2026-09-23 现场：焦点在其他程序后点 GUI 按钮，右侧视口跳回顶部。
        # 修复：鼠标按下子树随后的 FocusIn 属于点击聚焦，不触发滚动露出；
        # 键盘 Tab 的 FocusIn 保留"滚入聚焦控件"的可达性行为。
        viewport = self.app.gait_widgets["run_viewport"]
        canvas = viewport.canvas
        self.root.geometry("720x460")
        self.root.update()
        advance = self.app.gait_widgets["advance"]
        # 2026-09-30 预览并入态势图后右列内容变矮：滚到底部时 advance
        # 已在视口内。改为滚到顶部——advance 远在下方，键盘聚焦必须下滚
        # 露出，与具体布局高度解耦。
        canvas.yview_moveto(0.0)
        before = canvas.yview()
        self.assertNotEqual(before, (0.0, 1.0), "右侧视口在此窗口尺寸下应可滚动")
        advance.event_generate("<Button-1>")    # 鼠标按下：标记点击子树
        advance.event_generate("<FocusIn>")     # 随后的点击聚焦
        self.root.update()
        self.assertEqual(canvas.yview(), before)   # 不得滚动
        viewport._pointer_click = None             # 清除标记（等同 0.5s 后）
        advance.event_generate("<FocusIn>")     # 键盘聚焦：应滚入 advance
        self.root.update()
        self.assertNotEqual(canvas.yview(), before)

    def test_wheel_over_direction_field_scrolls_without_changing_direction(self):
        page = self.app.gait_widgets["param_pages"]["calibration"]
        self.app.gait_widgets["params_book"].select(page)
        self.root.geometry("720x380")
        self.root.update()
        combo = next(w for w in page.content.winfo_children() if isinstance(w, ttk.Combobox))
        before = page.canvas.yview()
        value = combo.get()
        combo.event_generate("<MouseWheel>", delta=-120)
        self.root.update()
        self.assertEqual(combo.get(), value)
        self.assertGreater(page.canvas.yview()[0], before[0])
        self.assertFalse(self.errors)

    def test_preview_toggles_pause_resume_and_full_stop(self):
        # 2026-09-23 现场要求："停止模拟"改为暂停——画面停在当前帧，
        # 再点继续；只有轨迹变化才彻底停止回静态预览。
        from motor_control.gait_avoidance import TWO_MODE
        from motor_control.ui.gait_tab import (
            _preview_redraw, pause_preview_animation, play_preview_animation,
            resume_preview_animation, stop_preview_animation)
        params = replace(
            self.app.gait_params, trajectory_mode=TWO_MODE,
            geometry=replace(self.app.gait_params.geometry,
                             d_mm=math.sqrt(3) * 60, arm_length_mm=60))
        report = plan_swing_trajectory(params, side="left")
        self.app._gait_last_report = report
        button = self.app.gait_widgets["play_btn"]

        play_preview_animation(self.app)
        anim = self.app.gait_widgets["preview_anim"]
        self.assertTrue(anim["playing"])
        self.assertEqual(button.cget("text"), "⏸ 暂停回放")
        # 播放=想看评估模拟：态势图自动切计划层（蓝抬头）
        self.assertEqual(self.app.gait_widgets["twin"]["mode_var"].get(), "plan")

        pause_preview_animation(self.app)
        self.assertFalse(anim["playing"])
        self.assertTrue(anim["paused"])
        self.assertGreater(anim["frame"], 0)
        self.assertEqual(button.cget("text"), "▶ 继续回放")

        # 暂停中滚轮缩放：重画必须保持暂停帧（帧号不变、仍处暂停态）
        self.app.gait_widgets["twin_view"].update(active=True, zoom=1.5)
        _preview_redraw(self.app)
        self.assertTrue(anim["paused"])
        self.assertFalse(self.errors)

        paused_frame = anim["frame"]
        resume_preview_animation(self.app)
        self.assertTrue(anim["playing"])
        self.assertFalse(anim["paused"])
        self.assertGreaterEqual(anim["frame"], paused_frame)
        self.assertEqual(button.cget("text"), "⏸ 暂停回放")

        pause_preview_animation(self.app)
        stop_preview_animation(self.app)
        self.assertFalse(anim["playing"] or anim["paused"])
        self.assertEqual(button.cget("text"), "▶ 回放预览")

    def test_live_canvas_follows_pulses_and_freezes_on_disconnect(self):
        from motor_control import AxisMotionTelemetry
        from motor_control.ui.gait_twin import refresh_twin_panel
        from test_gait_twin import linked_app
        controller = linked_app()
        run, _ = controller._gait_begin_run("left")
        run.rotation_start = {2: 0, 3: 0}
        for axis, count in ((2, 800), (3, 267)):
            controller._pending_step[axis] = count
            controller.stepper_in_progress[axis] = True
            controller.axis_motion_telemetry[axis] = AxisMotionTelemetry.starting(0, count)
        self.app._gait_twin_snapshot = controller._gait_twin_snapshot
        self.root.geometry("1024x620")
        self.root.update()
        refresh_twin_panel(self.app)
        view = self.app.gait_widgets["twin"]
        canvas = view["canvas"]
        before = canvas.coords(canvas.find_withtag("live_left")[0])
        # 2026-09-29 按用户要求：孪生画面不再画虚线指令目标（运动轨迹）
        self.assertFalse(canvas.find_withtag("target_left"))
        controller._on_step_progress(2, 400, 800)
        controller._on_step_progress(3, 133, 267)
        refresh_twin_panel(self.app)
        after = canvas.coords(canvas.find_withtag("live_left")[0])
        self.assertNotEqual(before, after)
        # 2026-09-30 按用户要求：5×5 上放大后看最近足——6 爪中与高杆最近
        # 者黄圈高亮 + mm 数字（口径=爪端到杆表面）。
        self.assertTrue(canvas.find_withtag("nearest_tip"))
        self.assertIn("6爪最近", canvas.itemcget(
            canvas.find_withtag("hud_nearest")[0], "text"))
        # 实机轨迹与评估层同款：历史点带 ψ，画三爪端虚线轨迹（蓝系=左足）
        self.assertTrue(canvas.find_withtag("history_tips_left_0"))
        self.assertTrue(canvas.find_withtag("history_tips_right_0"))
        self.assertIn("90.00", view["rows"]["Mr1"][1].cget("text"))
        controller._is_serial_connected = lambda: False
        refresh_twin_panel(self.app)
        line = canvas.find_withtag("live_left")[0]
        self.assertEqual(canvas.coords(line), after)
        self.assertEqual(canvas.itemcget(line, "fill"), "#94a3b8")
        self.assertFalse(canvas.find_withtag("target_left"))
        self.assertIn("断连", view["status"].cget("text"))
        self.assertFalse(self.errors)

    def test_twin_canvas_zoom_needs_click_activation(self):
        # 2026-09-29 孪生画面与预览同一交互：左键点一下选中后滚轮缩放，
        # 未选中时滚轮不抢（返回给页面滚动），移出画面自动取消选中。
        # （event_generate 无法合成 Double 修饰，双击复位绑定仅断言存在，
        # 复位几何本身由 PreviewViewTransformTests 的 reset 用例覆盖。）
        canvas = self.app.gait_widgets["twin"]["canvas"]
        view = self.app.gait_widgets["twin_view"]
        self.root.geometry("1024x620")
        self.root.update()
        self.assertTrue(canvas.bind("<Double-Button-1>"))
        canvas.event_generate("<MouseWheel>", delta=120)
        self.root.update()
        self.assertEqual(view["zoom"], 1.0)   # 未选中：不缩放
        canvas.event_generate("<Button-1>")   # 点一下：选中（蓝框）
        self.root.update()
        self.assertTrue(view["active"])
        canvas.event_generate("<MouseWheel>", delta=120)
        self.root.update()
        self.assertGreater(view["zoom"], 1.0)
        canvas.event_generate("<Leave>")
        self.root.update()
        self.assertFalse(view["active"])
        canvas.event_generate("<MouseWheel>", delta=-120)
        self.root.update()
        self.assertGreater(view["zoom"], 1.0)   # 已取消选中：不再缩放
        self.assertFalse(self.errors)

    def test_view_mode_switches_plan_layer_live_layer_and_title(self):
        # 2026-09-30 按用户要求：态势图双数据源——评估模拟（计划层，蓝
        # 抬头）与实机脉冲（红抬头）。切计划画轨迹/支撑/最近足高亮，切实
        # 机隐藏计划层；无实机 pose 时画"起步待确认"占位而非估算位置。
        from motor_control.gait_avoidance import TWO_MODE
        from motor_control.ui.gait_twin import _draw
        params = replace(
            self.app.gait_params, trajectory_mode=TWO_MODE,
            geometry=replace(self.app.gait_params.geometry,
                             d_mm=math.sqrt(3) * 60, arm_length_mm=60))
        report = plan_swing_trajectory(params, side="left")
        self.app._gait_last_report = report
        view = self.app.gait_widgets["twin"]
        canvas, curve = view["canvas"], view["curve_canvas"]
        self.root.geometry("1024x620")
        self.root.update()

        view["mode_var"].set("plan")
        _draw(self.app, view)
        for tag in ("plan_center", "plan_track_0", "plan_track_1", "plan_track_2",
                    "plan_support", "plan_target", "nearest_tip", "hud_plan"):
            self.assertTrue(canvas.find_withtag(tag), tag)
        self.assertFalse(canvas.find_withtag("twin_placeholder"))
        self.assertEqual(view["title"].cget("text"),
                         "🔵 单步预览 · 候选轨迹（不累计/不动电机）")
        self.assertEqual(str(view["title"].cget("foreground")), "#1d4ed8")
        # 曲线图：6 条爪距曲线（左右各 3）+ 最小值包络
        for tag in ("curve_left_0", "curve_left_1", "curve_left_2",
                    "curve_right_0", "curve_right_1", "curve_right_2", "curve_min"):
            self.assertTrue(curve.find_withtag(tag), tag)

        view["mode_var"].set("live")
        _draw(self.app, view)
        for tag in ("plan_center", "plan_track_0", "nearest_tip", "hud_plan"):
            self.assertFalse(canvas.find_withtag(tag), tag)
        self.assertTrue(canvas.find_withtag("twin_placeholder"))
        self.assertFalse(curve.find_withtag("curve_left_0"))   # 无采样数据
        self.assertIn("实机脉冲", view["title"].cget("text"))

    def test_auto_button_state_follows_run_and_auto_flag(self):
        # 2026-09-29 一键执行按钮：未开始流程时禁用；自动进行中始终可点
        # （=停在当前阶段后回人工），文本切换为"停止自动"。
        from motor_control.ui.gait_tab import refresh_gait_panel
        auto = self.app.gait_widgets["auto"]
        refresh_gait_panel(self.app)
        self.assertEqual(str(auto.cget("state")), "disabled")
        self.assertIn("一键执行", auto.cget("text"))
        self.app._gait_auto_run = True
        refresh_gait_panel(self.app)
        self.assertEqual(str(auto.cget("state")), "normal")
        self.assertIn("停止自动", auto.cget("text"))
        self.app._gait_auto_run = False
        self.assertFalse(self.errors)

    def test_map_click_after_zoom_selects_pad_without_changing_physical_reference(self):
        from motor_control.ui.gait_twin import _draw
        from motor_control.gait_map import map_label, map_pads
        view = self.app.gait_widgets["twin"]
        canvas = view["canvas"]
        self.root.geometry("1024x620")
        self.root.update()
        self.assertEqual(len(canvas.find_withtag("map_hex")), 25)
        original = self.app.gait_params
        self.app.gait_widgets["twin_view"].update(zoom=1.5, pan_x=14, pan_y=-8)
        _draw(self.app, view)
        view["select_side"].set("left")
        x, y = view["project"](map_pads()['C'])
        canvas.event_generate('<Button-1>', x=round(x), y=round(y))
        self.root.update()
        self.assertEqual(view["start_left"].get(), map_label('C'))
        self.assertIs(self.app.gait_params, original)
        self.assertEqual(self.app.gait_params.initial_supports, ('A', 'B'))
        self.assertFalse(self.errors)

    def test_clear_path_preserves_live_pose_motor_ledger_and_zoom(self):
        from motor_control.ui.gait_twin import refresh_twin_panel
        from test_gait_twin import linked_app, DigitalTwinTests
        controller = linked_app()
        controller._gait_begin_run('left')
        DigitalTwinTests().send_sync_without_terminal(controller)
        self.app._gait_twin_snapshot = controller._gait_twin_snapshot
        self.root.geometry('1024x620')
        self.root.update()
        refresh_twin_panel(self.app)
        controller._on_step_progress(2, 400, 800)
        controller._on_step_progress(3, 133, 267)
        refresh_twin_panel(self.app)
        view = self.app.gait_widgets['twin']
        canvas = view['canvas']
        self.assertTrue(canvas.find_withtag('history_left'))
        self.assertEqual(str(view['apply_button'].cget('state')), 'disabled')
        before = canvas.coords(canvas.find_withtag('left_center')[0])
        ledger = [r.position_steps for r in controller.axis_runtime]
        commands = list(controller.commands)
        zoom = dict(self.app.gait_widgets['twin_view'])
        view['clear_button'].invoke()
        self.assertFalse(canvas.find_withtag('history_left'))
        self.assertFalse(view['history'].points)
        refresh_twin_panel(self.app)
        self.assertFalse(canvas.find_withtag('history_left'))
        self.assertEqual(canvas.coords(canvas.find_withtag('left_center')[0]), before)
        self.assertEqual([r.position_steps for r in controller.axis_runtime], ledger)
        self.assertEqual(controller.commands, commands)
        self.assertEqual(self.app.gait_widgets['twin_view'], zoom)
        self.assertFalse(self.errors)

    def test_map_apply_cancel_and_invalid_pair_do_not_change_controller(self):
        from unittest.mock import patch
        from motor_control.ui.gait_twin import apply_twin_start
        from motor_control.gait_map import map_label
        from test_gait_twin import linked_app
        controller = linked_app()
        self.app.state_lock = controller.state_lock
        self.app._gait_check_start_edit = controller._gait_check_start_edit
        self.app._gait_apply_start_pair = controller._gait_apply_start_pair
        view = self.app.gait_widgets['twin']
        view['start_left'].set(map_label('C'))
        view['start_right'].set(map_label('A'))
        with patch('motor_control.ui.gait_twin.messagebox.askokcancel', return_value=False):
            apply_twin_start(self.app)
        self.assertEqual(controller._gait_supports, ('A', 'B'))
        view['start_right'].set(map_label('C'))
        with patch('motor_control.ui.gait_twin.messagebox.showwarning') as warning:
            apply_twin_start(self.app)
            warning.assert_called_once()
        self.assertEqual(controller.commands, [])
        self.assertFalse(self.errors)

    def test_apply_map_pair_preserves_typed_parameters_and_refreshes_calibration(self):
        from unittest.mock import patch
        from motor_control.ui.gait_twin import apply_twin_start, refresh_twin_panel
        from motor_control.gait_map import map_label
        from test_gait_twin import linked_app
        controller = linked_app()
        saved = []
        controller.state_store = SimpleNamespace(save_gait_params=saved.append)
        self.app.state_lock = controller.state_lock
        self.app._gait_check_start_edit = controller._gait_check_start_edit
        def apply(left, right, **kwargs):
            controller._gait_apply_start_pair(left, right, **kwargs)
            self.app.gait_params = controller.gait_params
        self.app._gait_apply_start_pair = apply
        self.app._gait_twin_snapshot = controller._gait_twin_snapshot
        self.app._refresh_gait_ui = lambda: refresh_twin_panel(self.app)
        self.app.gait_field_vars['lift_mm'].set('42.5')
        view = self.app.gait_widgets['twin']
        view['start_left'].set(map_label('C'))
        view['start_right'].set(map_label('A'))
        with patch('motor_control.ui.gait_twin.messagebox.askokcancel', return_value=True):
            apply_twin_start(self.app)
        self.assertEqual(controller._gait_supports, ('C', 'A'))
        self.assertEqual(controller.gait_params.lift_mm, 42.5)
        self.assertEqual(self.app.gait_field_vars['beam_reference_deg'].get(), '60.0')
        self.assertIn('地图起步', self.app.gait_placement_var.get())
        self.assertFalse(self.app.gait_calibrated_var.get())
        self.assertEqual(self.app.gait_zero_vars['Mr1'].get(), '未标定')
        self.assertEqual(view['select_side'].get(), '')
        self.assertEqual(controller.commands, [])
        self.assertEqual(len(saved), 1)
        self.assertFalse(self.errors)


if __name__ == "__main__":
    unittest.main()
