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
    """滚轮缩放/中键平移的视图复合变换（无需真实画布）。"""

    WIDTH, HEIGHT = 800, 600

    def setUp(self):
        params = GaitParams(
            trajectory_mode="two_mode_v1",
            geometry=GaitGeometry(d_mm=104.0, arm_length_mm=60.0))
        self.app = SimpleNamespace(
            gait_params=params,
            gait_widgets={"preview_view": {"zoom": 1.0, "pan_x": 0.0,
                                           "pan_y": 0.0}},
        )
        self.report = plan_swing_trajectory(params, side="left")
        self.canvas_stub = SimpleNamespace(
            winfo_width=lambda: self.WIDTH,
            winfo_height=lambda: self.HEIGHT)

    def _to_canvas(self):
        from motor_control.ui.gait_tab import _preview_scene
        scene = _preview_scene(self.app, self.report, self.WIDTH, self.HEIGHT)
        return scene["to_canvas"]

    def test_zoom_requires_activation_and_keeps_pointer_point_fixed(self):
        from motor_control.ui.common import canvas_view_zoom
        world = self.report.samples[len(self.report.samples) // 2].center
        view = self.app.gait_widgets["preview_view"]
        # 未左键选中画面：滚轮不缩放，返回 None 让页面滚动绑定继续处理
        idle = canvas_view_zoom(view, SimpleNamespace(
            widget=self.canvas_stub, x=50, y=50, delta=120), lambda: None)
        self.assertIsNone(idle)
        self.assertEqual(view["zoom"], 1.0)
        view["active"] = True
        mx, my = self._to_canvas()(*world)
        for delta in (120, 120, -120):
            result = canvas_view_zoom(view, SimpleNamespace(
                widget=self.canvas_stub, x=int(mx), y=int(my), delta=delta),
                lambda: None)
            self.assertEqual(result, "break")   # 激活后阻断页面滚动
        self.assertGreater(view["zoom"], 1.0)
        nx, ny = self._to_canvas()(*world)
        self.assertAlmostEqual(nx, mx, delta=0.2)
        self.assertAlmostEqual(ny, my, delta=0.2)

    def test_pan_drag_shifts_view_and_reset_restores(self):
        from motor_control.ui.common import (
            canvas_view_pan_move, canvas_view_pan_start, reset_canvas_view)
        world = self.report.samples[0].center
        x0, y0 = self._to_canvas()(*world)
        canvas_view_pan_start(self.app.gait_widgets["preview_view"],
                              SimpleNamespace(x=100, y=100))
        canvas_view_pan_move(self.app.gait_widgets["preview_view"],
                             SimpleNamespace(x=140, y=75), lambda: None)
        x1, y1 = self._to_canvas()(*world)
        self.assertAlmostEqual(x1 - x0, 40.0, places=6)
        self.assertAlmostEqual(y1 - y0, -25.0, places=6)
        reset_canvas_view(self.app.gait_widgets["preview_view"])
        x2, y2 = self._to_canvas()(*world)
        self.assertAlmostEqual(x2, x0, places=6)
        self.assertAlmostEqual(y2, y0, places=6)

    def test_twin_pad_window_resolves_two_rings_of_neighbors(self):
        # 2026-09-29 孪生画面固定晶格窗口：ABC+两圈邻座，名字必须能被
        # pad_center 解析回同一坐标（与规划器同一几何），保证换 route
        # 不跳视野的同时落点支座一定在画面内。
        from motor_control.gait_twin import pad_center
        from motor_control.ui.gait_twin import twin_pad_window
        pads = twin_pad_window()
        self.assertTrue(set("ABC") <= set(pads))
        self.assertGreaterEqual(len(pads), 15)
        xs = [x for x, _y in pads.values()]
        self.assertGreaterEqual(max(xs) - min(xs), 4.0)   # 覆盖到两圈邻座
        for name, xy in pads.items():
            self.assertEqual(pad_center(name), xy)


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
        canvas.yview_moveto(1.0)
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
        self.assertEqual(button.cget("text"), "⏸ 暂停模拟")

        pause_preview_animation(self.app)
        self.assertFalse(anim["playing"])
        self.assertTrue(anim["paused"])
        self.assertGreater(anim["frame"], 0)
        self.assertEqual(button.cget("text"), "▶ 继续模拟")

        # 暂停中滚轮缩放：重画必须保持暂停帧（帧号不变、仍处暂停态）
        self.app.gait_widgets["preview_view"].update(active=True, zoom=1.5)
        _preview_redraw(self.app)
        self.assertTrue(anim["paused"])
        self.assertFalse(self.errors)

        paused_frame = anim["frame"]
        resume_preview_animation(self.app)
        self.assertTrue(anim["playing"])
        self.assertFalse(anim["paused"])
        self.assertGreaterEqual(anim["frame"], paused_frame)
        self.assertEqual(button.cget("text"), "⏸ 暂停模拟")

        pause_preview_animation(self.app)
        stop_preview_animation(self.app)
        self.assertFalse(anim["playing"] or anim["paused"])
        self.assertEqual(button.cget("text"), "▶ 模拟动作")

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


if __name__ == "__main__":
    unittest.main()
