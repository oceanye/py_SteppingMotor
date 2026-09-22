"""Real Tk layout checks using an isolated page; no controller or serial port.

Run with a Python installation containing Tk, e.g. C:\\Python313\\python.exe.
The embedded project Python can skip these checks when Tk is unavailable.
"""
import unittest
import math
from dataclasses import replace
from types import SimpleNamespace

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = hasattr(tk, "Tk")
except ImportError:
    HAS_TK = False

from motor_control.gait_planner import GaitParams


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
        for name in ("_gait_record_zero_clicked", "_gait_save_params_clicked",
                     "_gait_reload_params_clicked", "_gait_reestablish_baseline",
                     "_gait_run_dry_run", "_gait_start_run_clicked",
                     "_gait_play_preview_clicked", "_gait_stage_confirmed",
                     "_gait_abort_clicked", "_gait_reset_run"):
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
        self.assertTrue(canvas.find_withtag("target_left"))
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


if __name__ == "__main__":
    unittest.main()
