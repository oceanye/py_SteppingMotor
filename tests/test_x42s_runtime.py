"""Offline regressions for the 2026-10-16 X42S branch (no serial hardware)."""
import os
import sys
import time
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_x42s_pc import _x42s_gait_app
from motor_control import LogicalRole
from motor_control import desktop_app
from motor_control.gait_executor import GaitExecutorError
from motor_control.protocol import parse_line
from motor_control.ui.scrollable import ScrollableFrame


def status(*, online=1, enabled=1, reached=1, measured=215,
           age=5, mask=7, fault="none"):
    return (f"X42S,2,S,{online},{enabled},{reached},{fault},"
            f"215,{measured},0,0,0,25,{age},0,{mask}")


class X42sRuntimeTests(unittest.TestCase):
    def test_offline_reply_must_not_revive_cached_driver(self):
        app = _x42s_gait_app()
        app._apply_x42s_reply(LogicalRole.MR1, 2, parse_line(status(online=0)))
        state = app._x42s_states["Mr1"]
        self.assertFalse(state.online)
        self.assertFalse(state.telemetry_trusted)
        self.assertIsNone(app._x42s_role_output_units(LogicalRole.MR1))

    def test_firmware_sample_age_is_not_reset_by_receipt(self):
        app = _x42s_gait_app()
        app._apply_x42s_reply(LogicalRole.MR1, 2, parse_line(status(age=1500)))
        state = app._x42s_states["Mr1"]
        self.assertGreaterEqual(state.age_ms(), 1500)
        self.assertFalse(state.telemetry_trusted)

    def test_invalid_fields_clear_previous_valid_values(self):
        app = _x42s_gait_app()
        app._apply_x42s_reply(LogicalRole.MR1, 2, parse_line(status(mask=0)))
        state = app._x42s_states["Mr1"]
        for name in ("command_position", "measured_position", "position_error",
                     "speed", "current_a", "temperature_c"):
            self.assertIsNone(getattr(state, name), name)
        self.assertFalse(state.telemetry_trusted)

    def test_reply_for_previous_binding_is_ignored(self):
        app = _x42s_gait_app()
        before = app._x42s_states["Mr1"]
        app._apply_x42s_reply(LogicalRole.MR1, 3,
                             parse_line(status().replace("X42S,2,", "X42S,3,")))
        self.assertIs(app._x42s_states["Mr1"], before)

    def test_terminal_reply_updates_position_used_by_next_segment(self):
        app = _x42s_gait_app()
        app._send_and_read = lambda *_a, **_k: status(measured=215)
        with patch.object(desktop_app, "X42S_WAIT_BLIND_S", 0):
            self.assertEqual(app._gait_wait_terminal(102, 0.2), "DONE")
        self.assertAlmostEqual(app._x42s_role_output_units(LogicalRole.MR1), 10)

    def test_next_cumulative_segment_uses_confirmed_endpoint(self):
        app = _x42s_gait_app()
        run, _ = app._gait_begin_run("left", arc_deg=-60)
        run.x42s_sync_origin = {102: 0.0, 104: 0.0}
        controller = app._send_and_read
        app._send_and_read = lambda *_a, **_k: status(measured=215)
        with patch.object(desktop_app, "X42S_WAIT_BLIND_S", 0):
            self.assertEqual(app._gait_wait_terminal(102, 0.2), "DONE")
        app._send_and_read = controller
        self.assertEqual(app._gait_send_synchronized(
            [(102, 2, 1), (104, 0, 1)], 2, cumulative_targets=(12, 0)), "sent")
        commands = [c for c in app.commands if ",MOVE," in c]
        expected_steps = round(2 * 21.5 * desktop_app.X42S_PULSE_PER_REV / 360)
        self.assertEqual(len(commands), 1)
        self.assertEqual(int(commands[0].split(",")[3]), expected_steps)

    def test_offline_or_disabled_reached_reply_is_not_done(self):
        for kwargs in ({"online": 0}, {"enabled": 0}):
            with self.subTest(**kwargs):
                app = _x42s_gait_app()
                app._send_and_read = lambda *_a, **_k: status(**kwargs)
                with patch.object(desktop_app, "X42S_WAIT_BLIND_S", 0):
                    self.assertEqual(app._gait_wait_terminal(102, 0.1), "ABORTED")

    def test_stale_or_unknown_position_cannot_complete_segment(self):
        for kwargs in ({"age": 1000}, {"mask": 0}):
            with self.subTest(**kwargs):
                app = _x42s_gait_app()
                app._send_and_read = lambda *_a, **_k: status(**kwargs)
                with patch.object(desktop_app, "X42S_WAIT_BLIND_S", 0):
                    self.assertEqual(app._gait_wait_terminal(102, 0.05), "TIMEOUT")

    def test_staggered_background_samples_are_refreshed_before_start(self):
        app = _x42s_gait_app()
        # 四台按 2Hz 轮询时，最早一台已经超过 200ms；不能要求碰巧同时新鲜。
        for i, (role, state) in enumerate(app._x42s_states.items()):
            app._x42s_states[role] = replace(
                state, last_update_monotonic=time.monotonic() - (0.6 - i * 0.125))
        run, _ = app._gait_begin_run("left", arc_deg=-60)
        self.assertIsNotNone(run)
        self.assertEqual([c for c in app.commands if c.startswith("X42S,S,")],
                         ["X42S,S,1", "X42S,S,2", "X42S,S,3", "X42S,S,4"])
        self.assertFalse(any(",MOVE," in c for c in app.commands))

    def test_preflight_refresh_timeout_does_not_bypass_freshness(self):
        app = _x42s_gait_app()
        state = app._x42s_states["Mr1"]
        app._x42s_states["Mr1"] = replace(
            state, last_update_monotonic=time.monotonic() - 1)
        app._send_and_read = lambda *_a, **_k: ""
        with self.assertRaises(GaitExecutorError):
            app._gait_begin_run("left", arc_deg=-60)
        self.assertEqual(app._gait_owned, {})

    def test_zero_refreshes_stale_sample_before_existing_safety_checks(self):
        # 2026-10-19：记零改 PC 侧基准（measured 默认 0 ⇒ 基准 0），
        # 仍先刷新过期遥测再走静止/可信检查；不再发驱动器 ZERO。
        app = _x42s_gait_app()
        app.state_store = SimpleNamespace(save_gait_params=lambda _doc: None)
        state = app._x42s_states["Mr1"]
        app._x42s_states["Mr1"] = replace(
            state, last_update_monotonic=time.monotonic() - 1)
        self.assertEqual(app._gait_record_role_zero("Mr1"), 0)
        self.assertEqual(app.commands, ["X42S,S,2"])

    def test_manual_timeout_has_actionable_message(self):
        app = _x42s_gait_app()
        app._send_and_read = lambda *_a, **_k: ""
        ok, reason = app._send_manual_x42s("X42S,2,STOP", "Mr1", 2)
        self.assertFalse(ok)
        self.assertIn("无应答", reason)

    def test_light_probe_without_speed_or_reached_cannot_zero(self):
        app = _x42s_gait_app()
        app._apply_x42s_reply(LogicalRole.MR1, 2,
                             parse_line(status(reached=0, mask=2)))
        with self.assertRaisesRegex(GaitExecutorError, "不能记零"):
            app._gait_record_role_zero("Mr1")
        self.assertNotIn("X42S,2,ZERO", app.commands)


class FocusRuntimeTests(unittest.TestCase):
    def test_tk_unknown_focus_time_is_not_subtracted(self):
        widget = SimpleNamespace(master=None)
        viewport = SimpleNamespace(update_idletasks=Mock())
        with patch("motor_control.ui.scrollable.time.monotonic", return_value=20):
            ScrollableFrame._mark_pointer_focus(
                viewport, SimpleNamespace(widget=widget, time=1234))
            ScrollableFrame._reveal_focus(
                viewport, SimpleNamespace(widget=widget, time="??"))
        viewport.update_idletasks.assert_not_called()

    def test_tab_navigation_clears_pointer_suppression(self):
        viewport = SimpleNamespace(_pointer_click=(object(), 20))
        ScrollableFrame._keyboard_focus(viewport, None)
        self.assertIsNone(viewport._pointer_click)


if __name__ == "__main__":
    unittest.main()
