import math
import threading
import time
import unittest
from unittest.mock import patch

from motor_control.axis_math import (
    MODE_ROTARY,
    PULSE_RATE_WARN_PPS,
    delay_ms_to_pulse_rate,
    speed_clamps_delay,
    speed_to_delay_ms,
)
from motor_control.axis_model import AxisProfile, AxisRuntime
from motor_control.topology import (
    NUM_STEPPER_AXES,
    step_delay_ms_to_pulse_rate,
    step_speed_to_delay_ms,
)
from motor_control.ui_dispatch import UiDispatchTimeout
from motor_control.web_adapter import (
    DIR_INWARD,
    DIR_OUTWARD,
    DesktopWebController,
)


class FakeVar:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeDesktopApplication:
    """Small non-Tk application double for the HTTP adapter boundary."""

    def __init__(self):
        self.state_lock = threading.RLock()
        self._estop_lock = threading.Lock()
        self._closing = False
        self._disconnecting = False
        self._estop_in_progress = False
        self._estop_unconfirmed = False
        self._control_generation = 7
        self._track_lease_generation = 0
        self._track_direction = "STOP"
        self._track_last_response = ""
        self._track_lease_ms = 800
        self._fw_mode_cache = "FOC"
        self._connected = True

        self.running = [False] * NUM_STEPPER_AXES
        self.stepper_in_progress = [False] * NUM_STEPPER_AXES
        self._move_dispatching = [False] * NUM_STEPPER_AXES
        self._move_reservation = [None] * NUM_STEPPER_AXES
        self._web_step_pending = [None] * NUM_STEPPER_AXES
        self._axis_motion_generation = [0] * NUM_STEPPER_AXES
        self._pending_step = [None] * NUM_STEPPER_AXES
        self.axis_param_valid = [True] * NUM_STEPPER_AXES
        self.axis_profiles = [AxisProfile() for _ in range(NUM_STEPPER_AXES)]
        self.axis_runtime = [
            AxisRuntime(position_trusted=True) for _ in range(NUM_STEPPER_AXES)
        ]
        self._motor_status = [
            {
                "state": "2",
                "current_deg": 1.5,
                "target_deg": 2.0,
                "fault": None,
            },
            {
                "state": "0",
                "current_deg": None,
                "target_deg": None,
                "fault": None,
            },
        ]

        self.axis_mode_var = [FakeVar("linear") for _ in range(NUM_STEPPER_AXES)]
        self.axis_ppr_var = [FakeVar(200.0) for _ in range(NUM_STEPPER_AXES)]
        self.axis_gr_var = [FakeVar(1.0) for _ in range(NUM_STEPPER_AXES)]
        self.axis_lead_var = [FakeVar(1.0) for _ in range(NUM_STEPPER_AXES)]
        self.v_track_status = FakeVar("")

        self.call_ui_timeouts = []
        self.ui_callback_completed = False
        self.stop_all_calls = []
        self.stop_all_result = True
        self.serial_commands = []
        self.stepper_moves = []
        self.move_called = threading.Event()
        self.track_stop_calls = []
        self.serial_response_override = None
        self.track_stop_response = "OK,TRACK,D"

    def _is_serial_connected(self):
        return self._connected

    def _ensure_web_control_available(self, *, allow_estop_retry=False):
        if self._closing:
            raise RuntimeError("控制器正在关闭")
        if self._disconnecting:
            raise RuntimeError("串口正在断开")
        if self._estop_in_progress:
            raise RuntimeError("软件急停正在执行")
        if self._estop_unconfirmed and not allow_estop_retry:
            raise RuntimeError("ESTOP 未获确认")
        if not self._connected:
            raise RuntimeError("串口未连接")
        return self._control_generation

    def _call_ui(self, callback, timeout=2.0):
        self.call_ui_timeouts.append(timeout)
        result = callback()
        self.ui_callback_completed = True
        return result

    def _post_ui(self, callback):
        callback()

    def _send_and_read(self, command, timeout=0.8, guard=None, **_kwargs):
        self.serial_commands.append((command, timeout, guard() if guard else None))
        if self.serial_response_override is not None:
            return self.serial_response_override
        fields = command.split(",")
        if fields[0] in ("STOP", "FOC") and len(fields) > 1:
            return f"OK,{fields[1]}"
        if fields[0] == "TRACK":
            return "OK,TRACK,D"
        return "OK"

    def _send_mm(
        self,
        axis,
        distance,
        direction,
        delay_ms,
        profile=None,
        guard=None,
        reservation=None,
        motion_generation=None,
    ):
        with self.state_lock:
            if motion_generation is None:
                motion_generation = self._axis_motion_generation[axis]
            if (
                motion_generation != self._axis_motion_generation[axis]
                or (
                    reservation is not None
                    and self._move_reservation[axis] is not reservation
                )
            ):
                return False
        self.stepper_moves.append(
            (
                axis,
                distance,
                direction,
                delay_ms,
                profile,
                guard() if guard else None,
            )
        )
        if reservation is not None:
            with self.state_lock:
                if self._move_reservation[axis] is reservation:
                    self._move_reservation[axis] = None
        self.move_called.set()
        return True

    def _begin_axis_stop(self, axis):
        with self.state_lock:
            self._axis_motion_generation[axis] += 1
            stop_reservation = object()
            self.running[axis] = False
            self._move_reservation[axis] = stop_reservation
            self._web_step_pending[axis] = None
        return stop_reservation

    def _send_axis_stop(self, axis, stop_reservation=None, guard=None):
        if stop_reservation is None:
            stop_reservation = self._begin_axis_stop(axis)
        try:
            return self._send_and_read(
                f"STOP,{axis}", timeout=0.6, guard=guard
            )
        finally:
            with self.state_lock:
                if self._move_reservation[axis] is stop_reservation:
                    self._move_reservation[axis] = None

    def _send_track_stop(self, generation, control_guard=None):
        self.track_stop_calls.append(
            (generation, control_guard() if control_guard else None)
        )
        return self.track_stop_response

    def _on_axis_param_change(self, axis):
        current = self.axis_profiles[axis]
        self.axis_profiles[axis] = AxisProfile(
            mode=current.mode,
            pulse_per_rev=self.axis_ppr_var[axis].get(),
            gear_ratio=self.axis_gr_var[axis].get(),
            lead_mm=self.axis_lead_var[axis].get(),
        )
        return True

    def _on_axis_mode_change(self, axis):
        self.axis_profiles[axis] = self.axis_profiles[axis].with_mode(
            self.axis_mode_var[axis].get()
        )
        return True

    def _stop_all_outputs(self, allow_closing=False):
        self.stop_all_calls.append(allow_closing)
        self._estop_unconfirmed = not self.stop_all_result
        return self.stop_all_result


class DesktopWebControllerTests(unittest.TestCase):
    def setUp(self):
        self.app = FakeDesktopApplication()
        self.controller = DesktopWebController(self.app)

    def test_direction_parser_keeps_all_existing_aliases(self):
        for direction in ("FWD", " forward ", "OUT", "outward", "UP", "+", 1):
            with self.subTest(direction=direction):
                self.assertEqual(
                    self.controller.parse_direction(direction), DIR_OUTWARD
                )
        for direction in ("REV", " reverse ", "IN", "inward", "DOWN", "-", 0):
            with self.subTest(direction=direction):
                self.assertEqual(
                    self.controller.parse_direction(direction), DIR_INWARD
                )
        with self.assertRaises(ValueError):
            self.controller.parse_direction("sideways")

    def test_axis_config_waits_and_returns_values_actually_applied(self):
        self.app.axis_runtime[0].position_steps = 1234.0
        result = self.controller.web_stepper_config(
            0,
            mode=MODE_ROTARY,
            pulse_per_rev="400",
            gear_ratio=2,
            lead_mm=4,
        )

        self.assertTrue(self.app.ui_callback_completed)
        self.assertEqual(self.app.call_ui_timeouts, [2.0])
        self.assertEqual(
            result,
            {
                "ok": True,
                "axis": 0,
                "mode": "rotary",
                "pulse_per_rev": 400.0,
                "gear_ratio": 2.0,
                "lead_mm": 4.0,
            },
        )
        self.assertEqual(self.app.axis_profiles[0].mode, "rotary")
        self.assertEqual(self.app.axis_runtime[0].position_steps, 1234.0)

    def test_axis_config_converts_ui_timeout_to_api_error(self):
        def timeout(_callback, timeout=2.0):
            raise UiDispatchTimeout("late")

        self.app._call_ui = timeout
        with self.assertRaisesRegex(RuntimeError, "GUI 主线程未及时应用轴配置"):
            self.controller.web_stepper_config(0, mode=MODE_ROTARY)

    def test_axis_config_rejects_non_finite_parameter_before_ui_dispatch(self):
        with self.assertRaises(ValueError):
            self.controller.web_stepper_config(0, pulse_per_rev=math.nan)
        self.assertEqual(self.app.call_ui_timeouts, [])

    def test_config_and_move_share_one_profile_reservation_boundary(self):
        entered_config = threading.Event()
        release_config = threading.Event()
        config_results = []
        move_results = []
        original_apply = self.app._on_axis_param_change

        def paused_apply(axis):
            entered_config.set()
            self.assertTrue(release_config.wait(1.0))
            return original_apply(axis)

        self.app._on_axis_param_change = paused_apply
        config_thread = threading.Thread(
            target=lambda: config_results.append(
                self.controller.web_stepper_config(
                    0, mode=MODE_ROTARY, pulse_per_rev=400
                )
            )
        )
        config_thread.start()
        self.assertTrue(entered_config.wait(1.0))

        move_thread = threading.Thread(
            target=lambda: move_results.append(
                self.controller.web_stepper_move(0, "FWD", 10.0, 3.0)
            )
        )
        move_thread.start()
        move_thread.join(0.05)
        self.assertFalse(self.app.move_called.is_set())

        release_config.set()
        config_thread.join(1.0)
        move_thread.join(1.0)
        self.assertEqual(config_results[0]["mode"], MODE_ROTARY)
        self.assertTrue(move_results[0]["accepted"])
        self.assertTrue(self.app.move_called.wait(1.0))
        captured_profile = self.app.stepper_moves[0][4]
        self.assertEqual(captured_profile.mode, MODE_ROTARY)
        self.assertEqual(captured_profile.pulse_per_rev, 400.0)

    def test_status_keeps_http_field_names_and_step_based_values(self):
        self.app.axis_runtime[0].position_steps = 500.0
        status = self.controller.web_get_status()

        self.assertTrue(status["connected"])
        self.assertEqual(status["topology"]["total_axes"], 30)
        self.assertEqual(status["stepper_axes"][0]["position_mm"], 2.5)
        self.assertEqual(status["stepper_axes"][6]["controller"], "pico")
        self.assertEqual(status["stepper_axes"][29]["node"], 6)
        self.assertIn("position_mm", status["stepper_axes"][0])
        self.assertIn("travel_min_mm", status["stepper_axes"][0])

    def test_estop_invalidates_prior_generation_and_clears_busy_flag(self):
        result = self.controller.web_emergency_stop()

        self.assertEqual(result, {"ok": True, "confirmed": True})
        self.assertEqual(self.app._control_generation, 8)
        self.assertFalse(self.app._estop_in_progress)
        self.assertEqual(self.app.stop_all_calls, [True])

    def test_unconfirmed_estop_latches_motion_but_allows_estop_retry(self):
        self.app.stop_all_result = False
        with self.assertRaisesRegex(RuntimeError, "未获得固件确认"):
            self.controller.web_emergency_stop()
        self.assertTrue(self.app._estop_unconfirmed)
        self.assertFalse(self.app._estop_in_progress)
        with self.assertRaisesRegex(RuntimeError, "ESTOP 未获确认"):
            self.controller.web_stepper_move(0, "FWD", 1.0, 3.0)

        self.app.stop_all_result = True
        self.assertEqual(
            self.controller.web_emergency_stop(),
            {"ok": True, "confirmed": True},
        )
        self.assertFalse(self.app._estop_unconfirmed)

    def test_normal_rate_move_needs_no_confirmation_and_reports_metadata(self):
        result = self.controller.web_stepper_move(0, "FWD", 2.5, 3.0)

        profile = self.app.axis_profiles[0]
        requested_rate = 3.0 * profile.pulses_per_unit
        expected_delay = speed_to_delay_ms(3.0, profile.pulses_per_unit)
        self.assertLessEqual(requested_rate, PULSE_RATE_WARN_PPS)
        self.assertTrue(result["ok"])
        self.assertTrue(result["accepted"])
        self.assertEqual(result["axis"], 0)
        self.assertAlmostEqual(
            result["requested_pulse_rate_pps"], requested_rate
        )
        self.assertAlmostEqual(
            result["effective_pulse_rate_pps"],
            delay_ms_to_pulse_rate(expected_delay),
        )
        self.assertFalse(result["speed_clamped"])
        self.assertTrue(self.app.move_called.wait(1.0))
        axis, distance, direction, delay_ms, profile, guard_result = (
            self.app.stepper_moves[0]
        )
        self.assertEqual((axis, distance, direction), (0, 2.5, DIR_OUTWARD))
        self.assertAlmostEqual(delay_ms, 1.4666666666666666)
        self.assertIs(profile, self.app.axis_profiles[0])
        self.assertTrue(guard_result)

    def test_remote_move_uses_complete_rp2040_pulse_period(self):
        axis = 6
        speed = 3.0
        profile = self.app.axis_profiles[axis]

        result = self.controller.web_stepper_move(
            axis, "FWD", 2.5, speed
        )

        expected_delay = step_speed_to_delay_ms(
            axis, speed, profile.pulses_per_unit
        )
        self.assertAlmostEqual(expected_delay, 1000.0 / 600.0)
        self.assertAlmostEqual(
            result["effective_pulse_rate_pps"],
            step_delay_ms_to_pulse_rate(axis, expected_delay),
        )
        self.assertAlmostEqual(result["effective_pulse_rate_pps"], 600.0)
        self.assertTrue(self.app.move_called.wait(1.0))
        move_axis, _distance, _direction, delay_ms, _profile, _guard = (
            self.app.stepper_moves[0]
        )
        self.assertEqual(move_axis, axis)
        self.assertAlmostEqual(delay_ms, expected_delay)

    def test_high_rate_move_requires_explicit_confirmation(self):
        profile = self.app.axis_profiles[0]
        speed = 6.0
        self.assertGreater(
            speed * profile.pulses_per_unit, PULSE_RATE_WARN_PPS
        )
        self.assertFalse(speed_clamps_delay(speed, profile.pulses_per_unit))

        with self.assertRaises(RuntimeError):
            self.controller.web_stepper_move(0, "FWD", 1.0, speed)

        self.assertIsNone(self.app._move_reservation[0])
        self.assertIsNone(self.app._web_step_pending[0])
        self.assertFalse(self.app.move_called.is_set())

    def test_confirmed_clamped_move_reports_requested_and_effective_rates(self):
        self.app.axis_profiles[0] = AxisProfile(
            pulse_per_rev=200.0,
            gear_ratio=21.5,
            lead_mm=1.0,
        )
        profile = self.app.axis_profiles[0]
        speed = 3.0
        requested_rate = speed * profile.pulses_per_unit
        expected_delay = speed_to_delay_ms(speed, profile.pulses_per_unit)
        self.assertGreater(requested_rate, PULSE_RATE_WARN_PPS)
        self.assertTrue(speed_clamps_delay(speed, profile.pulses_per_unit))

        result = self.controller.web_stepper_move(
            0,
            "FWD",
            1.0,
            speed,
            confirm_high_rate=True,
        )

        self.assertTrue(result["ok"])
        self.assertTrue(result["accepted"])
        self.assertEqual(result["axis"], 0)
        self.assertAlmostEqual(
            result["requested_pulse_rate_pps"], requested_rate
        )
        self.assertAlmostEqual(
            result["effective_pulse_rate_pps"],
            delay_ms_to_pulse_rate(expected_delay),
        )
        self.assertTrue(result["speed_clamped"])
        self.assertTrue(self.app.move_called.wait(1.0))

    def test_move_rejects_non_boolean_high_rate_confirmation(self):
        for value in (0, 1, "true", None, [], {}):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self.controller.web_stepper_move(
                        0,
                        "FWD",
                        1.0,
                        3.0,
                        confirm_high_rate=value,
                    )

        self.assertIsNone(self.app._move_reservation[0])
        self.assertIsNone(self.app._web_step_pending[0])
        self.assertFalse(self.app.move_called.is_set())

    def test_stepper_move_rejects_continuous_or_unsettled_axis(self):
        self.app.running[0] = True
        with self.assertRaisesRegex(RuntimeError, "正在运动"):
            self.controller.web_stepper_move(0, "FWD", 1.0, 3.0)

        self.app.running[0] = False
        self.app._pending_step[0] = 100
        with self.assertRaisesRegex(RuntimeError, "正在运动"):
            self.controller.web_stepper_move(0, "FWD", 1.0, 3.0)
        self.assertFalse(self.app.move_called.is_set())

    def test_web_reservation_is_fully_released_after_app_wrapper_returns(self):
        self.controller.web_stepper_move(0, "FWD", 1.0, 3.0)
        deadline = time.monotonic() + 1.0
        while self.app._web_step_pending[0] is not None:
            if time.monotonic() >= deadline:
                self.fail("web reservation remained sticky")
            time.sleep(0.005)

        self.app.move_called.clear()
        second = self.controller.web_stepper_move(0, "FWD", 1.0, 3.0)
        self.assertTrue(second["accepted"])
        self.assertTrue(self.app.move_called.wait(1.0))

    def test_web_stop_cancels_move_worker_that_has_not_reached_send(self):
        entered_worker = threading.Event()
        release_worker = threading.Event()
        worker_finished = threading.Event()
        original_send = self.app._send_mm

        def delayed_send(*args, **kwargs):
            entered_worker.set()
            self.assertTrue(release_worker.wait(1.0))
            try:
                return original_send(*args, **kwargs)
            finally:
                worker_finished.set()

        self.app._send_mm = delayed_send
        move_result = self.controller.web_stepper_move(0, "FWD", 1.0, 3.0)
        self.assertTrue(move_result["accepted"])
        self.assertTrue(entered_worker.wait(1.0))

        stop_result = self.controller.web_stepper_stop(0)
        release_worker.set()
        self.assertTrue(worker_finished.wait(1.0))

        self.assertEqual(
            stop_result, {"ok": True, "confirmed": True, "axis": 0}
        )
        self.assertEqual(self.app.stepper_moves, [])
        self.assertEqual(self.app.serial_commands[-1][0], "STOP,0")

    def test_web_stop_barrier_rejects_new_move_until_confirmation(self):
        stop_waiting = threading.Event()
        release_stop = threading.Event()
        original_send = self.app._send_and_read

        def delayed_stop(command, **kwargs):
            if command == "STOP,0":
                stop_waiting.set()
                self.assertTrue(release_stop.wait(1.0))
            return original_send(command, **kwargs)

        self.app._send_and_read = delayed_stop
        stop_results = []
        stop_thread = threading.Thread(
            target=lambda: stop_results.append(
                self.controller.web_stepper_stop(0)
            )
        )
        stop_thread.start()
        self.assertTrue(stop_waiting.wait(1.0))

        with self.assertRaisesRegex(RuntimeError, "正在运动"):
            self.controller.web_stepper_move(0, "FWD", 1.0, 3.0)

        release_stop.set()
        stop_thread.join(1.0)
        self.assertEqual(
            stop_results,
            [{"ok": True, "confirmed": True, "axis": 0}],
        )
        self.assertEqual(self.app.stepper_moves, [])

    def test_direct_move_and_target_reject_non_finite_values(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(operation="move-distance", value=value):
                with self.assertRaises(ValueError):
                    self.controller.web_stepper_move(0, "FWD", value, 3.0)
            with self.subTest(operation="move-speed", value=value):
                with self.assertRaises(ValueError):
                    self.controller.web_stepper_move(0, "FWD", 1.0, value)
            with self.subTest(operation="target", value=value):
                with self.assertRaises(ValueError):
                    self.controller.web_motor_command("FOC", 0, "target", value)
        self.assertFalse(self.app.move_called.is_set())
        self.assertEqual(self.app.serial_commands, [])

    def test_move_rejects_speed_below_protocol_delay_cap_before_reserving(self):
        self.app.axis_profiles[0] = AxisProfile(
            mode=MODE_ROTARY, pulse_per_rev=1.0, gear_ratio=0.001
        )
        with self.assertRaisesRegex(ValueError, "速度过低"):
            self.controller.web_stepper_move(0, "FWD", 1.0, 0.3)
        self.assertIsNone(self.app._move_reservation[0])
        self.assertIsNone(self.app._web_step_pending[0])
        self.assertFalse(self.app.move_called.is_set())

    def test_stop_motor_and_track_stop_keep_wire_commands_and_confirmations(self):
        self.assertEqual(
            self.controller.web_stepper_stop(29),
            {"ok": True, "confirmed": True, "axis": 29},
        )
        self.assertEqual(self.app.serial_commands[-1], ("STOP,29", 0.6, True))

        self.assertEqual(
            self.controller.web_motor_command("foc", 1, "target", 12.34),
            {
                "ok": True,
                "confirmed": True,
                "mode": "FOC",
                "axis": 1,
                "action": "target",
            },
        )
        self.assertEqual(self.app.serial_commands[-1], ("FOC,1,A,12.3", 0.8, True))

        self.assertEqual(
            self.controller.web_track_command("stop"),
            {"ok": True, "confirmed": True, "action": "stop"},
        )
        self.assertEqual(self.app.track_stop_calls, [(1, True)])
        self.assertEqual(self.app.v_track_status.get(), "已停止（网页）")

    def test_stop_endpoints_reject_firmware_errors(self):
        self.app.serial_response_override = "ERR:node timeout"
        with self.assertRaisesRegex(RuntimeError, "ERR:node timeout"):
            self.controller.web_stepper_stop(6)
        with self.assertRaisesRegex(RuntimeError, "ERR:node timeout"):
            self.controller.web_motor_command("FOC", 0, "enable")

        self.app.track_stop_response = "ERR:track failure"
        with self.assertRaisesRegex(RuntimeError, "ERR:track failure"):
            self.controller.web_track_command("stop")

    def test_stop_endpoints_reject_unexpected_nonempty_replies(self):
        self.app.serial_response_override = "OK,1"
        with self.assertRaisesRegex(RuntimeError, "非预期响应"):
            self.controller.web_stepper_stop(0)

        self.app.track_stop_response = "OK,TRACK,D,STOP"
        with self.assertRaisesRegex(RuntimeError, "非预期响应"):
            self.controller.web_track_command("stop")

    def test_old_track_timer_cannot_stop_new_same_direction_lease(self):
        timers = []

        class CapturedTimer:
            def __init__(self, _delay, callback):
                self.callback = callback
                self.daemon = False
                timers.append(self)

            def start(self):
                return None

        with patch("motor_control.web_adapter.threading.Timer", CapturedTimer):
            self.controller.web_track_command("forward", pwm=20, lease_ms=100)
            old_timer = timers[-1]
            self.controller.web_track_command("forward", pwm=30, lease_ms=2_000)

        old_timer.callback()
        with self.app.state_lock:
            self.assertEqual(self.app._track_lease_generation, 2)
            self.assertEqual(self.app._track_direction, "FWD")
            self.assertEqual(self.app._track_lease_ms, 2_000)


if __name__ == "__main__":
    unittest.main()
