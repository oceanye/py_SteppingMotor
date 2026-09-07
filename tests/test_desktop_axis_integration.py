import math
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch


def _ensure_tkinter_import_surface():
    try:
        import tkinter  # noqa: F401
        return
    except ImportError:
        pass
    tkinter_module = types.ModuleType("tkinter")
    tkinter_module.__path__ = []

    class TclError(Exception):
        pass

    tkinter_module.TclError = TclError
    ttk_module = types.ModuleType("tkinter.ttk")
    messagebox_module = types.ModuleType("tkinter.messagebox")
    for name in ("showerror", "showwarning", "showinfo", "askyesno", "askokcancel"):
        setattr(messagebox_module, name, lambda *_args, **_kwargs: None)
    tkinter_module.ttk = ttk_module
    tkinter_module.messagebox = messagebox_module
    sys.modules["tkinter"] = tkinter_module
    sys.modules["tkinter.ttk"] = ttk_module
    sys.modules["tkinter.messagebox"] = messagebox_module


def _ensure_serial_import_surface():
    try:
        import serial.tools.list_ports  # noqa: F401
        return
    except ImportError:
        pass
    serial_module = types.ModuleType("serial")
    serial_module.__path__ = []
    tools_module = types.ModuleType("serial.tools")
    tools_module.__path__ = []
    ports_module = types.ModuleType("serial.tools.list_ports")
    ports_module.comports = lambda: []
    tools_module.list_ports = ports_module
    serial_module.tools = tools_module
    serial_module.Serial = object
    sys.modules["serial"] = serial_module
    sys.modules["serial.tools"] = tools_module
    sys.modules["serial.tools.list_ports"] = ports_module


_ensure_tkinter_import_surface()
_ensure_serial_import_surface()

from motor_control import (
    AxisMotionTelemetry,
    BindingSet,
    BindingValidationError,
    MODE_ROTARY,
    AxisProfile,
    AxisRuntime,
    NUM_STEPPER_AXES,
)
from motor_control.desktop_app import (
    DIR_OUTWARD,
    StepperGUI,
    _continuous_burst_units,
)
from motor_control.serial_session import RequestCancelled, RequestTimeout
from motor_control.serial_session import SessionClosed
from motor_control.protocol import StepResult, StepTerminal


def _headless_app():
    app = StepperGUI.__new__(StepperGUI)
    app.axis_profiles = [AxisProfile() for _ in range(NUM_STEPPER_AXES)]
    app.axis_runtime = [
        AxisRuntime(position_trusted=True) for _ in range(NUM_STEPPER_AXES)
    ]
    app.running = [False] * NUM_STEPPER_AXES
    app.stepper_in_progress = [False] * NUM_STEPPER_AXES
    app._move_dispatching = [False] * NUM_STEPPER_AXES
    app._move_reservation = [None] * NUM_STEPPER_AXES
    app._web_step_pending = [None] * NUM_STEPPER_AXES
    app._axis_motion_generation = [0] * NUM_STEPPER_AXES
    app._pending_step = [None] * NUM_STEPPER_AXES
    app.axis_motion_telemetry = [
        AxisMotionTelemetry() for _ in range(NUM_STEPPER_AXES)
    ]
    app.control_bindings = BindingSet.empty()
    app.pico_node_health = {node: "unknown" for node in range(1, 7)}
    app.axis_param_valid = [True] * NUM_STEPPER_AXES
    app.state_lock = threading.RLock()
    app.sw = [{} for _ in range(NUM_STEPPER_AXES)]
    app._logs = []
    app.log = app._logs.append
    app._save_calib = lambda: None
    app._update_pos_label = lambda _axis: None
    app._update_range_display = lambda _axis: None
    return app


class DesktopAxisIntegrationTests(unittest.TestCase):
    def test_absolute_target_is_recomputed_after_worker_delay(self):
        app = _headless_app()
        app.axis_param_valid = [True] * NUM_STEPPER_AXES
        app._require_axis_params = lambda _axis: True
        app._is_serial_connected = lambda: True
        commands = []
        app._send_and_read = lambda command, **_kwargs: (
            commands.append(command) or "ACK,0"
        )

        class Value:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        app.v_delay = [Value(1.0) for _ in range(NUM_STEPPER_AXES)]
        queued = []
        def queue_worker(target, *args, **kwargs):
            queued.append((target, args, kwargs))
            return object()

        app._start_control_worker = queue_worker

        StepperGUI._goto(app, 0, 15.0)
        self.assertEqual(len(queued), 1)
        # A previous command settles before the newly scheduled worker runs.
        app.axis_runtime[0].position_steps = 2_000
        target, args, kwargs = queued[0]
        accepted = target(*args, **kwargs)

        self.assertTrue(accepted)
        self.assertEqual(commands, ["MOVE,0,1000,1,1000"])

    def test_scheduled_gui_move_pins_profile_until_worker_send(self):
        app = _headless_app()
        app.axis_param_valid = [True] * NUM_STEPPER_AXES
        app._require_axis_params = lambda _axis: True
        app._is_serial_connected = lambda: True
        commands = []
        app._send_and_read = lambda command, **_kwargs: (
            commands.append(command) or "ACK,0"
        )

        class Value:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

            def set(self, value):
                self.value = value

        app.v_dist = [Value(1.0) for _ in range(NUM_STEPPER_AXES)]
        app.v_dir = [Value(DIR_OUTWARD) for _ in range(NUM_STEPPER_AXES)]
        app.v_delay = [Value(1.0) for _ in range(NUM_STEPPER_AXES)]
        app.axis_ppr_var = [Value(200.0) for _ in range(NUM_STEPPER_AXES)]
        app.axis_gr_var = [Value(1.0) for _ in range(NUM_STEPPER_AXES)]
        app.axis_lead_var = [Value(1.0) for _ in range(NUM_STEPPER_AXES)]
        queued = []

        def queue_worker(target, *args, **kwargs):
            queued.append((target, args, kwargs))
            return object()

        app._start_control_worker = queue_worker
        StepperGUI.send_move(app, 0)
        self.assertEqual(len(queued), 1)
        self.assertIsNotNone(app._move_reservation[0])

        app.axis_ppr_var[0].set(400.0)
        self.assertFalse(StepperGUI._on_axis_param_change(app, 0))
        self.assertEqual(app.axis_profiles[0].pulse_per_rev, 200.0)

        target, args, kwargs = queued[0]
        self.assertTrue(target(*args, **kwargs))
        self.assertEqual(commands, ["MOVE,0,200,1,1000"])

    def test_axis_stop_invalidates_move_waiting_at_presend_guard(self):
        app = _headless_app()
        app._is_serial_connected = lambda: True
        move_waiting = threading.Event()
        release_move = threading.Event()
        commands = []

        def send(command, guard=None, propagate_request_error=False, **_kwargs):
            if command.startswith("MOVE,"):
                move_waiting.set()
                self.assertTrue(release_move.wait(1.0))
                if guard is not None and not guard():
                    if propagate_request_error:
                        raise RequestCancelled("cancelled by axis STOP")
                    return ""
                commands.append(command)
                return "ACK,0"
            commands.append(command)
            return "OK,0"

        app._send_and_read = send
        results = []
        move_thread = threading.Thread(
            target=lambda: results.append(
                StepperGUI._send_mm(
                    app,
                    0,
                    1.0,
                    DIR_OUTWARD,
                    1.0,
                    profile=app.axis_profiles[0],
                    motion_generation=0,
                )
            )
        )
        move_thread.start()
        self.assertTrue(move_waiting.wait(1.0))

        stop_reservation = StepperGUI._begin_axis_stop(app, 0)
        response = StepperGUI._send_axis_stop(app, 0, stop_reservation)
        release_move.set()
        move_thread.join(1.0)

        self.assertEqual(response, "OK,0")
        self.assertEqual(results, [False])
        self.assertEqual(commands, ["STOP,0"])
        self.assertIsNone(app._pending_step[0])
        self.assertFalse(app.stepper_in_progress[0])

    def test_gui_stepper_inputs_reject_non_finite_values(self):
        class Value:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        for invalid in (math.nan, math.inf, -math.inf):
            with self.subTest(value=invalid):
                app = _headless_app()
                app.axis_param_valid = [True] * NUM_STEPPER_AXES
                app._require_axis_params = lambda _axis: True
                app.v_dist = [Value(invalid) for _ in range(NUM_STEPPER_AXES)]
                app.v_goto = [Value(invalid) for _ in range(NUM_STEPPER_AXES)]
                app.v_dir = [Value(DIR_OUTWARD) for _ in range(NUM_STEPPER_AXES)]
                app.v_delay = [Value(1.0) for _ in range(NUM_STEPPER_AXES)]
                workers = []
                app._start_control_worker = (
                    lambda *args, **kwargs: workers.append((args, kwargs))
                )

                with patch(
                    "motor_control.desktop_app.messagebox.showerror"
                ) as showerror:
                    StepperGUI.send_move(app, 0)
                    StepperGUI.calibrate_position(app, 0)
                    StepperGUI.goto_target_position(app, 0)

                self.assertEqual(workers, [])
                self.assertEqual(app.axis_runtime[0].position_steps, 0)
                self.assertGreaterEqual(showerror.call_count, 3)

    def test_continuous_burst_is_at_least_one_pulse_in_rotary_mode(self):
        linear = AxisProfile()
        rotary = AxisProfile(mode=MODE_ROTARY)

        self.assertEqual(_continuous_burst_units(linear), 0.2)
        self.assertAlmostEqual(_continuous_burst_units(rotary), 1.8)
        self.assertEqual(
            rotary.command_steps_from_units(_continuous_burst_units(rotary)), 1
        )

    def test_calibration_profile_metadata_detects_cross_file_mismatch(self):
        profile = AxisProfile(mode=MODE_ROTARY, pulse_per_rev=400, gear_ratio=2)
        metadata = StepperGUI._profile_metadata(profile)

        self.assertTrue(
            StepperGUI._profile_metadata_matches(profile, metadata)
        )
        metadata["gear_ratio"] = 3
        self.assertFalse(
            StepperGUI._profile_metadata_matches(profile, metadata)
        )

    def test_default_rotary_low_speed_delay_is_encodable(self):
        app = _headless_app()
        app._is_serial_connected = lambda: True
        rotary = AxisProfile(mode=MODE_ROTARY)
        app.axis_profiles[0] = rotary
        commands = []
        app._send_and_read = lambda command, **_kwargs: (
            commands.append(command) or "ACK,0"
        )

        accepted = StepperGUI._send_mm(
            app,
            0,
            _continuous_burst_units(rotary),
            DIR_OUTWARD,
            rotary.delay_ms_for_speed(0.3),
            profile=rotary,
        )

        self.assertTrue(accepted)
        self.assertEqual(commands, ["MOVE,0,1,1,6000000"])

    def test_compatibility_launcher_preserves_historical_constants(self):
        import pc_gui

        expected_names = (
            "PULSES_PER_MM",
            "DELAY_DEFAULT_MS",
            "DIR_INVERT",
            "CONTINUOUS_BURST_MM",
            "FOC_STATE_NAMES",
            "GEAR_STATE_NAMES",
            "FOC_POLL_INTERVAL_S",
            "CALIB_FILE",
            "FOC_TUNE_FILE",
            "GEAR_TUNE_FILE",
            "LOG_DIR",
        )
        for name in expected_names:
            with self.subTest(name=name):
                self.assertTrue(hasattr(pc_gui, name))
        self.assertEqual(Path(pc_gui.PROJECT_DIR), Path(__file__).parents[1])
        self.assertEqual(Path(pc_gui.CALIB_FILE).name, ".stepper_calib.json")

    def test_remote_move_is_clamped_to_rp2040_delay_and_tracks_steps(self):
        app = _headless_app()
        commands = []
        app._is_serial_connected = lambda: True

        def send(command, **_kwargs):
            commands.append(command)
            return "ACK,6"

        app._send_and_read = send
        accepted = StepperGUI._send_pulses(
            app, axis=6, steps=100, direction=DIR_OUTWARD, delay_ms=0.05)

        self.assertTrue(accepted)
        self.assertEqual(commands, ["MOVE,6,100,1,100"])
        self.assertEqual(app._pending_step[6], 100)
        self.assertTrue(app.stepper_in_progress[6])

    def test_done_applies_reported_fraction_in_canonical_steps(self):
        app = _headless_app()
        app._pending_step[0] = 200
        app.stepper_in_progress[0] = True

        StepperGUI._on_step_done(app, 0, executed_steps=50, requested_steps=200)

        self.assertEqual(app.axis_runtime[0].position_steps, 50)
        self.assertAlmostEqual(
            app.axis_runtime[0].position_in(app.axis_profiles[0]), 0.25)
        self.assertIsNone(app._pending_step[0])
        self.assertFalse(app.stepper_in_progress[0])

    def test_progress_projects_without_committing_position(self):
        app = _headless_app()
        app._pending_step[0] = 200
        app.stepper_in_progress[0] = True
        app.axis_motion_telemetry[0] = AxisMotionTelemetry.starting(
            0, 200, now=10.0
        )

        StepperGUI._on_step_progress(app, 0, 50, 200)

        self.assertEqual(app.axis_runtime[0].position_steps, 0)
        self.assertEqual(app.axis_motion_telemetry[0].executed_steps, 50)
        self.assertEqual(app.axis_motion_telemetry[0].requested_steps, 200)

    def test_binding_save_is_atomic_and_releases_all_axis_reservations(self):
        app = _headless_app()
        app.axis_profiles[2] = AxisProfile(mode=MODE_ROTARY)
        app.axis_profiles[3] = AxisProfile(mode=MODE_ROTARY)
        saved = []

        class Store:
            def save_coordinated_bindings(self, document):
                saved.append(document)

        app.state_store = Store()
        applied = StepperGUI._apply_control_bindings(
            app, BindingSet.suggested()
        )

        self.assertEqual(applied.revision, 1)
        self.assertIs(app.control_bindings, applied)
        self.assertEqual(saved, [applied.as_document()])
        self.assertTrue(
            all(app._move_reservation[axis] is None for axis in (0, 1, 2, 3))
        )

    def test_binding_save_rejects_busy_or_mode_mismatched_axis(self):
        app = _headless_app()
        app.axis_profiles[2] = AxisProfile(mode=MODE_ROTARY)
        app.axis_profiles[3] = AxisProfile(mode=MODE_ROTARY)

        class Store:
            def save_coordinated_bindings(self, _document):
                raise AssertionError("invalid binding must not be persisted")

        app.state_store = Store()
        app.running[0] = True
        with self.assertRaisesRegex(RuntimeError, "正在运动或已预约"):
            StepperGUI._apply_control_bindings(app, BindingSet.suggested())
        app.running[0] = False
        app.axis_profiles[2] = AxisProfile()
        with self.assertRaises(BindingValidationError):
            StepperGUI._apply_control_bindings(app, BindingSet.suggested())

    def test_abort_applies_partial_steps_and_marks_position_untrusted(self):
        app = _headless_app()
        app._pending_step[0] = -200
        app.stepper_in_progress[0] = True
        app.running[0] = True

        StepperGUI._on_step_aborted(app, 0, executed_steps=25, requested_steps=100)

        self.assertEqual(app.axis_runtime[0].position_steps, -50)
        self.assertFalse(app.axis_runtime[0].position_trusted)
        self.assertFalse(app.running[0])
        self.assertIn("-0.250 mm", app._logs[-1])

    def test_done_before_ack_settles_provisional_move_once(self):
        app = _headless_app()
        app._is_serial_connected = lambda: True

        def done_then_ack(_command, **_kwargs):
            StepperGUI._on_step_done(
                app, 0, executed_steps=100, requested_steps=100
            )
            return "ACK,0"

        app._send_and_read = done_then_ack
        accepted = StepperGUI._send_pulses(
            app, axis=0, steps=100, direction=DIR_OUTWARD, delay_ms=1.0
        )

        self.assertTrue(accepted)
        self.assertEqual(app.axis_runtime[0].position_steps, 100)
        self.assertIsNone(app._pending_step[0])
        self.assertFalse(app.stepper_in_progress[0])

    def test_progress_is_projected_but_done_commits_position_only_once(self):
        app = _headless_app()
        app.root = types.SimpleNamespace(after=lambda *_args, **_kwargs: None)
        app._pending_step[0] = 200
        app.stepper_in_progress[0] = True
        app.axis_motion_telemetry[0] = AxisMotionTelemetry.starting(1, 200)

        StepperGUI._on_step_progress(app, 0, 50, 200)
        self.assertEqual(app.axis_runtime[0].position_steps, 0)
        self.assertEqual(app.axis_motion_telemetry[0].executed_steps, 50)

        StepperGUI._on_step_done(app, 0, executed_steps=200, requested_steps=200)
        StepperGUI._on_step_done(app, 0, executed_steps=200, requested_steps=200)

        self.assertEqual(app.axis_runtime[0].position_steps, 200)
        self.assertEqual(app.axis_motion_telemetry[0].state, "IDLE")
        self.assertEqual(app.axis_motion_telemetry[0].last_result, "DONE")
        self.assertIsNone(app._pending_step[0])

    def test_out_of_range_progress_event_is_ignored(self):
        app = _headless_app()

        StepperGUI._on_step_progress(app, NUM_STEPPER_AXES, 10, 100)

        self.assertTrue(
            all(item == AxisMotionTelemetry() for item in app.axis_motion_telemetry)
        )

    def test_move_timeout_marks_position_untrusted(self):
        app = _headless_app()
        app._is_serial_connected = lambda: True

        def timeout(command, **_kwargs):
            raise RequestTimeout(command, 1.0)

        app._send_and_read = timeout
        accepted = StepperGUI._send_pulses(
            app, axis=0, steps=100, direction=DIR_OUTWARD, delay_ms=1.0
        )

        self.assertFalse(accepted)
        self.assertFalse(app.axis_runtime[0].position_trusted)
        self.assertIsNone(app._pending_step[0])
        self.assertFalse(app.stepper_in_progress[0])

    def test_second_move_is_not_queued_behind_stale_range_preflight(self):
        app = _headless_app()
        app._is_serial_connected = lambda: True
        app.axis_runtime[0].max_steps = 3_000
        app._pending_step[0] = 2_000
        app.stepper_in_progress[0] = True
        commands = []
        app._send_and_read = lambda command, **_kwargs: commands.append(command)

        accepted = StepperGUI._send_mm(
            app,
            axis=0,
            distance_mm=10.0,
            direction=DIR_OUTWARD,
            delay_ms=1.0,
            profile=app.axis_profiles[0],
        )

        self.assertFalse(accepted)
        self.assertEqual(commands, [])
        self.assertEqual(app._pending_step[0], 2_000)

    def test_move_reservation_cannot_be_stolen_by_an_unrelated_worker(self):
        app = _headless_app()
        app._is_serial_connected = lambda: True
        commands = []
        app._send_and_read = lambda command, **_kwargs: (
            commands.append(command) or "ACK,0"
        )
        reservation = object()
        app._move_reservation[0] = reservation

        self.assertFalse(
            StepperGUI._send_mm(
                app, 0, 1.0, DIR_OUTWARD, 1.0,
                profile=app.axis_profiles[0],
            )
        )
        self.assertEqual(commands, [])

        self.assertTrue(
            StepperGUI._send_mm(
                app, 0, 1.0, DIR_OUTWARD, 1.0,
                profile=app.axis_profiles[0], reservation=reservation,
            )
        )
        self.assertEqual(commands, ["MOVE,0,200,1,1000"])

    def test_position_calibration_is_rejected_while_move_is_reserved(self):
        app = _headless_app()
        app.axis_runtime[0].position_steps = 500
        app._move_reservation[0] = object()

        with patch("motor_control.desktop_app.messagebox.showwarning") as warning:
            StepperGUI.set_home(app, 0)

        self.assertEqual(app.axis_runtime[0].position_steps, 500)
        self.assertTrue(app.axis_runtime[0].position_trusted)
        warning.assert_called_once()

    def test_terminal_callback_from_old_session_cannot_settle_new_move(self):
        app = _headless_app()
        old_session = object()
        new_session = object()
        app._serial_generation = 1
        app.serial_session = old_session
        queued = []
        app._post_ui = queued.append

        StepperGUI._handle_serial_event(
            app,
            StepTerminal("STEP,0,DONE,100,100", 0, StepResult.DONE, 100, 100),
            1,
            old_session,
        )
        self.assertEqual(len(queued), 1)

        app._serial_generation = 2
        app.serial_session = new_session
        app._pending_step[0] = 200
        app.stepper_in_progress[0] = True
        queued[0]()

        self.assertEqual(app.axis_runtime[0].position_steps, 0)
        self.assertEqual(app._pending_step[0], 200)
        self.assertTrue(app.stepper_in_progress[0])

    def test_exception_from_old_request_cannot_disconnect_new_session(self):
        app = _headless_app()
        app._serial_generation = 1
        app._control_generation = 1
        app._control_context = threading.local()
        app._closing = False
        app._disconnecting = False
        failure_calls = []
        new_session = object()

        class OldSession:
            is_open = True

            def request_line(self, _command, **_kwargs):
                app._serial_generation = 2
                app.serial_session = new_session
                raise SessionClosed("old session closed")

        old_session = OldSession()
        app.serial_session = old_session
        app._handle_serial_failure = lambda *args: failure_calls.append(args)

        response = StepperGUI._send_and_read(app, "MODE")

        self.assertEqual(response, "")
        self.assertEqual(failure_calls, [])
        self.assertIs(app.serial_session, new_session)


if __name__ == "__main__":
    unittest.main()
