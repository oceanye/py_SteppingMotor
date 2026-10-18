"""Diagnostics tests: temporary files and fake serial ONLY, no motors/network."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_serial_session import FakeSerial
from test_x42s_pc import _x42s_gait_app
from motor_control import LogicalRole
from motor_control.diagnostics import (
    DiagnosticRecorder, SCHEMA, install_exception_hooks, logged_action,
    operation_id, operation_scope, sanitize,
)
from motor_control.diagnostic_export import export_bundle
from motor_control.protocol import parse_line
from motor_control.serial_session import SerialSession


def records(recorder, channel):
    recorder.flush()
    result = []
    for path in recorder.directory.glob(channel + ".jsonl*"):
        result.extend(json.loads(line) for line in path.read_text(encoding="utf-8").splitlines())
    return sorted(result, key=lambda row: row["seq"])


class DiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.recorder = DiagnosticRecorder(self.root, metadata={"origin": "offline-test"})

    def tearDown(self):
        self.recorder.close()
        self.tmp.cleanup()

    def test_records_have_session_clock_and_operation(self):
        with operation_scope("op-test"):
            self.recorder.event("motor", "sample", axis=2)
        record = records(self.recorder, "motor")[0]
        self.assertEqual(record["operation_id"], "op-test")
        self.assertEqual(record["schema"], SCHEMA)
        self.assertEqual(record["session_id"], self.recorder.session_id)
        self.assertIn("+00:00", record["utc"])
        self.assertGreaterEqual(record["elapsed_ms"], 0)
        self.assertEqual(record["data"]["axis"], 2)

    def test_cross_thread_producers_have_unique_sequence(self):
        threads = [threading.Thread(target=lambda: [self.recorder.event("motor", "sample")
                                                   for _ in range(40)]) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        rows = records(self.recorder, "motor")
        self.assertEqual(len(rows), 160)
        self.assertEqual(len({r["seq"] for r in rows}), 160)

    def test_secrets_paths_and_network_identifiers_are_redacted(self):
        payload = {"token": "do-not-upload", "password": "very-private",
                   "message": r"C:\Users\private-person\repo http://user:pass@10.2.3.4/ Bearer abcdef123 user@example.com api_key=abcdef"}
        text = json.dumps(sanitize(payload))
        for value in ("do-not-upload", "very-private", "private-person", "user:pass",
                      "10.2.3.4", "abcdef123", "user@example.com", "abcdef"):
            self.assertNotIn(value, text)
        self.assertIn("<redacted>", text)

    def test_nonfinite_and_unknown_values_are_json_safe(self):
        self.recorder.event("telemetry", "sample", measured=None, invalid=float("nan"))
        row = records(self.recorder, "telemetry")[0]["data"]
        self.assertIsNone(row["measured"])
        self.assertEqual(row["invalid"], "nan")

    def test_rotation_bounds_channel_files(self):
        self.recorder.close()
        self.recorder = DiagnosticRecorder(self.root, max_bytes=700, backups=1)
        for index in range(30):
            self.recorder.event("motor", "sample", index=index, text="x"*100)
        self.recorder.flush()
        paths = list(self.recorder.directory.glob("motor.jsonl*"))
        self.assertEqual(len(paths), 2)
        self.assertLessEqual(sum(p.stat().st_size for p in paths), 1400)

    def test_overflow_counted_without_blocking_control(self):
        self.recorder.close()
        self.recorder = DiagnosticRecorder(self.root, queue_size=2)
        self.recorder.flush()
        entered, release = threading.Event(), threading.Event()
        def blocked(_record):
            entered.set()
            release.wait(2)
        with patch.object(self.recorder._handlers["motor"], "emit", side_effect=blocked):
            self.recorder.event("motor", "block")
            self.assertTrue(entered.wait(1))
            for _ in range(20):
                self.recorder.event("motor", "sample")
            self.assertGreater(self.recorder.health()["dropped"], 0)
            release.set()
            self.assertTrue(self.recorder.flush())

    def test_disk_failure_visible_and_other_channel_survives(self):
        with patch.object(self.recorder._handlers["motor"], "emit", side_effect=OSError("disk full")):
            self.recorder.event("motor", "sample")
            self.recorder.flush()
        self.assertEqual(self.recorder.health()["write_errors"], 1)
        self.recorder.event("gui", "still.alive")
        self.assertEqual(records(self.recorder, "gui")[-1]["event"], "still.alive")

    def test_action_decorator_preserves_return_and_exception(self):
        class App:
            diagnostics = self.recorder
            @logged_action
            def action(self, fail=False):
                if fail:
                    raise ValueError("invalid angle")
                return 42
        self.assertEqual(App().action(), 42)
        with self.assertRaisesRegex(ValueError, "invalid angle"):
            App().action(True)
        rows = records(self.recorder, "gui")
        error = next(row for row in rows if row["event"] == "action.error")
        self.assertIn("ValueError", error["data"]["traceback"])
        self.assertIsNone(operation_id())

    def test_hooks_capture_and_preserve_existing_callbacks(self):
        previous_sys, previous_thread = sys.excepthook, threading.excepthook
        original = Mock()
        root = SimpleNamespace(report_callback_exception=original)
        try:
            with patch.object(sys, "excepthook", Mock()) as previous:
                with patch.object(threading, "excepthook", Mock()) as thread_previous:
                    install_exception_hooks(self.recorder)
                    error = ValueError("callback failed")
                    sys.excepthook(ValueError, error, None)
                    threading.excepthook(SimpleNamespace(exc_value=error, exc_traceback=None,
                                                         thread=threading.current_thread()))
                    previous.assert_called_once()
                    thread_previous.assert_called_once()
            install_exception_hooks(self.recorder, root)
            root.report_callback_exception(ValueError, error, None)
            original.assert_called_once()
        finally:
            sys.excepthook, threading.excepthook = previous_sys, previous_thread
        events = {r["event"] for r in records(self.recorder, "gui")}
        self.assertTrue({"exception.process", "exception.thread", "exception.tk"} <= events)

    def test_serial_request_reply_and_async_events_correlate(self):
        port = FakeSerial()
        port.on_write = lambda _data: port.feed("X42S,3,OFFLINE", "ACK,X42S,2,MOVE,100,50")
        with SerialSession(port, diagnostics=self.recorder) as session:
            with operation_scope("jog-123"):
                reply = session.request("X42S,2,MOVE,100,50")
        self.assertEqual(reply.steps, 100)
        rows = records(self.recorder, "motor")
        relevant = [r for r in rows if r["event"] in ("request.queued", "serial.tx_written", "reply.matched", "request.completed")]
        self.assertEqual(len(relevant), 4)
        self.assertEqual({r["operation_id"] for r in relevant}, {"jog-123"})
        self.assertEqual(len({r["data"]["request_id"] for r in relevant}), 1)
        async_row = next(r for r in rows if r["event"] == "serial.event")
        self.assertIsNone(async_row["data"]["request_id"])

    def test_soft_timeout_and_cancel_are_recorded(self):
        port = FakeSerial()
        with SerialSession(port, diagnostics=self.recorder) as session:
            self.assertIsNone(session.request("X42S,S,2", timeout=.01, soft_timeout=True))
            from motor_control.serial_session import RequestCancelled
            with self.assertRaises(RequestCancelled):
                session.request("X42S,S,2", guard=lambda:False)
        events = {r["event"] for r in records(self.recorder, "motor")}
        self.assertTrue({"request.timeout", "request.cancelled"} <= events)

    def test_closed_session_rejection_has_request_identity(self):
        from motor_control.serial_session import SessionClosed
        session = SerialSession(FakeSerial(), diagnostics=self.recorder)
        session.close()
        with self.assertRaises(SessionClosed):
            session.request("X42S,S,2")
        row = next(r for r in records(self.recorder, "motor") if r["event"] == "request.error")
        self.assertEqual(row["data"]["phase"], "before_send")
        self.assertTrue(row["data"]["request_id"])

    def test_failed_write_is_not_reported_as_written(self):
        from motor_control.serial_session import SerialIoError
        port = FakeSerial()
        with SerialSession(port, diagnostics=self.recorder) as session:
            with patch.object(port, "write", side_effect=OSError("unplugged")):
                with self.assertRaises(SerialIoError):
                    session.request("X42S,S,2")
        rows = records(self.recorder, "motor")
        self.assertFalse(any(r["event"] == "serial.tx_written" for r in rows))
        error = next(r for r in rows if r["event"] == "request.error")
        self.assertEqual(error["data"]["phase"], "write")

    def test_snapshot_does_not_read_foreign_files(self):
        (self.recorder.directory / "private.txt").write_text("do not read")
        self.assertNotIn("private.txt", self.recorder.snapshot_files())

    def test_x42s_records_raw_and_role_units_and_unknowns(self):
        app = _x42s_gait_app()
        app.diagnostics = self.recorder
        app._apply_x42s_reply(LogicalRole.MR1, 2,
                             parse_line("X42S,2,S,1,1,1,none,220,215,5,0,0,25,8,0,7"))
        data = records(self.recorder, "telemetry")[0]["data"]
        self.assertEqual(data["motor_deg"], 215)
        self.assertEqual(data["output_position"], 10)
        self.assertIsNone(data["speed_motor_rpm"])
        self.assertEqual(data["age_ms"], 8)

    def test_bundle_redacts_legacy_and_contains_csv_not_config(self):
        self.recorder.event("telemetry", "x42s.telemetry", role="Mr1", driver_id=2,
                            motor_deg=215, output_position=10, unit="deg")
        (self.root / ".coordinated_bindings.json").write_text('{"token":"not-for-upload"}')
        (self.root / "logs" / "gui_stderr.log").write_text("token=private123 100.10.2.3", encoding="utf-8")
        (self.root / "logs" / "secret.json").write_text("private-other", encoding="utf-8")
        with patch("motor_control.diagnostic_export.environment_summary", return_value={"kind":"offline-test"}):
            output = export_bundle(self.root, recorder=self.recorder)
        with zipfile.ZipFile(output) as archive:
            all_text = "\n".join(archive.read(name).decode("utf-8-sig") for name in archive.namelist())
            for secret in ("not-for-upload", "private123", "100.10.2.3", "private-other"):
                self.assertNotIn(secret, all_text)
            manifest = json.loads(archive.read("manifest.json"))
            self.assertEqual(manifest["telemetry_rows"], 1)
            self.assertIn("output_position", archive.read("telemetry.csv").decode("utf-8-sig"))
            self.assertFalse(any(".coordinated" in n for n in archive.namelist()))

    def test_bundle_refuses_overwrite(self):
        output = self.root / "existing.zip"
        output.write_bytes(b"preserve")
        with patch("motor_control.diagnostic_export.environment_summary", return_value={}):
            with self.assertRaises(FileExistsError):
                export_bundle(self.root, output, recorder=self.recorder)
        self.assertEqual(output.read_bytes(), b"preserve")

    def test_close_drains_and_marks_manifest(self):
        self.recorder.event("motor", "final")
        self.recorder.close()
        meta = json.loads((self.recorder.directory / "session.json").read_text(encoding="utf-8"))
        self.assertTrue(meta["closed"])
        self.assertEqual(records(self.recorder, "motor")[-1]["event"], "final")

    def test_ipv6_and_basic_credentials_redacted_but_clock_preserved(self):
        text = sanitize("http://private-token@server/ ::1 2001:db8::1 14:20:50 Basic abcdef==")
        for value in ("private-token", "::1", "2001:db8", "abcdef"):
            self.assertNotIn(value, text)
        self.assertIn("14:20:50", text)

    def test_retention_does_not_remove_foreign_files(self):
        self.recorder.close()
        first = self.recorder.directory
        (first / "user-data.txt").write_text("preserve")
        second = DiagnosticRecorder(self.root)
        second.close()
        self.recorder = DiagnosticRecorder(self.root, keep_sessions=0)
        self.assertTrue((first / "user-data.txt").is_file())
        self.assertFalse(second.directory.exists())

    def test_worker_inherits_action_and_exceptions_are_diagnosable(self):
        app = _x42s_gait_app()
        app.diagnostics = self.recorder
        app._control_generation = 1
        app._control_context = threading.local()
        for key in ("_estop_in_progress", "_hardware_estop_active", "_estop_unconfirmed", "_closing", "_disconnecting"):
            setattr(app, key, False)
        def fail():
            raise ValueError("test worker failure")
        with patch.object(threading, "excepthook", Mock()):
            with operation_scope("worker-op"):
                worker = app._start_control_worker(fail)
            worker.join(2)
        row = next(r for r in records(self.recorder, "motor") if r["event"] == "worker.error")
        self.assertEqual(row["operation_id"], "worker-op")
        self.assertIn("test worker failure", row["data"]["traceback"])

    def test_empty_telemetry_bundle_is_valid(self):
        with patch("motor_control.diagnostic_export.environment_summary", return_value={}):
            output = export_bundle(self.root, recorder=self.recorder)
        with zipfile.ZipFile(output) as archive:
            self.assertEqual(json.loads(archive.read("manifest.json"))["telemetry_rows"], 0)

    def test_export_reports_tail_truncation_and_budget_omissions(self):
        path = self.root / "logs" / "gui_stderr.log"
        path.write_text("line\n" * 1000, encoding="utf-8")
        with patch("motor_control.diagnostic_export.environment_summary", return_value={}):
            output = export_bundle(self.root, recorder=self.recorder, max_file_bytes=100)
        with zipfile.ZipFile(output) as archive:
            manifest = json.loads(archive.read("manifest.json"))
            self.assertIn("legacy/gui_stderr.log", manifest["truncated_files"])
        with patch("motor_control.diagnostic_export.environment_summary", return_value={}):
            output = export_bundle(self.root, recorder=self.recorder, max_total_bytes=0)
        with zipfile.ZipFile(output) as archive:
            self.assertTrue(json.loads(archive.read("manifest.json"))["omitted_files"])


if __name__ == "__main__":
    unittest.main()
