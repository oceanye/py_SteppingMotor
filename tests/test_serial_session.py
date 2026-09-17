import queue
import threading
import time
import unittest

from motor_control.protocol import (
    AckReply,
    HardwareEstopEvent,
    ModeReply,
    NodeOfflineEvent,
    NodeStatusReply,
    OkReply,
    StepProgress,
    StepResult,
    StepTerminal,
    UnknownMessage,
)
from motor_control.serial_session import (
    RequestCancelled,
    RequestTimeout,
    SerialSession,
    SessionDesynchronized,
)


class FakeSerial:
    """Small timeout-based serial double; callbacks model firmware replies."""

    def __init__(self):
        self.is_open = True
        self.writes = []
        self.on_write = None
        self._received = queue.Queue()
        self._write_condition = threading.Condition()

    def readline(self):
        try:
            return self._received.get(timeout=0.02)
        except queue.Empty:
            return b""

    def write(self, data):
        if not self.is_open:
            raise OSError("closed")
        with self._write_condition:
            self.writes.append(data)
            self._write_condition.notify_all()
        if self.on_write is not None:
            self.on_write(data)
        return len(data)

    def close(self):
        self.is_open = False
        self._received.put(b"")

    def feed(self, *lines):
        for line in lines:
            data = line if isinstance(line, bytes) else line.encode("utf-8") + b"\r\n"
            self._received.put(data)

    def wait_for_writes(self, count, timeout=1.0):
        deadline = time.monotonic() + timeout
        with self._write_condition:
            while len(self.writes) < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._write_condition.wait(remaining)
        return True


class SerialSessionTests(unittest.TestCase):
    def setUp(self):
        self.port = FakeSerial()
        self.session = SerialSession(self.port)
        self.session.start()

    def tearDown(self):
        self.session.close()

    def test_step_and_node_events_are_split_from_move_reply(self):
        def respond(_data):
            self.port.feed(
                "STEP,6,P,20,100",
                "NODE,2,OFFLINE",
                "ACK,6",
            )

        self.port.on_write = respond
        reply = self.session.request("MOVE,6,100,1,100", timeout=0.5)

        self.assertEqual(reply, AckReply("ACK,6", 6))
        self.assertEqual(self.session.next_event(0.5), StepProgress("STEP,6,P,20,100", 6, 20, 100))
        self.assertEqual(self.session.next_event(0.5), NodeOfflineEvent("NODE,2,OFFLINE", 2))
        self.assertEqual(self.port.writes, [b"MOVE,6,100,1,100\n"])

    def test_hardware_estop_event_does_not_consume_estop_reply(self):
        self.port.on_write = lambda _data: self.port.feed("HWESTOP,TRIGGERED", "OK,ESTOP")

        reply = self.session.request("ESTOP", timeout=0.5)

        self.assertEqual(reply, OkReply("OK,ESTOP", ("ESTOP",)))
        self.assertIsInstance(self.session.next_event(0.5), HardwareEstopEvent)

    def test_done_and_abort_events_do_not_consume_stop_reply(self):
        self.port.on_write = lambda _data: self.port.feed(
            "STEP,0,DONE,10,10",
            "STEP,1,ABORT,4,10",
            "OK,0",
        )

        reply = self.session.request("STOP,0", timeout=0.5)

        self.assertEqual(reply, OkReply("OK,0", ("0",)))
        done = self.session.next_event(0.5)
        abort = self.session.next_event(0.5)
        self.assertEqual((done.result, abort.result), (StepResult.DONE, StepResult.ABORT))
        self.assertIsInstance(done, StepTerminal)
        self.assertIsInstance(abort, StepTerminal)

    def test_node_offline_event_is_separate_from_requested_node_status(self):
        self.port.on_write = lambda _data: self.port.feed(
            "NODE,2,OFFLINE",
            "NODE,2,S,OFFLINE,0,0",
        )

        reply = self.session.request("NODE,2,S", timeout=0.5)

        self.assertEqual(reply, NodeStatusReply("NODE,2,S,OFFLINE,0,0", 2, False, 0, 0))
        self.assertEqual(self.session.next_event(0.5), NodeOfflineEvent("NODE,2,OFFLINE", 2))

    def test_unknown_and_nonmatching_known_messages_are_unsolicited(self):
        self.port.on_write = lambda _data: self.port.feed(
            "ESP32 Controller Ready",
            "ACK,0",
            "MODE,GEAR",
        )

        reply = self.session.request("MODE", timeout=0.5)

        self.assertIsInstance(reply, ModeReply)
        self.assertIsInstance(self.session.next_unsolicited(0.5), UnknownMessage)
        self.assertEqual(self.session.next_unsolicited(0.5), AckReply("ACK,0", 0))

    def test_unknown_text_alone_cannot_complete_request(self):
        self.port.on_write = lambda _data: self.port.feed("NUM_AXES = 6")

        with self.assertRaises(RequestTimeout):
            self.session.request("MODE", timeout=0.05)
        self.assertIsInstance(self.session.next_unsolicited(0.5), UnknownMessage)

    def test_timeout_retires_stream_before_late_same_shape_reply(self):
        with self.assertRaises(RequestTimeout):
            self.session.request("FOC,0,PA,5", timeout=0.02)

        self.assertFalse(self.session.is_open)
        self.port.feed("OK,0")
        with self.assertRaises(SessionDesynchronized):
            self.session.request("FOC,0,A,60", timeout=0.1)
        self.assertEqual(self.port.writes, [b"FOC,0,PA,5\n"])

    def test_concurrent_requests_are_serialized_for_full_transactions(self):
        results = {}
        errors = []

        def call(name, command):
            try:
                results[name] = self.session.request(command, timeout=1.0)
            except Exception as exc:  # pragma: no cover - asserted empty below
                errors.append(exc)

        first = threading.Thread(target=call, args=("move", "MOVE,0,10,1,100"))
        second = threading.Thread(target=call, args=("mode", "MODE"))
        first.start()
        self.assertTrue(self.port.wait_for_writes(1))
        second.start()

        time.sleep(0.05)
        self.assertEqual(self.port.writes, [b"MOVE,0,10,1,100\n"])

        self.port.feed("ACK,0")
        self.assertTrue(self.port.wait_for_writes(2))
        self.assertEqual(self.port.writes[1], b"MODE\n")
        self.port.feed("MODE,FOC")

        first.join(1.0)
        second.join(1.0)
        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(errors, [])
        self.assertIsInstance(results["move"], AckReply)
        self.assertIsInstance(results["mode"], ModeReply)

    def test_callbacks_receive_events_without_stealing_queue_delivery(self):
        observed = []
        delivered = threading.Event()

        def handler(event):
            observed.append(event)
            delivered.set()

        self.session.add_event_handler(handler)
        self.port.feed("NODE,5,OFFLINE")

        self.assertTrue(delivered.wait(0.5))
        queued = self.session.next_event(0.5)
        self.assertEqual(observed, [queued])

    def test_guard_cancels_before_any_bytes_are_sent(self):
        with self.assertRaises(RequestCancelled):
            self.session.request("MODE", guard=lambda: False)
        self.assertEqual(self.port.writes, [])


class SoftTimeoutTests(unittest.TestCase):
    """幂等只读轮询的软超时：跳过本轮而不退役会话。"""

    def setUp(self):
        self.port = FakeSerial()
        self.session = SerialSession(self.port)
        self.session.start()

    def tearDown(self):
        self.session.close()

    def test_soft_timeout_returns_none_and_keeps_session_open(self):
        self.assertIsNone(self.session.request("MODE", timeout=0.02,
                                               soft_timeout=True))
        self.assertTrue(self.session.is_open)

        self.port.on_write = lambda _data: self.port.feed("MODE,FOC")
        reply = self.session.request("MODE", timeout=0.5)
        self.assertEqual(reply.raw, "MODE,FOC")
        self.assertEqual(reply.mode.value, "FOC")

    def test_late_reply_goes_unsolicited_instead_of_poisoning_next(self):
        self.assertIsNone(self.session.request("MODE", timeout=0.02,
                                               soft_timeout=True))
        # 上一个 MODE 的迟到回复不能吞掉本请求；它自己的回复才算数。
        self.port.on_write = lambda _data: self.port.feed("MODE,GEAR")
        reply = self.session.request("MODE", timeout=0.5)
        self.assertEqual(reply.raw, "MODE,GEAR")
        self.port.feed("MODE,FOC")  # 迟到的第一轮回复
        late = self.session.next_unsolicited(0.5)
        self.assertEqual(late.raw, "MODE,FOC")
        self.assertTrue(self.session.is_open)

    def test_late_reply_satisfies_next_poll_of_same_command(self):
        # 同命令轮询场景：迟到的上一轮回复直接满足下一轮，内容幂等等价。
        self.assertIsNone(self.session.request("MODE", timeout=0.02,
                                               soft_timeout=True))
        self.port.feed("MODE,GEAR")
        reply = self.session.request("MODE", timeout=0.5)
        self.assertEqual(reply.raw, "MODE,GEAR")
        self.assertTrue(self.session.is_open)


if __name__ == "__main__":
    unittest.main()
