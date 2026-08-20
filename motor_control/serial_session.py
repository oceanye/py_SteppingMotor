"""Thread-safe ownership of the ESP32 text serial session.

``SerialSession`` is intentionally independent of pyserial at construction
time: any object implementing ``readline()``, ``write()`` and ``close()`` can
be injected.  Production code may use :meth:`SerialSession.open`, while tests
can supply a deterministic fake port.

There is exactly one reader thread.  Synchronous requests are serialized and
known asynchronous events are delivered to their own queue/callbacks, so an
interleaved ``STEP,...`` frame cannot be mistaken for a command response.
"""

from __future__ import annotations

from dataclasses import dataclass
import queue
import threading
from typing import Callable, Optional, Protocol, Union, runtime_checkable

from .protocol import (
    AsyncEvent,
    ProtocolMessage,
    Reply,
    ReplyMatcher,
    UnknownMessage,
    encode_line,
    parse_line,
    reply_matcher_for,
)


@runtime_checkable
class SerialPort(Protocol):
    """The small pyserial-compatible surface owned by this session."""

    is_open: bool

    def readline(self) -> bytes: ...

    def write(self, data: bytes) -> object: ...

    def close(self) -> None: ...


class SerialSessionError(RuntimeError):
    pass


class SessionNotRunning(SerialSessionError):
    pass


class SessionClosed(SerialSessionError):
    pass


class RequestTimeout(TimeoutError, SerialSessionError):
    def __init__(self, command: str, timeout: float):
        super().__init__(f"no matching reply for {command!r} within {timeout:.3f}s")
        self.command = command
        self.timeout = timeout


class RequestCancelled(SerialSessionError):
    """Raised when a pre-send guard rejects a queued request."""


class SessionDesynchronized(SerialSessionError):
    """The reply stream can no longer be correlated safely with requests."""


class SerialIoError(SerialSessionError):
    pass


EventHandler = Callable[[AsyncEvent], None]
UnsolicitedHandler = Callable[[ProtocolMessage], None]
ErrorHandler = Callable[[BaseException], None]
TraceHandler = Callable[[str, str], None]


@dataclass(slots=True)
class _PendingRequest:
    matcher: ReplyMatcher
    completed: threading.Event
    reply: Optional[Reply] = None
    error: Optional[BaseException] = None


class SerialSession:
    """Own a serial port, one reader, and at most one in-flight request."""

    def __init__(
        self,
        serial_port: SerialPort,
        *,
        event_handler: Optional[EventHandler] = None,
        unsolicited_handler: Optional[UnsolicitedHandler] = None,
        error_handler: Optional[ErrorHandler] = None,
        trace_handler: Optional[TraceHandler] = None,
        reader_name: str = "motor-serial-reader",
    ) -> None:
        self._serial = serial_port
        self._event_handlers: list[EventHandler] = []
        if event_handler is not None:
            self._event_handlers.append(event_handler)
        self._unsolicited_handlers: list[UnsolicitedHandler] = []
        if unsolicited_handler is not None:
            self._unsolicited_handlers.append(unsolicited_handler)
        self._error_handler = error_handler
        self._trace_handler = trace_handler
        self._reader_name = reader_name

        self._events: queue.Queue[AsyncEvent] = queue.Queue()
        self._unsolicited: queue.Queue[ProtocolMessage] = queue.Queue()
        self._request_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._running = threading.Event()
        self._closed = False
        self._reader: Optional[threading.Thread] = None
        self._pending: Optional[_PendingRequest] = None
        self._failure: Optional[BaseException] = None

    @classmethod
    def open(
        cls,
        port: str,
        baudrate: int = 115_200,
        timeout: float = 0.2,
        **kwargs: object,
    ) -> "SerialSession":
        """Open a pyserial port and immediately start its sole reader thread."""

        import serial  # Imported lazily so protocol-only tests need no pyserial.

        serial_port = serial.Serial(port, baudrate, timeout=timeout)
        session = cls(serial_port, **kwargs)
        session.start()
        return session

    @property
    def is_running(self) -> bool:
        return self._running.is_set() and not self._closed

    @property
    def is_open(self) -> bool:
        return self.is_running and bool(getattr(self._serial, "is_open", True))

    @property
    def failure(self) -> Optional[BaseException]:
        with self._state_lock:
            return self._failure

    def start(self) -> None:
        with self._state_lock:
            if self._closed:
                raise SessionClosed("a closed serial session cannot be restarted")
            if self._running.is_set():
                return
            if not bool(getattr(self._serial, "is_open", True)):
                raise SessionClosed("serial port is not open")
            self._running.set()
            self._reader = threading.Thread(
                target=self._reader_loop, name=self._reader_name, daemon=True
            )
            self._reader.start()

    def close(self, join_timeout: float = 1.0) -> None:
        with self._state_lock:
            if self._closed:
                return
            self._closed = True
            self._running.clear()
            pending = self._pending
            self._pending = None
            if pending is not None:
                pending.error = SessionClosed("serial session closed")
                pending.completed.set()
            reader = self._reader
        try:
            self._serial.close()
        except Exception as exc:
            self._notify_error(SerialIoError(f"serial close failed: {exc}"))
        if reader is not None and reader is not threading.current_thread():
            reader.join(max(0.0, join_timeout))

    def __enter__(self) -> "SerialSession":
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def add_event_handler(self, handler: EventHandler) -> None:
        with self._state_lock:
            if handler not in self._event_handlers:
                self._event_handlers.append(handler)

    def remove_event_handler(self, handler: EventHandler) -> None:
        with self._state_lock:
            if handler in self._event_handlers:
                self._event_handlers.remove(handler)

    def add_unsolicited_handler(self, handler: UnsolicitedHandler) -> None:
        with self._state_lock:
            if handler not in self._unsolicited_handlers:
                self._unsolicited_handlers.append(handler)

    def remove_unsolicited_handler(self, handler: UnsolicitedHandler) -> None:
        with self._state_lock:
            if handler in self._unsolicited_handlers:
                self._unsolicited_handlers.remove(handler)

    def next_event(self, timeout: Optional[float] = None) -> AsyncEvent:
        return self._events.get(timeout=timeout)

    def next_unsolicited(self, timeout: Optional[float] = None) -> ProtocolMessage:
        return self._unsolicited.get(timeout=timeout)

    def send(self, command: str) -> None:
        """Write a command that intentionally has no response.

        This still acquires the request lock, ensuring it cannot insert a
        second command while another caller is awaiting its response.
        """

        payload = encode_line(command)
        with self._request_lock:
            self._ensure_running()
            self._write(payload, command)

    def request(
        self,
        command: str,
        timeout: float = 1.0,
        matcher: Optional[ReplyMatcher] = None,
        guard: Optional[Callable[[], bool]] = None,
    ) -> Reply:
        """Send one command and wait for its matching typed reply.

        Calls are serialized for the full write/wait transaction.  Unknown
        lines and known replies that do not match the command are published as
        unsolicited messages and cannot accidentally complete the request.
        """

        if timeout < 0:
            raise ValueError("timeout must be non-negative")
        payload = encode_line(command)
        effective_matcher = matcher if matcher is not None else reply_matcher_for(command)
        pending = _PendingRequest(effective_matcher, threading.Event())

        with self._request_lock:
            self._ensure_running()
            if guard is not None and not guard():
                raise RequestCancelled(f"request cancelled before send: {command!r}")
            with self._state_lock:
                if self._pending is not None:
                    raise SerialSessionError("internal error: concurrent pending request")
                self._pending = pending
            try:
                self._write(payload, command)
            except BaseException:
                with self._state_lock:
                    if self._pending is pending:
                        self._pending = None
                raise

            completed = pending.completed.wait(timeout)
            with self._state_lock:
                if self._pending is pending:
                    self._pending = None
                reply = pending.reply
                error = pending.error

            if error is not None:
                if isinstance(error, SerialSessionError):
                    raise error
                raise SerialIoError(str(error)) from error
            if reply is None:
                timeout_error = RequestTimeout(command, timeout)
                self._fail(
                    SessionDesynchronized(
                        f"serial reply stream desynchronized after timeout: {command!r}"
                    )
                )
                raise timeout_error
            return reply

    def request_line(
        self,
        command: str,
        timeout: float = 1.0,
        matcher: Optional[ReplyMatcher] = None,
        guard: Optional[Callable[[], bool]] = None,
    ) -> str:
        """Compatibility convenience for callers that still consume raw text."""

        return self.request(
            command, timeout=timeout, matcher=matcher, guard=guard).raw

    def _ensure_running(self) -> None:
        with self._state_lock:
            if self._closed:
                raise SessionClosed("serial session is closed")
            if self._failure is not None:
                if isinstance(self._failure, SerialSessionError):
                    raise self._failure
                raise SerialIoError(str(self._failure)) from self._failure
            if not self._running.is_set():
                raise SessionNotRunning("call start() before using the serial session")
            if not bool(getattr(self._serial, "is_open", True)):
                raise SessionClosed("serial port is not open")

    def _write(self, payload: bytes, line: str) -> None:
        try:
            self._serial.write(payload)
            self._trace("TX", line)
        except Exception as exc:
            error = SerialIoError(f"serial write failed: {exc}")
            self._fail(error)
            raise error from exc

    def _reader_loop(self) -> None:
        while self._running.is_set():
            try:
                data = self._serial.readline()
            except Exception as exc:
                if self._running.is_set():
                    self._fail(SerialIoError(f"serial read failed: {exc}"))
                return
            if not data:
                continue
            message = parse_line(data)
            if not message.raw:
                continue
            self._trace("RX", message.raw)
            if isinstance(message, AsyncEvent):
                self._publish_event(message)
            elif isinstance(message, Reply):
                self._route_reply(message)
            else:
                self._publish_unsolicited(message)

    def _route_reply(self, reply: Reply) -> None:
        with self._state_lock:
            pending = self._pending
        if pending is None:
            self._publish_unsolicited(reply)
            return
        try:
            matches = pending.matcher(reply)
        except Exception as exc:
            matches = False
            self._notify_error(exc)
        if matches:
            with self._state_lock:
                # A timeout or close may have retired this request while its
                # custom matcher was executing.
                if self._pending is pending:
                    pending.reply = reply
                    self._pending = None
                    pending.completed.set()
                    return
        self._publish_unsolicited(reply)

    def _publish_event(self, event: AsyncEvent) -> None:
        self._events.put(event)
        with self._state_lock:
            handlers = tuple(self._event_handlers)
        for handler in handlers:
            try:
                handler(event)
            except Exception as exc:
                self._notify_error(exc)

    def _publish_unsolicited(self, message: ProtocolMessage) -> None:
        self._unsolicited.put(message)
        with self._state_lock:
            handlers = tuple(self._unsolicited_handlers)
        for handler in handlers:
            try:
                handler(message)
            except Exception as exc:
                self._notify_error(exc)

    def _fail(self, error: BaseException) -> None:
        with self._state_lock:
            if self._failure is not None or self._closed:
                return
            self._failure = error
            self._running.clear()
            pending = self._pending
            self._pending = None
            if pending is not None:
                pending.error = error
                pending.completed.set()
        self._notify_error(error)

    def _notify_error(self, error: BaseException) -> None:
        if self._error_handler is None:
            return
        try:
            self._error_handler(error)
        except Exception:
            # Error reporting must never terminate the sole reader thread.
            pass

    def _trace(self, direction: str, line: str) -> None:
        if self._trace_handler is None:
            return
        try:
            self._trace_handler(direction, line)
        except Exception as exc:
            self._notify_error(exc)
