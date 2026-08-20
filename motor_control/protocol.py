"""Pure helpers for the ESP32/Pico line-oriented serial protocol.

The PC talks only to the ESP32 over this protocol.  Messages originating at a
Pico node are translated by the ESP32 into the same ``STEP``/``NODE`` frames,
so the application does not need to understand the RS485 ``N,...`` frames.

This module deliberately has no serial, threading, or Tk dependencies.  It
contains only command encoding and conservative parsing: a line is considered
an asynchronous event or a synchronous reply only when its complete shape is
known.  Everything else remains :class:`UnknownMessage` instead of being able
to satisfy an unrelated request.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable, Optional, Union


BAUD_RATE = 115_200
MAX_MOVE_STEPS = 20_000_000
MAX_STEP_DELAY_US = 10_000_000


class ProtocolEncodingError(ValueError):
    """Raised when a command cannot be represented by the text protocol."""


def _integer(value: object, name: str, minimum: int, maximum: Optional[int] = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolEncodingError(f"{name} must be an integer")
    if value < minimum or (maximum is not None and value > maximum):
        limit = f"{minimum}..{maximum}" if maximum is not None else f">={minimum}"
        raise ProtocolEncodingError(f"{name} must be {limit}")
    return value


def _token(value: object, name: str) -> str:
    text = str(value)
    if not text or any(character in text for character in ",\r\n"):
        raise ProtocolEncodingError(f"{name} is not a valid protocol field")
    try:
        text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolEncodingError(f"{name} must contain ASCII characters only") from exc
    return text


def _axis_token(axis: Union[int, str], *, wildcard: bool = False) -> str:
    if wildcard and axis == "*":
        return "*"
    return str(_integer(axis, "axis", 0))


def command_line(*fields: object) -> str:
    """Join already separated command fields after preventing frame injection."""

    if not fields:
        raise ProtocolEncodingError("a command needs at least one field")
    return ",".join(_token(field, f"field {index}") for index, field in enumerate(fields))


def encode_line(line: str) -> bytes:
    """Encode one complete command with the firmware-required LF terminator."""

    if not isinstance(line, str) or not line or "\r" in line or "\n" in line:
        raise ProtocolEncodingError("command must be one non-empty line")
    try:
        return line.encode("ascii") + b"\n"
    except UnicodeEncodeError as exc:
        raise ProtocolEncodingError("command must contain ASCII characters only") from exc


def build_move_command(
    axis: Union[int, str], steps: int, direction: Union[int, bool], delay_us: int
) -> str:
    """Build ``MOVE,<axis>,<steps>,<direction>,<delay_us>``.

    The ESP32 permits a 1 us minimum for directly wired axes.  RP2040 axes
    additionally require at least 100 us; that topology-specific rule belongs
    in the controller, because the wire format itself does not identify which
    axes are remote.
    """

    axis_field = _axis_token(axis, wildcard=True)
    steps_value = _integer(steps, "steps", 1, MAX_MOVE_STEPS)
    if isinstance(direction, bool):
        direction_value = int(direction)
    else:
        direction_value = _integer(direction, "direction", 0, 1)
    delay_value = _integer(delay_us, "delay_us", 1, MAX_STEP_DELAY_US)
    return command_line("MOVE", axis_field, steps_value, direction_value, delay_value)


def build_stop_command(axis: Union[int, str]) -> str:
    return command_line("STOP", _axis_token(axis, wildcard=True))


def build_estop_command() -> str:
    return "ESTOP"


def build_mode_command() -> str:
    return "MODE"


def build_foc_command(
    axis: Union[int, str], operation: str, value: Optional[object] = None
) -> str:
    fields = ["FOC", _axis_token(axis, wildcard=True), _token(operation, "operation").upper()]
    if value is not None:
        fields.append(_token(value, "value"))
    return command_line(*fields)


def build_track_command(
    action: str, duty_pct: Optional[int] = None, lease_ms: Optional[int] = None
) -> str:
    action_field = _token(action, "action").upper()
    fields: list[object] = ["TRACK", "D", action_field]
    if duty_pct is not None:
        fields.append(_integer(duty_pct, "duty_pct", 1, 100))
    if lease_ms is not None:
        if duty_pct is None:
            raise ProtocolEncodingError("lease_ms requires duty_pct")
        fields.append(_integer(lease_ms, "lease_ms", 0))
    return command_line(*fields)


def build_node_status_command(node: Union[int, str] = "*") -> str:
    return command_line("NODE", _axis_token(node, wildcard=True), "S")


def build_encoder_status_command(axis: int) -> str:
    return command_line("ENC", _axis_token(axis), "S")


@dataclass(frozen=True, slots=True)
class ProtocolMessage:
    raw: str


@dataclass(frozen=True, slots=True)
class AsyncEvent(ProtocolMessage):
    """Marker base for unsolicited, state-changing firmware messages."""


@dataclass(frozen=True, slots=True)
class Reply(ProtocolMessage):
    """Marker base for messages that may complete a synchronous request."""


@dataclass(frozen=True, slots=True)
class UnknownMessage(ProtocolMessage):
    """A diagnostic, startup, blank, or malformed line of unknown semantics."""

    reason: str = "unrecognised line"


class StepResult(Enum):
    DONE = "DONE"
    ABORT = "ABORT"


@dataclass(frozen=True, slots=True)
class StepProgress(AsyncEvent):
    axis: int
    executed_steps: int
    requested_steps: int


@dataclass(frozen=True, slots=True)
class StepTerminal(AsyncEvent):
    axis: int
    result: StepResult
    executed_steps: Optional[int]
    requested_steps: Optional[int]


@dataclass(frozen=True, slots=True)
class TrackTimeout(AsyncEvent):
    pass


class HardwareEstopState(Enum):
    TRIGGERED = "TRIGGERED"
    CLEARED = "CLEARED"


@dataclass(frozen=True, slots=True)
class HardwareEstopEvent(AsyncEvent):
    state: HardwareEstopState


@dataclass(frozen=True, slots=True)
class MotorFaultEvent(AsyncEvent):
    axis: int
    details: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class NodeOfflineEvent(AsyncEvent):
    node: int


@dataclass(frozen=True, slots=True)
class AckReply(Reply):
    axis: int


@dataclass(frozen=True, slots=True)
class OkReply(Reply):
    values: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ErrorReply(Reply):
    reason: str


class FirmwareMode(Enum):
    FOC = "FOC"
    GEAR = "GEAR"


@dataclass(frozen=True, slots=True)
class ModeReply(Reply):
    mode: FirmwareMode


@dataclass(frozen=True, slots=True)
class MotorStatusReply(Reply):
    axis: int
    state: int
    current_deg: float
    target_deg: float
    fault_latched: bool


class TrackDirection(Enum):
    FORWARD = "FWD"
    REVERSE = "REV"
    STOPPED = "STOP"


@dataclass(frozen=True, slots=True)
class TrackStatusReply(Reply):
    direction: TrackDirection
    duty_pct: int
    remaining_ms: int


@dataclass(frozen=True, slots=True)
class EncoderStatusReply(Reply):
    axis: int
    diagnostics_enabled: bool
    mux_online: bool
    sensor_online: bool
    magnet_detected: bool
    raw_angle: int
    single_turn_deg: float
    multi_turn_deg: float
    age_ms: int
    error_count: int


@dataclass(frozen=True, slots=True)
class NodeStatusReply(Reply):
    node: int
    online: bool
    busy_mask: int
    age_ms: int


def _parse_nonnegative(text: str) -> Optional[int]:
    if not text or not text.isdecimal():
        return None
    return int(text, 10)


def _parse_bool(text: str) -> Optional[bool]:
    if text == "0":
        return False
    if text == "1":
        return True
    return None


def _unknown(raw: str, reason: str) -> UnknownMessage:
    return UnknownMessage(raw=raw, reason=reason)


def parse_line(data: Union[str, bytes]) -> ProtocolMessage:
    """Parse one ESP32-to-PC line without guessing at malformed shapes."""

    if isinstance(data, bytes):
        raw = data.decode("utf-8", errors="replace").strip()
    elif isinstance(data, str):
        raw = data.strip()
    else:
        raise TypeError("serial line must be str or bytes")
    if not raw:
        return _unknown(raw, "blank line")

    fields = raw.split(",")

    if len(fields) == 5 and fields[0] == "STEP" and fields[2] == "P":
        axis = _parse_nonnegative(fields[1])
        executed = _parse_nonnegative(fields[3])
        requested = _parse_nonnegative(fields[4])
        if axis is not None and executed is not None and requested is not None:
            return StepProgress(raw, axis, executed, requested)
        return _unknown(raw, "malformed step progress")

    if len(fields) in (3, 5) and fields[0] == "STEP" and fields[2] in ("DONE", "ABORT"):
        axis = _parse_nonnegative(fields[1])
        if axis is None:
            return _unknown(raw, "malformed step terminal axis")
        executed: Optional[int] = None
        requested: Optional[int] = None
        if len(fields) == 5:
            executed = _parse_nonnegative(fields[3])
            requested = _parse_nonnegative(fields[4])
            if executed is None or requested is None:
                return _unknown(raw, "malformed step terminal counts")
        return StepTerminal(raw, axis, StepResult(fields[2]), executed, requested)

    if raw == "TRACK,D,TIMEOUT":
        return TrackTimeout(raw)

    if len(fields) == 2 and fields[0] == "HWESTOP" and fields[1] in ("TRIGGERED", "CLEARED"):
        return HardwareEstopEvent(raw, HardwareEstopState(fields[1]))

    if len(fields) >= 3 and fields[0] == "FOC" and fields[2] == "FAULT":
        axis = _parse_nonnegative(fields[1])
        if axis is not None:
            return MotorFaultEvent(raw, axis, tuple(fields[3:]))
        return _unknown(raw, "malformed motor fault")

    if len(fields) == 3 and fields[0] == "NODE" and fields[2] == "OFFLINE":
        node = _parse_nonnegative(fields[1])
        if node is not None and node > 0:
            return NodeOfflineEvent(raw, node)
        return _unknown(raw, "malformed node offline event")

    if len(fields) == 2 and fields[0] == "ACK":
        axis = _parse_nonnegative(fields[1])
        if axis is not None:
            return AckReply(raw, axis)
        return _unknown(raw, "malformed acknowledgement")

    if fields[0] == "OK" and len(fields) >= 2:
        return OkReply(raw, tuple(fields[1:]))

    if raw.startswith("ERR:"):
        reason = raw[4:]
        if reason:
            return ErrorReply(raw, reason)
        return _unknown(raw, "empty error reply")

    if len(fields) == 2 and fields[0] == "MODE" and fields[1] in ("FOC", "GEAR"):
        return ModeReply(raw, FirmwareMode(fields[1]))

    if len(fields) == 7 and fields[0] == "FOC" and fields[2] == "S":
        axis = _parse_nonnegative(fields[1])
        state = _parse_nonnegative(fields[3])
        fault = _parse_bool(fields[6])
        try:
            current = float(fields[4])
            target = float(fields[5])
        except ValueError:
            current = target = float("nan")
        if (
            axis is not None
            and state is not None
            and fault is not None
            and math.isfinite(current)
            and math.isfinite(target)
        ):
            return MotorStatusReply(raw, axis, state, current, target, fault)
        return _unknown(raw, "malformed motor status")

    if len(fields) == 6 and fields[:3] == ["TRACK", "D", "S"]:
        try:
            direction = TrackDirection(fields[3])
        except ValueError:
            return _unknown(raw, "malformed track direction")
        duty = _parse_nonnegative(fields[4])
        remaining = _parse_nonnegative(fields[5])
        if duty is not None and remaining is not None:
            return TrackStatusReply(raw, direction, duty, remaining)
        return _unknown(raw, "malformed track status")

    if len(fields) == 12 and fields[0] == "ENC" and fields[2] == "S":
        axis = _parse_nonnegative(fields[1])
        flags = tuple(_parse_bool(value) for value in fields[3:7])
        raw_angle = _parse_nonnegative(fields[7])
        age_ms = _parse_nonnegative(fields[10])
        error_count = _parse_nonnegative(fields[11])
        try:
            single_turn = float(fields[8])
            multi_turn = float(fields[9])
        except ValueError:
            single_turn = multi_turn = float("nan")
        if (
            axis is not None
            and all(flag is not None for flag in flags)
            and raw_angle is not None
            and age_ms is not None
            and error_count is not None
            and math.isfinite(single_turn)
            and math.isfinite(multi_turn)
        ):
            return EncoderStatusReply(
                raw,
                axis,
                bool(flags[0]),
                bool(flags[1]),
                bool(flags[2]),
                bool(flags[3]),
                raw_angle,
                single_turn,
                multi_turn,
                age_ms,
                error_count,
            )
        return _unknown(raw, "malformed encoder status")

    if len(fields) == 6 and fields[0] == "NODE" and fields[2] == "S":
        node = _parse_nonnegative(fields[1])
        busy_mask = _parse_nonnegative(fields[4])
        age_ms = _parse_nonnegative(fields[5])
        if (
            node is not None
            and node > 0
            and fields[3] in ("ONLINE", "OFFLINE")
            and busy_mask is not None
            and age_ms is not None
        ):
            return NodeStatusReply(raw, node, fields[3] == "ONLINE", busy_mask, age_ms)
        return _unknown(raw, "malformed node status")

    return _unknown(raw, "unrecognised or malformed line")


ReplyMatcher = Callable[[Reply], bool]


def _ok_has_axis(reply: Reply, axis: str) -> bool:
    return isinstance(reply, OkReply) and bool(reply.values) and (
        axis == "*" or reply.values[0] == axis
    )


def reply_matcher_for(command: str) -> ReplyMatcher:
    """Return a conservative matcher for the single reply to ``command``.

    Firmware errors are accepted for every well-formed request.  Known success
    replies must match both the command family and, where available, its axis.
    This prevents an unrelated late status line from completing a newer call.
    """

    tokens = command.strip().split(",")
    verb = tokens[0].upper() if tokens else ""

    def matches(reply: Reply) -> bool:
        if isinstance(reply, ErrorReply):
            return True
        if verb == "MOVE" and len(tokens) >= 2:
            return isinstance(reply, AckReply) and (tokens[1] == "*" or reply.axis == _parse_nonnegative(tokens[1]))
        if verb == "STOP" and len(tokens) >= 2:
            return _ok_has_axis(reply, tokens[1])
        if verb == "ESTOP":
            return isinstance(reply, OkReply) and reply.values == ("ESTOP",)
        if verb == "MODE":
            return isinstance(reply, ModeReply)
        if verb == "FOC" and len(tokens) >= 3:
            axis = _parse_nonnegative(tokens[1])
            if tokens[2].upper() == "S":
                return isinstance(reply, MotorStatusReply) and (
                    tokens[1] == "*" or reply.axis == axis
                )
            return _ok_has_axis(reply, tokens[1])
        if verb == "TRACK" and len(tokens) >= 3:
            if tokens[2].upper() == "S":
                return isinstance(reply, TrackStatusReply)
            return isinstance(reply, OkReply) and reply.values == ("TRACK", "D")
        if verb == "ENC" and len(tokens) >= 3:
            axis = _parse_nonnegative(tokens[1])
            return isinstance(reply, EncoderStatusReply) and reply.axis == axis
        if verb == "NODE" and len(tokens) >= 3:
            node = _parse_nonnegative(tokens[1])
            return isinstance(reply, NodeStatusReply) and (
                tokens[1] == "*" or reply.node == node
            )
        if verb in ("STDIAG", "DIAG"):
            return isinstance(reply, AckReply)
        return True

    return matches
