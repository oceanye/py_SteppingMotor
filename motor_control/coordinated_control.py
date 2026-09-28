"""Logical mechanism bindings and read-only motion telemetry projection.

The fixed stepper topology describes wiring and routing.  This module adds a
separate, persisted layer that says which physical stepper axis performs each
mechanism role.  It intentionally has no Tk, HTTP or serial dependencies so
the same validation and snapshots can be used by desktop and web views.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import time
from types import MappingProxyType
from typing import Any, Mapping, Sequence

from .axis_math import MODE_LINEAR, MODE_ROTARY
from .axis_model import AxisProfile, AxisRuntime
from .topology import NUM_STEPPER_AXES, stepper_axis_topology, validate_stepper_axis


BINDING_SCHEMA_VERSION = 1
POSITION_SOURCE = "host_pulse_accounting"


class LogicalRole(str, Enum):
    """The four mechanism roles named by the coordinated-motion handoff."""

    MUP1 = "Mup1"
    MR1 = "Mr1"
    MUP2 = "Mup2"
    MR2 = "Mr2"


LOGICAL_ROLE_ORDER: tuple[LogicalRole, ...] = (
    LogicalRole.MUP1,
    LogicalRole.MR1,
    LogicalRole.MUP2,
    LogicalRole.MR2,
)


@dataclass(frozen=True, slots=True)
class RoleSpec:
    role: LogicalRole
    display_name: str
    side: str
    action: str
    required_mode: str


ROLE_SPECS: Mapping[LogicalRole, RoleSpec] = MappingProxyType(
    {
        LogicalRole.MUP1: RoleSpec(
            LogicalRole.MUP1, "左侧升降", "left", "左侧直线升降", MODE_LINEAR
        ),
        LogicalRole.MR1: RoleSpec(
            LogicalRole.MR1, "左侧旋转", "left", "左侧转动", MODE_ROTARY
        ),
        LogicalRole.MUP2: RoleSpec(
            LogicalRole.MUP2, "右侧升降", "right", "右侧直线升降", MODE_LINEAR
        ),
        LogicalRole.MR2: RoleSpec(
            LogicalRole.MR2, "右侧旋转", "right", "右侧转动", MODE_ROTARY
        ),
    }
)

SUGGESTED_AXIS_BY_ROLE: Mapping[LogicalRole, int] = MappingProxyType(
    {
        LogicalRole.MUP1: 0,
        LogicalRole.MR1: 2,
        LogicalRole.MUP2: 1,
        LogicalRole.MR2: 3,
    }
)

AUTOMATION_BLOCKERS: tuple[str, ...] = (
    "direction_and_zero_unverified",
    "contact_feedback_unavailable",
    "support_load_feedback_unavailable",
    "collision_model_unverified",
    "hardware_limit_chain_incomplete",
    "coordinated_start_protocol_unavailable",
    "pico_node_health_incomplete",
)


class BindingValidationError(ValueError):
    """A binding document or candidate violates the stable domain contract."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True, slots=True)
class BindingIssue:
    code: str
    message: str
    role: LogicalRole | None = None
    axis: int | None = None


@dataclass(frozen=True, slots=True)
class ActuatorBinding:
    """One logical role's v1 binding to a physical stepper axis."""

    axis: int
    kind: str = "stepper"

    def __post_init__(self) -> None:
        if self.kind != "stepper":
            raise BindingValidationError(
                "unsupported_kind", "v1 逻辑电机绑定只支持 kind=stepper"
            )
        try:
            normalized = validate_stepper_axis(self.axis)
        except (TypeError, ValueError) as exc:
            raise BindingValidationError("invalid_axis", str(exc)) from exc
        object.__setattr__(self, "axis", normalized)

    def as_dict(self) -> dict[str, object]:
        return {"kind": self.kind, "axis": self.axis}


@dataclass(frozen=True, slots=True)
class BindingSet:
    """Immutable, complete snapshot of all four role assignments."""

    assignments: tuple[ActuatorBinding | None, ...] = (None, None, None, None)
    revision: int = 0

    def __post_init__(self) -> None:
        if len(self.assignments) != len(LOGICAL_ROLE_ORDER):
            raise BindingValidationError(
                "role_set", "绑定必须恰好包含 Mup1、Mr1、Mup2、Mr2"
            )
        if any(
            binding is not None and not isinstance(binding, ActuatorBinding)
            for binding in self.assignments
        ):
            raise BindingValidationError(
                "binding_type", "每个角色必须是步进轴绑定或 null"
            )
        if (
            isinstance(self.revision, bool)
            or not isinstance(self.revision, int)
            or self.revision < 0
        ):
            raise BindingValidationError("invalid_revision", "revision 必须是非负整数")
        axes = [item.axis for item in self.assignments if item is not None]
        if len(axes) != len(set(axes)):
            raise BindingValidationError(
                "duplicate_axis", "同一物理步进轴不能绑定多个逻辑角色"
            )

    @classmethod
    def empty(cls) -> "BindingSet":
        return cls()

    @classmethod
    def suggested(cls) -> "BindingSet":
        return cls.from_axis_mapping(SUGGESTED_AXIS_BY_ROLE)

    @classmethod
    def from_axis_mapping(
        cls,
        values: Mapping[LogicalRole | str, int | None],
        *,
        revision: int = 0,
    ) -> "BindingSet":
        normalized: dict[str, int | None] = {}
        for key, value in values.items():
            name = key.value if isinstance(key, LogicalRole) else key
            if not isinstance(name, str):
                raise BindingValidationError("unknown_role", "逻辑角色名称必须是字符串")
            normalized[name] = value
        expected = {role.value for role in LOGICAL_ROLE_ORDER}
        actual = set(normalized)
        if actual != expected:
            missing = sorted(expected - actual)
            unknown = sorted(actual - expected)
            detail = []
            if missing:
                detail.append("缺少 " + ", ".join(missing))
            if unknown:
                detail.append("未知 " + ", ".join(unknown))
            raise BindingValidationError(
                "role_set", "绑定角色集合无效：" + "；".join(detail)
            )
        assignments: list[ActuatorBinding | None] = []
        for role in LOGICAL_ROLE_ORDER:
            axis = normalized[role.value]
            assignments.append(None if axis is None else ActuatorBinding(axis=axis))
        return cls(tuple(assignments), revision=revision)

    def for_role(self, role: LogicalRole | str) -> ActuatorBinding | None:
        try:
            normalized = role if isinstance(role, LogicalRole) else LogicalRole(role)
        except ValueError as exc:
            raise KeyError(role) from exc
        return self.assignments[LOGICAL_ROLE_ORDER.index(normalized)]

    def role_for_axis(self, axis: int) -> LogicalRole | None:
        normalized = validate_stepper_axis(axis)
        for role, binding in zip(LOGICAL_ROLE_ORDER, self.assignments):
            if binding is not None and binding.axis == normalized:
                return role
        return None

    @property
    def is_complete(self) -> bool:
        return all(binding is not None for binding in self.assignments)

    @property
    def bound_axes(self) -> tuple[int, ...]:
        return tuple(
            binding.axis for binding in self.assignments if binding is not None
        )

    def with_revision(self, revision: int) -> "BindingSet":
        return replace(self, revision=revision)

    def as_axis_mapping(self) -> dict[str, int | None]:
        return {
            role.value: None if binding is None else binding.axis
            for role, binding in zip(LOGICAL_ROLE_ORDER, self.assignments)
        }

    def as_document(self) -> dict[str, object]:
        return {
            "schema_version": BINDING_SCHEMA_VERSION,
            "revision": self.revision,
            "bindings": {
                role.value: None if binding is None else binding.as_dict()
                for role, binding in zip(LOGICAL_ROLE_ORDER, self.assignments)
            },
        }


def parse_binding_document(value: Mapping[str, Any]) -> BindingSet:
    """Parse persisted JSON without accepting ambiguous or partial schemas."""

    if not isinstance(value, Mapping):
        raise BindingValidationError("document_type", "绑定配置顶层必须是对象")
    expected_top = {"schema_version", "revision", "bindings"}
    actual_top = set(value)
    if actual_top != expected_top:
        raise BindingValidationError("document_fields", "绑定配置顶层字段不完整或未知")
    if value.get("schema_version") != BINDING_SCHEMA_VERSION:
        raise BindingValidationError("schema_version", "不支持的绑定配置版本")
    revision = value.get("revision")
    if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
        raise BindingValidationError("invalid_revision", "revision 必须是非负整数")
    raw_bindings = value.get("bindings")
    if not isinstance(raw_bindings, Mapping):
        raise BindingValidationError("bindings_type", "bindings 必须是对象")
    expected_roles = {role.value for role in LOGICAL_ROLE_ORDER}
    if set(raw_bindings) != expected_roles:
        raise BindingValidationError(
            "role_set", "绑定必须恰好包含 Mup1、Mr1、Mup2、Mr2"
        )

    parsed: dict[str, int | None] = {}
    for role in LOGICAL_ROLE_ORDER:
        item = raw_bindings[role.value]
        if item is None:
            parsed[role.value] = None
            continue
        if not isinstance(item, Mapping) or set(item) != {"kind", "axis"}:
            raise BindingValidationError(
                "binding_fields", f"{role.value} 绑定必须只包含 kind 和 axis"
            )
        if item.get("kind") != "stepper":
            raise BindingValidationError(
                "unsupported_kind", f"{role.value} 仅支持 kind=stepper"
            )
        axis = item.get("axis")
        if isinstance(axis, bool) or not isinstance(axis, int):
            raise BindingValidationError(
                "invalid_axis", f"{role.value} 的 axis 必须是整数"
            )
        parsed[role.value] = axis
    return BindingSet.from_axis_mapping(parsed, revision=revision)


def binding_compatibility_issues(
    bindings: BindingSet,
    profiles: Sequence[AxisProfile],
    axis_param_valid: Sequence[bool],
) -> tuple[BindingIssue, ...]:
    """Return applied-profile issues without treating an unbound role as corrupt."""

    issues: list[BindingIssue] = []
    for role in LOGICAL_ROLE_ORDER:
        binding = bindings.for_role(role)
        if binding is None:
            continue
        axis = binding.axis
        spec = ROLE_SPECS[role]
        if axis >= len(profiles) or axis >= len(axis_param_valid):
            issues.append(
                BindingIssue("axis_state_missing", "物理轴状态不可用", role, axis)
            )
            continue
        if profiles[axis].mode != spec.required_mode:
            required = "直线" if spec.required_mode == MODE_LINEAR else "旋转"
            actual = "直线" if profiles[axis].mode == MODE_LINEAR else "旋转"
            issues.append(
                BindingIssue(
                    "mode_mismatch",
                    f"{role.value} 要求{required}模式，步进轴 {axis} 当前为{actual}模式",
                    role,
                    axis,
                )
            )
        if not bool(axis_param_valid[axis]):
            issues.append(
                BindingIssue(
                    "axis_params_invalid",
                    f"步进轴 {axis} 参数无效或尚未应用",
                    role,
                    axis,
                )
            )
    return tuple(issues)


@dataclass(frozen=True, slots=True)
class AxisMotionTelemetry:
    """Transient, pulse-reported progress for one physical stepper axis."""

    generation: int = 0
    state: str = "IDLE"
    requested_steps: int | None = None
    executed_steps: int | None = None
    started_monotonic: float | None = None
    updated_monotonic: float | None = None
    last_result: str | None = None

    @classmethod
    def starting(
        cls, generation: int, requested_steps: int, *, now: float | None = None
    ) -> "AxisMotionTelemetry":
        if isinstance(requested_steps, bool) or not isinstance(requested_steps, int):
            raise TypeError("requested_steps must be an integer")
        if requested_steps <= 0:
            raise ValueError("requested_steps must be positive")
        stamp = time.monotonic() if now is None else float(now)
        return cls(
            generation=generation,
            state="STARTING",
            requested_steps=requested_steps,
            executed_steps=None,
            started_monotonic=stamp,
            updated_monotonic=stamp,
        )

    def with_progress(
        self, executed_steps: int, requested_steps: int, *, now: float | None = None
    ) -> "AxisMotionTelemetry":
        if (
            isinstance(executed_steps, bool)
            or isinstance(requested_steps, bool)
            or not isinstance(executed_steps, int)
            or not isinstance(requested_steps, int)
            or requested_steps <= 0
            or executed_steps < 0
            or executed_steps > requested_steps
        ):
            return self
        if self.requested_steps is not None and requested_steps != self.requested_steps:
            return self
        stamp = time.monotonic() if now is None else float(now)
        return replace(
            self,
            state="MOVING",
            requested_steps=requested_steps,
            executed_steps=executed_steps,
            updated_monotonic=stamp,
        )

    def stopping(self, *, now: float | None = None) -> "AxisMotionTelemetry":
        stamp = time.monotonic() if now is None else float(now)
        return replace(self, state="STOPPING", updated_monotonic=stamp)

    def terminal(
        self,
        result: str,
        executed_steps: int | None,
        requested_steps: int | None,
        *,
        now: float | None = None,
    ) -> "AxisMotionTelemetry":
        stamp = time.monotonic() if now is None else float(now)
        return replace(
            self,
            state="IDLE",
            requested_steps=None,
            executed_steps=None,
            updated_monotonic=stamp,
            last_result=result,
        )

    def reset(self, *, result: str | None = None) -> "AxisMotionTelemetry":
        return AxisMotionTelemetry(generation=self.generation, last_result=result)


def projected_step_position(
    committed_steps: float,
    pending_signed_steps: float | None,
    telemetry: AxisMotionTelemetry,
) -> float | None:
    """Project only from a real progress frame; never mutate committed state."""

    if pending_signed_steps is None:
        return None
    executed = telemetry.executed_steps
    requested = telemetry.requested_steps
    if executed is None or requested is None or requested <= 0:
        return None
    fraction = max(0.0, min(1.0, float(executed) / float(requested)))
    return float(committed_steps) + float(pending_signed_steps) * fraction


def build_coordinated_snapshot(
    *,
    bindings: BindingSet,
    profiles: Sequence[AxisProfile],
    runtimes: Sequence[AxisRuntime],
    axis_param_valid: Sequence[bool],
    running: Sequence[bool],
    stepper_in_progress: Sequence[bool],
    move_dispatching: Sequence[bool],
    move_reservations: Sequence[object | None],
    web_step_pending: Sequence[object | None],
    pending_steps: Sequence[float | None],
    telemetry: Sequence[AxisMotionTelemetry],
    connected: bool,
    safety_state: str = "normal",
    pico_node_health: Mapping[int, str] | None = None,
    now: float | None = None,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    """Build one coherent, JSON-compatible snapshot for desktop and web."""

    stamp = time.monotonic() if now is None else float(now)
    node_health = pico_node_health or {}
    issues = binding_compatibility_issues(bindings, profiles, axis_param_valid)
    issue_by_role = {issue.role: issue for issue in issues if issue.role is not None}
    logical_motors: list[dict[str, object]] = []

    for role in LOGICAL_ROLE_ORDER:
        spec = ROLE_SPECS[role]
        binding = bindings.for_role(role)
        base: dict[str, object] = {
            "role": role.value,
            "display_name": spec.display_name,
            "side": spec.side,
            "action": spec.action,
            "required_mode": spec.required_mode,
            "binding_revision": bindings.revision,
            "position_source": POSITION_SOURCE,
            "measured": False,
        }
        if binding is None:
            base.update(
                {
                    "binding": None,
                    "bound": False,
                    "binding_valid": False,
                    "binding_error": "未绑定物理步进轴",
                    "state": "UNBOUND",
                    "position": None,
                    "committed_position": None,
                    "projected_position": None,
                    "target_position": None,
                    "position_trusted": False,
                    "unit": "mm" if spec.required_mode == MODE_LINEAR else "°",
                    "progress_percent": None,
                    "executed_steps": None,
                    "requested_steps": None,
                    "last_update_age_ms": None,
                    "stale": True,
                }
            )
            logical_motors.append(base)
            continue

        axis = binding.axis
        profile = profiles[axis]
        runtime = runtimes[axis]
        topology = stepper_axis_topology(axis)
        motion = telemetry[axis]
        committed_steps = runtime.position_steps
        projected_steps = projected_step_position(
            committed_steps, pending_steps[axis], motion
        )
        committed_position = profile.units_from_steps(committed_steps)
        projected_position = (
            None
            if projected_steps is None
            else profile.units_from_steps(projected_steps)
        )
        target_position = (
            None
            if pending_steps[axis] is None
            else profile.units_from_steps(committed_steps + pending_steps[axis])
        )
        minimum, maximum = runtime.limits_in(profile)
        progress_percent = None
        if motion.executed_steps is not None and motion.requested_steps:
            progress_percent = max(
                0.0,
                min(100.0, 100.0 * motion.executed_steps / motion.requested_steps),
            )
        update_age_ms = (
            None
            if motion.updated_monotonic is None
            else max(0, int(round((stamp - motion.updated_monotonic) * 1000)))
        )
        active = bool(
            running[axis]
            or stepper_in_progress[axis]
            or move_dispatching[axis]
            or move_reservations[axis] is not None
            or web_step_pending[axis] is not None
            or pending_steps[axis] is not None
        )
        stale = active and (update_age_ms is None or update_age_ms > 1000)
        issue = issue_by_role.get(role)

        if issue is not None:
            state = "INVALID"
        elif not connected:
            state = "DISCONNECTED"
        elif topology["controller"] == "pico" and node_health.get(
            int(topology["node"]), "unknown"
        ) == "offline":
            state = "NODE_OFFLINE"
        elif topology["controller"] == "pico" and node_health.get(
            int(topology["node"]), "unknown"
        ) != "online":
            state = "NODE_UNKNOWN"
        elif safety_state != "normal":
            state = safety_state.upper()
        elif motion.state == "STOPPING":
            state = "STOPPING"
        elif running[axis]:
            state = "CONTINUOUS"
        elif stepper_in_progress[axis] or pending_steps[axis] is not None:
            state = "MOVING" if motion.executed_steps is not None else "STARTING"
        elif (
            move_dispatching[axis]
            or move_reservations[axis] is not None
            or web_step_pending[axis] is not None
        ):
            state = "STARTING"
        elif motion.last_result == "ABORTED" and not runtime.position_trusted:
            state = "ABORTED_UNTRUSTED"
        elif not runtime.position_trusted:
            state = "IDLE_UNTRUSTED"
        else:
            state = "IDLE"

        base.update(
            {
                "binding": binding.as_dict(),
                "bound": True,
                "binding_valid": issue is None,
                "binding_error": None if issue is None else issue.message,
                "axis": axis,
                "physical_label": topology["label"],
                "controller": topology["controller"],
                "node": topology["node"],
                "local_axis": topology["local_axis"],
                "pins": topology["pins"],
                "mode": profile.mode,
                "unit": profile.unit,
                "config_valid": bool(axis_param_valid[axis]),
                "position": (
                    projected_position
                    if projected_position is not None
                    else committed_position
                ),
                "committed_position": committed_position,
                "projected_position": projected_position,
                "target_position": target_position,
                "travel_min": minimum,
                "travel_max": maximum,
                "position_trusted": runtime.position_trusted,
                "state": state,
                "in_progress": active,
                "continuous": bool(running[axis]),
                "executed_steps": motion.executed_steps,
                "requested_steps": motion.requested_steps,
                "progress_percent": progress_percent,
                "last_update_monotonic_ms": (
                    None
                    if motion.updated_monotonic is None
                    else int(round(motion.updated_monotonic * 1000))
                ),
                "last_update_age_ms": update_age_ms,
                "stale": stale,
            }
        )
        logical_motors.append(base)

    configuration_valid = bindings.is_complete and not issues
    blockers = list(AUTOMATION_BLOCKERS)
    if not configuration_valid:
        blockers.insert(0, "logical_motor_bindings_incomplete_or_invalid")
    control = {
        "binding_revision": bindings.revision,
        "configuration_valid": configuration_valid,
        "automation_ready": False,
        "edit_scope": "desktop_only",
        "position_source": POSITION_SOURCE,
        "measured": False,
        "blockers": blockers,
    }
    return control, logical_motors


__all__ = [
    "AUTOMATION_BLOCKERS",
    "ActuatorBinding",
    "AxisMotionTelemetry",
    "BINDING_SCHEMA_VERSION",
    "BindingIssue",
    "BindingSet",
    "BindingValidationError",
    "LOGICAL_ROLE_ORDER",
    "LogicalRole",
    "POSITION_SOURCE",
    "ROLE_SPECS",
    "RoleSpec",
    "SUGGESTED_AXIS_BY_ROLE",
    "binding_compatibility_issues",
    "build_coordinated_snapshot",
    "parse_binding_document",
    "projected_step_position",
]
