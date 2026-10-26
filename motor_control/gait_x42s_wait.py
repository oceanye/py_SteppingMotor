"""Fair, bounded-rate X42S group completion; no assumed synchronous driver clock."""
import time

from .protocol import X42sStatusReply, build_x42s_status_command, parse_line
from .x42s_motion import ArrivalEvidence
from .x42s_units import X42S_AXIS_HANDLE_BASE


def owned_health_issue(app):
    """Explicit faults of ANY gait-owned axis matter, including the support/lift hold."""
    for axis in getattr(app, "_gait_owned", {}):
        if axis < X42S_AXIS_HANDLE_BASE:
            continue
        role = app.control_bindings.role_for_driver_id(axis-X42S_AXIS_HANDLE_BASE)
        state = app._x42s_states.get(role.value) if role is not None else None
        if state is None or not state.online or not state.enabled or state.fault is not None:
            return axis
    return None


def wait_moves(app, axes, timeout_s, *, blind_s=.45, poll_s=.3):
    """Return a group terminal. Poll both members, even if one is already done.

    At most one foreground request per poll interval (not a burst per axis).
    Background telemetry remains active. A peer fault must not wait for the
    first axis's full trajectory timeout. ACK/target/owner checks apply to all.
    """
    axes = tuple(axes)
    with app.state_lock:
        expected = {a: getattr(app, "_gait_x42s_moves", {}).get(a) for a in axes}
        if not axes or any(e is None or e.owner is not getattr(app, "_gait_owned", {}).get(a)
                           for a, e in expected.items()):
            app.log("⛔ X42S 缺少本组命令目标/预约，不能确认到位")
            return "ABORTED"
    now = time.monotonic()
    deadline, next_ui = now+timeout_s, now
    next_poll = max(now, max((e.acknowledged_at or now) + (0 if e.no_motion else blind_s)
                             for e in expected.values()) + .02)
    done, index = set(), 0
    evidence = {axis: ArrivalEvidence() for axis in axes}

    def terminal(result, axis=None, reply=None):
        recorder = getattr(app, "diagnostics", None)
        if recorder is not None:
            recorder.event("motor", "gait.group_terminal", result=result, axes=axes,
                           failing_axis=axis, expectations={a: dict(target=e.target_motor_deg,
                           origin=e.origin_motor_deg, ack_at=e.acknowledged_at) for a, e in expected.items()},
                           reply=reply)
        return result

    while True:
        if app._control_worker_cancelled() or getattr(getattr(app, "_gait_run", None), "_stop_requested", False):
            return terminal("CANCELLED")
        issue = owned_health_issue(app)
        if issue is not None:
            app.log(f"⛔ 步态占用轴 ID{issue-X42S_AXIS_HANDLE_BASE} 离线/失能/故障；即使该轴当前不动也必须中止")
            return terminal("ABORTED", issue)
        for axis, e in expected.items():
            if (getattr(app, "_gait_x42s_moves", {}).get(axis) is not e
                    or app._move_reservation.get(axis) is not e.owner):
                return terminal("CANCELLED", axis)
            role = app.control_bindings.role_for_driver_id(axis-X42S_AXIS_HANDLE_BASE)
            if role is None:
                return terminal("ABORTED", axis)
            state = app._x42s_states.get(role.value)
            if state is not None and (not state.online or not state.enabled or state.fault is not None):
                app.log(f"⛔ {role.value} 同组反馈离线/失能/保护故障；停止全部步态轴")
                return terminal("ABORTED", axis)
        now = time.monotonic()
        if now >= deadline:
            app.log("⛔ X42S 本组目标实测到位核验超时（不以 ACK/reached 单独放行）")
            return terminal("TIMEOUT")
        if now >= next_ui:
            next_ui = now+.25
            app._post_ui(app._refresh_gait_ui)
        if now < next_poll:
            time.sleep(min(.02, next_poll-now))
            continue
        next_poll = now+poll_s
        axis = axes[index % len(axes)]
        index += 1
        e = expected[axis]
        driver_id = axis-X42S_AXIS_HANDLE_BASE
        role = app.control_bindings.role_for_driver_id(driver_id)
        response = app._send_and_read(build_x42s_status_command(driver_id), soft_timeout=True)
        reply = parse_line(response) if response else None
        if not isinstance(reply, X42sStatusReply) or reply.driver_id != driver_id:
            done.discard(axis)
            evidence[axis] = ArrivalEvidence()
            continue
        app._apply_x42s_reply(role, driver_id, reply)
        if not reply.online or not reply.enabled or reply.fault is not None:
            app.log(f"⛔ {role.value} / ID{driver_id} 离线/失能/保护故障 {reply.fault}，联动中止")
            return terminal("ABORTED", axis, reply)
        now = time.monotonic()
        if (e.acknowledged_at is not None and now-reply.age_ms/1000 > e.acknowledged_at
                and e.outside_travel(reply.measured_position)):
            app.log(f"⛔ {role.value} 实测位置越出本段包络，联动中止")
            return terminal("ABORTED", axis, reply)
        if evidence[axis].observe(e, reply, now, blind_s):
            done.add(axis)
        else:
            done.discard(axis)
        if len(done) == len(axes):
            with app.state_lock:
                if any(getattr(app, "_gait_x42s_moves", {}).get(a) is not ex
                       or app._move_reservation.get(a) is not ex.owner for a, ex in expected.items()):
                    return terminal("CANCELLED")
                for a in axes:
                    app._gait_x42s_moves.pop(a)
            return terminal("DONE")
