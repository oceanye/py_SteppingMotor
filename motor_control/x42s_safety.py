"""Evidence for a group ESTOP: bridge ACK is NOT a driver stop certificate."""
import math
import time

from .coordinated_control import BACKEND_X42S_UART, LOGICAL_ROLE_ORDER
from .protocol import X42sStatusReply, parse_line, build_x42s_status_command
from .x42s_motion import endpoint_tolerance


def confirm_disabled_standstill(app, sent_at, *, allow_closing=False, timeout_s=3.0):
    """Two distinct post-ESTOP samples per bound driver, disabled and stable.

    Disabled drivers need not report reached or clear a latched fault. Never
    infer stationary from old reached/zero-speed cache or from the bridge ACK.
    This proves observed standstill only, not a mechanical brake or support.
    """
    with app.state_lock:
        bindings = app.control_bindings
        if bindings.backend != BACKEND_X42S_UART:
            return True
        generation = app._control_generation
        targets = [(r, bindings.for_role(r)) for r in LOGICAL_ROLE_ORDER]
    if any(b is None or b.kind != "x42s" for _, b in targets):
        return False
    if len({b.driver_id for _, b in targets}) != 4:
        return False
    previous, confirmed = {}, set()
    deadline, index = time.monotonic()+timeout_s, 0
    def current():
        return app._control_generation == generation and app.control_bindings == bindings
    while time.monotonic() < deadline and current():
        role, binding = targets[index % len(targets)]
        index += 1
        driver_id = binding.driver_id
        response = app._send_and_read(build_x42s_status_command(driver_id),
            timeout=min(.5, max(.01, deadline-time.monotonic())), soft_timeout=True,
            allow_closing=allow_closing, guard=current)
        reply = parse_line(response) if response else None
        now = time.monotonic()
        valid = (isinstance(reply, X42sStatusReply) and reply.driver_id == driver_id
                 and reply.online and not reply.enabled
                 and reply.measured_position is not None
                 and math.isfinite(reply.measured_position)
                 and reply.age_ms <= 1500 and now-reply.age_ms/1000 > sent_at
                 and (reply.speed is None or abs(reply.speed) <= .01))
        if isinstance(reply, X42sStatusReply) and reply.driver_id == driver_id:
            app._apply_x42s_reply(role, driver_id, reply)
        if valid:
            sample = (now-reply.age_ms/1000, reply.measured_position)
            old = previous.get(driver_id)
            if (old and sample[0]-old[0] >= .1
                    and abs(sample[1]-old[1]) <= endpoint_tolerance(role, binding)):
                confirmed.add(driver_id)
            else:
                confirmed.discard(driver_id)
            previous[driver_id] = sample
        else:
            previous.pop(driver_id, None)
            confirmed.discard(driver_id)
        if len(confirmed) == len(targets) and current():
            return True
        time.sleep(min(.1, max(0, deadline-time.monotonic())))
    app.log("⛔ ESP32 已接收急停，但四轴失能/新鲜静止反馈未全部确认；保持运动锁定，请现场确认安全")
    return False
