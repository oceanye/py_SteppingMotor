"""Thread-safe HTTP controller adapter for the desktop application.

``DesktopWebController`` owns the public operations consumed by
``web_control.WebControlServer``.  It deliberately imports no Tk modules: UI
changes are handed back to the application through its synchronous
``_call_ui`` or asynchronous ``_post_ui`` boundary.
"""

from __future__ import annotations

import math
import threading
from typing import Any

from .axis_math import (
    MODE_LINEAR,
    MODE_ROTARY,
    SPEED_DEFAULT,
    coerce_finite_in_range,
    speed_to_delay_ms,
)
from .topology import (
    AXIS_LABEL,
    NUM_LOCAL_STEPPER_AXES,
    NUM_PICO_NODES,
    NUM_STEPPER_AXES,
    PICO_AXES_PER_NODE,
    stepper_axis_topology,
)
from .protocol import (
    MAX_STEP_DELAY_US,
    build_foc_command,
    build_track_command,
)
from .ui_dispatch import UiDispatchTimeout


NUM_MOTOR_AXES = 2
MODE_FOC = "FOC"
MODE_GEAR = "GEAR"
DIR_OUTWARD = 1
DIR_INWARD = 0


class DesktopWebController:
    """Expose a desktop application's motor controls to HTTP worker threads.

    The wrapped application remains the owner of hardware state, locks, serial
    I/O and UI dispatch.  Keeping that ownership intact makes this extraction
    behaviour-preserving while providing a non-Tk seam for tests and future
    controller refactoring.
    """

    def __init__(self, application: Any) -> None:
        self._app = application

    @staticmethod
    def _require_confirmation(response: str, expected: str, operation: str) -> None:
        if response == expected:
            return
        if response.startswith("ERR"):
            raise RuntimeError(f"{operation}被固件拒绝: {response}")
        if not response:
            raise RuntimeError(f"{operation}未获固件确认")
        raise RuntimeError(
            f"{operation}收到非预期响应: {response}（期望 {expected}）"
        )

    @staticmethod
    def parse_direction(direction: str | int) -> int:
        """Normalize all direction spellings accepted by the existing API."""

        if isinstance(direction, str):
            value = direction.strip().upper()
            if value in ("FWD", "FORWARD", "OUT", "OUTWARD", "UP", "+"):
                return DIR_OUTWARD
            if value in ("REV", "REVERSE", "IN", "INWARD", "DOWN", "-"):
                return DIR_INWARD
        elif direction in (DIR_OUTWARD, DIR_INWARD):
            return int(direction)
        raise ValueError("direction 必须是 FWD/REV（或 1/0）")

    @staticmethod
    def _coerce_axis_param(
        value: object,
        minimum: float,
        maximum: float,
        label: str,
    ) -> float:
        try:
            return coerce_finite_in_range(value, minimum, maximum, label)
        except ValueError as exc:
            raise ValueError(
                f"{label}必须在 {minimum:g}–{maximum:g} 范围内"
            ) from exc

    def web_get_status(self) -> dict[str, Any]:
        """Return a thread-safe snapshot containing JSON-compatible values."""

        app = self._app
        topology_axes = [
            stepper_axis_topology(axis) for axis in range(NUM_STEPPER_AXES)
        ]
        with app.state_lock:
            steppers = [
                {
                    "axis": axis,
                    "label": AXIS_LABEL[axis],
                    "position_mm": app.axis_runtime[axis].position_in(
                        app.axis_profiles[axis]
                    ),
                    "position_trusted": app.axis_runtime[axis].position_trusted,
                    "in_progress": (
                        app.running[axis]
                        or app.stepper_in_progress[axis]
                        or app._move_dispatching[axis]
                        or app._move_reservation[axis] is not None
                        or app._web_step_pending[axis] is not None
                        or app._pending_step[axis] is not None
                    ),
                    "continuous": app.running[axis],
                    "travel_min_mm": app.axis_runtime[axis].limits_in(
                        app.axis_profiles[axis]
                    )[0],
                    "travel_max_mm": app.axis_runtime[axis].limits_in(
                        app.axis_profiles[axis]
                    )[1],
                    "mode": app.axis_profiles[axis].mode,
                    "unit": app.axis_profiles[axis].unit,
                    "pulse_per_rev": app.axis_profiles[axis].pulse_per_rev,
                    "gear_ratio": app.axis_profiles[axis].gear_ratio,
                    "lead_mm": app.axis_profiles[axis].lead_mm,
                    "controller": topology_axes[axis]["controller"],
                    "node": topology_axes[axis]["node"],
                    "local_axis": topology_axes[axis]["local_axis"],
                    "pins": topology_axes[axis]["pins"],
                }
                for axis in range(NUM_STEPPER_AXES)
            ]
            motors = [
                dict(axis=axis, label=AXIS_LABEL[axis], **app._motor_status[axis])
                for axis in range(NUM_MOTOR_AXES)
            ]
            track = {
                "axis": "D",
                "direction": app._track_direction,
                "last_response": app._track_last_response,
                "lease_ms": app._track_lease_ms,
            }
            mode = app._fw_mode_cache
        return {
            "connected": app._is_serial_connected(),
            "mode": mode,
            "steppers": [
                (
                    f"{item['position_mm']:.1f} {item['unit']}"
                    if item["position_trusted"]
                    else "需重新校准"
                )
                + (" · 运行中" if item["in_progress"] else "")
                for item in steppers
            ],
            "motors": [
                (
                    f"{item['current_deg']:.1f}°"
                    if item["current_deg"] is not None
                    else "--"
                )
                + (" · 故障" if item["fault"] else "")
                for item in motors
            ],
            "track": track["direction"],
            "stepper_axes": steppers,
            "motor_axes": motors,
            "track_detail": track,
            "topology": {
                "total_axes": NUM_STEPPER_AXES,
                "local_axes": NUM_LOCAL_STEPPER_AXES,
                "pico_nodes": NUM_PICO_NODES,
                "axes_per_pico": PICO_AXES_PER_NODE,
                "axes": topology_axes,
            },
        }

    def web_stepper_move(
        self,
        axis: int,
        direction: str | int,
        distance_mm: float,
        speed_mm_s: float = SPEED_DEFAULT,
    ) -> dict[str, Any]:
        """Asynchronously move a stepper without touching Tk on this thread."""

        app = self._app
        generation = app._ensure_web_control_available()
        try:
            axis = int(axis)
            distance_mm = float(distance_mm)
            speed_mm_s = float(speed_mm_s)
            direction = self.parse_direction(direction)
        except (TypeError, ValueError) as exc:
            raise ValueError(str(exc)) from exc
        if not 0 <= axis < NUM_STEPPER_AXES:
            raise ValueError("步进轴编号越界")
        if (
            not math.isfinite(distance_mm)
            or not math.isfinite(speed_mm_s)
            or distance_mm <= 0
            or speed_mm_s <= 0
        ):
            raise ValueError("距离和速度必须大于 0")
        with app.state_lock:
            if not app.axis_param_valid[axis]:
                raise RuntimeError("轴参数无效或尚未输入完成，请先在 GUI 中修正")
            runtime = app.axis_runtime[axis]
            profile = app.axis_profiles[axis]
            if not runtime.position_trusted:
                raise RuntimeError("位置不可信，请在 GUI 中重新校准")
            if (
                app.running[axis]
                or app.stepper_in_progress[axis]
                or app._move_dispatching[axis]
                or app._move_reservation[axis] is not None
                or app._web_step_pending[axis] is not None
                or app._pending_step[axis] is not None
            ):
                raise RuntimeError("该轴正在运动")
            sign = 1.0 if direction == DIR_OUTWARD else -1.0
            target_steps = runtime.target_steps(profile, sign * distance_mm)
            tolerance_steps = profile.exact_steps_from_units(0.05)
            if (
                runtime.min_steps is not None
                and target_steps < runtime.min_steps - tolerance_steps
            ):
                raise RuntimeError("目标超出软件下限")
            if (
                runtime.max_steps is not None
                and target_steps > runtime.max_steps + tolerance_steps
            ):
                raise RuntimeError("目标超出软件上限")
            delay_ms = speed_to_delay_ms(speed_mm_s, profile.pulses_per_unit)
            if round(delay_ms * 1_000) > MAX_STEP_DELAY_US:
                minimum_speed = (
                    1_000_000.0
                    / (profile.pulses_per_unit * MAX_STEP_DELAY_US)
                )
                raise ValueError(
                    f"速度过低；当前轴最小可达约 {minimum_speed:g} "
                    f"{profile.speed_unit}"
                )
            reservation = object()
            motion_generation = app._axis_motion_generation[axis]
            app._move_reservation[axis] = reservation
            app._web_step_pending[axis] = reservation

        def worker() -> None:
            try:
                app._send_mm(
                    axis,
                    distance_mm,
                    direction,
                    delay_ms,
                    profile=profile,
                    guard=lambda: generation == app._control_generation,
                    reservation=reservation,
                    motion_generation=motion_generation,
                )
            finally:
                with app.state_lock:
                    if app._move_reservation[axis] is reservation:
                        app._move_reservation[axis] = None
                    if app._web_step_pending[axis] is reservation:
                        app._web_step_pending[axis] = None

        threading.Thread(target=worker, daemon=True).start()
        return {"ok": True, "accepted": True, "axis": axis}

    def web_stepper_stop(self, axis: int) -> dict[str, Any]:
        app = self._app
        generation = app._ensure_web_control_available()
        try:
            axis = int(axis)
        except (TypeError, ValueError):
            raise ValueError("无效步进轴编号") from None
        if not 0 <= axis < NUM_STEPPER_AXES:
            raise ValueError("步进轴编号越界")
        stop_reservation = app._begin_axis_stop(axis)
        response = app._send_axis_stop(
            axis,
            stop_reservation=stop_reservation,
            guard=lambda: generation == app._control_generation,
        )
        self._require_confirmation(response, f"OK,{axis}", "步进停止命令")
        return {"ok": True, "confirmed": True, "axis": axis}

    def web_stepper_config(
        self,
        axis: int,
        mode: str | None = None,
        pulse_per_rev: float | None = None,
        gear_ratio: float | None = None,
        lead_mm: float | None = None,
    ) -> dict[str, Any]:
        """Apply axis configuration on the UI thread before returning."""

        app = self._app
        try:
            axis = int(axis)
        except (TypeError, ValueError):
            raise ValueError("axis 必须是整数") from None
        if not 0 <= axis < NUM_STEPPER_AXES:
            raise ValueError("步进轴编号越界")
        if mode is not None and mode not in (MODE_LINEAR, MODE_ROTARY):
            raise ValueError("mode 必须是 linear 或 rotary")
        try:
            if pulse_per_rev is not None:
                pulse_per_rev = self._coerce_axis_param(
                    pulse_per_rev, 1.0, 10_000.0, "pulse_per_rev"
                )
            if gear_ratio is not None:
                gear_ratio = self._coerce_axis_param(
                    gear_ratio, 0.001, 1_000.0, "gear_ratio"
                )
            if lead_mm is not None:
                lead_mm = self._coerce_axis_param(
                    lead_mm, 0.01, 100.0, "lead_mm"
                )
        except (TypeError, ValueError) as exc:
            raise ValueError(str(exc)) from exc

        def ui() -> dict[str, Any]:
            with app.state_lock:
                moving = (
                    app.running[axis]
                    or app.stepper_in_progress[axis]
                    or app._move_dispatching[axis]
                    or app._move_reservation[axis] is not None
                    or app._web_step_pending[axis] is not None
                    or app._pending_step[axis] is not None
                )
                current = app.axis_profiles[axis]
                changes_profile = (
                    (mode is not None and mode != current.mode)
                    or (
                        pulse_per_rev is not None
                        and pulse_per_rev != current.pulse_per_rev
                    )
                    or (
                        gear_ratio is not None
                        and gear_ratio != current.gear_ratio
                    )
                    or (lead_mm is not None and lead_mm != current.lead_mm)
                )
                if moving and changes_profile:
                    raise RuntimeError("该轴正在运动，停止后才能修改轴配置")

                # Keep the reservation check and all profile mutations under
                # the same lock used by web_stepper_move.  RLock is required
                # here because Tk variable traces call back into the app.
                if pulse_per_rev is not None:
                    app.axis_ppr_var[axis].set(float(pulse_per_rev))
                if gear_ratio is not None:
                    app.axis_gr_var[axis].set(float(gear_ratio))
                if lead_mm is not None:
                    app.axis_lead_var[axis].set(float(lead_mm))
                if (
                    pulse_per_rev is not None
                    or gear_ratio is not None
                    or lead_mm is not None
                ) and not app._on_axis_param_change(axis):
                    raise ValueError("轴参数未能应用")
                if mode is not None:
                    app.axis_mode_var[axis].set(mode)
                    if not app._on_axis_mode_change(axis):
                        raise RuntimeError("轴模式未能应用")
                applied = app.axis_profiles[axis]
                return {
                    "ok": True,
                    "axis": axis,
                    "mode": applied.mode,
                    "pulse_per_rev": applied.pulse_per_rev,
                    "gear_ratio": applied.gear_ratio,
                    "lead_mm": applied.lead_mm,
                }

        try:
            return app._call_ui(ui, timeout=2.0)
        except UiDispatchTimeout as exc:
            raise RuntimeError("GUI 主线程未及时应用轴配置") from exc

    def web_motor_command(
        self,
        mode: str,
        axis: int,
        action: str,
        target_deg: float | None = None,
    ) -> dict[str, Any]:
        app = self._app
        generation = app._ensure_web_control_available()
        mode = str(mode).strip().upper()
        if mode not in (MODE_FOC, MODE_GEAR):
            raise ValueError("模式必须是 FOC 或 GEAR")
        action = str(action).strip().lower()
        with app.state_lock:
            active_mode = app._fw_mode_cache
        if (
            action != "disable"
            and active_mode in (MODE_FOC, MODE_GEAR)
            and mode != active_mode
        ):
            raise RuntimeError(f"当前固件模式是 {active_mode}，不能按 {mode} 控制")
        try:
            axis = int(axis)
        except (TypeError, ValueError):
            raise ValueError("无效电机轴编号") from None
        if not 0 <= axis < NUM_MOTOR_AXES:
            raise ValueError("电机轴编号越界")
        if action == "enable":
            command = build_foc_command(axis, "EN", 1)
        elif action == "disable":
            command = build_foc_command(axis, "EN", 0)
        elif action == "zero":
            command = build_foc_command(axis, "H")
        elif action == "target":
            try:
                amount = float(target_deg)
            except (TypeError, ValueError):
                raise ValueError("target 动作需要有效角度") from None
            if not math.isfinite(amount):
                raise ValueError("target 动作需要有效角度")
            command = build_foc_command(axis, "A", f"{amount:.1f}")
        else:
            raise ValueError("动作必须是 enable/disable/zero/target")
        response = app._send_and_read(
            command,
            timeout=0.8,
            guard=lambda: generation == app._control_generation,
        )
        self._require_confirmation(response, f"OK,{axis}", "闭环电机命令")
        return {
            "ok": True,
            "confirmed": True,
            "mode": mode,
            "axis": axis,
            "action": action,
        }

    def web_track_command(
        self,
        action: str,
        pwm: int = 0,
        lease_ms: int = 0,
    ) -> dict[str, Any]:
        app = self._app
        control_generation = app._ensure_web_control_available()
        action = str(action).strip().lower()
        if action == "stop":
            with app.state_lock:
                app._track_lease_generation += 1
                generation = app._track_lease_generation
                app._track_direction = "STOP"
            response = app._send_track_stop(
                generation,
                control_guard=lambda: (
                    control_generation == app._control_generation
                ),
            )
            self._require_confirmation(response, "OK,TRACK,D", "轨道停止命令")

            def show_stopped() -> None:
                with app.state_lock:
                    if (
                        generation != app._track_lease_generation
                        or app._track_direction != "STOP"
                    ):
                        return
                app.v_track_status.set("已停止（网页）")

            app._post_ui(show_stopped)
            return {"ok": True, "confirmed": True, "action": "stop"}
        if action not in ("forward", "reverse"):
            raise ValueError("动作必须是 forward/reverse/stop")
        direction = "FWD" if action == "forward" else "REV"
        try:
            pwm = max(1, min(100, int(pwm)))
            lease_ms = int(lease_ms) if int(lease_ms) > 0 else 1_000
            lease_ms = max(100, min(5_000, lease_ms))
        except (TypeError, ValueError):
            raise ValueError("PWM/租约参数无效") from None
        with app.state_lock:
            app._track_lease_generation += 1
            generation = app._track_lease_generation
            app._track_direction = direction
            app._track_lease_ms = lease_ms

        def lease_is_current() -> bool:
            with app.state_lock:
                return (
                    generation == app._track_lease_generation
                    and app._track_direction == direction
                    and control_generation == app._control_generation
                )

        def worker() -> None:
            response = app._send_and_read(
                build_track_command(direction, pwm, lease_ms),
                timeout=0.25,
                guard=lease_is_current,
            )
            with app.state_lock:
                current = (
                    generation == app._track_lease_generation
                    and app._track_direction == direction
                )
                if current:
                    app._track_last_response = response
                    if response != "OK,TRACK,D":
                        app._track_direction = "STOP"
            if current and response != "OK,TRACK,D":
                def show_failure() -> None:
                    with app.state_lock:
                        if (
                            generation != app._track_lease_generation
                            or app._track_direction != "STOP"
                            or app._track_last_response != response
                        ):
                            return
                    app.v_track_status.set(
                        f"轨道命令失败: {response or '无响应'}"
                    )

                app._post_ui(show_failure)

        threading.Thread(target=worker, daemon=True).start()
        def show_running() -> None:
            if lease_is_current():
                app.v_track_status.set(
                    f"{direction} · PWM {pwm}% · 网页租约 {lease_ms}ms"
                )

        app._post_ui(show_running)

        def expire_cached_state() -> None:
            with app.state_lock:
                expired = (
                    generation == app._track_lease_generation
                    and app._track_direction == direction
                )
                if expired:
                    app._track_direction = "STOP"
            if expired:
                def show_expired() -> None:
                    with app.state_lock:
                        if (
                            generation != app._track_lease_generation
                            or app._track_direction != "STOP"
                        ):
                            return
                    app.v_track_status.set("网页租约到期，已停止")

                app._post_ui(show_expired)

        timer = threading.Timer(lease_ms / 1_000.0 + 0.1, expire_cached_state)
        timer.daemon = True
        timer.start()
        return {
            "ok": True,
            "accepted": True,
            "action": action,
            "pwm": pwm,
            "lease_ms": lease_ms,
        }

    def web_emergency_stop(self) -> dict[str, Any]:
        app = self._app
        with app._estop_lock:
            app._ensure_web_control_available(allow_estop_retry=True)
            with app.state_lock:
                app._estop_in_progress = True
                app._control_generation += 1
            try:
                if not app._stop_all_outputs(allow_closing=True):
                    raise RuntimeError(
                        "急停命令未获得固件确认；请立即使用物理断电急停"
                    )
                return {"ok": True, "confirmed": True}
            finally:
                with app.state_lock:
                    app._estop_in_progress = False
