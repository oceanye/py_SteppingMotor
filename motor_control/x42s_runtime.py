"""X42S desktop runtime seam: manual dispatch, unit conversion and telemetry.

The host retains locks, serial ownership, UI dispatch, diagnostics and gait state.
No window or transport is created here; inherited method names remain stable.
"""
import math
import time
from dataclasses import replace
from tkinter import messagebox

from .axis_math import MODE_LINEAR, MODE_ROTARY
from .coordinated_control import BACKEND_X42S_UART, LOGICAL_ROLE_ORDER, LogicalRole, ROLE_SPECS
from .actuator_model import X42sActuatorState
from .diagnostics import logged_action
from .protocol import (
    OkReply, X42sMoveAck, X42sStatusReply, build_x42s_enable_command,
    build_x42s_move_command, build_x42s_status_command, build_x42s_stop_command,
    build_x42s_zero_command, parse_line,
)
from .x42s_units import (
    X42S_AXIS_HANDLE_BASE, X42S_PULSE_PER_REV, X42S_SPEED_PPS_MAX,
    motor_degrees_to_role, role_move_to_pulses, role_to_output_degrees,
    output_degrees_to_pulses,
)
from .x42s_motion import endpoint_tolerance

X42S_DEBUG_JOG_DEG_S = 8.4


class X42sRuntimeMixin:
    def _query_x42s_firmware_info(self):
        response = self._send_and_read("X42S,INFO", timeout=.5, soft_timeout=True)
        reply = parse_line(response) if response else None
        known = (isinstance(reply, OkReply) and len(reply.values) == 4
                 and reply.values[:2] == ("X42S", "INFO") and reply.values[2] == "2")
        self._x42s_firmware_info = {"verified_revision": known, "reply": response or None}
        recorder = getattr(self, "diagnostics", None)
        if recorder is not None:
            recorder.event("motor", "firmware.x42s_info", **self._x42s_firmware_info,
                           level="INFO" if known else "WARNING")
        self.log(("X42S 固件能力/构建："+response) if known else
                 "⚠️ 未识别现场X42S固件构建：请核对/更新ESP32固件；PC版本不等于已烧录版本，旧速度0不证明静止")

    def _on_x42s_manual_role_change(self):
        """手动调试区角色切换：单位标签/范围/默认值随角色形态切换。"""

        from motor_control.ui.coordinated_tab import apply_manual_debug_units
        apply_manual_debug_units(self)


    @logged_action
    def _x42s_manual_action(self, action):
        """UI 线程入口：action ∈ enable/disable/plus/minus/stop/zero。

        输入单位按角色形态（2026-10-13 双形态电机）：旋转=输出轴°/°/s；
        直线=机构 mm/mm/s——这里经绑定 lead_mm 预换算为输出轴角度后
        交给 worker，worker 语义（输出轴角度→脉冲）保持不变。
        """
        title = "X42S 手动调试"
        if action == "zero":
            messagebox.showinfo(title, "步态不需要驱动器归零；请在步态页【确认摆放并记录双侧基准】。")
            return
        if getattr(self, "_gait_owned", {}):
            if action == "stop":
                self._gait_abort_clicked()
            else:
                messagebox.showwarning(title, "步态已占用四轴，请先完成或取消计划，再做手动调试")
            return
        if self.control_bindings.backend != BACKEND_X42S_UART:
            messagebox.showinfo(title, "当前后端不是 x42s_uart：请先在绑定页切换后端。")
            return
        if not self._is_serial_connected():
            messagebox.showwarning(title, "串口未连接：请先连接 ESP32。")
            return
        if action in ("plus", "minus") and (
                getattr(self, "_active_control_workers", 0)
                or getattr(self, "_x42s_manual_pending", {})):
            messagebox.showwarning(title, "上一条控制命令/点动尚未确认完成；等待实测到位，"
                                   "或先点【停止】确认停车，不需要归零。")
            return
        role_value = self.x42s_manual_role_var.get()
        # 2026-10-20：双升降同步（"Mup1+Mup2"=两台丝杆同动，调试回位用）。
        dual_lift = role_value == "Mup1+Mup2"
        linear_lead = None
        if dual_lift:
            role = binding = None
        else:
            role = LogicalRole(role_value)
            binding = self.control_bindings.for_role(role)
        # 全部 X42S 绑定目标（2026-10-15 台架教训：驱动器使能是单台易失
        # 的，断开串口/驱动器断电即清空——使能/失能改为一键全部，换台测
        # 试不再因漏点某台使能而 E2 弹窗）。
        all_targets = []
        for each_role in LOGICAL_ROLE_ORDER:
            each = self.control_bindings.for_role(each_role)
            if each is not None and each.kind == "x42s":
                all_targets.append(
                    (each_role.value, each.driver_id, each.gear_ratio))
        if action in ("enable", "disable"):
            if not all_targets:
                messagebox.showwarning(title, "没有任何角色绑定 X42S 驱动器。")
                return
            targets = all_targets
            is_linear = ROLE_SPECS[LogicalRole(targets[0][0])].required_mode == MODE_LINEAR
        elif dual_lift:
            lifts = []
            for each in ("Mup1", "Mup2"):
                each_binding = self.control_bindings.for_role(LogicalRole(each))
                if each_binding is None or each_binding.kind != "x42s":
                    messagebox.showwarning(title, f"{each} 未绑定 X42S 驱动器。")
                    return
                lifts.append((each, each_binding.driver_id,
                              each_binding.gear_ratio, each_binding.lead_mm))
            if action in ("plus", "minus") and not math.isclose(lifts[0][3], lifts[1][3]):
                messagebox.showwarning(
                    title, f"Mup1/Mup2 导程不同（{lifts[0][3]:g}/{lifts[1][3]:g}mm），"
                    "暂不支持双侧联动。")
                return
            targets = [(r, i, g) for r, i, g, _l in lifts]
            linear_lead = lifts[0][3]
            is_linear = True
        else:
            if binding is None or binding.kind != "x42s":
                messagebox.showwarning(title, f"{role_value} 未绑定 X42S 驱动器。")
                return
            targets = [(role_value, binding.driver_id, binding.gear_ratio)]
            is_linear = ROLE_SPECS[role].required_mode == MODE_LINEAR
            if is_linear:
                linear_lead = binding.lead_mm
        unit = "mm" if is_linear else "°"

        angle_deg = speed_deg_s = None
        if action in ("plus", "minus"):
            try:
                value = float(self.x42s_manual_angle_var.get())
                speed = float(self.x42s_manual_speed_var.get())
            except ValueError:
                messagebox.showwarning(title, "位移/速度必须是数字。")
                return
            if not (math.isfinite(value) and math.isfinite(speed)):
                messagebox.showwarning(title, "位移/速度必须是有限数字。")
                return
            limit = 500.0 if is_linear else 360.0
            if abs(value) > limit or value == 0.0:
                messagebox.showwarning(
                    title, f"位移需在 ±{limit:g}{unit} 内且不为 0"
                    + ("（机构行程）" if is_linear else "（输出轴角度）") + "。")
                return
            if speed <= 0.0:
                messagebox.showwarning(
                    title, "速度需为正（机构 mm/s）。" if is_linear
                    else "速度需为正（输出轴 °/s）。")
                return
            angle_deg = role_to_output_degrees(value, linear=is_linear, lead_mm=linear_lead)
            speed_deg_s = role_to_output_degrees(speed, linear=is_linear, lead_mm=linear_lead)
        if self._start_control_worker(
            self._x42s_manual_worker, action, targets, angle_deg, speed_deg_s,
        ) is None:
            self.log("⛔ 控制器忙，稍后再操作 X42S")


    def _x42s_manual_worker(self, action, targets, angle_deg, speed_deg_s):
        """控制工作线程：构造命令发往 ESP32，校验应答类型后报日志。

        targets: [(role_value, driver_id, gear_ratio), ...]
        enable/disable 逐台发全部绑定（一键全部使能/失能，台间留静默
        间隔——驱动器背靠背连发命令帧会整批丢帧）；其余动作只作用于
        目标列表（单台或双升降）。双升降为顺序下发，非硬同步。
        """
        if action in ("plus", "minus"):
            # All conversions/encodings must succeed BEFORE the first MOVE.
            failures = []
            prepared = []
            try:
                if not targets or len({i for _, i, _ in targets}) != len(targets):
                    raise ValueError("目标为空或驱动器地址重复")
                for role_value, driver_id, gear in targets:
                    steps, speed_pps = output_degrees_to_pulses(
                        angle_deg if action == "plus" else -angle_deg, speed_deg_s, gear)
                    if not steps:
                        raise ValueError(f"{role_value} 位移不足 1 脉冲")
                    if abs(steps) > 20000000 or speed_pps > X42S_SPEED_PPS_MAX:
                        raise ValueError(f"{role_value} 位移/速度超限（{steps} pulse，{speed_pps} pps）")
                    command = build_x42s_move_command(driver_id, steps, speed_pps)
                    prepared.append((role_value, driver_id, steps, speed_pps, command))
            except ValueError as exc:
                detail = str(exc)
                self._post_ui(lambda: messagebox.showwarning(
                    "X42S 手动调试", f"整组未下发：{detail}"))
                return
            attempted = []
            for role_value, driver_id, steps, speed_pps, command in prepared:
                attempted.append((role_value, driver_id))
                ok, reason = self._send_manual_x42s(
                    command, role_value, driver_id, steps, speed_pps)
                if not ok:
                    failures.append(f"{role_value}·ID{driver_id} → {reason}")
                    # A lost ACK may mean the motor DID start. Stop every
                    # attempted target, including the failed one; never carry
                    # on with a partial two-lift move.
                    for stop_role, stop_id in attempted:
                        stopped, why = self._send_manual_x42s(
                            build_x42s_stop_command(stop_id), stop_role, stop_id)
                        if not stopped:
                            failures.append(f"ID{stop_id} 停车未确认：{why}；请使用硬件急停")
                    break
                time.sleep(0.05)
            if failures:
                detail = "\n".join(failures)
                self._post_ui(lambda: messagebox.showwarning(
                    "X42S 手动调试", f"未被接受的命令：\n{detail}"))
            return
        if action in ("enable", "disable"):
            word = "使能" if action == "enable" else "失能"
            failures = []
            for role_value, driver_id, _gear in targets:
                command = build_x42s_enable_command(
                    driver_id, action == "enable")
                ok, reason = self._send_manual_x42s(
                    command, role_value, driver_id)
                if not ok:
                    failures.append(f"{role_value}·ID{driver_id} → {reason}")
                time.sleep(0.05)   # 台间静默间隔
            if failures:
                detail = "\n".join(failures)
                self._post_ui(lambda: messagebox.showwarning(
                    "X42S 手动调试",
                    f"{word}失败的驱动器：\n{detail}"))
            return
        # stop：逐台（双升降同步时停两台；单台=1 项）。
        if action == "stop":
            failures = []
            for role_value, driver_id, _gear in targets:
                command = build_x42s_stop_command(driver_id)
                ok, reason = self._send_manual_x42s(command, role_value, driver_id)
                if not ok:
                    failures.append(f"{role_value}·ID{driver_id} → {reason}")
                time.sleep(0.05)
            if failures:
                detail = "\n".join(failures)
                self._post_ui(lambda: messagebox.showwarning(
                    "X42S 手动调试", f"未被接受的命令：\n{detail}"))
            return
        # zero：单台（action 层已提示改用步态页基准，此路仅防御）。
        role_value, driver_id, _gear = targets[0]
        if action == "zero":
            command = build_x42s_zero_command(driver_id)
        else:  # pragma: no cover - 按钮不会产生其他值
            return
        ok, reason = self._send_manual_x42s(command, role_value, driver_id)
        if not ok:
            self._post_ui(lambda: messagebox.showwarning(
                "X42S 手动调试", f"命令未被接受：\n{command}\n→ {reason}"))


    def _send_manual_x42s(self, command, role_value, driver_id,
                          steps=None, speed_pps=None):
        """发一条手动调试命令并校验应答（MOVE 按台号+步数认领 ACK，其余
        按完整子命令认领 OK 行），报日志；返回 (ok, reason)，弹窗由调用方决定。"""
        if command.endswith(",ZERO"):
            return False, "常规调试不发送驱动器ZERO；请使用步态页双侧基准"
        def allowed():
            return (not getattr(self, "_gait_owned", {})
                    and not getattr(self, "_gait_baseline_recording", False))
        if not allowed():
            return False, "步态执行/基准记录占用四轴，手动命令未发送"
        if steps is not None:
            with self.state_lock:
                pending = getattr(self, "_x42s_manual_pending", None)
                if pending is None:
                    pending = self._x42s_manual_pending = {}
                if driver_id in pending:
                    return False, "上一条 X42S 点动尚未确认到位/停车"
                state = getattr(self, "_x42s_states", {}).get(role_value)
                origin = (state.measured_position if state is not None
                          and state.driver_id == driver_id and state.telemetry_trusted else None)
                pending[driver_id] = {
                    "sent_at": time.monotonic(),
                    "acknowledged": False,
                    "target": None if origin is None else origin + steps * 360 / X42S_PULSE_PER_REV,
                    "tolerance": endpoint_tolerance(role_value, self.control_bindings.for_role(LogicalRole(role_value))),
                }
                # Invalidate on the command attempt, even if it returns to the
                # same coordinate between GUI frames or its ACK is lost.
                run = getattr(self, "_gait_run", None)
                if run is not None:
                    run.twin_manual_invalid = True
        resp = self._send_and_read(command, soft_timeout=True, guard=allowed)
        reply = parse_line(resp) if resp else None
        if steps is None:
            ok = (isinstance(reply, OkReply)
                  and reply.raw == "OK," + command)
        else:
            ok = (isinstance(reply, X42sMoveAck)
                  and reply.driver_id == driver_id
                  and reply.steps == steps and reply.speed == speed_pps)
            if ok:
                with self.state_lock:
                    move = getattr(self, "_x42s_manual_pending", {}).get(driver_id)
                    if move is not None:
                        move.update(acknowledged=True, sent_at=time.monotonic())
        if ok and command.endswith(",STOP"):
            with self.state_lock:
                getattr(self, "_x42s_manual_pending", {}).pop(driver_id, None)
        reason = "无应答（超时或请求已取消）；请检查连接及驱动器状态" if reply is None else reply.raw
        self._post_ui(lambda: self.log(
            ("✅" if ok else "⛔")
            + f" X42S {role_value}·ID{driver_id} {command}"
            + ("" if ok else f" → {reason}")))
        return ok, reason


    def _x42s_move_conversion(self, role, delta, speed):
        """机构坐标位移/速度 → (电机轴脉冲, pps)；速度取绝对值。"""

        binding = self.control_bindings.for_role(role)
        if binding is None or binding.kind != "x42s":
            return None, None
        try:
            return role_move_to_pulses(
                delta, speed, linear=ROLE_SPECS[role].required_mode == MODE_LINEAR,
                gear_ratio=binding.gear_ratio, lead_mm=binding.lead_mm)
        except ValueError:
            return None, None


    def _x42s_role_output_units(self, role):
        """角色机构坐标实测值（Mr=输出轴°，Mup=换算 mm）；未知返回 None。

        §12：实测位置缺失（掉线/本轮未上报）返回 None，绝不回退 0。
        """
        binding = self.control_bindings.for_role(role)
        if binding is None or binding.kind != "x42s":
            return None
        with self.state_lock:
            state = self._x42s_states.get(role.value)
        if (state is None or state.driver_id != binding.driver_id
                or not state.telemetry_trusted or state.fault is not None):
            return None
        return motor_degrees_to_role(
            state.measured_position, linear=ROLE_SPECS[role].required_mode == MODE_LINEAR,
            gear_ratio=binding.gear_ratio, lead_mm=binding.lead_mm)


    def _role_debug_move_x42s(self, role_name, handle, axis_positive,
                              amount, unit):
        """X42S 单角色点动：机构坐标 ± 小步换算输出轴角度后发 MOVE。

        与手动调试区共用 worker 与应答校验；点动速度取输出轴 8.4°/s
        （台架验证过的手动默认值），Mr/Mup 同速。未使能时提示先使能。
        """
        if getattr(self, "_gait_owned", {}):
            self.log("⛔ 步态已占用四轴；请先完成或取消计划，再点动")
            return
        driver_id = handle - X42S_AXIS_HANDLE_BASE
        binding = self.control_bindings.for_role(LogicalRole(role_name))
        signed_amount = amount if axis_positive else -amount
        out_deg = (signed_amount if unit == "°"
                   else signed_amount * 360.0 / binding.lead_mm)
        with self.state_lock:
            state = self._x42s_states.get(role_name)
        if (state is not None and state.driver_id == driver_id
                and state.online and not state.enabled):
            self.log(f"⚠️ {role_name}（X42S ID{driver_id}）当前未使能；"
                     "请先在绑定页 X42S 手动调试区【使能】再点动")
            return
        drive_text = (self._role_positive_direction_text(role_name)
                      if axis_positive
                      else self._role_negative_direction_text(role_name))
        est_s = abs(out_deg) / X42S_DEBUG_JOG_DEG_S
        self.log(f"🔧 {role_name} 调试点动：X42S ID{driver_id} 机构坐标"
                 f"{'+' if axis_positive else '−'}{amount:g}{unit}"
                 f"（输出轴 {out_deg:+.1f}°，约 {est_s:.1f}s；"
                 f"程序驱动方向={drive_text}；请观察机构真实方向）")
        # targets: [(role, driver_id, gear)] 单台列表（57a720c worker 签名；
        # 2026-10-16 修正：此处仍按旧签名传 6 参，worker 线程启动即
        # TypeError，点动无任何反应也无提示——gui_2026-10-16.log 实证）。
        if self._start_control_worker(
            self._x42s_manual_worker, "plus",
            [(role_name, driver_id, binding.gear_ratio)],
            out_deg, X42S_DEBUG_JOG_DEG_S,
        ) is None:
            self.log("⛔ 控制器忙，稍后再操作 X42S 点动")


    def _x42s_poll_loop(self, generation, session):
        while (self.x42s_poll_running
               and self._serial_source_is_current(generation, session)):
            with self.state_lock:
                bindings = self.control_bindings
                polled = [
                    (role, binding.driver_id)
                    for role in LOGICAL_ROLE_ORDER
                    if (binding := bindings.for_role(role)) is not None
                    and binding.kind == "x42s"
                ]
            if not polled:
                time.sleep(self.X42S_POLL_INTERVAL_S)
                continue
            for role, driver_id in polled:
                if not (self.x42s_poll_running
                        and self._serial_source_is_current(generation, session)):
                    break
                # 只读状态轮询用软超时：总线瞬时无应答只跳过本轮。
                resp = self._send_and_read(
                    build_x42s_status_command(driver_id),
                    guard=lambda: self._serial_source_is_current(
                        generation, session
                    ),
                    soft_timeout=True,
                )
                if resp:
                    message = parse_line(resp)
                    # 固件可能回 ERR（如 x42s uart disabled）或错配行；
                    # 只有台号对上的遥测行才计入状态。
                    if (isinstance(message, X42sStatusReply)
                            and message.driver_id == driver_id):
                        self._apply_x42s_reply(role, driver_id, message)
                time.sleep(self.X42S_POLL_INTERVAL_S / len(polled))
            time.sleep(0.05)


    def _x42s_preflight_state(self, role, driver_id):
        """Re-read an aged online sample before start/zero checks.

        The PC background loop is deliberately slow (about 2Hz). The current
        cache freshness window is 1500ms, allowing staggered four-axis polling.
        Re-read only the expired sample, then keep the existing safety
        checks; a timeout must never make the previous position fresh again.
        Firmware S returns its cache, not a forced UART sample: this can still
        fail the freshness gate until firmware polling has produced a new sample.
        """
        with self.state_lock:
            state = self._x42s_states.get(role.value)
        if (state is not None and state.driver_id == driver_id and state.online
                and not state.telemetry_trusted):
            resp = self._send_and_read(
                build_x42s_status_command(driver_id), soft_timeout=True)
            reply = parse_line(resp) if resp else None
            if isinstance(reply, X42sStatusReply) and reply.driver_id == driver_id:
                self._apply_x42s_reply(role, driver_id, reply)
            else:
                return None  # Do not accept a cache update from another poll.
            with self.state_lock:
                state = self._x42s_states.get(role.value)
        return state


    def _apply_x42s_reply(self, role, driver_id, reply):
        """合并一帧遥测；保留 online/age/valid 的真实含义（§12）。"""

        if reply.driver_id != driver_id:
            return  # 错台号回复不得并入该角色（调用方已查，这里兜底）
        fields = {
            "enabled": reply.enabled,
            "reached": reply.reached,
            "fault": reply.fault,
            "online": reply.online,
            "last_update_monotonic": time.monotonic() - reply.age_ms / 1000.0,
        }
        for name in ("command_position", "measured_position",
                     "position_error", "speed", "current_a", "temperature_c"):
            # 本轮无效必须清空；不能给上一轮位置/速度刷新时间戳。
            fields[name] = getattr(reply, name)
        with self.state_lock:
            binding = self.control_bindings.for_role(role)
            if (binding is None or binding.kind != "x42s"
                    or binding.driver_id != driver_id):
                return  # 查询在途时已换绑定，旧回复不得覆盖新角色。
            current = self._x42s_states.get(role.value)
            base = (
                current
                if current is not None and current.driver_id == driver_id
                else X42sActuatorState(role=role, driver_id=driver_id)
            )
            state = self._x42s_states[role.value] = replace(base, **fields)
            pending = getattr(self, "_x42s_manual_pending", {})
            move = pending.get(driver_id)
            # ACK != DONE. Reject the old reached=1 cached just after dispatch;
            # require a new measured endpoint (2 pulses incl. 0.1deg quantization).
            if (move is not None and move.get("acknowledged") and move["target"] is not None
                    and state.last_update_monotonic > move["sent_at"] + 0.45
                    and state.telemetry_trusted and state.enabled
                    and state.reached is True and state.fault is None
                    and (state.speed is None or abs(state.speed) <= .01)
                    and abs(state.measured_position - move["target"]) <= move.get("tolerance", 720 / X42S_PULSE_PER_REV)):
                pending.pop(driver_id, None)
        recorder = getattr(self, "diagnostics", None)
        if recorder is not None:
            position = motor_degrees_to_role(
                reply.measured_position, linear=ROLE_SPECS[role].required_mode == MODE_LINEAR,
                gear_ratio=binding.gear_ratio, lead_mm=binding.lead_mm)
            recorder.event("telemetry", "x42s.telemetry", role=role.value, driver_id=driver_id,
                           motor_deg=reply.measured_position, command_motor_deg=reply.command_position,
                           output_position=position,
                           unit="deg" if ROLE_SPECS[role].required_mode == MODE_ROTARY else "mm",
                           gear_ratio=binding.gear_ratio, lead_mm=binding.lead_mm,
                           error_motor_deg=reply.position_error, speed_motor_rpm=reply.speed,
                           current_a=reply.current_a, temperature_c=reply.temperature_c,
                           online=reply.online, enabled=reply.enabled, reached=reply.reached,
                           fault=reply.fault, age_ms=reply.age_ms, raw=reply.raw)
