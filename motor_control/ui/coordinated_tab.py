"""Desktop view for logical motor binding and honest position monitoring."""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

from motor_control import (
    LOGICAL_ROLE_ORDER,
    LogicalRole,
    MODE_LINEAR,
    ROLE_SPECS,
    stepper_axis_topology,
)
from motor_control.ui.common import PAD


UNBOUND_OPTION = "未绑定"

STATE_TEXT = {
    "UNBOUND": "未绑定",
    "INVALID": "配置无效",
    "DISCONNECTED": "串口未连接",
    "NODE_OFFLINE": "Pico 离线",
    "NODE_UNKNOWN": "Pico 在线状态未知",
    "HARDWARE_ESTOP": "硬件急停",
    "SOFTWARE_ESTOP": "软件急停",
    "ESTOP_UNCONFIRMED": "急停未确认",
    "STOPPING": "停止中",
    "STARTING": "启动中",
    "CONTINUOUS": "连续运动",
    "MOVING": "运动中",
    "ABORTED_UNTRUSTED": "已中止/需校准",
    "IDLE_UNTRUSTED": "空闲/需校准",
    "IDLE": "空闲",
}

STATE_COLORS = {
    "IDLE": "#16803a",
    "MOVING": "#1565c0",
    "CONTINUOUS": "#1565c0",
    "STARTING": "#1565c0",
    "STOPPING": "#b26a00",
    "UNBOUND": "#b26a00",
    "NODE_UNKNOWN": "#b26a00",
    "IDLE_UNTRUSTED": "#b26a00",
    "ABORTED_UNTRUSTED": "#b26a00",
}

BLOCKER_TEXT = {
    "logical_motor_bindings_incomplete_or_invalid": "四个逻辑角色尚未全部正确绑定",
    "direction_and_zero_unverified": "方向与机械零位尚未实机确认",
    "contact_feedback_unavailable": "接触状态反馈不可用",
    "support_load_feedback_unavailable": "支撑载荷/电流反馈不可用",
    "collision_model_unverified": "碰撞包络尚未验证",
    "hardware_limit_chain_incomplete": "硬限位链不完整",
    "coordinated_start_protocol_unavailable": "四轴协调启动协议尚未实现",
    "pico_node_health_incomplete": "Pico 节点在线状态尚未完整缓存",
}


def _route_text(topology):
    if topology["controller"] == "esp32":
        pins = topology["pins"]
        return (
            f"ESP32 本地 · PUL GPIO{pins['pulse']} / DIR GPIO{pins['direction']}"
            if pins
            else "ESP32 本地"
        )
    return (
        f"RS485 · Pico {topology['node']} · "
        f"本地轴 {int(topology['local_axis']) + 1}"
    )


def _axis_option(axis):
    topology = stepper_axis_topology(axis)
    return (
        f"步进轴 {axis} · {topology['label']} · {_route_text(topology)}"
    )


def sync_binding_editor(app, bindings) -> None:
    """Replace the editor with an applied snapshot after an explicit action."""

    for role in LOGICAL_ROLE_ORDER:
        binding = bindings.for_role(role)
        option = (
            UNBOUND_OPTION
            if binding is None
            else app._binding_option_by_axis[binding.axis]
        )
        app.control_binding_vars[role.value].set(option)
    app._binding_editor_dirty = False
    app._binding_editor_revision = bindings.revision
    dirty = app.coordinated_widgets.get("dirty_label")
    if dirty is not None:
        dirty.configure(text="已应用", foreground="#16803a")


def build_coordinated_tab(app, parent) -> None:
    """Build the binding editor and four role cards."""

    parent.columnconfigure(0, weight=1)
    parent.rowconfigure(2, weight=1)
    app.coordinated_widgets = {"roles": {}, "binding_status": {}}

    warning = ttk.LabelFrame(parent, text="安全边界")
    warning.grid(row=0, column=0, sticky="ew", **PAD)
    ttk.Label(
        warning,
        text=(
            "本页只配置逻辑角色绑定并监控，不执行自动 A↔C / S0–S7 换位。\n"
            "位置来自主机脉冲累计，不是编码器实测；未收到 STEP,P 时不伪造运动中位置。"
        ),
        foreground="#b42318",
        justify="left",
    ).pack(anchor="w", padx=8, pady=5)

    binding_frame = ttk.LabelFrame(parent, text="逻辑电机绑定（仅步进轴）")
    binding_frame.grid(row=1, column=0, sticky="ew", **PAD)
    headings = ("角色", "固定动作", "要求模式", "物理步进轴", "校验", "定位")
    for column, text in enumerate(headings):
        ttk.Label(binding_frame, text=text, font=("Microsoft YaHei", 9, "bold")).grid(
            row=0, column=column, sticky="w", padx=4, pady=2
        )
    binding_frame.columnconfigure(3, weight=1)

    options = [UNBOUND_OPTION]
    app._binding_axis_by_option = {}
    app._binding_option_by_axis = {}
    for axis in range(len(app.axis_profiles)):
        option = _axis_option(axis)
        options.append(option)
        app._binding_axis_by_option[option] = axis
        app._binding_option_by_axis[axis] = option

    app.control_binding_vars = {}
    for row, role in enumerate(LOGICAL_ROLE_ORDER, start=1):
        spec = ROLE_SPECS[role]
        ttk.Label(binding_frame, text=role.value, font=("Consolas", 10, "bold")).grid(
            row=row, column=0, sticky="w", padx=4, pady=2
        )
        ttk.Label(binding_frame, text=spec.action).grid(
            row=row, column=1, sticky="w", padx=4, pady=2
        )
        required = "直线 (mm)" if spec.required_mode == MODE_LINEAR else "旋转 (°)"
        ttk.Label(binding_frame, text=required).grid(
            row=row, column=2, sticky="w", padx=4, pady=2
        )
        variable = tk.StringVar(value=UNBOUND_OPTION)
        app.control_binding_vars[role.value] = variable
        combo = ttk.Combobox(
            binding_frame,
            textvariable=variable,
            values=options,
            state="readonly",
            width=58,
        )
        combo.grid(row=row, column=3, sticky="ew", padx=4, pady=2)
        combo.bind("<<ComboboxSelected>>", app._on_binding_editor_change)
        status = ttk.Label(
            binding_frame,
            text="未绑定",
            foreground="#b26a00",
            wraplength=240,
        )
        status.grid(row=row, column=4, sticky="w", padx=4, pady=2)
        app.coordinated_widgets["binding_status"][role.value] = status
        ttk.Button(
            binding_frame,
            text="跳转",
            width=6,
            command=lambda selected_role=role: app._jump_to_binding_axis(selected_role),
        ).grid(row=row, column=5, padx=4, pady=2)

    button_row = ttk.Frame(binding_frame)
    button_row.grid(row=5, column=0, columnspan=6, sticky="ew", padx=4, pady=5)
    ttk.Button(
        button_row, text="填入建议映射 0/2/1/3", command=app._fill_suggested_bindings
    ).pack(side="left", padx=3)
    ttk.Button(
        button_row, text="验证并保存绑定", command=app._save_binding_editor
    ).pack(side="left", padx=3)
    ttk.Button(
        button_row, text="全部解除绑定", command=app._clear_binding_editor
    ).pack(side="left", padx=3)
    dirty_label = ttk.Label(button_row, text="已应用", foreground="#16803a")
    dirty_label.pack(side="right", padx=6)
    app.coordinated_widgets["dirty_label"] = dirty_label

    cards = ttk.LabelFrame(parent, text="四电机状态（软件脉冲估算）")
    cards.grid(row=2, column=0, sticky="nsew", **PAD)
    for column in range(2):
        cards.columnconfigure(column, weight=1, uniform="role")
    for row in range(2):
        cards.rowconfigure(row, weight=1, uniform="role")

    positions = {
        LogicalRole.MUP1.value: (0, 0),
        LogicalRole.MR1.value: (1, 0),
        LogicalRole.MUP2.value: (0, 1),
        LogicalRole.MR2.value: (1, 1),
    }
    for role in LOGICAL_ROLE_ORDER:
        spec = ROLE_SPECS[role]
        row, column = positions[role.value]
        card = ttk.LabelFrame(cards, text=f"{role.value} · {spec.display_name}")
        card.grid(row=row, column=column, sticky="nsew", padx=4, pady=3)
        card.columnconfigure(1, weight=1)
        canvas = tk.Canvas(card, width=92, height=112, bg="#f5f7fa", highlightthickness=0)
        canvas.grid(row=0, column=0, rowspan=4, padx=6, pady=4)
        state = ttk.Label(card, text="未绑定", foreground="#b26a00", font=("Microsoft YaHei", 10, "bold"))
        state.grid(row=0, column=1, sticky="w", padx=4, pady=(4, 1))
        position = ttk.Label(card, text="--", font=("Consolas", 14, "bold"))
        position.grid(row=1, column=1, sticky="w", padx=4)
        detail = ttk.Label(card, text="未绑定物理步进轴", justify="left")
        detail.grid(row=2, column=1, sticky="w", padx=4)
        progress = ttk.Label(card, text="", foreground="#555", justify="left")
        progress.grid(row=3, column=1, sticky="w", padx=4, pady=(1, 4))
        app.coordinated_widgets["roles"][role.value] = {
            "canvas": canvas,
            "state": state,
            "position": position,
            "detail": detail,
            "progress": progress,
        }

    _build_direction_calibration(app, parent)

    readiness = ttk.LabelFrame(parent, text="自主步态就绪度（不替代分阶段人工确认联动）")
    readiness.grid(row=4, column=0, sticky="ew", **PAD)
    ready_label = ttk.Label(
        readiness,
        text="未就绪",
        foreground="#b42318",
        font=("Microsoft YaHei", 10, "bold"),
    )
    ready_label.pack(side="left", padx=8, pady=4)
    blocker_label = ttk.Label(readiness, text="", justify="left")
    blocker_label.pack(side="left", fill="x", expand=True, padx=8, pady=4)
    app.coordinated_widgets["ready_label"] = ready_label
    app.coordinated_widgets["blocker_label"] = blocker_label

    sync_binding_editor(app, app.control_bindings)


# ── 逐电机调试与方向标定（真实方向 vs 驱动方向，2026-09-28）────
# 每个逻辑电机单独：小步点动（轴坐标 +/-）→ 俯视观察机构真实运动 →
# 记录符号。符号写入步态参数并联动零位/标定失效，见 desktop_app。
ROLE_SIGN_FIELD = {
    "Mr1": "mr1_sign",
    "Mr2": "mr2_sign",
    "Mup1": "mup1_lift_sign",
    "Mup2": "mup2_lift_sign",
}


def _direction_sign_text(role_value: str, sign: int) -> str:
    """符号含义文案：+1=轴坐标增大即约定正方向，−1=相反。"""
    rotary = role_value.startswith("Mr")
    positive = "俯视逆时针(ψ增)" if rotary else "抬起"
    negative = "俯视顺时针(ψ减)" if rotary else "下降"
    return f"{sign:+d}：轴+ ⇒ 实际{positive if sign > 0 else negative}"


def _build_direction_calibration(app, parent) -> None:
    frame = ttk.LabelFrame(
        parent, text="逐电机调试与方向标定（真实方向 vs 驱动方向）")
    frame.grid(row=3, column=0, sticky="ew", **PAD)
    headings = ("角色", "当前方向符号", "调试点动（驱动单个电机）",
                "点动“轴+”后机构实际方向（记录）")
    for column, text in enumerate(headings):
        ttk.Label(frame, text=text,
                  font=("Microsoft YaHei", 9, "bold")).grid(
                      row=0, column=column, sticky="w", padx=4, pady=2)
    app.coordinated_widgets["direction"] = {}
    for row, role in enumerate(LOGICAL_ROLE_ORDER, start=1):
        spec = ROLE_SPECS[role]
        ttk.Label(frame, text=f"{role.value}·{spec.display_name}",
                  font=("Consolas", 10, "bold")).grid(
                      row=row, column=0, sticky="w", padx=4, pady=2)
        sign = getattr(app.gait_params, ROLE_SIGN_FIELD[role.value])
        sign_label = ttk.Label(
            frame, text=_direction_sign_text(role.value, sign),
            font=("Consolas", 10, "bold"),
            foreground="#16803a" if sign > 0 else "#b42318")
        sign_label.grid(row=row, column=1, sticky="w", padx=4, pady=2)
        rotary = role.value.startswith("Mr")
        unit, amount = ("°", 10) if rotary else ("mm", 1)
        jog_frame = ttk.Frame(frame)
        jog_frame.grid(row=row, column=2, sticky="w", padx=4, pady=2)
        jog_plus = ttk.Button(
            jog_frame, text=f"⟲ 轴+{amount}{unit}" if rotary else f"▲ 轴+{amount}{unit}",
            width=9,
            command=lambda r=role.value: app._role_debug_move(r, True))
        jog_plus.pack(side="left", padx=2)
        jog_minus = ttk.Button(
            jog_frame, text=f"⟳ 轴−{amount}{unit}" if rotary else f"▼ 轴−{amount}{unit}",
            width=9,
            command=lambda r=role.value: app._role_debug_move(r, False))
        jog_minus.pack(side="left", padx=2)
        record_frame = ttk.Frame(frame)
        record_frame.grid(row=row, column=3, sticky="w", padx=4, pady=2)
        record_positive = ttk.Button(
            record_frame,
            text="俯视逆时针" if rotary else "抬起",
            width=10,
            command=lambda r=role.value: app._gait_record_direction_sign(r, True))
        record_positive.pack(side="left", padx=2)
        record_negative = ttk.Button(
            record_frame,
            text="俯视顺时针" if rotary else "下降",
            width=10,
            command=lambda r=role.value: app._gait_record_direction_sign(r, False))
        record_negative.pack(side="left", padx=2)
        app.coordinated_widgets["direction"][role.value] = {
            "sign": sign_label,
            "jog_plus": jog_plus,
            "jog_minus": jog_minus,
            "record_positive": record_positive,
            "record_negative": record_negative,
        }
    ttk.Label(
        frame, foreground="#555", justify="left", wraplength=760,
        text=(
            "每台电机单独标定：① 串口连接后点【轴+】小步驱动（旋转 10°/直线 1mm，"
            "按钮箭头是程序判定的驱动方向）；② 俯视观察机构真实运动；"
            "③ 点对应【实际…】按钮记录符号（+1=轴坐标增大即真实正向，−1=相反）。"
            "符号变更立即保存并使步态标定失效；Mr 符号变更后该侧零位作废，"
            "须重新记零。步态执行中四轴被占用，点动与改符号均被拒绝。"
        ),
    ).grid(row=5, column=0, columnspan=4, sticky="w", padx=6, pady=(2, 4))


def _refresh_direction_calibration(app, by_role) -> None:
    """按快照刷新方向标定区：符号文本/颜色 + 点动可用性（绑定有效且空闲）。"""

    direction = app.coordinated_widgets.get("direction") or {}
    params = getattr(app, "gait_params", None)
    for role in LOGICAL_ROLE_ORDER:
        widgets = direction.get(role.value)
        if widgets is None or params is None:
            continue
        sign = getattr(params, ROLE_SIGN_FIELD[role.value])
        widgets["sign"].configure(
            text=_direction_sign_text(role.value, sign),
            foreground="#16803a" if sign > 0 else "#b42318")
        snapshot = by_role.get(role.value, {})
        movable = (bool(snapshot.get("binding_valid"))
                   and snapshot.get("state") == "IDLE")
        for key in ("jog_plus", "jog_minus"):
            widgets[key].configure(state="normal" if movable else "disabled")


def _draw_linear(canvas, snapshot):
    canvas.delete("all")
    width, height = 92, 112
    canvas.create_rectangle(37, 9, 55, height - 9, outline="#94a3b8", width=2)
    position = snapshot.get("position")
    minimum = snapshot.get("travel_min")
    maximum = snapshot.get("travel_max")
    valid_range = (
        isinstance(position, (int, float))
        and isinstance(minimum, (int, float))
        and isinstance(maximum, (int, float))
        and math.isfinite(float(position))
        and math.isfinite(float(minimum))
        and math.isfinite(float(maximum))
        and maximum > minimum
    )
    if valid_range:
        fraction = max(0.0, min(1.0, (position - minimum) / (maximum - minimum)))
        y = height - 9 - fraction * (height - 18)
        canvas.create_rectangle(39, y, 53, height - 11, fill="#2f80ed", outline="")
        canvas.create_line(25, y, 67, y, fill="#1565c0", width=2)
        canvas.create_text(46, 4, text="max", anchor="s", fill="#64748b", font=("Arial", 7))
        canvas.create_text(46, height - 3, text="min", anchor="n", fill="#64748b", font=("Arial", 7))
    else:
        canvas.create_text(
            width / 2,
            height / 2,
            text="无有效\n行程范围",
            justify="center",
            fill="#64748b",
            font=("Microsoft YaHei", 9),
        )


def _draw_rotary(canvas, snapshot):
    canvas.delete("all")
    center_x, center_y, radius = 46, 55, 36
    canvas.create_oval(
        center_x - radius,
        center_y - radius,
        center_x + radius,
        center_y + radius,
        outline="#94a3b8",
        width=2,
    )
    for degree in range(0, 360, 45):
        angle = math.radians(degree - 90)
        x1 = center_x + (radius - 5) * math.cos(angle)
        y1 = center_y + (radius - 5) * math.sin(angle)
        x2 = center_x + radius * math.cos(angle)
        y2 = center_y + radius * math.sin(angle)
        canvas.create_line(x1, y1, x2, y2, fill="#64748b")
    position = snapshot.get("position")
    if isinstance(position, (int, float)) and math.isfinite(float(position)):
        angle = math.radians((float(position) % 360.0) - 90.0)
        x = center_x + (radius - 9) * math.cos(angle)
        y = center_y + (radius - 9) * math.sin(angle)
        canvas.create_line(center_x, center_y, x, y, fill="#1565c0", width=3)
        canvas.create_oval(center_x - 3, center_y - 3, center_x + 3, center_y + 3, fill="#1565c0", outline="")
        canvas.create_text(center_x, 101, text=f"{float(position) % 360.0:.1f}°", fill="#334155", font=("Consolas", 9))
    else:
        canvas.create_text(center_x, center_y, text="--", fill="#64748b", font=("Consolas", 12))


def refresh_coordinated_tab(app, control, snapshots) -> None:
    """Render a previously captured controller snapshot on the Tk thread."""

    by_role = {item["role"]: item for item in snapshots}
    _refresh_direction_calibration(app, by_role)
    for role in LOGICAL_ROLE_ORDER:
        snapshot = by_role[role.value]
        widgets = app.coordinated_widgets["roles"][role.value]
        status_widget = app.coordinated_widgets["binding_status"][role.value]
        state = str(snapshot.get("state", "INVALID"))
        color = STATE_COLORS.get(state, "#b42318")
        widgets["state"].configure(
            text=STATE_TEXT.get(state, state), foreground=color
        )
        # A periodic live refresh must not erase validation feedback for an
        # unsaved draft in the binding editor.
        if not getattr(app, "_binding_editor_dirty", False):
            status_widget.configure(
                text=("有效" if snapshot.get("binding_valid") else snapshot.get("binding_error") or "无效"),
                foreground="#16803a" if snapshot.get("binding_valid") else "#b42318",
            )

        position = snapshot.get("position")
        unit = snapshot.get("unit") or ("mm" if role.value.startswith("Mup") else "°")
        trusted = bool(snapshot.get("position_trusted"))
        if isinstance(position, (int, float)) and math.isfinite(float(position)):
            prefix = "" if trusted else "≈ "
            widgets["position"].configure(
                text=f"{prefix}{float(position):.3f} {unit}",
                foreground="#1565c0" if trusted else "#b26a00",
            )
        else:
            widgets["position"].configure(text="--", foreground="#555")

        binding = snapshot.get("binding")
        if binding:
            axis = binding["axis"]
            route = (
                f"ESP32 本地"
                if snapshot.get("controller") == "esp32"
                else f"Pico {snapshot.get('node')} / 本地轴 {int(snapshot.get('local_axis', 0)) + 1}"
            )
            detail = (
                f"步进轴 {axis} · {snapshot.get('physical_label')} · {route}\n"
                f"模式 {snapshot.get('mode')} · 软件脉冲估算 · 非实测"
            )
        else:
            detail = "未绑定物理步进轴"
        widgets["detail"].configure(text=detail)

        requested = snapshot.get("requested_steps")
        executed = snapshot.get("executed_steps")
        percent = snapshot.get("progress_percent")
        target = snapshot.get("target_position")
        if percent is not None:
            progress = f"进度 {float(percent):.1f}% · {executed}/{requested} 脉冲"
        elif snapshot.get("in_progress"):
            progress = "运动中，暂无中间进度帧"
        else:
            progress = "空闲"
        if isinstance(target, (int, float)):
            progress += f" · 目标 {float(target):.3f} {unit}"
        if snapshot.get("stale"):
            progress += " · 数据陈旧"
        widgets["progress"].configure(text=progress)

        if role.value.startswith("Mup"):
            _draw_linear(widgets["canvas"], snapshot)
        else:
            _draw_rotary(widgets["canvas"], snapshot)

    blockers = [BLOCKER_TEXT.get(code, code) for code in control.get("blockers", [])]
    app.coordinated_widgets["ready_label"].configure(
        text="就绪" if control.get("automation_ready") else "未就绪",
        foreground="#16803a" if control.get("automation_ready") else "#b42318",
    )
    app.coordinated_widgets["blocker_label"].configure(
        text="；".join(blockers)
    )


__all__ = [
    "UNBOUND_OPTION",
    "build_coordinated_tab",
    "refresh_coordinated_tab",
    "sync_binding_editor",
]
