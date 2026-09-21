"""Desktop view for the tripod gait: calibration, dry-run preview, staged run.

页面分两列：左侧运行/标定/几何参考分页，右侧干跑预览
（俯视 Canvas）与 S0–S7 阶段推进。所有回调都在 ``desktop_app`` 上，
本模块只负责构建与刷新控件。
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from dataclasses import replace

from motor_control.gait_planner import GaitParams
from motor_control.ui.common import PAD
from motor_control.ui.gait_twin import build_twin_panel, refresh_twin_panel

GEOMETRY_FIELDS = (
    ("d_mm", "中心距 d (mm)"),
    ("arm_length_mm", "爪臂长/节点环半径 (mm)"),
    ("hub_radius_mm", "壳体等水平包络半径 (mm)"),
    ("arm_radius_mm", "爪臂等效半径 (mm)"),
    ("node_radius_mm", "高节点等效半径 (mm)"),
    ("high_node_height_mm", "高低节点高差 (mm)"),
    ("body_drop_mm", "壳体等最低点下伸 (mm)"),
    ("beam_height_mm", "横梁中心线高度 (mm)"),
    ("beam_radius_mm", "横梁/连接件包络半径 (mm)"),
)
GAP_FIELDS = (("safety_margin_mm", "参考安全间隙 δ (mm)"),)
RUN_FIELDS = (
    ("phase_gain", "自转/公转增益 k（默认-2）"),
    ("swing_speed_deg_s", "公转峰值速度 (°/s)"),
    ("rotation_limit_deg", "线缆角度限位 ±(°)"),
    ("lift_mm", "实际抬足行程 (mm)"),
    ("lift_speed_mm_s", "抬足速度 (mm/s)"),
    ("settle_speed_mm_s", "落足速度 (mm/s)"),
)
CALIBRATION_FIELDS = (
    ("beam_reference_deg", "记零时横梁世界角 β₀ (°)"),
)
PREVIEW_FIELDS = (
    ("swing_segments", "预览爪臂展示数量"),
    ("feasibility_samples", "间隙估算采样数"),
)
BEAT_FIELDS = RUN_FIELDS + CALIBRATION_FIELDS + PREVIEW_FIELDS
SIGN_FIELDS = (
    ("mr1_sign", "Mr1 旋转方向", "轴坐标增大 = q 增大"),
    ("mr2_sign", "Mr2 旋转方向", "轴坐标增大 = q 增大"),
    ("mup1_lift_sign", "Mup1 抬升方向", "轴坐标增大 = 抬起"),
    ("mup2_lift_sign", "Mup2 抬升方向", "轴坐标增大 = 抬起"),
)
INTEGER_FIELDS = {"swing_segments", "feasibility_samples"}

PREVIEW_MARGIN = 16


def build_gait_tab(app, parent) -> None:
    """Build calibration params, dry-run preview and stage controls."""

    parent.columnconfigure(0, weight=1)
    parent.rowconfigure(1, weight=1)
    app.gait_widgets = {}
    app.gait_field_vars = {}
    app.gait_sign_vars = {}
    app.gait_side_var = tk.StringVar(value="left")
    app.gait_zero_vars = {
        "Mr1": tk.StringVar(value="未标定"),
        "Mr2": tk.StringVar(value="未标定"),
    }
    app.gait_report_var = tk.StringVar(value="尚未干跑")
    app.gait_stage_var = tk.StringVar(value="未开始")
    app.gait_state_var = tk.StringVar(value="")
    app.gait_calibrated_var = tk.BooleanVar(value=False)
    app._gait_loading_fields = False
    app._gait_last_report = None

    warning = ttk.LabelFrame(parent, text="角度联动与实机执行")
    warning.grid(row=0, column=0, sticky="ew", **PAD)
    notice = ttk.Label(
        warning,
        text=(
            "旋转联动只按角度计算；中心距和六边形尺寸不改变电机转角。实机按 S0–S7 逐阶段确认，"
            "旋转轴需支持 SYNC 的同一 ESP32。几何参考仅作间隙提示；接触和支撑状态仍需现场确认。"
        ),
        foreground="#334155", wraplength=680,
        justify="left",
    )
    notice.pack(fill="x", padx=8, pady=5)
    warning.bind("<Configure>", lambda e: notice.configure(wraplength=max(200, e.width - 24)))

    panes = tk.PanedWindow(parent, orient="horizontal", sashwidth=6,
                          borderwidth=0, opaqueresize=True)
    panes.grid(row=1, column=0, sticky="nsew", **PAD)
    left, right = ttk.Frame(panes), ttk.Frame(panes)
    panes.add(left, minsize=350, width=350, stretch="always")
    panes.add(right, minsize=260, width=500, stretch="always")
    app.gait_widgets["panes"] = panes
    _build_params_column(app, left)
    _build_run_column(app, right)
    load_gait_fields(app)
    refresh_gait_panel(app)


def _build_params_column(app, parent) -> None:
    from motor_control.ui.scrollable import ScrollableFrame

    parent.columnconfigure(0, weight=1)
    parent.rowconfigure(0, weight=1)
    book = ttk.Notebook(parent)
    book.grid(row=0, column=0, sticky="nsew")
    app.gait_widgets["params_book"] = book
    pages = {}
    for name, title in (("run", "运行"), ("calibration", "标定"), ("geometry", "几何参考")):
        page = ScrollableFrame(book, width=330)
        book.add(page, text=title)
        pages[name] = page
        page.content.columnconfigure(0, weight=1)
    app.gait_widgets["param_pages"] = pages

    def note(inner, row, text):
        ttk.Label(inner, text=text, wraplength=280, justify="left", foreground="#64748b").grid(
            row=row, column=0, columnspan=3, sticky="ew", padx=6, pady=6)

    def add_number_field(inner, row, key, label):
        ttk.Label(inner, text=label, wraplength=185, justify="left").grid(
            row=row, column=0, sticky="w", padx=6, pady=3)
        var = tk.StringVar()
        var.trace_add("write", lambda *_: _invalidate_calibration(app))
        app.gait_field_vars[key] = var
        ttk.Entry(inner, textvariable=var, width=9).grid(
            row=row, column=1, columnspan=2, sticky="ew", padx=6, pady=3)

    inner = pages["run"].content
    note(inner, 0, "每步公转 60°；Δψ = kφ，Δq摆 = (k+1)φ，Δq支 = φ。\n"
         "默认 k=-2：摆动电机与公转反向转 60°，支撑（公转）电机 +60°。\n"
         "换位目标按顺向 60° 几何落点自动选取（含邻座）。")
    for row, (key, label) in enumerate(RUN_FIELDS + GAP_FIELDS, 1):
        add_number_field(inner, row, key, label)
    note(inner, 8, "抬足行程直接控制升降电机。参考间隙只用于几何估算，"
         "不会增加抬升距离或改变旋转角度。")

    inner = pages["calibration"].content
    note(inner, 0, "先在电机绑定页设置 PPR、减速比和升降导程，再点动验证方向。")
    add_number_field(inner, 1, *CALIBRATION_FIELDS[0])
    row = 2
    for key, label, hint in SIGN_FIELDS:
        ttk.Label(inner, text=label).grid(
            row=row, column=0, sticky="w", padx=6, pady=3)
        var = tk.StringVar(value="+1")
        var.trace_add("write", lambda *_: _invalidate_calibration(app))
        app.gait_sign_vars[key] = var
        ttk.Combobox(
            inner, textvariable=var, values=("+1", "-1"),
            state="readonly", width=5,
        ).grid(row=row, column=1, sticky="w", padx=4, pady=1)
        ttk.Label(inner, text=hint, foreground="#666").grid(
            row=row+1, column=0, columnspan=3, sticky="w", padx=6)
        row += 2

    note(inner, row, "旋转零位：把三足摆到 ψ=30° 基准后分别记零。")
    row += 1
    for role_name in ("Mr1", "Mr2"):
        ttk.Label(inner, text=f"{role_name} 零位").grid(
            row=row, column=0, sticky="w", padx=6, pady=1)
        ttk.Label(
            inner, textvariable=app.gait_zero_vars[role_name],
            font=("Consolas", 10, "bold"),
        ).grid(row=row, column=1, sticky="w", padx=4)
        ttk.Button(
            inner, text="记零",
            command=lambda name=role_name: app._gait_record_zero_clicked(name),
        ).grid(row=row, column=2, sticky="w", padx=4, pady=1)
        row += 1

    ttk.Checkbutton(inner, variable=app.gait_calibrated_var,
                    text="已确认电机标定与现场支撑").grid(
                        row=row, column=0, columnspan=3, sticky="w", padx=6, pady=6)
    row += 1
    ttk.Button(inner, text="人工重建 A/B 基准（不动电机）",
               command=app._gait_reestablish_baseline).grid(
                   row=row, column=0, columnspan=3, sticky="w", padx=6, pady=4)

    inner = pages["geometry"].content
    note(inner, 0, "仅用于画图和碰撞间隙提示，不参与旋转角度计算。"
         "无需每次运行填写；旧配置会保留。未实测时，图示和间隙结果不能代表实际机构。")
    for row, (key, label) in enumerate(GEOMETRY_FIELDS + PREVIEW_FIELDS, 1):
        add_number_field(inner, row, key, label)
    note(inner, len(GEOMETRY_FIELDS + PREVIEW_FIELDS)+1,
         "紧贴正六边形满足 d=√3×节点环半径；只有参考模型使用这些尺寸。"
         "壳体包络应包含电机、轴承和连接件。")
    for page in pages.values():
        page.bind_navigation()

    buttons = ttk.Frame(parent)
    buttons.grid(row=1, column=0, sticky="ew", pady=4)
    ttk.Button(
        buttons, text="保存参数", command=app._gait_save_params_clicked
    ).pack(side="left", padx=3)
    ttk.Button(
        buttons, text="放弃修改并重读", command=app._gait_reload_params_clicked
    ).pack(side="left", padx=3)
    app.gait_widgets["readiness"] = ttk.Label(parent, text="", justify="left", wraplength=280)
    app.gait_widgets["readiness"].grid(
        row=2, column=0, sticky="ew", padx=6, pady=(0, 4))


def _build_run_column(app, parent) -> None:
    from motor_control.ui.scrollable import ScrollableFrame

    parent.columnconfigure(0, weight=1)
    parent.rowconfigure(1, weight=1)
    toolbar = ttk.Frame(parent)
    toolbar.grid(row=0, column=0, sticky="ew", padx=4, pady=(0, 4))
    # Abort remains visible even when the preview or stage text is scrolled.
    app.gait_widgets["abort"] = ttk.Button(
        toolbar, text="⛔ 中止", command=app._gait_abort_clicked)
    app.gait_widgets["abort"].pack(side="left", padx=3)
    app.gait_widgets["reset"] = ttk.Button(
        toolbar, text="重置流程", command=app._gait_reset_run)
    app.gait_widgets["reset"].pack(side="left", padx=3)
    viewport = ScrollableFrame(parent, width=450)
    viewport.grid(row=1, column=0, sticky="nsew")
    app.gait_widgets["run_viewport"] = viewport
    right = viewport.content
    right.columnconfigure(0, weight=1)
    build_twin_panel(app, right)

    preview = ttk.LabelFrame(right, text="轨迹与间隙参考（不动电机）")
    preview.grid(row=2, column=0, sticky="nsew", **PAD)
    preview.columnconfigure(0, weight=1)
    canvas = tk.Canvas(
        preview, width=240, height=260,
        bg="#ffffff", highlightthickness=1, highlightbackground="#cbd5e1",
    )
    canvas.grid(row=1, column=0, sticky="ew", padx=6, pady=6)
    canvas.bind("<Configure>", lambda _e: draw_gait_preview(app))
    controls = ttk.Frame(preview)
    controls.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 2))
    ttk.Radiobutton(
        controls, text="左侧换位", value="left",
        variable=app.gait_side_var,
    ).pack(side="left", padx=4)
    ttk.Radiobutton(
        controls, text="右侧换位", value="right",
        variable=app.gait_side_var,
    ).pack(side="left", padx=4)
    ttk.Button(
        controls, text="预览 / 校验",
        command=app._gait_run_dry_run,
    ).pack(side="left", padx=4)
    result = ttk.Label(
        preview, textvariable=app.gait_report_var,
        wraplength=260, justify="left",
    )
    result.grid(row=2, column=0, sticky="ew", padx=6)
    hint = ttk.Label(
        preview,
        text="绿=低节点(落脚) 红=高节点(避让)\n蓝虚线=三足中心轨迹 细线=爪臂采样",
        foreground="#666", justify="left", wraplength=260,
    )
    hint.grid(row=3, column=0, sticky="ew", padx=6, pady=6)
    app.gait_widgets["preview_canvas"] = canvas
    app.gait_widgets["report_label"] = result

    stage = ttk.LabelFrame(right, text="S0–S7 分阶段执行（每按钮一阶段）")
    stage.grid(row=1, column=0, sticky="ew", **PAD)
    current = ttk.Label(
        stage, textvariable=app.gait_stage_var,
        font=("Microsoft YaHei", 10, "bold"), justify="left", wraplength=260,
    )
    current.pack(fill="x", padx=8, pady=(6, 2))
    app.gait_widgets["confirm_text"] = ttk.Label(
        stage, text="", wraplength=260, justify="left", foreground="#334155",
    )
    app.gait_widgets["confirm_text"].pack(fill="x", padx=8, pady=2)
    app.gait_widgets["progress"] = ttk.Label(
        stage, text="", foreground="#1565c0", font=("Consolas", 10, "bold"),
        wraplength=260, justify="left",
    )
    app.gait_widgets["progress"].pack(fill="x", padx=8)
    app.gait_widgets["angles"] = ttk.Label(stage, text="", foreground="#1565c0",
                                           wraplength=260, justify="left")
    app.gait_widgets["angles"].pack(fill="x", padx=8, pady=2)

    buttons = ttk.Frame(stage)
    buttons.pack(fill="x", padx=8, pady=8)
    buttons.columnconfigure((0, 1), weight=1)
    app.gait_widgets["start_left"] = ttk.Button(
        buttons, text="开始左侧换位",
        command=lambda: app._gait_start_run_clicked("left"),
    )
    app.gait_widgets["start_left"].grid(row=0, column=0, sticky="ew", padx=3)
    app.gait_widgets["start_right"] = ttk.Button(
        buttons, text="开始右侧换位",
        command=lambda: app._gait_start_run_clicked("right"),
    )
    app.gait_widgets["start_right"].grid(row=0, column=1, sticky="ew", padx=3)
    app.gait_widgets["advance"] = ttk.Button(
        buttons, text="✓ 确认并执行本阶段", command=app._gait_stage_confirmed,
    )
    app.gait_widgets["advance"].grid(row=1, column=0, columnspan=2, sticky="ew", padx=3, pady=(6, 0))

    app.gait_widgets["state"] = ttk.Label(
        stage, textvariable=app.gait_state_var,
        font=("Microsoft YaHei", 10, "bold"), wraplength=260, justify="left",
    )
    app.gait_widgets["state"].pack(fill="x", padx=8, pady=(0, 6))

    def resize_text(event):
        width = max(220, event.width - 36)
        for label in (current, result, hint, *(app.gait_widgets[key] for key in
                      ("confirm_text", "progress", "angles", "state"))):
            label.configure(wraplength=width)
    viewport.canvas.bind("<Configure>", resize_text, add="+")
    viewport.bind_navigation()


# ── 参数字段 ↔ GaitParams ──────────────────────────────────

def _invalidate_calibration(app):
    if not getattr(app, "_gait_loading_fields", False):
        app.gait_calibrated_var.set(False)


def load_gait_fields(app) -> None:
    """把 app.gait_params 写进输入框（构造与【放弃修改并重读】共用）。"""

    params = app.gait_params
    app._gait_loading_fields = True
    for key, _label in GEOMETRY_FIELDS + GAP_FIELDS:
        value = getattr(params.geometry, key)
        # Preserve the saved value exactly (especially d = sqrt(3) * R).
        # Six-significant-digit display formatting can make valid touching
        # hexagons overlap on the next save, even if geometry was not edited.
        app.gait_field_vars[key].set(str(value))
    for key, _label in BEAT_FIELDS:
        value = getattr(params, key)
        if key in INTEGER_FIELDS:
            value = int(value)
        app.gait_field_vars[key].set(str(value))
    for key, _label, _hint in SIGN_FIELDS:
        app.gait_sign_vars[key].set(f"{getattr(params, key):+d}")
    refresh_zero_labels(app)
    app.gait_calibrated_var.set(params.calibration_confirmed)
    app._gait_loading_fields = False


def refresh_zero_labels(app) -> None:
    params = app.gait_params
    for role_name, zero in (("Mr1", params.mr1_zero_deg),
                            ("Mr2", params.mr2_zero_deg)):
        app.gait_zero_vars[role_name].set(
            "未标定" if zero is None else f"{zero:+.3f}°")


def _field_number(app, key) -> float:
    raw = app.gait_field_vars[key].get().strip()
    try:
        value = float(raw)
    except (tk.TclError, TypeError, ValueError):
        raise ValueError(f"步态参数 {key} 不是数值：{raw!r}") from None
    if not math.isfinite(value):
        raise ValueError(f"步态参数 {key} 必须是有限数值")
    return value


def collect_gait_params(app, base: GaitParams) -> GaitParams:
    """从输入框收集并构造校验后的 GaitParams；非法抛 ValueError。"""

    geometry_updates = {}
    for key, _label in GEOMETRY_FIELDS + GAP_FIELDS:
        value = _field_number(app, key)
        if key in INTEGER_FIELDS:
            if value != int(value):
                raise ValueError(f"步态参数 {key} 必须是整数")
            value = int(value)
        geometry_updates[key] = value
    param_updates = {}
    for key, _label in BEAT_FIELDS:
        value = _field_number(app, key)
        if key in INTEGER_FIELDS:
            if value != int(value):
                raise ValueError(f"步态参数 {key} 必须是整数")
            value = int(value)
        param_updates[key] = value
    for key, _label, _hint in SIGN_FIELDS:
        raw = app.gait_sign_vars[key].get()
        if raw not in ("+1", "-1"):
            raise ValueError(f"方向符号 {key} 必须是 +1 或 -1")
        param_updates[key] = int(raw)
    confirmed = bool(app.gait_calibrated_var.get())
    param_updates["calibration_confirmed"] = confirmed
    param_updates["calibration_fingerprint"] = (
        app._gait_hardware_fingerprint() if confirmed else None)
    return replace(
        base,
        geometry=replace(base.geometry, **geometry_updates),
        **param_updates,
    ).validated()


# ── 干跑预览绘制 ──────────────────────────────────────────

def draw_gait_preview(app) -> None:
    """按最近一次干跑报告重画俯视图（无报告时画占位提示）。"""

    canvas = app.gait_widgets.get("preview_canvas")
    if canvas is None:
        return
    report = getattr(app, "_gait_last_report", None)
    width = max(1, canvas.winfo_width())
    height = max(1, canvas.winfo_height())
    canvas.delete("all")
    if report is None:
        canvas.create_text(
            width / 2, height / 2, text="选择摆动侧后点【预览 / 校验】",
            fill="#64748b", font=("Microsoft YaHei", 11),
        )
        return
    geometry = app.gait_params.geometry

    # 收集所有需要显示的世界坐标点，求整体包围盒。
    points = []
    hexagon_nodes = {}
    for hexagon in report.hexagons:
        nodes = hexagon.low_nodes(geometry.arm_length_mm) + \
            hexagon.high_nodes(geometry.arm_length_mm)
        hexagon_nodes[hexagon.name] = nodes
        points.extend(nodes)
        points.append(hexagon.center)
    for sample in report.samples:
        points.append(sample.center)
        for arm_index in range(3):
            angle = math.radians(sample.psi_deg + 120.0 * arm_index)
            points.append((
                sample.center[0] + geometry.arm_length_mm * math.cos(angle),
                sample.center[1] + geometry.arm_length_mm * math.sin(angle),
            ))
    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    min_y = min(p[1] for p in points)
    max_y = max(p[1] for p in points)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    scale = min(
        max(1, width - 2 * PREVIEW_MARGIN) / span_x,
        max(1, height - 2 * PREVIEW_MARGIN) / span_y,
    )

    def to_canvas(x, y):
        return (
            PREVIEW_MARGIN + (x - min_x) * scale
            + (width - 2 * PREVIEW_MARGIN - span_x * scale) / 2,
            height - PREVIEW_MARGIN - (y - min_y) * scale
            - (height - 2 * PREVIEW_MARGIN - span_y * scale) / 2,
        )

    # 六边形：六个节点连线 + 中心标签
    for name, nodes in hexagon_nodes.items():
        ordered = sorted(
            nodes,
            key=lambda node: math.atan2(
                node[1] - sum(n[1] for n in nodes) / 6.0,
                node[0] - sum(n[0] for n in nodes) / 6.0,
            ),
        )
        polygon = []
        for node in ordered:
            polygon.extend(to_canvas(*node))
        polygon.extend(polygon[:2])
        canvas.create_line(
            *polygon, fill="#94a3b8", width=1.5, dash=(3, 2))
        label_x, label_y = to_canvas(*(
            sum(n[0] for n in nodes) / 6.0,
            sum(n[1] for n in nodes) / 6.0,
        ))
        canvas.create_text(
            label_x, label_y, text=name, fill="#475569",
            font=("Microsoft YaHei", 11, "bold"))
        for node in nodes[:3]:          # low_nodes 在前
            x, y = to_canvas(*node)
            canvas.create_oval(
                x - 3.5, y - 3.5, x + 3.5, y + 3.5,
                fill="#16a34a", outline="")
        for node in nodes[3:]:
            x, y = to_canvas(*node)
            canvas.create_oval(
                x - 3.5, y - 3.5, x + 3.5, y + 3.5,
                fill="#dc2626", outline="")

    # 三足中心轨迹
    arc = []
    for sample in report.samples:
        arc.extend(to_canvas(*sample.center))
    canvas.create_line(*arc, fill="#2563eb", width=2, dash=(6, 3))

    # 爪臂采样（每 8 个采样画一组，避免过密）
    step = max(1, len(report.samples) // int(app.gait_params.swing_segments))
    for sample in report.samples[::step]:
        for arm_index in range(3):
            angle = math.radians(sample.psi_deg + 120.0 * arm_index)
            start = (
                sample.center[0] + geometry.hub_radius_mm * math.cos(angle),
                sample.center[1] + geometry.hub_radius_mm * math.sin(angle),
            )
            end = (
                sample.center[0] + geometry.arm_length_mm * math.cos(angle),
                sample.center[1] + geometry.arm_length_mm * math.sin(angle),
            )
            canvas.create_line(
                *to_canvas(*start), *to_canvas(*end),
                fill="#93c5fd", width=1)

    # 最小间隙点
    worst = report.min_margin_sample
    if worst is not None:
        x, y = to_canvas(*worst.center)
        canvas.create_oval(
            x - 6, y - 6, x + 6, y + 6, outline="#f59e0b", width=2)
        canvas.create_text(
            PREVIEW_MARGIN + 4, PREVIEW_MARGIN + 10,
            text=f"最紧点 φ={worst.phi_deg:.1f}° ψ={worst.psi_deg:.1f}°",
            anchor="w", fill="#b45309", font=("Microsoft YaHei", 9))


# ── 阶段面板刷新 ──────────────────────────────────────────

GAIT_STATE_TEXT = {
    "ready": "待推进",
    "running": "运动中",
    "done": "已完成",
    "aborted": "已中止",
    "failed": "失败",
}
GAIT_STATE_COLORS = {
    "ready": "#1565c0",
    "running": "#1565c0",
    "done": "#16803a",
    "aborted": "#b26a00",
    "failed": "#b42318",
}


def refresh_gait_panel(app, progress_text=None) -> None:
    """把执行器/参数状态刷到控件上（Tk 线程调用）。"""

    if not getattr(app, "gait_widgets", None) or app._closing:
        return
    refresh_zero_labels(app)

    run = getattr(app, "_gait_run", None)
    verified = app.gait_params.calibration_confirmed and app.gait_calibrated_var.get()
    needs_recovery = getattr(app, "_gait_needs_recovery", False)
    app.gait_widgets["readiness"].configure(
        text="须人工重建物理基准" if needs_recovery else (
            "标定已确认；启动前仍须完整预检" if verified else "未确认标定：仅可预览"),
        foreground="#b42318" if needs_recovery or not verified else "#16803a")
    snapshot = run.describe() if run is not None else None
    if snapshot is None:
        app.gait_stage_var.set("未开始（选好摆动侧 → 先干跑 → 再开始）")
        app.gait_widgets["confirm_text"].configure(text="")
        app.gait_widgets["state"].configure(text="", foreground="#555")
    else:
        stage_id = snapshot["stage_id"]
        if stage_id is None:
            app.gait_stage_var.set("全部阶段完成 ✅")
            app.gait_widgets["confirm_text"].configure(text="")
        else:
            motion = "运动" if snapshot["is_motion_stage"] else "确认"
            app.gait_stage_var.set(
                f"阶段 {snapshot['stage_index'] + 1}/{snapshot['stage_count']}"
                f" · {stage_id} {snapshot['title']}（{motion}型，"
                f"{snapshot['move_group_count']} 组）")
            app.gait_widgets["confirm_text"].configure(
                text=f"确认清单：{snapshot['confirm_text']}")
        state = snapshot["state"]
        app.gait_widgets["state"].configure(
            text=f"状态：{GAIT_STATE_TEXT.get(state, state)}"
                 + (f" · {snapshot['last_error']}" if snapshot["last_error"] else ""),
            foreground=GAIT_STATE_COLORS.get(state, "#555"),
        )

    executing = snapshot is not None and snapshot["state"] == "running"
    active = snapshot is not None and snapshot["state"] in ("ready", "running")
    app.gait_widgets["start_left"].configure(state="disabled" if active else "normal")
    app.gait_widgets["start_right"].configure(state="disabled" if active else "normal")
    app.gait_widgets["advance"].configure(
        state="normal" if active and not executing and snapshot["stage_id"] is not None else "disabled")
    app.gait_widgets["abort"].configure(
        state="normal" if active else "disabled")
    app.gait_widgets["reset"].configure(
        state="normal" if snapshot is not None else "disabled")
    app.gait_widgets["progress"].configure(text=progress_text or "")
    refresh_twin_panel(app)


__all__ = [
    "build_gait_tab",
    "collect_gait_params",
    "draw_gait_preview",
    "load_gait_fields",
    "refresh_gait_panel",
]
