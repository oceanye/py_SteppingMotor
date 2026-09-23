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

from motor_control.gait_planner import GaitParams, LOW_NODE_PHASE_DEG
from motor_control.gait_avoidance import LEGACY, TWO_MODE
from motor_control.ui.common import PAD
from motor_control.ui.gait_twin import build_twin_panel, refresh_twin_panel

GEOMETRY_FIELDS = (
    ("d_mm", "旧中心距 d（两模态忽略）"),
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
    ("phase_gain", "旧模式增益 k（两模态忽略）"),
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


def _gait_mode_selected(app, value: str) -> None:
    """四个换位方式 radio 的回调：同步 side/arc 变量并静默重跑预览。"""

    app.gait_side_var.set("left" if value[0] == "L" else "right")
    app.gait_arc_var.set("60.0" if value[1] == "+" else "-60.0")
    app._gait_run_dry_run(interactive=False)


def build_gait_tab(app, parent) -> None:
    """Build calibration params, dry-run preview and stage controls."""

    parent.columnconfigure(0, weight=1)
    parent.rowconfigure(1, weight=1)
    app.gait_widgets = {}
    app.gait_field_vars = {}
    app.gait_sign_vars = {}
    app.gait_side_var = tk.StringVar(value="left")
    # 换位方向（顺向 +60 / 逆向 -60）。gait_mode_var 是四个换位方式
    # （"L+/L-/R+/R-"）的合并选择，切换时同步下面两个变量。
    app.gait_arc_var = tk.StringVar(value="60.0")
    app.gait_mode_var = tk.StringVar(value="L+")
    app._gait_preview_alt_pad = None
    app.gait_zero_vars = {
        "Mr1": tk.StringVar(value="未标定"),
        "Mr2": tk.StringVar(value="未标定"),
    }
    app.gait_report_var = tk.StringVar(value="尚未干跑")
    app.gait_stage_var = tk.StringVar(value="未开始")
    app.gait_state_var = tk.StringVar(value="")
    app.gait_calibrated_var = tk.BooleanVar(value=False)
    app.gait_trajectory_var = tk.StringVar(value=TWO_MODE)
    app.gait_trajectory_var.trace_add("write", lambda *_: _invalidate_calibration(app))
    app._gait_loading_fields = False
    app._gait_last_report = None

    warning = ttk.LabelFrame(parent, text="角度联动与实机执行")
    warning.grid(row=0, column=0, sticky="ew", **PAD)
    notice = ttk.Label(
        warning,
        text=(
            "旋转联动只按角度计算；中心距和六边形尺寸不改变电机转角。实机按 S0–S7 逐阶段确认，"
            "旋转轴需支持 SYNC 的同一 ESP32。两模态按当前站位自动判断，腿—高杆净间隙不合格禁止执行；"
            "旧模式保留历史行为。接触和支撑仍需现场确认。"
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
    ttk.Label(inner, text="避杆轨迹模式").grid(row=0, column=0, sticky="w", padx=6)
    ttk.Combobox(inner, textvariable=app.gait_trajectory_var,
                 values=(TWO_MODE, LEGACY), state="readonly", width=16).grid(
                     row=0, column=1, columnspan=2, sticky="ew", padx=6)
    note(inner, 1, "two_mode_v1：低节点侧反向比例自转；高节点侧同向变比例自转。\n"
         "按当前站位自动分类，四按钮均支持。分段双轴SYNC，换向时停稳。\n"
         "legacy_gain：兼容旧固定k；不是已验证的腿避杆策略。")
    for row, (key, label) in enumerate(RUN_FIELDS + GAP_FIELDS, 2):
        add_number_field(inner, row, key, label)
    note(inner, 9, "两模态使用紧贴晶格 d=√3R，保留但不使用旧中心距。"
         "腿/杆半径和δ必须实测，不能调小数值强行放行。抬高不能替代腿的平面避让。")

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
    note(inner, 0, "尺寸不改变候选角度规律，但腿/杆半径、R和δ决定两模态是否允许执行。"
         "旧中心距仅用于旧模式。无需每次填写；未实测时结果不能代表实际机构。")
    for row, (key, label) in enumerate(GEOMETRY_FIELDS + PREVIEW_FIELDS, 1):
        add_number_field(inner, row, key, label)
    note(inner, len(GEOMETRY_FIELDS + PREVIEW_FIELDS)+1,
         "两模态自动使用 d=√3×节点环半径。壳体/横梁参数为额外结构诊断，"
         "不用于豁免腿的平面冲突；壳体包络应包含电机、轴承和连接件。")
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
    canvas.grid(row=2, column=0, sticky="ew", padx=6, pady=6)
    canvas.bind("<Configure>", lambda _e: draw_gait_preview(app))
    # 2026-09-22 按用户要求：摆动侧与顺/逆方向合并为四个换位方式选项
    # （左顺移/左逆移/右顺移/右逆移）。切换即同步 side/arc 并静默重跑，
    # 预览图与【▶ 模拟动作】永远对应当前选择。
    modes = ttk.Frame(preview)
    modes.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 2))
    for value, text in (("L+", "左顺移"), ("L-", "左逆移"),
                        ("R+", "右顺移"), ("R-", "右逆移")):
        ttk.Radiobutton(
            modes, text=text, value=value,
            variable=app.gait_mode_var,
            command=lambda v=value: _gait_mode_selected(app, v),
        ).pack(side="left", padx=3)
    actions = ttk.Frame(preview)
    actions.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 2))
    ttk.Button(
        actions, text="预览 / 校验",
        command=app._gait_run_dry_run,
    ).pack(side="left", padx=4)
    app.gait_widgets["play_btn"] = ttk.Button(
        actions, text="▶ 模拟动作",
        command=app._gait_play_preview_clicked,
    )
    app.gait_widgets["play_btn"].pack(side="left", padx=4)
    result = ttk.Label(
        preview, textvariable=app.gait_report_var,
        wraplength=260, justify="left",
    )
    result.grid(row=3, column=0, sticky="ew", padx=6)
    hint = ttk.Label(
        preview,
        text="蓝=左足 橙=右足 绿点=低节点(落脚) 红点=高节点(避让)\n"
             "虚线=摆动中心轨迹；六边形按共边几何紧贴摆放\n"
             "▶=逐帧模拟旋转段(不含抬升/下降)\n"
             "四个【开始…移】按钮与换位方式一一对应，点哪个走哪个",
        foreground="#666", justify="left", wraplength=260,
    )
    hint.grid(row=4, column=0, sticky="ew", padx=6, pady=6)
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
    # 2026-09-22 按用户要求：执行入口与四种换位方式一一对应，方向由
    # 按钮显式携带，不依赖预览区当前选择（防止“看逆向、走顺向”误操作）。
    for key, text, side, arc in (
        ("start_left_cw", "开始左顺移", "left", 60.0),
        ("start_left_ccw", "开始左逆移", "left", -60.0),
        ("start_right_cw", "开始右顺移", "right", 60.0),
        ("start_right_ccw", "开始右逆移", "right", -60.0),
    ):
        app.gait_widgets[key] = ttk.Button(
            buttons, text=text,
            command=lambda s=side, a=arc: app._gait_start_run_clicked(s, a),
        )
    app.gait_widgets["start_left_cw"].grid(row=0, column=0, sticky="ew", padx=3)
    app.gait_widgets["start_left_ccw"].grid(row=0, column=1, sticky="ew", padx=3)
    app.gait_widgets["start_right_cw"].grid(
        row=1, column=0, sticky="ew", padx=3, pady=(3, 0))
    app.gait_widgets["start_right_ccw"].grid(
        row=1, column=1, sticky="ew", padx=3, pady=(3, 0))
    app.gait_widgets["advance"] = ttk.Button(
        buttons, text="✓ 确认并执行本阶段", command=app._gait_stage_confirmed,
    )
    # row=2：四个方向按钮占 2×2（row0/row1），advance 独占下一行。
    # 2026-09-22 修复：advance 原在 row=1 且 columnspan=2，与右顺/右逆
    # 两按钮同行同格，后 grid 者覆盖前者——右侧两按钮被整个遮住不可见。
    app.gait_widgets["advance"].grid(row=2, column=0, columnspan=2, sticky="ew", padx=3, pady=(6, 0))

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
        # Changing the law/geometry must not replay a cached, different path.
        app._gait_last_report = None
        if getattr(app, "gait_widgets", {}).get("preview_anim"):
            stop_preview_animation(app)


def load_gait_fields(app) -> None:
    """把 app.gait_params 写进输入框（构造与【放弃修改并重读】共用）。"""

    params = app.gait_params
    app._gait_loading_fields = True
    app.gait_trajectory_var.set(params.trajectory_mode)
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
    param_updates["trajectory_mode"] = app.gait_trajectory_var.get()
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

ANIM_FRAME_MS = 40          # 动画帧间隔
ANIM_TARGET_FRAMES = 180    # 总帧数上限（约 7 秒），采样多时跳步


def draw_gait_preview(app) -> None:
    """按最近一次干跑报告重画俯视图（无报告时画占位提示）。

    聚焦本次 route 的起点/支点/目标；两模态额外显示扫掠邻域支座，
    避免图上漏掉相邻高杆。碰撞校验覆盖完整规划邻域。
    """

    canvas = app.gait_widgets.get("preview_canvas")
    if canvas is None:
        return
    anim = app.gait_widgets.get("preview_anim")
    if anim is not None and anim.get("playing"):
        return   # 动画逐帧重画并自适应尺寸；不闪静态帧
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
    scene = _preview_scene(app, report, width, height)
    if scene is None:
        return
    _draw_preview_pads(canvas, scene)
    _draw_preview_path(canvas, scene, report, app.gait_params)
    worst = report.min_margin_sample
    if worst is not None:
        x, y = scene["to_canvas"](*worst.center)
        canvas.create_oval(
            x - 6, y - 6, x + 6, y + 6, outline="#f59e0b", width=2)
        canvas.create_text(
            PREVIEW_MARGIN + 4, PREVIEW_MARGIN + 10,
            text=f"最紧点 φ={worst.phi_deg:.1f}° ψ={worst.psi_deg:.1f}°",
            anchor="w", fill="#b45309", font=("Microsoft YaHei", 9))


def _preview_scene(app, report, width, height):
    """route 支座（起点/支点/目标 + 反向落点）+ 轨迹包围盒 → 坐标换算。"""

    geometry = app.gait_params.geometry
    route = report.route
    names = {route[0], route[1], route[2]} if route is not None else None
    alt = getattr(app, "_gait_preview_alt_pad", None)
    if alt and names is not None:
        names.add(alt)   # 反向落点也画出：顺/逆两种方式同图可比
    points = []
    hexagon_nodes = {}
    for hexagon in report.hexagons:
        if names is not None and hexagon.name not in names:
            if (app.gait_params.trajectory_mode != TWO_MODE or
                    min(math.dist(hexagon.center, s.center) for s in report.samples)
                    > 2*geometry.arm_length_mm+geometry.node_radius_mm):
                continue
        nodes = hexagon.low_nodes(geometry.arm_length_mm) + \
            hexagon.high_nodes(geometry.arm_length_mm)
        hexagon_nodes[hexagon.name] = nodes
        points.extend(nodes)
        points.append(hexagon.center)
    if not hexagon_nodes:
        return None
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
    return {"to_canvas": to_canvas, "hexagon_nodes": hexagon_nodes}


def _draw_preview_pads(canvas, scene) -> None:
    """六边形：六个节点连线 + 中心标签 + 绿(低)/红(高)节点。"""

    to_canvas = scene["to_canvas"]
    for name, nodes in scene["hexagon_nodes"].items():
        center = (sum(n[0] for n in nodes) / 6.0,
                  sum(n[1] for n in nodes) / 6.0)
        ordered = sorted(
            nodes,
            key=lambda node: math.atan2(
                node[1] - center[1], node[0] - center[0]),
        )
        polygon = []
        for node in ordered:
            polygon.extend(to_canvas(*node))
        polygon.extend(polygon[:2])
        canvas.create_line(
            *polygon, fill="#94a3b8", width=1.5, dash=(3, 2))
        label_x, label_y = to_canvas(*center)
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


def _draw_preview_path(canvas, scene, report, params) -> None:
    """静态参考：整条摆动中心轨迹虚线 + 爪臂采样细线。"""

    to_canvas = scene["to_canvas"]
    geometry = params.geometry
    arc = []
    for sample in report.samples:
        arc.extend(to_canvas(*sample.center))
    canvas.create_line(*arc, fill="#2563eb", width=2, dash=(6, 3))
    # 爪臂采样（按 swing_segments 抽稀，避免过密）
    step = max(1, len(report.samples) // int(params.swing_segments))
    for sample in report.samples[::step]:
        for arm_index in range(3):
            angle = math.radians(sample.psi_deg + 120.0 * arm_index)
            inner_radius = 0.0 if params.trajectory_mode == TWO_MODE else geometry.hub_radius_mm
            start = (sample.center[0] + inner_radius * math.cos(angle),
                     sample.center[1] + inner_radius * math.sin(angle))
            end = (
                sample.center[0] + geometry.arm_length_mm * math.cos(angle),
                sample.center[1] + geometry.arm_length_mm * math.sin(angle),
            )
            canvas.create_line(
                *to_canvas(*start), *to_canvas(*end),
                fill="#93c5fd", width=1)


# ── 预览动作模拟（只动视图，不发任何命令） ─────────────────

def play_preview_animation(app) -> None:
    """沿最近一次干跑的 samples 逐帧回放摆动旋转段。"""

    report = getattr(app, "_gait_last_report", None)
    canvas = app.gait_widgets.get("preview_canvas")
    if report is None or canvas is None or not report.samples:
        app.log("⚠️ 模拟动作：请先点【预览 / 校验】生成轨迹")
        return
    stop_preview_animation(app)
    anim = app.gait_widgets.setdefault(
        "preview_anim", {"job": None, "frame": 0, "playing": False})
    anim.update(frame=0, playing=True)
    button = app.gait_widgets.get("play_btn")
    if button is not None:
        button.configure(text="⏹ 停止模拟")
    start, target, pivot, _bearing = report.route
    app.log(f"▶ 预览模拟：{start}→{target} 绕{pivot}"
            f"（{len(report.samples)} 采样，仅视图动画，不动电机）")
    _preview_anim_tick(app)


def stop_preview_animation(app, *, redraw: bool = False) -> None:
    """停止动画；redraw=True 时恢复静态预览图（Tk 线程调用）。"""

    anim = app.gait_widgets.get("preview_anim")
    if anim is None:
        return
    if anim.get("job") is not None:
        try:
            app.root.after_cancel(anim["job"])
        except tk.TclError:
            pass
    anim.update(job=None, playing=False)
    button = app.gait_widgets.get("play_btn")
    if button is not None:
        button.configure(text="▶ 模拟动作")
    if redraw:
        draw_gait_preview(app)


def _preview_anim_tick(app) -> None:
    anim = app.gait_widgets.get("preview_anim")
    report = getattr(app, "_gait_last_report", None)
    canvas = app.gait_widgets.get("preview_canvas")
    if (anim is None or report is None or canvas is None
            or not anim.get("playing")):
        return
    samples = report.samples
    index = min(anim["frame"], len(samples) - 1)
    draw_preview_frame(app, report, index)
    if index + 1 >= len(samples):
        stop_preview_animation(app)   # 终点帧保留在画布上
        return
    step = max(1, round(len(samples) / ANIM_TARGET_FRAMES))
    anim["frame"] = min(index + step, len(samples) - 1)
    try:
        anim["job"] = canvas.after(
            ANIM_FRAME_MS, lambda: _preview_anim_tick(app))
    except tk.TclError:
        anim["job"] = None
        anim["playing"] = False


def draw_preview_frame(app, report, index: int) -> None:
    """动画单帧：支座 + 轨迹 + 支撑足/横梁/摆动足当前位置。

    配色与机构左右固定对应（不能搞混）：蓝=左足、橙=右足；摆动足
    用它自身左右侧的颜色——左侧换位蓝足移动、右侧换位橙足移动。
    支撑足站在支点低节点上，公转期间世界姿态不变；摆动足三爪按 ψ
    按选定模态自转。横梁两端缩进壳体半径，不贯穿足本体。动画只覆盖 S4
    旋转段；抬升/收腿/落位是直线动作，不在本图。
    """

    canvas = app.gait_widgets.get("preview_canvas")
    if canvas is None:
        return
    width = max(1, canvas.winfo_width())
    height = max(1, canvas.winfo_height())
    canvas.delete("all")
    scene = _preview_scene(app, report, width, height)
    if scene is None:
        return
    to_canvas = scene["to_canvas"]
    _draw_preview_pads(canvas, scene)
    geometry = app.gait_params.geometry
    samples = report.samples
    index = max(0, min(index, len(samples) - 1))
    swing_color, support_color = (
        ("#2563eb", "#ea580c") if report.side == "left"
        else ("#ea580c", "#2563eb"))

    faint = [to_canvas(*s.center) for s in samples]
    canvas.create_line(*faint, fill="#cbd5e1", width=1, dash=(4, 3))
    walked = [to_canvas(*s.center) for s in samples[:index + 1]]
    if len(walked) >= 2:
        canvas.create_line(*walked, fill=swing_color, width=2)

    sample = samples[index]
    route = report.route
    pivot_center = None
    if route is not None and route[2] in scene["hexagon_nodes"]:
        nodes = scene["hexagon_nodes"][route[2]]
        pivot_center = (sum(n[0] for n in nodes) / 6.0,
                        sum(n[1] for n in nodes) / 6.0)
    if pivot_center is not None:
        px, py = to_canvas(*pivot_center)
        # 横梁：两端各缩进壳体半径，只画两足壳体之间的部分
        dx = sample.center[0] - pivot_center[0]
        dy = sample.center[1] - pivot_center[1]
        span = math.hypot(dx, dy)
        hub = geometry.hub_radius_mm
        if span > 2 * hub:
            ux, uy = dx / span, dy / span
            beam_start = (pivot_center[0] + ux * hub, pivot_center[1] + uy * hub)
            beam_end = (sample.center[0] - ux * hub, sample.center[1] - uy * hub)
            canvas.create_line(*to_canvas(*beam_start), *to_canvas(*beam_end),
                               fill="#64748b", width=3)
        for k in range(3):
            angle = math.radians(LOW_NODE_PHASE_DEG + 120.0 * k)
            end = (pivot_center[0] + geometry.arm_length_mm * math.cos(angle),
                   pivot_center[1] + geometry.arm_length_mm * math.sin(angle))
            canvas.create_line(px, py, *to_canvas(*end),
                               fill=support_color, width=3)
        canvas.create_oval(px - 5, py - 5, px + 5, py + 5,
                           fill=support_color, outline="")

    x, y = to_canvas(*sample.center)
    for k in range(3):
        angle = math.radians(sample.psi_deg + 120.0 * k)
        end = (sample.center[0] + geometry.arm_length_mm * math.cos(angle),
               sample.center[1] + geometry.arm_length_mm * math.sin(angle))
        canvas.create_line(x, y, *to_canvas(*end), fill=swing_color, width=3)
    canvas.create_oval(x - 5, y - 5, x + 5, y + 5,
                       fill=swing_color, outline="")
    canvas.create_text(
        PREVIEW_MARGIN + 4, PREVIEW_MARGIN + 10,
        text=(f"模拟 φ={sample.phi_deg:.1f}° ψ={sample.psi_deg:.1f}°"
              f"（{index + 1}/{len(samples)}）"),
        anchor="w", fill="#1d4ed8", font=("Microsoft YaHei", 9))


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
    for key in ("start_left_cw", "start_left_ccw",
                "start_right_cw", "start_right_ccw"):
        app.gait_widgets[key].configure(
            state="disabled" if active else "normal")
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
    "play_preview_animation",
    "refresh_gait_panel",
    "stop_preview_animation",
]
