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

from motor_control.gait_planner import (
    GaitParams, INITIAL_PLACEMENTS, LOW_NODE_PHASE_DEG,
)
from motor_control.gait_avoidance import LEGACY, TWO_MODE
from motor_control.ui.common import PAD, reset_canvas_view
from motor_control.ui.gait_twin import (
    build_twin_panel, draw_twin, refresh_twin_panel, sync_twin_start_fields,
)

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

# 2026-09-24 初始摆放（红杆在横梁左/右侧的镜像摆法）下拉：标签面向
# 操作员，内部值进 params.initial_placement；β₀ 由摆放派生并联动覆写。
# 2026-09-28 实机校正：左右标注对调（初版误按"第三支座C在哪侧"判，
# 与"紧挨横梁的红杆在哪侧"恰相反）。左足A·右足B=红杆右侧（原有默认）。
PLACEMENT_LABELS = {
    "red_left": "红杆在横梁左侧（左足B·右足A）",
    "red_right": "红杆在横梁右侧（左足A·右足B）",
}
LABEL_TO_PLACEMENT = {label: key for key, label in PLACEMENT_LABELS.items()}


def _placement_label(params):
    if params.initial_pad_pair is not None:
        side = "左" if params.initial_placement == "red_left" else "右"
        return f"地图起步 · 红杆{side}侧 · β₀={params.initial_beam_deg:g}°"
    return PLACEMENT_LABELS[params.initial_placement]


def _gait_placement_changed(app) -> None:
    """初始摆放切换：基准变更——账本重置、零位作废、标定失效。"""

    if getattr(app, "_gait_loading_fields", False):
        return
    placement = LABEL_TO_PLACEMENT.get(app.gait_placement_var.get())
    if placement is None or (placement == app.gait_params.initial_placement
                             and app.gait_params.initial_pad_pair is None):
        return
    params = replace(app.gait_params, initial_placement=placement,
                     initial_pad_pair=None, calibration_confirmed=False,
                     beam_reference_deg=INITIAL_PLACEMENTS[placement][1],
                     mr1_zero_deg=None, mr2_zero_deg=None)
    # The legacy preset must use the same atomic idle check as map selection.
    setter = getattr(app, "_gait_set_start_reference", None)
    try:
        if setter:
            setter(params)
        elif getattr(app, "_gait_owned", {}):
            raise ValueError("步态执行中不能更改初始摆放")
        else:  # Isolated view hosts (no controller / no physical axes).
            app.gait_params = params
            app._gait_supports = tuple(params.initial_supports)
            app._gait_beta_deg = params.initial_beam_deg
            app._gait_world_epoch = getattr(app, "_gait_world_epoch", 0) + 1
            app._gait_run = None
    except (ValueError, RuntimeError) as exc:
        from tkinter import messagebox
        app._gait_loading_fields = True
        app.gait_placement_var.set(_placement_label(app.gait_params))
        app._gait_loading_fields = False
        messagebox.showwarning("基准未更改", str(exc))
        return
    _invalidate_calibration(app)
    app.gait_field_vars["beam_reference_deg"].set(str(params.beam_reference_deg))
    refresh_zero_labels(app)
    sync_twin_start_fields(app)

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
    ttk.Label(inner, text="运动模态").grid(row=0, column=0, sticky="w", padx=6)
    app.gait_widgets["trajectory_status"] = ttk.Label(
        inner, text="", wraplength=150, justify="left")
    app.gait_widgets["trajectory_status"].grid(
        row=0, column=1, columnspan=2, sticky="ew", padx=6)
    note(inner, 1, "只选择左/右足与顺/逆动作，无需选择 HIGH / LOW。\n"
         "后台按当前站位判断本步横梁扫掠：跨高杆 → HIGH；不跨 → LOW。\n"
         "每次换步重新判断，不与按钮固定绑定；自动判定不代表避让校验通过。")
    run_fields = tuple(field for field in RUN_FIELDS if field[0] != "phase_gain") + GAP_FIELDS
    for row, (key, label) in enumerate(run_fields, 2):
        add_number_field(inner, row, key, label)
    note(inner, len(run_fields)+2, "两模态使用紧贴晶格 d=√3R，保留但不使用旧中心距。"
         "腿/杆半径和δ必须实测，不能调小数值强行放行。抬高不能替代腿的平面避让。")

    inner = pages["calibration"].content
    note(inner, 0, "先在“电机绑定与状态”页设置 PPR、减速比和升降导程，\n"
         "并用页内【逐电机调试与方向标定】点动验证、记录每个电机的真实方向。")
    ttk.Label(inner, text="初始摆放（俯视·从左足看向右足）",
              wraplength=185, justify="left").grid(
                  row=1, column=0, sticky="w", padx=6, pady=3)
    app.gait_placement_var = tk.StringVar(
        value=_placement_label(app.gait_params))
    app.gait_placement_var.trace_add(
        "write", lambda *_: _gait_placement_changed(app))
    ttk.Combobox(
        inner, textvariable=app.gait_placement_var,
        values=tuple(PLACEMENT_LABELS[key] for key in PLACEMENT_LABELS),
        state="readonly", width=22,
    ).grid(row=1, column=1, columnspan=2, sticky="ew", padx=6, pady=3)
    note(inner, 2, "机构实际怎么摆就选什么：俯视、站在左足处看向右足，\n"
         "紧挨横梁的那根红杆在哪只手边就选哪侧。红杆在哪侧决定同一按钮\n"
         "的模态序列（两摆法互为镜像）。切换即基准变更：零位作废，须重新\n"
         "摆机构到该初始状态、重新记零并确认标定。\n"
         "任意两格在右侧数字孪生中选择；此下拉仅用于恢复 A/B 预设。")
    add_number_field(inner, 3, *CALIBRATION_FIELDS[0])
    row = 4
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
    ttk.Button(inner, text="人工重建所选起步基准（不动电机）",
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
    row = len(GEOMETRY_FIELDS + PREVIEW_FIELDS)+2
    ttk.Label(inner, text="历史兼容策略（非模态选择）", wraplength=185).grid(
        row=row, column=0, sticky="w", padx=6)
    app.gait_widgets["trajectory_strategy"] = ttk.Combobox(
        inner, textvariable=app.gait_trajectory_var,
        values=(TWO_MODE, LEGACY), state="readonly", width=16)
    app.gait_widgets["trajectory_strategy"].grid(
        row=row, column=1, columnspan=2, sticky="ew", padx=6)
    add_number_field(inner, row+1, "phase_gain", "旧模式增益 k（两模态忽略）")
    note(inner, row+2, "正常避杆使用 two_mode_v1，由后台自动选 HIGH / LOW。"
         "legacy_gain 仅兼容历史固定增益，不是另一种避杆模态，不能用它绕过避让失败。"
         "保留旧配置，不静默更改既有标定。")
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

    # 2026-09-30 按用户要求：预览画面并入态势图（同一 5×5 地图分"评估
    # 模拟(计划)"与"实机脉冲"两种数据源模式）。换位方式/预览校验/模拟
    # 动作/干跑报告挂在态势图顶部槽位（gait_twin 提供 plan_slot 容器），
    # 右列不再有独立预览画布；旧入口 draw_gait_preview/reset_preview_view
    # 保留为兼容 shim（desktop_app 引用不变）。
    preview = app.gait_widgets["twin"]["plan_slot"]
    # 2026-09-22 按用户要求：摆动侧与顺/逆方向合并为四个换位方式选项
    # （左顺移/左逆移/右顺移/右逆移）。切换即同步 side/arc 并静默重跑，
    # 预览图与【▶ 模拟动作】永远对应当前选择。
    modes = ttk.Frame(preview)
    modes.grid(row=0, column=0, sticky="ew")
    for value, text in (("L+", "左顺移"), ("L-", "左逆移"),
                        ("R+", "右顺移"), ("R-", "右逆移")):
        ttk.Radiobutton(
            modes, text=text, value=value,
            variable=app.gait_mode_var,
            command=lambda v=value: _gait_mode_selected(app, v),
        ).pack(side="left", padx=3)
    actions = ttk.Frame(preview)
    actions.grid(row=1, column=0, sticky="ew", pady=(2, 0))
    ttk.Button(
        actions, text="单步预览 / 校验",
        command=app._gait_run_dry_run,
    ).pack(side="left", padx=4)
    app.gait_widgets["play_btn"] = ttk.Button(
        actions, text="▶ 回放预览",
        command=app._gait_play_preview_clicked,
    )
    app.gait_widgets["play_btn"].pack(side="left", padx=4)
    from .gait_simulation import toggle_simulation, reset_simulation, discard_simulation_step
    simulation = ttk.Frame(preview)
    simulation.grid(row=3, column=0, sticky="ew", pady=3)
    app.gait_widgets["sim_btn"] = ttk.Button(
        simulation, text="▶ 模拟下一步", command=lambda: toggle_simulation(app))
    app.gait_widgets["sim_btn"].grid(row=0, column=0, padx=3)
    ttk.Button(simulation, text="取消本步", command=lambda: discard_simulation_step(app)).grid(row=0, column=1, padx=3)
    ttk.Button(simulation, text="重置模拟", command=lambda: reset_simulation(app)).grid(row=0, column=2, padx=3)
    app.gait_widgets["sim_status"] = ttk.Label(
        preview, text="累计模拟：使用所选起步两格，不改实机标定；清路径不重置站位。",
        wraplength=260, foreground="#7c3aed", justify="left")
    app.gait_widgets["sim_status"].grid(row=4, column=0, sticky="ew")
    result = ttk.Label(
        preview, textvariable=app.gait_report_var,
        wraplength=260, justify="left",
    )
    result.grid(row=2, column=0, sticky="ew", pady=(2, 0))
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
    # 2026-09-29 按用户要求：一键执行——本步剩余阶段自动连走，不再逐
    # 阶段弹窗确认（录视频用）；执行中可【⛔ 中止】停车，再点本按钮
    # 停在当前阶段后回到人工确认。
    app.gait_widgets["auto"] = ttk.Button(
        buttons, text="⚡ 一键执行", command=app._gait_auto_clicked,
    )
    app.gait_widgets["auto"].grid(row=3, column=0, columnspan=2, sticky="ew", padx=3, pady=(6, 0))

    app.gait_widgets["state"] = ttk.Label(
        stage, textvariable=app.gait_state_var,
        font=("Microsoft YaHei", 10, "bold"), wraplength=260, justify="left",
    )
    app.gait_widgets["state"].pack(fill="x", padx=8, pady=(0, 6))

    def resize_text(event):
        width = max(220, event.width - 36)
        for label in (current, result, *(app.gait_widgets[key] for key in
                      ("confirm_text", "progress", "angles", "state", "sim_status"))):
            label.configure(wraplength=width)
    viewport.canvas.bind("<Configure>", resize_text, add="+")
    viewport.bind_navigation()


# ── 参数字段 ↔ GaitParams ──────────────────────────────────

def _refresh_trajectory_status(app):
    label = getattr(app, "gait_widgets", {}).get("trajectory_status")
    if label is not None:
        automatic = app.gait_trajectory_var.get() == TWO_MODE
        label.configure(
            text="HIGH / LOW 后台自动判定" if automatic else "旧策略：未启用自动避杆",
            foreground="#334155" if automatic else "#b42318")


def _invalidate_calibration(app):
    _refresh_trajectory_status(app)
    if not getattr(app, "_gait_loading_fields", False):
        app.gait_calibrated_var.set(False)
        # Changing the law/geometry must not replay a cached, different path.
        app._gait_last_report = None
        if getattr(app, "gait_widgets", {}).get("preview_anim"):
            stop_preview_animation(app)
        from .gait_simulation import cancel_simulation
        cancel_simulation(app)


def load_gait_fields(app) -> None:
    """把 app.gait_params 写进输入框（构造与【放弃修改并重读】共用）。"""

    params = app.gait_params
    app._gait_loading_fields = True
    app.gait_trajectory_var.set(params.trajectory_mode)
    app.gait_placement_var.set(_placement_label(params))
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
    sync_twin_start_fields(app)


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
    """干跑后重画态势图（兼容入口：预览已并入态势图，desktop_app 引用不变）。

    态势图聚焦本次 route 的支座并以计划层画出轨迹/间隙；碰撞校验仍覆盖
    完整规划邻域（含 5×5 地图外高杆），不因画面范围缩小。
    """

    draw_twin(app)


# ── 爪端轨迹与爪-红杆最近距离（2026-09-23 目测观察用） ─────
# 旧预览画布的绘制已并入态势图（ui/gait_twin.py 计划层）；这里保留纯
# 几何计算供测试与干跑复用。配色与态势图计划层共用同一份。

LEG_TRACK_COLORS = ("#0d9488", "#7c3aed", "#b45309")   # 腿1/2/3 爪端轨迹色


def tip_track_points(samples, geometry, arm_index):
    """第 arm_index 条爪臂末端沿整条轨迹的世界坐标折线（mm）。

    爪端点 = 摆动中心 + R·dir(ψ+120°·k)，与动画/干跑同一 ψ 采样。
    """
    radius = geometry.arm_length_mm
    track = []
    for sample in samples:
        angle = math.radians(sample.psi_deg + 120.0 * arm_index)
        track.append((
            sample.center[0] + radius * math.cos(angle),
            sample.center[1] + radius * math.sin(angle),
        ))
    return track


def _high_rods(hexagons, geometry):
    """全部支座高杆 [(标签, 世界坐标), ...]，杆径=node_radius 参数。"""
    rods = []
    for pad in hexagons:
        for j, node in enumerate(pad.high_nodes(geometry.arm_length_mm)):
            rods.append((f"{pad.name}·{90 + 120.0 * j:g}°高杆", node))
    return rods


def tip_rod_clearance(tracks, hexagons, geometry):
    """三爪末端折线对全部高杆的最近表面距离（mm）。

    surface = 爪端到杆心距离 − node_radius（红杆直径 10mm → 扣 5）。
    只看爪端点是否擦杆，供目测参考；放行门槛仍是干跑的整段腿
    胶囊净间隙（leg_clearance，已扣腿半径与 δ），二者不可互相替代。
    返回 (surface, leg_index, sample_index, rod_label, tip, rod) 或 None。
    """
    best = None
    for leg, track in enumerate(tracks):
        for index, tip in enumerate(track):
            for label, rod in _high_rods(hexagons, geometry):
                surface = math.dist(tip, rod) - geometry.node_radius_mm
                if best is None or surface < best[0]:
                    best = (surface, leg, index, label, tip, rod)
    return best


def _tip_tracks_and_rods(app, report):
    """爪端三条折线 + 高杆表，按 report 身份缓存（动画逐帧复用）。"""
    cache = getattr(app, "_gait_tip_cache", None)
    if cache is not None and cache[0] is report:
        return cache[1]
    geometry = app.gait_params.geometry
    tracks = tuple(tip_track_points(report.samples, geometry, k)
                   for k in range(3))
    rods = _high_rods(report.hexagons, geometry)
    stats = tip_rod_clearance(tracks, report.hexagons, geometry)
    result = (tracks, rods, stats)
    app._gait_tip_cache = (report, result)
    return result


def frame_tip_clearance(report, geometry, index, rods):
    """当前帧三爪端对全部高杆的最近表面距离（供距离连线绘制）。"""
    sample = report.samples[index]
    radius = geometry.arm_length_mm
    best = None
    for leg in range(3):
        angle = math.radians(sample.psi_deg + 120.0 * leg)
        tip = (sample.center[0] + radius * math.cos(angle),
               sample.center[1] + radius * math.sin(angle))
        for label, rod in rods:
            surface = math.dist(tip, rod) - geometry.node_radius_mm
            if best is None or surface < best[0]:
                best = (surface, leg, tip, rod, label)
    return best


# ── 态势图视图交互：滚轮缩放 / 中键平移 / 双击复位 ──────────
# 事件绑定与几何算法在 ui/common.py 的 bind_canvas_view/canvas_view_*；
# 这里只保留视图变化后的重画分派与新报告生成时的视图复位（态势图
# 计划层的动画帧号真源在 preview_anim dict，_draw 自行取用）。

def _preview_redraw(app) -> None:
    """视图变化后重画态势图（动画播放/暂停中画当前帧，否则画静态）。"""
    draw_twin(app)


def reset_preview_view(app) -> None:
    """新报告/新方向生成时由外部调用，避免旧视图卡住新地图视野。"""
    view = (getattr(app, "gait_widgets", None) or {}).get("twin_view")
    if view is not None:
        reset_canvas_view(view)


# ── 预览动作模拟（只动视图，不发任何命令） ─────────────────

def play_preview_animation(app) -> None:
    """沿最近一次干跑的 samples 从头逐帧回放摆动旋转段。"""

    if getattr(app, "_gait_owned", {}):
        return
    from .gait_simulation import cancel_simulation
    cancel_simulation(app)
    report = getattr(app, "_gait_last_report", None)
    if report is None or not report.samples:
        app.log("⚠️ 模拟动作：请先点【预览 / 校验】生成轨迹")
        return
    # 播放=想看评估模拟：态势图切评估模式（计划层），动画才可见。
    twin = (getattr(app, "gait_widgets", None) or {}).get("twin")
    if twin is not None:
        twin["mode_var"].set("plan")
    stop_preview_animation(app)
    anim = app.gait_widgets.setdefault(
        "preview_anim", {"job": None, "frame": 0, "playing": False,
                         "paused": False})
    anim.update(frame=0, playing=True, paused=False)
    button = app.gait_widgets.get("play_btn")
    if button is not None:
        button.configure(text="⏸ 暂停回放")
    start, target, pivot, _bearing = report.route
    app.log(f"▶ 单步回放（不累计）：{start}→{target} 绕{pivot}"
            f"（{len(report.samples)} 采样，仅视图动画，不动电机）")
    _preview_anim_tick(app)


def pause_preview_animation(app) -> None:
    """暂停：停掉计时回调但画面保留当前帧（2026-09-23 应用户要求，
    点"停止模拟"整页闪回静态预览，改为暂停/继续）。"""

    anim = app.gait_widgets.get("preview_anim")
    if anim is None or not anim.get("playing"):
        return
    if anim.get("job") is not None:
        try:
            app.root.after_cancel(anim["job"])
        except tk.TclError:
            pass
    anim.update(job=None, playing=False, paused=True)
    button = app.gait_widgets.get("play_btn")
    if button is not None:
        button.configure(text="▶ 继续回放")


def resume_preview_animation(app) -> None:
    """从暂停帧继续回放（不从头重播）。"""

    anim = app.gait_widgets.get("preview_anim")
    if anim is None or not anim.get("paused"):
        return
    anim.update(playing=True, paused=False)
    button = app.gait_widgets.get("play_btn")
    if button is not None:
        button.configure(text="⏸ 暂停回放")
    _preview_anim_tick(app)


def toggle_preview_animation(app) -> None:
    """按钮三态入口：播放中→暂停；已暂停→继续；否则从头播放。"""

    anim = app.gait_widgets.get("preview_anim")
    if anim is not None and anim.get("playing"):
        pause_preview_animation(app)
    elif anim is not None and anim.get("paused"):
        resume_preview_animation(app)
    else:
        play_preview_animation(app)


def stop_preview_animation(app, *, redraw: bool = False) -> None:
    """彻底停止（轨迹变化/新报告/收尾用）；暂停状态一并清除。"""

    anim = app.gait_widgets.get("preview_anim")
    if anim is None:
        return
    if anim.get("job") is not None:
        try:
            app.root.after_cancel(anim["job"])
        except tk.TclError:
            pass
    anim.update(job=None, playing=False, paused=False)
    button = app.gait_widgets.get("play_btn")
    if button is not None:
        button.configure(text="▶ 回放预览")
    if redraw:
        draw_gait_preview(app)


def _preview_anim_tick(app) -> None:
    if getattr(app, "_gait_owned", {}):
        stop_preview_animation(app)
        return
    anim = app.gait_widgets.get("preview_anim")
    report = getattr(app, "_gait_last_report", None)
    if (anim is None or report is None
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
    # 计时器必须由 root 注册/取消（pause/stop 都用 root.after_cancel）。
    # canvas.after 注册的 command 记在 canvas 名下，root.after_cancel 只会
    # 从 root 的记录表移除——destroy(canvas) 时二次删除同一 command 抛
    # "can't delete Tcl command"，销毁半途而废（2026-09-23 孤儿窗根源之一）。
    try:
        anim["job"] = app.root.after(
            ANIM_FRAME_MS, lambda: _preview_anim_tick(app))
    except tk.TclError:
        anim["job"] = None
        anim["playing"] = False


def draw_preview_frame(app, report, index: int) -> None:
    """动画单帧：重画态势图——计划层按 preview_anim 的 frame 绘制。

    帧号真源在 gait_widgets["preview_anim"]（_draw 自行取用），index 参数
    仅为兼容旧签名保留。动画只覆盖 S4 旋转段；抬升/收腿/落位是直线
    动作，不在本图。
    """

    draw_twin(app)


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
    # 一键执行：进行中始终可点（=停在当前阶段后回人工）；未在自动时
    # 与单步 advance 同门槛（流程活着、当前无运动、还有剩余阶段）。
    auto_running = bool(getattr(app, "_gait_auto_run", False))
    app.gait_widgets["auto"].configure(
        text="⏹ 停止自动（本阶段后）" if auto_running else "⚡ 一键执行",
        state="normal" if (auto_running or (active and not executing
                                            and snapshot["stage_id"] is not None))
               else "disabled")
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
    "pause_preview_animation",
    "play_preview_animation",
    "refresh_gait_panel",
    "resume_preview_animation",
    "stop_preview_animation",
    "toggle_preview_animation",
]
