"""Desktop view for the tripod gait: calibration, dry-run preview, staged run.

页面分两列：左侧运行/标定/几何参考分页，右侧干跑预览
（俯视 Canvas）与 S0–S7 阶段推进。所有回调都在 ``desktop_app`` 上，
本模块只负责构建与刷新控件。
"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk, messagebox
from dataclasses import replace

from motor_control.gait_planner import (
    GaitParams, INITIAL_PLACEMENTS, LOW_NODE_PHASE_DEG,
)
from motor_control.gait_avoidance import TWO_MODE
from motor_control.ui.common import PAD, reset_canvas_view
from motor_control.ui.gait_twin import (
    build_twin_panel, draw_twin, refresh_twin_panel, sync_twin_start_fields,
)

GEOMETRY_FIELDS = (
    ("arm_length_mm", "正六边形投影边长 a (mm)"),
    ("d_mm", "相邻中心距 d (mm)"),
    ("hub_radius_mm", "壳体等水平包络半径 (mm)"),
    ("arm_radius_mm", "腿投影半宽 / 等效半径 (mm)"),
    ("node_radius_mm", "高杆投影半径 (mm)"),
    ("high_node_height_mm", "高低节点高差 (mm)"),
    ("body_drop_mm", "壳体等最低点下伸 (mm)"),
    ("beam_height_mm", "横梁中心线高度 (mm)"),
    ("beam_radius_mm", "横梁/连接件包络半径 (mm)"),
)
GAP_FIELDS = (("safety_margin_mm", "参考安全间隙 δ (mm)"),)
RUN_FIELDS = (
    ("swing_speed_deg_s", "公转峰值速度 (°/s)"),
    ("rotation_limit_deg", "线缆角度限位 ±(°)"),
    ("lift_mm", "实际抬足行程 (mm)"),
    ("lift_speed_mm_s", "抬足速度 (mm/s)"),
    ("settle_speed_mm_s", "落足速度 (mm/s)"),
)
CALIBRATION_FIELDS = (
    ("beam_reference_deg", "起步横梁角 β₀（地图派生，°）"),
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

BASELINE_GUIDE = (
    "首次装机或更换传动：\n"
    "1. 在绑定页核对四台 ID、减速比、升降导程；低速点动确认方向一次并保存。\n"
    "2. 连接并使能四台，选择与实物一致的起步两格/红杆侧。人工摆好双足，三爪对准低节点。\n"
    "3. 点击【确认摆放并记录双侧基准】，确认一次即可记录左右基准并保存。\n"
    "4. 选择四动作之一并开始；启动仍检查状态、相位、间隙、速度和限位。\n\n"
    "正常连续换步：直接选择下一动作，不重新记零、不重建地图、不重复方向标定。\n"
    "修改速度、行程、预览采样：保留基准，执行前仍重新校验计划。\n"
    "运动前取消计划：保留基准；运动已排队/开始后中止：先恢复可靠支撑和所选起步摆放，"
    "再执行第3步，硬件参数不变无需重做方向测试。\n\n"
    "断电、坐标跳变或手动搬动后：先核对实物与驱动器坐标；无法确认连续性时重新摆放并执行第3步。"
    "PC保存了基准，不等于编码器断电后仍保留同一多圈坐标。\n"
    "无需使用驱动器ZERO；它不是机械回原点，也不是步态准备的前置。"
)

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
    """四动作只生成虚线候选，不写配置、不启动模拟或实机。"""

    view = app.gait_widgets.get("twin", {})
    sim = view.get("simulation")
    from .gait_simulation import _busy
    if _busy(app) or (sim and sim.active):
        app.gait_mode_var.set(("L" if app.gait_side_var.get() == "left" else "R")
                             + ("+" if float(app.gait_arc_var.get()) > 0 else "-"))
        return
    app.gait_side_var.set("left" if value[0] == "L" else "right")
    app.gait_arc_var.set("60.0" if value[1] == "+" else "-60.0")
    from .gait_candidate import select_candidate
    select_candidate(app)


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
    # The desktop has one strategy; retain a private variable for view refresh.
    app.gait_derived_spacing_var = tk.StringVar(value="—")
    app.gait_trajectory_var.trace_add("write", lambda *_: _invalidate_calibration(app))
    app._gait_loading_fields = False
    app._gait_last_report = None

    warning = ttk.LabelFrame(parent, text="角度联动与实机执行")
    warning.grid(row=0, column=0, sticky="ew", **PAD)
    notice = ttk.Label(
        warning,
        text=(
            "旋转联动只按角度计算；中心距和六边形尺寸不改变电机转角。实机按 S0–S7 逐阶段确认，"
            "X42S 通过同一 ESP32 协调执行。两模态按当前站位自动判断，腿—高杆净间隙不合格禁止执行；"
            "本机构采用190 mm投影边长的自动避让。接触和支撑仍需现场确认。"
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
    panes.add(left, minsize=350, width=350, stretch="never")
    panes.add(right, minsize=260, width=950, stretch="always")
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
        caption = ttk.Label(inner, text=label, wraplength=185, justify="left")
        caption.grid(row=row, column=0, sticky="w", padx=6, pady=3)
        var = tk.StringVar()
        var.trace_add("write", lambda *_, field=key: _invalidate_calibration(
            app, reference_changed=field not in dict(RUN_FIELDS + PREVIEW_FIELDS)))
        app.gait_field_vars[key] = var
        entry = ttk.Entry(inner, textvariable=var, width=9)
        entry.grid(row=row, column=1, columnspan=2, sticky="ew", padx=6, pady=3)
        if key == "beam_reference_deg":
            entry.configure(state="readonly")
        if key == "d_mm":
            app.gait_widgets["spacing_entry"] = entry
            app.gait_widgets["spacing_caption"] = caption

    inner = pages["run"].content
    ttk.Label(inner, text="运动模态").grid(row=0, column=0, sticky="w", padx=6)
    app.gait_widgets["trajectory_status"] = ttk.Label(
        inner, text="", wraplength=150, justify="left")
    app.gait_widgets["trajectory_status"].grid(
        row=0, column=1, columnspan=2, sticky="ew", padx=6)
    note(inner, 1, "只选择左/右足与顺/逆动作，无需选择 HIGH / LOW。\n"
         "后台按当前站位判断本步横梁扫掠：跨高杆 → HIGH；不跨 → LOW。\n"
         "每次换步重新判断，不与按钮固定绑定；自动判定不代表避让校验通过。")
    run_fields = RUN_FIELDS + GAP_FIELDS
    for row, (key, label) in enumerate(run_fields, 2):
        add_number_field(inner, row, key, label)
    note(inner, len(run_fields)+2, "两模态仅填写投影边长 a：腿投影长度 R=a，中心距 d=√3a 自动派生。"
         "腿/杆投影宽度和δ仍需实测，不能调小数值强行放行。抬高不替代平面避让。")

    inner = pages["calibration"].content
    note(inner, 0, "首次装机才需核对绑定、减速比、升降导程并点动确认方向（STEP/DIR还需PPR）。\n"
         "日常只需摆好实物，点下方【确认摆放并记录双侧基准】一次；正常连续换步不重做。")
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
         "摆机构到该初始状态，再点下方【确认摆放并记录双侧基准】。\n"
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

    note(inner, row, "已保存的旋转基准（不要求驱动器读数为0）：")
    row += 1
    for role_name in ("Mr1", "Mr2"):
        ttk.Label(inner, text=f"{role_name} 零位").grid(
            row=row, column=0, sticky="w", padx=6, pady=1)
        ttk.Label(
            inner, textvariable=app.gait_zero_vars[role_name],
            font=("Consolas", 10, "bold"),
        ).grid(row=row, column=1, sticky="w", padx=4)
        row += 1
    ttk.Button(inner, text="查看首次准备与日常运行指引",
               command=lambda: messagebox.showinfo("步态基准操作指引", BASELINE_GUIDE)).grid(
                   row=row, column=0, columnspan=3, sticky="w", padx=6, pady=4)

    inner = pages["geometry"].content
    note(inner, 0, "平面验核只使用俯视投影：边长 a = 中心到顶点距离 = 腿投影长度。"
         "两模态中心距 d=√3a 只读；只需输入一个尺度，不填空间斜长或竖向厚度。"
         "腿/杆投影半宽和δ仍决定是否允许执行。")
    for row, (key, label) in enumerate(GEOMETRY_FIELDS + PREVIEW_FIELDS, 1):
        add_number_field(inner, row, key, label)
    note(inner, len(GEOMETRY_FIELDS + PREVIEW_FIELDS)+1,
         "中心距显示值仅供参考，计算保留完整精度，不能独立输入中心距。"
         "壳体/横梁高度参数为额外结构诊断，"
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
    app.gait_widgets["prepare_baseline"] = ttk.Button(
        parent, text="确认摆放并记录双侧基准（首次/恢复）",
        command=app._gait_reestablish_baseline)
    app.gait_widgets["prepare_baseline"].grid(row=2, column=0, sticky="ew", padx=6, pady=4)
    app.gait_widgets["readiness"] = ttk.Label(parent, text="", justify="left", wraplength=280)
    app.gait_widgets["readiness"].grid(
        row=3, column=0, sticky="ew", padx=6, pady=(0, 4))


def _build_run_column(app, parent) -> None:
    from motor_control.ui.scrollable import ScrollableFrame

    parent.columnconfigure(0, weight=1)
    parent.rowconfigure(2, weight=1)
    # 2026-10-08 按用户要求：右列整体下移一行，最顶固定两个总开关
    # （🟣 模拟 / 🔴 S0–S7 分阶段执行），下方只显示所选组的操作区，
    # 视口（态势图等）再往下——实操的阶段文字/进度不随页面滚动。
    # 换位方式四选项与模拟关联，随模拟组显示；中止/重置流程属实操，
    # 随实操组显示。
    ops = ttk.Frame(parent)
    ops.grid(row=0, column=0, sticky="ew", padx=4, pady=(0, 2))
    app.gait_op_mode_var = tk.StringVar(value="sim")
    app.gait_widgets["operation_controls"] = []
    for value, text in (("sim", "🟣 模拟"), ("live", "🔴 S0–S7 分阶段执行")):
        control = ttk.Radiobutton(ops, text=text, value=value,
                                 variable=app.gait_op_mode_var)
        control.pack(side="left", padx=3)
        app.gait_widgets["operation_controls"].append(control)
    sim_area = ttk.Frame(parent)
    live_area = ttk.Frame(parent)

    def _select_operation_group(*_args):
        from .gait_twin import sync_twin_mode
        sync_twin_mode(app)
        show_sim = app.gait_op_mode_var.get() != "live"
        (sim_area if show_sim else live_area).grid(row=1, column=0, sticky="ew", padx=4)
        (live_area if show_sim else sim_area).grid_remove()
    # var 写入即切组：radiobutton 点击与测试 var.set 走同一条路。
    app.gait_op_mode_var.trace_add("write", _select_operation_group)

    # ── 模拟组：换位方式 + 模拟三键 + 状态/干跑报告 ──
    # 2026-09-22 按用户要求：摆动侧与顺/逆方向合并为四个换位方式选项
    # （左顺移/左逆移/右顺移/右逆移）。切换即同步 side/arc 并静默重跑，
    # 预览与【▶ 模拟下一步】永远对应当前选择。
    modes = ttk.Frame(sim_area)
    modes.grid(row=0, column=0, sticky="ew")
    app.gait_widgets["action_choices"] = []
    for value, text in (("L+", "左顺移"), ("L-", "左逆移"),
                        ("R+", "右顺移"), ("R-", "右逆移")):
        choice = ttk.Radiobutton(
            modes, text=text, value=value,
            variable=app.gait_mode_var,
            command=lambda v=value: _gait_mode_selected(app, v),
        )
        choice.pack(side="left", padx=3)
        app.gait_widgets["action_choices"].append(choice)
    ttk.Label(sim_area, text="点选动作显示虚线候选落点（左蓝/右橙）；不会移动电机。",
              wraplength=260, foreground="#6b7280").grid(row=1, column=0, sticky="w")
    from .gait_simulation import toggle_simulation, reset_simulation, discard_simulation_step
    sim_buttons = ttk.Frame(sim_area)
    sim_buttons.grid(row=2, column=0, sticky="w", pady=3)
    app.gait_widgets["sim_btn"] = ttk.Button(
        sim_buttons, text="▶ 模拟下一步", command=lambda: toggle_simulation(app))
    app.gait_widgets["sim_btn"].pack(side="left", padx=3)
    for key, text, command in (("sim_cancel", "取消本步", discard_simulation_step),
                               ("sim_reset", "重置模拟", reset_simulation)):
        app.gait_widgets[key] = ttk.Button(sim_buttons, text=text, command=lambda c=command: c(app))
        app.gait_widgets[key].pack(side="left", padx=3)
    app.gait_widgets["sim_status"] = ttk.Label(
        sim_area, text="曲线跨步滑动；地图仅显示上步淡色、当前深色。清路径不重置站位。",
        wraplength=260, foreground="#7c3aed", justify="left")
    app.gait_widgets["sim_status"].grid(row=3, column=0, sticky="ew")
    result = ttk.Label(
        sim_area, textvariable=app.gait_report_var,
        wraplength=260, justify="left",
    )
    result.grid(row=4, column=0, sticky="ew", pady=(2, 0))
    app.gait_widgets["report_label"] = result

    # ── 实操组：中止/重置流程 + 四个开始 + 单步确认/一键执行 + 阶段信息 ──
    control = ttk.Frame(live_area)
    control.grid(row=0, column=0, sticky="w")
    # Abort stays at the top of the live operation group.
    app.gait_widgets["abort"] = ttk.Button(
        control, text="⛔ 中止", command=app._gait_abort_clicked)
    app.gait_widgets["abort"].pack(side="left", padx=3)
    app.gait_widgets["reset"] = ttk.Button(
        control, text="重置流程", command=app._gait_reset_run)
    app.gait_widgets["reset"].pack(side="left", padx=3)
    # 2026-10-20：中止/卡住后按执行前快照低速回移（仅 X42S 后端）。
    app.gait_widgets["restore_pre_run"] = ttk.Button(
        control, text="↩ 恢复到本次开始前",
        command=getattr(app, "_gait_restore_pre_run_clicked", lambda: None))
    app.gait_widgets["restore_pre_run"].pack(side="left", padx=3)
    live = ttk.Frame(live_area)
    live.grid(row=1, column=0, sticky="w", pady=3)
    # 执行入口与四种换位方式一一对应，方向由按钮显式携带，不依赖预览区
    # 当前选择（防止"看逆向、走顺向"误操作）。
    for row, row_actions in enumerate((
        (("start_left_cw", "开始左顺移", "left", 60.0),
         ("start_left_ccw", "开始左逆移", "left", -60.0)),
        (("start_right_cw", "开始右顺移", "right", 60.0),
         ("start_right_ccw", "开始右逆移", "right", -60.0)),
    )):
        for col, (key, text, side, arc) in enumerate(row_actions):
            app.gait_widgets[key] = ttk.Button(
                live, text=text,
                command=lambda s=side, a=arc: app._gait_start_run_clicked(s, a))
            app.gait_widgets[key].grid(row=row, column=col, padx=3, pady=1, sticky="ew")
    app.gait_widgets["advance"] = ttk.Button(
        live, text="✓ 确认并执行本阶段", command=app._gait_stage_confirmed)
    app.gait_widgets["advance"].grid(row=2, column=0, sticky="ew", padx=3)
    # 2026-09-29 按用户要求：一键执行——本步剩余阶段自动连走，不再逐
    # 阶段弹窗确认（录视频用）；执行中可【⛔ 中止】停车，再点本按钮
    # 停在当前阶段后回到人工确认。
    app.gait_widgets["auto"] = ttk.Button(
        live, text="⚡ 一键执行", command=app._gait_auto_clicked)
    app.gait_widgets["auto"].grid(row=2, column=1, sticky="ew", padx=3)

    current = ttk.Label(
        live_area, textvariable=app.gait_stage_var,
        font=("Microsoft YaHei", 10, "bold"), justify="left", wraplength=260,
    )
    current.grid(row=2, column=0, sticky="ew", pady=(6, 2))
    app.gait_widgets["confirm_text"] = ttk.Label(
        live_area, text="", wraplength=260, justify="left", foreground="#334155",
    )
    app.gait_widgets["confirm_text"].grid(row=3, column=0, sticky="ew", pady=2)
    app.gait_widgets["progress"] = ttk.Label(
        live_area, text="", foreground="#1565c0", font=("Consolas", 10, "bold"),
        wraplength=260, justify="left",
    )
    app.gait_widgets["progress"].grid(row=4, column=0, sticky="ew")
    app.gait_widgets["angles"] = ttk.Label(live_area, text="", foreground="#1565c0",
                                           wraplength=260, justify="left")
    app.gait_widgets["angles"].grid(row=5, column=0, sticky="ew", pady=2)
    app.gait_widgets["state"] = ttk.Label(
        live_area, textvariable=app.gait_state_var,
        font=("Microsoft YaHei", 10, "bold"), wraplength=260, justify="left",
    )
    app.gait_widgets["state"].grid(row=6, column=0, sticky="ew", pady=(0, 4))
    _select_operation_group()

    viewport = ScrollableFrame(parent, width=450)
    viewport.grid(row=2, column=0, sticky="nsew")
    app.gait_widgets["run_viewport"] = viewport
    right = viewport.content
    right.columnconfigure(0, weight=1)
    build_twin_panel(app, right)

    def resize_text(event):
        width = max(220, event.width - 36)
        for label in (current, result, *(app.gait_widgets[key] for key in
                      ("confirm_text", "progress", "angles", "state", "sim_status"))):
            label.configure(wraplength=width)
    parent.bind("<Configure>", resize_text, add="+")
    viewport.bind_navigation()


# ── 参数字段 ↔ GaitParams ──────────────────────────────────

def _refresh_trajectory_status(app):
    label = getattr(app, "gait_widgets", {}).get("trajectory_status")
    if label is not None:
        label.configure(
            text="HIGH / LOW 后台自动判定", foreground="#334155")


def _refresh_geometry_fields(app):
    entry = getattr(app, "gait_widgets", {}).get("spacing_entry")
    if entry is None:
        return
    try:
        edge = _field_number(app, "arm_length_mm")
        if edge <= 0:
            raise ValueError("边长必须大于0")
        spacing = replace(app.gait_params.geometry, arm_length_mm=edge).touching_center_distance_mm
        if not math.isfinite(spacing):
            raise ValueError("中心距必须是有限数值")
        display = f"{spacing:.6f}"
    except (ValueError, tk.TclError):
        display = "—"
    app.gait_derived_spacing_var.set(display)
    entry.configure(state="readonly", textvariable=app.gait_derived_spacing_var)
    app.gait_widgets["spacing_caption"].configure(
        text="中心距 d=√3a（自动，mm）")


def _invalidate_calibration(app, *, reference_changed=True):
    _refresh_trajectory_status(app)
    _refresh_geometry_fields(app)
    if not getattr(app, "_gait_loading_fields", False):
        app.gait_widgets.get("twin", {}).pop("candidate", None)
        if reference_changed:
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
    app.gait_trajectory_var.set(TWO_MODE)
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
        if key == "d_mm":
            # Ignore stale independent spacing from historical documents.
            continue
        value = _field_number(app, key)
        if key in INTEGER_FIELDS:
            if value != int(value):
                raise ValueError(f"步态参数 {key} 必须是整数")
            value = int(value)
        geometry_updates[key] = value
    param_updates = {}
    geometry_updates["d_mm"] = math.sqrt(3) * geometry_updates["arm_length_mm"]
    param_updates["trajectory_mode"] = TWO_MODE
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
    # Ordinary save/start may retain a confirmation, never re-sign changed
    # hardware. Only the explicitly confirmed baseline transaction signs it.
    param_updates["calibration_fingerprint"] = (
        base.calibration_fingerprint if confirmed else None)
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
    missing = [role for role, zero, signature in (
        ("Mr1", app.gait_params.mr1_zero_deg, app.gait_params.mr1_zero_signature),
        ("Mr2", app.gait_params.mr2_zero_deg, app.gait_params.mr2_zero_signature))
        if zero is None or not signature]
    reference_problem = None
    if missing:
        reference_problem = "缺少" + "/".join(missing) + "基准"
    elif verified:
        try:
            if app.gait_params.calibration_fingerprint != app._gait_hardware_fingerprint():
                reference_problem = "绑定/传动已变化"
            check_signature = getattr(app, "_gait_zero_signature", None)
            if check_signature and any(saved != check_signature(role, app.gait_params)
                    for role, saved in (("Mr1", app.gait_params.mr1_zero_signature),
                                        ("Mr2", app.gait_params.mr2_zero_signature))):
                reference_problem = "旋转方向/基准签名已变化"
        except (RuntimeError, ValueError):
            reference_problem = "绑定未完整"
    verified = verified and reference_problem is None
    needs_recovery = getattr(app, "_gait_needs_recovery", False)
    app.gait_widgets["prepare_baseline"].configure(
        state="disabled" if getattr(app, "_gait_owned", {}) else "normal")
    app.gait_widgets["readiness"].configure(
        text="中止后须恢复支撑/所选起步摆放，再记录双侧基准" if needs_recovery else (
            "基准已保存；连续换步无需重记，启动仍完整预检" if verified else
            (reference_problem or "尚未确认现场基准") + "：摆好实物后点上方按钮一次"),
        foreground="#b42318" if needs_recovery or not verified else "#16803a")
    snapshot = run.describe() if run is not None else None
    if snapshot is None:
        app.gait_stage_var.set("未开始（点选动作看虚线候选 → 确认后开始；启动仍完整校验）")
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
    restore = app.gait_widgets.get("restore_pre_run")
    if restore is not None:
        # 有执行前快照且步态未占用四轴时可恢复（中止/卡住后的回位入口）。
        restore.configure(
            state="normal" if (getattr(app, "_gait_recovery_snapshot", None)
                               is not None and not getattr(app, "_gait_owned", {}))
            else "disabled")
    app.gait_widgets["progress"].configure(text=progress_text or "")
    refresh_twin_panel(app)
    # 2026-10-08 阶段标签没内容时是空行，把 5×5 图压出视口（实操区比
    # 模拟区高出一截的"空行"来源）。空文字收起（grid_remove 不占行），
    # 有文字再展开——未开始时实操区高度与模拟区基本一致，执行时阶段
    # 文字/确认清单/进度/角度/状态逐项出现。
    for key in ("confirm_text", "progress", "angles", "state"):
        label = app.gait_widgets[key]
        label.grid() if label.cget("text") else label.grid_remove()


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
