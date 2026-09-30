"""Situational twin map: planned-trajectory overlay + live pulse-estimated pose.

2026-09-30 合并"轨迹与间隙"预览到孪生画面：同一 5×5 地图上分两种数据源
模式——评估模拟（计划层，蓝抬头，不动电机）与实机脉冲（实机层，红抬头，
无编码器估算）。执行开始自动切实机并隐藏计划层；重新干跑/重置流程回
评估。最近足高亮与爪端-高杆距离曲线在两种模式下都提供（口径=爪端点到
杆表面，mm）。后续编码器接入时在此层再加"编码器实测"数据源即可。
"""
import math
import time
import tkinter as tk
from collections import deque
from tkinter import ttk, messagebox

from motor_control.gait_avoidance import lattice_coordinates
from motor_control.gait_twin import ROLES, TwinHistory, pad_center
from motor_control.gait_map import map_pads, map_label, start_pair_reference
from motor_control.ui.common import bind_canvas_view, canvas_view_zoom, reset_canvas_view

# ── 爪端轨迹配色（计划层三条摆动爪轨迹，与旧预览一致） ──────
LEG_TRACK_COLORS = ("#0d9488", "#7c3aed", "#b45309")   # 腿1/2/3 爪端轨迹色
# 曲线 6 条：左足三爪蓝系深浅、右足三爪橙系深浅
CURVE_COLORS = (("#1d4ed8", "#3b82f6", "#93c5fd"),   # 左足 爪1/2/3
                ("#9a3412", "#ea580c", "#fdba74"))   # 右足 爪1/2/3
_SIX_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, -1), (-1, 1))
CURVE_SAMPLE_INTERVAL_S = 0.15     # 实机曲线采样节流
CURVE_MAX_POINTS = 1800            # 实机曲线有界缓存（≈4.5 分钟@0.15s）
CURVE_WINDOW_S = 90.0              # 实机曲线显示滚动窗口


def twin_pad_window():
    """Fixed 5 rows x 5 columns, same world coordinates throughout a session."""
    return map_pads()


# 地图邻域（25 格外扩一圈）：只用于"最近足/曲线"的高杆距离计算——避障
# 认证范围不因 5×5 裁剪（边缘支座旁边地图外的高杆同样可能最近）。
def _neighborhood_pads():
    pads = map_pads()
    inside = {lattice_coordinates(n) for n in pads}
    merged = dict(pads)
    for i, j in list(inside):
        for di, dj in _SIX_STEPS:
            key = (i + di, j + dj)
            if key in inside:
                continue
            name = f"邻座({key[0]},{key[1]})"
            merged.setdefault(name, (key[0] + key[1] / 2,
                                     math.sqrt(3) * key[1] / 2))
    return merged


_NEIGHBORHOOD = None


def neighborhood_pads():
    global _NEIGHBORHOOD
    if _NEIGHBORHOOD is None:
        _NEIGHBORHOOD = _neighborhood_pads()
    return _NEIGHBORHOOD


def high_rods_mm(params):
    """邻域全部高杆的 mm 坐标 [(name, j, x, y), ...]（含地图外一圈）。"""

    d = params.geometry.d_mm
    arm = params.geometry.arm_length_mm
    rods = []
    for name, (nx, ny) in neighborhood_pads().items():
        cx, cy = nx * d, ny * d
        for j in range(3):
            a = math.radians(90.0 + 120.0 * j)
            rods.append((name, j, cx + arm * math.cos(a), cy + arm * math.sin(a)))
    return rods


_ROD_CACHE = None


def _rods_cached(params):
    global _ROD_CACHE
    key = (params.geometry.d_mm, params.geometry.arm_length_mm)
    if _ROD_CACHE is None or _ROD_CACHE[0] != key:
        _ROD_CACHE = (key, high_rods_mm(params))
    return _ROD_CACHE[1]


def foot_tips_mm(center_norm, psi_deg, params):
    """一足三爪端 mm 坐标：中心（归一化）+ 爪臂长·dir(ψ+120k)。"""

    d = params.geometry.d_mm
    arm = params.geometry.arm_length_mm
    cx, cy = center_norm[0] * d, center_norm[1] * d
    tips = []
    for k in range(3):
        a = math.radians(psi_deg + 120.0 * k)
        tips.append((cx + arm * math.cos(a), cy + arm * math.sin(a)))
    return tips


def tip_surface_distances(pose, params):
    """live pose 的 6 爪端对邻域高杆的最近表面距离 [左0..2, 右0..2] (mm)。"""

    node_r = params.geometry.node_radius_mm
    rods = _rods_cached(params)
    values = []
    for side in ("left", "right"):
        foot = pose["feet"][side]
        for tip in foot_tips_mm(foot["center"], foot["psi_deg"], params):
            values.append(min(math.dist(tip, (r[2], r[3])) for r in rods) - node_r)
    return values


def plan_tip_series(report, params):
    """计划层 6 爪距离序列：[(t_s, [左0..2, 右0..2]), ...]。

    摆动足 3 爪按 samples 逐点计算；支撑足 3 爪踩在支点低节点上，
    距离为常数（起点一次算清）。时间轴按 S4 段时长把等 φ 采样近似
    映射到秒（速度剖面差异忽略，作目测标尺）。
    """

    node_r = params.geometry.node_radius_mm
    rods = _rods_cached(params)
    d, arm = params.geometry.d_mm, params.geometry.arm_length_mm
    start, target, pivot, _bearing = report.route
    pxy = pad_center(pivot)
    px, py = pxy[0] * d, pxy[1] * d
    support = []
    for k in range(3):
        a = math.radians(30.0 + 120.0 * k)
        tip = (px + arm * math.cos(a), py + arm * math.sin(a))
        support.append(min(math.dist(tip, (r[2], r[3])) for r in rods) - node_r)
    # 6 条顺序固定 [左足0..2, 右足0..2]：支撑足占另一侧的三个槽位。
    swing_slot = 0 if report.side == "left" else 1
    support_slot = 1 - swing_slot
    n = max(1, len(report.samples) - 1)
    series = []
    for index, sample in enumerate(report.samples):
        swing_vals = []
        for k in range(3):
            a = math.radians(sample.psi_deg + 120.0 * k)
            tip = (sample.center[0] + arm * math.cos(a),
                   sample.center[1] + arm * math.sin(a))
            swing_vals.append(min(math.dist(tip, (r[2], r[3])) for r in rods) - node_r)
        vals = [0.0] * 6
        vals[swing_slot * 3:swing_slot * 3 + 3] = swing_vals
        vals[support_slot * 3:support_slot * 3 + 3] = support
        series.append((index / n * _plan_duration_s(report, params), vals))
    return series


def _plan_duration_s(report, params):
    try:
        from motor_control.gait_planner import plan_gait_stages
        s4 = next(s for s in plan_gait_stages(
            params, side=report.side,
            swing_psi_start_deg=report.samples[0].psi_deg) if s.stage_id == "S4")
        return max(1e-6, s4.duration_s)
    except Exception:
        return max(1.0, len(report.samples) - 1.0)


def sync_twin_start_fields(app):
    view = app.gait_widgets.get("twin")
    if view:
        for side, name in zip(("left", "right"), app.gait_params.initial_supports):
            view[f"start_{side}"].set(map_label(name))


def clear_twin_history(app):
    """Display operation only: retain stance, calibration and pulse positions."""
    view = app.gait_widgets["twin"]
    view["history"].clear(view["snapshot"])
    view["curve_live"].clear()
    view["curve_seg"], view["curve_last_t"] = 0, None
    _draw(app, view)


def apply_twin_start(app):
    from motor_control.gait_executor import GaitExecutorError
    from motor_control.state_store import StateStoreError
    from motor_control.ui.gait_tab import collect_gait_params, load_gait_fields, stop_preview_animation
    view = app.gait_widgets["twin"]
    names = {map_label(n): n for n in map_pads()}
    try:
        left, right = (names[view[f"start_{s}"].get()] for s in ("left", "right"))
        beta, _ = start_pair_reference((left, right))
        with app.state_lock:
            app._gait_check_start_edit()
        params = collect_gait_params(app, app.gait_params)
        if not messagebox.askokcancel(
                "确认起步位置（不移动电机）",
                f"左足：{map_label(left)}；右足：{map_label(right)}；横梁 β₀={beta:g}°。\n"
                "请先人工摆到选定两格，三爪均踩低节点，所有轴静止。\n"
                "应用后清除旧轨迹、旧旋转零位与标定确认；不会改变电机脉冲坐标。\n"
                "随后须重新记两侧旋转零位并确认标定。"):
            return
        app._gait_apply_start_pair(left, right, params=params)  # Rechecks idle after the dialog.
    except (ValueError, KeyError, GaitExecutorError, StateStoreError) as exc:
        messagebox.showwarning("起步位置未更改", str(exc))
        return
    stop_preview_animation(app)
    load_gait_fields(app)
    view["select_side"].set("")
    view["curve_live"].clear()
    view["curve_seg"], view["curve_last_t"] = 0, None
    app._refresh_gait_ui()


def _map_click(app, event):
    view = app.gait_widgets["twin"]
    side = view["select_side"].get()
    if side not in ("left", "right") or view.get("start_busy"):
        return
    project = view.get("project")
    if project is None:
        return
    pads = map_pads()
    name = min(pads, key=lambda n: math.dist(project(pads[n]), (event.x, event.y)))
    if math.dist(project(pads[name]), (event.x, event.y)) > view["map_scale"] / math.sqrt(3):
        return
    view[f"start_{side}"].set(map_label(name))
    _draw(app, view)


def build_twin_panel(app, parent):
    panel = ttk.LabelFrame(parent, text="态势图 · 5×5 地图（评估模拟 / 实机脉冲）")
    panel.grid(row=0, column=0, sticky="ew", padx=6, pady=3)
    panel.columnconfigure(0, weight=1)
    # 2026-09-30 按用户要求：抬头变色区分数据源——评估模拟（计划）蓝、
    # 实机脉冲（真机）红；执行开始自动切红并隐藏计划层，重新干跑/重置
    # 回蓝。模式也可手动切换（工具栏两态选择）。
    title = ttk.Label(panel, text="🔴 实机脉冲 · 无编码器估算",
                      foreground="#b91c1c", font=("Microsoft YaHei", 10, "bold"))
    title.grid(row=0, column=0, sticky="w", padx=8, pady=(4, 0))
    # 计划控件槽位（换位方式 radio / 预览校验 / 模拟动作 / 干跑报告）由
    # gait_tab 填充：参数与联动回调都在那一侧，本模块只提供容器。
    plan_slot = ttk.Frame(panel)
    plan_slot.grid(row=1, column=0, sticky="ew", padx=6)
    plan_slot.columnconfigure(0, weight=1)
    canvas = tk.Canvas(panel, width=240, height=340, background="white",
                       highlightthickness=1, highlightbackground="#cbd5e1")
    canvas.grid(row=2, column=0, sticky="ew", padx=6, pady=5)
    status = ttk.Label(panel, text="等待电机绑定", wraplength=260, justify="left")
    status.grid(row=6, column=0, sticky="ew", padx=6, pady=3)
    coordinates = ttk.Label(panel, text="", wraplength=260, justify="left")
    coordinates.grid(row=7, column=0, sticky="ew", padx=6, pady=3)
    table = ttk.Frame(panel)
    table.grid(row=8, column=0, sticky="ew", padx=6, pady=3)
    table.columnconfigure((0, 1, 2), weight=1)
    for col, text in enumerate(("电机 / 轴", "估算坐标", "指令目标")):
        ttk.Label(table, text=text, foreground="#64748b").grid(row=0, column=col, sticky="w")
    rows = {}
    for row, role in enumerate(ROLES, 1):
        rows[role] = tuple(ttk.Label(table, text="—", font=("Consolas", 9)) for _ in range(3))
        for col, label in enumerate(rows[role]):
            label.grid(row=row, column=col, sticky="w", padx=(0, 5), pady=2)
    legend = ttk.Label(panel, text="蓝抬头=评估模拟(计划·不动电机) 红抬头=实机脉冲(无编码器)\n"
                       "足色:蓝=左足 橙=右足 灰=断连冻结 绿点=低节点(落脚) 红点=高节点(红杆)\n"
                       "评估层:虚线=摆动中心轨迹 彩线=三爪端轨迹(青/紫/棕) 细足=动画帧\n"
                       "黄圈+数字=6爪中与红杆最近者(爪端到杆表面距离,mm)\n"
                       "曲线图:6爪距离-时间,粗线=最小值;0=杆表面,虚线=参考间隙δ\n"
                       "画面:左键点一下(蓝框)=选中,滚轮缩放;移出自动取消 中键拖动=平移 双击=复位\n"
                       "固定5×5地图，B=(0,0)，上为+Y、右为+X；行号自下向上。\n"
                       "XY 为中心距=1的估算；Z 为本步位移，非接触检测。\n"
                       "实线=已回报脉冲轨迹，圆点=完成步落脚；执行中不画计划层。\n"
                       "▶ 模拟=逐帧回放 再点=暂停/继续；四个【开始…移】按钮与换位方式一一对应",
                       wraplength=260, justify="left", foreground="#64748b")
    legend.grid(row=9, column=0, sticky="ew", padx=6, pady=3)
    view = {"canvas": canvas, "status": status, "rows": rows, "snapshot": {},
            "last_pose": None, "reference_key": None, "coordinates": coordinates,
            "history": TwinHistory(), "show_path": tk.BooleanVar(value=True),
            "select_side": tk.StringVar(value=""),
            "start_left": tk.StringVar(), "start_right": tk.StringVar(),
            "title": title, "plan_slot": plan_slot,
            "mode_var": tk.StringVar(value="live"),
            "curve_live": deque(maxlen=CURVE_MAX_POINTS),
            "curve_plan": [], "curve_seg": 0, "curve_last_t": None}
    app.gait_widgets["twin"] = view
    toolbar = ttk.Frame(panel)
    toolbar.grid(row=3, column=0, sticky="ew", padx=6)
    ttk.Checkbutton(toolbar, text="显示路径", variable=view["show_path"],
                    command=lambda: _draw(app, view)).grid(row=0, column=0)
    view["clear_button"] = ttk.Button(toolbar, text="清空路径", command=lambda: clear_twin_history(app))
    view["clear_button"].grid(row=0, column=1, padx=3)
    # 模式切换（评估模拟/实机脉冲）短文本放首行尾部——整行请求宽度必须
    # 低于右列视口宽（第二行放满会把起步下拉框挤出滚动视口，布局测试覆盖）。
    for col, (mode, text) in enumerate((("plan", "🔵 评估"), ("live", "🔴 实机"))):
        ttk.Radiobutton(toolbar, text=text, variable=view["mode_var"], value=mode,
                        command=lambda: _draw(app, view)).grid(row=0, column=col + 2, padx=3)

    def zoom(delta):
        from types import SimpleNamespace
        state = app.gait_widgets["twin_view"]
        state["active"] = True
        canvas_view_zoom(state, SimpleNamespace(widget=canvas, delta=delta,
                         x=canvas.winfo_width()/2, y=canvas.winfo_height()/2),
                         lambda: _draw(app, view))

    for col, (label, callback) in enumerate((
            ("放大 +", lambda: zoom(240)), ("缩小 −", lambda: zoom(-240)),
            ("全图", lambda: (reset_canvas_view(app.gait_widgets["twin_view"]), _draw(app, view))))):
        ttk.Button(toolbar, text=label, width=8, command=callback).grid(row=1, column=col, pady=3)

    # 爪端-高杆表面距离曲线（评估/实机两模式共用一图）
    curve_frame = ttk.LabelFrame(panel, text="爪端-高杆表面距离 (mm) · 横轴时间")
    curve_frame.grid(row=4, column=0, sticky="ew", padx=6, pady=3)
    curve_frame.columnconfigure(0, weight=1)
    curve_canvas = tk.Canvas(curve_frame, width=240, height=110, background="white",
                             highlightthickness=1, highlightbackground="#e2e8f0")
    curve_canvas.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
    view["curve_canvas"] = curve_canvas

    start = ttk.LabelFrame(panel, text="起步两格（仅设置，不移动电机）")
    start.grid(row=5, column=0, sticky="ew", padx=6, pady=3)
    start.columnconfigure(1, weight=1)
    options = tuple(map_label(n) for n in map_pads())
    view["start_controls"] = []
    for row, (side, label) in enumerate((("left", "左足"), ("right", "右足"))):
        radio = ttk.Radiobutton(start, text=f"地图选{label}", variable=view["select_side"], value=side)
        radio.grid(row=row, column=0, padx=3)
        choice = ttk.Combobox(start, textvariable=view[f"start_{side}"], values=options,
                              state="readonly", width=12)
        choice.grid(row=row, column=1, sticky="ew", padx=3, pady=2)
        choice.bind("<<ComboboxSelected>>", lambda _e: _draw(app, view))
        view["start_controls"].extend(((radio, "normal"), (choice, "readonly")))
    browse = ttk.Radiobutton(start, text="仅浏览", variable=view["select_side"], value="")
    browse.grid(row=2, column=0)
    view["apply_button"] = ttk.Button(start, text="应用起步位置", command=lambda: apply_twin_start(app))
    view["apply_button"].grid(row=2, column=1, sticky="ew", padx=3, pady=3)
    view["start_controls"].append((view["apply_button"], "normal"))
    sync_twin_start_fields(app)
    # 视图状态与计划层共用同一交互（左键选中+滚轮缩放+中键平移+双击复位）。
    app.gait_widgets["twin_view"] = {"zoom": 1.0, "pan_x": 0.0, "pan_y": 0.0,
                                     "active": False}
    bind_canvas_view(canvas, app.gait_widgets["twin_view"],
                     lambda: _draw(app, view))
    canvas.bind("<Button-1>", lambda e: _map_click(app, e), add="+")

    def resized(event):
        for label in (status, legend, coordinates):
            label.configure(wraplength=max(200, event.width - 8))
        _draw(app, view)
    canvas.bind("<Configure>", resized)


def draw_twin(app):
    """公开重画入口（gait_tab 兼容 shim / 缩放平移回调共用）。"""
    view = (getattr(app, "gait_widgets", None) or {}).get("twin")
    if view is not None:
        _draw(app, view)


def refresh_twin_panel(app):
    view = app.gait_widgets.get("twin")
    if view is None:
        return
    provider = getattr(app, "_gait_twin_snapshot", None)
    snapshot = provider() if provider else {"axes": {}, "pose": None, "message": "等待电机绑定"}
    view["history"].observe(snapshot)
    if snapshot.get("reference_key") != view["reference_key"]:
        view["last_pose"] = None
        view["reference_key"] = snapshot.get("reference_key")
        view["curve_seg"] += 1
    if snapshot.get("pose") is not None:
        view["last_pose"] = snapshot["pose"]
    view["snapshot"] = snapshot
    busy = bool(getattr(app, "_gait_owned", {})) or any(
        a.get("target_position") is not None or a.get("state") in ("STARTING", "MOVING", "CONTINUOUS")
        for a in snapshot.get("axes", {}).values())
    view["start_busy"] = busy
    for control, idle_state in view["start_controls"]:
        control.configure(state="disabled" if busy else idle_state)
    healthy = snapshot.get("state") in ("estimated", "waiting")
    route = snapshot.get("pose", {}).get("route") if snapshot.get("pose") else None
    view["status"].configure(text=snapshot["message"] + (f"\n{route}" if route else ""),
                             foreground="#1565c0" if healthy else "#b45309")
    for role, labels in view["rows"].items():
        axis = snapshot["axes"].get(role, {})
        unit = axis.get("unit", "")
        labels[0].configure(text=f"{role} / {axis.get('axis', '—')}")
        labels[1].configure(text=_coordinate(axis.get("position"), unit, estimated=True))
        labels[2].configure(text=_coordinate(axis.get("target_position"), unit))
        good = (axis.get("position_trusted") and axis.get("binding_valid")
                and axis.get("state") in ("IDLE", "STARTING", "MOVING", "CONTINUOUS")
                and not (axis.get("stale") and axis.get("target_position") is not None))
        for label in labels:
            label.configure(foreground="#334155" if good else "#9a6700")
    pose = snapshot.get("pose")
    if pose:
        left, right = pose["feet"]["left"], pose["feet"]["right"]
        text = (f"估算 φ={pose['phi_deg']:.2f}°，β={pose['beta_deg']:.2f}°\n"
                f"世界 ψ 左/右={left['psi_deg']:.2f}° / {right['psi_deg']:.2f}°")
    else:
        text = snapshot["message"]
    app.gait_widgets["angles"].configure(text=text)
    if pose is not None and healthy:
        _record_live_curve(app, view, pose)
    _draw(app, view)


def _record_live_curve(app, view, pose):
    """实机曲线采样：节流追加 6 爪距离；换步/断连分段。"""

    now = time.monotonic()
    last = view.get("curve_last_t")
    if last is not None and now - last < CURVE_SAMPLE_INTERVAL_S:
        return
    view["curve_last_t"] = now
    base = view.get("curve_t0")
    if base is None:
        base = view["curve_t0"] = now
    try:
        values = tip_surface_distances(pose, app.gait_params)
    except Exception:
        return
    view["curve_live"].append((now - base, values, view["curve_seg"]))


def _coordinate(value, unit, estimated=False):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return "—"
    return f"{'≈' if estimated else ''}{value:+.2f}{unit}"


def twin_projection(width, height, xmin, xmax, ymin, ymax,
                    zoom=1.0, pan_x=0.0, pan_y=0.0):
    """固定地图包围盒 → 画布投影（缩放/平移以画布中心为原点复合）。

    独立成纯函数供离线测试：滚轮缩放的"指针不动点"性质直接可验。
    """

    scale = min((width - 26) / (xmax - xmin), (height - 100) / (ymax - ymin))

    def project(point):
        x, y = point
        base_x = width / 2 + (x - (xmin + xmax) / 2) * scale
        base_y = 15 + (ymax - y) * scale
        return (width / 2 + (base_x - width / 2) * zoom + pan_x,
                height / 2 + (base_y - height / 2) * zoom + pan_y)
    return project, scale


def _mode(app, view):
    var = view.get("mode_var")
    try:
        return var.get()
    except Exception:
        return "live"


def _draw(app, view):
    canvas = view["canvas"]
    canvas.delete("all")
    width, height = canvas.winfo_width(), canvas.winfo_height()
    if width < 20 or height < 20:
        return
    mode = _mode(app, view)
    snapshot = view["snapshot"]
    live = snapshot.get("pose")
    pose = live or view["last_pose"]
    all_pads = twin_pad_window()
    report = getattr(app, "_gait_last_report", None)
    plan_report = report if (mode == "plan" and report is not None) else None
    route_pads = (pose.get("route_pads", {}) if pose else {})
    if plan_report is not None:
        start, target, pivot, _bearing = plan_report.route
        route_pads = {name: pad_center(name) for name in (start, target, pivot)}
    xs, ys = zip(*all_pads.values())
    xmin, xmax = min(xs)-.6, max(xs)+.6
    ymin, ymax = min(ys)-.6, max(ys)+.6
    twin_view = (getattr(app, "gait_widgets", None) or {}).get("twin_view") or {}
    zoom = float(twin_view.get("zoom", 1.0))
    pan_x = float(twin_view.get("pan_x", 0.0))
    pan_y = float(twin_view.get("pan_y", 0.0))
    project, scale = twin_projection(width, height, xmin, xmax, ymin, ymax,
                                     zoom, pan_x, pan_y)

    view["project"], view["map_scale"] = project, scale*zoom
    _update_title(app, view, mode, snapshot, live)
    selected = {view[f"start_{side}"].get(): side for side in ("left", "right")}
    applied = dict(zip(("left", "right"), app.gait_params.initial_supports))

    for name, center in all_pads.items():
        on_route = name in route_pads
        selection = selected.get(map_label(name))
        nodes = [(center[0]+math.cos(math.radians(30+i*60))/math.sqrt(3),
                  center[1]+math.sin(math.radians(30+i*60))/math.sqrt(3)) for i in range(6)]
        canvas.create_polygon(*(v for node in nodes for v in project(node)),
                              fill=("#eff6ff" if selection == "left" else
                                    "#fff7ed" if selection == "right" else ""),
                              outline="#94a3b8" if on_route else "#d1d5db",
                              width=1.5 if on_route else 1,
                              dash=() if on_route else (3, 3), tags="map_hex")
        x, y = project(center)
        canvas.create_text(x, y+10, text=map_label(name), fill="#475569",
                           font=("Microsoft YaHei", 8), tags="map_label")
        if selection:
            caption = ("起步" if applied[selection] == name else "待应用") + ("左" if selection == "left" else "右")
            canvas.create_text(x, y-9, text=caption,
                               fill="#2563eb" if selection == "left" else "#ea580c", tags="start_pad")
        dot = 2.5 if on_route else 1.8
        for i, node in enumerate(nodes):
            nx, ny = project(node)
            canvas.create_oval(nx-dot, ny-dot, nx+dot, ny+dot,
                               fill="#16a34a" if i % 2 == 0 else "#dc2626", outline="")

    history = view["history"]
    if view["show_path"].get():
        for segment in history.segments():
            if len(segment) < 2:
                continue
            for index, side, color in ((1, "left", "#60a5fa"), (2, "right", "#fb923c"),
                                       (3, "center", "#94a3b8")):
                coords = [v for point in segment for v in project(point[index])]
                canvas.create_line(*coords, fill=color, width=1.5, tags=f"history_{side}")
        for step, left, right in history.landings:
            x, y = project(tuple((a+b)/2 for a, b in zip(left, right)))
            canvas.create_text(x, y-8, text=str(step), fill="#64748b", tags="step_number")
            for point, color in ((left, "#2563eb"), (right, "#ea580c")):
                px, py = project(point)
                canvas.create_oval(px-3, py-3, px+3, py+3, outline=color, tags="landing")

    if plan_report is not None:
        _draw_plan_layer(app, view, plan_report, project)
        _draw_curve(app, view)
        return
    if pose is None:
        canvas.create_text(width/2, 14, text="起步位置待确认 / 标定，非实测位置",
                           width=width-12, fill="#b45309", tags="twin_placeholder")
        view["coordinates"].configure(text=f"已记录完成步：{history.completed_steps}；世界位置尚不可用")
        _draw_curve(app, view)
        return

    # 实机层：只画已回报/落定的脉冲姿态，绝不画指令的未来目标。
    left_xy, right_xy = (pose["feet"][s]["center"] for s in ("left", "right"))
    mid = tuple((a+b)/2 for a, b in zip(left_xy, right_xy))
    view["coordinates"].configure(text=(
        f"{'估算' if live else '历史冻结'} XY（中心距=1）· 已记录完成步 {history.completed_steps}\n"
        f"左 ({left_xy[0]:+.3f}, {left_xy[1]:+.3f})；右 ({right_xy[0]:+.3f}, {right_xy[1]:+.3f})\n"
        f"机构中心 ({mid[0]:+.3f}, {mid[1]:+.3f})"))
    centers = [project(pose["feet"][side]["center"]) for side in ("left", "right")]
    canvas.create_line(*centers[0], *centers[1], fill="#64748b",
                       width=2, tags="live_beam")
    params = app.gait_params
    nearest = None
    for side, color in (("left", "#2563eb"), ("right", "#ea580c")):
        foot = pose["feet"][side]
        color = color if live else "#94a3b8"
        x, y = project(foot["center"])
        arm = params.geometry.arm_length_mm / params.geometry.d_mm
        for i in range(3):
            theta = math.radians(foot["psi_deg"] + i*120)
            end = (foot["center"][0]+arm*math.cos(theta),
                   foot["center"][1]+arm*math.sin(theta))
            tip_xy = project(end)
            canvas.create_line(x, y, *tip_xy, fill=color, width=3,
                               tags=f"live_{side}")
            surface = _tip_surface(app, end, params, side, i)
            if surface is not None and (nearest is None or surface < nearest[0]):
                nearest = (surface, tip_xy)
        canvas.create_oval(x-5, y-5, x+5, y+5, outline=color,
                           fill=color, tags=f"{side}_center")
    if nearest is not None:
        _highlight_nearest(canvas, nearest[0], nearest[1])
        canvas.create_text(
            8, 26, anchor="w", fill="#b45309", font=("Microsoft YaHei", 9),
            text=f"6爪最近(当前) {nearest[0]:.2f}mm（爪端到杆表面）", tags="hud_nearest")
    max_z = max(1, *(abs(pose["feet"][side]["z_mm"]) for side in ("left", "right")))
    for side, fraction, label, color in (("left", 0.25, "左", "#2563eb"),
                                         ("right", 0.75, "右", "#ea580c")):
        x, y = width*fraction, height-45
        z = pose["feet"][side]["z_mm"]
        canvas.create_line(x-32, y, x+32, y, fill="#cbd5e1")
        canvas.create_line(x, y, x, y-z/max_z*28, width=5, fill=color if live else "#94a3b8",
                           tags=f"lift_{side}")
        canvas.create_text(x, height-17, text=f"{label} ΔZ≈{z:+.2f}mm", fill="#475569")
    canvas.create_text(width-8, 10, text="实机脉冲" if live else "实机脉冲（已冻结）",
                       anchor="ne", fill="#b91c1c" if live else "#9a6700", tags="corner_mode")
    _draw_curve(app, view)


def _update_title(app, view, mode, snapshot, live):
    if mode == "plan":
        view["title"].configure(text="🔵 评估模拟 · 计划轨迹（不动电机）",
                                foreground="#1d4ed8")
        return
    state = snapshot.get("state")
    frozen = state in ("untrusted", "stale") or live is None
    if frozen:
        view["title"].configure(text="🔴 实机脉冲 · 已冻结（断连/失信，非实测）",
                                foreground="#9a6700")
    else:
        view["title"].configure(text="🔴 实机脉冲 · 无编码器估算",
                                foreground="#b91c1c")


def _tip_surface(app, tip_norm, params, side, k):
    """单个爪端（归一化坐标）对邻域高杆的最近表面距离 mm。"""

    d = params.geometry.d_mm
    node_r = params.geometry.node_radius_mm
    tip = (tip_norm[0] * d, tip_norm[1] * d)
    best = min(math.dist(tip, (r[2], r[3])) for r in _rods_cached(params))
    return best - node_r


def _highlight_nearest(canvas, surface, tip_xy):
    x, y = tip_xy
    canvas.create_oval(x-7, y-7, x+7, y+7, outline="#f59e0b", width=2.5,
                       fill="", tags="nearest_tip")
    canvas.create_text(x, y-13, text=f"{surface:.1f}mm", fill="#b45309",
                       font=("Microsoft YaHei", 8, "bold"), tags="nearest_tip")


# ── 计划层（评估模拟）：轨迹 + 动画帧 ghost 足 + 最近足高亮 ──

def _draw_plan_layer(app, view, report, project):
    canvas = view["canvas"]
    params = app.gait_params
    geometry = params.geometry
    d = geometry.d_mm
    k_norm = 1.0 / d if d else 0.0
    anim = app.gait_widgets.get("preview_anim") or {}
    frame = anim.get("frame") if (anim.get("playing") or anim.get("paused")) else None

    def to_map(point_mm):
        return project((point_mm[0] * k_norm, point_mm[1] * k_norm))

    samples = report.samples
    # 摆动中心轨迹（蓝虚线）+ 三条爪端轨迹（彩线）
    arc = [v for s in samples for v in to_map(s.center)]
    if len(arc) >= 4:
        canvas.create_line(*arc, fill="#2563eb", width=2, dash=(6, 3), tags="plan_center")
    for leg in range(3):
        points = []
        for s in samples:
            a = math.radians(s.psi_deg + 120.0 * leg)
            tip = (s.center[0] + geometry.arm_length_mm * math.cos(a),
                   s.center[1] + geometry.arm_length_mm * math.sin(a))
            points.extend(to_map(tip))
        if len(points) >= 4:
            canvas.create_line(*points, fill=LEG_TRACK_COLORS[leg], width=1.5,
                               tags=f"plan_track_{leg}")
    index = len(samples) - 1 if frame is None else max(0, min(frame, len(samples) - 1))
    if frame is not None:
        walked = [v for s in samples[:index + 1] for v in to_map(s.center)]
        if len(walked) >= 4:
            canvas.create_line(*walked, fill="#2563eb", width=2.5, tags="plan_walked")
    sample = samples[index]
    # 摆动足 ghost（评估层一律细线/虚线观感，与实机实色区分）
    x, y = to_map(sample.center)
    for leg in range(3):
        a = math.radians(sample.psi_deg + 120.0 * leg)
        end = (sample.center[0] + geometry.arm_length_mm * math.cos(a),
               sample.center[1] + geometry.arm_length_mm * math.sin(a))
        canvas.create_line(x, y, *to_map(end), fill="#2563eb", width=2,
                           dash=(4, 2), tags="plan_ghost")
    canvas.create_oval(x-5, y-5, x+5, y+5, outline="#2563eb", width=2,
                       fill="", tags="plan_ghost")
    # 支撑足（支点低节点基准）与横梁
    start, target, pivot, _bearing = report.route
    pivot_norm = pad_center(pivot)
    pivot_mm = (pivot_norm[0] * d, pivot_norm[1] * d)
    px, py = to_map(pivot_mm)
    for leg in range(3):
        a = math.radians(30.0 + 120.0 * leg)
        end = (pivot_mm[0] + geometry.arm_length_mm * math.cos(a),
               pivot_mm[1] + geometry.arm_length_mm * math.sin(a))
        canvas.create_line(px, py, *to_map(end), fill="#94a3b8", width=2,
                           tags="plan_support")
    canvas.create_oval(px-5, py-5, px+5, py+5, outline="#475569", width=2,
                       fill="", tags="plan_support")
    canvas.create_line(x, y, px, py, fill="#94a3b8", width=2, tags="plan_beam")
    # 落脚点标记
    target_norm = pad_center(target)
    tx, ty = to_map((target_norm[0] * d, target_norm[1] * d))
    canvas.create_oval(tx-6, ty-6, tx+6, ty+6, outline="#16a34a", width=2,
                       dash=(3, 2), tags="plan_target")
    # 最近足高亮：全程最小（静态）或当前帧（动画中）
    nearest = None
    for leg in range(3):
        a = math.radians(sample.psi_deg + 120.0 * leg)
        tip = (sample.center[0] + geometry.arm_length_mm * math.cos(a),
               sample.center[1] + geometry.arm_length_mm * math.sin(a))
        surface = _tip_surface(app, (tip[0] * k_norm, tip[1] * k_norm), params, report.side, leg)
        if surface is not None and (nearest is None or surface < nearest[0]):
            nearest = (surface, to_map(tip))
    if nearest is not None:
        _highlight_nearest(canvas, nearest[0], nearest[1])
    worst = report.min_margin_sample
    hud = (f"模拟 φ={sample.phi_deg:.1f}° ψ={sample.psi_deg:.1f}°"
           f"（{index + 1}/{len(samples)}）" if frame is not None else
           f"评估计划 {start}→{target} 绕{pivot}")
    canvas.create_text(8, 26, anchor="w", fill="#1d4ed8",
                       font=("Microsoft YaHei", 9), text=hud, tags="hud_plan")
    if nearest is not None:
        canvas.create_text(
            8, 42, anchor="w", fill="#b45309", font=("Microsoft YaHei", 9),
            text=(f"6爪最近(当前帧) {nearest[0]:.2f}mm（爪端到杆表面）"),
            tags="hud_nearest")
    if worst is not None:
        canvas.create_text(
            8, 58, anchor="w", fill="#92400e", font=("Microsoft YaHei", 9),
            text=f"整腿模型净间隙下界 {report.min_margin_mm:.2f}mm", tags="hud_margin")


# ── 爪端-高杆距离曲线（评估/实机共用绘制） ────────────────

def _draw_curve(app, view):
    canvas = view["curve_canvas"]
    canvas.delete("all")
    width, height = canvas.winfo_width(), canvas.winfo_height()
    if width < 40 or height < 30:
        return
    mode = _mode(app, view)
    params = app.gait_params
    margin_l, margin_r, margin_t, margin_b = 34, 10, 12, 16
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b
    if mode == "plan":
        report = getattr(app, "_gait_last_report", None)
        if report is None or not report.samples:
            canvas.create_text(width/2, height/2, fill="#94a3b8",
                               text="评估曲线：先点【预览 / 校验】生成轨迹")
            return
        if not view["curve_plan"] or view.get("curve_plan_report") is not report:
            view["curve_plan"] = plan_tip_series(report, params)
            view["curve_plan_report"] = report
        data = [(t, tuple(v), 0) for t, v in view["curve_plan"]]
    else:
        data = list(view["curve_live"])
    if len(data) < 2:
        canvas.create_text(width/2, height/2, fill="#94a3b8",
                           text="实机曲线：开始步态后自动逐点记录（0.15s 采样）"
                           if mode == "live" else "曲线数据不足")
        return
    t_max = max(t for t, _v, _s in data)
    t_min = max(0.0, t_max - CURVE_WINDOW_S) if mode == "live" else 0.0
    if t_max - t_min < 1e-6:
        t_max = t_min + 1.0
    all_vals = [v for _t, vals, _s in data for v in vals]
    y_top = max(1.0, max(all_vals)) * 1.1
    y_bot = min(0.0, min(all_vals)) - abs(min(0.0, min(all_vals))) * 0.1 - 1.0

    def px(t):
        return margin_l + (t - t_min) / (t_max - t_min) * plot_w

    def py(v):
        return margin_t + (y_top - v) / (y_top - y_bot) * plot_h
    # 参考线：0=杆表面（灰实线）、δ 参考安全间隙（橙虚线）
    if y_bot < 0 <= y_top:
        canvas.create_line(margin_l, py(0), width - margin_r, py(0),
                           fill="#94a3b8", dash=(4, 3), tags="curve_zero")
        canvas.create_text(4, py(0) - 7, anchor="w", fill="#64748b",
                           font=("Microsoft YaHei", 7), text="0=杆表面")
    delta = params.geometry.safety_margin_mm
    if y_bot < delta < y_top:
        canvas.create_line(margin_l, py(delta), width - margin_r, py(delta),
                           fill="#f59e0b", dash=(2, 3), tags="curve_delta")
        canvas.create_text(4, py(delta) - 7, anchor="w", fill="#b45309",
                           font=("Microsoft YaHei", 7), text=f"δ={delta:g}")
    # 6 条爪曲线（分段：断连/换步 seg 变化处断线）
    for slot in range(6):
        color = CURVE_COLORS[slot // 3][slot % 3]
        for segment in _curve_segments(data, t_min):
            points = [v for t, vals, _s in segment for v in (px(t), py(vals[slot]))]
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=1,
                                   tags=f"curve_{'left' if slot < 3 else 'right'}_{slot % 3}")
    # 最小值包络加粗
    for segment in _curve_segments(data, t_min):
        points = [v for t, vals, _s in segment
                  for v in (px(t), py(min(vals)))]
        if len(points) >= 4:
            canvas.create_line(*points, fill="#111827", width=2, tags="curve_min")
    best = min(min(vals) for _t, vals, _s in data)
    label = "全程最低" if mode == "plan" else "窗口最低"
    canvas.create_text(width - margin_r - 4, margin_t + 2, anchor="ne",
                       fill="#111827", font=("Microsoft YaHei", 8, "bold"),
                       text=f"{label} {best:.2f}mm", tags="curve_best")
    canvas.create_text(margin_l + 2, height - 4, anchor="w", fill="#64748b",
                       font=("Microsoft YaHei", 7),
                       text=f"t={t_min:.0f}s", tags="curve_axis")
    canvas.create_text(width - margin_r, height - 4, anchor="e", fill="#64748b",
                       font=("Microsoft YaHei", 7),
                       text=f"t={t_max:.0f}s", tags="curve_axis")
    canvas.create_text(4, margin_t + 4, anchor="w", fill="#64748b",
                       font=("Microsoft YaHei", 7), text=f"{y_top:.0f}", tags="curve_axis")


def _curve_segments(data, t_min):
    chunks, current = [], []
    for point in data:
        if point[0] < t_min - 1e-9:
            continue
        if current and current[-1][2] != point[2]:
            chunks.append(current)
            current = []
        current.append(point)
    if current:
        chunks.append(current)
    return [c for c in chunks if len(c) >= 2]
