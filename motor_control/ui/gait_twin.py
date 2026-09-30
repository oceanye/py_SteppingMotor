"""Live pulse-estimated motor/foot view, separate from the planned path plot."""
import math
import tkinter as tk
from tkinter import ttk, messagebox

from motor_control.gait_twin import ROLES, TwinHistory
from motor_control.gait_map import map_pads, map_label, start_pair_reference
from motor_control.ui.common import bind_canvas_view, canvas_view_zoom, reset_canvas_view


def twin_pad_window():
    """Fixed 5 rows x 5 columns, same world coordinates throughout a session."""
    return map_pads()


def sync_twin_start_fields(app):
    view = app.gait_widgets.get("twin")
    if view:
        for side, name in zip(("left", "right"), app.gait_params.initial_supports):
            view[f"start_{side}"].set(map_label(name))


def clear_twin_history(app):
    """Display operation only: retain stance, calibration and pulse positions."""
    view = app.gait_widgets["twin"]
    view["history"].clear(view["snapshot"])
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
    panel = ttk.LabelFrame(parent, text="实时数字孪生 · 脉冲估算（无编码器）")
    panel.grid(row=0, column=0, sticky="ew", padx=6, pady=3)
    panel.columnconfigure(0, weight=1)
    canvas = tk.Canvas(panel, width=240, height=340, background="white",
                       highlightthickness=1, highlightbackground="#cbd5e1")
    canvas.grid(row=0, column=0, sticky="ew", padx=6, pady=5)
    status = ttk.Label(panel, text="等待电机绑定", wraplength=260, justify="left")
    status.grid(row=3, column=0, sticky="ew", padx=6, pady=3)
    coordinates = ttk.Label(panel, text="", wraplength=260, justify="left")
    coordinates.grid(row=4, column=0, sticky="ew", padx=6, pady=3)
    table = ttk.Frame(panel)
    table.grid(row=5, column=0, sticky="ew", padx=6, pady=3)
    table.columnconfigure((0, 1, 2), weight=1)
    for col, text in enumerate(("电机 / 轴", "估算坐标", "指令目标")):
        ttk.Label(table, text=text, foreground="#64748b").grid(row=0, column=col, sticky="w")
    rows = {}
    for row, role in enumerate(ROLES, 1):
        rows[role] = tuple(ttk.Label(table, text="—", font=("Consolas", 9)) for _ in range(3))
        for col, label in enumerate(rows[role]):
            label.grid(row=row, column=col, sticky="w", padx=(0, 5), pady=2)
    legend = ttk.Label(panel, text="蓝=左足  橙=右足  灰=断连冻结  绿点=低节点(落脚)  红点=高节点(红杆)\n"
                       "画面:左键点一下(蓝框)=选中,滚轮缩放;移出画面自动取消\n"
                       "中键拖动=平移 双击=复位视图\n"
                       "固定5×5地图，B=(0,0)，上为+Y、右为+X；行号自下向上。\n"
                       "XY 为中心距=1的估算；Z 为本步位移，非接触检测。\n"
                       "实线=已回报脉冲轨迹，圆点=完成步落脚；不绘制未来目标轨迹。",
                       wraplength=260, justify="left", foreground="#64748b")
    legend.grid(row=6, column=0, sticky="ew", padx=6, pady=3)
    view = {"canvas": canvas, "status": status, "rows": rows, "snapshot": {},
            "last_pose": None, "reference_key": None, "coordinates": coordinates,
            "history": TwinHistory(), "show_path": tk.BooleanVar(value=True),
            "select_side": tk.StringVar(value=""),
            "start_left": tk.StringVar(), "start_right": tk.StringVar()}
    app.gait_widgets["twin"] = view
    toolbar = ttk.Frame(panel)
    toolbar.grid(row=1, column=0, sticky="ew", padx=6)
    ttk.Checkbutton(toolbar, text="显示路径", variable=view["show_path"],
                    command=lambda: _draw(app, view)).grid(row=0, column=0)
    view["clear_button"] = ttk.Button(toolbar, text="清空路径", command=lambda: clear_twin_history(app))
    view["clear_button"].grid(row=0, column=1, padx=3)

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

    start = ttk.LabelFrame(panel, text="起步两格（仅设置，不移动电机）")
    start.grid(row=2, column=0, sticky="ew", padx=6, pady=3)
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
    # 视图状态与预览画面同一交互模式（左键选中+滚轮缩放+中键平移+双击复位）。
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
    _draw(app, view)


def _coordinate(value, unit, estimated=False):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return "—"
    return f"{'≈' if estimated else ''}{value:+.2f}{unit}"


def _draw(app, view):
    canvas = view["canvas"]
    canvas.delete("all")
    width, height = canvas.winfo_width(), canvas.winfo_height()
    if width < 20 or height < 20:
        return
    snapshot = view["snapshot"]
    live = snapshot.get("pose")
    pose = live or view["last_pose"]
    all_pads = twin_pad_window()
    route_pads = pose.get("route_pads", {}) if pose else {}
    xs, ys = zip(*all_pads.values())
    xmin, xmax = min(xs)-.6, max(xs)+.6
    ymin, ymax = min(ys)-.6, max(ys)+.6
    scale = min((width-26)/(xmax-xmin), (height-100)/(ymax-ymin))
    # 用户视图（滚轮缩放/中键平移）叠加在固定包围盒之上：画布中心为
    # 缩放原点；下方 ΔZ 条与角标是固定 HUD，不随缩放移动。
    twin_view = (getattr(app, "gait_widgets", None) or {}).get("twin_view") or {}
    zoom = float(twin_view.get("zoom", 1.0))
    pan_x = float(twin_view.get("pan_x", 0.0))
    pan_y = float(twin_view.get("pan_y", 0.0))

    def project(point):
        x, y = point
        base_x = width/2+(x-(xmin+xmax)/2)*scale
        base_y = 15+(ymax-y)*scale
        return (width/2+(base_x-width/2)*zoom+pan_x,
                height/2+(base_y-height/2)*zoom+pan_y)

    view["project"], view["map_scale"] = project, scale*zoom
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
    if pose is None:
        canvas.create_text(width/2, 14, text="起步位置待确认 / 标定，非实测位置",
                           width=width-12, fill="#b45309", tags="twin_placeholder")
        view["coordinates"].configure(text=f"已记录完成步：{history.completed_steps}；世界位置尚不可用")
        return

    # Show only reported/settled pulses, never the command's future target.
    left_xy, right_xy = (pose["feet"][s]["center"] for s in ("left", "right"))
    mid = tuple((a+b)/2 for a, b in zip(left_xy, right_xy))
    view["coordinates"].configure(text=(
        f"{'估算' if live else '历史冻结'} XY（中心距=1）· 已记录完成步 {history.completed_steps}\n"
        f"左 ({left_xy[0]:+.3f}, {left_xy[1]:+.3f})；右 ({right_xy[0]:+.3f}, {right_xy[1]:+.3f})\n"
        f"机构中心 ({mid[0]:+.3f}, {mid[1]:+.3f})"))
    centers = [project(pose["feet"][side]["center"]) for side in ("left", "right")]
    canvas.create_line(*centers[0], *centers[1], fill="#64748b",
                       width=2, tags="live_beam")
    for side, color in (("left", "#2563eb"), ("right", "#ea580c")):
        foot = pose["feet"][side]
        color = color if live else "#94a3b8"
        x, y = project(foot["center"])
        for i in range(3):
            theta = math.radians(foot["psi_deg"] + i*120)
            end = (foot["center"][0]+math.cos(theta)/math.sqrt(3),
                   foot["center"][1]+math.sin(theta)/math.sqrt(3))
            canvas.create_line(x, y, *project(end), fill=color, width=3,
                               tags=f"live_{side}")
        canvas.create_oval(x-5, y-5, x+5, y+5, outline=color,
                           fill=color, tags=f"{side}_center")
    max_z = max(1, *(abs(pose["feet"][side]["z_mm"]) for side in ("left", "right")))
    for side, fraction, label, color in (("left", 0.25, "左", "#2563eb"),
                                         ("right", 0.75, "右", "#ea580c")):
        x, y = width*fraction, height-45
        z = pose["feet"][side]["z_mm"]
        canvas.create_line(x-32, y, x+32, y, fill="#cbd5e1")
        canvas.create_line(x, y, x, y-z/max_z*28, width=5, fill=color if live else "#94a3b8",
                           tags=f"lift_{side}")
        canvas.create_text(x, height-17, text=f"{label} ΔZ≈{z:+.2f}mm", fill="#475569")
    canvas.create_text(width-8, 10, text="实时估算" if live else "历史估算（已冻结）",
                       anchor="ne", fill="#1565c0" if live else "#9a6700")
