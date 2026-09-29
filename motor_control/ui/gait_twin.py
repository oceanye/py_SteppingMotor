"""Live pulse-estimated motor/foot view, separate from the planned path plot."""
import math
import tkinter as tk
from tkinter import ttk

from motor_control.gait_avoidance import lattice_coordinates
from motor_control.gait_twin import PAD_CENTERS, ROLES
from motor_control.ui.common import bind_canvas_view

# 2026-09-29 按用户要求：孪生画面固定画"ABC + 两圈邻座"的晶格窗口——
# 完成一步继续走下一 route 时视野不再随 route 跳变，支座/节点（红点
# 绿点及连线）始终在画面内；邻座名用 lattice 坐标格式，可被 pad_center
# 解析，与规划器同一几何。
_NEIGHBOR_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, -1), (-1, 1))


def twin_pad_window(extra_rings=2):
    """ABC 三支座向外扩 extra_rings 圈的固定视野支座 {name: (x, y)}。"""

    base = {name: lattice_coordinates(name) for name in PAD_CENTERS}
    seen = set(base.values())
    frontier = set(seen)
    for _ in range(extra_rings):
        ring = {(i + di, j + dj) for i, j in frontier
                for di, dj in _NEIGHBOR_STEPS} - seen
        seen |= ring
        frontier = ring
    pads = dict(PAD_CENTERS)
    for i, j in sorted(seen - set(base.values())):
        pads[f"邻座({i},{j})"] = (i + j / 2, math.sqrt(3) * j / 2)
    return pads


def build_twin_panel(app, parent):
    panel = ttk.LabelFrame(parent, text="实时数字孪生 · 脉冲估算（无编码器）")
    panel.grid(row=0, column=0, sticky="ew", padx=6, pady=3)
    panel.columnconfigure(0, weight=1)
    canvas = tk.Canvas(panel, width=240, height=290, background="white",
                       highlightthickness=1, highlightbackground="#cbd5e1")
    canvas.grid(row=0, column=0, sticky="ew", padx=6, pady=5)
    status = ttk.Label(panel, text="等待电机绑定", wraplength=260, justify="left")
    status.grid(row=1, column=0, sticky="ew", padx=6, pady=3)
    table = ttk.Frame(panel)
    table.grid(row=2, column=0, sticky="ew", padx=6, pady=3)
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
                       "XY 为中心距=1的示意；Z 为本步起点起算位移，非接触检测。",
                       wraplength=260, justify="left", foreground="#64748b")
    legend.grid(row=3, column=0, sticky="ew", padx=6, pady=3)
    view = {"canvas": canvas, "status": status, "rows": rows, "snapshot": {},
            "last_pose": None, "reference_key": None}
    app.gait_widgets["twin"] = view
    # 视图状态与预览画面同一交互模式（左键选中+滚轮缩放+中键平移+双击复位）。
    app.gait_widgets["twin_view"] = {"zoom": 1.0, "pan_x": 0.0, "pan_y": 0.0,
                                     "active": False}
    bind_canvas_view(canvas, app.gait_widgets["twin_view"],
                     lambda: _draw(app, view))

    def resized(event):
        for label in (status, legend):
            label.configure(wraplength=max(200, event.width - 8))
        _draw(app, view)
    canvas.bind("<Configure>", resized)


def refresh_twin_panel(app):
    view = app.gait_widgets.get("twin")
    if view is None:
        return
    provider = getattr(app, "_gait_twin_snapshot", None)
    snapshot = provider() if provider else {"axes": {}, "pose": None, "message": "等待电机绑定"}
    if snapshot.get("reference_key") != view["reference_key"]:
        view["last_pose"] = None
        view["reference_key"] = snapshot.get("reference_key")
    if snapshot.get("pose") is not None:
        view["last_pose"] = snapshot["pose"]
    view["snapshot"] = snapshot
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
    if pose is None:
        canvas.create_text(width/2, 24, text="世界姿态待建立 · 电机轴坐标仍可跟踪",
                           width=width-12, fill="#64748b", tags="twin_placeholder")
        for role, fraction in (("Mr1", 0.25), ("Mr2", 0.75)):
            axis = snapshot.get("axes", {}).get(role, {})
            x, y, radius = width*fraction, height/2, min(48, width/6)
            canvas.create_oval(x-radius, y-radius, x+radius, y+radius, outline="#94a3b8")
            angle = axis.get("position")
            if axis.get("binding_valid") and isinstance(angle, (int, float)) and math.isfinite(angle):
                theta = math.radians(angle)
                canvas.create_line(x, y, x+radius*math.cos(theta), y-radius*math.sin(theta),
                                   fill="#64748b", width=2, tags=f"axis_{role}")
            canvas.create_text(x, y+radius+18, text=role, fill="#475569")
        return

    # 固定晶格窗口（ABC+两圈邻座），不随 route 变：完成一步继续走下一
    # 步时画面不跳变；当前 route 的三个支座画重一点，邻座淡一些。
    all_pads = twin_pad_window()
    route_pads = pose.get("route_pads", {})
    xs, ys = zip(*all_pads.values())
    xmin, xmax = min(xs)-.4, max(xs)+.4
    ymin, ymax = min(ys)-.4, max(ys)+.4
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

    for name, center in all_pads.items():
        on_route = name in route_pads
        named = name in PAD_CENTERS
        nodes = [(center[0]+math.cos(math.radians(30+i*60))/math.sqrt(3),
                  center[1]+math.sin(math.radians(30+i*60))/math.sqrt(3)) for i in range(6)]
        canvas.create_polygon(*(v for node in nodes for v in project(node)),
                              fill="", outline="#94a3b8" if on_route else "#d1d5db",
                              width=1.5 if on_route else 1,
                              dash=() if on_route else (3, 3))
        x, y = project(center)
        if named:
            canvas.create_text(x, y+12, text=name, fill="#475569")
        dot = 2.5 if on_route else 1.8
        for i, node in enumerate(nodes):
            nx, ny = project(node)
            canvas.create_oval(nx-dot, ny-dot, nx+dot, ny+dot,
                               fill="#16a34a" if i % 2 == 0 else "#dc2626", outline="")

    # 2026-09-29 按用户要求：孪生画面只画当前实时姿态，不再显示虚线
    # 指令目标（轨迹性辅助线全部去掉），便于录视频观察机构本身。
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
