"""Resizable inspection window sharing the main twin's read-only data stream."""
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace

from .common import bind_canvas_view, canvas_view_zoom, reset_canvas_view
from .gait_charts import build_twin_charts


def open_twin_window(app):
    from .gait_twin import draw_twin, _map_click
    main = app.gait_widgets["twin"]
    if main.get("large_window"):
        main["large_window"]["window"].deiconify()
        main["large_window"]["window"].lift()
        return
    window = tk.Toplevel(app.root)
    window.title("数字孪生大图 · 只读观察（无编码器估算）")
    window.geometry(f"{min(1500, window.winfo_screenwidth()-80)}x{min(820, window.winfo_screenheight()-80)}")
    window.minsize(640, 500)
    window.columnconfigure(0, weight=1)
    window.rowconfigure(1, weight=1)
    toolbar = ttk.Frame(window)
    toolbar.grid(row=0, column=0, sticky="ew", padx=8, pady=4)
    title = ttk.Label(toolbar, text="", wraplength=600, font=("Microsoft YaHei", 11, "bold"))
    title.grid(row=0, column=0, columnspan=6, sticky="w")
    charts = build_twin_charts(window, height=550)
    charts["chart_panes"].grid(row=1, column=0, sticky="nsew", padx=8)
    canvas, curve = charts["canvas"], charts["curve_canvas"]
    coordinates = ttk.Label(window, text="", justify="left")
    coordinates.grid(row=2, column=0, sticky="ew", padx=8)
    footer = ttk.Label(window, text="地图与曲线共用数据源；中间分隔条可拖动。滚轮缩放地图，中键平移；关闭大图不停止电机。",
                       foreground="#64748b", wraplength=600)
    footer.grid(row=4, column=0, sticky="w", padx=8, pady=4)
    detail = {**charts, "window": window, "title": title, "coordinates": coordinates,
              "view_state": {"zoom": 1.0, "pan_x": 0.0, "pan_y": 0.0, "active": False}}
    main["large_window"] = detail
    state = detail["view_state"]

    def redraw():
        refresh_twin_window(app, main)

    def zoom(delta):
        state["active"] = True
        canvas_view_zoom(state, SimpleNamespace(widget=canvas, delta=delta,
                         x=canvas.winfo_width()/2, y=canvas.winfo_height()/2), redraw)

    def close():
        main.pop("large_window", None)
        window.destroy()

    controls = []
    for label, command in (
            ("放大 +", lambda: zoom(240)), ("缩小 −", lambda: zoom(-240)),
            ("全图", lambda: (reset_canvas_view(state), redraw())),
            ("⛔ 实机中止", app._gait_abort_clicked), ("关闭大图", close)):
        controls.append(ttk.Button(toolbar, text=label, command=command))
    controls.append(ttk.Checkbutton(toolbar, text="显示路径", variable=main["show_path"],
                                   command=lambda: draw_twin(app)))
    detail["toolbar_controls"] = controls

    def resize_toolbar(event):
        available = max(200, event.width-16)
        title.configure(wraplength=available)
        footer.configure(wraplength=available)
        # High DPI/narrow windows must keep the stop and close buttons visible.
        cols = 6 if sum(w.winfo_reqwidth()+6 for w in controls) <= available else 3
        for index, control in enumerate(controls):
            control.grid(row=1+index//cols, column=index%cols, padx=3, pady=4)

    toolbar.bind("<Configure>", resize_toolbar)
    resize_toolbar(SimpleNamespace(width=640))
    window.protocol("WM_DELETE_WINDOW", close)
    window.bind("<Escape>", lambda _e: close())
    bind_canvas_view(canvas, state, redraw)
    canvas.bind("<Button-1>", lambda event: _map_click(app, event, detail), add="+")
    canvas.bind("<Configure>", lambda _e: redraw())
    curve.bind("<Configure>", lambda _e: redraw())
    redraw()


def refresh_twin_window(app, main):
    from .gait_twin import _draw_map
    detail = main.get("large_window")
    if not detail:
        return
    for key in ("snapshot", "last_pose", "history", "mode_var", "show_path", "start_left",
                "start_right", "select_side", "start_busy", "curve_live", "curve_plan",
                "curve_plan_report", "curve_plan_key", "simulation"):
        if key in main:
            detail[key] = main[key]
    detail["coordinates"].configure(wraplength=max(200, detail["window"].winfo_width()-24))
    _draw_map(app, detail)
