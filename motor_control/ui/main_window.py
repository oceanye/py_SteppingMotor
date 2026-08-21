import os
import tkinter as tk
from tkinter import ttk

from motor_control import (
    AXIS_LABEL,
    NUM_LOCAL_STEPPER_AXES,
    NUM_PICO_NODES,
    NUM_STEPPER_AXES,
    PICO_AXES_PER_NODE,
)
from motor_control.ui.common import PAD


def build_ui(app, log_dir, num_motor_axes):
    """Build the application shell while keeping behavior on the controller."""
    pad = PAD

    conn_frame = ttk.LabelFrame(app.root, text="串口连接")
    conn_frame.grid(row=0, column=0, sticky="ew", **pad)
    ttk.Label(conn_frame, text="串口:").grid(row=0, column=0, **pad)
    app.port_var = tk.StringVar()
    app.port_cb = ttk.Combobox(conn_frame, textvariable=app.port_var, width=12, state="readonly")
    app.port_cb.grid(row=0, column=1, **pad)
    ttk.Button(conn_frame, text="刷新", command=app.refresh_ports).grid(row=0, column=2, **pad)
    ttk.Label(conn_frame, text="波特率:").grid(row=0, column=3, **pad)
    app.baud_var = tk.StringVar(value="115200")
    ttk.Combobox(conn_frame, textvariable=app.baud_var, values=["9600", "115200"],
                 width=8, state="readonly").grid(row=0, column=4, **pad)
    app.conn_btn = ttk.Button(conn_frame, text="连接", command=app.toggle_connect)
    app.conn_btn.grid(row=0, column=5, **pad)
    app.conn_status = ttk.Label(conn_frame, text="● 未连接", foreground="red")
    app.conn_status.grid(row=0, column=6, **pad)

    ttk.Label(conn_frame, text="生效模式:").grid(row=0, column=7, **pad)
    app.mode_label = ttk.Label(conn_frame, textvariable=app.fw_mode_var,
                               width=8, font=("Consolas", 11, "bold"),
                               foreground="gray")
    app.mode_label.grid(row=0, column=8, **pad)

    ttk.Label(conn_frame, text="模式选择:").grid(row=1, column=0, sticky="e", **pad)
    mode_inner = ttk.Frame(conn_frame)
    mode_inner.grid(row=1, column=1, columnspan=8, sticky="w", **pad)
    ttk.Radiobutton(mode_inner, text="Auto (固件自报)", variable=app.mode_select_var,
                    value="Auto", command=app._on_mode_select_change).pack(side="left", padx=6)
    ttk.Radiobutton(mode_inner, text="强制 FOC", variable=app.mode_select_var,
                    value="FOC", command=app._on_mode_select_change).pack(side="left", padx=6)
    ttk.Radiobutton(mode_inner, text="强制 GEAR", variable=app.mode_select_var,
                    value="GEAR", command=app._on_mode_select_change).pack(side="left", padx=6)

    ttk.Label(conn_frame, text="网页服务:").grid(row=2, column=0, sticky="e", **pad)
    app.web_status_label = ttk.Label(
        conn_frame, textvariable=app.web_status_var, foreground="gray")
    app.web_status_label.grid(row=2, column=1, columnspan=2, sticky="w", **pad)
    ttk.Label(conn_frame, text="手机访问地址:").grid(row=2, column=3, sticky="e", **pad)
    ttk.Entry(conn_frame, textvariable=app.web_address_var, width=28,
              state="readonly").grid(row=2, column=4, columnspan=3, sticky="ew", **pad)
    ttk.Label(conn_frame, text="端口:").grid(row=2, column=7, sticky="e", **pad)
    ttk.Label(conn_frame, textvariable=app.web_port_var,
              font=("Consolas", 11, "bold")).grid(row=2, column=8, sticky="w", **pad)

    ttk.Label(conn_frame, text="手机完整网址:").grid(row=3, column=0, sticky="e", **pad)
    app.web_url_entry = ttk.Entry(conn_frame, textvariable=app.web_url_var,
                                  width=69, state="readonly")
    app.web_url_entry.grid(row=3, column=1, columnspan=6, sticky="ew", **pad)
    ttk.Button(conn_frame, text="复制手机网址", command=app._copy_web_url).grid(
        row=3, column=7, **pad)
    ttk.Button(conn_frame, text="本机打开", command=app._open_web_url).grid(
        row=3, column=8, **pad)

    ttk.Label(
        conn_frame,
        text="流程：① 手机与台式机连接同一局域网  ② GUI 连接 ESP32 串口  "
             "③ 手机打开上方固定网址  ④ 页面显示“串口已连接”后控制电机",
        foreground="#555",
    ).grid(row=4, column=0, columnspan=9, sticky="w", padx=6, pady=(1, 2))

    app.notebook = ttk.Notebook(app.root)
    app.notebook.grid(row=1, column=0, sticky="nsew", padx=6, pady=3)
    app.tab_index_step = [None] * NUM_STEPPER_AXES
    app.tab_index_foc = [None] * num_motor_axes
    app.tab_index_gear = [None] * num_motor_axes
    stepper_tab = ttk.Frame(app.notebook)
    stepper_index = app.notebook.index("end")
    app.notebook.add(stepper_tab, text="🔩 步进轴 (30)")
    app.tab_index_step = [stepper_index] * NUM_STEPPER_AXES
    group_book = ttk.Notebook(stepper_tab)
    group_book.pack(fill="both", expand=True, padx=3, pady=2)
    groups = [("ESP32 本地", list(range(NUM_LOCAL_STEPPER_AXES)))]
    for node in range(1, NUM_PICO_NODES + 1):
        start = NUM_LOCAL_STEPPER_AXES + (node - 1) * PICO_AXES_PER_NODE
        groups.append((f"Pico {node}", list(range(start, start + PICO_AXES_PER_NODE))))
    app.stepper_axis_tabs = [None] * NUM_STEPPER_AXES
    for group_label, axes in groups:
        group = ttk.Frame(group_book)
        group_book.add(group, text=group_label)
        axis_book = ttk.Notebook(group)
        axis_book.pack(fill="both", expand=True)
        for axis in axes:
            tab = ttk.Frame(axis_book)
            axis_book.add(tab, text=f"轴 {AXIS_LABEL[axis]}")
            app.stepper_axis_tabs[axis] = tab
            app._build_stepper_tab(tab, axis)
    for axis in range(num_motor_axes):
        tab = ttk.Frame(app.notebook)
        app.tab_index_foc[axis] = app.notebook.index("end")
        app.notebook.add(tab, text=f"🧲 FOC {AXIS_LABEL[axis]}")
        app._build_foc_tab(tab, axis)
    for axis in range(num_motor_axes):
        tab = ttk.Frame(app.notebook)
        app.tab_index_gear[axis] = app.notebook.index("end")
        app.notebook.add(tab, text=f"⚙ 减速 {AXIS_LABEL[axis]}")
        app._build_gear_tab(tab, axis)

    track_tab = ttk.Frame(app.notebook)
    app.tab_index_track = app.notebook.index("end")
    app.notebook.add(track_tab, text="↔ 轨道 D")
    app._build_track_tab(track_tab)

    log_frame = ttk.LabelFrame(app.root,
                               text=f"日志（同时写到 {os.path.basename(log_dir)}/gui_<日期>.log）")
    log_frame.grid(row=2, column=0, sticky="ew", **pad)
    app.log_text = tk.Text(log_frame, height=6, width=96, state="disabled", font=("Consolas", 9))
    app.log_text.pack(side="left", fill="both", padx=4, pady=3)
    app.root.bind("<MouseWheel>", app._on_root_mousewheel, add="+")
    scroll = ttk.Scrollbar(log_frame, command=app.log_text.yview)
    scroll.pack(side="right", fill="y")
    app.log_text.config(yscrollcommand=scroll.set)
    app.log_text.tag_config("err", foreground="#c00")
    app.log_text.tag_config("warn", foreground="#e65100")
    app.log_text.tag_config("axisL", foreground="#1565c0")
    app.log_text.tag_config("axisR", foreground="#7b1fa2")
    app.log_text.tag_config("ok", foreground="#2e7d32")
    app.log_text.tag_config("rx", foreground="#666")

    log_btn_frame = ttk.Frame(app.root)
    log_btn_frame.grid(row=3, column=0, sticky="e", padx=6, pady=2)
    ttk.Button(log_btn_frame, text="清空", command=app.clear_log).pack(side="left", padx=2)
    ttk.Button(log_btn_frame, text="打开日志目录", command=app._open_log_dir).pack(side="left", padx=2)
    app.raw_log_var = tk.BooleanVar(value=False)
    ttk.Checkbutton(log_btn_frame, text="串口原始流(新文件)", variable=app.raw_log_var,
                    command=app._toggle_raw_log).pack(side="left", padx=8)

    app._raw_log_fh = None
    app.root.rowconfigure(1, weight=1)
    app.root.columnconfigure(0, weight=1)
    app.root.update_idletasks()
    req_w = app.root.winfo_reqwidth()
    req_h = app.root.winfo_reqheight()
    screen_w = app.root.winfo_screenwidth()
    screen_h = app.root.winfo_screenheight()
    win_w = min(req_w, screen_w - 40)
    win_h = min(req_h, screen_h - 80)
    app.root.geometry(f"{int(win_w)}x{int(win_h)}")
    app.root.minsize(720, 480)

    app.refresh_ports()
