import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import queue
import json
import os

# ============== 轴配置 ==============
NUM_AXES = 2
AXIS_LABEL = ["L", "R"]   # 左腿 / 右腿

# ============== 步进标定 ==============
PULSES_PER_MM = 50
DIR_OUTWARD = 0
DIR_INWARD  = 1
CONTINUOUS_BURST_MM = 0.2

# ============== FOC ==============
FOC_STATE_NAMES = {"0": "失能", "1": "对齐中", "2": "运行", "3": "故障"}
GEAR_STATE_NAMES = {"0": "失能", "1": "—", "2": "运行", "3": "故障"}  # GEAR 没有"对齐"阶段
FOC_POLL_INTERVAL_S = 0.1

# ============== 模式 ==============
MODE_FOC  = "FOC"
MODE_GEAR = "GEAR"

CALIB_FILE     = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".stepper_calib.json")
FOC_TUNE_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".foc_tune.json")
GEAR_TUNE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".gear_tune.json")
LOG_DIR        = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
os.makedirs(LOG_DIR, exist_ok=True)


def _empty_axis_dict():
    return {i: None for i in range(NUM_AXES)}


class StepperGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("双轴步进 + FOC/GEAR 控制器")
        self.root.resizable(False, False)

        # ── 共享：串口 ──
        self.ser = None
        self.serial_lock = threading.Lock()
        self._resp_queue = queue.Queue()
        self._reader_running = False
        self.foc_poll_running = False

        # ── 每轴状态（list 按 axis 索引）──
        self.running            = [False] * NUM_AXES          # 连续运动 flag
        self.position_mm        = [0.0]   * NUM_AXES
        self.travel_min_mm      = [None]  * NUM_AXES
        self.travel_max_mm      = [None]  * NUM_AXES
        self.stepper_in_progress= [False] * NUM_AXES
        self._pending_step      = [None]  * NUM_AXES
        self._foc_enabled_ui    = [False] * NUM_AXES
        self.foc_trace_buf      = [[] for _ in range(NUM_AXES)]
        # goto watcher 代数：每次新 goto +1，旧 watcher 检测到 gen 变了就退出
        self._goto_watcher_gen  = [0] * NUM_AXES

        # ── 每轴 Tk 变量 ──
        self.v_dist   = [tk.DoubleVar(value=10.0) for _ in range(NUM_AXES)]
        self.v_dir    = [tk.IntVar   (value=DIR_OUTWARD) for _ in range(NUM_AXES)]
        self.v_delay  = [tk.DoubleVar(value=20.0) for _ in range(NUM_AXES)]
        self.v_goto   = [tk.DoubleVar(value=0.0)  for _ in range(NUM_AXES)]
        self.v_foctgt = [tk.DoubleVar(value=0.0)  for _ in range(NUM_AXES)]
        self.v_focstate   = [tk.StringVar(value="未连接") for _ in range(NUM_AXES)]
        self.v_focfault   = [tk.StringVar(value="--")    for _ in range(NUM_AXES)]
        self.v_foccur     = [tk.StringVar(value="--")    for _ in range(NUM_AXES)]
        self.v_focvlimit  = [tk.DoubleVar(value=10.0)    for _ in range(NUM_AXES)]
        self.v_focpangle  = [tk.DoubleVar(value=25.0)    for _ in range(NUM_AXES)]
        self.v_focvp      = [tk.DoubleVar(value=0.2)     for _ in range(NUM_AXES)]
        self.v_focpp      = [tk.IntVar   (value=7)       for _ in range(NUM_AXES)]

        # ── GEAR 模式调参变量（和 FOC 共享 v_foctgt / v_focstate / v_foccur / v_focfault）──
        self.v_gearpwm = [tk.DoubleVar(value=100.0)  for _ in range(NUM_AXES)]   # PWM duty cap %（0-100）
        self.v_gearkp  = [tk.DoubleVar(value=1.0)    for _ in range(NUM_AXES)]
        self.v_gearki  = [tk.DoubleVar(value=0.0)    for _ in range(NUM_AXES)]
        self.v_gearkd  = [tk.DoubleVar(value=0.05)   for _ in range(NUM_AXES)]
        self.v_geargr  = [tk.DoubleVar(value=1000.0) for _ in range(NUM_AXES)]   # 齿轮比（NVS）

        # ── 模式：
        #    mode_select_var: 用户选择 (Auto/FOC/GEAR)；Auto 时从固件 MODE 命令读
        #    fw_mode_var:      当前实际生效的模式显示 (FOC/GEAR/?)
        self.mode_select_var = tk.StringVar(value="Auto")
        self.fw_mode_var = tk.StringVar(value="?")

        # ── 每轴 widget refs（dict-per-axis）──
        self.sw = [dict() for _ in range(NUM_AXES)]  # stepper widgets
        self.fw = [dict() for _ in range(NUM_AXES)]  # FOC widgets
        self.gw = [dict() for _ in range(NUM_AXES)]  # GEAR widgets

        self._build_ui()
        self._load_calib()
        self._load_foc_tune()
        self._load_gear_tune()

    # ═════════════ 顶层 UI ═════════════
    def _build_ui(self):
        pad = dict(padx=10, pady=5)

        conn_frame = ttk.LabelFrame(self.root, text="串口连接")
        conn_frame.grid(row=0, column=0, sticky="ew", **pad)
        ttk.Label(conn_frame, text="串口:").grid(row=0, column=0, **pad)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_cb.grid(row=0, column=1, **pad)
        ttk.Button(conn_frame, text="刷新", command=self.refresh_ports).grid(row=0, column=2, **pad)
        ttk.Label(conn_frame, text="波特率:").grid(row=0, column=3, **pad)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["9600", "115200"],
                     width=8, state="readonly").grid(row=0, column=4, **pad)
        self.conn_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connect)
        self.conn_btn.grid(row=0, column=5, **pad)
        self.conn_status = ttk.Label(conn_frame, text="● 未连接", foreground="red")
        self.conn_status.grid(row=0, column=6, **pad)

        # 实际生效的固件模式显示
        ttk.Label(conn_frame, text="生效模式:").grid(row=0, column=7, **pad)
        self.mode_label = ttk.Label(conn_frame, textvariable=self.fw_mode_var,
                                    width=8, font=("Consolas", 11, "bold"),
                                    foreground="gray")
        self.mode_label.grid(row=0, column=8, **pad)

        # 用户模式选择（Auto/FOC/GEAR）—— 第二行
        ttk.Label(conn_frame, text="模式选择:").grid(row=1, column=0, sticky="e", **pad)
        mode_inner = ttk.Frame(conn_frame)
        mode_inner.grid(row=1, column=1, columnspan=8, sticky="w", **pad)
        ttk.Radiobutton(mode_inner, text="Auto (固件自报)", variable=self.mode_select_var,
                        value="Auto", command=self._on_mode_select_change).pack(side="left", padx=6)
        ttk.Radiobutton(mode_inner, text="强制 FOC", variable=self.mode_select_var,
                        value="FOC", command=self._on_mode_select_change).pack(side="left", padx=6)
        ttk.Radiobutton(mode_inner, text="强制 GEAR", variable=self.mode_select_var,
                        value="GEAR", command=self._on_mode_select_change).pack(side="left", padx=6)

        # Notebook: 步进 ×2 + FOC ×2 + GEAR ×2 = 6 tabs。
        # FOC/GEAR 互斥：连接后根据固件 MODE 灰掉另一组。
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        self.tab_index_step = [None] * NUM_AXES
        self.tab_index_foc  = [None] * NUM_AXES
        self.tab_index_gear = [None] * NUM_AXES
        for axis in range(NUM_AXES):
            tab = ttk.Frame(self.notebook)
            self.tab_index_step[axis] = self.notebook.index("end")
            self.notebook.add(tab, text=f"🔩 步进 {AXIS_LABEL[axis]}")
            self._build_stepper_tab(tab, axis)
        for axis in range(NUM_AXES):
            tab = ttk.Frame(self.notebook)
            self.tab_index_foc[axis] = self.notebook.index("end")
            self.notebook.add(tab, text=f"🧲 FOC {AXIS_LABEL[axis]}")
            self._build_foc_tab(tab, axis)
        for axis in range(NUM_AXES):
            tab = ttk.Frame(self.notebook)
            self.tab_index_gear[axis] = self.notebook.index("end")
            self.notebook.add(tab, text=f"⚙ 减速 {AXIS_LABEL[axis]}")
            self._build_gear_tab(tab, axis)

        log_frame = ttk.LabelFrame(self.root,
                                   text=f"日志（同时写到 {os.path.basename(LOG_DIR)}/gui_<日期>.log）")
        log_frame.grid(row=2, column=0, sticky="ew", **pad)
        self.log_text = tk.Text(log_frame, height=10, width=96, state="disabled", font=("Consolas", 9))
        self.log_text.pack(side="left", fill="both", padx=5, pady=5)
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)
        # 颜色标注
        self.log_text.tag_config("err",   foreground="#c00")
        self.log_text.tag_config("warn",  foreground="#e65100")
        self.log_text.tag_config("axisL", foreground="#1565c0")
        self.log_text.tag_config("axisR", foreground="#7b1fa2")
        self.log_text.tag_config("ok",    foreground="#2e7d32")
        self.log_text.tag_config("rx",    foreground="#666")

        log_btn_frame = ttk.Frame(self.root)
        log_btn_frame.grid(row=3, column=0, sticky="e", padx=10, pady=2)
        ttk.Button(log_btn_frame, text="清空", command=self.clear_log).pack(side="left", padx=2)
        ttk.Button(log_btn_frame, text="打开日志目录", command=self._open_log_dir).pack(side="left", padx=2)
        self.raw_log_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(log_btn_frame, text="串口原始流(新文件)", variable=self.raw_log_var,
                        command=self._toggle_raw_log).pack(side="left", padx=8)

        self._raw_log_fh = None  # 串口原始日志文件句柄（开关控制）

        self.refresh_ports()

    # ═════════════ 步进 Tab（参数化）═════════════
    def _build_stepper_tab(self, parent, axis):
        pad = dict(padx=10, pady=5)
        sw = self.sw[axis]

        pf = ttk.LabelFrame(parent, text=f"运动参数 — 轴 {AXIS_LABEL[axis]}")
        pf.grid(row=0, column=0, sticky="nsew", **pad)
        ttk.Label(pf, text="距离 (mm):").grid(row=0, column=0, sticky="w", **pad)
        ttk.Spinbox(pf, from_=0.2, to=500.0, increment=1.0,
                    textvariable=self.v_dist[axis], width=10, format="%.1f").grid(row=0, column=1, **pad)
        ttk.Label(pf, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
        qbf = ttk.Frame(pf); qbf.grid(row=1, column=1, sticky="w", pady=5)
        for label, mm in [("1mm", 1), ("10mm", 10), ("50mm", 50), ("100mm", 100)]:
            ttk.Button(qbf, text=label, width=6,
                       command=lambda d=mm, a=axis: self.v_dist[a].set(d)).pack(side="left", padx=2)
        ttk.Label(pf, text="方向:").grid(row=2, column=0, sticky="w", **pad)
        df = ttk.Frame(pf); df.grid(row=2, column=1, sticky="w")
        ttk.Radiobutton(df, text="向外 ▶", variable=self.v_dir[axis], value=DIR_OUTWARD).pack(side="left", padx=4)
        ttk.Radiobutton(df, text="◀ 向内", variable=self.v_dir[axis], value=DIR_INWARD).pack(side="left", padx=4)
        ttk.Label(pf, text="速度:").grid(row=3, column=0, sticky="w", **pad)
        sf = ttk.Frame(pf); sf.grid(row=3, column=1, sticky="w", pady=5)
        sw['speed_slider'] = ttk.Scale(sf, from_=0.1, to=50.0, orient="horizontal",
                                       variable=self.v_delay[axis], length=160)
        sw['speed_slider'].pack(side="left")
        sw['delay_label'] = ttk.Label(sf, text="", width=24)
        sw['delay_label'].pack(side="left", padx=6)
        self.v_delay[axis].trace_add("write", lambda *_, a=axis: self._update_speed_label(a))
        self._update_speed_label(axis)

        cf = ttk.LabelFrame(parent, text="控制")
        cf.grid(row=0, column=1, sticky="nsew", **pad)
        sw['move_btn'] = ttk.Button(cf, text="执行运动",
                                    command=lambda a=axis: self.send_move(a), state="disabled")
        sw['move_btn'].grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=8)
        sw['jog_out_btn'] = ttk.Button(cf, text="向外 1mm ▶",
                                       command=lambda a=axis: self._quick_move(a, 1.0, DIR_OUTWARD),
                                       state="disabled")
        sw['jog_out_btn'].grid(row=1, column=0, **pad)
        sw['jog_in_btn'] = ttk.Button(cf, text="◀ 向内 1mm",
                                      command=lambda a=axis: self._quick_move(a, 1.0, DIR_INWARD),
                                      state="disabled")
        sw['jog_in_btn'].grid(row=1, column=1, **pad)
        sw['cont_out_btn'] = ttk.Button(cf, text="向外 (按住) ▶▶", state="disabled")
        sw['cont_out_btn'].grid(row=2, column=0, **pad)
        sw['cont_out_btn'].bind("<ButtonPress-1>",   lambda e, a=axis: self._press_continuous(a, DIR_OUTWARD))
        sw['cont_out_btn'].bind("<ButtonRelease-1>", lambda e, a=axis: self._release_continuous(a))
        sw['cont_out_btn'].bind("<Leave>",           lambda e, a=axis: self._release_continuous(a))
        sw['cont_in_btn'] = ttk.Button(cf, text="◀◀ 向内 (按住)", state="disabled")
        sw['cont_in_btn'].grid(row=2, column=1, **pad)
        sw['cont_in_btn'].bind("<ButtonPress-1>",   lambda e, a=axis: self._press_continuous(a, DIR_INWARD))
        sw['cont_in_btn'].bind("<ButtonRelease-1>", lambda e, a=axis: self._release_continuous(a))
        sw['cont_in_btn'].bind("<Leave>",           lambda e, a=axis: self._release_continuous(a))
        sw['stop_btn'] = ttk.Button(cf, text="■ 紧急停止",
                                    command=lambda a=axis: self.stop_continuous(a), state="disabled")
        sw['stop_btn'].grid(row=3, column=0, columnspan=2, **pad, ipadx=10)

        rf = ttk.LabelFrame(parent, text="位置与原点（软件跟踪）")
        rf.grid(row=1, column=0, columnspan=2, sticky="ew", **pad)
        ttk.Label(rf, text="当前位置:").grid(row=0, column=0, sticky="w", **pad)
        sw['pos_label'] = ttk.Label(rf, text="0.0 mm", font=("Consolas", 14, "bold"), foreground="blue")
        sw['pos_label'].grid(row=0, column=1, sticky="w", **pad)
        sw['set_home_btn'] = ttk.Button(rf, text="⌂ 设为原点 (0 mm)",
                                        command=lambda a=axis: self.set_home(a), state="disabled")
        sw['set_home_btn'].grid(row=0, column=2, **pad)
        sw['go_home_btn'] = ttk.Button(rf, text="⟲ 回到原点",
                                       command=lambda a=axis: self.go_home(a), state="disabled")
        sw['go_home_btn'].grid(row=0, column=3, **pad)
        ttk.Label(rf, text="前往位置 (mm):").grid(row=1, column=0, sticky="w", **pad)
        ttk.Spinbox(rf, from_=-1000.0, to=1000.0, increment=1.0,
                    textvariable=self.v_goto[axis], width=10, format="%.1f").grid(row=1, column=1, **pad)
        sw['goto_btn'] = ttk.Button(rf, text="前往",
                                    command=lambda a=axis: self.goto_target_position(a), state="disabled")
        sw['goto_btn'].grid(row=1, column=2, **pad)
        sw['calib_btn'] = ttk.Button(rf, text="把当前位置校准为此值",
                                     command=lambda a=axis: self.calibrate_position(a), state="disabled")
        sw['calib_btn'].grid(row=1, column=3, **pad)

        ttk.Label(rf, text="行程范围:").grid(row=2, column=0, sticky="w", **pad)
        rgf = ttk.Frame(rf); rgf.grid(row=2, column=1, columnspan=3, sticky="w", **pad)
        sw['range_label'] = ttk.Label(rgf, text="未校准", foreground="gray", font=("Consolas", 10))
        sw['range_label'].pack(side="left")
        sw['set_min_btn'] = ttk.Button(rgf, text="⊖ 标记当前为最小",
                                       command=lambda a=axis: self._mark_min(a), state="disabled")
        sw['set_min_btn'].pack(side="left", padx=10)
        sw['set_max_btn'] = ttk.Button(rgf, text="⊕ 标记当前为最大",
                                       command=lambda a=axis: self._mark_max(a), state="disabled")
        sw['set_max_btn'].pack(side="left", padx=2)
        sw['clear_range_btn'] = ttk.Button(rgf, text="清除", width=6,
                                           command=lambda a=axis: self._clear_range(a), state="disabled")
        sw['clear_range_btn'].pack(side="left", padx=10)

    # ═════════════ FOC Tab（参数化）═════════════
    def _build_foc_tab(self, parent, axis):
        pad = dict(padx=10, pady=5)
        fw = self.fw[axis]

        sf = ttk.LabelFrame(parent, text=f"状态 — 轴 {AXIS_LABEL[axis]} (100ms 轮询)")
        sf.grid(row=0, column=0, sticky="ew", **pad)
        ttk.Label(sf, text="状态:").grid(row=0, column=0, sticky="w", **pad)
        ttk.Label(sf, textvariable=self.v_focstate[axis], width=10,
                  font=("Consolas", 11, "bold"), foreground="gray").grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(sf, text="故障:").grid(row=0, column=2, sticky="w", **pad)
        fw['fault_label'] = ttk.Label(sf, textvariable=self.v_focfault[axis], width=6,
                                      font=("Consolas", 11, "bold"), foreground="gray")
        fw['fault_label'].grid(row=0, column=3, sticky="w", **pad)
        ttk.Label(sf, text="当前角度:").grid(row=1, column=0, sticky="w", **pad)
        ttk.Label(sf, textvariable=self.v_foccur[axis], width=16,
                  font=("Consolas", 14, "bold"), foreground="blue").grid(row=1, column=1, columnspan=3, sticky="w", **pad)

        tf = ttk.LabelFrame(parent, text="目标控制")
        tf.grid(row=1, column=0, sticky="ew", **pad)
        ttk.Label(tf, text="目标角度 (°):").grid(row=0, column=0, sticky="w", **pad)
        ttk.Spinbox(tf, from_=-3600.0, to=3600.0, increment=1.0,
                    textvariable=self.v_foctgt[axis], width=10, format="%.1f").grid(row=0, column=1, **pad)
        fw['goto_btn'] = ttk.Button(tf, text="前往",
                                    command=lambda a=axis: self._foc_goto(a), state="disabled")
        fw['goto_btn'].grid(row=0, column=2, **pad)
        ttk.Label(tf, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
        qf = ttk.Frame(tf); qf.grid(row=1, column=1, columnspan=2, sticky="w", pady=5)
        fw['quick_btns'] = []
        for deg in (0, 45, 90, 180, 270):
            b = ttk.Button(qf, text=f"{deg}°", width=5,
                           command=lambda d=deg, a=axis: self._foc_quick(a, float(d)), state="disabled")
            b.pack(side="left", padx=2)
            fw['quick_btns'].append(b)
        ttk.Label(tf, text="增量:").grid(row=2, column=0, sticky="w", **pad)
        incf = ttk.Frame(tf); incf.grid(row=2, column=1, columnspan=2, sticky="w", pady=5)
        fw['inc_btns'] = []
        for delta in (-10, -1, +1, +10):
            b = ttk.Button(incf, text=f"{delta:+d}°", width=5,
                           command=lambda d=float(delta), a=axis: self._foc_increment(a, d), state="disabled")
            b.pack(side="left", padx=2)
            fw['inc_btns'].append(b)

        hf = ttk.LabelFrame(parent, text="原点")
        hf.grid(row=2, column=0, sticky="ew", **pad)
        fw['home_btn'] = ttk.Button(hf, text="⌂ 把当前位置设为 0°",
                                    command=lambda a=axis: self._foc_home(a), state="disabled")
        fw['home_btn'].grid(row=0, column=0, **pad)

        ef = ttk.LabelFrame(parent, text="使能 / 调试")
        ef.grid(row=3, column=0, sticky="ew", **pad)
        fw['enable_btn'] = ttk.Button(ef, text="▶ 使能 FOC",
                                      command=lambda a=axis: self._foc_toggle_enable(a), state="disabled")
        fw['enable_btn'].grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=6)

        ttk.Label(ef, text="电压限幅:").grid(row=1, column=0, sticky="w", **pad)
        vf = ttk.Frame(ef); vf.grid(row=1, column=1, sticky="w", pady=5)
        fw['vlimit_slider'] = ttk.Scale(vf, from_=0.5, to=24.0, orient="horizontal",
                                        variable=self.v_focvlimit[axis], length=180,
                                        command=lambda v, a=axis: self._foc_on_vlimit(a, v), state="disabled")
        fw['vlimit_slider'].pack(side="left")
        fw['vlimit_slider'].bind("<ButtonRelease-1>", lambda e: self._save_foc_tune())
        fw['vlimit_label'] = ttk.Label(vf, text="10.0 V (扭矩)", width=22)
        fw['vlimit_label'].pack(side="left", padx=6)

        ttk.Label(ef, text="位置环 P:").grid(row=2, column=0, sticky="w", **pad)
        pgf = ttk.Frame(ef); pgf.grid(row=2, column=1, sticky="w", pady=5)
        fw['pangle_slider'] = ttk.Scale(pgf, from_=1.0, to=50.0, orient="horizontal",
                                        variable=self.v_focpangle[axis], length=180,
                                        command=lambda v, a=axis: self._foc_on_pangle(a, v), state="disabled")
        fw['pangle_slider'].pack(side="left")
        fw['pangle_slider'].bind("<ButtonRelease-1>", lambda e: self._save_foc_tune())
        fw['pangle_label'] = ttk.Label(pgf, text="25.0 (刚度)", width=14)
        fw['pangle_label'].pack(side="left", padx=6)

        ttk.Label(ef, text="速度环 P:").grid(row=3, column=0, sticky="w", **pad)
        vpf = ttk.Frame(ef); vpf.grid(row=3, column=1, sticky="w", pady=5)
        fw['vp_slider'] = ttk.Scale(vpf, from_=0.05, to=1.0, orient="horizontal",
                                    variable=self.v_focvp[axis], length=180,
                                    command=lambda v, a=axis: self._foc_on_vp(a, v), state="disabled")
        fw['vp_slider'].pack(side="left")
        fw['vp_slider'].bind("<ButtonRelease-1>", lambda e: self._save_foc_tune())
        fw['vp_label'] = ttk.Label(vpf, text="0.20 (阻尼)", width=14)
        fw['vp_label'].pack(side="left", padx=6)

        ttk.Label(ef, text="极对数:").grid(row=4, column=0, sticky="w", **pad)
        ppf = ttk.Frame(ef); ppf.grid(row=4, column=1, sticky="w", pady=5)
        ttk.Spinbox(ppf, from_=1, to=50, textvariable=self.v_focpp[axis], width=6).pack(side="left")
        fw['pp_save_btn'] = ttk.Button(ppf, text="保存到 NVS（重启生效）",
                                       command=lambda a=axis: self._foc_save_pp(a), state="disabled")
        fw['pp_save_btn'].pack(side="left", padx=6)
        fw['clear_btn'] = ttk.Button(ef, text="🧹 清除故障",
                                     command=lambda a=axis: self._foc_clear_fault(a), state="disabled")
        fw['clear_btn'].grid(row=5, column=0, columnspan=2, **pad, ipadx=10)
        fw['autotune_btn'] = ttk.Button(ef, text="🤖 自动优化 PID",
                                        command=lambda a=axis: self._foc_autotune(a), state="disabled")
        fw['autotune_btn'].grid(row=6, column=0, columnspan=2, **pad, ipadx=10)

        scf = ttk.LabelFrame(parent, text="响应曲线 (10s · 蓝=目标 红=实测)")
        scf.grid(row=0, column=1, rowspan=4, sticky="nsew", **pad)
        fw['scope'] = tk.Canvas(scf, width=420, height=420, bg="white",
                                highlightthickness=1, highlightbackground="#999")
        fw['scope'].pack(padx=5, pady=5)

        # 聚合：状态门控用
        fw['motion_btns'] = [fw['goto_btn']] + fw['quick_btns'] + fw['inc_btns']
        fw['cfg_btns']    = [fw['home_btn'], fw['vlimit_slider'], fw['pangle_slider'],
                             fw['vp_slider'], fw['pp_save_btn'], fw['autotune_btn']]

    # ═════════════ 共享辅助 ═════════════
    def _update_speed_label(self, axis):
        d = max(0.1, self.v_delay[axis].get())
        mm_s = 1000.0 / (PULSES_PER_MM * d)
        self.sw[axis]['delay_label'].config(text=f"{d:.1f} ms ≈ {mm_s:.0f} mm/s")

    def _update_pos_label(self, axis):
        self.sw[axis]['pos_label'].config(text=f"{self.position_mm[axis]:.1f} mm")

    def _enable_stepper_buttons(self, axis, state):
        sw = self.sw[axis]
        keys = ['move_btn', 'jog_out_btn', 'jog_in_btn', 'cont_out_btn', 'cont_in_btn',
                'stop_btn', 'set_home_btn', 'go_home_btn', 'goto_btn', 'calib_btn',
                'set_min_btn', 'set_max_btn', 'clear_range_btn']
        for k in keys:
            sw[k].config(state=state)

    # ═════════════ 串口 ═════════════
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.set(ports[0])

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.foc_poll_running = False
            self._reader_running = False
            time.sleep(0.3)
            try: self.ser.close()
            except Exception: pass
            self.ser = None
            self.conn_status.config(text="● 未连接", foreground="red")
            self.conn_btn.config(text="连接")
            self.fw_mode_var.set("?")
            self.mode_label.config(foreground="gray")
            for a in range(NUM_AXES):
                self._enable_stepper_buttons(a, "disabled")
                self._apply_foc_gating(a, state="?", fault="?")
                self._apply_gear_gating(a, state="?", fault="?")
                self.v_focstate[a].set("未连接")
                self.v_foccur[a].set("--")
                self.v_focfault[a].set("--")
                # 断开后两组 motor tabs 都重新可见可选，等下次连接重新决定
                self.notebook.tab(self.tab_index_foc[a],  state="normal")
                self.notebook.tab(self.tab_index_gear[a], state="normal")
            self.log("串口已断开")
            return
        try:
            self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=0.2)
            time.sleep(1.0)
            while self.ser.in_waiting:
                self.ser.readline()
            while not self._resp_queue.empty():
                try: self._resp_queue.get_nowait()
                except queue.Empty: break
            self.conn_status.config(text="● 已连接", foreground="green")
            self.conn_btn.config(text="断开")
            for a in range(NUM_AXES):
                self._enable_stepper_buttons(a, "normal")
            self.log(f"已连接 {self.port_var.get()} @ {self.baud_var.get()}")
            self._reader_running = True
            threading.Thread(target=self._reader_loop, daemon=True).start()
            self.foc_poll_running = True
            threading.Thread(target=self._foc_poll_loop, daemon=True).start()
            # 自动查固件模式，并根据模式灰掉另一组 tab + 下发对应模式的 tune
            self._query_mode_and_apply()
        except Exception as e:
            messagebox.showerror("连接失败", str(e))

    def _reader_loop(self):
        while self._reader_running and self.ser and self.ser.is_open:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw: continue
            line = raw.decode(errors="replace").strip()
            if not line: continue
            # 串口原始流可选记录
            if self._raw_log_fh:
                try: self._raw_log_fh.write(f"{time.time():.3f}  RX: {line}\n"); self._raw_log_fh.flush()
                except Exception: pass
            # 解析轴号异步事件
            if line.startswith("STEP,") and line.endswith(",DONE"):
                # STEP,<axis>,DONE
                parts = line.split(",")
                if len(parts) == 3:
                    try:
                        axis = int(parts[1])
                        self.root.after(0, lambda a=axis: self._on_step_done(a))
                        continue
                    except ValueError: pass
            if line.startswith("FOC,") and ",FAULT" in line:
                parts = line.split(",")
                if len(parts) >= 3 and parts[2] == "FAULT":
                    try:
                        axis = int(parts[1])
                        self.root.after(0, lambda a=axis, l=line: self.log(f"⚠️ 轴{AXIS_LABEL[a]}: {l}"))
                        continue
                    except ValueError: pass
            if line.startswith("MOT:") or line.startswith("[FOC") or \
               line.startswith("ESP32") or line.startswith("Protocol:") or \
               line.startswith("NUM_AXES") or line.startswith("  "):
                self.root.after(0, lambda l=line: self.log(l))
                continue
            try:
                self._resp_queue.put_nowait(line)
            except queue.Full: pass

    def _send_and_read(self, cmd, timeout=1.0):
        if not self.ser or not self.ser.is_open: return ""
        with self.serial_lock:
            while not self._resp_queue.empty():
                try: self._resp_queue.get_nowait()
                except queue.Empty: break
            try:
                if self._raw_log_fh:
                    try: self._raw_log_fh.write(f"{time.time():.3f}  TX: {cmd}\n"); self._raw_log_fh.flush()
                    except Exception: pass
                self.ser.write((cmd + "\n").encode())
            except Exception as e:
                self.log(f"串口写异常: {e}")
                return ""
            try:
                return self._resp_queue.get(timeout=timeout)
            except queue.Empty:
                return ""

    # ═════════════ 步进：发送 ═════════════
    def _send_pulses(self, axis, steps, direction, delay_ms):
        if steps <= 0 or not self.ser or not self.ser.is_open: return False
        # 等上次脉冲完成（避免 ESP32 ERR:busy）；显式提示+超时
        if self.stepper_in_progress[axis] and not self.running[axis]:
            self.log(f"轴{AXIS_LABEL[axis]} 等待上次步进完成...")
            wait_deadline = time.time() + 30.0
            while self.stepper_in_progress[axis] and not self.running[axis]:
                if time.time() > wait_deadline:
                    self.log(f"⚠️ 轴{AXIS_LABEL[axis]} 等待超时(30s)，强制清 in_progress 标志")
                    self.stepper_in_progress[axis] = False
                    break
                time.sleep(0.02)
        delay_us = max(1, int(round(delay_ms * 1000)))
        resp = self._send_and_read(f"MOVE,{axis},{steps},{direction},{delay_us}")
        dist_mm = steps / PULSES_PER_MM
        dir_txt = "向外" if direction == DIR_OUTWARD else "向内"
        mm_s = 1000.0 / (PULSES_PER_MM * max(0.001, delay_ms))
        self.log(f"轴{AXIS_LABEL[axis]} {dist_mm:.1f}mm {dir_txt} @ {delay_ms:.1f}ms ({mm_s:.0f}mm/s) → {resp}")
        if resp.startswith("ACK,"):
            sign = +1.0 if direction == DIR_OUTWARD else -1.0
            self._pending_step[axis] = sign * dist_mm
            self.stepper_in_progress[axis] = True
            return True
        return False

    def _send_mm(self, axis, distance_mm, direction, delay_ms):
        steps = int(round(distance_mm * PULSES_PER_MM))
        if steps <= 0:
            self.log(f"轴{AXIS_LABEL[axis]}: 忽略距离过小 ({distance_mm})")
            return False
        sign = +1.0 if direction == DIR_OUTWARD else -1.0
        target = self.position_mm[axis] + sign * (steps / PULSES_PER_MM)
        if not self._check_range(axis, target): return False
        return self._send_pulses(axis, steps, direction, delay_ms)

    def _on_step_done(self, axis):
        if self._pending_step[axis] is not None:
            self.position_mm[axis] += self._pending_step[axis]
            self._pending_step[axis] = None
            self._update_pos_label(axis)
        self.stepper_in_progress[axis] = False
        self.log(f"轴{AXIS_LABEL[axis]} 步进完成")

    # ═════════════ 步进：命令 ═════════════
    def send_move(self, axis):
        threading.Thread(target=self._send_mm,
                         args=(axis, self.v_dist[axis].get(), self.v_dir[axis].get(),
                               self.v_delay[axis].get()), daemon=True).start()

    def _quick_move(self, axis, distance_mm, direction):
        threading.Thread(target=self._send_mm,
                         args=(axis, distance_mm, direction, self.v_delay[axis].get()),
                         daemon=True).start()

    def _press_continuous(self, axis, direction):
        if self.running[axis] or not self.ser or not self.ser.is_open: return
        self.running[axis] = True
        dir_txt = "向外" if direction == DIR_OUTWARD else "向内"
        self.log(f"轴{AXIS_LABEL[axis]} 按住连续{dir_txt}")
        def worker():
            while self.running[axis]:
                while self.stepper_in_progress[axis] and self.running[axis]:
                    time.sleep(0.02)
                if not self.running[axis]: break
                if not self._send_mm(axis, CONTINUOUS_BURST_MM, direction, self.v_delay[axis].get()):
                    break
            self.log(f"轴{AXIS_LABEL[axis]} 连续运动停止")
        threading.Thread(target=worker, daemon=True).start()

    def _release_continuous(self, axis):
        if self.running[axis]: self.running[axis] = False

    def stop_continuous(self, axis):
        self.running[axis] = False
        if self.ser and self.ser.is_open:
            threading.Thread(target=lambda: self._send_and_read(f"FOC,{axis},EN,0"),
                             daemon=True).start()
        self.log(f"⛔ 轴{AXIS_LABEL[axis]} 紧急停止")

    # ═════════════ 步进：位置/原点 ═════════════
    def set_home(self, axis):
        self.position_mm[axis] = 0.0
        self._update_pos_label(axis)
        self.log(f"✓ 轴{AXIS_LABEL[axis]} 当前位置设为原点")

    def calibrate_position(self, axis):
        try: val = float(self.v_goto[axis].get())
        except (tk.TclError, ValueError):
            messagebox.showerror("输入无效", "请先填目标位置"); return
        self.position_mm[axis] = val
        self._update_pos_label(axis)
        self.log(f"✓ 轴{AXIS_LABEL[axis]} 位置校准为 {val:.1f} mm")

    def go_home(self, axis):
        self._goto(axis, 0.0)

    def goto_target_position(self, axis):
        try: target = float(self.v_goto[axis].get())
        except (tk.TclError, ValueError):
            messagebox.showerror("输入无效", "请输入有效目标位置"); return
        self._goto(axis, target)

    def _goto(self, axis, target_mm):
        if not self._check_range(axis, target_mm): return
        delta = target_mm - self.position_mm[axis]
        if abs(delta) < 1.0 / PULSES_PER_MM:
            self.log(f"轴{AXIS_LABEL[axis]} 已在目标附近"); return
        direction = DIR_OUTWARD if delta > 0 else DIR_INWARD
        distance = abs(delta)
        self.log(f"轴{AXIS_LABEL[axis]} 前往 {target_mm:.1f}mm (移动 {distance:.1f}mm)")
        threading.Thread(target=self._send_mm,
                         args=(axis, distance, direction, self.v_delay[axis].get()),
                         daemon=True).start()

    # ═════════════ 行程校准 ═════════════
    def _mark_min(self, axis):
        if self.travel_max_mm[axis] is not None and self.position_mm[axis] >= self.travel_max_mm[axis]:
            messagebox.showerror("范围无效", "最小不能 ≥ 最大"); return
        self.travel_min_mm[axis] = self.position_mm[axis]
        self._update_range_display(axis); self._save_calib()
        self.log(f"⊖ 轴{AXIS_LABEL[axis]} 最小 = {self.travel_min_mm[axis]:.1f}mm")

    def _mark_max(self, axis):
        if self.travel_min_mm[axis] is not None and self.position_mm[axis] <= self.travel_min_mm[axis]:
            messagebox.showerror("范围无效", "最大不能 ≤ 最小"); return
        self.travel_max_mm[axis] = self.position_mm[axis]
        self._update_range_display(axis); self._save_calib()
        self.log(f"⊕ 轴{AXIS_LABEL[axis]} 最大 = {self.travel_max_mm[axis]:.1f}mm")

    def _clear_range(self, axis):
        self.travel_min_mm[axis] = None
        self.travel_max_mm[axis] = None
        self._update_range_display(axis); self._save_calib()
        self.log(f"轴{AXIS_LABEL[axis]} 行程限位已清除")

    def _update_range_display(self, axis):
        mn, mx = self.travel_min_mm[axis], self.travel_max_mm[axis]
        label = self.sw[axis]['range_label']
        if mn is None and mx is None:
            label.config(text="未校准（无软件限位）", foreground="gray")
        else:
            mn_s = f"{mn:.1f}" if mn is not None else "?"
            mx_s = f"{mx:.1f}" if mx is not None else "?"
            travel = f"  (行程 {mx-mn:.1f}mm)" if (mn is not None and mx is not None) else ""
            label.config(text=f"min={mn_s} max={mx_s}{travel}", foreground="black")

    def _check_range(self, axis, target_mm):
        mn, mx = self.travel_min_mm[axis], self.travel_max_mm[axis]
        if mn is not None and target_mm < mn - 0.05:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target_mm:.1f}<下限{mn:.1f}"); return False
        if mx is not None and target_mm > mx + 0.05:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target_mm:.1f}>上限{mx:.1f}"); return False
        return True

    def _save_calib(self):
        data = {str(a): {"min": self.travel_min_mm[a], "max": self.travel_max_mm[a]}
                for a in range(NUM_AXES)}
        try:
            with open(CALIB_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.log(f"⚠️ 校准保存失败: {e}")

    def _load_calib(self):
        if not os.path.exists(CALIB_FILE): return
        try:
            with open(CALIB_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            for a in range(NUM_AXES):
                d = data.get(str(a), {})
                self.travel_min_mm[a] = d.get("min")
                self.travel_max_mm[a] = d.get("max")
                self._update_range_display(a)
            any_set = any(self.travel_min_mm[a] is not None or self.travel_max_mm[a] is not None
                          for a in range(NUM_AXES))
            if any_set:
                self.log("已加载行程校准")
        except Exception as e:
            self.log(f"⚠️ 校准读取失败: {e}")

    # ═════════════ FOC 调参持久化 ═════════════
    def _save_foc_tune(self):
        """把每轴 V/PA/VP/PP 当前值写到 .foc_tune.json。"""
        try:
            data = {str(a): {
                "V":  round(self.v_focvlimit[a].get(), 2),
                "PA": round(self.v_focpangle[a].get(), 2),
                "VP": round(self.v_focvp[a].get(),     3),
                "PP": int(self.v_focpp[a].get()),
            } for a in range(NUM_AXES)}
            with open(FOC_TUNE_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.log(f"⚠️ FOC 调参保存失败: {e}")

    def _load_foc_tune(self):
        """启动时从 json 读回 Tk 变量 + 更新滑条标签。不自动下发到固件，
        要等 toggle_connect 成功后 _apply_foc_tune_to_firmware 再推。"""
        if not os.path.exists(FOC_TUNE_FILE): return
        try:
            with open(FOC_TUNE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            for a in range(NUM_AXES):
                d = data.get(str(a), {})
                if "V"  in d: self.v_focvlimit[a].set(float(d["V"]))
                if "PA" in d: self.v_focpangle[a].set(float(d["PA"]))
                if "VP" in d: self.v_focvp[a].set(float(d["VP"]))
                if "PP" in d: self.v_focpp[a].set(int(d["PP"]))
                # 同步标签
                self.fw[a]['vlimit_label'].config(text=f"{self.v_focvlimit[a].get():.1f} V (扭矩)")
                self.fw[a]['pangle_label'].config(text=f"{self.v_focpangle[a].get():.1f} (刚度)")
                self.fw[a]['vp_label'].config(text=f"{self.v_focvp[a].get():.2f} (阻尼)")
            self.log("已加载 FOC 调参")
        except Exception as e:
            self.log(f"⚠️ FOC 调参读取失败: {e}")

    def _apply_foc_tune_to_firmware(self):
        """连接成功后将已加载的 V/PA/VP 下发给固件；PP 在固件 NVS 里，不需要重发。"""
        def worker():
            time.sleep(0.3)
            for axis in range(NUM_AXES):
                v  = self.v_focvlimit[axis].get()
                pa = self.v_focpangle[axis].get()
                vp = self.v_focvp[axis].get()
                self._send_and_read(f"FOC,{axis},V,{v:.1f}")
                self._send_and_read(f"FOC,{axis},PA,{pa:.1f}")
                self._send_and_read(f"FOC,{axis},VP,{vp:.2f}")
            self.log("FOC 调参已下发固件")
        threading.Thread(target=worker, daemon=True).start()

    # ═════════════ FOC：命令 ═════════════
    def _send_foc(self, axis, sub_and_arg):
        """例：_send_foc(0, 'EN,1') → 发 FOC,0,EN,1 → 日志记响应"""
        cmd = f"FOC,{axis},{sub_and_arg}"
        def worker():
            resp = self._send_and_read(cmd)
            self.log(f"{cmd} → {resp}")
        threading.Thread(target=worker, daemon=True).start()

    def _foc_goto(self, axis):
        self._send_foc(axis, f"A,{self.v_foctgt[axis].get():.1f}")

    def _foc_quick(self, axis, deg):
        self.v_foctgt[axis].set(deg); self._foc_goto(axis)

    def _foc_increment(self, axis, delta):
        self.v_foctgt[axis].set(self.v_foctgt[axis].get() + delta); self._foc_goto(axis)

    def _foc_home(self, axis):
        self._send_foc(axis, "H")

    def _foc_toggle_enable(self, axis):
        self._foc_enabled_ui[axis] = not self._foc_enabled_ui[axis]
        v = 1 if self._foc_enabled_ui[axis] else 0
        self._send_foc(axis, f"EN,{v}")

    def _foc_on_vlimit(self, axis, _):
        v = self.v_focvlimit[axis].get()
        if v > 12.0:
            txt = f"{v:.1f} V ⚠️ 超额定"
            self.fw[axis]['vlimit_label'].config(text=txt, foreground="red")
        else:
            self.fw[axis]['vlimit_label'].config(text=f"{v:.1f} V (扭矩)", foreground="black")
        self._send_foc(axis, f"V,{v:.1f}")

    def _foc_on_pangle(self, axis, _):
        p = self.v_focpangle[axis].get()
        self.fw[axis]['pangle_label'].config(text=f"{p:.1f} (刚度)")
        self._send_foc(axis, f"PA,{p:.1f}")

    def _foc_on_vp(self, axis, _):
        p = self.v_focvp[axis].get()
        self.fw[axis]['vp_label'].config(text=f"{p:.2f} (阻尼)")
        self._send_foc(axis, f"VP,{p:.2f}")

    def _foc_save_pp(self, axis):
        n = int(self.v_focpp[axis].get())
        self._send_foc(axis, f"PP,{n}")
        self._save_foc_tune()

    def _foc_clear_fault(self, axis):
        self._send_foc(axis, "CLR")
        self._foc_enabled_ui[axis] = False

    # ═════════════ FOC：轮询 + 显示 ═════════════
    def _foc_poll_loop(self):
        while self.foc_poll_running and self.ser and self.ser.is_open:
            for axis in range(NUM_AXES):
                if not (self.foc_poll_running and self.ser and self.ser.is_open): break
                resp = self._send_and_read(f"FOC,{axis},S")
                prefix = f"FOC,{axis},S,"
                if resp.startswith(prefix):
                    parts = resp.split(",")
                    # 格式: FOC,<axis>,S,<state>,<cur>,<tgt>,<fault>
                    if len(parts) == 7:
                        s, c, t, f = parts[3], parts[4], parts[5], parts[6]
                        self.root.after(0, lambda a=axis, s=s, c=c, t=t, f=f:
                                        self._update_foc_display(a, s, c, t, f))
                time.sleep(FOC_POLL_INTERVAL_S / NUM_AXES)  # 总周期仍 ~100ms

    def _update_foc_display(self, axis, state, cur, tgt, fault):
        self.v_focstate[axis].set(FOC_STATE_NAMES.get(state, "?"))
        try:
            cur_f = float(cur); tgt_f = float(tgt)
        except ValueError:
            cur_f = tgt_f = None
        cur_text = f"{cur_f:.1f}°" if cur_f is not None else "--"
        if self.stepper_in_progress[axis]: cur_text += " ⏸"
        self.v_foccur[axis].set(cur_text)
        self.v_focfault[axis].set("报警" if fault == "1" else "正常")
        self.fw[axis]['fault_label'].config(foreground="red" if fault == "1" else "green")
        self.gw[axis]['fault_label'].config(foreground="red" if fault == "1" else "green")
        # 两个模式都按响应门控（只激活当前模式的会真正生效）
        self._apply_foc_gating(axis, state, fault)
        self._apply_gear_gating(axis, state, fault)

        if cur_f is not None and state == "2":
            now = time.time()
            self.foc_trace_buf[axis].append((now, tgt_f, cur_f))
            cutoff = now - 10.0
            self.foc_trace_buf[axis] = [x for x in self.foc_trace_buf[axis] if x[0] >= cutoff]
            self._redraw_scope(axis)

    def _apply_gear_gating(self, axis, state, fault):
        gw = self.gw[axis]
        if not self.ser or not self.ser.is_open:
            for w in gw['motion_btns'] + gw['cfg_widgets']:
                w.config(state="disabled")
            gw['enable_btn'].config(state="disabled")
            gw['clear_btn'].config(state="disabled")
            return
        is_fault    = (fault == "1" or state == "3")
        is_disabled = (state == "0")
        is_running  = (state == "2")
        motion_state = "normal" if is_running else "disabled"
        for b in gw['motion_btns']: b.config(state=motion_state)
        cfg_state = "normal" if (is_disabled or is_running) else "disabled"
        for w in gw['cfg_widgets']: w.config(state=cfg_state)
        en_state = "normal" if (is_disabled or is_running) else "disabled"
        gw['enable_btn'].config(state=en_state)
        if is_running:    gw['enable_btn'].config(text="■ 失能 PID")
        elif is_fault:    gw['enable_btn'].config(text="(故障，先清除)")
        else:             gw['enable_btn'].config(text="▶ 使能 PID")
        gw['clear_btn'].config(state="normal" if is_fault else "disabled")

    def _apply_foc_gating(self, axis, state, fault):
        fw = self.fw[axis]
        if not self.ser or not self.ser.is_open:
            for w in fw['motion_btns'] + fw['cfg_btns']:
                w.config(state="disabled")
            fw['enable_btn'].config(state="disabled")
            fw['clear_btn'].config(state="disabled")
            return
        is_fault    = (fault == "1" or state == "3")
        is_disabled = (state == "0")
        is_running  = (state == "2")
        is_aligning = (state == "1")
        motion_state = "normal" if is_running else "disabled"
        for b in fw['motion_btns']: b.config(state=motion_state)
        cfg_state = "normal" if (is_disabled or is_running) else "disabled"
        for w in fw['cfg_btns']: w.config(state=cfg_state)
        en_state = "normal" if (is_disabled or is_running) else "disabled"
        fw['enable_btn'].config(state=en_state)
        if is_running:    fw['enable_btn'].config(text="■ 失能 FOC")
        elif is_aligning: fw['enable_btn'].config(text="… 对齐中")
        elif is_fault:    fw['enable_btn'].config(text="(故障，先清除)")
        else:             fw['enable_btn'].config(text="▶ 使能 FOC")
        fw['clear_btn'].config(state="normal" if is_fault else "disabled")

    def _redraw_scope(self, axis):
        # 在 FOC tab 和 GEAR tab 的画布上都画（共用同一份 trace 缓冲）
        canvases = []
        if axis < len(self.fw) and 'scope' in self.fw[axis]: canvases.append(self.fw[axis]['scope'])
        if axis < len(self.gw) and 'scope' in self.gw[axis]: canvases.append(self.gw[axis]['scope'])
        for cv in canvases:
            self._draw_scope_on(cv, axis)

    def _draw_scope_on(self, cv, axis):
        W, H = 420, 420
        cv.delete("all")
        buf = self.foc_trace_buf[axis]
        if len(buf) < 2:
            cv.create_text(W/2, H/2, text="(等待数据，使能 + 设目标后开始)", fill="#888")
            return
        t0 = buf[0][0]; t_span = max(0.1, buf[-1][0] - t0)
        ys = [p[1] for p in buf] + [p[2] for p in buf]
        y_min, y_max = min(ys), max(ys)
        if y_max - y_min < 10:
            c = (y_min + y_max) / 2; y_min, y_max = c - 5, c + 5
        y_pad = (y_max - y_min) * 0.1
        y_min -= y_pad; y_max += y_pad
        for frac in (0.25, 0.5, 0.75):
            y = H * frac; cv.create_line(0, y, W, y, fill="#e5e5e5")
        if y_min < 0 < y_max:
            y0 = H * (y_max - 0) / (y_max - y_min)
            cv.create_line(0, y0, W, y0, fill="#aaa", dash=(3, 3))
        cv.create_text(3, 3, text=f"{y_max:.0f}°", anchor="nw", fill="#555", font=("Arial", 8))
        cv.create_text(3, H-3, text=f"{y_min:.0f}°", anchor="sw", fill="#555", font=("Arial", 8))
        pts_t, pts_c = [], []
        for t, tgt, cur in buf:
            x = W * (t - t0) / t_span
            yt = H * (y_max - tgt) / (y_max - y_min)
            yc = H * (y_max - cur) / (y_max - y_min)
            pts_t.extend([x, yt]); pts_c.extend([x, yc])
        if len(pts_t) >= 4: cv.create_line(*pts_t, fill="#1565c0", width=1)
        if len(pts_c) >= 4: cv.create_line(*pts_c, fill="#d32f2f", width=2)

    # ═════════════ GEAR Tab（参数化）═════════════
    def _build_gear_tab(self, parent, axis):
        pad = dict(padx=10, pady=5)
        gw = self.gw[axis]

        sf = ttk.LabelFrame(parent, text=f"状态 — 减速 {AXIS_LABEL[axis]} (100ms 轮询)")
        sf.grid(row=0, column=0, sticky="ew", **pad)
        ttk.Label(sf, text="状态:").grid(row=0, column=0, sticky="w", **pad)
        # 复用 v_focstate / v_foccur / v_focfault（FOC 和 GEAR 协议响应一样格式）
        ttk.Label(sf, textvariable=self.v_focstate[axis], width=10,
                  font=("Consolas", 11, "bold"), foreground="gray").grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(sf, text="故障:").grid(row=0, column=2, sticky="w", **pad)
        gw['fault_label'] = ttk.Label(sf, textvariable=self.v_focfault[axis], width=6,
                                      font=("Consolas", 11, "bold"), foreground="gray")
        gw['fault_label'].grid(row=0, column=3, sticky="w", **pad)
        ttk.Label(sf, text="当前角度:").grid(row=1, column=0, sticky="w", **pad)
        ttk.Label(sf, textvariable=self.v_foccur[axis], width=16,
                  font=("Consolas", 14, "bold"), foreground="blue").grid(row=1, column=1, columnspan=3, sticky="w", **pad)

        tf = ttk.LabelFrame(parent, text="目标控制")
        tf.grid(row=1, column=0, sticky="ew", **pad)
        ttk.Label(tf, text="目标角度 (°):").grid(row=0, column=0, sticky="w", **pad)
        ttk.Spinbox(tf, from_=-3600.0, to=3600.0, increment=1.0,
                    textvariable=self.v_foctgt[axis], width=10, format="%.1f").grid(row=0, column=1, **pad)
        gw['goto_btn'] = ttk.Button(tf, text="前往",
                                    command=lambda a=axis: self._gear_goto(a), state="disabled")
        gw['goto_btn'].grid(row=0, column=2, **pad)
        ttk.Label(tf, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
        qf = ttk.Frame(tf); qf.grid(row=1, column=1, columnspan=2, sticky="w", pady=5)
        gw['quick_btns'] = []
        for deg in (0, 45, 90, 180, 270):
            b = ttk.Button(qf, text=f"{deg}°", width=5,
                           command=lambda d=deg, a=axis: self._gear_quick(a, float(d)), state="disabled")
            b.pack(side="left", padx=2)
            gw['quick_btns'].append(b)
        ttk.Label(tf, text="增量:").grid(row=2, column=0, sticky="w", **pad)
        incf = ttk.Frame(tf); incf.grid(row=2, column=1, columnspan=2, sticky="w", pady=5)
        gw['inc_btns'] = []
        for delta in (-10, -1, +1, +10):
            b = ttk.Button(incf, text=f"{delta:+d}°", width=5,
                           command=lambda d=float(delta), a=axis: self._gear_increment(a, d), state="disabled")
            b.pack(side="left", padx=2)
            gw['inc_btns'].append(b)

        hf = ttk.LabelFrame(parent, text="原点")
        hf.grid(row=2, column=0, sticky="ew", **pad)
        gw['home_btn'] = ttk.Button(hf, text="⌂ 把当前位置设为 0°",
                                    command=lambda a=axis: self._gear_home(a), state="disabled")
        gw['home_btn'].grid(row=0, column=0, **pad)

        ef = ttk.LabelFrame(parent, text="使能 / 调参")
        ef.grid(row=3, column=0, sticky="ew", **pad)
        gw['enable_btn'] = ttk.Button(ef, text="▶ 使能 PID",
                                      command=lambda a=axis: self._gear_toggle_enable(a), state="disabled")
        gw['enable_btn'].grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=6)

        ttk.Label(ef, text="PWM 上限 %:").grid(row=1, column=0, sticky="w", **pad)
        vf = ttk.Frame(ef); vf.grid(row=1, column=1, sticky="w", pady=5)
        gw['pwm_slider'] = ttk.Scale(vf, from_=10.0, to=100.0, orient="horizontal",
                                     variable=self.v_gearpwm[axis], length=180,
                                     command=lambda v, a=axis: self._gear_on_pwm(a, v), state="disabled")
        gw['pwm_slider'].pack(side="left")
        gw['pwm_slider'].bind("<ButtonRelease-1>", lambda e: self._save_gear_tune())
        gw['pwm_label'] = ttk.Label(vf, text="100.0 %", width=10)
        gw['pwm_label'].pack(side="left", padx=6)

        ttk.Label(ef, text="Kp (位置比例):").grid(row=2, column=0, sticky="w", **pad)
        kpf = ttk.Frame(ef); kpf.grid(row=2, column=1, sticky="w", pady=5)
        gw['kp_slider'] = ttk.Scale(kpf, from_=0.1, to=50.0, orient="horizontal",
                                    variable=self.v_gearkp[axis], length=180,
                                    command=lambda v, a=axis: self._gear_on_kp(a, v), state="disabled")
        gw['kp_slider'].pack(side="left")
        gw['kp_slider'].bind("<ButtonRelease-1>", lambda e: self._save_gear_tune())
        gw['kp_label'] = ttk.Label(kpf, text="1.0", width=10)
        gw['kp_label'].pack(side="left", padx=6)

        ttk.Label(ef, text="Ki (积分):").grid(row=3, column=0, sticky="w", **pad)
        kif = ttk.Frame(ef); kif.grid(row=3, column=1, sticky="w", pady=5)
        gw['ki_slider'] = ttk.Scale(kif, from_=0.0, to=10.0, orient="horizontal",
                                    variable=self.v_gearki[axis], length=180,
                                    command=lambda v, a=axis: self._gear_on_ki(a, v), state="disabled")
        gw['ki_slider'].pack(side="left")
        gw['ki_slider'].bind("<ButtonRelease-1>", lambda e: self._save_gear_tune())
        gw['ki_label'] = ttk.Label(kif, text="0.00", width=10)
        gw['ki_label'].pack(side="left", padx=6)

        ttk.Label(ef, text="Kd (微分):").grid(row=4, column=0, sticky="w", **pad)
        kdf = ttk.Frame(ef); kdf.grid(row=4, column=1, sticky="w", pady=5)
        gw['kd_slider'] = ttk.Scale(kdf, from_=0.0, to=2.0, orient="horizontal",
                                    variable=self.v_gearkd[axis], length=180,
                                    command=lambda v, a=axis: self._gear_on_kd(a, v), state="disabled")
        gw['kd_slider'].pack(side="left")
        gw['kd_slider'].bind("<ButtonRelease-1>", lambda e: self._save_gear_tune())
        gw['kd_label'] = ttk.Label(kdf, text="0.050", width=10)
        gw['kd_label'].pack(side="left", padx=6)

        ttk.Label(ef, text="齿轮比 GR:").grid(row=5, column=0, sticky="w", **pad)
        grf = ttk.Frame(ef); grf.grid(row=5, column=1, sticky="w", pady=5)
        ttk.Spinbox(grf, from_=1.0, to=10000.0, increment=10.0,
                    textvariable=self.v_geargr[axis], width=10, format="%.1f").pack(side="left")
        gw['gr_save_btn'] = ttk.Button(grf, text="保存到 NVS（立即生效）",
                                       command=lambda a=axis: self._gear_save_gr(a), state="disabled")
        gw['gr_save_btn'].pack(side="left", padx=6)

        gw['clear_btn'] = ttk.Button(ef, text="🧹 清除故障",
                                     command=lambda a=axis: self._gear_clear_fault(a), state="disabled")
        gw['clear_btn'].grid(row=6, column=0, columnspan=2, **pad, ipadx=10)
        gw['autotune_btn'] = ttk.Button(ef, text="🤖 自动调 PID（约 3 分钟）",
                                        command=lambda a=axis: self._gear_autotune(a), state="disabled")
        gw['autotune_btn'].grid(row=7, column=0, columnspan=2, **pad, ipadx=10)

        # 简单响应曲线（复用 foc_trace_buf）
        scf = ttk.LabelFrame(parent, text="响应曲线 (10s · 蓝=目标 红=实测)")
        scf.grid(row=0, column=1, rowspan=4, sticky="nsew", **pad)
        gw['scope'] = tk.Canvas(scf, width=420, height=420, bg="white",
                                highlightthickness=1, highlightbackground="#999")
        gw['scope'].pack(padx=5, pady=5)

        # 聚合：状态门控用
        gw['motion_btns'] = [gw['goto_btn']] + gw['quick_btns'] + gw['inc_btns']
        gw['cfg_widgets'] = [gw['home_btn'], gw['pwm_slider'], gw['kp_slider'],
                             gw['ki_slider'], gw['kd_slider'], gw['gr_save_btn'],
                             gw['autotune_btn']]

    # ═════════════ GEAR：命令 ═════════════
    def _send_gear(self, axis, sub_and_arg):
        """和 _send_foc 一样格式，GEAR 固件也用 FOC,<axis>,... 命名空间。"""
        cmd = f"FOC,{axis},{sub_and_arg}"
        def worker():
            resp = self._send_and_read(cmd)
            self.log(f"{cmd} → {resp}")
        threading.Thread(target=worker, daemon=True).start()

    def _gear_goto(self, axis):
        target = self.v_foctgt[axis].get()
        self._send_gear(axis, f"A,{target:.1f}")
        self._start_goto_watcher(axis, target)

    def _start_goto_watcher(self, axis, target_deg, tol_deg=2.0, timeout_s=15.0):
        """后台监视 cur → tgt 的逼近，每 1.5s 打印进度。新 goto / 失能 自动取消旧 watcher。"""
        self._goto_watcher_gen[axis] += 1
        my_gen = self._goto_watcher_gen[axis]
        def worker():
            t0 = time.time()
            last_log = 0.0
            self.log(f"→ 轴{AXIS_LABEL[axis]} 前往 {target_deg:.1f}°")
            while self.ser and self.ser.is_open:
                # 1. 被新 goto 取代 → 退出（不打日志）
                if self._goto_watcher_gen[axis] != my_gen:
                    return
                # 2. PID 已失能 → 退出（电机滑行，不可能到位）
                state_txt = self.v_focstate[axis].get()
                if state_txt != "运行":
                    self.log(f"  轴{AXIS_LABEL[axis]} watcher 退出（PID 状态={state_txt}）")
                    return
                # 3. 读当前角度
                raw = self.v_foccur[axis].get()
                try:
                    cur = float(raw.rstrip("°").rstrip(" ⏸").strip())
                except (ValueError, AttributeError):
                    time.sleep(0.3); continue
                err = target_deg - cur
                dt = time.time() - t0
                # 4. 到位
                if abs(err) < tol_deg:
                    self.log(f"✓ 轴{AXIS_LABEL[axis]} 到位 cur={cur:.1f}° (用时 {dt:.1f}s)")
                    return
                # 5. 超时
                if dt > timeout_s:
                    self.log(f"⚠️ 轴{AXIS_LABEL[axis]} {timeout_s:.0f}s 未到位 "
                             f"cur={cur:.1f}° 差 {err:+.1f}° (Kp 太小？发 DIAG,0 看 PWM)")
                    return
                # 6. 周期进度
                if time.time() - last_log > 1.5:
                    self.log(f"  轴{AXIS_LABEL[axis]} cur={cur:.1f}° 差 {err:+.1f}° (t={dt:.1f}s)")
                    last_log = time.time()
                time.sleep(0.3)
        threading.Thread(target=worker, daemon=True).start()

    def _gear_quick(self, axis, deg):
        self.v_foctgt[axis].set(deg); self._gear_goto(axis)

    def _gear_increment(self, axis, delta):
        self.v_foctgt[axis].set(self.v_foctgt[axis].get() + delta); self._gear_goto(axis)

    def _gear_home(self, axis):
        self._send_gear(axis, "H")

    def _gear_toggle_enable(self, axis):
        self._foc_enabled_ui[axis] = not self._foc_enabled_ui[axis]
        v = 1 if self._foc_enabled_ui[axis] else 0
        self._send_gear(axis, f"EN,{v}")

    def _gear_on_pwm(self, axis, _):
        v = self.v_gearpwm[axis].get()
        self.gw[axis]['pwm_label'].config(text=f"{v:.1f} %")
        self._send_gear(axis, f"V,{v:.1f}")   # GEAR 固件 V 是 PWM 百分比

    def _gear_on_kp(self, axis, _):
        v = self.v_gearkp[axis].get()
        self.gw[axis]['kp_label'].config(text=f"{v:.2f}")
        self._send_gear(axis, f"PA,{v:.2f}")

    def _gear_on_ki(self, axis, _):
        v = self.v_gearki[axis].get()
        self.gw[axis]['ki_label'].config(text=f"{v:.2f}")
        self._send_gear(axis, f"PI,{v:.2f}")

    def _gear_on_kd(self, axis, _):
        v = self.v_gearkd[axis].get()
        self.gw[axis]['kd_label'].config(text=f"{v:.3f}")
        self._send_gear(axis, f"PD,{v:.3f}")

    def _gear_save_gr(self, axis):
        gr = float(self.v_geargr[axis].get())
        self._send_gear(axis, f"GR,{gr:.1f}")
        self._save_gear_tune()

    def _gear_clear_fault(self, axis):
        self._send_gear(axis, "CLR")
        self._foc_enabled_ui[axis] = False

    # ═════════════ GEAR 调参持久化 ═════════════
    def _save_gear_tune(self):
        try:
            data = {str(a): {
                "PWM": round(self.v_gearpwm[a].get(), 2),
                "Kp":  round(self.v_gearkp[a].get(),  3),
                "Ki":  round(self.v_gearki[a].get(),  3),
                "Kd":  round(self.v_gearkd[a].get(),  4),
                "GR":  round(self.v_geargr[a].get(),  1),
            } for a in range(NUM_AXES)}
            with open(GEAR_TUNE_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            # 简短日志反馈 axis 0 当前值（最常用）
            d = data["0"]
            self.log(f"💾 GEAR 调参已保存到 {os.path.basename(GEAR_TUNE_FILE)} "
                     f"(L: PWM={d['PWM']}% Kp={d['Kp']} Ki={d['Ki']} Kd={d['Kd']} GR={d['GR']})")
        except Exception as e:
            self.log(f"⚠️ GEAR 调参保存失败: {e}")

    def _load_gear_tune(self):
        if not os.path.exists(GEAR_TUNE_FILE): return
        try:
            with open(GEAR_TUNE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            for a in range(NUM_AXES):
                d = data.get(str(a), {})
                if "PWM" in d: self.v_gearpwm[a].set(float(d["PWM"]))
                if "Kp"  in d: self.v_gearkp[a].set(float(d["Kp"]))
                if "Ki"  in d: self.v_gearki[a].set(float(d["Ki"]))
                if "Kd"  in d: self.v_gearkd[a].set(float(d["Kd"]))
                if "GR"  in d: self.v_geargr[a].set(float(d["GR"]))
                self.gw[a]['pwm_label'].config(text=f"{self.v_gearpwm[a].get():.1f} %")
                self.gw[a]['kp_label'].config(text=f"{self.v_gearkp[a].get():.2f}")
                self.gw[a]['ki_label'].config(text=f"{self.v_gearki[a].get():.2f}")
                self.gw[a]['kd_label'].config(text=f"{self.v_gearkd[a].get():.3f}")
            self.log("已加载 GEAR 调参")
        except Exception as e:
            self.log(f"⚠️ GEAR 调参读取失败: {e}")

    def _apply_gear_tune_to_firmware(self):
        def worker():
            time.sleep(0.3)
            for axis in range(NUM_AXES):
                self._send_and_read(f"FOC,{axis},V,{self.v_gearpwm[axis].get():.1f}")
                self._send_and_read(f"FOC,{axis},PA,{self.v_gearkp[axis].get():.2f}")
                self._send_and_read(f"FOC,{axis},PI,{self.v_gearki[axis].get():.2f}")
                self._send_and_read(f"FOC,{axis},PD,{self.v_gearkd[axis].get():.3f}")
            self.log("GEAR 调参已下发固件")
        threading.Thread(target=worker, daemon=True).start()

    # ═════════════ 模式自动检测 + tab 灰显 ═════════════
    def _query_mode_and_apply(self):
        """连接成功后调一次。用户选 Auto 时发 MODE 命令自动检测；
        用户选 FOC/GEAR 时直接强制应用，不查询固件。"""
        sel = self.mode_select_var.get()
        if sel in (MODE_FOC, MODE_GEAR):
            self.log(f"模式选择 = 强制 {sel}（跳过固件 MODE 查询）")
            self.root.after(0, lambda: self._apply_mode_to_tabs(sel))
            return
        # Auto
        def worker():
            time.sleep(0.4)
            resp = self._send_and_read("MODE", timeout=1.5)
            mode = MODE_FOC
            if resp.startswith("MODE,"):
                m = resp.split(",", 1)[1].strip().upper()
                if m in (MODE_FOC, MODE_GEAR): mode = m
            else:
                self.log(f"⚠️ 固件未返回 MODE（响应={resp!r}）。请烧含 MODE 命令的固件，"
                         f"或用'强制 FOC/GEAR'手动选。先按 FOC 处理。")
            self.root.after(0, lambda: self._apply_mode_to_tabs(mode))
        threading.Thread(target=worker, daemon=True).start()

    def _on_mode_select_change(self):
        """用户切换 Auto/FOC/GEAR radio 时调用。如果已连接，立即重新应用模式。"""
        sel = self.mode_select_var.get()
        self.log(f"模式选择 → {sel}")
        if self.ser and self.ser.is_open:
            self._query_mode_and_apply()

    def _apply_mode_to_tabs(self, mode):
        """灰掉非当前模式的 tab，更新 mode 标签，下发对应模式的 tune。"""
        self.fw_mode_var.set(mode)
        if mode == MODE_FOC:
            self.mode_label.config(foreground="#1565c0")
            for a in range(NUM_AXES):
                self.notebook.tab(self.tab_index_foc[a],  state="normal")
                self.notebook.tab(self.tab_index_gear[a], state="disabled")
            # 自动跳到第一个 FOC tab
            self.notebook.select(self.tab_index_foc[0])
            self._apply_foc_tune_to_firmware()
        elif mode == MODE_GEAR:
            self.mode_label.config(foreground="#2e7d32")
            for a in range(NUM_AXES):
                self.notebook.tab(self.tab_index_foc[a],  state="disabled")
                self.notebook.tab(self.tab_index_gear[a], state="normal")
            self.notebook.select(self.tab_index_gear[0])
            self._apply_gear_tune_to_firmware()
        self.log(f"固件模式 = {mode}")

    # ═════════════ FOC：自动调参（单轴）═════════════
    def _foc_autotune(self, axis):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("未连接", "请先连接串口"); return
        if not messagebox.askokcancel(
            "自动调参确认",
            f"轴 {AXIS_LABEL[axis]} 两阶段扫描 PA + VP，约 2 分钟。\n电机会来回转动，请先固定好。"):
            return
        self.fw[axis]['autotune_btn'].config(state="disabled")
        threading.Thread(target=lambda: self._foc_autotune_worker(axis), daemon=True).start()

    def _step_response_test(self, axis, target, pre_settle=2.5, duration=5.0):
        # duration 默认 5s（之前 2.5s 对扭矩不足的轴会超时占位 rt=99）
        self._send_and_read(f"FOC,{axis},A,0"); time.sleep(pre_settle)
        t0 = time.time()
        self._send_and_read(f"FOC,{axis},A,{target}"); time.sleep(duration)
        data = [(t - t0, cur) for t, tgt, cur in self.foc_trace_buf[axis]
                if t >= t0 and abs(tgt - target) < 0.5]
        if len(data) < 5: return None
        curs = [c for _, c in data]
        overshoot = max(0.0, max(curs) - target) if target > 0 else max(0.0, target - min(curs))
        ss_error = abs(curs[-1] - target)
        rt = None
        thr = target * 0.9
        for t, c in data:
            if c >= thr: rt = t; break
        tail = [c for t, c in data if t >= duration - 0.8]
        if len(tail) >= 3:
            m = sum(tail)/len(tail)
            jit = (sum((x-m)**2 for x in tail)/len(tail))**0.5
        else: jit = 0.0
        return (overshoot, ss_error, rt or 99.0, jit)

    def _foc_autotune_worker(self, axis):
        try:
            self.log(f"🤖 轴{AXIS_LABEL[axis]} 自动调参开始")
            self._send_and_read(f"FOC,{axis},EN,1"); time.sleep(5.5)
            self._send_and_read(f"FOC,{axis},H"); time.sleep(0.5)
            self._send_and_read(f"FOC,{axis},VP,0.15")
            pa_results = []
            for pa in [5, 10, 15, 20, 25, 30]:
                self._send_and_read(f"FOC,{axis},PA,{pa}"); time.sleep(0.3)
                m = self._step_response_test(axis, 60.0)  # 60° + 默认 5s 窗口
                if m is None: continue
                ov, sse, rt, jt = m
                self.log(f"  PA={pa}: 过冲={ov:.1f}° 误差={sse:.1f}° 上升={rt:.2f}s 抖={jt:.2f}°")
                pa_results.append((pa, ov, sse, rt, jt))
                if ov > 30: break
            if not pa_results:
                self.log("❌ 无数据"); return
            good = [r for r in pa_results if r[1] <= 15]
            pool = good if good else pa_results
            best_pa = min(pool, key=lambda r: r[1]*2 + r[3]*3 + r[2]*2 + r[4]*5)[0]
            self._send_and_read(f"FOC,{axis},PA,{best_pa}"); time.sleep(0.3)
            vp_results = []
            for vp in [0.10, 0.15, 0.20, 0.30, 0.40, 0.55, 0.70]:
                self._send_and_read(f"FOC,{axis},VP,{vp}"); time.sleep(0.3)
                m = self._step_response_test(axis, 60.0)  # 60° + 默认 5s 窗口
                if m is None: continue
                ov, sse, rt, jt = m
                self.log(f"  VP={vp:.2f}: 过冲={ov:.1f}° 上升={rt:.2f}s 抖={jt:.2f}°")
                vp_results.append((vp, ov, sse, rt, jt))
                if jt > 3.0: break
            best_vp = 0.15
            if vp_results:
                good = [r for r in vp_results if r[4] <= 2.0]
                pool = good if good else vp_results
                best_vp = min(pool, key=lambda r: r[1]*2 + r[3]*2 + r[4]*10)[0]
            self._send_and_read(f"FOC,{axis},PA,{best_pa}"); time.sleep(0.2)
            self._send_and_read(f"FOC,{axis},VP,{best_vp}"); time.sleep(0.2)
            self._send_and_read(f"FOC,{axis},A,0")
            self.log(f"✅ 轴{AXIS_LABEL[axis]} 推荐：PA={best_pa}  VP={best_vp:.2f}")
            self.root.after(0, lambda: self.v_focpangle[axis].set(float(best_pa)))
            self.root.after(0, lambda: self.fw[axis]['pangle_label'].config(text=f"{best_pa:.1f} (刚度)"))
            self.root.after(0, lambda: self.v_focvp[axis].set(float(best_vp)))
            self.root.after(0, lambda: self.fw[axis]['vp_label'].config(text=f"{best_vp:.2f} (阻尼)"))
            # 持久化自动调参结果
            self._save_foc_tune()
        finally:
            self.root.after(0, lambda: self.fw[axis]['autotune_btn'].config(state="normal"))

    # ═════════════ GEAR：自动调参（三阶段 Kp → Kd → Ki）═════════════
    def _gear_autotune(self, axis):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("未连接", "请先连接串口"); return
        if not messagebox.askokcancel(
            "GEAR 自动调参",
            f"轴 {AXIS_LABEL[axis]} 三阶段扫描 Kp → Kd → Ki，约 3 分钟。\n"
            f"电机会反复在 0° ↔ 60° 之间走，请确认机械空间够。"):
            return
        self.gw[axis]['autotune_btn'].config(state="disabled")
        threading.Thread(target=lambda: self._gear_autotune_worker(axis), daemon=True).start()

    def _gear_autotune_worker(self, axis):
        # 阶段评分权重（越小越好）：[overshoot, ss_error, rise_time, jitter]
        TARGET = 60.0
        try:
            self.log(f"🤖 轴{AXIS_LABEL[axis]} GEAR 自动调参开始 (目标 ±{TARGET:.0f}°)")
            # 重置到已知状态
            self._send_and_read(f"FOC,{axis},V,100");  time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},PI,0");   time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},PD,0.05"); time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},PA,5");   time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},EN,1");   time.sleep(0.5)
            self._send_and_read(f"FOC,{axis},H");      time.sleep(0.3)

            # ── 阶段 1: Kp 扫描 ──
            self.log(f"--- 阶段 1/3: Kp 扫描 ---")
            kp_results = []
            for kp in [5, 10, 15, 20, 25, 30, 35]:
                self._send_and_read(f"FOC,{axis},PA,{kp}"); time.sleep(0.3)
                m = self._step_response_test(axis, TARGET, pre_settle=2.0, duration=5.0)
                if m is None:
                    self.log(f"  Kp={kp}: 无数据"); continue
                ov, sse, rt, jt = m
                self.log(f"  Kp={kp}: 过冲={ov:.1f}° 稳态误差={sse:.1f}° "
                         f"上升={rt:.2f}s 抖={jt:.2f}°")
                kp_results.append((kp, ov, sse, rt, jt))
                if ov > 40:  # 过冲太大，停止扫描避免机械冲击
                    self.log(f"  ⚠️ Kp={kp} 过冲 {ov:.1f}° 太大，停止扫描"); break
            if not kp_results:
                self.log("❌ Kp 阶段无有效数据，autotune 中止"); return
            # 选最佳 Kp：过冲适中 + 上升快 + 稳态误差小
            good = [r for r in kp_results if r[1] <= 15.0]
            pool = good if good else kp_results
            best_kp = min(pool, key=lambda r: r[1]*2 + r[3]*3 + r[2]*2 + r[4]*4)[0]
            self.log(f"→ 选 Kp = {best_kp}")
            self._send_and_read(f"FOC,{axis},PA,{best_kp}"); time.sleep(0.3)

            # ── 阶段 2: Kd 扫描（抑制过冲）──
            self.log(f"--- 阶段 2/3: Kd 扫描 ---")
            kd_results = []
            for kd in [0.05, 0.1, 0.2, 0.3, 0.5, 0.8]:
                self._send_and_read(f"FOC,{axis},PD,{kd}"); time.sleep(0.3)
                m = self._step_response_test(axis, TARGET, pre_settle=2.0, duration=5.0)
                if m is None: continue
                ov, sse, rt, jt = m
                self.log(f"  Kd={kd:.2f}: 过冲={ov:.1f}° 上升={rt:.2f}s 抖={jt:.2f}°")
                kd_results.append((kd, ov, sse, rt, jt))
                if jt > 4.0:  # 抖动太大，停止
                    self.log(f"  ⚠️ Kd={kd} 抖动 {jt:.1f}° 太大，停止扫描"); break
            best_kd = 0.1
            if kd_results:
                # 优先：过冲小 + 抖动小
                good = [r for r in kd_results if r[4] <= 2.0]
                pool = good if good else kd_results
                best_kd = min(pool, key=lambda r: r[1]*3 + r[3]*1 + r[4]*5)[0]
            self.log(f"→ 选 Kd = {best_kd:.2f}")
            self._send_and_read(f"FOC,{axis},PD,{best_kd}"); time.sleep(0.3)

            # ── 阶段 3: Ki 扫描（消稳态误差）──
            self.log(f"--- 阶段 3/3: Ki 扫描 ---")
            ki_results = []
            for ki in [0.0, 0.3, 0.7, 1.5, 3.0]:
                self._send_and_read(f"FOC,{axis},PI,{ki}"); time.sleep(0.3)
                m = self._step_response_test(axis, TARGET, pre_settle=2.0, duration=5.0)
                if m is None: continue
                ov, sse, rt, jt = m
                self.log(f"  Ki={ki:.2f}: 过冲={ov:.1f}° 稳态误差={sse:.1f}° 抖={jt:.2f}°")
                ki_results.append((ki, ov, sse, rt, jt))
                if jt > 5.0 or ov > 25:
                    self.log(f"  ⚠️ Ki={ki} 失稳，停止扫描"); break
            best_ki = 0.0
            if ki_results:
                # 优先：稳态误差小 + 抖动小，过冲控制
                good = [r for r in ki_results if r[1] <= 12.0 and r[4] <= 3.0]
                pool = good if good else ki_results
                best_ki = min(pool, key=lambda r: r[2]*5 + r[1]*1 + r[4]*3)[0]
            self.log(f"→ 选 Ki = {best_ki:.2f}")
            self._send_and_read(f"FOC,{axis},PI,{best_ki}"); time.sleep(0.3)

            # 收尾：回 0°、更新 GUI 滑块、写 .gear_tune.json
            self._send_and_read(f"FOC,{axis},A,0"); time.sleep(2.0)
            self.log(f"✅ 轴{AXIS_LABEL[axis]} 推荐：Kp={best_kp}  Kd={best_kd:.2f}  Ki={best_ki:.2f}")
            self.root.after(0, lambda: self.v_gearkp[axis].set(float(best_kp)))
            self.root.after(0, lambda: self.gw[axis]['kp_label'].config(text=f"{best_kp:.2f}"))
            self.root.after(0, lambda: self.v_gearkd[axis].set(float(best_kd)))
            self.root.after(0, lambda: self.gw[axis]['kd_label'].config(text=f"{best_kd:.3f}"))
            self.root.after(0, lambda: self.v_gearki[axis].set(float(best_ki)))
            self.root.after(0, lambda: self.gw[axis]['ki_label'].config(text=f"{best_ki:.2f}"))
            self._save_gear_tune()
        finally:
            self.root.after(0, lambda: self.gw[axis]['autotune_btn'].config(state="normal"))

    # ═════════════ 日志 ═════════════
    def _classify_log(self, msg):
        if any(k in msg for k in ("⛔", "ERR", "FAULT", "失败", "异常", "拒绝")):
            return "err"
        if any(k in msg for k in ("⚠️", "warn")):
            return "warn"
        if "轴L" in msg or "axis 0" in msg or "FOC,0," in msg or "MOVE,0" in msg:
            return "axisL"
        if "轴R" in msg or "axis 1" in msg or "FOC,1," in msg or "MOVE,1" in msg:
            return "axisR"
        if any(k in msg for k in ("✓", "OK", "DONE", "完成", "通过")):
            return "ok"
        if msg.startswith("MOT:") or msg.startswith("[FOC"):
            return "rx"
        return ""

    def _log_file_path(self):
        return os.path.join(LOG_DIR, f"gui_{time.strftime('%Y-%m-%d')}.log")

    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        full = f"{ts}  {msg}"
        # 写日期分割的日志文件（追加）
        try:
            with open(self._log_file_path(), "a", encoding="utf-8") as f:
                f.write(full + "\n")
        except Exception:
            pass
        # UI 上色显示
        tag = self._classify_log(msg)
        def _append():
            self.log_text.config(state="normal")
            if tag:
                self.log_text.insert("end", full + "\n", tag)
            else:
                self.log_text.insert("end", full + "\n")
            self.log_text.see("end")
            self.log_text.config(state="disabled")
        self.root.after(0, _append)

    def clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _open_log_dir(self):
        try:
            os.startfile(LOG_DIR)  # Windows
        except Exception:
            messagebox.showinfo("日志目录", LOG_DIR)

    def _toggle_raw_log(self):
        """打开/关闭串口原始流文件记录（每条 RX 行写到独立 log）。"""
        if self.raw_log_var.get():
            try:
                path = os.path.join(LOG_DIR, f"raw_{time.strftime('%Y%m%d_%H%M%S')}.log")
                self._raw_log_fh = open(path, "w", encoding="utf-8")
                self.log(f"串口原始流 → {os.path.basename(path)}")
            except Exception as e:
                self.log(f"⚠️ 原始流打开失败: {e}")
                self.raw_log_var.set(False)
        else:
            if self._raw_log_fh:
                try: self._raw_log_fh.close()
                except Exception: pass
                self._raw_log_fh = None
                self.log("串口原始流已停止")


if __name__ == "__main__":
    root = tk.Tk()
    app = StepperGUI(root)
    root.mainloop()
