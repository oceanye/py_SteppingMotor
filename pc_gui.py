import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import queue
import json
import os
import socket
import webbrowser

# ============== 轴配置 ==============
# 步进轴和闭环电机是两套独立的轴集合。兼容常量仅供旧配置使用，
# 新代码必须明确选择 NUM_STEPPER_AXES 或 NUM_MOTOR_AXES。
NUM_STEPPER_AXES = 6
NUM_MOTOR_AXES = 2
NUM_AXES = NUM_STEPPER_AXES       # 向后兼容
NUM_GEAR_AXES = NUM_MOTOR_AXES    # 向后兼容
AXIS_LABEL = ["L", "R", "A", "B", "C", "D"]

# ============== 步进标定 ==============
# ESP32-S3 PUL/DIR 引脚（与 esp32_stepper/src/config.h DRIVE_MODE_GEAR 一致），仅用于界面显示。
STEPPER_PINS = [(5, 6), (7, 15), (1, 2), (4, 8), (9, 10), (38, 39)]
# 步进控制模式：直线（高度-导程，单位 mm）或 旋转（圈/角度，单位 °）。
MODE_LINEAR = "linear"
MODE_ROTARY = "rotary"
# 每轴电机默认参数：脉冲每转（微步/圈）、减速比（电机→负载）、导程（mm/转，仅直线模式）。
DEFAULT_PULSE_PER_REV = 200.0
DEFAULT_GEAR_RATIO = 1.0
DEFAULT_LEAD_MM = 1.0
# 兼容：默认 200 微步/圈 ÷ 1.0mm 导程 = 200 pulse/mm（28HD140GT81-200LR 贯通式步进）。
PULSES_PER_MM = [DEFAULT_PULSE_PER_REV / DEFAULT_LEAD_MM] * NUM_STEPPER_AXES
# 速度档位（单位/s：直线=mm/s, 旋转=°/s）
SPEED_PRESETS = [0.3, 0.5, 0.6, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 8, 10]
SPEED_DEFAULT = 3.0
# 实测：delayMicroseconds 路径每步固定开销 ~200us（50us HIGH + yield + 调度）
# 用于速度补偿：发送的 delay 比理论值少 200us，使实际速度逼近设定值
DELAY_OVERHEAD_US = 200

def _speed_to_delay_ms(speed, ppm):
    """速度(mm/s) → 补偿后延迟(ms)。快速档减去固定开销，慢速档(vTaskDelay)不补偿。"""
    target_us = 1000000.0 / (ppm * speed)
    if target_us >= 2000:
        return target_us / 1000.0
    return max(0.05, (target_us - DELAY_OVERHEAD_US) / 1000.0)

DELAY_DEFAULT_MS = [_speed_to_delay_ms(SPEED_DEFAULT, PULSES_PER_MM[a]) for a in range(NUM_STEPPER_AXES)]
DIR_OUTWARD = 1
DIR_INWARD  = 0
# 每轴方向翻转标志（电机安装方向不同时用）
# 0 = 不翻转, 1 = 翻转 DIR 信号。两轴原始方向均正确，不翻转。
DIR_INVERT = [0] * NUM_STEPPER_AXES
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
    return {i: None for i in range(NUM_STEPPER_AXES)}


class StepperGUI:
    def __init__(self, root, web_server_factory=None):
        self.root = root
        self.root.title("六轴步进 + FOC/GEAR + 轨道 D 控制器")
        self.root.resizable(True, True)

        # ── 共享：串口 ──
        self.ser = None
        self.serial_lock = threading.Lock()
        self.state_lock = threading.RLock()
        self._disconnect_lock = threading.Lock()
        self._estop_lock = threading.Lock()
        self._control_context = threading.local()
        self._resp_queue = queue.Queue()
        self._ui_actions = queue.Queue()
        self._reader_running = False
        self.foc_poll_running = False
        self._last_serial_error_log = 0.0
        self._serial_failure_handled = False
        self._serial_generation = 0
        self._control_generation = 0
        self._closing = False
        self._disconnecting = False
        self._estop_in_progress = False

        # ── 每轴状态（list 按 axis 索引）──
        self.running            = [False] * NUM_STEPPER_AXES  # 连续运动 flag
        self.position_mm        = [0.0]   * NUM_STEPPER_AXES
        self.position_trusted   = [True]  * NUM_STEPPER_AXES
        self.travel_min_mm      = [None]  * NUM_STEPPER_AXES
        self.travel_max_mm      = [None]  * NUM_STEPPER_AXES
        self.stepper_in_progress= [False] * NUM_STEPPER_AXES
        self._pending_step      = [None]  * NUM_STEPPER_AXES
        self._move_dispatching  = [False] * NUM_STEPPER_AXES
        self._web_step_pending  = [False] * NUM_STEPPER_AXES
        self._foc_enabled_ui    = [False] * NUM_MOTOR_AXES
        self.foc_trace_buf      = [[] for _ in range(NUM_MOTOR_AXES)]
        self._motor_status      = [
            {"state": "?", "current_deg": None, "target_deg": None, "fault": None}
            for _ in range(NUM_MOTOR_AXES)
        ]
        # goto watcher 代数：每次新 goto +1，旧 watcher 检测到 gen 变了就退出
        self._goto_watcher_gen  = [0] * NUM_MOTOR_AXES

        # ── 每轴 Tk 变量 ──
        self.v_dist   = [tk.DoubleVar(value=10.0) for _ in range(NUM_STEPPER_AXES)]
        self.v_dir    = [tk.IntVar   (value=DIR_OUTWARD) for _ in range(NUM_STEPPER_AXES)]
        self.v_delay  = [tk.DoubleVar(value=DELAY_DEFAULT_MS[a]) for a in range(NUM_STEPPER_AXES)]
        self.v_speed_str = [tk.StringVar(value=str(SPEED_DEFAULT)) for _ in range(NUM_STEPPER_AXES)]
        self.v_goto   = [tk.DoubleVar(value=0.0)  for _ in range(NUM_STEPPER_AXES)]
        self.v_foctgt = [tk.DoubleVar(value=0.0)  for _ in range(NUM_MOTOR_AXES)]
        self.v_focstate   = [tk.StringVar(value="未连接") for _ in range(NUM_MOTOR_AXES)]
        self.v_focfault   = [tk.StringVar(value="--")    for _ in range(NUM_MOTOR_AXES)]
        self.v_foccur     = [tk.StringVar(value="--")    for _ in range(NUM_MOTOR_AXES)]
        self.v_focvlimit  = [tk.DoubleVar(value=10.0)    for _ in range(NUM_MOTOR_AXES)]
        self.v_focpangle  = [tk.DoubleVar(value=25.0)    for _ in range(NUM_MOTOR_AXES)]
        self.v_focvp      = [tk.DoubleVar(value=0.2)     for _ in range(NUM_MOTOR_AXES)]
        self.v_focpp      = [tk.IntVar   (value=7)       for _ in range(NUM_MOTOR_AXES)]

        # ── GEAR 模式调参变量（和 FOC 共享 v_foctgt / v_focstate / v_foccur / v_focfault）──
        self.v_gearpwm = [tk.DoubleVar(value=100.0)  for _ in range(NUM_MOTOR_AXES)]   # PWM duty cap %（0-100）
        self.v_gearkp  = [tk.DoubleVar(value=1.0)    for _ in range(NUM_MOTOR_AXES)]
        self.v_gearki  = [tk.DoubleVar(value=0.0)    for _ in range(NUM_MOTOR_AXES)]
        self.v_gearkd  = [tk.DoubleVar(value=0.05)   for _ in range(NUM_MOTOR_AXES)]
        self.v_geargr  = [tk.DoubleVar(value=1000.0) for _ in range(NUM_MOTOR_AXES)]   # 齿轮比（NVS）

        # 轨道 D（独立 DRV8871）状态。命令采用 800 ms 租约，GUI 按住时续租。
        self.v_track_duty = tk.IntVar(value=60)
        self.v_track_status = tk.StringVar(value="已停止")
        self._track_direction = "STOP"
        self._track_lease_ms = 800
        self._track_lease_generation = 0
        self._track_last_response = ""

        # ── 模式：
        #    mode_select_var: 用户选择 (Auto/FOC/GEAR)；Auto 时从固件 MODE 命令读
        #    fw_mode_var:      当前实际生效的模式显示 (FOC/GEAR/?)
        self.mode_select_var = tk.StringVar(value="Auto")
        self.fw_mode_var = tk.StringVar(value="?")
        self._fw_mode_cache = "?"

        # ── 每轴 widget refs（dict-per-axis）──
        self.sw = [dict() for _ in range(NUM_STEPPER_AXES)]  # stepper widgets
        self.fw = [dict() for _ in range(NUM_MOTOR_AXES)]    # FOC widgets
        self.gw = [dict() for _ in range(NUM_MOTOR_AXES)]    # GEAR widgets
        self.tw = {}
        self.web_server = None
        self.web_status_var = tk.StringVar(value="● 服务未启动")
        self.web_address_var = tk.StringVar(value="—")
        self.web_port_var = tk.StringVar(value="—")
        self.web_url_var = tk.StringVar(value="网页服务未启动")

        # ── 每轴步进配置：控制模式 + 电机参数（脉冲每转/减速比/导程）──
        self.axis_mode          = [MODE_LINEAR] * NUM_STEPPER_AXES
        self.axis_pulse_per_rev = [DEFAULT_PULSE_PER_REV] * NUM_STEPPER_AXES
        self.axis_gear_ratio    = [DEFAULT_GEAR_RATIO] * NUM_STEPPER_AXES
        self.axis_lead_mm       = [DEFAULT_LEAD_MM] * NUM_STEPPER_AXES
        self.axis_mode_var = [tk.StringVar(value=MODE_LINEAR) for _ in range(NUM_STEPPER_AXES)]
        self.axis_ppr_var  = [tk.DoubleVar(value=DEFAULT_PULSE_PER_REV) for _ in range(NUM_STEPPER_AXES)]
        self.axis_gr_var   = [tk.DoubleVar(value=DEFAULT_GEAR_RATIO) for _ in range(NUM_STEPPER_AXES)]
        self.axis_lead_var = [tk.DoubleVar(value=DEFAULT_LEAD_MM) for _ in range(NUM_STEPPER_AXES)]
        self._load_axis_config()

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(25, self._drain_ui_actions)
        self._load_calib()
        self._load_foc_tune()
        self._load_gear_tune()
        if web_server_factory is not None:
            self._start_web_server(web_server_factory)

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

        ttk.Label(conn_frame, text="网页服务:").grid(row=2, column=0, sticky="e", **pad)
        self.web_status_label = ttk.Label(
            conn_frame, textvariable=self.web_status_var, foreground="gray")
        self.web_status_label.grid(row=2, column=1, columnspan=2, sticky="w", **pad)
        ttk.Label(conn_frame, text="手机访问地址:").grid(row=2, column=3, sticky="e", **pad)
        ttk.Entry(conn_frame, textvariable=self.web_address_var, width=28,
                  state="readonly").grid(row=2, column=4, columnspan=3, sticky="ew", **pad)
        ttk.Label(conn_frame, text="端口:").grid(row=2, column=7, sticky="e", **pad)
        ttk.Label(conn_frame, textvariable=self.web_port_var,
                  font=("Consolas", 11, "bold")).grid(row=2, column=8, sticky="w", **pad)

        ttk.Label(conn_frame, text="手机完整网址:").grid(row=3, column=0, sticky="e", **pad)
        self.web_url_entry = ttk.Entry(conn_frame, textvariable=self.web_url_var,
                                       width=69, state="readonly")
        self.web_url_entry.grid(row=3, column=1, columnspan=6, sticky="ew", **pad)
        ttk.Button(conn_frame, text="复制手机网址", command=self._copy_web_url).grid(
            row=3, column=7, **pad)
        ttk.Button(conn_frame, text="本机打开", command=self._open_web_url).grid(
            row=3, column=8, **pad)

        ttk.Label(
            conn_frame,
            text="流程：① 手机与台式机连接同一局域网  ② GUI 连接 ESP32 串口  "
                 "③ 手机打开上方固定网址  ④ 页面显示“串口已连接”后控制电机",
            foreground="#555",
        ).grid(row=4, column=0, columnspan=9, sticky="w", padx=8, pady=(1, 5))

        # Notebook: 步进 ×6 + FOC ×2 + GEAR ×2 + 轨道 D。
        # FOC/GEAR 互斥：连接后根据固件 MODE 灰掉另一组。
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        self.tab_index_step = [None] * NUM_STEPPER_AXES
        self.tab_index_foc  = [None] * NUM_MOTOR_AXES
        self.tab_index_gear = [None] * NUM_MOTOR_AXES
        for axis in range(NUM_STEPPER_AXES):
            tab = ttk.Frame(self.notebook)
            self.tab_index_step[axis] = self.notebook.index("end")
            self.notebook.add(tab, text=f"🔩 步进 {AXIS_LABEL[axis]}")
            self._build_stepper_tab(tab, axis)
        for axis in range(NUM_MOTOR_AXES):
            tab = ttk.Frame(self.notebook)
            self.tab_index_foc[axis] = self.notebook.index("end")
            self.notebook.add(tab, text=f"🧲 FOC {AXIS_LABEL[axis]}")
            self._build_foc_tab(tab, axis)
        for axis in range(NUM_MOTOR_AXES):
            tab = ttk.Frame(self.notebook)
            self.tab_index_gear[axis] = self.notebook.index("end")
            self.notebook.add(tab, text=f"⚙ 减速 {AXIS_LABEL[axis]}")
            self._build_gear_tab(tab, axis)

        track_tab = ttk.Frame(self.notebook)
        self.tab_index_track = self.notebook.index("end")
        self.notebook.add(track_tab, text="↔ 轨道 D")
        self._build_track_tab(track_tab)

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

        # 窗口自适应屏幕：允许缩放，notebook 行可伸缩，初始高度不超出屏幕。
        self.root.rowconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.update_idletasks()
        req_w = self.root.winfo_reqwidth()
        req_h = self.root.winfo_reqheight()
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        win_w = min(req_w, screen_w - 40)
        win_h = min(req_h, screen_h - 80)
        self.root.geometry(f"{int(win_w)}x{int(win_h)}")
        self.root.minsize(720, 480)

        self.refresh_ports()

    # ═════════════ 步进 Tab（参数化）═════════════
    def _build_stepper_tab(self, parent, axis):
        pad = dict(padx=10, pady=5)
        sw = self.sw[axis]
        unit = self._unit_label(axis)

        # 轴配置：GPIO 引脚 + 控制模式 + 电机参数（脉冲每转/减速比/导程）。
        axf = ttk.LabelFrame(parent, text=f"轴配置 — 轴 {AXIS_LABEL[axis]}")
        axf.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)
        pul, dr = STEPPER_PINS[axis] if axis < len(STEPPER_PINS) else ("?", "?")
        ttk.Label(axf, text=f"引脚:  PUL=GPIO{pul}   DIR=GPIO{dr}",
                  font=("Consolas", 10, "bold"), foreground="#1565c0").grid(
            row=0, column=0, columnspan=6, sticky="w", **pad)
        ttk.Label(axf, text="模式:").grid(row=1, column=0, sticky="w", **pad)
        mf = ttk.Frame(axf); mf.grid(row=1, column=1, columnspan=5, sticky="w")
        ttk.Radiobutton(mf, text="直线 高度/导程 (mm)", variable=self.axis_mode_var[axis],
                        value=MODE_LINEAR, command=lambda a=axis: self._on_axis_mode_change(a)).pack(side="left", padx=6)
        ttk.Radiobutton(mf, text="旋转 圈/角度 (°)", variable=self.axis_mode_var[axis],
                        value=MODE_ROTARY, command=lambda a=axis: self._on_axis_mode_change(a)).pack(side="left", padx=6)
        ttk.Label(axf, text="脉冲/转:").grid(row=2, column=0, sticky="w", **pad)
        ttk.Spinbox(axf, from_=1.0, to=10000.0, increment=1.0,
                    textvariable=self.axis_ppr_var[axis], width=8, format="%.1f",
                    command=lambda a=axis: self._on_axis_param_change(a)).grid(row=2, column=1, **pad)
        ttk.Label(axf, text="减速比:").grid(row=2, column=2, sticky="w", padx=(16,0))
        ttk.Spinbox(axf, from_=0.001, to=1000.0, increment=0.01,
                    textvariable=self.axis_gr_var[axis], width=8, format="%.2f",
                    command=lambda a=axis: self._on_axis_param_change(a)).grid(row=2, column=3, **pad)
        sw['lead_label'] = ttk.Label(axf, text="导程(mm/转):")
        sw['lead_label'].grid(row=2, column=4, sticky="w", padx=(16,0))
        sw['lead_spin'] = ttk.Spinbox(axf, from_=0.01, to=100.0, increment=0.1,
                                      textvariable=self.axis_lead_var[axis], width=8, format="%.3f",
                                      command=lambda a=axis: self._on_axis_param_change(a))
        sw['lead_spin'].grid(row=2, column=5, **pad)
        self._apply_axis_param_ui(axis)

        pf = ttk.LabelFrame(parent, text=f"运动参数 — 轴 {AXIS_LABEL[axis]}")
        pf.grid(row=1, column=0, sticky="nsew", **pad)
        sw['dist_label'] = ttk.Label(pf, text=f"距离 ({unit}):")
        sw['dist_label'].grid(row=0, column=0, sticky="w", **pad)
        ttk.Spinbox(pf, from_=0.2, to=500.0, increment=1.0,
                    textvariable=self.v_dist[axis], width=10, format="%.1f").grid(row=0, column=1, **pad)
        ttk.Label(pf, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
        qbf = ttk.Frame(pf); qbf.grid(row=1, column=1, sticky="w", pady=5)
        for label, val in [("1", 1), ("10", 10), ("50", 50), ("100", 100)]:
            ttk.Button(qbf, text=label, width=6,
                       command=lambda d=val, a=axis: self.v_dist[a].set(d)).pack(side="left", padx=2)
        ttk.Label(pf, text="方向:").grid(row=2, column=0, sticky="w", **pad)
        df = ttk.Frame(pf); df.grid(row=2, column=1, sticky="w")
        ttk.Radiobutton(df, text="正向 ▶", variable=self.v_dir[axis], value=DIR_OUTWARD).pack(side="left", padx=4)
        ttk.Radiobutton(df, text="◀ 反向", variable=self.v_dir[axis], value=DIR_INWARD).pack(side="left", padx=4)
        ttk.Label(pf, text="速度档位:").grid(row=3, column=0, sticky="w", **pad)
        sf = ttk.Frame(pf); sf.grid(row=3, column=1, sticky="w", pady=5)
        sw['speed_combo'] = ttk.Combobox(sf, textvariable=self.v_speed_str[axis],
                                         values=[str(s) for s in SPEED_PRESETS],
                                         width=6, state="readonly")
        sw['speed_combo'].pack(side="left")
        sw['speed_unit_label'] = ttk.Label(sf, text=self._unit_per_s(axis))
        sw['speed_unit_label'].pack(side="left", padx=2)
        sw['delay_label'] = ttk.Label(sf, text="", width=16)
        sw['delay_label'].pack(side="left", padx=6)
        sw['speed_combo'].bind("<<ComboboxSelected>>", lambda e, a=axis: self._on_speed_select(a))
        self._update_speed_label(axis)

        cf = ttk.LabelFrame(parent, text="控制")
        cf.grid(row=1, column=1, sticky="nsew", **pad)
        sw['move_btn'] = ttk.Button(cf, text="执行运动",
                                    command=lambda a=axis: self.send_move(a), state="disabled")
        sw['move_btn'].grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=8)
        sw['jog_out_btn'] = ttk.Button(cf, text="正向 1 ▶",
                                       command=lambda a=axis: self._quick_move(a, 1.0, DIR_OUTWARD),
                                       state="disabled")
        sw['jog_out_btn'].grid(row=1, column=0, **pad)
        sw['jog_in_btn'] = ttk.Button(cf, text="◀ 反向 1",
                                      command=lambda a=axis: self._quick_move(a, 1.0, DIR_INWARD),
                                      state="disabled")
        sw['jog_in_btn'].grid(row=1, column=1, **pad)
        sw['cont_out_btn'] = ttk.Button(cf, text="正向 (按住) ▶▶", state="disabled")
        sw['cont_out_btn'].grid(row=2, column=0, **pad)
        sw['cont_out_btn'].bind("<ButtonPress-1>",   lambda e, a=axis: self._press_continuous(a, DIR_OUTWARD))
        sw['cont_out_btn'].bind("<ButtonRelease-1>", lambda e, a=axis: self._release_continuous(a))
        sw['cont_out_btn'].bind("<Leave>",           lambda e, a=axis: self._release_continuous(a))
        sw['cont_in_btn'] = ttk.Button(cf, text="◀◀ 反向 (按住)", state="disabled")
        sw['cont_in_btn'].grid(row=2, column=1, **pad)
        sw['cont_in_btn'].bind("<ButtonPress-1>",   lambda e, a=axis: self._press_continuous(a, DIR_INWARD))
        sw['cont_in_btn'].bind("<ButtonRelease-1>", lambda e, a=axis: self._release_continuous(a))
        sw['cont_in_btn'].bind("<Leave>",           lambda e, a=axis: self._release_continuous(a))
        sw['stop_btn'] = ttk.Button(cf, text="■ 紧急停止",
                                    command=lambda a=axis: self.stop_continuous(a), state="disabled")
        sw['stop_btn'].grid(row=3, column=0, columnspan=2, **pad, ipadx=10)
        sw['progress'] = ttk.Progressbar(cf, orient="horizontal", length=200, mode="determinate")
        sw['progress'].grid(row=4, column=0, columnspan=2, sticky="ew", padx=10, pady=(8,0))
        sw['progress_label'] = ttk.Label(cf, text="", font=("Consolas", 9))
        sw['progress_label'].grid(row=5, column=0, columnspan=2, sticky="w", padx=10)

        rf = ttk.LabelFrame(parent, text="位置与原点（软件跟踪）")
        rf.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)
        ttk.Label(rf, text="当前位置:").grid(row=0, column=0, sticky="w", **pad)
        sw['pos_label'] = ttk.Label(rf, text=f"0.0 {unit}", font=("Consolas", 14, "bold"), foreground="blue")
        sw['pos_label'].grid(row=0, column=1, sticky="w", **pad)
        sw['set_home_btn'] = ttk.Button(rf, text=f"⌂ 设为原点 (0 {unit})",
                                        command=lambda a=axis: self.set_home(a), state="disabled")
        sw['set_home_btn'].grid(row=0, column=2, **pad)
        sw['go_home_btn'] = ttk.Button(rf, text="⟲ 回到原点",
                                       command=lambda a=axis: self.go_home(a), state="disabled")
        sw['go_home_btn'].grid(row=0, column=3, **pad)
        sw['goto_label'] = ttk.Label(rf, text=f"前往位置 ({unit}):")
        sw['goto_label'].grid(row=1, column=0, sticky="w", **pad)
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
        fw['scope'] = tk.Canvas(scf, width=420, height=280, bg="white",
                                highlightthickness=1, highlightbackground="#999")
        fw['scope'].pack(padx=5, pady=5)

        # 聚合：状态门控用
        fw['motion_btns'] = [fw['goto_btn']] + fw['quick_btns'] + fw['inc_btns']
        fw['cfg_btns']    = [fw['home_btn'], fw['vlimit_slider'], fw['pangle_slider'],
                             fw['vp_slider'], fw['pp_save_btn'], fw['autotune_btn']]

    # ═════════════ 轨道 D（独立 DRV8871）═════════════
    def _build_track_tab(self, parent):
        pad = dict(padx=12, pady=8)
        frame = ttk.LabelFrame(parent, text="轨道 D 直流电机（DRV8871）")
        frame.grid(row=0, column=0, sticky="nsew", **pad)

        ttk.Label(frame, text="PWM 占空比:").grid(row=0, column=0, sticky="w", **pad)
        duty = ttk.Spinbox(frame, from_=1, to=100, increment=1,
                           textvariable=self.v_track_duty, width=8, state="disabled")
        duty.grid(row=0, column=1, sticky="w", **pad)
        ttk.Label(frame, text="%（按住按钮期间有效）").grid(row=0, column=2, sticky="w", **pad)

        fwd = ttk.Button(frame, text="▶ 按住前进", state="disabled")
        fwd.grid(row=1, column=0, **pad, ipadx=18, ipady=12)
        rev = ttk.Button(frame, text="◀ 按住后退", state="disabled")
        rev.grid(row=1, column=1, **pad, ipadx=18, ipady=12)
        stop = ttk.Button(frame, text="■ STOP", command=self._track_release,
                          state="disabled")
        stop.grid(row=1, column=2, **pad, ipadx=18, ipady=12)

        for button, direction in ((fwd, "FWD"), (rev, "REV")):
            button.bind("<ButtonPress-1>",
                        lambda _event, d=direction: self._track_press(d))
            button.bind("<ButtonRelease-1>", lambda _event: self._track_release())
            button.bind("<Leave>", lambda _event: self._track_release())

        ttk.Label(frame, text="状态:").grid(row=2, column=0, sticky="e", **pad)
        status = ttk.Label(frame, textvariable=self.v_track_status,
                           font=("Consolas", 12, "bold"), foreground="#555")
        status.grid(row=2, column=1, columnspan=2, sticky="w", **pad)
        ttk.Label(
            frame,
            text="安全机制：运行命令带 800 ms 租约；按住时约每 250~500 ms 续租，\n"
                 "松开、鼠标移出、串口断开或网页停止时立即发送 STOP。",
            foreground="#666",
        ).grid(row=3, column=0, columnspan=3, sticky="w", **pad)

        self.tw.update(duty=duty, fwd_btn=fwd, rev_btn=rev,
                       stop_btn=stop, status_label=status)

    def _set_track_controls(self, state):
        for key in ("duty", "fwd_btn", "rev_btn", "stop_btn"):
            widget = self.tw.get(key)
            if widget is not None:
                widget.config(state=state)

    def _track_press(self, direction):
        try:
            duty = max(1, min(100, int(self.v_track_duty.get())))
        except (TypeError, ValueError, tk.TclError):
            duty = 60
            self.v_track_duty.set(duty)
        self._start_track_lease(direction, duty)

    def _start_track_lease(self, direction, duty):
        if direction not in ("FWD", "REV"):
            return False
        if not self._is_serial_connected():
            self.v_track_status.set("未连接")
            return False
        self._track_lease_generation += 1
        generation = self._track_lease_generation
        with self.state_lock:
            self._track_direction = direction
            self._track_lease_ms = 800
        self.v_track_status.set(f"{direction} · PWM {duty}% · 租约续租中")
        self.tw['status_label'].config(foreground="#1565c0")
        self._track_lease_tick(generation, direction, duty)
        return True

    def _track_lease_tick(self, generation, direction, duty):
        if generation != self._track_lease_generation or self._track_direction != direction:
            return

        def worker():
            response = self._send_and_read(
                f"TRACK,D,{direction},{duty},800",
                timeout=0.25,
                guard=lambda: (generation == self._track_lease_generation
                               and self._track_direction == direction),
            )
            with self.state_lock:
                self._track_last_response = response
            if generation == self._track_lease_generation and self._track_direction == direction:
                self._post_ui(lambda: self.root.after(
                    250, lambda: self._track_lease_tick(generation, direction, duty)))

        threading.Thread(target=worker, daemon=True).start()

    def _track_release(self):
        self._track_lease_generation += 1
        generation = self._track_lease_generation
        with self.state_lock:
            self._track_direction = "STOP"
        self.v_track_status.set("已停止")
        if self.tw.get('status_label'):
            self.tw['status_label'].config(foreground="#555")
        threading.Thread(target=self._send_track_stop,
                         args=(generation,), daemon=True).start()

    def _send_track_stop(self, generation=None, control_guard=None):
        def guard():
            track_ok = (generation is None or
                        (generation == self._track_lease_generation
                         and self._track_direction == "STOP"))
            return track_ok and (control_guard is None or control_guard())
        response = self._send_and_read("TRACK,D,STOP", timeout=0.6, guard=guard)
        with self.state_lock:
            self._track_last_response = response
        return response

    # ═════════════ 共享辅助 ═════════════
    def _post_ui(self, callback):
        """从工作线程安全投递 Tk 操作；HTTP/串口线程不得直接调用 Tk。"""
        self._ui_actions.put(callback)

    def _start_control_worker(self, target, *args):
        """启动绑定当前控制代数的写命令线程；ESTOP 后旧代数自动失效。"""
        with self.state_lock:
            if self._estop_in_progress:
                self.log("软件急停正在执行，已拒绝新的 GUI 控制命令")
                return None
            generation = self._control_generation

        def runner():
            self._control_context.generation = generation
            try:
                target(*args)
            finally:
                try:
                    del self._control_context.generation
                except AttributeError:
                    pass

        thread = threading.Thread(target=runner, daemon=True)
        thread.start()
        return thread

    def _drain_ui_actions(self):
        for _ in range(100):
            try:
                callback = self._ui_actions.get_nowait()
            except queue.Empty:
                break
            try:
                callback()
            except Exception:
                # UI 已销毁或单个刷新失败时不影响后续安全控制命令。
                pass
        try:
            self.root.after(25, self._drain_ui_actions)
        except tk.TclError:
            pass

    def _on_speed_select(self, axis):
        try:
            speed = float(self.v_speed_str[axis].get())
        except ValueError:
            return
        if speed <= 0: return
        self.v_delay[axis].set(_speed_to_delay_ms(speed, self._pulses_per_unit(axis)))
        self._update_speed_label(axis)

    def _update_speed_label(self, axis):
        d = self.v_delay[axis].get()
        self.sw[axis]['delay_label'].config(text=f"({d:.2f} ms/脉冲)")

    def _on_step_progress(self, axis, done, total):
        if 0 <= axis < len(self.sw) and 'progress' in self.sw[axis]:
            pct = int(done * 100 / total) if total > 0 else 0
            self.sw[axis]['progress']['value'] = pct
            self.sw[axis]['progress_label'].config(text=f"{pct}%  ({done}/{total} 步)")

    def _reset_progress(self, axis):
        if axis < len(self.sw) and 'progress' in self.sw[axis]:
            self.sw[axis]['progress']['value'] = 0
            self.sw[axis]['progress_label'].config(text="")

    # ═════════════ 步进：每轴配置（模式/减速比/导程）═════════════
    def _pulses_per_unit(self, axis):
        """当前模式下每单位（直线=mm, 旋转=度）对应的脉冲数。"""
        ppr = max(1.0, self.axis_pulse_per_rev[axis])
        gr = max(1e-6, self.axis_gear_ratio[axis])
        if self.axis_mode[axis] == MODE_ROTARY:
            return ppr * gr / 360.0
        lead = max(1e-6, self.axis_lead_mm[axis])
        return ppr * gr / lead

    def _unit_label(self, axis):
        return "°" if self.axis_mode[axis] == MODE_ROTARY else "mm"

    def _unit_per_s(self, axis):
        return "°/s" if self.axis_mode[axis] == MODE_ROTARY else "mm/s"

    def _on_axis_mode_change(self, axis):
        mode = self.axis_mode_var[axis].get()
        if mode not in (MODE_LINEAR, MODE_ROTARY) or mode == self.axis_mode[axis]:
            return
        self.axis_mode[axis] = mode
        # 模式切换后位置语义改变：重置位置与行程，标记需要重新校准。
        with self.state_lock:
            self.position_mm[axis] = 0.0
            self.position_trusted[axis] = False
            self.travel_min_mm[axis] = None
            self.travel_max_mm[axis] = None
        self._apply_axis_param_ui(axis)
        self._refresh_axis_unit_labels(axis)
        self._update_pos_label(axis)
        self._update_range_display(axis)
        self._on_axis_param_change(axis)
        self._save_axis_config()
        self.log(f"轴{AXIS_LABEL[axis]} 切换为 {'旋转(°)' if mode == MODE_ROTARY else '直线(mm)'}，位置已重置需重新校准")

    def _on_axis_param_change(self, axis):
        try:
            self.axis_pulse_per_rev[axis] = max(1.0, float(self.axis_ppr_var[axis].get()))
            self.axis_gear_ratio[axis] = max(1e-6, float(self.axis_gr_var[axis].get()))
            self.axis_lead_mm[axis] = max(1e-6, float(self.axis_lead_var[axis].get()))
        except (tk.TclError, ValueError):
            return
        try:
            speed = float(self.v_speed_str[axis].get())
        except ValueError:
            speed = SPEED_DEFAULT
        self.v_delay[axis].set(_speed_to_delay_ms(speed, self._pulses_per_unit(axis)))
        self._update_speed_label(axis)
        self._save_axis_config()

    def _apply_axis_param_ui(self, axis):
        state = "normal" if self.axis_mode[axis] == MODE_LINEAR else "disabled"
        sw = self.sw[axis]
        if 'lead_spin' in sw:
            sw['lead_spin'].config(state=state)
            sw['lead_label'].config(state=state)

    def _refresh_axis_unit_labels(self, axis):
        unit = self._unit_label(axis)
        sw = self.sw[axis]
        if 'dist_label' in sw: sw['dist_label'].config(text=f"距离 ({unit}):")
        if 'speed_unit_label' in sw: sw['speed_unit_label'].config(text=self._unit_per_s(axis))
        if 'goto_label' in sw: sw['goto_label'].config(text=f"前往位置 ({unit}):")
        if 'set_home_btn' in sw: sw['set_home_btn'].config(text=f"⌂ 设为原点 (0 {unit})")

    AXIS_CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".stepper_axis.json")

    def _save_axis_config(self):
        try:
            data = {
                "mode": list(self.axis_mode),
                "pulse_per_rev": list(self.axis_pulse_per_rev),
                "gear_ratio": list(self.axis_gear_ratio),
                "lead_mm": list(self.axis_lead_mm),
            }
            with open(self.AXIS_CONFIG_FILE, "w", encoding="utf-8") as fh:
                json.dump(data, fh, ensure_ascii=False, indent=2)
        except OSError:
            pass

    def _load_axis_config(self):
        try:
            with open(self.AXIS_CONFIG_FILE, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError, json.JSONDecodeError):
            return
        mode_list = data.get("mode", [])
        ppr_list = data.get("pulse_per_rev", [])
        gr_list = data.get("gear_ratio", [])
        lead_list = data.get("lead_mm", [])
        for axis in range(NUM_STEPPER_AXES):
            if axis < len(mode_list) and mode_list[axis] in (MODE_LINEAR, MODE_ROTARY):
                self.axis_mode[axis] = mode_list[axis]
                self.axis_mode_var[axis].set(self.axis_mode[axis])
            if axis < len(ppr_list):
                try:
                    v = max(1.0, float(ppr_list[axis]))
                    self.axis_pulse_per_rev[axis] = v
                    self.axis_ppr_var[axis].set(v)
                except (ValueError, TypeError):
                    pass
            if axis < len(gr_list):
                try:
                    v = max(1e-6, float(gr_list[axis]))
                    self.axis_gear_ratio[axis] = v
                    self.axis_gr_var[axis].set(v)
                except (ValueError, TypeError):
                    pass
            if axis < len(lead_list):
                try:
                    v = max(1e-6, float(lead_list[axis]))
                    self.axis_lead_mm[axis] = v
                    self.axis_lead_var[axis].set(v)
                except (ValueError, TypeError):
                    pass

    def _update_pos_label(self, axis):
        trusted = self.position_trusted[axis]
        unit = self._unit_label(axis)
        text = f"{self.position_mm[axis]:.1f} {unit}" if trusted else f"≈ {self.position_mm[axis]:.1f} {unit}（需校准）"
        self.sw[axis]['pos_label'].config(
            text=text, foreground="blue" if trusted else "#d84315")

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
        if self._is_serial_connected():
            # 正常断开前停止所有输出；否则断开串口后电机仍可能继续执行或保持使能。
            with self.state_lock:
                self._disconnecting = True
            self._stop_all_outputs(allow_closing=True)
            self._disconnect_serial("用户断开串口", is_error=False)
            return
        try:
            self._serial_failure_handled = False
            with self.state_lock:
                self._disconnecting = False
            self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=0.2)
            self._serial_generation += 1
            generation = self._serial_generation
            time.sleep(1.0)
            while self.ser.in_waiting:
                self.ser.readline()
            while not self._resp_queue.empty():
                try: self._resp_queue.get_nowait()
                except queue.Empty: break
            self.conn_status.config(text="● 已连接", foreground="green")
            self.conn_btn.config(text="断开")
            for a in range(NUM_STEPPER_AXES):
                self._enable_stepper_buttons(a, "normal")
            self._set_track_controls("normal")
            self.log(f"已连接 {self.port_var.get()} @ {self.baud_var.get()}")
            self._reader_running = True
            threading.Thread(target=self._reader_loop, args=(generation, self.ser), daemon=True).start()
            self.foc_poll_running = True
            threading.Thread(target=self._foc_poll_loop, args=(generation,), daemon=True).start()
            # 自动查固件模式，并根据模式灰掉另一组 tab + 下发对应模式的 tune
            self._query_mode_and_apply()
        except Exception as e:
            self._disconnect_serial(str(e), is_error=True)
            messagebox.showerror("连接失败", str(e))

    def _is_serial_connected(self):
        ser = self.ser
        return bool(ser and getattr(ser, "is_open", False))

    def _disconnect_serial(self, reason="", is_error=False):
        """统一停止读写/轮询并关闭串口；可安全地从任意线程重复调用。"""
        with self._disconnect_lock:
            if self._serial_failure_handled and is_error:
                return
            if is_error:
                self._serial_failure_handled = True
            self._reader_running = False
            self.foc_poll_running = False
            self._serial_generation += 1
            ser, self.ser = self.ser, None
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

        def update_ui():
            self._track_lease_generation += 1
            with self.state_lock:
                self._track_direction = "STOP"
            self.v_track_status.set("连接中断，已停止")
            self._set_track_controls("disabled")
            if self._raw_log_fh:
                try:
                    self._raw_log_fh.close()
                except Exception:
                    pass
                self._raw_log_fh = None
                self.raw_log_var.set(False)
            self.conn_status.config(text="● 未连接", foreground="red")
            self.conn_btn.config(text="连接")
            self.fw_mode_var.set("?")
            with self.state_lock:
                self._fw_mode_cache = "?"
                self._motor_status = [
                    {"state": "?", "current_deg": None, "target_deg": None, "fault": None}
                    for _ in range(NUM_MOTOR_AXES)
                ]
            self.mode_label.config(foreground="gray")
            for axis in range(NUM_STEPPER_AXES):
                self.running[axis] = False
                if (self.stepper_in_progress[axis] or self._move_dispatching[axis]
                        or self._pending_step[axis] is not None):
                    self.position_trusted[axis] = False
                    self._pending_step[axis] = None
                    self._move_dispatching[axis] = False
                    self.stepper_in_progress[axis] = False
                    self._update_pos_label(axis)
                self._enable_stepper_buttons(axis, "disabled")
            for axis in range(NUM_MOTOR_AXES):
                self._apply_foc_gating(axis, state="?", fault="?")
                self._apply_gear_gating(axis, state="?", fault="?")
                self.v_focstate[axis].set("未连接")
                self.v_foccur[axis].set("--")
                self.v_focfault[axis].set("--")
                self.notebook.tab(self.tab_index_foc[axis], state="normal")
                self.notebook.tab(self.tab_index_gear[axis], state="normal")
            self._save_calib()

        self._post_ui(update_ui)
        now = time.monotonic()
        if not is_error or now - self._last_serial_error_log >= 5.0:
            self._last_serial_error_log = now
            prefix = "串口 I/O 异常，已停止轮询并断开" if is_error else "串口已断开"
            self.log(f"{prefix}: {reason}" if reason else prefix)

    def _handle_serial_failure(self, exc, source):
        self._disconnect_serial(f"{source}: {exc}", is_error=True)

    def _reader_loop(self, generation, ser):
        while (self._reader_running and generation == self._serial_generation
               and ser is self.ser and ser.is_open):
            try:
                raw = ser.readline()
            except Exception as exc:
                if generation == self._serial_generation:
                    self._handle_serial_failure(exc, "读取")
                break
            if not raw: continue
            line = raw.decode(errors="replace").strip()
            if not line: continue
            # 串口原始流可选记录
            if self._raw_log_fh:
                try: self._raw_log_fh.write(f"{time.time():.3f}  RX: {line}\n"); self._raw_log_fh.flush()
                except Exception: pass
            # 解析轴号异步事件
            if line.startswith("STEP,") and ",P," in line:
                # STEP,<axis>,P,<done>,<total>
                parts = line.split(",")
                if len(parts) == 5 and parts[2] == "P":
                    try:
                        axis = int(parts[1])
                        done = int(parts[3])
                        total = int(parts[4])
                        self._post_ui(lambda a=axis, d=done, t=total: self._on_step_progress(a, d, t))
                        continue
                    except ValueError: pass
            if line.startswith("STEP,"):
                # 新格式: STEP,<axis>,DONE|ABORT,<executed_steps>,<requested_steps>
                # 兼容旧格式: STEP,<axis>,DONE|ABORT
                parts = line.split(",")
                if len(parts) in (3, 5) and parts[2] in ("DONE", "ABORT"):
                    try:
                        axis = int(parts[1])
                        result = parts[2]
                        executed = requested = None
                        if len(parts) == 5:
                            executed = int(parts[3])
                            requested = int(parts[4])
                        if 0 <= axis < NUM_STEPPER_AXES:
                            if result == "DONE":
                                self._post_ui(lambda a=axis, e=executed, r=requested:
                                              self._on_step_done(a, e, r))
                            else:
                                self._post_ui(lambda a=axis, e=executed, r=requested:
                                              self._on_step_aborted(a, e, r))
                        continue
                    except ValueError: pass
            if line == "TRACK,D,TIMEOUT":
                self._track_lease_generation += 1
                with self.state_lock:
                    self._track_direction = "STOP"
                    self._track_last_response = line
                self._post_ui(lambda: self.v_track_status.set("轨道租约到期，已停止"))
                self._post_ui(lambda: self.log("轨道 D 固件租约到期，已自动停止"))
                continue
            if line == "HWESTOP,TRIGGERED":
                # 固件硬件急停已执行 system_estop()；本地同步状态，不重发 ESTOP。
                # 步进轴随后的 STEP ABORT 事件与 GEAR 轮询会各自更新对应状态。
                self._track_lease_generation += 1
                with self.state_lock:
                    self._track_direction = "STOP"
                    self._track_last_response = line
                self._post_ui(lambda: self.v_track_status.set("硬件急停触发，已停止"))
                self._post_ui(lambda: self.log("⚠️ 硬件急停触发（HWESTOP,TRIGGERED）：固件已停止全部步进/轨道并失能闭环轴；释放急停按钮后可重新操作"))
                continue
            if line.startswith("FOC,") and ",FAULT" in line:
                parts = line.split(",")
                if len(parts) >= 3 and parts[2] == "FAULT":
                    try:
                        axis = int(parts[1])
                        if not 0 <= axis < NUM_MOTOR_AXES:
                            continue
                        self._post_ui(lambda a=axis, l=line: self.log(f"⚠️ 轴{AXIS_LABEL[a]}: {l}"))
                        continue
                    except ValueError: pass
            if line.startswith("MOT:") or line.startswith("[FOC") or \
               line.startswith("ESP32") or line.startswith("Protocol:") or \
               line.startswith("NUM_AXES") or line.startswith("  "):
                self._post_ui(lambda l=line: self.log(l))
                continue
            try:
                self._resp_queue.put_nowait(line)
            except queue.Full: pass

    def _send_and_read(self, cmd, timeout=1.0, guard=None, allow_closing=False):
        blocked = self._closing or self._disconnecting
        context_generation = getattr(self._control_context, "generation", None)
        context_stale = (context_generation is not None and
                         context_generation != self._control_generation)
        if ((blocked and not allow_closing) or context_stale
                or not self._is_serial_connected()):
            return ""
        with self.serial_lock:
            context_stale = (context_generation is not None and
                             context_generation != self._control_generation)
            if (((self._closing or self._disconnecting) and not allow_closing)
                    or context_stale):
                return ""
            ser = self.ser
            if not ser or not ser.is_open:
                return ""
            if guard is not None and not guard():
                return ""
            while not self._resp_queue.empty():
                try: self._resp_queue.get_nowait()
                except queue.Empty: break
            try:
                if self._raw_log_fh:
                    try: self._raw_log_fh.write(f"{time.time():.3f}  TX: {cmd}\n"); self._raw_log_fh.flush()
                    except Exception: pass
                ser.write((cmd + "\n").encode())
            except Exception as e:
                self._handle_serial_failure(e, "写入")
                return ""
            try:
                return self._resp_queue.get(timeout=timeout)
            except queue.Empty:
                return ""

    # ═════════════ 步进：发送 ═════════════
    def _send_pulses(self, axis, steps, direction, delay_ms, guard=None):
        if not 0 <= axis < NUM_STEPPER_AXES:
            return False
        if steps <= 0 or not self._is_serial_connected():
            return False
        if not self.position_trusted[axis]:
            self.log(f"⚠️ 轴 {AXIS_LABEL[axis]} 位置不可信；请先设置原点或校准位置")
            return False
        # 等上次脉冲完成（避免 ESP32 ERR:busy）；显式提示+超时
        if self.stepper_in_progress[axis] and not self.running[axis]:
            self.log(f"轴{AXIS_LABEL[axis]} 等待上次步进完成...")
            wait_deadline = time.time() + 10.0
            while self.stepper_in_progress[axis] and not self.running[axis]:
                if time.time() > wait_deadline:
                    self.log(f"⚠️ 轴{AXIS_LABEL[axis]} 等待超时(10s)，强制清 in_progress 标志")
                    self.stepper_in_progress[axis] = False
                    break
                time.sleep(0.02)
        delay_us = max(1, int(round(delay_ms * 1000)))
        actual_dir = direction ^ DIR_INVERT[axis]   # 按轴翻转DIR信号
        with self.state_lock:
            self._move_dispatching[axis] = True
        resp = self._send_and_read(
            f"MOVE,{axis},{steps},{actual_dir},{delay_us}", guard=guard)
        dist_mm = steps / self._pulses_per_unit(axis)
        dir_txt = "正向" if direction == DIR_OUTWARD else "反向"
        mm_s = 1000.0 / (self._pulses_per_unit(axis) * max(0.001, delay_ms))
        self.log(f"轴{AXIS_LABEL[axis]} {dist_mm:.1f}{self._unit_label(axis)} {dir_txt} @ {delay_ms:.2f}ms ({mm_s:.1f}{self._unit_per_s(axis)}) → {resp}")
        with self.state_lock:
            self._move_dispatching[axis] = False
            accepted = resp.startswith("ACK,")
            if accepted:
                sign = +1.0 if direction == DIR_OUTWARD else -1.0
                self._pending_step[axis] = sign * dist_mm
                self.stepper_in_progress[axis] = True
        if accepted:
            if 'progress' in self.sw[axis]:
                self._post_ui(lambda a=axis: self._show_step_started(a))
            return True
        return False

    def _show_step_started(self, axis):
        self.sw[axis]['progress']['value'] = 0
        self.sw[axis]['progress_label'].config(text="运动中...")

    def _send_mm(self, axis, distance_mm, direction, delay_ms, guard=None):
        steps = int(round(distance_mm * self._pulses_per_unit(axis)))
        if steps <= 0:
            self.log(f"轴{AXIS_LABEL[axis]}: 忽略距离过小 ({distance_mm})")
            return False
        sign = +1.0 if direction == DIR_OUTWARD else -1.0
        target = self.position_mm[axis] + sign * (steps / self._pulses_per_unit(axis))
        if not self._check_range(axis, target): return False
        return self._send_pulses(axis, steps, direction, delay_ms, guard=guard)

    @staticmethod
    def _executed_fraction(executed_steps, requested_steps, default):
        if executed_steps is None or requested_steps is None or requested_steps <= 0:
            return default
        return max(0.0, min(1.0, float(executed_steps) / float(requested_steps)))

    def _on_step_done(self, axis, executed_steps=None, requested_steps=None):
        if self._pending_step[axis] is not None:
            fraction = self._executed_fraction(executed_steps, requested_steps, 1.0)
            with self.state_lock:
                self.position_mm[axis] += self._pending_step[axis] * fraction
            self._pending_step[axis] = None
            self._update_pos_label(axis)
        self.stepper_in_progress[axis] = False
        if 'progress' in self.sw[axis]:
            self.sw[axis]['progress']['value'] = 100
            self.sw[axis]['progress_label'].config(text="✓ 完成")
            self.root.after(2000, lambda a=axis: self._reset_progress(a))
        self._save_calib()
        self.log(f"轴{AXIS_LABEL[axis]} 步进完成")

    def _on_step_aborted(self, axis, executed_steps=None, requested_steps=None):
        """按固件回报记入部分位移；旧协议不回报脉冲时绝不记入整段位移。"""
        fraction = self._executed_fraction(executed_steps, requested_steps, 0.0)
        partial_mm = 0.0
        if self._pending_step[axis] is not None:
            partial_mm = self._pending_step[axis] * fraction
            with self.state_lock:
                self.position_mm[axis] += partial_mm
        self._pending_step[axis] = None
        self.stepper_in_progress[axis] = False
        self.running[axis] = False
        with self.state_lock:
            self.position_trusted[axis] = False
        self._update_pos_label(axis)
        if 'progress' in self.sw[axis]:
            self.sw[axis]['progress_label'].config(text="⚠ 已中止 · 位置需重新校准")
        self._save_calib()
        detail = (f"，已按固件回报记入 {partial_mm:+.3f} mm"
                  if executed_steps is not None and requested_steps else
                  "，旧固件未回报实际脉冲，未记入待执行位移")
        self.log(f"⚠️ 轴{AXIS_LABEL[axis]} 运动已中止{detail}；软件位置已标记为不可信")

    # ═════════════ 步进：命令 ═════════════
    def send_move(self, axis):
        self._start_control_worker(
            self._send_mm, axis, self.v_dist[axis].get(), self.v_dir[axis].get(),
            self.v_delay[axis].get())

    def _quick_move(self, axis, distance_mm, direction):
        self._start_control_worker(
            self._send_mm, axis, distance_mm, direction, self.v_delay[axis].get())

    def _press_continuous(self, axis, direction):
        if self.running[axis] or not self.ser or not self.ser.is_open: return
        self.running[axis] = True
        delay_ms = self.v_delay[axis].get()
        dir_txt = "向上" if direction == DIR_OUTWARD else "向下"
        self.log(f"轴{AXIS_LABEL[axis]} 按住连续{dir_txt}")
        def worker():
            while self.running[axis]:
                while self.stepper_in_progress[axis] and self.running[axis]:
                    time.sleep(0.02)
                if not self.running[axis]: break
                if not self._send_mm(axis, CONTINUOUS_BURST_MM, direction, delay_ms):
                    break
            self.log(f"轴{AXIS_LABEL[axis]} 连续运动停止")
        if self._start_control_worker(worker) is None:
            self.running[axis] = False

    def _release_continuous(self, axis):
        if self.running[axis]: self.running[axis] = False

    def stop_continuous(self, axis):
        self.running[axis] = False
        if self.ser and self.ser.is_open:
            threading.Thread(target=lambda: self._send_and_read(f"STOP,{axis}"),
                             daemon=True).start()
        self.log(f"⛔ 轴{AXIS_LABEL[axis]} 紧急停止")

    # ═════════════ 步进：位置/原点 ═════════════
    def set_home(self, axis):
        with self.state_lock:
            self.position_mm[axis] = 0.0
            self.position_trusted[axis] = True
        self._update_pos_label(axis)
        self._save_calib()
        self.log(f"✓ 轴{AXIS_LABEL[axis]} 当前位置设为原点")

    def calibrate_position(self, axis):
        try: val = float(self.v_goto[axis].get())
        except (tk.TclError, ValueError):
            messagebox.showerror("输入无效", "请先填目标位置"); return
        with self.state_lock:
            self.position_mm[axis] = val
            self.position_trusted[axis] = True
        self._update_pos_label(axis)
        self._save_calib()
        self.log(f"✓ 轴{AXIS_LABEL[axis]} 位置校准为 {val:.1f} {self._unit_label(axis)}")

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
        if abs(delta) < 1.0 / self._pulses_per_unit(axis):
            self.log(f"轴{AXIS_LABEL[axis]} 已在目标附近"); return
        direction = DIR_OUTWARD if delta > 0 else DIR_INWARD
        distance = abs(delta)
        unit = self._unit_label(axis)
        self.log(f"轴{AXIS_LABEL[axis]} 前往 {target_mm:.1f}{unit} (移动 {distance:.1f}{unit})")
        self._start_control_worker(
            self._send_mm, axis, distance, direction, self.v_delay[axis].get())

    # ═════════════ 行程校准 ═════════════
    def _mark_min(self, axis):
        if not self.position_trusted[axis]:
            messagebox.showerror("位置不可信", "请先设置原点或校准当前位置"); return
        if self.travel_max_mm[axis] is not None and self.position_mm[axis] >= self.travel_max_mm[axis]:
            messagebox.showerror("范围无效", "最小不能 ≥ 最大"); return
        self.travel_min_mm[axis] = self.position_mm[axis]
        self._update_range_display(axis); self._save_calib()
        self.log(f"⊖ 轴{AXIS_LABEL[axis]} 最小 = {self.travel_min_mm[axis]:.1f}{self._unit_label(axis)}")

    def _mark_max(self, axis):
        if not self.position_trusted[axis]:
            messagebox.showerror("位置不可信", "请先设置原点或校准当前位置"); return
        if self.travel_min_mm[axis] is not None and self.position_mm[axis] <= self.travel_min_mm[axis]:
            messagebox.showerror("范围无效", "最大不能 ≤ 最小"); return
        self.travel_max_mm[axis] = self.position_mm[axis]
        self._update_range_display(axis); self._save_calib()
        self.log(f"⊕ 轴{AXIS_LABEL[axis]} 最大 = {self.travel_max_mm[axis]:.1f}{self._unit_label(axis)}")

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
            travel = f"  (行程 {mx-mn:.1f}{self._unit_label(axis)})" if (mn is not None and mx is not None) else ""
            label.config(text=f"min={mn_s} max={mx_s}{travel}", foreground="black")

    def _check_range(self, axis, target_mm):
        if not self.position_trusted[axis]:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]} 位置不可信，禁止运动；请先重新校准")
            return False
        mn, mx = self.travel_min_mm[axis], self.travel_max_mm[axis]
        if mn is not None and target_mm < mn - 0.05:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target_mm:.1f}{self._unit_label(axis)}<下限{mn:.1f}"); return False
        if mx is not None and target_mm > mx + 0.05:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target_mm:.1f}{self._unit_label(axis)}>上限{mx:.1f}"); return False
        return True

    def _save_calib(self):
        data = {str(a): {
                    "min": self.travel_min_mm[a],
                    "max": self.travel_max_mm[a],
                    "position": self.position_mm[a],
                    "trusted": self.position_trusted[a],
                } for a in range(NUM_STEPPER_AXES)}
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
            for a in range(NUM_STEPPER_AXES):
                d = data.get(str(a), {})
                self.travel_min_mm[a] = d.get("min")
                self.travel_max_mm[a] = d.get("max")
                if "position" in d:
                    self.position_mm[a] = float(d["position"])
                self.position_trusted[a] = bool(d.get("trusted", True))
                self._update_pos_label(a)
                self._update_range_display(a)
            any_set = any(self.travel_min_mm[a] is not None or self.travel_max_mm[a] is not None
                          for a in range(NUM_STEPPER_AXES))
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
            } for a in range(NUM_MOTOR_AXES)}
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
            for a in range(NUM_MOTOR_AXES):
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
        values = [
            (self.v_focvlimit[axis].get(), self.v_focpangle[axis].get(),
             self.v_focvp[axis].get())
            for axis in range(NUM_MOTOR_AXES)
        ]
        def worker():
            time.sleep(0.3)
            for axis, (v, pa, vp) in enumerate(values):
                self._send_and_read(f"FOC,{axis},V,{v:.1f}")
                self._send_and_read(f"FOC,{axis},PA,{pa:.1f}")
                self._send_and_read(f"FOC,{axis},VP,{vp:.2f}")
            self.log("FOC 调参已下发固件")
        self._start_control_worker(worker)

    # ═════════════ FOC：命令 ═════════════
    def _send_foc(self, axis, sub_and_arg):
        """例：_send_foc(0, 'EN,1') → 发 FOC,0,EN,1 → 日志记响应"""
        cmd = f"FOC,{axis},{sub_and_arg}"
        def worker():
            resp = self._send_and_read(cmd)
            self.log(f"{cmd} → {resp}")
        self._start_control_worker(worker)

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
    def _foc_poll_loop(self, generation):
        while (self.foc_poll_running and generation == self._serial_generation
               and self._is_serial_connected()):
            for axis in range(NUM_MOTOR_AXES):
                if not (self.foc_poll_running and generation == self._serial_generation
                        and self._is_serial_connected()):
                    break
                resp = self._send_and_read(f"FOC,{axis},S")
                prefix = f"FOC,{axis},S,"
                if resp.startswith(prefix):
                    parts = resp.split(",")
                    # 格式: FOC,<axis>,S,<state>,<cur>,<tgt>,<fault>
                    if len(parts) == 7:
                        s, c, t, f = parts[3], parts[4], parts[5], parts[6]
                        self._post_ui(lambda a=axis, s=s, c=c, t=t, f=f:
                                      self._update_foc_display(a, s, c, t, f))
                time.sleep(FOC_POLL_INTERVAL_S / NUM_MOTOR_AXES)  # 总周期仍 ~100ms

    def _update_foc_display(self, axis, state, cur, tgt, fault):
        self.v_focstate[axis].set(FOC_STATE_NAMES.get(state, "?"))
        try:
            cur_f = float(cur); tgt_f = float(tgt)
        except ValueError:
            cur_f = tgt_f = None
        with self.state_lock:
            self._motor_status[axis] = {
                "state": state,
                "current_deg": cur_f,
                "target_deg": tgt_f,
                "fault": fault == "1",
            }
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
        gw['scope'] = tk.Canvas(scf, width=420, height=280, bg="white",
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
        self._start_control_worker(worker)

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
                # 2. 使用线程安全状态快照，不从后台线程读取 Tk 变量。
                with self.state_lock:
                    motor = dict(self._motor_status[axis])
                if motor["state"] != "2":
                    self.log(f"  轴{AXIS_LABEL[axis]} watcher 退出（PID 状态={motor['state']}）")
                    return
                # 3. 读当前角度
                cur = motor["current_deg"]
                if cur is None:
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
            } for a in range(NUM_MOTOR_AXES)}
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
            for a in range(NUM_MOTOR_AXES):
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
        values = [
            (self.v_gearpwm[axis].get(), self.v_gearkp[axis].get(),
             self.v_gearki[axis].get(), self.v_gearkd[axis].get())
            for axis in range(NUM_MOTOR_AXES)
        ]
        def worker():
            time.sleep(0.3)
            for axis, (pwm, kp, ki, kd) in enumerate(values):
                self._send_and_read(f"FOC,{axis},V,{pwm:.1f}")
                self._send_and_read(f"FOC,{axis},PA,{kp:.2f}")
                self._send_and_read(f"FOC,{axis},PI,{ki:.2f}")
                self._send_and_read(f"FOC,{axis},PD,{kd:.3f}")
            self.log("GEAR 调参已下发固件")
        self._start_control_worker(worker)

    # ═════════════ 模式自动检测 + tab 灰显 ═════════════
    def _query_mode_and_apply(self):
        """连接成功后调一次。用户选 Auto 时发 MODE 命令自动检测；
        用户选 FOC/GEAR 时直接强制应用，不查询固件。"""
        sel = self.mode_select_var.get()
        if sel in (MODE_FOC, MODE_GEAR):
            self.log(f"模式选择 = 强制 {sel}（跳过固件 MODE 查询）")
            self._post_ui(lambda: self._apply_mode_to_tabs(sel))
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
            self._post_ui(lambda: self._apply_mode_to_tabs(mode))
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
        with self.state_lock:
            self._fw_mode_cache = mode
        if mode == MODE_FOC:
            self.mode_label.config(foreground="#1565c0")
            for a in range(NUM_MOTOR_AXES):
                self.notebook.tab(self.tab_index_foc[a],  state="normal")
                self.notebook.tab(self.tab_index_gear[a], state="disabled")
            # 自动跳到第一个 FOC tab
            self.notebook.select(self.tab_index_foc[0])
            self._apply_foc_tune_to_firmware()
        elif mode == MODE_GEAR:
            self.mode_label.config(foreground="#2e7d32")
            for a in range(NUM_MOTOR_AXES):
                self.notebook.tab(self.tab_index_foc[a],  state="disabled")
                self.notebook.tab(self.tab_index_gear[a], state="normal")
            self.notebook.select(self.tab_index_gear[0])
            self._apply_gear_tune_to_firmware()
        self.log(f"固件模式 = {mode}")

    # ═════════════ 网页服务公开接口（允许从非 Tk 线程调用）═════════════
    @staticmethod
    def _lan_ipv4():
        """尽量取得手机可访问的局域网 IPv4；失败时安全回退到本机地址。"""
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("8.8.8.8", 80))
            address = probe.getsockname()[0]
            if address and not address.startswith("127."):
                return address
        except OSError:
            pass
        finally:
            probe.close()
        try:
            address = socket.gethostbyname(socket.gethostname())
            return address if address else "127.0.0.1"
        except OSError:
            return "127.0.0.1"

    def _start_web_server(self, factory):
        """启动随 GUI 生命周期运行的局域网网页服务。"""
        try:
            server = factory(self)
            server.start()
            self.attach_web_server(server)
            port = server.bound_port
            if port is None:
                raise RuntimeError("网页服务未取得监听端口")
            address = self._lan_ipv4()
            url = f"http://{address}:{port}/"
            self.web_address_var.set(address)
            self.web_port_var.set(str(port))
            self.web_url_var.set(url)
            if address.startswith("127."):
                self.web_status_var.set("● 已启动（仅检测到本机地址）")
                self.web_status_label.configure(foreground="#b26a00")
                self.log("⚠️ 网页服务已启动，但只检测到本机地址；请联网后重启 GUI")
            else:
                self.web_status_var.set("● 服务已启动")
                self.web_status_label.configure(foreground="#16803a")
                self.log(f"网页服务已启动：{url}（固定地址，无控制令牌）")
        except Exception as exc:
            self.web_server = None
            self.web_status_var.set("● 服务启动失败")
            self.web_address_var.set("—")
            self.web_port_var.set("—")
            self.web_url_var.set(f"网页服务启动失败：{exc}")
            self.web_status_label.configure(foreground="#b42318")
            self.log(f"⚠️ 网页服务启动失败：{exc}")

    def _copy_web_url(self):
        url = self.web_url_var.get()
        if not url.startswith("http://"):
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(url)
        self.root.update_idletasks()
        self.log("手机控制固定网址已复制")

    def _open_web_url(self):
        url = self.web_url_var.get()
        if url.startswith("http://"):
            webbrowser.open(url)

    def _on_close(self):
        """尽力停车，然后回收串口和网页服务器。"""
        with self.state_lock:
            if self._closing:
                return
            self._closing = True
            active_axes = [
                axis for axis in range(NUM_STEPPER_AXES)
                if (self.running[axis] or self.stepper_in_progress[axis]
                    or self._move_dispatching[axis]
                    or self._pending_step[axis] is not None)
            ]
        # 先关闭 HTTP 入口；_send_and_read 的 closing guard 同时封锁已排队的普通命令。
        if self.web_server is not None:
            try:
                self.web_server.stop()
            except Exception:
                pass
            self.web_server = None
        if self._is_serial_connected():
            self._stop_all_outputs(allow_closing=True)
            self._disconnect_serial("GUI 已关闭", is_error=False)
        # _disconnect_serial 的 UI 更新来不及在 destroy 前执行；关闭路径必须同步持久化失信状态。
        with self.state_lock:
            for axis in active_axes:
                self.position_trusted[axis] = False
                self.running[axis] = False
                self.stepper_in_progress[axis] = False
                self._move_dispatching[axis] = False
                self._pending_step[axis] = None
        self._save_calib()
        self.root.destroy()

    def _ensure_web_control_available(self):
        with self.state_lock:
            if self._closing:
                raise RuntimeError("控制器正在关闭")
            if self._disconnecting:
                raise RuntimeError("串口正在断开")
            if self._estop_in_progress:
                raise RuntimeError("软件急停正在执行")
            generation = self._control_generation
        if not self._is_serial_connected():
            raise RuntimeError("串口未连接")
        return generation

    def _stop_all_outputs(self, allow_closing=False):
        """发送固件原子软件急停；返回是否收到精确确认。"""
        self._track_lease_generation += 1
        for axis in range(NUM_STEPPER_AXES):
            self.running[axis] = False
        with self.state_lock:
            self._track_direction = "STOP"
        response = self._send_and_read("ESTOP", timeout=0.8, allow_closing=allow_closing)
        self._post_ui(lambda: self.v_track_status.set("已停止"))
        return response == "OK,ESTOP"

    def attach_web_server(self, server):
        """保存 WebControlServer 实例，供外部启动/关闭流程统一管理。"""
        self.web_server = server
        return server

    def web_get_status(self):
        """返回只含 JSON 基础类型的线程安全快照，不读取任何 Tk 变量。"""
        with self.state_lock:
            steppers = [
                {
                    "axis": axis,
                    "label": AXIS_LABEL[axis],
                    "position_mm": self.position_mm[axis],
                    "position_trusted": self.position_trusted[axis],
                    "in_progress": (self.stepper_in_progress[axis]
                                    or self._move_dispatching[axis]
                                    or self._web_step_pending[axis]),
                    "continuous": self.running[axis],
                    "travel_min_mm": self.travel_min_mm[axis],
                    "travel_max_mm": self.travel_max_mm[axis],
                    "mode": self.axis_mode[axis],
                    "unit": "°" if self.axis_mode[axis] == MODE_ROTARY else "mm",
                    "pulse_per_rev": self.axis_pulse_per_rev[axis],
                    "gear_ratio": self.axis_gear_ratio[axis],
                    "lead_mm": self.axis_lead_mm[axis],
                }
                for axis in range(NUM_STEPPER_AXES)
            ]
            motors = [dict(axis=axis, label=AXIS_LABEL[axis], **self._motor_status[axis])
                      for axis in range(NUM_MOTOR_AXES)]
            track = {
                "axis": "D",
                "direction": self._track_direction,
                "last_response": self._track_last_response,
                "lease_ms": self._track_lease_ms,
            }
            mode = self._fw_mode_cache
        return {
            "connected": self._is_serial_connected(),
            "mode": mode,
            "steppers": [
                (f"{item['position_mm']:.1f} {item['unit']}"
                 if item["position_trusted"] else "需重新校准")
                + (" · 运行中" if item["in_progress"] else "")
                for item in steppers
            ],
            "motors": [
                (f"{item['current_deg']:.1f}°" if item["current_deg"] is not None else "--")
                + (" · 故障" if item["fault"] else "")
                for item in motors
            ],
            "track": track["direction"],
            "stepper_axes": steppers,
            "motor_axes": motors,
            "track_detail": track,
        }

    @staticmethod
    def _web_direction(direction):
        if isinstance(direction, str):
            value = direction.strip().upper()
            if value in ("FWD", "FORWARD", "OUT", "OUTWARD", "UP", "+"):
                return DIR_OUTWARD
            if value in ("REV", "REVERSE", "IN", "INWARD", "DOWN", "-"):
                return DIR_INWARD
        elif direction in (DIR_OUTWARD, DIR_INWARD):
            return int(direction)
        raise ValueError("direction 必须是 FWD/REV（或 1/0）")

    def web_stepper_move(self, axis, direction, distance_mm, speed_mm_s=SPEED_DEFAULT):
        """异步移动步进轴；返回是否已接纳，不在 Web 请求线程中触碰 Tk。"""
        generation = self._ensure_web_control_available()
        try:
            axis = int(axis)
            distance_mm = float(distance_mm)
            speed_mm_s = float(speed_mm_s)
            direction = self._web_direction(direction)
        except (TypeError, ValueError) as exc:
            raise ValueError(str(exc)) from exc
        if not 0 <= axis < NUM_STEPPER_AXES:
            raise ValueError("步进轴编号越界")
        if distance_mm <= 0 or speed_mm_s <= 0:
            raise ValueError("距离和速度必须大于 0")
        with self.state_lock:
            if not self.position_trusted[axis]:
                raise RuntimeError("位置不可信，请在 GUI 中重新校准")
            if (self.stepper_in_progress[axis] or self._move_dispatching[axis]
                    or self._web_step_pending[axis]):
                raise RuntimeError("该轴正在运动")
            sign = 1.0 if direction == DIR_OUTWARD else -1.0
            target = self.position_mm[axis] + sign * distance_mm
            mn, mx = self.travel_min_mm[axis], self.travel_max_mm[axis]
            if mn is not None and target < mn - 0.05:
                raise RuntimeError("目标超出软件下限")
            if mx is not None and target > mx + 0.05:
                raise RuntimeError("目标超出软件上限")
            self._web_step_pending[axis] = True
        delay_ms = _speed_to_delay_ms(speed_mm_s, self._pulses_per_unit(axis))

        def worker():
            try:
                self._send_mm(
                    axis, distance_mm, direction, delay_ms,
                    guard=lambda: generation == self._control_generation,
                )
            finally:
                with self.state_lock:
                    self._web_step_pending[axis] = False

        threading.Thread(target=worker, daemon=True).start()
        return {"ok": True, "accepted": True, "axis": axis}

    def web_stepper_stop(self, axis):
        generation = self._ensure_web_control_available()
        try:
            axis = int(axis)
        except (TypeError, ValueError):
            raise ValueError("无效步进轴编号")
        if not 0 <= axis < NUM_STEPPER_AXES:
            raise ValueError("步进轴编号越界")
        self.running[axis] = False
        response = self._send_and_read(
            f"STOP,{axis}", timeout=0.6,
            guard=lambda: generation == self._control_generation,
        )
        if not response:
            raise RuntimeError("步进停止命令未获固件确认")
        return {"ok": True, "confirmed": True, "axis": axis}

    def web_stepper_config(self, axis, mode=None, pulse_per_rev=None, gear_ratio=None, lead_mm=None):
        """网页切换步进轴模式或参数；在 UI 线程改 Tk 变量并触发持久化。"""
        try:
            axis = int(axis)
        except (TypeError, ValueError):
            raise ValueError("axis 必须是整数")
        if not 0 <= axis < NUM_STEPPER_AXES:
            raise ValueError("步进轴编号越界")
        if mode is not None and mode not in (MODE_LINEAR, MODE_ROTARY):
            raise ValueError("mode 必须是 linear 或 rotary")

        def ui():
            if mode is not None:
                self.axis_mode_var[axis].set(mode)
                self._on_axis_mode_change(axis)
            if pulse_per_rev is not None:
                self.axis_ppr_var[axis].set(float(pulse_per_rev))
            if gear_ratio is not None:
                self.axis_gr_var[axis].set(float(gear_ratio))
            if lead_mm is not None:
                self.axis_lead_var[axis].set(float(lead_mm))
            if pulse_per_rev is not None or gear_ratio is not None or lead_mm is not None:
                self._on_axis_param_change(axis)
        self._post_ui(ui)
        return {"ok": True, "axis": axis}

    def web_motor_command(self, mode, axis, action, target_deg=None):
        """控制两路闭环电机；签名与 WebControlServer 的 HTTP 路由一致。"""
        generation = self._ensure_web_control_available()
        mode = str(mode).strip().upper()
        if mode not in (MODE_FOC, MODE_GEAR):
            raise ValueError("模式必须是 FOC 或 GEAR")
        action = str(action).strip().lower()
        with self.state_lock:
            active_mode = self._fw_mode_cache
        # 失能属于安全动作，即使网页下拉框与当前固件模式不一致也允许发送。
        if (action != "disable" and active_mode in (MODE_FOC, MODE_GEAR)
                and mode != active_mode):
            raise RuntimeError(f"当前固件模式是 {active_mode}，不能按 {mode} 控制")
        try:
            axis = int(axis)
        except (TypeError, ValueError):
            raise ValueError("无效电机轴编号")
        if not 0 <= axis < NUM_MOTOR_AXES:
            raise ValueError("电机轴编号越界")
        if action == "enable":
            subcommand = "EN,1"
        elif action == "disable":
            subcommand = "EN,0"
        elif action == "zero":
            subcommand = "H"
        elif action == "target":
            try:
                amount = float(target_deg)
            except (TypeError, ValueError):
                raise ValueError("target 动作需要有效角度")
            subcommand = f"A,{amount:.1f}"
        else:
            raise ValueError("动作必须是 enable/disable/zero/target")
        cmd = f"FOC,{axis},{subcommand}"
        response = self._send_and_read(
            cmd, timeout=0.8,
            guard=lambda: generation == self._control_generation,
        )
        if not response:
            raise RuntimeError("闭环电机命令未获固件确认")
        return {"ok": True, "confirmed": True, "mode": mode,
                "axis": axis, "action": action}

    def web_track_command(self, action, pwm=0, lease_ms=0):
        """网页轨道 D 命令；运行命令始终带固件租约，客户端失联会自动停车。"""
        control_generation = self._ensure_web_control_available()
        action = str(action).strip().lower()
        if action == "stop":
            self._track_lease_generation += 1
            generation = self._track_lease_generation
            with self.state_lock:
                self._track_direction = "STOP"
            response = self._send_track_stop(
                generation,
                control_guard=lambda: control_generation == self._control_generation,
            )
            if not response:
                raise RuntimeError("轨道停止命令未获固件确认")
            self._post_ui(lambda: self.v_track_status.set("已停止（网页）"))
            return {"ok": True, "confirmed": True, "action": "stop"}
        if action not in ("forward", "reverse"):
            raise ValueError("动作必须是 forward/reverse/stop")
        direction = "FWD" if action == "forward" else "REV"
        try:
            pwm = max(1, min(100, int(pwm)))
            lease_ms = int(lease_ms) if int(lease_ms) > 0 else 1000
            lease_ms = max(100, min(5000, lease_ms))
        except (TypeError, ValueError):
            raise ValueError("PWM/租约参数无效")
        self._track_lease_generation += 1
        generation = self._track_lease_generation
        with self.state_lock:
            self._track_direction = direction
            self._track_lease_ms = lease_ms

        def worker():
            response = self._send_and_read(
                f"TRACK,D,{direction},{pwm},{lease_ms}",
                timeout=0.25,
                guard=lambda: (generation == self._track_lease_generation
                               and self._track_direction == direction
                               and control_generation == self._control_generation),
            )
            with self.state_lock:
                self._track_last_response = response

        threading.Thread(target=worker, daemon=True).start()
        self._post_ui(lambda: self.v_track_status.set(
            f"{direction} · PWM {pwm}% · 网页租约 {lease_ms}ms"))

        def expire_cached_state():
            if generation != self._track_lease_generation:
                return
            with self.state_lock:
                if self._track_direction == direction:
                    self._track_direction = "STOP"
            self._post_ui(lambda: self.v_track_status.set("网页租约到期，已停止"))

        timer = threading.Timer(lease_ms / 1000.0 + 0.1, expire_cached_state)
        timer.daemon = True
        timer.start()
        return {"ok": True, "accepted": True, "action": action,
                "pwm": pwm, "lease_ms": lease_ms}

    def web_emergency_stop(self):
        """单个 HTTP 请求触发固件 ESTOP，并等待精确确认。"""
        with self._estop_lock:
            self._ensure_web_control_available()
            with self.state_lock:
                self._estop_in_progress = True
                self._control_generation += 1
            try:
                if not self._stop_all_outputs(allow_closing=True):
                    raise RuntimeError("急停命令未获得固件确认；请立即使用物理断电急停")
                return {"ok": True, "confirmed": True}
            finally:
                with self.state_lock:
                    self._estop_in_progress = False

    # ═════════════ FOC：自动调参（单轴）═════════════
    def _foc_autotune(self, axis):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("未连接", "请先连接串口"); return
        if not messagebox.askokcancel(
            "自动调参确认",
            f"轴 {AXIS_LABEL[axis]} 两阶段扫描 PA + VP，约 2 分钟。\n电机会来回转动，请先固定好。"):
            return
        self.fw[axis]['autotune_btn'].config(state="disabled")
        if self._start_control_worker(self._foc_autotune_worker, axis) is None:
            self.fw[axis]['autotune_btn'].config(state="normal")

    def _step_response_test(self, axis, target, pre_settle=2.5, duration=5.0):
        self._send_and_read(f"FOC,{axis},A,0"); time.sleep(pre_settle)
        t0 = time.time()
        self._send_and_read(f"FOC,{axis},A,{target}"); time.sleep(duration)
        data = [(t - t0, cur) for t, tgt, cur in self.foc_trace_buf[axis]
                if t >= t0 and abs(tgt - target) < 0.5]
        if len(data) < 5: return None
        curs = [c for _, c in data]
        overshoot = max(0.0, max(curs) - target) if target > 0 else max(0.0, target - min(curs))
        rt = None
        thr = target * 0.9
        for t, c in data:
            if c >= thr: rt = t; break
        # 稳态误差：用最后 2 秒的平均值（比取末点更稳定）
        tail = [c for t, c in data if t >= duration - 2.0]
        if len(tail) >= 3:
            tail_avg = sum(tail) / len(tail)
            ss_error = abs(tail_avg - target)
            jit = (sum((x - tail_avg)**2 for x in tail) / len(tail)) ** 0.5
        else:
            ss_error = abs(curs[-1] - target)
            jit = 0.0
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
            def apply_result():
                self.v_focpangle[axis].set(float(best_pa))
                self.fw[axis]['pangle_label'].config(text=f"{best_pa:.1f} (刚度)")
                self.v_focvp[axis].set(float(best_vp))
                self.fw[axis]['vp_label'].config(text=f"{best_vp:.2f} (阻尼)")
                self._save_foc_tune()
            self._post_ui(apply_result)
        finally:
            self._post_ui(lambda: self.fw[axis]['autotune_btn'].config(state="normal"))

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
        if self._start_control_worker(self._gear_autotune_worker, axis) is None:
            self.gw[axis]['autotune_btn'].config(state="normal")

    def _gear_autotune_worker(self, axis):
        TARGET = 60.0
        DUR = 8.0   # 加长到 8 秒，确保含 Ki 时充分收敛
        try:
            self.log(f"🤖 轴{AXIS_LABEL[axis]} GEAR 自动调参开始 (目标 ±{TARGET:.0f}°)")
            # 重置到已知状态
            self._send_and_read(f"FOC,{axis},V,100");  time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},PI,0");   time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},PD,0.5"); time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},PA,5");   time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},EN,1");   time.sleep(0.5)
            self._send_and_read(f"FOC,{axis},H");      time.sleep(0.3)

            # ── 阶段 1: Kp 扫描 ──
            self.log(f"--- 阶段 1/4: Kp 扫描 ---")
            kp_results = []
            for kp in [5, 10, 15, 20, 25, 30]:
                self._send_and_read(f"FOC,{axis},PA,{kp}"); time.sleep(0.3)
                m = self._step_response_test(axis, TARGET, pre_settle=2.0, duration=DUR)
                if m is None:
                    self.log(f"  Kp={kp}: 无数据"); continue
                ov, sse, rt, jt = m
                self.log(f"  Kp={kp}: 过冲={ov:.1f}° 稳态误差={sse:.1f}° "
                         f"上升={rt:.2f}s 抖={jt:.2f}°")
                kp_results.append((kp, ov, sse, rt, jt))
                if ov > 40:
                    self.log(f"  ⚠️ 过冲太大，停止"); break
            if not kp_results:
                self.log("❌ Kp 阶段无有效数据，中止"); return
            # 评分：稳态误差权重最高，其次过冲
            good = [r for r in kp_results if r[1] <= 15.0]
            pool = good if good else kp_results
            best_kp = min(pool, key=lambda r: r[2]*5 + r[1]*3 + r[3]*2 + r[4]*2)[0]
            self.log(f"→ 选 Kp = {best_kp}")
            self._send_and_read(f"FOC,{axis},PA,{best_kp}"); time.sleep(0.3)

            # ── 阶段 2: Kd 扫描（抑制过冲）──
            self.log(f"--- 阶段 2/4: Kd 扫描 ---")
            kd_results = []
            for kd in [0.3, 0.5, 0.8, 1.0, 1.5, 2.0]:
                self._send_and_read(f"FOC,{axis},PD,{kd}"); time.sleep(0.3)
                m = self._step_response_test(axis, TARGET, pre_settle=2.0, duration=DUR)
                if m is None: continue
                ov, sse, rt, jt = m
                self.log(f"  Kd={kd:.1f}: 过冲={ov:.1f}° 稳态误差={sse:.1f}° 抖={jt:.2f}°")
                kd_results.append((kd, ov, sse, rt, jt))
                if jt > 5.0:
                    self.log(f"  ⚠️ 抖动太大，停止"); break
            best_kd = 0.5
            if kd_results:
                good = [r for r in kd_results if r[1] <= 5.0 and r[4] <= 2.0]
                pool = good if good else kd_results
                best_kd = min(pool, key=lambda r: r[1]*4 + r[4]*5 + r[2]*3)[0]
            self.log(f"→ 选 Kd = {best_kd:.1f}")
            self._send_and_read(f"FOC,{axis},PD,{best_kd}"); time.sleep(0.3)

            # ── 阶段 3: Ki 扫描（小值范围，消稳态误差到 ≤1°）──
            self.log(f"--- 阶段 3/4: Ki 扫描 ---")
            ki_results = []
            for ki in [0.02, 0.05, 0.1, 0.15, 0.2, 0.3]:
                self._send_and_read(f"FOC,{axis},PI,{ki}"); time.sleep(0.3)
                m = self._step_response_test(axis, TARGET, pre_settle=2.5, duration=DUR)
                if m is None: continue
                ov, sse, rt, jt = m
                self.log(f"  Ki={ki:.2f}: 过冲={ov:.1f}° 稳态误差={sse:.1f}° 抖={jt:.2f}°")
                ki_results.append((ki, ov, sse, rt, jt))
                if ov > 15 or jt > 3.0:
                    self.log(f"  ⚠️ Ki={ki} 失稳，停止"); break
            best_ki = 0.0
            if ki_results:
                # 优先：稳态误差 ≤1°，同时过冲 ≤8°
                good = [r for r in ki_results if r[2] <= 1.0 and r[1] <= 8.0]
                pool = good if good else ki_results
                best_ki = min(pool, key=lambda r: r[2]*10 + r[1]*3 + r[4]*3)[0]
            self.log(f"→ 选 Ki = {best_ki:.2f}")
            self._send_and_read(f"FOC,{axis},PI,{best_ki}"); time.sleep(0.3)

            # ── 阶段 4: 验证最终参数 ──
            self.log(f"--- 阶段 4/4: 验证 ---")
            self._send_and_read(f"FOC,{axis},PA,{best_kp}"); time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},PD,{best_kd}"); time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},PI,{best_ki}"); time.sleep(0.1)
            self._send_and_read(f"FOC,{axis},H"); time.sleep(1.0)
            m = self._step_response_test(axis, TARGET, pre_settle=3.0, duration=DUR)
            if m:
                ov, sse, rt, jt = m
                self.log(f"  验证结果: 过冲={ov:.1f}° 稳态误差={sse:.1f}° 上升={rt:.2f}s 抖={jt:.2f}°")
                if sse <= 1.0:
                    self.log(f"✅ 稳态误差 ≤ 1°，达标！")
                else:
                    self.log(f"⚠️ 稳态误差 {sse:.1f}° > 1°，建议手动微调 Ki")
            else:
                self.log("  验证: 无数据")

            # 收尾
            self._send_and_read(f"FOC,{axis},A,0"); time.sleep(2.0)
            self.log(f"✅ 轴{AXIS_LABEL[axis]} 推荐：Kp={best_kp}  Kd={best_kd:.1f}  Ki={best_ki:.2f}")
            def apply_result():
                self.v_gearkp[axis].set(float(best_kp))
                self.gw[axis]['kp_label'].config(text=f"{best_kp:.2f}")
                self.v_gearkd[axis].set(float(best_kd))
                self.gw[axis]['kd_label'].config(text=f"{best_kd:.3f}")
                self.v_gearki[axis].set(float(best_ki))
                self.gw[axis]['ki_label'].config(text=f"{best_ki:.2f}")
                self._save_gear_tune()
            self._post_ui(apply_result)
        finally:
            self._post_ui(lambda: self.gw[axis]['autotune_btn'].config(state="normal"))

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
        self._post_ui(_append)

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
    from web_control import WebControlServer

    root = tk.Tk()
    app = StepperGUI(root, lambda controller: WebControlServer(controller))
    root.mainloop()
