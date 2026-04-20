import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

# ============== 步进标定常数 ==============
PULSES_PER_MM = 50       # 根据实测修正：GUI 设 50mm 实际走 5mm → 需 10× 脉冲数

DIR_OUTWARD = 0          # direction=0 对应物理"向外"
DIR_INWARD  = 1          # direction=1 对应物理"向内/回退"

CONTINUOUS_BURST_MM = 0.2  # 按住连续运动的每次脉冲包

# ============== FOC 常数 ==============
FOC_STATE_NAMES = {"0": "失能", "1": "对齐中", "2": "运行", "3": "故障"}
FOC_POLL_INTERVAL_S = 0.1


class StepperGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("步进 + FOC 电机控制器")
        self.root.resizable(False, False)
        self.ser = None
        self.serial_lock = threading.Lock()
        self.running = False
        self.position_mm = 0.0
        self.stepper_in_progress = False
        self._foc_enabled_ui = False
        self.foc_poll_running = False
        self.foc_trace_buf = []  # [(t, target_deg, cur_deg)]，示波器数据
        self._build_ui()

    # ═════════════ 顶层 UI ═════════════
    def _build_ui(self):
        pad = dict(padx=10, pady=5)

        # ── 串口连接（共用）──
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

        # ── Notebook 标签页 ──
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)

        self.stepper_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.stepper_tab, text="🔩 步进")
        self._build_stepper_tab(self.stepper_tab)

        self.foc_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.foc_tab, text="🧲 FOC 无刷")
        self._build_foc_tab(self.foc_tab)

        # ── 日志（共用，置底）──
        log_frame = ttk.LabelFrame(self.root, text="日志")
        log_frame.grid(row=2, column=0, sticky="ew", **pad)
        self.log_text = tk.Text(log_frame, height=8, width=80, state="disabled", font=("Consolas", 9))
        self.log_text.pack(side="left", fill="both", padx=5, pady=5)
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)
        ttk.Button(self.root, text="清空日志", command=self.clear_log).grid(
            row=3, column=0, sticky="e", padx=10, pady=2)

        self.refresh_ports()

    # ═════════════ 步进 Tab ═════════════
    def _build_stepper_tab(self, parent):
        pad = dict(padx=10, pady=5)

        # 运动参数
        param_frame = ttk.LabelFrame(parent, text="运动参数")
        param_frame.grid(row=0, column=0, sticky="nsew", **pad)

        ttk.Label(param_frame, text="距离 (mm):").grid(row=0, column=0, sticky="w", **pad)
        self.distance_var = tk.DoubleVar(value=10.0)
        ttk.Spinbox(param_frame, from_=0.2, to=500.0, increment=1.0,
                    textvariable=self.distance_var, width=10, format="%.1f").grid(row=0, column=1, **pad)

        ttk.Label(param_frame, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
        qbtn_frame = ttk.Frame(param_frame)
        qbtn_frame.grid(row=1, column=1, sticky="w", pady=5)
        for label, mm in [("1mm", 1), ("10mm", 10), ("50mm", 50), ("100mm", 100)]:
            ttk.Button(qbtn_frame, text=label, width=6,
                       command=lambda d=mm: self.distance_var.set(d)).pack(side="left", padx=2)

        ttk.Label(param_frame, text="方向:").grid(row=2, column=0, sticky="w", **pad)
        self.dir_var = tk.IntVar(value=DIR_OUTWARD)
        dir_frame = ttk.Frame(param_frame)
        dir_frame.grid(row=2, column=1, sticky="w")
        ttk.Radiobutton(dir_frame, text="向外 ▶", variable=self.dir_var, value=DIR_OUTWARD).pack(side="left", padx=4)
        ttk.Radiobutton(dir_frame, text="◀ 向内", variable=self.dir_var, value=DIR_INWARD).pack(side="left", padx=4)

        ttk.Label(param_frame, text="速度:").grid(row=3, column=0, sticky="w", **pad)
        speed_frame = ttk.Frame(param_frame)
        speed_frame.grid(row=3, column=1, sticky="w", pady=5)
        self.delay_var = tk.IntVar(value=20)
        self.speed_slider = ttk.Scale(speed_frame, from_=2, to=100, orient="horizontal",
                                      variable=self.delay_var, length=160,
                                      command=lambda v: self.delay_var.set(int(float(v))))
        self.speed_slider.pack(side="left")
        self.delay_label = ttk.Label(speed_frame, text="", width=22)
        self.delay_label.pack(side="left", padx=6)
        self.delay_var.trace_add("write", self._update_speed_label)
        self._update_speed_label()

        # 控制按钮
        ctrl_frame = ttk.LabelFrame(parent, text="控制")
        ctrl_frame.grid(row=0, column=1, sticky="nsew", **pad)

        self.move_btn = ttk.Button(ctrl_frame, text="执行运动", command=self.send_move, state="disabled")
        self.move_btn.grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=8)

        self.jog_out_btn = ttk.Button(ctrl_frame, text="向外 1mm ▶",
                                      command=lambda: self._quick_move(1.0, DIR_OUTWARD), state="disabled")
        self.jog_out_btn.grid(row=1, column=0, **pad)
        self.jog_in_btn = ttk.Button(ctrl_frame, text="◀ 向内 1mm",
                                     command=lambda: self._quick_move(1.0, DIR_INWARD), state="disabled")
        self.jog_in_btn.grid(row=1, column=1, **pad)

        self.cont_out_btn = ttk.Button(ctrl_frame, text="向外 (按住) ▶▶", state="disabled")
        self.cont_out_btn.grid(row=2, column=0, **pad)
        self.cont_out_btn.bind("<ButtonPress-1>",   lambda e: self._press_continuous(DIR_OUTWARD))
        self.cont_out_btn.bind("<ButtonRelease-1>", lambda e: self._release_continuous())
        self.cont_out_btn.bind("<Leave>",           lambda e: self._release_continuous())

        self.cont_in_btn = ttk.Button(ctrl_frame, text="◀◀ 向内 (按住)", state="disabled")
        self.cont_in_btn.grid(row=2, column=1, **pad)
        self.cont_in_btn.bind("<ButtonPress-1>",   lambda e: self._press_continuous(DIR_INWARD))
        self.cont_in_btn.bind("<ButtonRelease-1>", lambda e: self._release_continuous())
        self.cont_in_btn.bind("<Leave>",           lambda e: self._release_continuous())

        self.stop_btn = ttk.Button(ctrl_frame, text="■ 紧急停止", command=self.stop_continuous, state="disabled")
        self.stop_btn.grid(row=3, column=0, columnspan=2, **pad, ipadx=10)

        # 位置与原点
        pos_frame = ttk.LabelFrame(parent, text="位置与原点（软件跟踪）")
        pos_frame.grid(row=1, column=0, columnspan=2, sticky="ew", **pad)

        ttk.Label(pos_frame, text="当前位置:").grid(row=0, column=0, sticky="w", **pad)
        self.pos_label = ttk.Label(pos_frame, text="0.0 mm",
                                   font=("Consolas", 14, "bold"), foreground="blue")
        self.pos_label.grid(row=0, column=1, sticky="w", **pad)

        self.set_home_btn = ttk.Button(pos_frame, text="⌂ 设为原点 (0 mm)",
                                       command=self.set_home, state="disabled")
        self.set_home_btn.grid(row=0, column=2, **pad)
        self.go_home_btn = ttk.Button(pos_frame, text="⟲ 回到原点",
                                      command=self.go_home, state="disabled")
        self.go_home_btn.grid(row=0, column=3, **pad)

        ttk.Label(pos_frame, text="前往位置 (mm):").grid(row=1, column=0, sticky="w", **pad)
        self.target_var = tk.DoubleVar(value=0.0)
        ttk.Spinbox(pos_frame, from_=-1000.0, to=1000.0, increment=1.0,
                    textvariable=self.target_var, width=10, format="%.1f").grid(row=1, column=1, **pad)
        self.goto_btn = ttk.Button(pos_frame, text="前往", command=self.goto_target_position, state="disabled")
        self.goto_btn.grid(row=1, column=2, **pad)
        self.calib_btn = ttk.Button(pos_frame, text="把当前位置校准为此值",
                                    command=self.calibrate_position, state="disabled")
        self.calib_btn.grid(row=1, column=3, **pad)

    # ═════════════ FOC Tab ═════════════
    def _build_foc_tab(self, parent):
        pad = dict(padx=10, pady=5)

        # 状态区
        state_frame = ttk.LabelFrame(parent, text="状态（实时轮询 100ms）")
        state_frame.grid(row=0, column=0, sticky="ew", **pad)

        ttk.Label(state_frame, text="状态:").grid(row=0, column=0, sticky="w", **pad)
        self.foc_state_var = tk.StringVar(value="未连接")
        ttk.Label(state_frame, textvariable=self.foc_state_var, width=10,
                  font=("Consolas", 11, "bold"), foreground="gray").grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(state_frame, text="故障:").grid(row=0, column=2, sticky="w", **pad)
        self.foc_fault_var = tk.StringVar(value="--")
        self.foc_fault_label = ttk.Label(state_frame, textvariable=self.foc_fault_var, width=6,
                                         font=("Consolas", 11, "bold"), foreground="gray")
        self.foc_fault_label.grid(row=0, column=3, sticky="w", **pad)

        ttk.Label(state_frame, text="当前角度:").grid(row=1, column=0, sticky="w", **pad)
        self.foc_current_var = tk.StringVar(value="--")
        ttk.Label(state_frame, textvariable=self.foc_current_var, width=16,
                  font=("Consolas", 14, "bold"), foreground="blue").grid(row=1, column=1, columnspan=3, sticky="w", **pad)

        # 目标控制
        tgt_frame = ttk.LabelFrame(parent, text="目标控制")
        tgt_frame.grid(row=1, column=0, sticky="ew", **pad)

        ttk.Label(tgt_frame, text="目标角度 (°):").grid(row=0, column=0, sticky="w", **pad)
        self.foc_target_var = tk.DoubleVar(value=0.0)
        ttk.Spinbox(tgt_frame, from_=-3600.0, to=3600.0, increment=1.0,
                    textvariable=self.foc_target_var, width=10, format="%.1f").grid(row=0, column=1, **pad)
        self.foc_goto_btn = ttk.Button(tgt_frame, text="前往", command=self._foc_goto, state="disabled")
        self.foc_goto_btn.grid(row=0, column=2, **pad)

        ttk.Label(tgt_frame, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
        quick_frame = ttk.Frame(tgt_frame)
        quick_frame.grid(row=1, column=1, columnspan=2, sticky="w", pady=5)
        self._foc_quick_btns = []
        for deg in (0, 45, 90, 180, 270):
            b = ttk.Button(quick_frame, text=f"{deg}°", width=5,
                           command=lambda d=deg: self._foc_quick(float(d)), state="disabled")
            b.pack(side="left", padx=2)
            self._foc_quick_btns.append(b)

        ttk.Label(tgt_frame, text="增量:").grid(row=2, column=0, sticky="w", **pad)
        inc_frame = ttk.Frame(tgt_frame)
        inc_frame.grid(row=2, column=1, columnspan=2, sticky="w", pady=5)
        self._foc_inc_btns = []
        for delta in (-10, -1, +1, +10):
            b = ttk.Button(inc_frame, text=f"{delta:+d}°", width=5,
                           command=lambda d=float(delta): self._foc_increment(d), state="disabled")
            b.pack(side="left", padx=2)
            self._foc_inc_btns.append(b)

        # 原点
        home_frame = ttk.LabelFrame(parent, text="原点")
        home_frame.grid(row=2, column=0, sticky="ew", **pad)
        self.foc_home_btn = ttk.Button(home_frame, text="⌂ 把当前位置设为 0°",
                                       command=self._foc_home, state="disabled")
        self.foc_home_btn.grid(row=0, column=0, **pad)

        # 使能 / 调试
        en_frame = ttk.LabelFrame(parent, text="使能 / 调试")
        en_frame.grid(row=3, column=0, sticky="ew", **pad)

        self.foc_enable_btn = ttk.Button(en_frame, text="▶ 使能 FOC",
                                         command=self._foc_toggle_enable, state="disabled")
        self.foc_enable_btn.grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=6)

        ttk.Label(en_frame, text="电压限幅:").grid(row=1, column=0, sticky="w", **pad)
        vf = ttk.Frame(en_frame)
        vf.grid(row=1, column=1, sticky="w", pady=5)
        self.foc_vlimit_var = tk.DoubleVar(value=8.0)
        self.foc_vlimit_slider = ttk.Scale(vf, from_=0.5, to=12.0, orient="horizontal",
                                           variable=self.foc_vlimit_var, length=180,
                                           command=self._foc_on_vlimit_change, state="disabled")
        self.foc_vlimit_slider.pack(side="left")
        self.foc_vlimit_label = ttk.Label(vf, text="8.0 V (扭矩)", width=14)
        self.foc_vlimit_label.pack(side="left", padx=6)

        ttk.Label(en_frame, text="位置环 P:").grid(row=2, column=0, sticky="w", **pad)
        pg_frame = ttk.Frame(en_frame)
        pg_frame.grid(row=2, column=1, sticky="w", pady=5)
        self.foc_pangle_var = tk.DoubleVar(value=40.0)
        self.foc_pangle_slider = ttk.Scale(pg_frame, from_=1.0, to=50.0, orient="horizontal",
                                           variable=self.foc_pangle_var, length=180,
                                           command=self._foc_on_pangle_change, state="disabled")
        self.foc_pangle_slider.pack(side="left")
        self.foc_pangle_label = ttk.Label(pg_frame, text="40.0 (刚度)", width=14)
        self.foc_pangle_label.pack(side="left", padx=6)

        ttk.Label(en_frame, text="速度环 P:").grid(row=3, column=0, sticky="w", **pad)
        vp_frame = ttk.Frame(en_frame)
        vp_frame.grid(row=3, column=1, sticky="w", pady=5)
        self.foc_vp_var = tk.DoubleVar(value=0.4)
        self.foc_vp_slider = ttk.Scale(vp_frame, from_=0.05, to=1.0, orient="horizontal",
                                       variable=self.foc_vp_var, length=180,
                                       command=self._foc_on_vp_change, state="disabled")
        self.foc_vp_slider.pack(side="left")
        self.foc_vp_label = ttk.Label(vp_frame, text="0.40 (阻尼)", width=14)
        self.foc_vp_label.pack(side="left", padx=6)

        ttk.Label(en_frame, text="极对数:").grid(row=4, column=0, sticky="w", **pad)
        pp_frame = ttk.Frame(en_frame)
        pp_frame.grid(row=4, column=1, sticky="w", pady=5)
        self.foc_pp_var = tk.IntVar(value=7)
        ttk.Spinbox(pp_frame, from_=1, to=50, textvariable=self.foc_pp_var, width=6).pack(side="left")
        self.foc_pp_save_btn = ttk.Button(pp_frame, text="保存到 NVS（重启生效）",
                                          command=self._foc_save_pp, state="disabled")
        self.foc_pp_save_btn.pack(side="left", padx=6)

        self.foc_clear_btn = ttk.Button(en_frame, text="🧹 清除故障",
                                        command=self._foc_clear_fault, state="disabled")
        self.foc_clear_btn.grid(row=5, column=0, columnspan=2, **pad, ipadx=10)

        self.foc_autotune_btn = ttk.Button(en_frame, text="🤖 自动优化 PID (约 60 秒)",
                                           command=self._foc_autotune, state="disabled")
        self.foc_autotune_btn.grid(row=6, column=0, columnspan=2, **pad, ipadx=10)

        # 示波器区
        scope_frame = ttk.LabelFrame(parent, text="响应曲线 (最近 10s · 蓝=目标 红=实测)")
        scope_frame.grid(row=4, column=0, sticky="ew", **pad)
        self.foc_scope = tk.Canvas(scope_frame, width=560, height=180,
                                   bg="white", highlightthickness=1, highlightbackground="#999")
        self.foc_scope.pack(padx=5, pady=5)

        # 聚合按钮组（E5 状态门控使用）
        self._foc_motion_btns = [self.foc_goto_btn] + self._foc_quick_btns + self._foc_inc_btns
        self._foc_cfg_btns    = [self.foc_home_btn, self.foc_vlimit_slider,
                                 self.foc_pangle_slider, self.foc_vp_slider,
                                 self.foc_pp_save_btn, self.foc_autotune_btn]

    # ═════════════ 公共辅助 ═════════════
    def _update_speed_label(self, *_):
        d = max(1, self.delay_var.get())
        speed_mm_s = 1000.0 / (PULSES_PER_MM * d)
        self.delay_label.config(text=f"{d} ms/脉冲 ≈ {speed_mm_s:.1f} mm/s")

    def _update_pos_label(self):
        self.pos_label.config(text=f"{self.position_mm:.1f} mm")

    def _set_stepper_buttons_state(self, state):
        for b in (self.move_btn, self.jog_out_btn, self.jog_in_btn,
                  self.cont_out_btn, self.cont_in_btn,
                  self.set_home_btn, self.go_home_btn, self.goto_btn, self.calib_btn):
            b.config(state=state)

    # ═════════════ 串口连接 ═════════════
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.set(ports[0])

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            # 先停轮询
            self.foc_poll_running = False
            time.sleep(0.15)
            with self.serial_lock:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
            self.conn_status.config(text="● 未连接", foreground="red")
            self.conn_btn.config(text="连接")
            self._set_stepper_buttons_state("disabled")
            self._apply_foc_gating(state="?", fault="?")
            self.foc_state_var.set("未连接")
            self.foc_current_var.set("--")
            self.foc_fault_var.set("--")
            self.log("串口已断开")
            return

        try:
            self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=0.2)
            time.sleep(1.0)
            # 清空启动 banner
            with self.serial_lock:
                while self.ser.in_waiting:
                    self.ser.readline()
            self.conn_status.config(text="● 已连接", foreground="green")
            self.conn_btn.config(text="断开")
            self._set_stepper_buttons_state("normal")
            self.stop_btn.config(state="disabled")  # 仅连续运动时启用
            self.log(f"已连接 {self.port_var.get()} @ {self.baud_var.get()}")
            # 启动 FOC 状态轮询
            self.foc_poll_running = True
            threading.Thread(target=self._foc_poll_loop, daemon=True).start()
        except Exception as e:
            messagebox.showerror("连接失败", str(e))

    # ═════════════ 共享底层 I/O ═════════════
    def _send_and_read(self, cmd: str) -> str:
        """所有串口访问的统一入口，带锁。返回单行响应（已 strip）。"""
        if not self.ser or not self.ser.is_open:
            return ""
        with self.serial_lock:
            try:
                self.ser.write((cmd + "\n").encode())
                return self.ser.readline().decode(errors="replace").strip()
            except Exception as e:
                self.log(f"串口异常: {e}")
                return ""

    # ═════════════ 步进：脉冲发送 ═════════════
    def _send_pulses(self, steps, direction, delay_ms):
        if steps <= 0 or not self.ser or not self.ser.is_open:
            return False
        self.stepper_in_progress = True
        try:
            resp = self._send_and_read(f"MOVE,{steps},{direction},{delay_ms}")
            dist_mm = steps / PULSES_PER_MM
            dir_txt = "向外" if direction == DIR_OUTWARD else "向内"
            self.log(f"发送 {dist_mm:.1f}mm {dir_txt} @ {delay_ms}ms/脉冲 → {resp}")
            if resp == "OK":
                sign = +1.0 if direction == DIR_OUTWARD else -1.0
                self.position_mm += sign * dist_mm
                self.root.after(0, self._update_pos_label)
                return True
            return False
        finally:
            self.stepper_in_progress = False

    def _send_mm(self, distance_mm, direction, delay_ms):
        steps = int(round(distance_mm * PULSES_PER_MM))
        if steps <= 0:
            self.log(f"忽略：距离过小 ({distance_mm} mm)")
            return False
        return self._send_pulses(steps, direction, delay_ms)

    # ═════════════ 步进：运动指令 ═════════════
    def send_move(self):
        threading.Thread(target=self._send_mm,
                         args=(self.distance_var.get(), self.dir_var.get(), self.delay_var.get()),
                         daemon=True).start()

    def _quick_move(self, distance_mm, direction):
        threading.Thread(target=self._send_mm,
                         args=(distance_mm, direction, self.delay_var.get()),
                         daemon=True).start()

    def _press_continuous(self, direction):
        if self.running or not self.ser or not self.ser.is_open:
            return
        self.running = True
        self.stop_btn.config(state="normal")
        dir_txt = "向外" if direction == DIR_OUTWARD else "向内"
        self.log(f"按住连续{dir_txt} (burst={CONTINUOUS_BURST_MM}mm)")

        def worker():
            while self.running:
                if not self._send_mm(CONTINUOUS_BURST_MM, direction, self.delay_var.get()):
                    break
            self.log("连续运动已停止")
            self.root.after(0, lambda: self.stop_btn.config(state="disabled"))

        threading.Thread(target=worker, daemon=True).start()

    def _release_continuous(self):
        if self.running:
            self.running = False

    def stop_continuous(self):
        self.running = False

    # ═════════════ 步进：原点/定位 ═════════════
    def set_home(self):
        self.position_mm = 0.0
        self._update_pos_label()
        self.log("✓ 当前位置已设为原点 (0.0 mm)")

    def calibrate_position(self):
        try:
            val = float(self.target_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("输入无效", "请先在前往位置框内输入数值")
            return
        self.position_mm = val
        self._update_pos_label()
        self.log(f"✓ 当前位置已校准为 {val:.1f} mm")

    def go_home(self):
        self._goto(0.0)

    def goto_target_position(self):
        try:
            target = float(self.target_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("输入无效", "请输入有效的目标位置")
            return
        self._goto(target)

    def _goto(self, target_mm):
        delta = target_mm - self.position_mm
        if abs(delta) < 1.0 / PULSES_PER_MM:
            self.log(f"已在目标位置附近 ({self.position_mm:.1f} mm)")
            return
        direction = DIR_OUTWARD if delta > 0 else DIR_INWARD
        distance = abs(delta)
        self.log(f"前往 {target_mm:.1f} mm  (当前 {self.position_mm:.1f} mm, 需移动 {distance:.1f} mm)")
        threading.Thread(target=self._send_mm,
                         args=(distance, direction, self.delay_var.get()),
                         daemon=True).start()

    # ═════════════ FOC：命令绑定 ═════════════
    def _send_foc(self, cmd: str):
        """异步发 FOC 命令，日志记录。"""
        def worker():
            resp = self._send_and_read(cmd)
            self.log(f"发送 {cmd} → {resp}")
        threading.Thread(target=worker, daemon=True).start()

    def _foc_goto(self):
        self._send_foc(f"FOC,A,{self.foc_target_var.get():.1f}")

    def _foc_quick(self, deg):
        self.foc_target_var.set(deg)
        self._foc_goto()

    def _foc_increment(self, delta):
        self.foc_target_var.set(self.foc_target_var.get() + delta)
        self._foc_goto()

    def _foc_home(self):
        self._send_foc("FOC,H")

    def _foc_toggle_enable(self):
        self._foc_enabled_ui = not self._foc_enabled_ui
        val = 1 if self._foc_enabled_ui else 0
        self._send_foc(f"FOC,EN,{val}")

    def _foc_on_vlimit_change(self, _value):
        v = self.foc_vlimit_var.get()
        self.foc_vlimit_label.config(text=f"{v:.1f} V (扭矩)")
        self._send_foc(f"FOC,V,{v:.1f}")

    def _foc_on_pangle_change(self, _value):
        p = self.foc_pangle_var.get()
        self.foc_pangle_label.config(text=f"{p:.1f} (刚度)")
        self._send_foc(f"FOC,PA,{p:.1f}")

    def _foc_on_vp_change(self, _value):
        p = self.foc_vp_var.get()
        self.foc_vp_label.config(text=f"{p:.2f} (阻尼)")
        self._send_foc(f"FOC,VP,{p:.2f}")

    def _foc_save_pp(self):
        n = int(self.foc_pp_var.get())
        self._send_foc(f"FOC,PP,{n}")

    def _foc_clear_fault(self):
        self._send_foc("FOC,CLR")
        self._foc_enabled_ui = False  # 清故障后需要用户手动再使能

    # ═════════════ FOC：自动调参 ═════════════
    def _foc_autotune(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("未连接", "请先连接 COM 口并使能 FOC")
            return
        if not messagebox.askokcancel(
            "自动调参确认",
            "将依次测试不同 P_angle（阶跃 0°→90°→0°），\n"
            "电机会来回转动约 60 秒。\n\n"
            "开始吗？"):
            return
        self.foc_autotune_btn.config(state="disabled")
        threading.Thread(target=self._foc_autotune_worker, daemon=True).start()

    def _foc_autotune_worker(self):
        try:
            self.log("=" * 40)
            self.log("🤖 自动调参开始")
            self.log("=" * 40)

            # 确保使能 + 归零
            self._send_and_read("FOC,EN,1")
            time.sleep(5.5)  # 给首次对齐时间
            self._send_and_read("FOC,H")
            time.sleep(0.5)
            # VP 固定低值，只扫 PA
            self._send_and_read("FOC,VP,0.15")
            self.log("固定 VP=0.15，扫描 PA...")

            results = []  # (pa, overshoot, ss_error, rise_time)

            for pa in [5, 10, 15, 20, 25, 30, 35, 40]:
                self.log(f"  测试 PA={pa} ...")
                self._send_and_read(f"FOC,PA,{pa}")
                time.sleep(0.3)

                # 先回 0 稳定
                self._send_and_read("FOC,A,0")
                time.sleep(2.0)

                # 记录阶跃响应
                t0 = time.time()
                self._send_and_read("FOC,A,90")
                time.sleep(2.5)  # 2.5s 足够 2208 完成阶跃

                # 从 trace_buf 取这段数据分析
                data = [(t - t0, cur) for t, tgt, cur in self.foc_trace_buf
                        if t >= t0 and abs(tgt - 90) < 0.5]
                if len(data) < 5:
                    self.log(f"    数据不足，跳过")
                    continue
                curs = [c for _, c in data]
                max_cur = max(curs)
                final_cur = curs[-1]
                overshoot = max(0.0, max_cur - 90.0)
                ss_error = abs(final_cur - 90.0)

                # 粗略上升时间：首次到 81° (90% of 90)
                rise_time = None
                for t, c in data:
                    if c >= 81:
                        rise_time = t
                        break

                rt_str = f"{rise_time:.2f}s" if rise_time else ">2.5s"
                self.log(f"    overshoot={overshoot:.1f}° ss_err={ss_error:.1f}° rise={rt_str}")
                results.append((pa, overshoot, ss_error, rise_time or 99.0))

                # 过冲太大提前停止
                if overshoot > 30:
                    self.log(f"    过冲 >30°，停止继续增大 PA")
                    break

            # 评分：small overshoot + fast rise + small ss_err
            # 权重：过冲最重要（防振荡），上升时间次之
            def score(r):
                pa, ov, sse, rt = r
                penalty_overshoot = ov * 2
                penalty_rise = rt * 3
                penalty_sserr = sse * 2
                return penalty_overshoot + penalty_rise + penalty_sserr

            if not results:
                self.log("❌ 没有有效数据")
                return
            # 过滤过冲 >15 的
            good = [r for r in results if r[1] <= 15]
            pool = good if good else results
            best = min(pool, key=score)
            best_pa, best_ov, best_sse, best_rt = best

            self.log("=" * 40)
            self.log(f"✅ 推荐 PA={best_pa}")
            self.log(f"   overshoot={best_ov:.1f}°  ss_err={best_sse:.1f}°  rise={best_rt:.2f}s")
            self.log("   已自动应用此值")
            self.log("=" * 40)

            # 应用并回零
            self._send_and_read(f"FOC,PA,{best_pa}")
            time.sleep(0.2)
            self._send_and_read("FOC,A,0")
            # 同步 GUI 滑条显示
            self.root.after(0, lambda: self.foc_pangle_var.set(float(best_pa)))
            self.root.after(0, lambda: self.foc_pangle_label.config(text=f"{best_pa:.1f} (刚度)"))
        finally:
            self.root.after(0, lambda: self.foc_autotune_btn.config(state="normal"))

    # ═════════════ FOC：状态轮询 ═════════════
    def _foc_poll_loop(self):
        while self.foc_poll_running and self.ser and self.ser.is_open:
            resp = self._send_and_read("FOC,S")
            if resp.startswith("FOC,S,"):
                parts = resp.split(",")
                if len(parts) == 6:
                    state, cur, tgt, fault = parts[2], parts[3], parts[4], parts[5]
                    self.root.after(0, lambda s=state, c=cur, t=tgt, f=fault:
                                    self._update_foc_display(s, c, t, f))
            time.sleep(FOC_POLL_INTERVAL_S)

    def _update_foc_display(self, state, cur, tgt, fault):
        name = FOC_STATE_NAMES.get(state, "?")
        self.foc_state_var.set(name)
        try:
            cur_f = float(cur)
            tgt_f = float(tgt)
        except ValueError:
            cur_f = tgt_f = None
        cur_text = f"{cur_f:.1f}°" if cur_f is not None else "--"
        if self.stepper_in_progress:
            cur_text += " ⏸"
        self.foc_current_var.set(cur_text)
        fault_text = "报警" if fault == "1" else "正常"
        self.foc_fault_var.set(fault_text)
        self.foc_fault_label.config(foreground="red" if fault == "1" else "green")
        self._apply_foc_gating(state, fault)

        # 示波器：追加采样并重绘（只有 RUNNING 且数据合法时）
        if cur_f is not None and state == "2":
            now = time.time()
            self.foc_trace_buf.append((now, tgt_f, cur_f))
            cutoff = now - 10.0
            self.foc_trace_buf = [x for x in self.foc_trace_buf if x[0] >= cutoff]
            self._redraw_foc_scope()

    def _redraw_foc_scope(self):
        W, H = 560, 180
        cv = self.foc_scope
        cv.delete("all")
        buf = self.foc_trace_buf
        if len(buf) < 2:
            cv.create_text(W/2, H/2, text="(等待数据，先使能 FOC)", fill="#888")
            return
        t0 = buf[0][0]
        t_span = max(0.1, buf[-1][0] - t0)

        ys = [p[1] for p in buf] + [p[2] for p in buf]
        y_min, y_max = min(ys), max(ys)
        if y_max - y_min < 10:
            c = (y_min + y_max) / 2
            y_min, y_max = c - 5, c + 5
        y_pad = (y_max - y_min) * 0.1
        y_min -= y_pad
        y_max += y_pad

        # 浅色网格
        for frac in (0.25, 0.5, 0.75):
            y = H * frac
            cv.create_line(0, y, W, y, fill="#e5e5e5")

        # 零度线
        if y_min < 0 < y_max:
            y0 = H * (y_max - 0) / (y_max - y_min)
            cv.create_line(0, y0, W, y0, fill="#aaa", dash=(3, 3))

        # 轴标签
        cv.create_text(3, 3, text=f"{y_max:.0f}°", anchor="nw", fill="#555", font=("Arial", 8))
        cv.create_text(3, H-3, text=f"{y_min:.0f}°", anchor="sw", fill="#555", font=("Arial", 8))
        cv.create_text(W-3, H-3, text=f"now", anchor="se", fill="#555", font=("Arial", 8))

        # 构造两条折线
        pts_t, pts_c = [], []
        for t, tgt, cur in buf:
            x = W * (t - t0) / t_span
            yt = H * (y_max - tgt) / (y_max - y_min)
            yc = H * (y_max - cur) / (y_max - y_min)
            pts_t.extend([x, yt])
            pts_c.extend([x, yc])
        if len(pts_t) >= 4:
            cv.create_line(*pts_t, fill="#1565c0", width=1)  # 目标 蓝
        if len(pts_c) >= 4:
            cv.create_line(*pts_c, fill="#d32f2f", width=2)  # 实测 红

    def _apply_foc_gating(self, state, fault):
        """状态门控 per spec §7.5"""
        if not self.ser or not self.ser.is_open:
            # 全部灰
            for w in self._foc_motion_btns + self._foc_cfg_btns:
                w.config(state="disabled")
            self.foc_enable_btn.config(state="disabled")
            self.foc_clear_btn.config(state="disabled")
            return

        is_fault    = (fault == "1" or state == "3")
        is_disabled = (state == "0")
        is_running  = (state == "2")
        is_aligning = (state == "1")

        # 运动按钮：仅 RUNNING
        motion_state = "normal" if is_running else "disabled"
        for b in self._foc_motion_btns:
            b.config(state=motion_state)

        # 配置类（归零、电压限幅、极对数保存）：DISABLED 或 RUNNING
        cfg_state = "normal" if (is_disabled or is_running) else "disabled"
        for w in self._foc_cfg_btns:
            w.config(state=cfg_state)

        # 使能切换按钮：DISABLED 或 RUNNING 可切
        en_state = "normal" if (is_disabled or is_running) else "disabled"
        self.foc_enable_btn.config(state=en_state)
        # 标签更新
        if is_running:
            self.foc_enable_btn.config(text="■ 失能 FOC")
        elif is_aligning:
            self.foc_enable_btn.config(text="… 对齐中")
        elif is_fault:
            self.foc_enable_btn.config(text="(故障，先清除)")
        else:
            self.foc_enable_btn.config(text="▶ 使能 FOC")

        # 清故障：仅 FAULT
        self.foc_clear_btn.config(state="normal" if is_fault else "disabled")

    # ═════════════ 日志 ═════════════
    def log(self, msg):
        def _append():
            self.log_text.config(state="normal")
            self.log_text.insert("end", f"{time.strftime('%H:%M:%S')}  {msg}\n")
            self.log_text.see("end")
            self.log_text.config(state="disabled")
        self.root.after(0, _append)

    def clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")


if __name__ == "__main__":
    root = tk.Tk()
    app = StepperGUI(root)
    root.mainloop()
