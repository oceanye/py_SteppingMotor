import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

# ============== 标定常数 ==============
PULSES_PER_MM = 50       # 根据实测修正：GUI 设 50mm 实际走 5mm → 需 10× 脉冲数

DIR_OUTWARD = 0          # direction=0 对应物理"向外"（根据实测）
DIR_INWARD  = 1          # direction=1 对应物理"向内/回退"

CONTINUOUS_BURST_MM = 0.2  # 按住连续运动的每次脉冲包（mm）。小=松手后停得快，大=通信更稳


class StepperGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("步进电机控制器 (长度模式)")
        self.root.resizable(False, False)
        self.ser = None
        self.running = False
        self.position_mm = 0.0
        self._build_ui()

    def _build_ui(self):
        pad = dict(padx=10, pady=5)

        # ── 串口连接区 ──────────────────────────────────────
        conn_frame = ttk.LabelFrame(self.root, text="串口连接")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)

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

        # ── 运动参数区 ──────────────────────────────────────
        param_frame = ttk.LabelFrame(self.root, text="运动参数")
        param_frame.grid(row=1, column=0, sticky="nsew", **pad)

        # 距离 (mm)
        ttk.Label(param_frame, text="距离 (mm):").grid(row=0, column=0, sticky="w", **pad)
        self.distance_var = tk.DoubleVar(value=10.0)
        ttk.Spinbox(param_frame, from_=0.2, to=500.0, increment=1.0,
                    textvariable=self.distance_var, width=10, format="%.1f").grid(row=0, column=1, **pad)

        # 快捷距离
        ttk.Label(param_frame, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
        qbtn_frame = ttk.Frame(param_frame)
        qbtn_frame.grid(row=1, column=1, sticky="w", pady=5)
        for label, mm in [("1mm", 1), ("10mm", 10), ("50mm", 50), ("100mm", 100)]:
            ttk.Button(qbtn_frame, text=label, width=6,
                       command=lambda d=mm: self.distance_var.set(d)).pack(side="left", padx=2)

        # 方向
        ttk.Label(param_frame, text="方向:").grid(row=2, column=0, sticky="w", **pad)
        self.dir_var = tk.IntVar(value=DIR_OUTWARD)
        dir_frame = ttk.Frame(param_frame)
        dir_frame.grid(row=2, column=1, sticky="w")
        ttk.Radiobutton(dir_frame, text="向外 ▶", variable=self.dir_var, value=DIR_OUTWARD).pack(side="left", padx=4)
        ttk.Radiobutton(dir_frame, text="◀ 向内", variable=self.dir_var, value=DIR_INWARD).pack(side="left", padx=4)

        # 速度
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

        # ── 控制按钮区 ──────────────────────────────────────
        ctrl_frame = ttk.LabelFrame(self.root, text="控制")
        ctrl_frame.grid(row=1, column=1, sticky="nsew", **pad)

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

        # ── 位置与原点区 ────────────────────────────────────
        pos_frame = ttk.LabelFrame(self.root, text="位置与原点（软件跟踪）")
        pos_frame.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)

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

        # ── 日志区 ──────────────────────────────────────────
        log_frame = ttk.LabelFrame(self.root, text="日志")
        log_frame.grid(row=3, column=0, columnspan=2, sticky="ew", **pad)

        self.log_text = tk.Text(log_frame, height=8, width=72, state="disabled", font=("Consolas", 9))
        self.log_text.pack(side="left", fill="both", padx=5, pady=5)
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)

        ttk.Button(self.root, text="清空日志", command=self.clear_log).grid(row=4, column=1, sticky="e",
                                                                            padx=10, pady=2)

        self.refresh_ports()

    # ── UI 辅助 ───────────────────────────────────────────
    def _update_speed_label(self, *_):
        d = max(1, self.delay_var.get())
        speed_mm_s = 1000.0 / (PULSES_PER_MM * d)
        self.delay_label.config(text=f"{d} ms/脉冲 ≈ {speed_mm_s:.1f} mm/s")

    def _update_pos_label(self):
        self.pos_label.config(text=f"{self.position_mm:.1f} mm")

    def _set_motion_buttons_state(self, state):
        for b in (self.move_btn, self.jog_out_btn, self.jog_in_btn,
                  self.cont_out_btn, self.cont_in_btn,
                  self.set_home_btn, self.go_home_btn, self.goto_btn, self.calib_btn):
            b.config(state=state)

    # ── 串口 ──────────────────────────────────────────────
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.set(ports[0])

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.conn_status.config(text="● 未连接", foreground="red")
            self.conn_btn.config(text="连接")
            self._set_motion_buttons_state("disabled")
            self.log("串口已断开")
        else:
            try:
                self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=2)
                time.sleep(1.5)
                self.ser.readline()  # 消耗启动信息
                self.conn_status.config(text="● 已连接", foreground="green")
                self.conn_btn.config(text="断开")
                self._set_motion_buttons_state("normal")
                self.stop_btn.config(state="disabled")  # 停止按钮只在连续运动时启用
                self.log(f"已连接 {self.port_var.get()} @ {self.baud_var.get()} (若固件在跑 DIAG，请稍等)")
            except Exception as e:
                messagebox.showerror("连接失败", str(e))

    # ── 底层发送 ──────────────────────────────────────────
    def _send_pulses(self, steps, direction, delay_ms):
        """发送脉冲指令。成功则更新软件位置。"""
        if steps <= 0:
            return False
        if not self.ser or not self.ser.is_open:
            return False
        cmd = f"MOVE,{steps},{direction},{delay_ms}\n"
        try:
            self.ser.write(cmd.encode())
            resp = self.ser.readline().decode().strip()
        except Exception as e:
            self.log(f"串口异常: {e}")
            return False

        dist_mm = steps / PULSES_PER_MM
        dir_txt = "向外" if direction == DIR_OUTWARD else "向内"
        self.log(f"发送 {dist_mm:.1f}mm {dir_txt} @ {delay_ms}ms/脉冲 → {resp}")

        if resp == "OK":
            sign = +1.0 if direction == DIR_OUTWARD else -1.0
            self.position_mm += sign * dist_mm
            self.root.after(0, self._update_pos_label)
            return True
        return False

    def _send_mm(self, distance_mm, direction, delay_ms):
        """按距离（mm）发送，内部换算为脉冲。"""
        steps = int(round(distance_mm * PULSES_PER_MM))
        if steps <= 0:
            self.log(f"忽略：距离过小 ({distance_mm} mm)")
            return False
        return self._send_pulses(steps, direction, delay_ms)

    # ── 运动指令 ──────────────────────────────────────────
    def send_move(self):
        threading.Thread(target=self._send_mm,
                         args=(self.distance_var.get(), self.dir_var.get(), self.delay_var.get()),
                         daemon=True).start()

    def _quick_move(self, distance_mm, direction):
        threading.Thread(target=self._send_mm,
                         args=(distance_mm, direction, self.delay_var.get()),
                         daemon=True).start()

    def _press_continuous(self, direction):
        """鼠标按下：启动连续发送 worker。仅在未运行且已连接时生效。"""
        if self.running:
            return
        if not self.ser or not self.ser.is_open:
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
        """鼠标松开 / 移出按钮：停止连续发送。"""
        if self.running:
            self.running = False

    def stop_continuous(self):
        """紧急停止按钮。"""
        self.running = False

    # ── 原点 / 定位 ───────────────────────────────────────
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
        if abs(delta) < 1.0 / PULSES_PER_MM:  # 不足 1 脉冲
            self.log(f"已在目标位置附近 ({self.position_mm:.1f} mm)")
            return
        direction = DIR_OUTWARD if delta > 0 else DIR_INWARD
        distance = abs(delta)
        self.log(f"前往 {target_mm:.1f} mm  (当前 {self.position_mm:.1f} mm, 需移动 {distance:.1f} mm)")
        threading.Thread(target=self._send_mm,
                         args=(distance, direction, self.delay_var.get()),
                         daemon=True).start()

    # ── 日志 ──────────────────────────────────────────────
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
