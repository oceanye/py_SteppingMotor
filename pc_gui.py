import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

class StepperGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("步进电机控制器")
        self.root.resizable(False, False)
        self.ser = None
        self.running = False
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
        ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["9600","115200"], width=8, state="readonly").grid(row=0, column=4, **pad)

        self.conn_btn = ttk.Button(conn_frame, text="连接", command=self.toggle_connect)
        self.conn_btn.grid(row=0, column=5, **pad)

        self.conn_status = ttk.Label(conn_frame, text="● 未连接", foreground="red")
        self.conn_status.grid(row=0, column=6, **pad)

        # ── 运动参数区 ──────────────────────────────────────
        param_frame = ttk.LabelFrame(self.root, text="运动参数")
        param_frame.grid(row=1, column=0, sticky="nsew", **pad)

        # 步数
        ttk.Label(param_frame, text="步数:").grid(row=0, column=0, sticky="w", **pad)
        self.steps_var = tk.IntVar(value=200)
        steps_spin = ttk.Spinbox(param_frame, from_=1, to=99999, textvariable=self.steps_var, width=10)
        steps_spin.grid(row=0, column=1, **pad)

        # 细分快捷按钮（DM422常用细分对应步数）
        ttk.Label(param_frame, text="快捷(圈数):").grid(row=1, column=0, sticky="w", **pad)
        btn_frame = ttk.Frame(param_frame)
        btn_frame.grid(row=1, column=1, sticky="w", pady=5)
        for label, steps in [("1圈", 3200), ("半圈", 1600), ("1/4圈", 800)]:
            ttk.Button(btn_frame, text=label, width=6,
                       command=lambda s=steps: self.steps_var.set(s)).pack(side="left", padx=2)

        # 方向
        ttk.Label(param_frame, text="方向:").grid(row=2, column=0, sticky="w", **pad)
        self.dir_var = tk.IntVar(value=1)
        dir_frame = ttk.Frame(param_frame)
        dir_frame.grid(row=2, column=1, sticky="w")
        ttk.Radiobutton(dir_frame, text="正转 ▶", variable=self.dir_var, value=1).pack(side="left", padx=4)
        ttk.Radiobutton(dir_frame, text="反转 ◀", variable=self.dir_var, value=0).pack(side="left", padx=4)

        # 速度（delay_ms）
        ttk.Label(param_frame, text="速度:").grid(row=3, column=0, sticky="w", **pad)
        speed_frame = ttk.Frame(param_frame)
        speed_frame.grid(row=3, column=1, sticky="w", pady=5)
        self.delay_var = tk.IntVar(value=10)
        self.speed_slider = ttk.Scale(speed_frame, from_=2, to=100, orient="horizontal",
                                      variable=self.delay_var, length=160,
                                      command=lambda v: self.delay_var.set(int(float(v))))
        self.speed_slider.pack(side="left")
        self.delay_label = ttk.Label(speed_frame, text="", width=12)
        self.delay_label.pack(side="left", padx=6)
        self.delay_var.trace_add("write", self._update_speed_label)
        self._update_speed_label()

        # ── 控制按钮区 ──────────────────────────────────────
        ctrl_frame = ttk.LabelFrame(self.root, text="控制")
        ctrl_frame.grid(row=1, column=1, sticky="nsew", **pad)

        self.move_btn = ttk.Button(ctrl_frame, text="执行运动", command=self.send_move, state="disabled")
        self.move_btn.grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=8)

        ttk.Button(ctrl_frame, text="正转一步 ▶", command=lambda: self._quick_move(1, 1)).grid(row=1, column=0, **pad)
        ttk.Button(ctrl_frame, text="◀ 反转一步", command=lambda: self._quick_move(1, 0)).grid(row=1, column=1, **pad)

        ttk.Button(ctrl_frame, text="正转连续 ▶▶", command=lambda: self._start_continuous(1)).grid(row=2, column=0, **pad)
        ttk.Button(ctrl_frame, text="◀◀ 反转连续", command=lambda: self._start_continuous(0)).grid(row=2, column=1, **pad)

        self.stop_btn = ttk.Button(ctrl_frame, text="■ 停止", command=self.stop_continuous, state="disabled")
        self.stop_btn.grid(row=3, column=0, columnspan=2, **pad, ipadx=10)

        # ── 日志区 ──────────────────────────────────────────
        log_frame = ttk.LabelFrame(self.root, text="日志")
        log_frame.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)

        self.log_text = tk.Text(log_frame, height=6, width=60, state="disabled", font=("Consolas", 9))
        self.log_text.pack(side="left", fill="both", padx=5, pady=5)
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)

        ttk.Button(self.root, text="清空日志", command=self.clear_log).grid(row=3, column=1, sticky="e", padx=10, pady=2)

        self.refresh_ports()

    def _update_speed_label(self, *_):
        d = self.delay_var.get()
        if d <= 5:
            desc = "快"
        elif d <= 20:
            desc = "中"
        else:
            desc = "慢"
        self.delay_label.config(text=f"{d} ms/步 ({desc})")

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
            self.move_btn.config(state="disabled")
            self.log("串口已断开")
        else:
            try:
                self.ser = serial.Serial(self.port_var.get(), int(self.baud_var.get()), timeout=2)
                time.sleep(1.5)
                self.ser.readline()  # 读掉启动信息
                self.conn_status.config(text="● 已连接", foreground="green")
                self.conn_btn.config(text="断开")
                self.move_btn.config(state="normal")
                self.log(f"已连接 {self.port_var.get()} @ {self.baud_var.get()}")
            except Exception as e:
                messagebox.showerror("连接失败", str(e))

    def _send_cmd(self, steps, direction, delay_ms):
        if not self.ser or not self.ser.is_open:
            return False
        cmd = f"MOVE,{steps},{direction},{delay_ms}\n"
        self.ser.write(cmd.encode())
        resp = self.ser.readline().decode().strip()
        self.log(f"发送: MOVE {steps}步 {'正转' if direction else '反转'} {delay_ms}ms/步 → {resp}")
        return resp == "OK"

    def send_move(self):
        threading.Thread(target=self._send_cmd,
                         args=(self.steps_var.get(), self.dir_var.get(), self.delay_var.get()),
                         daemon=True).start()

    def _quick_move(self, steps, direction):
        threading.Thread(target=self._send_cmd,
                         args=(steps, direction, self.delay_var.get()),
                         daemon=True).start()

    def _start_continuous(self, direction):
        if self.running:
            return
        self.running = True
        self.stop_btn.config(state="normal")
        self.log(f"开始连续{'正转' if direction else '反转'}...")

        def loop():
            while self.running:
                if not self._send_cmd(50, direction, self.delay_var.get()):
                    break
            self.log("连续运动已停止")
            self.root.after(0, lambda: self.stop_btn.config(state="disabled"))

        threading.Thread(target=loop, daemon=True).start()

    def stop_continuous(self):
        self.running = False

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
