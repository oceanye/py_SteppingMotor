import tkinter as tk
from tkinter import ttk

from motor_control import AXIS_LABEL


def build_foc_tab(app, parent, axis):
    pad = dict(padx=10, pady=5)
    fw = app.fw[axis]

    sf = ttk.LabelFrame(parent, text=f"状态 — 轴 {AXIS_LABEL[axis]} (100ms 轮询)")
    sf.grid(row=0, column=0, sticky="ew", **pad)
    ttk.Label(sf, text="状态:").grid(row=0, column=0, sticky="w", **pad)
    ttk.Label(sf, textvariable=app.v_focstate[axis], width=10,
              font=("Consolas", 11, "bold"), foreground="gray").grid(row=0, column=1, sticky="w", **pad)
    ttk.Label(sf, text="故障:").grid(row=0, column=2, sticky="w", **pad)
    fw['fault_label'] = ttk.Label(sf, textvariable=app.v_focfault[axis], width=6,
                                  font=("Consolas", 11, "bold"), foreground="gray")
    fw['fault_label'].grid(row=0, column=3, sticky="w", **pad)
    ttk.Label(sf, text="当前角度:").grid(row=1, column=0, sticky="w", **pad)
    ttk.Label(sf, textvariable=app.v_foccur[axis], width=16,
              font=("Consolas", 14, "bold"), foreground="blue").grid(row=1, column=1, columnspan=3, sticky="w", **pad)

    tf = ttk.LabelFrame(parent, text="目标控制")
    tf.grid(row=1, column=0, sticky="ew", **pad)
    ttk.Label(tf, text="目标角度 (°):").grid(row=0, column=0, sticky="w", **pad)
    ttk.Spinbox(tf, from_=-3600.0, to=3600.0, increment=1.0,
                textvariable=app.v_foctgt[axis], width=10, format="%.1f").grid(row=0, column=1, **pad)
    fw['goto_btn'] = ttk.Button(tf, text="前往",
                                command=lambda a=axis: app._foc_goto(a), state="disabled")
    fw['goto_btn'].grid(row=0, column=2, **pad)
    ttk.Label(tf, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
    qf = ttk.Frame(tf)
    qf.grid(row=1, column=1, columnspan=2, sticky="w", pady=5)
    fw['quick_btns'] = []
    for deg in (0, 45, 90, 180, 270):
        b = ttk.Button(qf, text=f"{deg}°", width=5,
                       command=lambda d=deg, a=axis: app._foc_quick(a, float(d)), state="disabled")
        b.pack(side="left", padx=2)
        fw['quick_btns'].append(b)
    ttk.Label(tf, text="增量:").grid(row=2, column=0, sticky="w", **pad)
    incf = ttk.Frame(tf)
    incf.grid(row=2, column=1, columnspan=2, sticky="w", pady=5)
    fw['inc_btns'] = []
    for delta in (-10, -1, +1, +10):
        b = ttk.Button(incf, text=f"{delta:+d}°", width=5,
                       command=lambda d=float(delta), a=axis: app._foc_increment(a, d), state="disabled")
        b.pack(side="left", padx=2)
        fw['inc_btns'].append(b)

    hf = ttk.LabelFrame(parent, text="原点")
    hf.grid(row=2, column=0, sticky="ew", **pad)
    fw['home_btn'] = ttk.Button(hf, text="⌂ 把当前位置设为 0°",
                                command=lambda a=axis: app._foc_home(a), state="disabled")
    fw['home_btn'].grid(row=0, column=0, **pad)

    ef = ttk.LabelFrame(parent, text="使能 / 调试")
    ef.grid(row=3, column=0, sticky="ew", **pad)
    fw['enable_btn'] = ttk.Button(ef, text="▶ 使能 FOC",
                                  command=lambda a=axis: app._foc_toggle_enable(a), state="disabled")
    fw['enable_btn'].grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=6)

    ttk.Label(ef, text="电压限幅:").grid(row=1, column=0, sticky="w", **pad)
    vf = ttk.Frame(ef)
    vf.grid(row=1, column=1, sticky="w", pady=5)
    fw['vlimit_slider'] = ttk.Scale(vf, from_=0.5, to=24.0, orient="horizontal",
                                    variable=app.v_focvlimit[axis], length=180,
                                    command=lambda v, a=axis: app._foc_on_vlimit(a, v), state="disabled")
    fw['vlimit_slider'].pack(side="left")
    fw['vlimit_slider'].bind("<ButtonRelease-1>", lambda e: app._save_foc_tune())
    fw['vlimit_label'] = ttk.Label(vf, text="10.0 V (扭矩)", width=22)
    fw['vlimit_label'].pack(side="left", padx=6)

    ttk.Label(ef, text="位置环 P:").grid(row=2, column=0, sticky="w", **pad)
    pgf = ttk.Frame(ef)
    pgf.grid(row=2, column=1, sticky="w", pady=5)
    fw['pangle_slider'] = ttk.Scale(pgf, from_=1.0, to=50.0, orient="horizontal",
                                    variable=app.v_focpangle[axis], length=180,
                                    command=lambda v, a=axis: app._foc_on_pangle(a, v), state="disabled")
    fw['pangle_slider'].pack(side="left")
    fw['pangle_slider'].bind("<ButtonRelease-1>", lambda e: app._save_foc_tune())
    fw['pangle_label'] = ttk.Label(pgf, text="25.0 (刚度)", width=14)
    fw['pangle_label'].pack(side="left", padx=6)

    ttk.Label(ef, text="速度环 P:").grid(row=3, column=0, sticky="w", **pad)
    vpf = ttk.Frame(ef)
    vpf.grid(row=3, column=1, sticky="w", pady=5)
    fw['vp_slider'] = ttk.Scale(vpf, from_=0.05, to=1.0, orient="horizontal",
                                variable=app.v_focvp[axis], length=180,
                                command=lambda v, a=axis: app._foc_on_vp(a, v), state="disabled")
    fw['vp_slider'].pack(side="left")
    fw['vp_slider'].bind("<ButtonRelease-1>", lambda e: app._save_foc_tune())
    fw['vp_label'] = ttk.Label(vpf, text="0.20 (阻尼)", width=14)
    fw['vp_label'].pack(side="left", padx=6)

    ttk.Label(ef, text="极对数:").grid(row=4, column=0, sticky="w", **pad)
    ppf = ttk.Frame(ef)
    ppf.grid(row=4, column=1, sticky="w", pady=5)
    ttk.Spinbox(ppf, from_=1, to=50, textvariable=app.v_focpp[axis], width=6).pack(side="left")
    fw['pp_save_btn'] = ttk.Button(ppf, text="保存到 NVS（重启生效）",
                                   command=lambda a=axis: app._foc_save_pp(a), state="disabled")
    fw['pp_save_btn'].pack(side="left", padx=6)
    fw['clear_btn'] = ttk.Button(ef, text="🧹 清除故障",
                                 command=lambda a=axis: app._foc_clear_fault(a), state="disabled")
    fw['clear_btn'].grid(row=5, column=0, columnspan=2, **pad, ipadx=10)
    fw['autotune_btn'] = ttk.Button(ef, text="🤖 自动优化 PID",
                                    command=lambda a=axis: app._foc_autotune(a), state="disabled")
    fw['autotune_btn'].grid(row=6, column=0, columnspan=2, **pad, ipadx=10)

    scf = ttk.LabelFrame(parent, text="响应曲线 (10s · 蓝=目标 红=实测)")
    scf.grid(row=0, column=1, rowspan=4, sticky="nsew", **pad)
    fw['scope'] = tk.Canvas(scf, width=420, height=280, bg="white",
                            highlightthickness=1, highlightbackground="#999")
    fw['scope'].pack(padx=5, pady=5)

    fw['motion_btns'] = [fw['goto_btn']] + fw['quick_btns'] + fw['inc_btns']
    fw['cfg_btns'] = [fw['home_btn'], fw['vlimit_slider'], fw['pangle_slider'],
                      fw['vp_slider'], fw['pp_save_btn'], fw['autotune_btn']]


def build_gear_tab(app, parent, axis):
    pad = dict(padx=10, pady=5)
    gw = app.gw[axis]

    sf = ttk.LabelFrame(parent, text=f"状态 — 减速 {AXIS_LABEL[axis]} (100ms 轮询)")
    sf.grid(row=0, column=0, sticky="ew", **pad)
    ttk.Label(sf, text="状态:").grid(row=0, column=0, sticky="w", **pad)
    ttk.Label(sf, textvariable=app.v_focstate[axis], width=10,
              font=("Consolas", 11, "bold"), foreground="gray").grid(row=0, column=1, sticky="w", **pad)
    ttk.Label(sf, text="故障:").grid(row=0, column=2, sticky="w", **pad)
    gw['fault_label'] = ttk.Label(sf, textvariable=app.v_focfault[axis], width=6,
                                  font=("Consolas", 11, "bold"), foreground="gray")
    gw['fault_label'].grid(row=0, column=3, sticky="w", **pad)
    ttk.Label(sf, text="当前角度:").grid(row=1, column=0, sticky="w", **pad)
    ttk.Label(sf, textvariable=app.v_foccur[axis], width=16,
              font=("Consolas", 14, "bold"), foreground="blue").grid(row=1, column=1, columnspan=3, sticky="w", **pad)

    tf = ttk.LabelFrame(parent, text="目标控制")
    tf.grid(row=1, column=0, sticky="ew", **pad)
    ttk.Label(tf, text="目标角度 (°):").grid(row=0, column=0, sticky="w", **pad)
    ttk.Spinbox(tf, from_=-3600.0, to=3600.0, increment=1.0,
                textvariable=app.v_foctgt[axis], width=10, format="%.1f").grid(row=0, column=1, **pad)
    gw['goto_btn'] = ttk.Button(tf, text="前往",
                                command=lambda a=axis: app._gear_goto(a), state="disabled")
    gw['goto_btn'].grid(row=0, column=2, **pad)
    ttk.Label(tf, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
    qf = ttk.Frame(tf)
    qf.grid(row=1, column=1, columnspan=2, sticky="w", pady=5)
    gw['quick_btns'] = []
    for deg in (0, 45, 90, 180, 270):
        b = ttk.Button(qf, text=f"{deg}°", width=5,
                       command=lambda d=deg, a=axis: app._gear_quick(a, float(d)), state="disabled")
        b.pack(side="left", padx=2)
        gw['quick_btns'].append(b)
    ttk.Label(tf, text="增量:").grid(row=2, column=0, sticky="w", **pad)
    incf = ttk.Frame(tf)
    incf.grid(row=2, column=1, columnspan=2, sticky="w", pady=5)
    gw['inc_btns'] = []
    for delta in (-10, -1, +1, +10):
        b = ttk.Button(incf, text=f"{delta:+d}°", width=5,
                       command=lambda d=float(delta), a=axis: app._gear_increment(a, d), state="disabled")
        b.pack(side="left", padx=2)
        gw['inc_btns'].append(b)

    hf = ttk.LabelFrame(parent, text="原点")
    hf.grid(row=2, column=0, sticky="ew", **pad)
    gw['home_btn'] = ttk.Button(hf, text="⌂ 把当前位置设为 0°",
                                command=lambda a=axis: app._gear_home(a), state="disabled")
    gw['home_btn'].grid(row=0, column=0, **pad)

    ef = ttk.LabelFrame(parent, text="使能 / 调参")
    ef.grid(row=3, column=0, sticky="ew", **pad)
    gw['enable_btn'] = ttk.Button(ef, text="▶ 使能 PID",
                                  command=lambda a=axis: app._gear_toggle_enable(a), state="disabled")
    gw['enable_btn'].grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=10, ipady=6)

    ttk.Label(ef, text="PWM 上限 %:").grid(row=1, column=0, sticky="w", **pad)
    vf = ttk.Frame(ef)
    vf.grid(row=1, column=1, sticky="w", pady=5)
    gw['pwm_slider'] = ttk.Scale(vf, from_=10.0, to=100.0, orient="horizontal",
                                 variable=app.v_gearpwm[axis], length=180,
                                 command=lambda v, a=axis: app._gear_on_pwm(a, v), state="disabled")
    gw['pwm_slider'].pack(side="left")
    gw['pwm_slider'].bind("<ButtonRelease-1>", lambda e: app._save_gear_tune())
    gw['pwm_label'] = ttk.Label(vf, text="100.0 %", width=10)
    gw['pwm_label'].pack(side="left", padx=6)

    ttk.Label(ef, text="Kp (位置比例):").grid(row=2, column=0, sticky="w", **pad)
    kpf = ttk.Frame(ef)
    kpf.grid(row=2, column=1, sticky="w", pady=5)
    gw['kp_slider'] = ttk.Scale(kpf, from_=0.1, to=50.0, orient="horizontal",
                                variable=app.v_gearkp[axis], length=180,
                                command=lambda v, a=axis: app._gear_on_kp(a, v), state="disabled")
    gw['kp_slider'].pack(side="left")
    gw['kp_slider'].bind("<ButtonRelease-1>", lambda e: app._save_gear_tune())
    gw['kp_label'] = ttk.Label(kpf, text="1.0", width=10)
    gw['kp_label'].pack(side="left", padx=6)

    ttk.Label(ef, text="Ki (积分):").grid(row=3, column=0, sticky="w", **pad)
    kif = ttk.Frame(ef)
    kif.grid(row=3, column=1, sticky="w", pady=5)
    gw['ki_slider'] = ttk.Scale(kif, from_=0.0, to=10.0, orient="horizontal",
                                variable=app.v_gearki[axis], length=180,
                                command=lambda v, a=axis: app._gear_on_ki(a, v), state="disabled")
    gw['ki_slider'].pack(side="left")
    gw['ki_slider'].bind("<ButtonRelease-1>", lambda e: app._save_gear_tune())
    gw['ki_label'] = ttk.Label(kif, text="0.00", width=10)
    gw['ki_label'].pack(side="left", padx=6)

    ttk.Label(ef, text="Kd (微分):").grid(row=4, column=0, sticky="w", **pad)
    kdf = ttk.Frame(ef)
    kdf.grid(row=4, column=1, sticky="w", pady=5)
    gw['kd_slider'] = ttk.Scale(kdf, from_=0.0, to=2.0, orient="horizontal",
                                variable=app.v_gearkd[axis], length=180,
                                command=lambda v, a=axis: app._gear_on_kd(a, v), state="disabled")
    gw['kd_slider'].pack(side="left")
    gw['kd_slider'].bind("<ButtonRelease-1>", lambda e: app._save_gear_tune())
    gw['kd_label'] = ttk.Label(kdf, text="0.050", width=10)
    gw['kd_label'].pack(side="left", padx=6)

    ttk.Label(ef, text="齿轮比 GR:").grid(row=5, column=0, sticky="w", **pad)
    grf = ttk.Frame(ef)
    grf.grid(row=5, column=1, sticky="w", pady=5)
    ttk.Spinbox(grf, from_=1.0, to=10000.0, increment=10.0,
                textvariable=app.v_geargr[axis], width=10, format="%.1f").pack(side="left")
    gw['gr_save_btn'] = ttk.Button(grf, text="保存到 NVS（立即生效）",
                                   command=lambda a=axis: app._gear_save_gr(a), state="disabled")
    gw['gr_save_btn'].pack(side="left", padx=6)

    gw['clear_btn'] = ttk.Button(ef, text="🧹 清除故障",
                                 command=lambda a=axis: app._gear_clear_fault(a), state="disabled")
    gw['clear_btn'].grid(row=6, column=0, columnspan=2, **pad, ipadx=10)
    gw['autotune_btn'] = ttk.Button(ef, text="🤖 自动调 PID（约 3 分钟）",
                                    command=lambda a=axis: app._gear_autotune(a), state="disabled")
    gw['autotune_btn'].grid(row=7, column=0, columnspan=2, **pad, ipadx=10)

    scf = ttk.LabelFrame(parent, text="响应曲线 (10s · 蓝=目标 红=实测)")
    scf.grid(row=0, column=1, rowspan=4, sticky="nsew", **pad)
    gw['scope'] = tk.Canvas(scf, width=420, height=280, bg="white",
                            highlightthickness=1, highlightbackground="#999")
    gw['scope'].pack(padx=5, pady=5)

    gw['motion_btns'] = [gw['goto_btn']] + gw['quick_btns'] + gw['inc_btns']
    gw['cfg_widgets'] = [gw['home_btn'], gw['pwm_slider'], gw['kp_slider'],
                         gw['ki_slider'], gw['kd_slider'], gw['gr_save_btn'],
                         gw['autotune_btn']]
