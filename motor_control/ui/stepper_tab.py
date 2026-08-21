from tkinter import ttk

from motor_control import AXIS_LABEL, MODE_LINEAR, MODE_ROTARY, stepper_axis_topology
from motor_control.ui.common import PAD, attach_numeric_input


def build_stepper_tab(app, parent, axis, speed_presets, dir_outward, dir_inward):
    pad = PAD
    sw = app.sw[axis]
    unit = app._unit_label(axis)

    axf = ttk.LabelFrame(parent, text=f"轴配置 — 轴 {AXIS_LABEL[axis]}")
    axf.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)
    topology = stepper_axis_topology(axis)
    if topology["controller"] == "esp32":
        pins = topology["pins"]
        route_text = f"ESP32 本地 · PUL=GPIO{pins['pulse']}   DIR=GPIO{pins['direction']}"
    else:
        route_text = (f"RS485 → Pico {topology['node']} · "
                      f"本地轴 {topology['local_axis'] + 1}")
    ttk.Label(axf, text=route_text,
              font=("Consolas", 10, "bold"), foreground="#1565c0").grid(
        row=0, column=0, columnspan=6, sticky="w", **pad)
    ttk.Label(axf, text="模式:").grid(row=1, column=0, sticky="w", **pad)
    mf = ttk.Frame(axf)
    mf.grid(row=1, column=1, columnspan=5, sticky="w")
    ttk.Radiobutton(mf, text="直线 高度/导程 (mm)", variable=app.axis_mode_var[axis],
                    value=MODE_LINEAR, command=lambda a=axis: app._on_axis_mode_change(a)).pack(side="left", padx=6)
    ttk.Radiobutton(mf, text="旋转 圈/角度 (°)", variable=app.axis_mode_var[axis],
                    value=MODE_ROTARY, command=lambda a=axis: app._on_axis_mode_change(a)).pack(side="left", padx=6)
    ttk.Label(axf, text="脉冲/转:").grid(row=2, column=0, sticky="w", **pad)
    ppr_spin = ttk.Spinbox(axf, from_=1.0, to=10000.0, increment=1.0,
                           textvariable=app.axis_ppr_var[axis], width=8, format="%.1f",
                           command=lambda a=axis: app._on_axis_param_change(a))
    attach_numeric_input(ppr_spin, app.axis_ppr_var[axis])
    ppr_spin.grid(row=2, column=1, **pad)
    ttk.Label(axf, text="减速比:").grid(row=2, column=2, sticky="w", padx=(16, 0))
    gr_spin = ttk.Spinbox(axf, from_=0.001, to=1000.0, increment=0.01,
                          textvariable=app.axis_gr_var[axis], width=8, format="%.2f",
                          command=lambda a=axis: app._on_axis_param_change(a))
    attach_numeric_input(gr_spin, app.axis_gr_var[axis])
    gr_spin.grid(row=2, column=3, **pad)
    sw['lead_label'] = ttk.Label(axf, text="导程(mm/转):")
    sw['lead_label'].grid(row=2, column=4, sticky="w", padx=(16, 0))
    sw['lead_spin'] = ttk.Spinbox(axf, from_=0.01, to=100.0, increment=0.1,
                                  textvariable=app.axis_lead_var[axis], width=8, format="%.3f",
                                  command=lambda a=axis: app._on_axis_param_change(a))
    attach_numeric_input(sw['lead_spin'], app.axis_lead_var[axis])
    sw['lead_spin'].grid(row=2, column=5, **pad)
    app._apply_axis_param_ui(axis)

    pf = ttk.LabelFrame(parent, text=f"运动参数 — 轴 {AXIS_LABEL[axis]}")
    pf.grid(row=1, column=0, sticky="nsew", **pad)
    dist_noun = "角度" if app.axis_profiles[axis].mode == MODE_ROTARY else "距离"
    sw['dist_label'] = ttk.Label(pf, text=f"{dist_noun} ({unit}):")
    sw['dist_label'].grid(row=0, column=0, sticky="w", **pad)
    dist_spin = ttk.Spinbox(pf, from_=0.2, to=500.0, increment=1.0,
                            textvariable=app.v_dist[axis], width=10, format="%.1f")
    attach_numeric_input(dist_spin, app.v_dist[axis])
    dist_spin.grid(row=0, column=1, **pad)
    ttk.Label(pf, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
    qbf = ttk.Frame(pf)
    qbf.grid(row=1, column=1, sticky="w", pady=2)
    for label, val in [("1", 1), ("10", 10), ("50", 50), ("100", 100)]:
        ttk.Button(qbf, text=label, width=6,
                   command=lambda d=val, a=axis: app.v_dist[a].set(d)).pack(side="left", padx=2)
    ttk.Label(pf, text="方向:").grid(row=2, column=0, sticky="w", **pad)
    df = ttk.Frame(pf)
    df.grid(row=2, column=1, sticky="w")
    ttk.Radiobutton(df, text="正向 ▶", variable=app.v_dir[axis], value=dir_outward).pack(side="left", padx=4)
    ttk.Radiobutton(df, text="◀ 反向", variable=app.v_dir[axis], value=dir_inward).pack(side="left", padx=4)
    ttk.Label(pf, text="速度档位:").grid(row=3, column=0, sticky="w", **pad)
    sf = ttk.Frame(pf)
    sf.grid(row=3, column=1, sticky="w", pady=2)
    sw['speed_combo'] = ttk.Combobox(sf, textvariable=app.v_speed_str[axis],
                                     values=[str(s) for s in speed_presets],
                                     width=6, state="readonly")
    sw['speed_combo'].pack(side="left")
    sw['speed_unit_label'] = ttk.Label(sf, text=app._unit_per_s(axis))
    sw['speed_unit_label'].pack(side="left", padx=2)
    sw['delay_label'] = ttk.Label(sf, text="", width=16)
    sw['delay_label'].pack(side="left", padx=6)
    sw['speed_combo'].bind("<<ComboboxSelected>>", lambda e, a=axis: app._on_speed_select(a))
    app._update_speed_label(axis)

    cf = ttk.LabelFrame(parent, text="控制")
    cf.grid(row=1, column=1, sticky="nsew", **pad)
    sw['move_btn'] = ttk.Button(cf, text="执行运动",
                                command=lambda a=axis: app.send_move(a), state="disabled")
    sw['move_btn'].grid(row=0, column=0, columnspan=2, padx=6, pady=4, ipadx=8, ipady=4)
    sw['jog_out_btn'] = ttk.Button(cf, text="正向 1 ▶",
                                   command=lambda a=axis: app._quick_move(a, 1.0, dir_outward),
                                   state="disabled")
    sw['jog_out_btn'].grid(row=1, column=0, **pad)
    sw['jog_in_btn'] = ttk.Button(cf, text="◀ 反向 1",
                                  command=lambda a=axis: app._quick_move(a, 1.0, dir_inward),
                                  state="disabled")
    sw['jog_in_btn'].grid(row=1, column=1, **pad)
    sw['cont_out_btn'] = ttk.Button(cf, text="正向 (按住) ▶▶", state="disabled")
    sw['cont_out_btn'].grid(row=2, column=0, **pad)
    sw['cont_out_btn'].bind("<ButtonPress-1>", lambda e, a=axis: app._press_continuous(a, dir_outward))
    sw['cont_out_btn'].bind("<ButtonRelease-1>", lambda e, a=axis: app._release_continuous(a))
    sw['cont_out_btn'].bind("<Leave>", lambda e, a=axis: app._release_continuous(a))
    sw['cont_in_btn'] = ttk.Button(cf, text="◀◀ 反向 (按住)", state="disabled")
    sw['cont_in_btn'].grid(row=2, column=1, **pad)
    sw['cont_in_btn'].bind("<ButtonPress-1>", lambda e, a=axis: app._press_continuous(a, dir_inward))
    sw['cont_in_btn'].bind("<ButtonRelease-1>", lambda e, a=axis: app._release_continuous(a))
    sw['cont_in_btn'].bind("<Leave>", lambda e, a=axis: app._release_continuous(a))
    sw['stop_btn'] = ttk.Button(cf, text="■ 紧急停止",
                                command=lambda a=axis: app.stop_continuous(a), state="disabled")
    sw['stop_btn'].grid(row=3, column=0, columnspan=2, **pad, ipadx=10)
    sw['progress'] = ttk.Progressbar(cf, orient="horizontal", length=200, mode="determinate")
    sw['progress'].grid(row=4, column=0, columnspan=2, sticky="ew", padx=6, pady=(4, 0))
    sw['progress_label'] = ttk.Label(cf, text="", font=("Consolas", 9))
    sw['progress_label'].grid(row=5, column=0, columnspan=2, sticky="w", padx=10)

    rf = ttk.LabelFrame(parent, text="位置与原点（软件跟踪）")
    rf.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)
    ttk.Label(rf, text="当前位置:").grid(row=0, column=0, sticky="w", **pad)
    sw['pos_label'] = ttk.Label(rf, text=f"0.0 {unit}", font=("Consolas", 14, "bold"), foreground="blue")
    sw['pos_label'].grid(row=0, column=1, sticky="w", **pad)
    sw['set_home_btn'] = ttk.Button(rf, text=f"⌂ 设为原点 (0 {unit})",
                                    command=lambda a=axis: app.set_home(a), state="disabled")
    sw['set_home_btn'].grid(row=0, column=2, **pad)
    sw['go_home_btn'] = ttk.Button(rf, text="⟲ 回到原点",
                                   command=lambda a=axis: app.go_home(a), state="disabled")
    sw['go_home_btn'].grid(row=0, column=3, **pad)
    sw['goto_label'] = ttk.Label(rf, text=f"前往位置 ({unit}):")
    sw['goto_label'].grid(row=1, column=0, sticky="w", **pad)
    goto_spin = ttk.Spinbox(rf, from_=-1000.0, to=1000.0, increment=1.0,
                            textvariable=app.v_goto[axis], width=10, format="%.1f")
    attach_numeric_input(goto_spin, app.v_goto[axis])
    goto_spin.grid(row=1, column=1, **pad)
    sw['goto_btn'] = ttk.Button(rf, text="前往",
                                command=lambda a=axis: app.goto_target_position(a), state="disabled")
    sw['goto_btn'].grid(row=1, column=2, **pad)
    sw['calib_btn'] = ttk.Button(rf, text="把当前位置校准为此值",
                                 command=lambda a=axis: app.calibrate_position(a), state="disabled")
    sw['calib_btn'].grid(row=1, column=3, **pad)

    ttk.Label(rf, text="行程范围:").grid(row=2, column=0, sticky="w", **pad)
    rgf = ttk.Frame(rf)
    rgf.grid(row=2, column=1, columnspan=3, sticky="w", **pad)
    sw['range_label'] = ttk.Label(rgf, text="未校准", foreground="gray", font=("Consolas", 10))
    sw['range_label'].pack(side="left")
    sw['set_min_btn'] = ttk.Button(rgf, text="⊖ 标记当前为最小",
                                   command=lambda a=axis: app._mark_min(a), state="disabled")
    sw['set_min_btn'].pack(side="left", padx=10)
    sw['set_max_btn'] = ttk.Button(rgf, text="⊕ 标记当前为最大",
                                   command=lambda a=axis: app._mark_max(a), state="disabled")
    sw['set_max_btn'].pack(side="left", padx=2)
    sw['clear_range_btn'] = ttk.Button(rgf, text="清除", width=6,
                                       command=lambda a=axis: app._clear_range(a), state="disabled")
    sw['clear_range_btn'].pack(side="left", padx=10)
