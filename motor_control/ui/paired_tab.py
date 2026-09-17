"""左右直线轴联动页：左侧直 + 右侧直同时执行同一命令。

不新写运动逻辑——所有命令走 desktop_app 既有的单轴链路
（``_quick_move`` / ``_press_continuous`` / ``stop_continuous``），
本页只是把同一命令同时发给两个轴，并在启动前要求两轴都空闲，
避免只动一边。速度档位与两轴单轴页共享（选择时同步写入）。
"""

from tkinter import ttk

from motor_control import AXIS_LABEL, direction_label_parts
from motor_control.ui.common import PAD, attach_numeric_input


def build_paired_tab(app, parent, axes, speed_presets,
                     dir_outward, dir_inward):
    pad = PAD
    pw = app.paired_widgets
    out_txt, in_txt, out_arrow, in_arrow = direction_label_parts(axes[0])
    names = " + ".join(f"轴 {AXIS_LABEL[axis]}" for axis in axes)

    intro = ttk.LabelFrame(
        parent, text=f"左右直线联动 — {names}")
    intro.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)
    ttk.Label(
        intro,
        text=(
            "两个直线轴同时执行同一命令（方向/距离/速度相同）。\n"
            "任何一轴正在运动时拒绝启动，防止只动一边；急停同时停两轴。"
        ),
        justify="left",
    ).pack(anchor="w", padx=8, pady=5)

    pf = ttk.LabelFrame(parent, text="运动参数（两轴共用）")
    pf.grid(row=1, column=0, sticky="nsew", **pad)
    ttk.Label(pf, text="距离 (mm):").grid(row=0, column=0, sticky="w", **pad)
    dist_spin = ttk.Spinbox(pf, from_=0.2, to=500.0, increment=1.0,
                            textvariable=app.pair_dist, width=10,
                            format="%.1f")
    attach_numeric_input(dist_spin, app.pair_dist)
    dist_spin.grid(row=0, column=1, **pad)
    ttk.Label(pf, text="快捷:").grid(row=1, column=0, sticky="w", **pad)
    qbf = ttk.Frame(pf)
    qbf.grid(row=1, column=1, sticky="w", pady=2)
    for label, val in [("1", 1), ("10", 10), ("50", 50), ("100", 100)]:
        ttk.Button(qbf, text=label, width=6,
                   command=lambda d=val: app.pair_dist.set(d)).pack(
                       side="left", padx=2)
    ttk.Label(pf, text="方向:").grid(row=2, column=0, sticky="w", **pad)
    df = ttk.Frame(pf)
    df.grid(row=2, column=1, sticky="w")
    ttk.Radiobutton(df, text=f"两轴同时 {out_txt} {out_arrow}",
                    variable=app.pair_dir,
                    value=dir_outward).pack(side="left", padx=4)
    ttk.Radiobutton(df, text=f"{in_arrow} 两轴同时 {in_txt}",
                    variable=app.pair_dir,
                    value=dir_inward).pack(side="left", padx=4)
    ttk.Label(pf, text="速度档位:").grid(row=3, column=0, sticky="w", **pad)
    sf = ttk.Frame(pf)
    sf.grid(row=3, column=1, sticky="w", pady=2)
    ttk.Combobox(sf, textvariable=app.pair_speed_str,
                 values=[str(s) for s in speed_presets],
                 width=6, state="readonly").pack(side="left")
    ttk.Label(sf, text=app._unit_per_s(axes[0])).pack(side="left", padx=2)

    cf = ttk.LabelFrame(parent, text="控制（同时作用于两轴）")
    cf.grid(row=1, column=1, sticky="nsew", **pad)
    pw['move_btn'] = ttk.Button(cf, text="⚙⚙ 执行联动运动",
                                command=app._paired_send_move,
                                state="disabled")
    pw['move_btn'].grid(row=0, column=0, columnspan=2, padx=6, pady=4,
                        ipadx=8, ipady=4)
    pw['jog_out_btn'] = ttk.Button(
        cf, text=f"{out_txt} 1 {out_arrow}",
        command=lambda: app._paired_quick_move(dir_outward),
        state="disabled")
    pw['jog_out_btn'].grid(row=1, column=0, **pad)
    pw['jog_in_btn'] = ttk.Button(
        cf, text=f"{in_arrow} {in_txt} 1",
        command=lambda: app._paired_quick_move(dir_inward),
        state="disabled")
    pw['jog_in_btn'].grid(row=1, column=1, **pad)
    pw['cont_out_btn'] = ttk.Button(
        cf, text=f"{out_txt} (按住) {out_arrow}{out_arrow}", state="disabled")
    pw['cont_out_btn'].grid(row=2, column=0, **pad)
    pw['cont_out_btn'].bind(
        "<ButtonPress-1>", lambda e: app._paired_press_continuous(dir_outward))
    pw['cont_out_btn'].bind(
        "<ButtonRelease-1>", lambda e: app._paired_release_continuous())
    pw['cont_out_btn'].bind(
        "<Leave>", lambda e: app._paired_release_continuous())
    pw['cont_in_btn'] = ttk.Button(
        cf, text=f"{in_arrow}{in_arrow} {in_txt} (按住)", state="disabled")
    pw['cont_in_btn'].grid(row=2, column=1, **pad)
    pw['cont_in_btn'].bind(
        "<ButtonPress-1>", lambda e: app._paired_press_continuous(dir_inward))
    pw['cont_in_btn'].bind(
        "<ButtonRelease-1>", lambda e: app._paired_release_continuous())
    pw['cont_in_btn'].bind(
        "<Leave>", lambda e: app._paired_release_continuous())
    pw['stop_btn'] = ttk.Button(cf, text="■■ 两轴紧急停止",
                                command=app._paired_stop, state="disabled")
    pw['stop_btn'].grid(row=3, column=0, columnspan=2, **pad, ipadx=10)

    rf = ttk.LabelFrame(parent, text="位置与原点（软件跟踪）")
    rf.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)
    for column, axis in enumerate(axes):
        ttk.Label(rf, text=f"轴 {AXIS_LABEL[axis]} 当前位置:").grid(
            row=0, column=column * 2, sticky="w", **pad)
        pw[f'pos_label_{axis}'] = ttk.Label(
            rf, text="0.0 mm", font=("Consolas", 14, "bold"),
            foreground="blue")
        pw[f'pos_label_{axis}'].grid(
            row=0, column=column * 2 + 1, sticky="w", **pad)
    ttk.Label(
        rf, text="设为原点 / 回原点等单轴操作请到各轴自己的页面执行。",
        foreground="#666",
    ).grid(row=1, column=0, columnspan=4, sticky="w", **pad)

    # ── 落地纠偏（独立功能，用户要求显眼）──
    lf = ttk.LabelFrame(parent, text="⚠ 落地纠偏 — 悬空腿落地自动释放旋转电机")
    lf.grid(row=3, column=0, columnspan=2, sticky="ew", **pad)
    pw['land_release_status'] = ttk.Label(
        lf, text="🔒 旋转电机已锁定", foreground="#2e7d32",
        font=("Microsoft YaHei UI", 11, "bold"))
    pw['land_release_status'].grid(
        row=0, column=0, columnspan=3, sticky="w", padx=8, pady=(6, 2))
    ttk.Checkbutton(
        lf, text="启用落地纠偏",
        variable=app.pair_land_release_enabled,
    ).grid(row=1, column=0, sticky="w", padx=8, pady=2)
    ttk.Label(lf, text="落地阈值 (mm):").grid(
        row=1, column=1, sticky="e", padx=(16, 0))
    threshold_spin = ttk.Spinbox(
        lf, from_=0.5, to=20.0, increment=0.5, width=6, format="%.1f",
        textvariable=app.pair_land_release_threshold_mm)
    attach_numeric_input(threshold_spin, app.pair_land_release_threshold_mm)
    threshold_spin.grid(row=1, column=2, sticky="w", padx=4, pady=2)
    ttk.Label(
        lf,
        text=(
            "触发：仅单轴运动（悬空侧“向上”或站立侧“向下”）使两轴标高差"
            "从大于阈值收敛到阈值内（= 悬空腿即将落地承载）；"
            "两轴联动的整体升降不触发。\n"
            "动作：同时释放左右旋转电机（Mr1/Mr2），机构在逐渐承载的"
            "重力下自正一次；软件角度显示不变、不重新校准。\n"
            "恢复：两直线轴都回到空闲后自动重新锁定；急停/断开/关闭"
            "立即锁定。释放期间旋转轴运动会被拒绝。"
        ),
        justify="left", foreground="#666",
    ).grid(row=2, column=0, columnspan=3, sticky="w", padx=8, pady=(0, 6))
