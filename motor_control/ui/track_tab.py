from tkinter import ttk


def build_track_tab(app, parent):
    pad = dict(padx=12, pady=8)
    frame = ttk.LabelFrame(parent, text="轨道 D 直流电机（DRV8871）")
    frame.grid(row=0, column=0, sticky="nsew", **pad)

    ttk.Label(frame, text="PWM 占空比:").grid(row=0, column=0, sticky="w", **pad)
    duty = ttk.Spinbox(frame, from_=1, to=100, increment=1,
                       textvariable=app.v_track_duty, width=8, state="disabled")
    duty.grid(row=0, column=1, sticky="w", **pad)
    ttk.Label(frame, text="%（按住按钮期间有效）").grid(row=0, column=2, sticky="w", **pad)

    fwd = ttk.Button(frame, text="▶ 按住前进", state="disabled")
    fwd.grid(row=1, column=0, **pad, ipadx=18, ipady=12)
    rev = ttk.Button(frame, text="◀ 按住后退", state="disabled")
    rev.grid(row=1, column=1, **pad, ipadx=18, ipady=12)
    stop = ttk.Button(frame, text="■ STOP", command=app._track_release,
                      state="disabled")
    stop.grid(row=1, column=2, **pad, ipadx=18, ipady=12)

    for button, direction in ((fwd, "FWD"), (rev, "REV")):
        button.bind("<ButtonPress-1>",
                    lambda _event, d=direction: app._track_press(d))
        button.bind("<ButtonRelease-1>", lambda _event: app._track_release())
        button.bind("<Leave>", lambda _event: app._track_release())

    ttk.Label(frame, text="状态:").grid(row=2, column=0, sticky="e", **pad)
    status = ttk.Label(frame, textvariable=app.v_track_status,
                       font=("Consolas", 12, "bold"), foreground="#555")
    status.grid(row=2, column=1, columnspan=2, sticky="w", **pad)
    ttk.Label(
        frame,
        text="安全机制：运行命令带 800 ms 租约；按住时约每 250~500 ms 续租，\n"
             "松开、鼠标移出、串口断开或网页停止时立即发送 STOP。",
        foreground="#666",
    ).grid(row=3, column=0, columnspan=3, sticky="w", **pad)

    app.tw.update(duty=duty, fwd_btn=fwd, rev_btn=rev,
                  stop_btn=stop, status_label=status)
