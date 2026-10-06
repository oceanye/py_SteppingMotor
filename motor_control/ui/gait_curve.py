"""Current-step distance display, shared by main and inspection windows."""
from motor_control.gait_curve import (
    CURVE_Y_MIN_MM, CURVE_Y_MAX_MM, EncoderCurveBuffer,
    clip_series, rolling_window, time_at_phi,
)


def sync_live_reference(app, view):
    """Called only on the main view; never sample axes or drive a motor."""
    from .gait_twin import plan_tip_series
    encoder = view.setdefault("encoder_curve", EncoderCurveBuffer())
    run = getattr(app, "_gait_run", None)
    snapshot = view.get("snapshot", {})
    report = getattr(run, "twin_report", None)
    valid = (report is not None and snapshot.get("reference_key") == id(run)
             and snapshot.get("state") not in ("axis_only", "uncalibrated")
             and run.params.geometry == app.gait_params.geometry)
    key = (id(run), snapshot.get("world_key"), run.params) if valid else None
    encoder.bind(key)
    if not valid:
        view.pop("live_curve_reference", None)
        return
    reference = view.get("live_curve_reference")
    if reference is None or reference["key"] != key:
        psis = getattr(run, "twin_psis", (30., 30.))
        series = plan_tip_series(report, run.params, psis)
        reference = {"key": key, "report": report, "series": series, "current_s": 0.0}
        view["live_curve_reference"] = reference
        view.pop("candidate", None)
    pose = snapshot.get("pose")
    if pose is not None:
        reference["current_s"] = time_at_phi(report, [p[0] for p in reference["series"]], pose["phi_deg"])


def curve_payload(app, view):
    from .gait_twin import _mode, _view_params, _plan_psis, plan_tip_series
    from .gait_candidate import current_candidate
    mode = _mode(app, view)
    params = _view_params(app, view, mode)
    candidate = current_candidate(app, view)
    data, measured, grey = [], [], False
    current, clock, status = 0., "本步时间 (s)", "等待选择动作或开始模拟"
    if candidate is not None or mode == "plan":
        report = candidate["report"] if candidate else getattr(app, "_gait_last_report", None)
        if report and report.samples:
            params = candidate["params"] if candidate else params
            psis = candidate["psis"] if candidate else _plan_psis(app, report)
            key = (params, psis)
            if view.get("curve_plan_report") is not report or view.get("curve_plan_key") != key:
                view["curve_plan"] = plan_tip_series(report, params, psis)
                view.update(curve_plan_report=report, curve_plan_key=key)
            data = [(t, tuple(v), 0) for t, v in view["curve_plan"]]
            grey, clock, status = True, "S4模型进度 (s)", "灰色候选参考 · 不动电机"
            if mode == "plan" and candidate is None:  # Old API playback compatibility.
                anim = app.gait_widgets.get("preview_anim") or {}
                frame = anim.get("frame") if anim.get("playing") or anim.get("paused") else None
                index = len(data)-1 if frame is None else max(0, min(frame, len(data)-1))
                current = data[index][0]
    elif mode == "sim":
        sim = view.get("simulation")
        if sim is not None:
            data = [(t, tuple(v), seg) for t, v, seg in sim.curve if seg == sim.segment]
            current = data[-1][0] if data else 0.
            clock, status = "S4模型进度 (s)", "模拟当前步 · 非实测"
    else:
        reference = view.get("live_curve_reference")
        grey, clock, status = True, "S4模型进度 (s)", "灰色=本步计划 · 编码器未接入"
        if reference:
            data = [(t, tuple(v), 0) for t, v in reference["series"]]
            current = reference["current_s"]
            encoder = view.get("encoder_curve")
            if encoder and encoder.step_key == reference["key"]:
                measured = list(encoder.samples)
                if measured:
                    status = "灰色=本步计划 · 彩色=编码器"
            if view.get("snapshot", {}).get("pose") is None:
                status += " · 进度冻结"
    start, end = rolling_window(current)
    return dict(data=clip_series(data, start, end), measured=clip_series(measured, start, end),
                current=current, start=start, end=end, grey=grey, clock=clock,
                status=status, delta=params.geometry.safety_margin_mm)


def draw_curve(app, view):
    from .gait_twin import CURVE_COLORS, _curve_segments
    canvas = view["curve_canvas"]
    canvas.delete("all")
    width, height = canvas.winfo_width(), canvas.winfo_height()
    if width < 60 or height < 130:
        return
    payload = curve_payload(app, view)
    view["curve_display"] = payload  # Read-only diagnostics/tests, not control state.
    left, right, top, bottom = 42, width-10, 87, height-43
    low, high = CURVE_Y_MIN_MM, CURVE_Y_MAX_MM
    px = lambda t: left+(t-payload["start"])/(payload["end"]-payload["start"])*(right-left)
    py = lambda v: top+(high-max(low, min(high, v)))/(high-low)*(bottom-top)
    canvas.create_rectangle(left, py(0), right, bottom, fill="#fff1f2", outline="", tags="curve_negative")
    for v in (low, 0, 50, 100, 150, high):
        canvas.create_line(left, py(v), right, py(v), fill="#b91c1c" if v == 0 else "#e2e8f0",
                           width=2 if v == 0 else 1, tags="curve_zero" if v == 0 else "curve_grid")
        canvas.create_text(left-4, py(v), anchor="e", text=f"{v:g}", fill="#b91c1c" if v == 0 else "#64748b",
                           font=("Microsoft YaHei", 8), tags="curve_y_tick")
    for slot in range(6):
        col, row = slot % 3, slot // 3
        x, y = 8+col*(width-16)/3, 12+row*17
        color = "#9ca3af" if payload["grey"] else CURVE_COLORS[row][col]
        canvas.create_line(x, y, x+10, y, fill=color, width=2, tags="curve_legend")
        canvas.create_text(x+13, y, anchor="w", text=f"{'左' if row == 0 else '右'}{col+1}",
                           fill=color, font=("Microsoft YaHei", 8), tags="curve_legend")
    canvas.create_text(8, 43, anchor="nw", width=width-16, text=payload["status"],
                       fill="#64748b", font=("Microsoft YaHei", 8), tags="curve_source")
    if low < payload["delta"] < high:
        canvas.create_line(left, py(payload["delta"]), right, py(payload["delta"]),
                           fill="#f59e0b", dash=(2, 3), tags="curve_delta")

    def lines(data, *, grey, measured=False):
        for segment in _curve_segments(data, payload["start"]):
            for slot in range(6):
                points = [c for t, vals, _ in segment for c in (px(t), py(vals[slot]))]
                tag = f"curve_{'left' if slot < 3 else 'right'}_{slot%3}"
                canvas.create_line(*points, fill="#b4b4b4" if grey else CURVE_COLORS[slot//3][slot%3],
                                   width=1.5, dash=(4, 3) if grey else (),
                                   tags=(tag, "curve_measured" if measured else "curve_reference" if grey else "curve_model"))
            points = [c for t, vals, _ in segment for c in (px(t), py(min(vals)))]
            canvas.create_line(*points, fill="#737373" if grey else "#111827", width=2,
                               dash=(4, 3) if grey else (), tags="curve_min")
    lines(payload["data"], grey=payload["grey"])
    lines(payload["measured"], grey=False, measured=True)
    values = [v for _, vals, _ in payload["data"]+payload["measured"] for v in vals]
    if values:
        best = min(values)
        canvas.create_text(right-4, top-4, anchor="se", font=("Microsoft YaHei", 8, "bold"),
                           fill="#b91c1c" if best <= 0 else "#475569",
                           text=f"本步窗内最低 {best:.2f}mm", tags="curve_best")
        if min(values) < low or max(values) > high:
            canvas.create_text(left+4, top+5, anchor="nw", width=right-left-8, fill="#b91c1c",
                               text=f"超出固定量程！实际 {min(values):.2f}～{max(values):.2f}mm",
                               tags="curve_overflow")
    else:
        canvas.create_text((left+right)/2, (top+bottom)/2, width=max(30, right-left-12),
                           text="当前步暂无距离数据\n0线以下为爪端与杆投影重叠", fill="#64748b", tags="curve_empty")
    canvas.create_line(px(payload["current"]), top, px(payload["current"]), bottom,
                       fill="#64748b", dash=(3, 3), tags="curve_cursor")
    canvas.create_text(8, height-4, anchor="sw", text=f"当前 {payload['current']:.1f}s",
                       font=("Microsoft YaHei", 8), tags="curve_current_time")
    canvas.create_text(right, height-4, anchor="se", text=f"δ={payload['delta']:g}mm",
                       fill="#b45309", font=("Microsoft YaHei", 8))
    for t, x, anchor in ((payload["start"], left, "w"), (payload["end"], right, "e")):
        canvas.create_text(x, height-27, anchor=anchor, text=f"{t:.0f}s", tags="curve_axis")
    if width >= 350:
        canvas.create_text(width/2, height-27, text=payload["clock"],
                           font=("Microsoft YaHei", 8), tags="curve_clock")
