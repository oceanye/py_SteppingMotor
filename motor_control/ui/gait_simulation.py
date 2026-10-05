"""Virtual multi-step controls. Never call executor, serial or save calibration."""
import tkinter as tk

from motor_control.gait_map import map_label, map_pads
from motor_control.gait_simulation import GaitSimulation, simulation_key


def _view(app):
    return (getattr(app, "gait_widgets", None) or {}).get("twin", {})


def _busy(app):
    return bool(getattr(app, "_gait_owned", {})) or _view(app).get("start_busy", False)


def _message(app, text):
    label = (getattr(app, "gait_widgets", None) or {}).get("sim_status")
    if label is not None:
        label.configure(text=text)


def _params(app):
    from .gait_tab import collect_gait_params
    return collect_gait_params(app, app.gait_params)


def cancel_simulation(app, *, pause=False):
    view = _view(app)
    job = view.pop("simulation_job", None)
    if job is not None:
        try:
            app.root.after_cancel(job)
        except tk.TclError:
            pass
    sim = view.get("simulation")
    if sim and sim.active:
        if pause:
            sim.paused = True
        else:
            sim.cancel()
        _message(app, f"模拟已{'暂停' if pause else '取消本步'} · 已完成 {sim.completed_steps} 步")
    button = (getattr(app, "gait_widgets", None) or {}).get("sim_btn")
    if button is not None:
        button.configure(text="▶ 继续模拟" if sim and sim.paused else "▶ 模拟下一步")


def reset_simulation(app):
    if _busy(app):
        return
    from .gait_tab import stop_preview_animation
    from .gait_twin import draw_twin
    view = _view(app)
    try:
        params = _params(app)
        names = {map_label(n): n for n in map_pads()}
        supports = tuple(names[view[f"start_{s}"].get()] for s in ("left", "right"))
        sim = GaitSimulation(params, supports)
    except (ValueError, KeyError, tk.TclError) as exc:
        app.log(f"模拟起步无效：{exc}")
        _message(app, f"模拟未重置：{exc}")
        return
    stop_preview_animation(app)
    cancel_simulation(app)
    view["simulation"] = sim
    app._gait_last_report = None
    app.gait_report_var.set("模拟已重置；单步预览只查看候选，模拟下一步才累计站位")
    view["mode_var"].set("sim")
    _message(app, f"模拟起步：左 {supports[0]} / 右 {supports[1]} · 0 步（不改实机标定）")
    app.log(f"模拟重置：左 {supports[0]} / 右 {supports[1]}；仅虚拟站位，不移动电机")
    draw_twin(app)
    return sim


def preview_from_simulation(app, interactive=True):
    """Return True if this preview was handled against virtual stance."""
    view = _view(app)
    sim = view.get("simulation")
    if sim is None or view["mode_var"].get() == "live":
        return False
    from .gait_tab import stop_preview_animation
    from .gait_twin import draw_twin
    if _busy(app):
        return True
    stop_preview_animation(app)
    cancel_simulation(app)
    try:
        if simulation_key(_params(app)) != simulation_key(sim.params):
            raise ValueError("参数已变化，请重置模拟后再预览")
        side, arc = app.gait_side_var.get(), float(app.gait_arc_var.get())
        report = sim.preview(side, arc)
    except (ValueError, tk.TclError) as exc:
        app._gait_last_report = None
        app.gait_report_var.set(str(exc))
        view["mode_var"].set("plan")
        draw_twin(app)
        return True
    app._gait_last_report = report
    app._gait_plan_context = (report, sim.params, sim.psis)
    app._gait_last_report_key = (side, arc)
    # Include virtual stance so an old physical preview is never reused.
    app._gait_preview_stance = ("simulation", sim.supports, sim.beta)
    app.gait_report_var.set(report.message)
    view["mode_var"].set("plan")
    app.log(f"模拟站位单步预览（未累计）{report.route[0]}→{report.route[1]}：{report.message}")
    draw_twin(app)
    return True


def toggle_simulation(app):
    from .gait_tab import stop_preview_animation
    from .gait_twin import draw_twin
    view = _view(app)
    if _busy(app):
        app.log("实机轴忙，不能启动累计模拟")
        return
    sim = view.get("simulation")
    if sim is None:
        sim = reset_simulation(app)
        if sim is None:
            return
    try:
        if simulation_key(_params(app)) != simulation_key(sim.params):
            raise ValueError("参数已变化，请重置模拟；旧路线不能混用新参数")
        if sim.active:
            if not sim.paused:
                cancel_simulation(app, pause=True)
                return
            sim.paused = False
        else:
            stop_preview_animation(app)
            report = sim.begin(app.gait_side_var.get(), float(app.gait_arc_var.get()))
            app.gait_report_var.set(report.message)
            app.log(f"累计模拟第 {sim.completed_steps+1} 步：{report.route[0]}→{report.route[1]} "
                    f"绕{report.route[2]} · {report.modality}（压缩时间演示，不动电机）")
    except (ValueError, tk.TclError) as exc:
        _message(app, f"模拟未推进：{exc}")
        app.log(f"模拟未推进：{exc}")
        view["mode_var"].set("sim")
        draw_twin(app)
        return
    view["mode_var"].set("sim")
    app.gait_widgets["sim_btn"].configure(text="⏸ 暂停模拟")
    _tick(app)


def _tick(app):
    from .gait_twin import draw_twin, tip_surface_distances
    view = _view(app)
    view.pop("simulation_job", None)
    sim = view.get("simulation")
    if sim is None or not sim.active or sim.paused:
        return
    if getattr(app, "_closing", False) or _busy(app):
        cancel_simulation(app)
        return
    if view["mode_var"].get() != "sim":
        cancel_simulation(app, pause=True)
        return
    done = sim.advance()
    sim.curve.append((sim.frame.time_s, tip_surface_distances(sim.frame.pose, sim.params), sim.segment))
    _message(app, f"模拟 {sim.frame.stage} · 已完成 {sim.completed_steps} 步\n"
                  f"左 {sim.supports[0]} / 右 {sim.supports[1]} · 压缩时间演示，非接触检测")
    draw_twin(app)
    if done:
        app.gait_widgets["sim_btn"].configure(text="▶ 模拟下一步")
        app.log(f"模拟完成 {sim.completed_steps} 步：左 {sim.supports[0]} / 右 {sim.supports[1]}；"
                "下一步从此站位继续，不改实机账本")
    else:
        view["simulation_job"] = app.root.after(30, lambda: _tick(app))


def discard_simulation_step(app):
    from .gait_twin import draw_twin
    cancel_simulation(app)
    draw_twin(app)
