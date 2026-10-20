"""Side-coloured dashed landing forecasts, independent of current position."""
import math
import tkinter as tk

from motor_control.gait_simulation import GaitSimulation, simulation_key
from motor_control.gait_planner import plan_swing_trajectory
from motor_control.gait_twin import pad_center


def stance_key(app, source):
    view = app.gait_widgets["twin"]
    sim = view.get("simulation")
    if source == "sim" and sim is not None:
        return (id(sim), sim.supports, sim.segment,
                tuple(view[f"start_{s}"].get() for s in ("left", "right")))
    return (getattr(app, "_gait_supports", app.gait_params.initial_supports),
            getattr(app, "_gait_beta_deg", app.gait_params.initial_beam_deg),
            getattr(app, "_gait_world_epoch", 0),
            tuple(view[f"start_{s}"].get() for s in ("left", "right")))


def select_candidate(app):
    from .gait_tab import collect_gait_params, stop_preview_animation
    from .gait_twin import _mode, draw_twin
    view = app.gait_widgets["twin"]
    sim = view.get("simulation")
    from .gait_simulation import _busy
    if _busy(app) or (sim and sim.active):
        return
    source = _mode(app, view)
    if source == "plan":  # Compatibility-only old preview state, not a GUI mode.
        source = "sim" if sim else "live"
        view["mode_var"].set(source)
    view.pop("candidate", None)
    stop_preview_animation(app)
    try:
        params = collect_gait_params(app, app.gait_params)
        side, arc = app.gait_side_var.get(), float(app.gait_arc_var.get())
        if source == "sim":
            if sim is None:
                from motor_control.gait_map import map_label, map_pads
                names = {map_label(n): n for n in map_pads()}
                supports = tuple(names[view[f"start_{s}"].get()] for s in ("left", "right"))
                basis = GaitSimulation(params, supports)
            else:
                if simulation_key(params) != simulation_key(sim.params):
                    raise ValueError("参数已变化，请重置模拟")
                from motor_control.gait_map import map_label
                if tuple(view[f"start_{s}"].get() for s in ("left", "right")) != tuple(map_label(n) for n in sim.origin):
                    raise ValueError("起步选择尚未应用，请点击【应用模拟起点】")
                basis = sim
            report, psis = basis.preview(side, arc), basis.psis
        else:
            route_provider = getattr(app, "_gait_landing_route", None)
            if route_provider:
                route = route_provider(side, params, arc_deg=arc)
                if route is None:
                    raise ValueError("当前方向落点超出5×5地图")
            else:  # Isolated UI hosts, still no physical state is changed.
                route = GaitSimulation(params, getattr(app, "_gait_supports", None)).route(side, arc)
            report = plan_swing_trajectory(params, side=side, route=route, arc_deg=arc)
            pose = view.get("snapshot", {}).get("pose")
            psis = tuple(pose["feet"][s]["psi_deg"] for s in ("left", "right")) if pose else (30., 30.)
        view["candidate"] = {"report": report, "params": params, "psis": psis,
                             "source": source, "stance": stance_key(app, source)}
        app.gait_report_var.set("虚线落点候选（未移动） · " + report.message)
    except (ValueError, KeyError, tk.TclError) as exc:
        app.gait_report_var.set(f"候选未生成：{exc}")
        app.log(f"候选未生成：{exc}")
    draw_twin(app)


def current_candidate(app, view):
    from .gait_twin import _mode
    from .gait_simulation import _busy
    candidate = view.get("candidate")
    sim = view.get("simulation")
    if (not candidate or _busy(app)
            or (sim and sim.active) or candidate["source"] != _mode(app, view)
            or candidate["stance"] != stance_key(app, candidate["source"])):
        return None
    return candidate


def draw_candidate(app, view):
    from .gait_twin import _mode, lattice_d_mm
    from .gait_paths import FOOT_COLORS
    candidate = current_candidate(app, view)
    tag, caption = "candidate_ghost", "候选落点"
    if candidate is None:
        mode, sim = _mode(app, view), view.get("simulation")
        if mode == "sim" and sim is not None and sim.active:
            candidate = {"report": sim.report, "params": sim.params, "psis": sim.step_psis}
        elif mode == "live" and getattr(app, "_gait_owned", {}) and view.get("live_curve_reference"):
            run = app._gait_run
            candidate = {"report": run.twin_report, "params": run.params, "psis": run.twin_psis}
        tag, caption = "forecast_ghost", "预计落点"
    if candidate is None or "project" not in view:
        return
    report, params = candidate["report"], candidate["params"]
    if not report.samples:
        return
    sample = report.samples[-1]
    slot = 0 if report.side == "left" else 1
    psi = sample.psi_deg + candidate["psis"][slot] - 30
    d, project, canvas = lattice_d_mm(params), view["project"], view["canvas"]
    center = (sample.center[0]/d, sample.center[1]/d)
    color = FOOT_COLORS[report.side]
    # At the endpoint the dashed target would just cover the current solid
    # foot, adding clutter without information. Keep only the real/current body.
    if tag == "forecast_ghost":
        sim = view.get("simulation") if _mode(app, view) == "sim" else None
        pose = sim.frame.pose if sim else view.get("snapshot", {}).get("pose")
        if pose:
            foot = pose["feet"][report.side]
            aligned = abs((foot["psi_deg"]-psi+60) % 120-60) < .1
            if math.dist(foot["center"], center) < 1e-5 and aligned:
                return
    x, y = project(center)
    for leg in range(3):
        angle = math.radians(psi+120*leg)
        arm = params.geometry.arm_length_mm/d
        end = (center[0]+arm*math.cos(angle), center[1]+arm*math.sin(angle))
        canvas.create_line(x, y, *project(end), fill=color, width=2, dash=(4, 3), tags=tag)
    canvas.create_line(x, y, *project(pad_center(report.route[2])), fill=color,
                       width=1.2, dash=(4, 3), tags=tag)
    canvas.create_oval(x-6, y-6, x+6, y+6, outline=color, width=2, dash=(4, 3), tags=tag)
    canvas.create_text(x, y+20, text=caption, fill=color, tags=tag)
    canvas.create_text(8, 58, anchor="nw", width=canvas.winfo_width()-16,
                       text=caption + "（虚线，非当前位置） · " + ("左足" if report.side == "left" else "右足")
                            + ("" if report.feasible else " · 校验未通过，禁止执行"),
                       fill=color if report.feasible else "#b91c1c", tags=tag)
