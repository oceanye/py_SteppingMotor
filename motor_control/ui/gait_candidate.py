"""Four action buttons produce a grey landing candidate without side effects."""
import math
import tkinter as tk

from motor_control.gait_simulation import GaitSimulation, simulation_key
from motor_control.gait_planner import plan_swing_trajectory
from motor_control.gait_twin import pad_center


def stance_key(app, source):
    view = app.gait_widgets["twin"]
    sim = view.get("simulation")
    if source == "sim" and sim is not None:
        return (id(sim), sim.supports, sim.segment)
    return (getattr(app, "_gait_supports", app.gait_params.initial_supports),
            getattr(app, "_gait_beta_deg", app.gait_params.initial_beam_deg),
            getattr(app, "_gait_world_epoch", 0),
            tuple(view[f"start_{s}"].get() for s in ("left", "right")))


def select_candidate(app):
    from .gait_tab import collect_gait_params, stop_preview_animation
    from .gait_twin import _mode, draw_twin
    view = app.gait_widgets["twin"]
    sim = view.get("simulation")
    if getattr(app, "_gait_owned", {}) or view.get("start_busy") or (sim and sim.active):
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
        app.gait_report_var.set("灰色落点候选（未移动） · " + report.message)
    except (ValueError, KeyError, tk.TclError) as exc:
        app.gait_report_var.set(f"候选未生成：{exc}")
    draw_twin(app)


def current_candidate(app, view):
    from .gait_twin import _mode
    candidate = view.get("candidate")
    sim = view.get("simulation")
    if (not candidate or getattr(app, "_gait_owned", {}) or view.get("start_busy")
            or (sim and sim.active) or candidate["source"] != _mode(app, view)
            or candidate["stance"] != stance_key(app, candidate["source"])):
        return None
    return candidate


def draw_candidate(app, view):
    from .gait_twin import lattice_d_mm
    candidate = current_candidate(app, view)
    if candidate is None or "project" not in view:
        return
    report, params = candidate["report"], candidate["params"]
    sample = report.samples[-1]
    slot = 0 if report.side == "left" else 1
    psi = sample.psi_deg + candidate["psis"][slot] - 30
    d, project, canvas = lattice_d_mm(params), view["project"], view["canvas"]
    center = (sample.center[0]/d, sample.center[1]/d)
    x, y = project(center)
    for leg in range(3):
        angle = math.radians(psi+120*leg)
        arm = params.geometry.arm_length_mm/d
        end = (center[0]+arm*math.cos(angle), center[1]+arm*math.sin(angle))
        canvas.create_line(x, y, *project(end), fill="#9ca3af", width=3, dash=(4, 3),
                           tags="candidate_ghost")
    canvas.create_line(x, y, *project(pad_center(report.route[2])), fill="#9ca3af",
                       width=2, dash=(4, 3), tags="candidate_ghost")
    canvas.create_oval(x-6, y-6, x+6, y+6, outline="#6b7280", width=2, tags="candidate_ghost")
    canvas.create_text(x, y+20, text="候选落点", fill="#6b7280", tags="candidate_ghost")
    canvas.create_text(8, 58, anchor="nw", width=canvas.winfo_width()-16,
                       text="灰色候选 · " + ("待执行确认" if report.feasible else "校验未通过，禁止执行"),
                       fill="#6b7280" if report.feasible else "#b91c1c", tags="candidate_ghost")
