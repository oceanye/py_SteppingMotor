"""Read-only audit: reproduce the 190 mm mode matrix and summarize saved logs.

Run from the repository: python -B scripts/gait_audit_report.py [session_dir ...]
Prints JSON to stdout; never opens serial ports, alters config or drives motors.
"""
import json
import math
from pathlib import Path
import sys
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motor_control.gait_planner import default_gait_params, plan_gait_stages
from motor_control.gait_simulation import GaitSimulation
from motor_control.gait_coordinates import action_values, rod_side_for_pair
from motor_control.gait_x42s_plan import certify_x42s_stages
from motor_control.gait_map import map_pads
from motor_control.diagnostics import sanitize


def mode_matrix():
    rows = []
    for pair in (("A", "B"), ("B", "A")):
        for key in ("L+", "L-", "R+", "R-"):
            # Site log signs used only in this reproducible example; no file writes.
            sim = GaitSimulation(replace(default_gait_params(), mr1_sign=-1, mr2_sign=-1), pair)
            p = sim.params
            side, arc = action_values(key)
            report = sim.preview(side, arc)
            stages = plan_gait_stages(p, side=side, arc_deg=arc, route=report.route)
            stage = next(s for s in certify_x42s_stages(stages, p, report, .5) if s.stage_id == "S4")
            rows.append(dict(pair=pair, high_rod_side=rod_side_for_pair(pair), action=key,
                             arc_deg=arc, mode=report.modality, route=report.route,
                             ideal_margin_mm=report.min_margin_mm,
                             independent_axis_segments=len(stage.move_groups),
                             rectangle_margin_mm=stage.certified_margin_mm,
                             endpoint_role_order=[m.role for m in stage.move_groups[0]],
                             signed_output_endpoints_deg=stage.sync_endpoints[-1],
                             nominal_S4_s=stage.duration_s))
    centers = map_pads()
    routes = 0
    for left, a in centers.items():
        for right, b in centers.items():
            if not math.isclose(math.dist(a, b), 1):
                continue
            sim = GaitSimulation(default_gait_params(), (left, right))
            for key in ("L+", "L-", "R+", "R-"):
                try:
                    sim.route(*action_values(key))
                    routes += 1
                except ValueError:
                    pass
    return dict(projected_edge_mm=190, leg_radius_mm=4, rod_radius_mm=5, delta_mm=2,
                illustrative_error_budget_mm=.5, mr_signs=[-1, -1],
                note="Offline model only. Runtime recomputes its actual pulse/reference budget. Not hardware proof.",
                geometrically_checked_map_routes=routes, rows=rows)


def session_summary(directory):
    directory = Path(directory)
    meta = json.loads((directory / "session.json").read_text(encoding="utf-8"))
    events = []
    malformed = 0
    for path in directory.glob("gui.jsonl*"):
        for line in path.read_text(encoding="utf-8").splitlines():
            try:
                events.append(json.loads(line))
            except ValueError:
                malformed += 1
    events.sort(key=lambda e: e.get("seq", 0))
    parameters, motion, faults = [], [], []
    for event in events:
        data = event.get("data", {})
        if event.get("event") == "configuration.gait_saved":
            gait = data.get("gait", {})
            geometry = gait.get("geometry", {})
            parameters.append(dict(utc=event.get("utc"), geometry=geometry,
                                   placement=gait.get("initial_placement"),
                                   signs=[gait.get("mr1_sign"), gait.get("mr2_sign")]))
        if event.get("event") != "gui.message":
            continue
        message = data.get("message", "")
        row = dict(utc=event.get("utc"), message=message)
        if any(term in message for term in ("S4", "联动校验", "一键执行完成")):
            motion.append(row)
        if any(term in message for term in ("timeout", "ABORTED", "掉线", "保护故障")):
            faults.append(row)
    return sanitize(dict(session=directory.name, closed=meta.get("closed"),
                         source_commit=meta.get("metadata", {}).get("commit"),
                         gui_records=len(events), malformed=malformed,
                         warning="Missing/rotated GUI records cannot prove an action passed." if not events else None,
                         configuration_snapshots=parameters, motion=motion,
                         fault_event_count=len(faults), fault_examples=faults[:16]))


if __name__ == "__main__":
    print(json.dumps(dict(schema="gait-systematic-audit-v1", model=mode_matrix(),
                          sessions=[session_summary(p) for p in sys.argv[1:]]), ensure_ascii=False, indent=2))
