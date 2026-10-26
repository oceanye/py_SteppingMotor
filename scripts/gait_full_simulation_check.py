"""Offline full-stage/map audit. NO serial, enable, MOVE or config writes.

Check the current 190 mm profile, every selected adjacent start/action, and
24 cumulative out-and-back steps. JSON/CSV are explicitly labelled model data,
not motor feedback or physical collision validation. Output files are exclusive
creates, never replacements. Use --all-map for all 25-cell map starts.
"""
import argparse
from collections import Counter
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motor_control.gait_coordinates import action_values
from motor_control.gait_map import map_pads
from motor_control.gait_planner import parse_gait_params, plan_gait_stages
from motor_control.gait_simulation import GaitSimulation
from motor_control.gait_x42s_plan import certify_x42s_stages

ACTIONS = ('L+', 'L-', 'R+', 'R-')
STAGES = ('S0', 'S1', 'S2', 'S2B', 'S4', 'S5', 'S5B', 'S6', 'S7')


def require(condition, description):
    if not condition:
        raise AssertionError(description)


def check_step(sim, action, *, csv_rows=None):
    before = (sim.supports, sim.beta, sim.psis, sim.completed_steps, sim.elapsed_s)
    side, arc = action_values(action)
    report = sim.begin(side, arc)
    require(report.feasible and report.min_margin_mm > 0, 'nonpositive model clearance')
    stages = certify_x42s_stages(plan_gait_stages(sim.params, side=side, arc_deg=arc,
                                route=report.route), sim.params, report, .5)
    require(tuple(s.stage_id for s in stages) == STAGES, 'full stage order mismatch')
    phase_ends = {}
    previous_time = before[4]
    pivot_side = 'right' if side == 'left' else 'left'
    pivot_center = map_pads()[report.route[2]]
    for frame in sim.frames:
        require(math.isfinite(frame.time_s) and frame.time_s >= previous_time, 'nonmonotonic model clock')
        previous_time = frame.time_s
        feet = frame.pose['feet']
        require(math.dist(feet[pivot_side]['center'], pivot_center) < 1e-8, 'support center moved')
        require(abs(math.dist(feet['left']['center'], feet['right']['center'])-1) < 1e-8,
                'beam length changed')
        for foot in feet.values():
            require(-1e-8 <= foot['z_mm'] <= sim.params.lift_mm+1e-8, 'lift outside planned range')
            require(math.isfinite(foot['psi_deg']), 'nonfinite world phase')
        phase_ends[frame.stage.split()[0]] = frame.pose
    require(tuple(phase_ends) == STAGES, 'display frame stage order mismatch')
    for stage, heights in (('S2', (70., 70.)), ('S2B', (0., 70.) if side == 'left' else (70., 0.)),
                           ('S5B', (70., 70.)), ('S7', (0., 0.))):
        actual = tuple(phase_ends[stage]['feet'][s]['z_mm'] for s in ('left', 'right'))
        require(actual == heights, f'{stage} displayed lift coordinates differ: {actual}')
    swing = next(s for s in stages if s.stage_id == 'S4')
    require(swing.certified_margin_mm > 0, 'independent X42S rectangle not clear')
    require(all(math.isfinite(x) for endpoint in swing.sync_endpoints for x in endpoint), 'nonfinite endpoint')
    final_offsets = dict(zip((m.role for m in swing.move_groups[-1]), swing.sync_endpoints[-1]))
    start_frame = sim.frames[0]
    require(all(abs(start_frame.pose['feet'][s]['psi_deg']-psi) < 1e-8
                for s, psi in zip(('left', 'right'), before[2])), 'phase jumped at next step start')
    while sim.active:
        done = sim.advance()
        if not done:
            require(sim.supports == before[0] and sim.completed_steps == before[3], 'early stance commit')
    require(sim.completed_steps == before[3]+1, 'step failed to commit exactly once')
    require(sim.supports == ((report.route[1], report.route[2]) if side == 'left'
                            else (report.route[2], report.route[1])), 'wrong landing cells')
    require(abs(sim.beta-(before[1]-arc)) < 1e-7, 'beam angle lost continuity')
    swing_index = 0 if side == 'left' else 1
    support_role = 'Mr2' if side == 'left' else 'Mr1'
    swing_role = 'Mr1' if side == 'left' else 'Mr2'
    swing_sign = sim.params.mr1_sign if side == 'left' else sim.params.mr2_sign
    support_sign = sim.params.mr2_sign if side == 'left' else sim.params.mr1_sign
    require(abs(-final_offsets[support_role]*support_sign-(sim.beta-before[1])) < 1e-7,
            'support motor / beam delta mismatch')
    require(abs(sim.psis[swing_index]-before[2][swing_index]-final_offsets[swing_role]*swing_sign
                -(sim.beta-before[1])) < 1e-7, 'swing motor / world phase mismatch')
    require(abs(sim.psis[1-swing_index]-before[2][1-swing_index]) < 1e-7, 'support world phase changed')
    require(all(abs((psi-30+60) % 120-60) < 1e-7 for psi in sim.psis), 'landing not low-node phase')
    if csv_rows is not None:
        for sample in report.samples:
            csv_rows.append(dict(action=action, modality=report.modality, source='offline_model_not_feedback',
                                 **asdict(sample)))
    return dict(start_pair=before[0], action=action, modality=report.modality, route=report.route,
                final_pair=sim.supports, final_beta_deg=sim.beta, final_world_psis_deg=sim.psis,
                ideal_lower_bound_mm=report.min_margin_mm,
                x42s_rectangle_lower_bound_mm=swing.certified_margin_mm,
                x42s_rotary_groups=len(swing.move_groups), final_offsets_output_deg=final_offsets,
                nominal_step_s=sim.elapsed_s-before[4], stage_sequence=STAGES)


def audit(params, *, all_map=False, progress=None):
    require(params.geometry.arm_length_mm == 190 and params.lift_mm == 70
            and params.trajectory_mode == 'two_mode_v1', 'audit expects current 190 mm / 70 mm two-mode profile')
    centers = map_pads()
    pairs = [(a, b) for a, pa in centers.items() for b, pb in centers.items()
             if math.isclose(math.dist(pa, pb), 1)] if all_map else [params.initial_supports]
    rows, curves, failures, rejected = [], [], [], 0
    for pair in pairs:
        for action in ACTIONS:
            sim = GaitSimulation(params, pair)
            try:
                sim.route(*action_values(action))
            except ValueError:
                try:
                    sim.begin(*action_values(action))
                    raise AssertionError('out-of-map action accepted')
                except ValueError:
                    require(sim.supports == pair and not sim.active and sim.completed_steps == 0,
                            'rejected map action changed state')
                    rejected += 1
                continue
            try:
                rows.append(check_step(sim, action))
            except Exception as exc:
                failures.append(dict(pair=pair, action=action, error=f'{type(exc).__name__}: {exc}'))
        if progress:
            progress(len(rows), rejected, len(failures))
    # Save the current site's four ideal curves separately from all-map coverage.
    local = [check_step(GaitSimulation(params), action, csv_rows=curves) for action in ACTIONS]
    sim = GaitSimulation(params)
    sequence = ('L+', 'L-', 'L-', 'L+', 'R+', 'R-', 'R-', 'R+') * 3
    cumulative, q, winding_peak = [], {'Mr1': 0., 'Mr2': 0.}, 0.
    for action in sequence:
        row = check_step(sim, action)
        for role, delta in row['final_offsets_output_deg'].items():
            q[role] += delta
        # Track every ideal S4 sample, not just endpoint equivalent phases.
        report = sim.report
        sr, pr = ('Mr1', 'Mr2') if report.side == 'left' else ('Mr2', 'Mr1')
        signs = {'Mr1': params.mr1_sign, 'Mr2': params.mr2_sign}
        origins = {r: q[r]-row['final_offsets_output_deg'][r] for r in q}
        winding_peak = max(winding_peak, *(abs(origins[sr]+s.swing_q_delta_deg*signs[sr]) for s in report.samples),
                           *(abs(origins[pr]+s.support_q_delta_deg*signs[pr]) for s in report.samples))
        row.update(step=sim.completed_steps, cumulative_output_deg=dict(q), elapsed_model_s=sim.elapsed_s)
        cumulative.append(row)
    require(sim.supports == params.initial_supports and all(abs(v) < 1e-7 for v in q.values()),
            '24-step out-and-back did not return to initial stance/joints')
    require(winding_peak <= params.rotation_limit_deg+1e-7, 'sequence crosses configured cable window')
    return dict(schema='gait-full-offline-audit-v1', hardware_motion=False,
                warning='All trajectories/clearances/times are model data; no contact or collision measurement.',
                illustrative_x42s_error_budget_mm=.5, parameters=params.as_document(),
                summary=dict(adjacent_ordered_starts=len(pairs), legal_actions_passed=len(rows),
                    out_of_map_actions_rejected=rejected, failures=len(failures),
                    modalities=dict(Counter(r['modality'] for r in rows)),
                    cumulative_steps=sim.completed_steps, cumulative_winding_peak_deg=winding_peak),
                failures=failures, local_four_actions=local, all_map_cases=rows,
                cumulative_steps=cumulative), curves


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--all-map', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--curve-csv', type=Path)
    args = parser.parse_args()
    raw = (ROOT / '.gait_params.json').read_bytes()
    params = parse_gait_params(json.loads(raw.decode('utf-8')))
    milestones = set()
    def progress(passed, rejected, failed):
        milestone = (passed+rejected)//40
        if milestone not in milestones:
            milestones.add(milestone)
            print(f'OFFLINE: passed={passed}, map_rejected={rejected}, failed={failed}', flush=True)
    result, rows = audit(params, all_map=args.all_map, progress=progress)
    result['input_sha256'] = hashlib.sha256(raw).hexdigest()
    result['source_sha256'] = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        for name in ('scripts/gait_full_simulation_check.py', 'motor_control/gait_simulation.py',
                     'motor_control/gait_planner.py', 'motor_control/gait_x42s_plan.py')}
    require((ROOT / '.gait_params.json').read_bytes() == raw, 'site config changed during audit')
    if args.output:
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    if args.curve_csv:
        with args.curve_csv.open('x', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(result['summary'], ensure_ascii=False), flush=True)
    return 2 if result['failures'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
