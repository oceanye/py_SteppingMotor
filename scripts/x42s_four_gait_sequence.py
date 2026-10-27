"""Four continuous suspended gait actions with ONE retained mechanical reference.

L+ -> L- -> R+ -> R-; dynamic route selection covers HIGH and LOW and returns
to the initial virtual stance and nominal rotary coordinates. Lifts 5..65 mm
(60 mm travel), NOT standard 70 mm acceptance or real loaded field walking.
No re-zero, driver configuration changes, automatic retries or error recovery.
Paired lifts <=0.4 mm, full independent progress envelope <=0.5 mm. Rotary
subsegments <=4 degrees inside the existing certified rectangles. 2x conservative
bench command caps, unchanged slow floor approach and >=1 second settled arrival.
"""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motor_control.gait_coordinates import action_values
from motor_control.gait_executor import GaitExecutor
from motor_control.gait_planner import parse_gait_params
from motor_control.gait_simulation import GaitSimulation
from scripts.x42s_lift_5mm import execute_test
from scripts.x42s_suspended_gait import build_plan, SuspendedHost, SCALES, TOL, STOP_FILE

ACTIONS = ('L+', 'L-', 'R+', 'R-')


@dataclass(frozen=True)
class SequenceStep:
    action: str
    params: object
    report: object
    stages: tuple
    start_rotary: dict
    end_rotary: dict
    supports_before: tuple
    supports_after: tuple


def build_sequence(params):
    p, _, _ = build_plan(params, ACTIONS[0], profile='interior', speed_scale=2)
    sim = GaitSimulation(p)
    original = sim.supports
    offsets, plans = {2: 0., 4: 0.}, []
    for action in ACTIONS:
        if any(abs((psi-30+60) % 120-60) > 1e-7 for psi in sim.psis):
            raise ValueError('unsupported non-equivalent tripod phase; no implicit phase reset')
        before, start = sim.supports, dict(offsets)
        side, arc = action_values(action)
        report = sim.begin(side, arc)
        pp, reviewed_report, stages = build_plan(params, action, profile='interior', speed_scale=2,
            route=report.route, rotary_segment_deg=4, paired_lift_step_mm=.4)
        if pp != p or reviewed_report.route != report.route or not report.feasible:
            raise ValueError('simulation/execution route or parameter mismatch')
        swing = next(s for s in stages if s.stage_id == 'S4')
        drivers = [2 if m.role == 'Mr1' else 4 for m in swing.move_groups[0]]
        for endpoint in swing.sync_endpoints:
            if any(abs(start[i]+q) > 180+1e-7 for i, q in zip(drivers, endpoint)):
                raise ValueError('cumulative rotary sweep exceeds previously cleared +/-180 degrees')
        for i, q in zip(drivers, swing.sync_endpoints[-1]):
            offsets[i] = start[i]+q
        while sim.active:
            sim.advance()
        plans.append(SequenceStep(action, pp, reviewed_report, stages, start,
                                  dict(offsets), before, sim.supports))
    if (sim.supports != original or any(abs(v) > 1e-7 for v in offsets.values())
            or {s.report.modality for s in plans} != {'high_junction', 'low_junction'}):
        raise ValueError('four-action sequence must cover both modes and return to initial stance')
    return tuple(plans)


def validate_start(reviewed, reference):
    for values in (reviewed, reference):
        if set(values) != set(SCALES) or any(not math.isfinite(v) for v in values.values()):
            raise ValueError('four finite reviewed readings and retained zero reference required')
    for i in SCALES:
        expected = reference[i]+(1800 if i in (1, 3) else 0)
        if abs(reviewed[i]-expected) > TOL[i]:
            raise ValueError(f'ID{i} not at retained 5 mm / rotary zero start; no automatic parking')


def run_sequence(session, recorder, _, plans, reviewed, reference, *, confirmed=False, stop_file=STOP_FILE, reference_provenance=None, short_endpoint_checks=False):
    validate_start(reviewed, reference)
    if not confirmed or stop_file.exists() or tuple(s.action for s in plans) != ACTIONS:
        raise ValueError('explicit reviewed four-action interior sequence required')
    reference_kind = 'operator_confirmed_manual'
    if reference_provenance is not None:
        if (reference_provenance.get('reference_kind') != 'operator_reviewed_powercycle_transfer'
                or reference_provenance.get('inferred_zero_raw_motor_deg') != reference
                or reference_provenance.get('rotary_alignment_visually_confirmed') is not True
                or reference_provenance.get('absolute_position_independently_verified') is not False):
            raise ValueError('reviewed restart provenance and visual rotary alignment required')
        reference_kind = reference_provenance['reference_kind']
        recorder.event('motor', 'bench.sequence_reference_transfer', **reference_provenance)
    host = SuspendedHost(session, recorder, reviewed, profile='interior', zero_reference=reference,
                         speed_scale=2, rotary_segment_deg=4, paired_progress_guard=True, stop_file=stop_file,
                         reference_kind=reference_kind, short_endpoint_checks=short_endpoint_checks)
    host.prepare()
    completed = []
    for index, step in enumerate(plans, 1):
        host.stage = f'{index}:{step.action}:preflight'
        if any(abs(host.heights[i]-5) > 1e-7 for i in (1, 3)):
            raise RuntimeError('previous gait did not return nominal lifts to 5 mm')
        samples = host.ready_snapshot()
        expected = {i: reference[i]+(1800 if i in (1, 3) else step.start_rotary[i]*SCALES[i]) for i in SCALES}
        if any(abs(samples[i][0].measured_position-expected[i]) > TOL[i] for i in SCALES):
            raise RuntimeError('retained cumulative start mismatch; no re-anchor or next step')
        # A nominal step command anchor is NOT a new mechanical zero. Raw
        # feedback stays untouched and every target remains tied to reference.
        for i in (2, 4):
            host.origins[i] = expected[i]
        host.verify_hold()
        recorder.event('motor', 'bench.sequence_step_plan', step=index, action=step.action,
            modality=step.report.modality, route=step.report.route, supports_before=step.supports_before,
            supports_after=step.supports_after, start_rotary_output_deg=step.start_rotary,
            end_rotary_output_deg=step.end_rotary, mechanical_zero_raw_motor_deg=reference,
            reference_kind=reference_kind,
            stages=[asdict(s) for s in step.stages], virtual_map_not_measured_world_pose=True,
            standard_70mm_acceptance=False)
        executor = GaitExecutor(host, step.params, step.stages, side=step.report.side)
        while executor.current_stage() is not None:
            stage = executor.current_stage()
            host.stage = f'{index}:{step.action}:{stage.stage_id}'
            if host.cancelled():
                raise RuntimeError('operator stop before stage')
            if stage.is_motion_stage:
                if not executor.execute_current_stage(progress=lambda sid, n, total:
                        host.log(f'SEQUENCE {index}/4 {step.action} {sid} {n}/{total}')
                        if n % 10 == 0 or n == total else None):
                    raise RuntimeError(executor.last_error)
                host.verify_stage_boundary()
            elif not executor.advance_confirm():
                raise RuntimeError('suspended-only checkpoint failed')
        samples = host.ready_snapshot()
        expected = {i: reference[i]+(1800 if i in (1, 3) else step.end_rotary[i]*SCALES[i]) for i in SCALES}
        if any(abs(samples[i][0].measured_position-expected[i]) > TOL[i] for i in SCALES):
            raise RuntimeError('nominal cumulative endpoint mismatch; no next gait')
        completed.append(step.action)
        recorder.event('motor', 'bench.sequence_step_completed', step=index, action=step.action,
            modality=step.report.modality, raw_motor_deg={i:r.measured_position for i,(r,_) in samples.items()},
            mechanical_zero_raw_motor_deg=reference, nominal_cumulative_rotary_output_deg=step.end_rotary)
        host.log(f'COMPLETE SUSPENDED GAIT {index}/4 {step.action} {step.report.modality}')
    return dict(completed_actions=completed, lift_range_mm=[5, 65], suspended_only=True,
                reference_kind=reference_kind, reference_provenance=reference_provenance,
                short_endpoint_checks=short_endpoint_checks,
                standard_70mm_acceptance=False, field_contact_avoidance_verified=False,
                mechanical_zero_raw_motor_deg=reference, groups_completed=host.groups_completed,
                raw_motor_deg={i:r.measured_position for i,(r,_) in samples.items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--reviewed-raw-motor-deg', nargs=4, type=float)
    parser.add_argument('--manual-zero-motor-deg', nargs=4, type=float)
    parser.add_argument('--supported-interior-four-actions-confirmed', action='store_true')
    args = parser.parse_args()
    params = parse_gait_params(json.loads((ROOT/'.gait_params.json').read_text(encoding='utf8')))
    plans = build_sequence(params)
    print(json.dumps([dict(action=s.action, modality=s.report.modality, route=s.report.route,
        end_rotary=s.end_rotary, groups={p.stage_id:len(p.move_groups) for p in s.stages})
        for s in plans], ensure_ascii=False), flush=True)
    if not args.execute:
        return 0
    if not (args.port and args.reviewed_raw_motor_deg and args.manual_zero_motor_deg
            and args.supported_interior_four_actions_confirmed) or STOP_FILE.exists():
        parser.error('explicit reviewed start, retained reference, interior support and clear stop required')
    reviewed = dict(zip(range(1, 5), args.reviewed_raw_motor_deg))
    reference = dict(zip(range(1, 5), args.manual_zero_motor_deg))
    validate_start(reviewed, reference)
    return execute_test(args.port, (1, 2, 3, 4),
        lambda s, r, d: run_sequence(s, r, d, plans, reviewed, reference, confirmed=True),
        purpose='four continuous suspended interior gait stages; NOT standard70mm or field walking acceptance',
        event_prefix='bench.sequence', diagnostic_max_bytes=64_000_000,
        test_context=dict(actions=ACTIONS, reviewed_raw_motor_deg=reviewed,
            mechanical_zero_raw_motor_deg=reference, lift_range_mm=[5, 65], speed_scale=2,
            paired_lift_segment_mm=.4, rotary_segment_deg=4, standard_70mm_acceptance=False,
            source_sha256={n:hashlib.sha256((ROOT/'scripts'/n).read_bytes()).hexdigest()
                for n in ('x42s_four_gait_sequence.py','x42s_suspended_gait.py','x42s_lift_5mm.py')}))


if __name__ == '__main__':
    raise SystemExit(main())
