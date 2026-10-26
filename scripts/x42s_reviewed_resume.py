"""ONE reviewed suspended HIGH continuation, pinned to the 18:02 abort record.

Not a generic restart, automatic fault recovery, homing, or continuous-step
acceptance. Keep original S4 absolute references and certified rectangles;
finish interrupted group 33, then the remaining S4/S5B/S6 stages. No speed,
hold, endpoint or travel tolerance changes. New failures always abort.
Default offline; live use requires renewed explicit operator authorization.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motor_control.gait_executor import GaitExecutor
from motor_control.gait_planner import parse_gait_params
from scripts.x42s_lift_5mm import execute_test
from scripts.x42s_suspended_gait import SuspendedHost, TOL, STOP_FILE, build_plan

# Deliberately one reviewed record. A different interruption requires a NEW
# review, not editing live coordinates or an unchecked '--skip-groups' flag.
SOURCE_SESSION = '20261023T095519-0245f374ef'
EVENTS_SHA256 = '49130acce981ee041ed9ecac8ffe9f59598a8744f853b257463866fcaaf0ffb4'
ORIGINAL_ORIGINS = {1: 1800., 2: 4.6, 3: 1800.1, 4: .2}
PARKED = {1: 1799.9, 2: -420.8, 3: 23399.9, 4: -595.2}
LAST_VERIFIED = {2: -425.1, 4: -594.6}
INTERRUPTED_TARGET = {2: -404.3, 4: -598.2}
PREFIX_GROUPS = 102
S4_COMPLETED = 32


def validate_start(reviewed):
    if set(reviewed) != {1, 2, 3, 4} or any(not math.isfinite(v) for v in reviewed.values()):
        raise ValueError('four finite reviewed coordinates required')
    if any(abs(reviewed[i]-PARKED[i]) > TOL[i] for i in reviewed):
        raise ValueError('not at independently reviewed parked pose; no automatic recovery')
    if any(abs(reviewed[i]-target) > TOL[i] for i, target in ((1, 1800.), (3, 23400.))):
        raise ValueError('held lifts no longer at the original S4 height')
    for i in (2, 4):
        low, high = sorted((LAST_VERIFIED[i], INTERRUPTED_TARGET[i]))
        if not low-TOL[i] <= reviewed[i] <= high+TOL[i]:
            raise ValueError('rotary pose outside original interrupted command rectangle')


def remaining_stages(stages):
    index = next(i for i, s in enumerate(stages) if s.stage_id == 'S4')
    stage = stages[index]
    if (len(stage.move_groups) != 170 or len(stage.sync_endpoints) != 170
            or stage.execution_profile != 'x42s_bounded_progress'
            or not stage.certified_margin_mm or stage.certified_margin_mm <= 0):
        raise ValueError('not the reviewed certified HIGH trajectory')
    for n, reference in ((S4_COMPLETED-1, LAST_VERIFIED), (S4_COMPLETED, INTERRUPTED_TARGET)):
        for driver, endpoint in zip((2, 4), stage.sync_endpoints[n]):
            if abs(ORIGINAL_ORIGINS[driver]+endpoint*21.5-reference[driver]) > TOL[driver]:
                raise ValueError('original cumulative rotary targets changed')
    rest = replace(stage, move_groups=stage.move_groups[S4_COMPLETED:],
        sync_endpoints=stage.sync_endpoints[S4_COMPLETED:],
        group_durations=stage.group_durations[S4_COMPLETED:],
        duration_s=sum(stage.group_durations[S4_COMPLETED:]))
    return (rest, *stages[index+1:])


def load_source():
    """Require immutable reviewed evidence and exactly reproducible current plan."""
    directory = ROOT / 'logs' / 'diagnostics' / SOURCE_SESSION
    raw = (directory / 'events.jsonl').read_bytes()
    if hashlib.sha256(raw).hexdigest() != EVENTS_SHA256:
        raise ValueError('reviewed source events missing/changed; no motion')
    metadata = json.loads((directory / 'session.json').read_text(encoding='utf-8'))
    if (not metadata['closed'] or metadata['health']['dropped']
            or metadata['health']['write_errors'] or metadata['health']['rotations']['events']):
        raise ValueError('source evidence not closed/intact')
    for name, digest in metadata['metadata']['test_context']['bench_source_sha256'].items():
        if Path(name).name != name or hashlib.sha256((ROOT / 'scripts' / name).read_bytes()).hexdigest() != digest:
            raise ValueError('original bench implementation changed; re-review required')
    events = [json.loads(line) for line in raw.decode('utf-8').splitlines() if line]
    source = next(e['data'] for e in events if e['event'] == 'bench.gait_plan')
    params = parse_gait_params(json.loads((ROOT / '.gait_params.json').read_text(encoding='utf-8')))
    p, report, stages = build_plan(params, 'L+', profile='interior')
    normalized = lambda value: json.loads(json.dumps(value, ensure_ascii=False))
    if (source['params'] != normalized(p.as_document())
            or source['stages'] != normalized([asdict(s) for s in stages])):
        raise ValueError('current plan/config differs from the original; no motion')
    remaining_stages(stages)
    return p, report, stages


class ReviewedResumeHost(SuspendedHost):
    def __init__(self, session, recorder, reviewed, *, stop_file=STOP_FILE):
        validate_start(reviewed)
        super().__init__(session, recorder, ORIGINAL_ORIGINS, profile='interior', stop_file=stop_file)
        # The fixed source origin is NOT replaced with today's stopped angles.
        # reviewed/held are admission checks; origins is the original trajectory.
        self.reviewed = dict(reviewed)
        self.held = dict(reviewed)
        self.heights = {1: 5., 3: 65.}

    def prepare(self):
        super().prepare()
        validate_start(self.origins)  # fresh enabled pose must still be eligible
        current = dict(self.origins)
        self.origins = dict(ORIGINAL_ORIGINS)
        self.recorder.event('motor', 'bench.reviewed_resume_reference',
            source_session=SOURCE_SESSION, original_motor_deg=self.origins,
            current_motor_deg=current, current_lift_mm=self.heights,
            original_reference_preserved=True, continuous_full_step=False)


def run_resume(session, recorder, _, p, report, stages, reviewed):
    rest = remaining_stages(stages)
    host = ReviewedResumeHost(session, recorder, reviewed)
    host.prepare()
    host.verify_hold()
    executor = GaitExecutor(host, p, rest, side=report.side)
    recorder.event('motor', 'bench.reviewed_resume_plan', source_session=SOURCE_SESSION,
        original_verified_groups=PREFIX_GROUPS, remaining_groups=sum(len(s.move_groups) for s in rest),
        stages=[asdict(s) for s in rest], continuous_full_step=False, no_guard_relaxation=True)
    while executor.current_stage() is not None:
        stage = executor.current_stage()
        host.stage = stage.stage_id
        if host.cancelled():
            raise RuntimeError('operator stop before stage')
        if stage.is_motion_stage:
            if not executor.execute_current_stage(progress=lambda sid, n, total:
                    host.log(f'RESUME {sid} {n}/{total}') if n % 5 == 0 or n == total else None):
                raise RuntimeError(executor.last_error)
        else:
            host.log(f'RESUME {stage.stage_id}: 架空流程检查点，不证明接触或落脚')
            if not executor.advance_confirm():
                raise RuntimeError('confirmation stage failed')
    samples = host.snapshot()
    return dict(source_session=SOURCE_SESSION, continuous_full_step=False,
        resumed_groups_completed=host.groups_completed,
        combined_verified_groups=PREFIX_GROUPS+host.groups_completed,
        positions_motor_deg={i: r.measured_position for i, (r, _) in samples.items()},
        max_endpoint_error_motor_deg=host.max_error, contact_and_avoidance_not_physically_verified=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reviewed-motor-deg', type=float, nargs=4, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--reviewed-suspended-continuation-confirmed', action='store_true')
    args = parser.parse_args()
    reviewed = dict(zip(range(1, 5), args.reviewed_motor_deg))
    validate_start(reviewed)
    p, report, stages = load_source()
    rest = remaining_stages(stages)
    print(json.dumps(dict(source_session=SOURCE_SESSION, original_verified_groups=PREFIX_GROUPS,
        remaining_groups={s.stage_id: len(s.move_groups) for s in rest},
        cumulative_final_rotary_output_deg=rest[0].sync_endpoints[-1],
        current_reviewed_motor_deg=reviewed, continuous_full_step=False), ensure_ascii=False), flush=True)
    if not args.execute:
        return 0
    if not args.port or not args.reviewed_suspended_continuation_confirmed or STOP_FILE.exists():
        parser.error('renewed reviewed continuation/support confirmation and clear stop state required')
    return execute_test(args.port, (1, 2, 3, 4),
        lambda s, r, d: run_resume(s, r, d, p, report, stages, reviewed),
        purpose='explicitly reviewed suspended HIGH continuation; NOT continuous full step',
        event_prefix='bench.reviewed_resume', diagnostic_max_bytes=32_000_000,
        test_context=dict(source_session=SOURCE_SESSION, source_events_sha256=EVENTS_SHA256,
            reviewed_motor_deg=reviewed, original_origins=ORIGINAL_ORIGINS,
            no_zero_or_guard_changes=True, continuous_full_step=False,
            resume_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))


if __name__ == '__main__':
    raise SystemExit(main())
