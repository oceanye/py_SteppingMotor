"""One operator-authorized positive lift stage, not automatic stroke discovery.

Fixed bench mapping ID1/3, lead 1 mm/rev, direct drive. Start must match the
expected bottom-referenced position on BOTH axes before enable. Each invocation
advances at most 10 mm nominally, in <=0.51 mm commands at 0.25 mm/s. Leaves
5 mm below the operator-declared safe height (never above absolute 70 mm).
65 -> 70 mm requires explicit safe height >=75 mm, confirmed by the operator.
No reverse, zero, PID/current writes or retries. Stall is failure, not a limit.
Shared lifecycle verifies four-axis ESTOP before/after, including exceptions.
Host-only checks cannot detect screw disengagement or guarantee loss-of-PC stop.
"""
import argparse
from dataclasses import dataclass, asdict
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.x42s_lift_5mm import execute_test, stationary, SPEED_PPS, MAX_SEGMENT_PULSES
from scripts.x42s_lift_cycle import prepare, read_all, check_passive
from scripts.x42s_micro_jog import quantized_target
from motor_control.protocol import X42sMoveAck
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation


@dataclass(frozen=True)
class StagePlan:
    expected_start_mm: float
    target_mm: float
    confirmed_safe_height_mm: float

    def validate(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('finite stage parameters required')
        if not 10 <= self.confirmed_safe_height_mm <= 75:
            raise ValueError('operator-declared safe height must be 10..75 mm')
        if (not 5 <= self.expected_start_mm <= 65
                or not self.expected_start_mm < self.target_mm <= min(70, self.confirmed_safe_height_mm-5)):
            raise ValueError('positive interior stage required; reserve 5 mm below safe height; target <=70 mm')
        if not .01 <= self.target_mm-self.expected_start_mm <= 10+1e-9:
            raise ValueError('one invocation must advance 0.01..10 mm nominally')
        return self

    def waypoints(self):
        self.validate()
        count = math.ceil((self.target_mm-self.expected_start_mm-1e-9)/.5)
        return tuple(min(self.target_mm, self.expected_start_mm+.5*i) for i in range(1, count+1))


def stage_steps(position, target_mm, plan):
    plan.validate()
    if not math.isfinite(position) or not math.isfinite(target_mm):
        raise ValueError('nonfinite segment')
    if (not (plan.expected_start_mm-.002)*360 <= position < target_mm*360
            or not plan.expected_start_mm < target_mm <= plan.target_mm):
        raise ValueError('segment outside positive stage')
    steps = math.floor((target_mm*360-position)*3200/360+1e-9)
    if not 0 < steps <= MAX_SEGMENT_PULSES:
        raise ValueError('segment exceeds 0.51 mm or is below command resolution')
    return steps


def run_stage(session, recorder, drivers, plan, *, confirmed=False):
    """Caller MUST use execute_test's initial/final verified ESTOP lifecycle."""
    plan.validate()
    drivers = tuple(drivers)
    if drivers != (1, 3):
        raise ValueError('this stage requires BOTH lift axes, fixed ID1 and ID3')
    samples, origins, tolerances = prepare(session, recorder, drivers,
        confirmed=confirmed, start_mm=plan.expected_start_mm)
    waypoints = plan.waypoints()
    max_spread_mm = 0.
    recorder.event('motor', 'bench.lift_stage_plan', **asdict(plan),
                   waypoints_mm=waypoints, speed_mm_s=.25, host_limits_only=True)
    for index, target_mm in enumerate(waypoints, 1):
        if any(not samples[i][0].enabled or not stationary(samples[i][0]) for i in drivers):
            raise RuntimeError('all active axes must be enabled and stationary before group MOVE')
        steps = {i: stage_steps(samples[i][0].measured_position, target_mm, plan) for i in drivers}
        expected, evidence, progress = {}, {}, {}
        for i in drivers:
            origin = samples[i][0].measured_position
            ack = session.request(f'X42S,{i},MOVE,{steps[i]},{SPEED_PPS}', timeout=1)
            if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (i, steps[i], SPEED_PPS):
                raise RuntimeError('MOVE not precisely acknowledged; no retry/return')
            expected[i] = MoveExpectation(object(), quantized_target(origin, steps[i]), time.monotonic(),
                origin_motor_deg=origin, tolerance_motor_deg=tolerances[i])
            evidence[i] = ArrivalEvidence()
            progress[i] = (expected[i].acknowledged_at, origin)
        deadline = time.monotonic()+6.1
        while True:
            samples = read_all(session, recorder, pending=drivers)
            check_passive(samples, origins, tolerances, drivers)
            if time.monotonic() > deadline:
                raise RuntimeError('stage arrival timeout; no retry/return')
            complete = True
            for i in drivers:
                r, now = samples[i]
                e = expected[i]
                if r.measured_position is None:
                    evidence[i].previous = None
                    if now-e.acknowledged_at > .75:
                        raise RuntimeError('position missing beyond start window')
                    complete = False
                    continue
                pos = r.measured_position
                if (not r.enabled or e.outside_travel(pos)
                        or not (plan.expected_start_mm-.002)*360 <= pos <= (plan.target_mm+.002)*360):
                    raise RuntimeError(f'ID{i} disabled/outside stage envelope: {r.raw}')
                sample_at = now-r.age_ms/1000
                _, old = progress[i]
                if sample_at > e.acknowledged_at and pos-old > .1:
                    progress[i] = (sample_at, pos)
                if abs(pos-e.target_motor_deg) > tolerances[i] and now-progress[i][0] > 1.25:
                    raise RuntimeError('no measured progress; NOT a mechanical limit')
                if not evidence[i].observe(e, r, now):
                    complete = False
                elif abs(pos-target_mm*360) > tolerances[i]:
                    raise RuntimeError('absolute waypoint residual too large; no retry')
            positions = [samples[i][0].measured_position for i in drivers]
            if all(v is not None for v in positions):
                spread = (max(positions)-min(positions))/360
                max_spread_mm = max(max_spread_mm, spread)
                if spread > .2:
                    raise RuntimeError('pair position spread exceeds 0.2 mm')
            if complete:
                recorder.event('motor', 'bench.lift_stage_waypoint', index=index,
                    target_mm=target_mm, positions_motor_deg=positions,
                    max_observed_spread_mm=max_spread_mm)
                print(f'WAYPOINT {index}/{len(waypoints)} target_mm={target_mm:.3f} '
                      f'measured_mm={[round(p/360, 6) for p in positions]} '
                      f'max_spread_mm={max_spread_mm:.5f}', flush=True)
                break
    return {'drivers': drivers, 'plan': asdict(plan),
            'positions_motor_deg': {i: samples[i][0].measured_position for i in drivers},
            'max_observed_spread_mm': max_spread_mm}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--expected-start-mm', type=float, required=True)
    parser.add_argument('--target-mm', type=float, required=True)
    parser.add_argument('--confirmed-safe-height-mm', type=float, required=True)
    parser.add_argument('--positive-extension-confirmed', action='store_true', required=True)
    parser.add_argument('--support-confirmed', action='store_true', required=True)
    args = parser.parse_args()
    try:
        plan = StagePlan(args.expected_start_mm, args.target_mm, args.confirmed_safe_height_mm).validate()
    except ValueError as exc:
        parser.error(str(exc))
    return execute_test(args.port, (1, 3),
        lambda s, r, d: run_stage(s, r, d, plan, confirmed=True),
        purpose='one bounded positive dual-lift stage; not homing/full gait',
        test_context=asdict(plan))


if __name__ == '__main__':
    raise SystemExit(main())
