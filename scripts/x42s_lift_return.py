"""Bounded dual-lift return to a previously recorded bottom coordinate, NOT homing.

Fixed ID1/ID3, lead 1 mm/rev, positive extension. One invocation <=10 mm;
initial position must match on both axes. Above 2 mm: 1.25 mm/s (5 x 0.25).
Last 2 mm: 0.25 mm/s; last 0.5 mm: 0.05 mm/s. No commanded target below 0.
Each MOVE <=0.51 mm; two fresh stationary endpoint samples before next segment.
No ZERO, raw/home commands, PID writes, retries, automatic outward recovery.
Stall means abort, not a new zero. Host checks are NOT mechanical limits or a
communications lease. Rotaries remain disabled; shared lifecycle verifies stop.
"""
import argparse
from dataclasses import dataclass, asdict
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.x42s_lift_5mm import execute_test, stationary, MAX_SEGMENT_PULSES
from scripts.x42s_lift_cycle import prepare, read_all, check_passive
from scripts.x42s_micro_jog import quantized_target
from motor_control.protocol import X42sMoveAck
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation


@dataclass(frozen=True)
class ReturnPlan:
    expected_start_mm: float
    target_mm: float

    def validate(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('finite return parameters required')
        if not .1 <= self.expected_start_mm <= 70 or not 0 <= self.target_mm < self.expected_start_mm:
            raise ValueError('return must stay inside confirmed 0..70 mm; no negative target')
        if not .01 <= self.expected_start_mm-self.target_mm <= 10+1e-9:
            raise ValueError('one invocation must retract 0.01..10 mm nominally')
        return self

    def waypoints(self):
        self.validate()
        current, result = self.expected_start_mm, []
        while current > self.target_mm+1e-9:
            if current > 2:
                floor, step, pps = 2., .5, 4000
            elif current > .5:
                floor, step, pps = .5, .25, 800
            else:
                floor, step, pps = 0., .1, 160
            current = max(self.target_mm, floor, round(current-step, 9))
            result.append((current, pps))
        return tuple(result)


def return_steps(position, target_mm, plan):
    plan.validate()
    if not math.isfinite(position) or not math.isfinite(target_mm):
        raise ValueError('nonfinite segment')
    if (not target_mm*360 < position <= (plan.expected_start_mm+.002)*360
            or not plan.target_mm <= target_mm < plan.expected_start_mm):
        raise ValueError('segment outside descending return plan')
    steps = -math.floor((position-target_mm*360)*3200/360+1e-9)
    if not -MAX_SEGMENT_PULSES <= steps < 0:
        raise ValueError('segment exceeds 0.51 mm or is below command resolution')
    if quantized_target(position, steps) < target_mm*360-1e-8:
        raise ValueError('quantization would command past the target')
    return steps


def run_return(session, recorder, drivers, plan, *, confirmed=False, expected_positions=None):
    """Caller MUST use execute_test's initial/final verified ESTOP lifecycle."""
    plan.validate()
    drivers = tuple(drivers)
    if drivers != (1, 3):
        raise ValueError('both lift axes ID1/ID3 required')
    if expected_positions is not None and (set(expected_positions) != set(drivers) or any(
            not math.isfinite(v) or not (plan.expected_start_mm-.02)*360 <= v <= (plan.expected_start_mm+.002)*360
            for v in expected_positions.values())):
        raise ValueError('reviewed starting coordinates outside small return-start envelope')
    samples, origins, tolerances = prepare(session, recorder, drivers,
        confirmed=confirmed, start_mm=plan.expected_start_mm, expected_positions=expected_positions)
    waypoints = plan.waypoints()
    recorder.event('motor', 'bench.lift_return_plan', **asdict(plan),
        waypoints_mm_pps=waypoints, reviewed_start_motor_deg=expected_positions,
        bottom_is_previous_coordinate=True, host_limits_only=True)
    max_spread_mm = 0.
    for index, (target_mm, pps) in enumerate(waypoints, 1):
        if any(not samples[i][0].enabled or not stationary(samples[i][0]) for i in drivers):
            raise RuntimeError('both axes must be enabled and stationary before group MOVE')
        steps = {i: return_steps(samples[i][0].measured_position, target_mm, plan) for i in drivers}
        expected, evidence, progress = {}, {}, {}
        for i in drivers:
            origin = samples[i][0].measured_position
            ack = session.request(f'X42S,{i},MOVE,{steps[i]},{pps}', timeout=1)
            if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (i, steps[i], pps):
                raise RuntimeError('MOVE not precisely acknowledged; no retry/recovery')
            expected[i] = MoveExpectation(object(), quantized_target(origin, steps[i]), time.monotonic(),
                origin_motor_deg=origin, tolerance_motor_deg=tolerances[i])
            evidence[i] = ArrivalEvidence()
            progress[i] = (expected[i].acknowledged_at, origin)
        deadline = time.monotonic()+max(abs(v)/pps for v in steps.values())+4
        while True:
            samples = read_all(session, recorder, pending=drivers)
            check_passive(samples, origins, tolerances, drivers)
            if time.monotonic() > deadline:
                raise RuntimeError('return arrival timeout; no retry/recovery')
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
                        or not max(-.002, plan.target_mm-.002)*360 <= pos <= (plan.expected_start_mm+.002)*360):
                    raise RuntimeError(f'ID{i} disabled/outside return envelope: {r.raw}')
                sample_at = now-r.age_ms/1000
                _, old = progress[i]
                if sample_at > e.acknowledged_at and old-pos > .1:
                    progress[i] = (sample_at, pos)
                if abs(pos-e.target_motor_deg) > tolerances[i] and now-progress[i][0] > 1.25:
                    raise RuntimeError('no measured progress; NOT proof of bottom; no retry')
                if not evidence[i].observe(e, r, now):
                    complete = False
                elif abs(pos-target_mm*360) > tolerances[i]:
                    raise RuntimeError('absolute endpoint mismatch; no retry')
            positions = [samples[i][0].measured_position for i in drivers]
            if all(v is not None for v in positions):
                spread = (max(positions)-min(positions))/360
                max_spread_mm = max(max_spread_mm, spread)
                if spread > .2:
                    raise RuntimeError('pair position spread exceeds 0.2 mm; no retry')
            if complete:
                recorder.event('motor', 'bench.lift_return_waypoint', index=index,
                    target_mm=target_mm, positions_motor_deg=positions, speed_pps=pps,
                    max_observed_spread_mm=max_spread_mm)
                print(f'WAYPOINT {index}/{len(waypoints)} target_mm={target_mm:.3f} '
                      f'measured_mm={[round(p/360, 6) for p in positions]} '
                      f'speed_mm_s={pps/3200:.2f} max_spread_mm={max_spread_mm:.5f}', flush=True)
                break
    return {'drivers': drivers, 'plan': asdict(plan),
            'positions_motor_deg': {i: samples[i][0].measured_position for i in drivers},
            'max_observed_spread_mm': max_spread_mm}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--expected-start-mm', type=float, required=True)
    parser.add_argument('--target-mm', type=float, required=True)
    parser.add_argument('--recorded-bottom-confirmed', action='store_true', required=True)
    parser.add_argument('--supported-return-confirmed', action='store_true', required=True)
    parser.add_argument('--reviewed-id1-deg', type=float)
    parser.add_argument('--reviewed-id3-deg', type=float)
    args = parser.parse_args()
    try:
        plan = ReturnPlan(args.expected_start_mm, args.target_mm).validate()
    except ValueError as exc:
        parser.error(str(exc))
    if (args.reviewed_id1_deg is None) != (args.reviewed_id3_deg is None):
        parser.error('both explicitly reviewed starting coordinates are required together')
    expected = None if args.reviewed_id1_deg is None else {1: args.reviewed_id1_deg, 3: args.reviewed_id3_deg}
    if expected is not None and any(not math.isfinite(v) or not (plan.expected_start_mm-.02)*360 <= v
                                   <= (plan.expected_start_mm+.002)*360 for v in expected.values()):
        parser.error('reviewed coordinates outside small return-start envelope')
    return execute_test(args.port, (1, 3),
        lambda s, r, d: run_return(s, r, d, plan, confirmed=True, expected_positions=expected),
        purpose='bounded dual-lift return to previous bottom coordinate; NOT homing',
        test_context={**asdict(plan), 'bulk_speed_mm_s': 1.25, 'near_bottom_speed_mm_s': .25,
                      'final_speed_mm_s': .05, 'no_zero_write': True, 'reviewed_start_motor_deg': expected})


if __name__ == '__main__':
    raise SystemExit(main())
