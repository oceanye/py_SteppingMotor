"""Explicit single-lift return inside retained absolute 5..65 mm coordinates.

For an independently supported fixture, NOT homing or automatic recovery.
One invocation retracts <=10 mm; each MOVE <=0.51 mm. All four starting
coordinates must be reviewed. Other axes remain disabled and monitored.
Original travel/endpoint guards plus >=1 s settled arrival; no ZERO/PID/retry.
Default offline. execute_test provides initial/final verified four-axis ESTOP.
"""
import argparse
from dataclasses import asdict
import hashlib
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motor_control.protocol import X42sMoveAck
from motor_control.x42s_motion import MoveExpectation
from scripts.x42s_lift_5mm import execute_test, stationary
from scripts.x42s_lift_cycle import read_all
from scripts.x42s_lift_return import ReturnPlan, return_steps
from scripts.x42s_micro_jog import quantized_target
from scripts.x42s_suspended_gait import STOP_FILE, TOL, SettledArrivalEvidence


def validate(driver, reviewed, plan):
    plan.validate()
    if driver not in (1, 3) or set(reviewed) != {1, 2, 3, 4} or any(
            not math.isfinite(v) for v in reviewed.values()):
        raise ValueError('one lift and four finite reviewed coordinates required')
    if not 5 <= plan.target_mm < plan.expected_start_mm <= 65:
        raise ValueError('return must remain inside absolute 5..65 mm')
    if any(not 1800-TOL[i] <= reviewed[i] <= 23400+TOL[i] for i in (1, 3)):
        raise ValueError('reviewed lift outside interior range')
    if (abs(reviewed[driver]-plan.expected_start_mm*360) > TOL[driver]
            or any(abs(reviewed[i]/21.5) > .5 for i in (2, 4))):
        raise ValueError('reviewed start mismatch or rotary not near original zero')


def waypoints(plan):
    """Slow the final 2 mm ABOVE the retained 5 mm floor, not above zero."""
    plan.validate()
    if not 5 <= plan.target_mm < plan.expected_start_mm <= 65:
        raise ValueError('interior return only')
    current, result = plan.expected_start_mm, []
    while current > plan.target_mm+1e-9:
        if current > 7:
            floor, step, pps = 7., .5, 4000
        elif current > 5.5:
            floor, step, pps = 5.5, .25, 800
        else:
            floor, step, pps = 5., .1, 160
        current = max(plan.target_mm, floor, round(current-step, 9))
        result.append((current, pps))
    return tuple(result)


def run_return(session, recorder, driver, reviewed, plan, *, confirmed=False, stop_file=STOP_FILE):
    validate(driver, reviewed, plan)
    if not confirmed or stop_file.exists():
        raise ValueError('independent support confirmation required; no active stop request')
    enabled, enabling, move = False, False, None
    held = reviewed[driver]

    def check(i, r, now):
        if stop_file.exists():
            raise RuntimeError('operator stop')
        if i != driver:
            if r.enabled or not stationary(r) or abs(r.measured_position-reviewed[i]) > TOL[i]:
                raise RuntimeError(f'passive ID{i} enabled/moving/drifted')
        elif move is None:
            if r.measured_position is not None and abs(r.measured_position-held) > TOL[i]:
                raise RuntimeError('active axis drift during hold/enable; no MOVE')
            if not enabling and (r.enabled != enabled or not r.reached or not stationary(r)):
                raise RuntimeError('stationary held start required')
        elif (not r.enabled or move.outside_travel(r.measured_position)
              or (r.measured_position is not None and not
                  plan.target_mm*360-TOL[i] <= r.measured_position <= plan.expected_start_mm*360+TOL[i])):
            raise RuntimeError(f'ID{i} disabled/outside commanded return: {r.raw}')

    read_all(session, recorder, check_sample=check)
    if session.request_line(f'X42S,{driver},EN,1', timeout=1) != f'OK,X42S,{driver},EN,1':
        raise RuntimeError('enable not exactly acknowledged')
    enabled, enabling, sent = True, True, time.monotonic()
    while True:
        samples = read_all(session, recorder, pending=(driver,), check_sample=check)
        r, now = samples[driver]
        if r.enabled and r.reached and stationary(r) and now-r.age_ms/1000 > sent:
            break
        if now-sent > 3:
            raise RuntimeError('fresh enabled stationary feedback unavailable')
    enabling = False
    points = waypoints(plan)
    recorder.event('motor', 'bench.interior_return_plan', driver_id=driver,
        reviewed_motor_deg=reviewed, plan=asdict(plan), waypoints_mm_pps=points)
    for index, (target_mm, pps) in enumerate(points, 1):
        samples = read_all(session, recorder, check_sample=check)
        origin = samples[driver][0].measured_position
        steps = return_steps(origin, target_mm, plan)
        if stop_file.exists():
            raise RuntimeError('operator stop before MOVE')
        ack = session.request(f'X42S,{driver},MOVE,{steps},{pps}', timeout=1)
        if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (driver, steps, pps):
            raise RuntimeError('MOVE not exactly acknowledged; no retry')
        move = MoveExpectation(object(), quantized_target(origin, steps), time.monotonic(),
            origin_motor_deg=origin, tolerance_motor_deg=TOL[driver])
        evidence = SettledArrivalEvidence()
        deadline = move.acknowledged_at+abs(steps)/pps+5
        progress_at, progress_pos = move.acknowledged_at, origin
        recorder.event('motor', 'bench.interior_return_move', driver_id=driver, index=index,
            origin_motor_deg=origin, target_motor_deg=move.target_motor_deg, steps=steps, speed_pps=pps)
        while True:
            samples = read_all(session, recorder, pending=(driver,), check_sample=check)
            r, now = samples[driver]
            if now > deadline:
                raise RuntimeError('settled endpoint timeout; no retry')
            pos = r.measured_position
            if pos is None:
                evidence.reset()
                if now-move.acknowledged_at > .75:
                    raise RuntimeError('position missing past start blind window')
                continue
            sample_at = now-r.age_ms/1000
            if sample_at > move.acknowledged_at and progress_pos-pos > .1:
                progress_at, progress_pos = sample_at, pos
            elif now-progress_at > 1.25 and abs(pos-move.target_motor_deg) > TOL[driver]:
                raise RuntimeError('no measured progress; NOT a limit measurement')
            if evidence.observe(move, r, now):
                if abs(pos-target_mm*360) > TOL[driver]:
                    raise RuntimeError('absolute endpoint mismatch')
                held, move = pos, None
                recorder.event('motor', 'bench.interior_return_arrived', driver_id=driver,
                    index=index, measured_motor_deg=pos, nominal_motor_deg=target_mm*360)
                print(f'RETURN ID{driver} {index}/{len(points)} measured_mm={pos/360:.6f}', flush=True)
                break
    samples = read_all(session, recorder, check_sample=check)
    return dict(driver_id=driver, plan=asdict(plan), original_zero_unchanged=True,
                positions_motor_deg={i: r.measured_position for i, (r, _) in samples.items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', type=int, choices=(1, 3), required=True)
    parser.add_argument('--reviewed-motor-deg', nargs=4, type=float, required=True)
    parser.add_argument('--expected-start-mm', type=float, required=True)
    parser.add_argument('--target-mm', type=float, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--supported-return-confirmed', action='store_true')
    args = parser.parse_args()
    reviewed = dict(zip(range(1, 5), args.reviewed_motor_deg))
    plan = ReturnPlan(args.expected_start_mm, args.target_mm)
    validate(args.driver, reviewed, plan)
    print(f'PLAN ID{args.driver}: {asdict(plan)}; {len(waypoints(plan))} segments; no ZERO', flush=True)
    if not args.execute:
        return 0
    if not args.port or not args.supported_return_confirmed or STOP_FILE.exists():
        parser.error('port/support confirmation required; no active stop request')
    return execute_test(args.port, args.driver,
        lambda s, r, d: run_return(s, r, d, reviewed, plan, confirmed=True),
        purpose='explicit supported single-lift interior return; NOT homing',
        event_prefix='bench.interior_return', test_context=dict(plan=asdict(plan),
            reviewed_motor_deg=reviewed, no_zero_or_config_writes=True,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))


if __name__ == '__main__':
    raise SystemExit(main())
