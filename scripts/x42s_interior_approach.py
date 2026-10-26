"""Explicit positive-only single-lift approach to retained coordinate 5 mm.

Supported suspended fixture only; reviewed four-axis coordinates required.
ID1/3, 1 mm lead, <=0.51 mm per command at 0.25 mm/s. No ZERO, reverse,
retry or PID/config writes. The small already-reviewed negative start is NOT
a new zero. Original endpoint/travel tolerances and verified ESTOP apply.
Default offline; operator stop file is shared with suspended gait.
"""
import argparse
import hashlib
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motor_control.protocol import X42sMoveAck
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation
from scripts.x42s_lift_5mm import execute_test, stationary
from scripts.x42s_lift_cycle import read_all
from scripts.x42s_micro_jog import quantized_target
from scripts.x42s_suspended_gait import TOL, STOP_FILE


def validate(driver, reviewed):
    if driver not in (1, 3) or set(reviewed) != {1, 2, 3, 4} or any(
            not math.isfinite(v) for v in reviewed.values()):
        raise ValueError('one lift and four finite reviewed coordinates required')
    if not -.72 <= reviewed[driver] <= 180 or any(
            not -.72 <= reviewed[i] <= 1800.72 for i in (1, 3)) or any(
            abs(reviewed[i]/21.5) > .5 for i in (2, 4)):
        raise ValueError('not in reviewed positive-approach start window')


def run_approach(session, recorder, driver, reviewed, *, confirmed=False, stop_file=STOP_FILE):
    validate(driver, reviewed)
    if not confirmed or stop_file.exists():
        raise ValueError('independent support/positive approach confirmation required')
    enabled, enabling, move = False, False, None
    def check(i, r, now):
        if stop_file.exists():
            raise RuntimeError('operator stop')
        if i != driver:
            if r.enabled or not stationary(r) or abs(r.measured_position-reviewed[i]) > TOL[i]:
                raise RuntimeError(f'passive ID{i} enabled/moving/drifted')
        elif move is None:
            if r.measured_position is not None and abs(r.measured_position-reviewed[i]) > TOL[i]:
                raise RuntimeError('active axis drift during enable; no MOVE')
            if not enabling and (r.enabled != enabled or not r.reached or not stationary(r)):
                raise RuntimeError('stationary reviewed start required')
        elif not r.enabled or move.outside_travel(r.measured_position):
            raise RuntimeError(f'ID{i} disabled/outside commanded approach: {r.raw}')
    samples = read_all(session, recorder, check_sample=check)
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
    for index in range(1, 11):
        if stop_file.exists():
            raise RuntimeError('operator stop before MOVE')
        r = samples[driver][0]
        origin, target = r.measured_position, index*180.
        if not r.enabled or not r.reached or not stationary(r) or not origin < target <= 1800:
            raise RuntimeError('invalid positive approach point; no retry')
        steps = math.floor((target-origin)*3200/360+1e-9)
        if not 0 < steps <= 1632 or quantized_target(origin, steps) > target+1e-8:
            raise RuntimeError('command exceeds positive 0.51 mm cap')
        ack = session.request(f'X42S,{driver},MOVE,{steps},800', timeout=1)
        if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (driver, steps, 800):
            raise RuntimeError('MOVE not exactly acknowledged')
        move = MoveExpectation(object(), quantized_target(origin, steps), time.monotonic(),
            origin_motor_deg=origin, tolerance_motor_deg=TOL[driver])
        evidence = ArrivalEvidence()
        deadline = move.acknowledged_at+steps/800+4
        progress_at, progress_pos = move.acknowledged_at, origin
        recorder.event('motor', 'bench.interior_approach_move', driver_id=driver, index=index,
            origin_motor_deg=origin, target_motor_deg=move.target_motor_deg, steps=steps, speed_pps=800)
        while True:
            samples = read_all(session, recorder, pending=(driver,), check_sample=check)
            r, now = samples[driver]
            if now > deadline:
                raise RuntimeError('endpoint timeout; no retry')
            pos = r.measured_position
            if pos is None:
                evidence.previous = None
                if now-move.acknowledged_at > .75:
                    raise RuntimeError('position missing past start blind window')
                continue
            if pos-progress_pos > .1:
                progress_at, progress_pos = now, pos
            elif now-progress_at > 1.25 and abs(pos-move.target_motor_deg) > TOL[driver]:
                raise RuntimeError('no measured progress; not a limit measurement')
            if evidence.observe(move, r, now):
                if abs(pos-target) > TOL[driver]:
                    raise RuntimeError('absolute endpoint mismatch')
                recorder.event('motor', 'bench.interior_approach_arrived', driver_id=driver,
                    index=index, measured_motor_deg=pos, nominal_motor_deg=target)
                print(f'APPROACH ID{driver} {index}/10 measured_mm={pos/360:.6f}', flush=True)
                break
    return dict(driver_id=driver, measured_motor_deg=samples[driver][0].measured_position,
                original_zero_unchanged=True, parking_target_mm=5.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', type=int, choices=(1, 3), required=True)
    parser.add_argument('--reviewed-motor-deg', nargs=4, type=float, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--supported-positive-approach-confirmed', action='store_true')
    args = parser.parse_args()
    reviewed = dict(zip(range(1, 5), args.reviewed_motor_deg))
    validate(args.driver, reviewed)
    print(f'PLAN ID{args.driver}: {reviewed[args.driver]/360:.6f} -> 5 mm; no zero writes', flush=True)
    if not args.execute:
        return 0
    if not args.port or not args.supported_positive_approach_confirmed or STOP_FILE.exists():
        parser.error('port/support confirmation required; no active stop request')
    return execute_test(args.port, args.driver,
        lambda s, r, d: run_approach(s, r, d, reviewed, confirmed=True),
        purpose='explicit outward approach for 5..65 mm suspended gait',
        event_prefix='bench.interior_approach', test_context=dict(reviewed_motor_deg=reviewed,
            parking_target_mm=5., no_zero_or_config_writes=True,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))


if __name__ == '__main__':
    raise SystemExit(main())
