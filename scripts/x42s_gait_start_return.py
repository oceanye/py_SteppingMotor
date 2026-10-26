"""Explicit single-lift preparation after an interrupted suspended test.

NOT automatic recovery or homing. Requires four reviewed motor coordinates and
independent support. ID1/3 only, start inside (0.003, 0.5] mm; retract <=0.1 mm
per command at 0.05 mm/s to 0.003 mm ABOVE the retained zero. No ZERO/config,
rotary MOVE, retries, relaxed endpoint tolerances, or physical-limit discovery.
Default offline; each execution uses verified initial/final four-driver ESTOP.
"""
import argparse
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motor_control.protocol import X42sMoveAck
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation
from scripts.x42s_lift_5mm import execute_test, stationary
from scripts.x42s_lift_cycle import read_all, prepare, check_passive
from scripts.x42s_micro_jog import quantized_target

PARK_DEG = 1.08  # 0.003 mm, preserving the original encoder zero
TOL = {1: .72, 2: .225, 3: .72, 4: .225}
STOP_FILE = ROOT / 'logs' / 'x42s-gait-stop.request'


def validate(driver, reviewed):
    if driver not in (1, 3) or set(reviewed) != {1, 2, 3, 4} or any(
            not math.isfinite(v) for v in reviewed.values()):
        raise ValueError('one lift and four finite reviewed motor coordinates required')
    if not PARK_DEG < reviewed[driver] <= 180 or any(
            not 0 <= reviewed[i] <= 180 for i in (1, 3)) or any(
            abs(reviewed[i]/21.5) > .5 for i in (2, 4)):
        raise ValueError('outside explicitly bounded near-bottom preparation window')
    return reviewed[driver]/360


def run_return(session, recorder, driver, reviewed, *, confirmed=False, stop_file=STOP_FILE):
    start = validate(driver, reviewed)
    if not confirmed or stop_file.exists():
        raise ValueError('explicit supported return required; no active stop request')
    # prepare() intentionally only supports start >=0.1 mm; do not bypass it.
    if start < .1:
        raise ValueError('start below supported preparation range')
    samples = read_all(session, recorder)
    if any(r.enabled or not r.reached or not stationary(r)
           or abs(r.measured_position-reviewed[i]) > TOL[i]
           for i, (r, _) in samples.items()):
        raise RuntimeError('reviewed four-axis start changed; no enable/MOVE')
    samples, origins, tolerances = prepare(session, recorder, (driver,), confirmed=True,
        start_mm=start, expected_positions={driver: reviewed[driver]})
    if any(abs(origins[i]-reviewed[i]) > TOL[i] for i in origins):
        raise RuntimeError('reviewed origin changed during preparation; no MOVE')
    # Targets derived once from reviewed origin, never a retry of a missed point.
    targets, current = [], reviewed[driver]
    while current > PARK_DEG+1e-9:
        current = max(PARK_DEG, current-36)
        targets.append(current)
    for index, target in enumerate(targets, 1):
        if stop_file.exists():
            raise RuntimeError('operator stop before MOVE')
        check_passive(samples, origins, tolerances, (driver,))
        reply = samples[driver][0]
        origin = reply.measured_position
        if not reply.enabled or not reply.reached or not stationary(reply):
            raise RuntimeError('fresh stationary active axis required')
        if not PARK_DEG <= target < origin <= reviewed[driver]+TOL[driver]:
            raise RuntimeError('invalid descending segment; no corrective retry')
        steps = -math.floor((origin-target)*3200/360+1e-9)
        if not -327 <= steps < 0 or quantized_target(origin, steps) < target-1e-8:
            raise RuntimeError('segment outside 0.1022 mm cap or command resolution')
        ack = session.request(f'X42S,{driver},MOVE,{steps},160', timeout=1)
        if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (driver, steps, 160):
            raise RuntimeError('MOVE not exactly acknowledged')
        move = MoveExpectation(object(), quantized_target(origin, steps), time.monotonic(),
            origin_motor_deg=origin, tolerance_motor_deg=TOL[driver])
        evidence = ArrivalEvidence()
        deadline = move.acknowledged_at+abs(steps)/160+4
        progress_at, progress_pos = move.acknowledged_at, origin
        recorder.event('motor', 'bench.start_return_move', driver_id=driver, index=index,
            target_motor_deg=move.target_motor_deg, steps=steps, speed_pps=160)
        while True:
            if stop_file.exists():
                raise RuntimeError('operator stop during return')
            samples = read_all(session, recorder, pending=(driver,))
            check_passive(samples, origins, tolerances, (driver,))
            r, now = samples[driver]
            if not r.enabled or now > deadline:
                raise RuntimeError('disabled or endpoint timeout; no retry')
            pos = r.measured_position
            if pos is None:
                evidence.previous = None
                if now-move.acknowledged_at > .75:
                    raise RuntimeError('missing position past blind window')
                continue
            if move.outside_travel(pos) or not 0 < pos <= reviewed[driver]+TOL[driver]:
                raise RuntimeError(f'ID{driver} outside positive return envelope: {r.raw}')
            if progress_pos-pos > .1:
                progress_at, progress_pos = now, pos
            elif now-progress_at > 1.25 and abs(pos-move.target_motor_deg) > TOL[driver]:
                raise RuntimeError('no measured progress; not a bottom measurement')
            if evidence.observe(move, r, now):
                if abs(pos-target) > TOL[driver]:
                    raise RuntimeError('nominal endpoint mismatch')
                recorder.event('motor', 'bench.start_return_arrived', driver_id=driver,
                    index=index, measured_motor_deg=pos, nominal_motor_deg=target)
                print(f'PREPARE ID{driver} {index}/{len(targets)} measured_mm={pos/360:.6f}', flush=True)
                break
    return dict(driver_id=driver, origin_motor_deg=origins[driver],
        measured_motor_deg=samples[driver][0].measured_position,
        parking_target_mm=PARK_DEG/360, original_zero_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', type=int, choices=(1, 3), required=True)
    parser.add_argument('--reviewed-motor-deg', nargs=4, type=float, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--supported-return-confirmed', action='store_true')
    args = parser.parse_args()
    reviewed = dict(zip(range(1, 5), args.reviewed_motor_deg))
    start = validate(args.driver, reviewed)
    if start < .1:
        parser.error('start below supported preparation range')
    print(f'PLAN ID{args.driver} {start:.6f} -> 0.003 mm, 0.05 mm/s, no zero writes', flush=True)
    if not args.execute:
        return 0
    if not args.port or not args.supported_return_confirmed or STOP_FILE.exists():
        parser.error('explicit port/support and no active stop request required')
    return execute_test(args.port, args.driver,
        lambda s, r, d: run_return(s, r, d, reviewed, confirmed=True),
        purpose='explicit bounded single-lift preparation; retained bottom zero',
        event_prefix='bench.start_return',
        test_context=dict(reviewed_motor_deg=reviewed, parking_target_mm=.003,
                          speed_mm_s=.05, independently_supported=True))


if __name__ == '__main__':
    raise SystemExit(main())
