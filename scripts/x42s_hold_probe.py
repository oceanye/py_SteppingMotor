"""Observe a single rotary enable/hold, with NO MOVE/ZERO/config commands.

Diagnostic only: never certifies a nonzero speed as standstill. Strict position
tube remains active; stops if >one speed LSB (0.1 RPM) or position drift. Caller
must use execute_test's verified initial/final ESTOP lifecycle.
"""
import argparse
from collections import Counter
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.x42s_lift_5mm import execute_test, stationary
from scripts.x42s_lift_cycle import read_all
from scripts.x42s_suspended_gait import STOP_FILE

TOL = {1: .72, 2: .225, 3: .72, 4: .225}


def observe_hold(session, recorder, driver, reviewed, *, seconds=12):
    if driver not in (2, 4) or not 1 <= seconds <= 15 or not math.isfinite(seconds):
        raise ValueError('bounded rotary-only hold observation')
    if set(reviewed) != set(TOL) or any(not math.isfinite(x) for x in reviewed.values()):
        raise ValueError('four explicit finite starting coordinates required')
    samples = read_all(session, recorder)
    if any(r.enabled or not stationary(r) or abs(r.measured_position-reviewed[i]) > TOL[i]
           for i, (r, _) in samples.items()):
        raise RuntimeError('reviewed disabled stationary start mismatch')
    origins = {i: r.measured_position for i, (r, _) in samples.items()}
    if session.request_line(f'X42S,{driver},EN,1', timeout=1) != f'OK,X42S,{driver},EN,1':
        raise RuntimeError('enable not exactly acknowledged')
    started, histogram, positions = time.monotonic(), Counter(), []
    while time.monotonic()-started < seconds:
        samples = read_all(session, recorder, pending=(driver,))
        for i, (r, now) in samples.items():
            if i != driver and (r.enabled or not stationary(r)):
                raise RuntimeError('passive axis enable/speed changed')
            if r.measured_position is None:
                if i == driver and now-started < .75:
                    continue
                raise RuntimeError('missing position')
            if abs(r.measured_position-origins[i]) > TOL[i]:
                raise RuntimeError(f'ID{i} position left unchanged strict tube')
            if i == driver:
                if not r.enabled and now-started > .75:
                    raise RuntimeError('enable not verified')
                if r.speed is not None and abs(r.speed) > .100001:
                    raise RuntimeError('hold speed exceeds one raw X-firmware LSB')
                if r.enabled and r.speed is not None and now-r.age_ms/1000 > started:
                    histogram[str(r.speed)] += 1
                    positions.append(r.measured_position)
    if len(positions) < 3:
        raise RuntimeError('not enough valid hold observations')
    return dict(driver_id=driver, no_move_sent=True, not_a_standstill_certificate=True,
                speed_rpm_histogram=dict(histogram), position_min_motor_deg=min(positions),
                position_max_motor_deg=max(positions), duration_s=seconds)


def observe_fixture_lift(session, recorder, driver, reviewed, *, seconds=60, stop_file=STOP_FILE):
    """Isolated lift diagnostic, NOT permission to move with nonzero hold speed.

    No MOVE. Original position tube; stop above observed 0.3 RPM or on missing
    feedback, drift/fault/stop file. Independently supported 5..65 mm fixture.
    Original rotary probe and all gait admission/hold guards are unchanged.
    """
    if (driver not in (1, 3) or not math.isfinite(seconds) or not 1 <= seconds <= 60
            or set(reviewed) != set(TOL) or any(not math.isfinite(v) for v in reviewed.values())):
        raise ValueError('bounded isolated lift observation with four reviewed coordinates required')
    if (any(not 1800-TOL[i] <= reviewed[i] <= 23400+TOL[i] for i in (1, 3))
            or any(abs(reviewed[i]/21.5) > 180 for i in (2, 4)) or stop_file.exists()):
        raise ValueError('fixture pose outside confirmed bounds or active stop request')
    started = None
    histogram, positions = Counter(), []

    def check(i, r, now):
        if stop_file.exists():
            raise RuntimeError('operator stop')
        initial = started is None
        if initial or i != driver:
            if r.enabled or not stationary(r):
                raise RuntimeError(f'passive ID{i} must remain disabled and stationary')
        else:
            if now-started > .75 and (not r.enabled or r.measured_position is None or r.speed is None):
                raise RuntimeError('fresh enabled position/speed missing after enable window')
            if r.speed is not None and (not math.isfinite(r.speed) or abs(r.speed) > .300001):
                raise RuntimeError('isolated hold speed exceeds 0.3 RPM diagnostic cap')
        if r.measured_position is not None and abs(r.measured_position-reviewed[i]) > TOL[i]:
            raise RuntimeError(f'ID{i} left original position tube')

    read_all(session, recorder, check_sample=check)
    if session.request_line(f'X42S,{driver},EN,1', timeout=1) != f'OK,X42S,{driver},EN,1':
        raise RuntimeError('enable not exactly acknowledged')
    started = time.monotonic()
    while time.monotonic()-started < seconds:
        samples = read_all(session, recorder, pending=(driver,), check_sample=check)
        r, now = samples[driver]
        if r.enabled and r.speed is not None and r.measured_position is not None and now-r.age_ms/1000 > started:
            histogram[str(r.speed)] += 1
            positions.append(r.measured_position)
    if len(positions) < 3:
        raise RuntimeError('insufficient isolated hold evidence')
    return dict(driver_id=driver, no_move_sent=True, not_a_standstill_certificate=True,
        independent_support_only=True, gait_guard_unchanged=True, duration_s=seconds,
        speed_rpm_histogram=dict(histogram), position_min_motor_deg=min(positions),
        position_max_motor_deg=max(positions), valid_samples=len(positions))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--driver', type=int, choices=(1, 2, 3, 4), required=True)
    parser.add_argument('--isolated-fixture-lift', action='store_true',
                        help='Explicit 60s supported lift observation; no MOVE, not gait approval')
    parser.add_argument('--reviewed-motor-deg', nargs=4, type=float, required=True)
    parser.add_argument('--supported-enable-confirmed', action='store_true', required=True)
    args = parser.parse_args()
    reviewed = dict(zip(range(1, 5), args.reviewed_motor_deg))
    if args.isolated_fixture_lift:
        if args.driver not in (1, 3):
            parser.error('isolated fixture lift needs ID1 or ID3')
        return execute_test(args.port, args.driver,
            lambda s, r, d: observe_fixture_lift(s, r, d, reviewed),
            purpose='isolated supported lift hold diagnostic, NO MOVE, NOT gait approval',
            event_prefix='bench.fixture_hold', diagnostic_max_bytes=8_000_000,
            test_context={'reviewed_motor_deg': reviewed, 'no_MOVE': True,
                          'duration_s': 60, 'gait_guards_unchanged': True})
    if args.driver not in (2, 4):
        parser.error('original rotary probe only supports ID2/ID4')
    return execute_test(args.port, args.driver, lambda s, r, d: observe_hold(s, r, d, reviewed),
                        purpose='rotary holding-speed diagnostic, no MOVE', event_prefix='bench.hold',
                        test_context={'reviewed_motor_deg': reviewed, 'no_MOVE': True})


if __name__ == '__main__':
    raise SystemExit(main())
