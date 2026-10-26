"""ONE operator-reviewed first return after the 2026-10-26 power cycle.

ID3 only: inferred 64.999164 mm -> 60 mm, <=0.51 mm per MOVE at 0.5 mm/s.
Raw driver coordinates stay raw. Historical coordinates are ESTIMATES based on
the operator's unchanged-pose confirmation, not retained absolute feedback.
No homing, ZERO, PID writes, GUI calibration changes, rotary motion, automatic
retry, or bottom approach. This is not a generic power-loss recovery mechanism.
Default offline. A fresh support/clearance confirmation is required to execute.
"""
import argparse
import hashlib
import json
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
from scripts.x42s_micro_jog import quantized_target
from scripts.x42s_suspended_gait import STOP_FILE, TOL, SettledArrivalEvidence

BEFORE_SESSION = '20261023T102135-68eb265462'
BEFORE_SHA256 = '5e0dcb8377f8301161bdbfe255b8aaecf4a3d3a0c2178205ed5d7558aff7fe3f'
AFTER_SESSION = '20261026T020214-94c622361f'
AFTER_SHA256 = '875551ac4bb35782dff7b774de24a175bb1cb87ed3810633c1bc39fabcfe2fc6'
HISTORICAL = {1: 1800., 2: -409.8, 3: 23399.699, 4: -596.9}
REBOOT_RAW = {i: 0. for i in range(1, 5)}
TARGET_MM = 60.
SPEED_PPS = 1600  # 0.5 mm/s, confirmed direct-drive lead 1 mm/rev


def estimated_motor_deg(driver, raw):
    if driver not in HISTORICAL or not math.isfinite(raw):
        raise ValueError('known driver and finite raw position required')
    return HISTORICAL[driver] + raw - REBOOT_RAW[driver]


def check_source(directory, digest, expected):
    raw = (directory / 'events.jsonl').read_bytes()
    if hashlib.sha256(raw).hexdigest() != digest:
        raise ValueError('reviewed evidence changed/missing; re-review required')
    metadata = json.loads((directory / 'session.json').read_text(encoding='utf-8'))
    health = metadata['health']
    if not metadata['closed'] or health['dropped'] or health['write_errors'] or health['rotations']['events']:
        raise ValueError('incomplete reviewed evidence')
    positions = {}
    for line in raw.decode('utf-8').splitlines():
        event = json.loads(line)
        row = event.get('data', {})
        if (event['event'] == 'x42s.telemetry' and row.get('online')
                and row.get('measured_position') is not None and row.get('age_ms', 99999) <= 500
                and row.get('speed') == 0 and not row.get('fault')):
            positions[row['driver_id']] = row['measured_position']
    if positions != expected:
        raise ValueError('reviewed endpoint differs from pinned reference')


def validate_evidence():
    check_source(ROOT / 'logs' / 'diagnostics' / BEFORE_SESSION, BEFORE_SHA256, HISTORICAL)
    check_source(ROOT / 'logs' / 'diagnostics' / AFTER_SESSION, AFTER_SHA256, REBOOT_RAW)


def waypoints():
    return tuple(64.5 - .5 * n for n in range(10))


def segment_steps(raw_origin, estimated_target_mm):
    if (not math.isfinite(raw_origin) or estimated_target_mm not in waypoints()
            or not TARGET_MM * 360 - TOL[3] <= estimated_motor_deg(3, raw_origin) <= HISTORICAL[3] + TOL[3]):
        raise ValueError('outside reviewed first-return interval')
    target_raw = estimated_target_mm * 360 - HISTORICAL[3] + REBOOT_RAW[3]
    delta = target_raw - raw_origin
    steps = -math.floor(abs(delta) * 3200 / 360 + 1e-9)
    if delta >= 0 or not -1632 <= steps < 0:
        raise ValueError('only <=0.51 mm inward segments permitted')
    return steps, target_raw


def run_return(session, recorder, _, *, confirmed=False, stop_file=STOP_FILE):
    """execute_test owns initial/final verified ESTOP, including exceptions."""
    if not confirmed or stop_file.exists():
        raise ValueError('unchanged pose and renewed independent support required; stop must be clear')
    enabled, enabling, move = False, False, None
    held = REBOOT_RAW[3]

    def check(i, r, now):
        if stop_file.exists():
            raise RuntimeError('operator stop')
        if i != 3:
            if r.enabled or not stationary(r) or abs(r.measured_position - REBOOT_RAW[i]) > TOL[i]:
                raise RuntimeError(f'passive ID{i} enabled/moving/drifted')
        elif move is None:
            if r.measured_position is not None and abs(r.measured_position - held) > TOL[i]:
                raise RuntimeError('raw start/hold changed; do not re-anchor automatically')
            if not enabling and (r.enabled != enabled or not r.reached or not stationary(r)):
                raise RuntimeError('stationary held start required')
        elif (not r.enabled or move.outside_travel(r.measured_position)
              or (r.measured_position is not None and not
                  TARGET_MM * 360 - TOL[i] <= estimated_motor_deg(i, r.measured_position) <= HISTORICAL[i] + TOL[i])):
            raise RuntimeError(f'ID{i} outside reviewed return: {r.raw}')

    read_all(session, recorder, check_sample=check)
    recorder.event('motor', 'bench.powercycle_reference',
        historical_motor_deg=HISTORICAL, reboot_raw_motor_deg=REBOOT_RAW,
        operator_confirmed_no_movement=True, absolute_position_independently_verified=False,
        historical_session=BEFORE_SESSION, reboot_session=AFTER_SESSION,
        target_estimated_mm=TARGET_MM, speed_pps=SPEED_PPS)
    if session.request_line('X42S,3,EN,1', timeout=1) != 'OK,X42S,3,EN,1':
        raise RuntimeError('enable not exactly acknowledged')
    enabled, enabling, sent = True, True, time.monotonic()
    while True:
        samples = read_all(session, recorder, pending=(3,), check_sample=check)
        r, now = samples[3]
        if r.enabled and r.reached and stationary(r) and now - r.age_ms / 1000 > sent:
            break
        if now - sent > 3:
            raise RuntimeError('fresh stationary enable unavailable')
    enabling = False

    for index, target_mm in enumerate(waypoints(), 1):
        samples = read_all(session, recorder, check_sample=check)
        origin = samples[3][0].measured_position
        steps, nominal_raw = segment_steps(origin, target_mm)
        if stop_file.exists():
            raise RuntimeError('operator stop before MOVE')
        ack = session.request(f'X42S,3,MOVE,{steps},{SPEED_PPS}', timeout=1)
        if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (3, steps, SPEED_PPS):
            raise RuntimeError('MOVE not exactly acknowledged; no retry')
        move = MoveExpectation(object(), quantized_target(origin, steps), time.monotonic(),
            origin_motor_deg=origin, tolerance_motor_deg=TOL[3])
        evidence = SettledArrivalEvidence()
        deadline = move.acknowledged_at + abs(steps) / SPEED_PPS + 5
        progress_at, progress_pos = move.acknowledged_at, origin
        recorder.event('motor', 'bench.powercycle_return_move', driver_id=3, index=index,
            origin_raw_motor_deg=origin, target_raw_motor_deg=move.target_motor_deg,
            target_estimated_mm=target_mm, steps=steps, speed_pps=SPEED_PPS)
        while True:
            samples = read_all(session, recorder, pending=(3,), check_sample=check)
            r, now = samples[3]
            if now > deadline:
                raise RuntimeError('settled endpoint timeout; no retry')
            pos = r.measured_position
            if pos is None:
                evidence.reset()
                if now - move.acknowledged_at > .75:
                    raise RuntimeError('position missing beyond start window')
                continue
            sample_at = now - r.age_ms / 1000
            if sample_at > move.acknowledged_at and progress_pos - pos > .1:
                progress_at, progress_pos = sample_at, pos
            elif now - progress_at > 1.25 and abs(pos - move.target_motor_deg) > TOL[3]:
                raise RuntimeError('no measured progress; NOT a physical limit')
            if evidence.observe(move, r, now):
                if abs(pos - nominal_raw) > TOL[3]:
                    raise RuntimeError('raw waypoint residual too large')
                held, move = pos, None
                recorder.event('motor', 'bench.powercycle_return_arrived', driver_id=3,
                    index=index, measured_raw_motor_deg=pos,
                    inferred_historical_mm=estimated_motor_deg(3, pos) / 360)
                print(f'RETURN {index}/10 ID3 raw_deg={pos:.3f} '
                      f'inferred_mm={estimated_motor_deg(3, pos)/360:.6f}', flush=True)
                break
    samples = read_all(session, recorder, check_sample=check)
    raw = {i: r.measured_position for i, (r, _) in samples.items()}
    return dict(raw_motor_deg=raw,
        inferred_historical_motor_deg={i: estimated_motor_deg(i, p) for i, p in raw.items()},
        absolute_position_independently_verified=False, home_completed=False,
        no_zero_or_config_writes=True, completed_waypoints=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--unchanged-pose-and-supported-return-confirmed', action='store_true')
    args = parser.parse_args()
    validate_evidence()
    print('PLAN: ID3 only, inferred 64.999164 -> 60 mm; ten segments at 0.5 mm/s; NOT homing', flush=True)
    if not args.execute:
        return 0
    if not args.port or not args.unchanged_pose_and_supported_return_confirmed or STOP_FILE.exists():
        parser.error('explicit unchanged-pose/support confirmation and clear stop state required')
    return execute_test(args.port, 3, lambda s, r, d: run_return(s, r, d, confirmed=True),
        purpose='operator-reviewed first return after power cycle; historical position is inferred',
        event_prefix='bench.powercycle_return', test_context=dict(
            before_session=BEFORE_SESSION, before_events_sha256=BEFORE_SHA256,
            after_session=AFTER_SESSION, after_events_sha256=AFTER_SHA256,
            operator_confirmed_no_movement=True, independent_support_confirmed=True,
            absolute_position_independently_verified=False, home_completed=False,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))


if __name__ == '__main__':
    raise SystemExit(main())
