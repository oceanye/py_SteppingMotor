"""Fixed, bounded lift return/group test in the already cleared interior.

ID1 or ID3, or BOTH: start at 5 mm; 5 -> 5.5 -> 5, twice, 0.25 mm/s.
No homing, zero, PID writes, corrective retry or bottom approach. Pair MOVE
commands are sequential on the bus, NOT hardware-synchronized. Group feedback
must stay within 0.2 mm inter-axis spread; host checks cannot replace limits.
"""
import argparse
from dataclasses import asdict
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.x42s_bench_probe import StopProbe
from scripts.x42s_lift_5mm import execute_test, stationary, SPEED_PPS, MAX_SEGMENT_PULSES
from scripts.x42s_micro_jog import quantized_target
from motor_control.coordinated_control import LOGICAL_ROLE_ORDER
from motor_control.protocol import X42sStatusReply, X42sMoveAck
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation, endpoint_tolerance
from motor_control.x42s_safety import confirm_disabled_standstill

TARGETS_MM = (5.5, 5.0, 5.5, 5.0)


def cycle_steps(position, target):
    if not all(math.isfinite(v) for v in (position, target)):
        raise ValueError('nonfinite position/target')
    if not 4.99*360 <= position <= 5.51*360 or target not in (1800., 1980.):
        raise ValueError('outside confirmed 5..5.5 mm cycle')
    delta = target-position
    count = math.floor(abs(delta)*3200/360 + 1e-9)
    if not 0 < count <= MAX_SEGMENT_PULSES:
        raise ValueError('cycle segment exceeds 0.51 mm or is below resolution')
    return count if delta > 0 else -count


def read_all(session, recorder, *, pending=(), check_sample=None):
    samples = {}
    for driver in range(1, 5):
        reply = session.request(f'X42S,S,{driver}', timeout=.6, soft_timeout=True)
        now = time.monotonic()
        if not isinstance(reply, X42sStatusReply) or reply.driver_id != driver:
            raise RuntimeError(f'ID{driver} missing/mismatched reply')
        recorder.event('telemetry', 'x42s.telemetry', role=f'ID{driver}', **asdict(reply))
        if not reply.online or reply.fault or reply.age_ms > 500:
            raise RuntimeError(f'ID{driver} unhealthy/stale: {reply.raw}')
        if ((reply.measured_position is None and driver not in pending)
                or (reply.measured_position is not None and not math.isfinite(reply.measured_position))):
            raise RuntimeError(f'ID{driver} missing/nonfinite position')
        if check_sample is not None:
            check_sample(driver, reply, now)
        samples[driver] = (reply, now)
        time.sleep(.04)
    return samples


def check_passive(samples, origins, tolerances, drivers):
    for i, (r, _) in samples.items():
        if i not in drivers and (r.enabled or not stationary(r)
                or abs(r.measured_position-origins[i]) > tolerances[i]):
            raise RuntimeError(f'passive ID{i} enabled/moving/drifted')


def prepare(session, recorder, drivers, *, confirmed=False, start_mm=5.0, expected_positions=None):
    if not confirmed or tuple(drivers) not in ((1,), (3,), (1, 3)):
        raise ValueError('confirmed supported lift test required')
    if not math.isfinite(start_mm) or not .1 <= start_mm <= 70:
        raise ValueError('bench start must be finite and inside 0.1..70 mm')
    drivers = tuple(drivers)
    expected_positions = ({i: start_mm*360 for i in drivers} if expected_positions is None
                          else dict(expected_positions))
    if set(expected_positions) != set(drivers) or any(
            not math.isfinite(v) or abs(v-start_mm*360) > .02*360 for v in expected_positions.values()):
        raise ValueError('explicit reviewed start positions must match drivers and nominal start within 0.02 mm')
    samples = read_all(session, recorder)
    if any(r.enabled or not stationary(r) for r, _ in samples.values()):
        raise RuntimeError('initial disabled stationary state required')
    origins = {i: r.measured_position for i, (r, _) in samples.items()}
    bindings = StopProbe(session).control_bindings
    tolerances = {b.driver_id: endpoint_tolerance(role, b)
                  for role in LOGICAL_ROLE_ORDER for b in [bindings.for_role(role)]}
    if any(abs(origins[i]-expected_positions[i]) > tolerances[i] for i in drivers):
        raise RuntimeError(f'not at verified {start_mm:g} mm start; no enable/MOVE')
    enabled = set()
    for driver in drivers:
        if session.request_line(f'X42S,{driver},EN,1', timeout=1) != f'OK,X42S,{driver},EN,1':
            raise RuntimeError('enable not precisely acknowledged')
        sent = time.monotonic()
        enabled.add(driver)
        while True:
            samples = read_all(session, recorder, pending=(driver,))
            check_passive(samples, origins, tolerances, enabled)
            for i in enabled:
                r, now = samples[i]
                if r.measured_position is not None and abs(r.measured_position-origins[i]) > tolerances[i]:
                    raise RuntimeError('position drift during enable')
            r, now = samples[driver]
            if r.enabled and stationary(r) and now-r.age_ms/1000 > sent:
                break
            if time.monotonic()-sent > 3:
                raise RuntimeError('fresh stationary enable unavailable')
    if any(not samples[i][0].enabled or not stationary(samples[i][0]) for i in drivers):
        raise RuntimeError('all active drivers must be enabled and stationary')
    return samples, origins, tolerances


def run_cycle(session, recorder, drivers, *, confirmed=False):
    samples, origins, tolerances = prepare(session, recorder, drivers, confirmed=confirmed)

    max_spread_mm = 0.
    for stage, target_mm in enumerate(TARGETS_MM, 1):
        target = target_mm*360
        if any(not samples[i][0].enabled or not stationary(samples[i][0]) for i in drivers):
            raise RuntimeError('all commanded axes must be enabled and stationary before group MOVE')
        plans = {i: cycle_steps(samples[i][0].measured_position, target) for i in drivers}
        expected, evidence, progress = {}, {}, {}
        for i in drivers:
            origin = samples[i][0].measured_position
            steps = plans[i]
            ack = session.request(f'X42S,{i},MOVE,{steps},{SPEED_PPS}', timeout=1)
            if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (i, steps, SPEED_PPS):
                raise RuntimeError('MOVE not precisely acknowledged; no retry/return')
            expected[i] = MoveExpectation(object(), quantized_target(origin, steps), time.monotonic(),
                origin_motor_deg=origin, tolerance_motor_deg=tolerances[i])
            evidence[i] = ArrivalEvidence()
            progress[i] = (expected[i].acknowledged_at, origin)
        deadline = time.monotonic()+6.1
        while True:
            samples = read_all(session, recorder, pending=drivers)
            check_passive(samples, origins, tolerances, drivers)
            if time.monotonic() > deadline:
                raise RuntimeError('group arrival timeout; no retry/return')
            complete = True
            for i in drivers:
                r, now = samples[i]
                e = expected[i]
                if r.measured_position is None:
                    if now-e.acknowledged_at > .75:
                        raise RuntimeError('position missing beyond start window')
                    complete = False
                    continue
                pos = r.measured_position
                if not r.enabled or e.outside_travel(pos) or not 4.99*360 <= pos <= 5.51*360:
                    raise RuntimeError(f'ID{i} disabled/overshot interior envelope: {r.raw}')
                sample_at = now-r.age_ms/1000
                at, old = progress[i]
                sign = 1 if plans[i] > 0 else -1
                if sample_at > e.acknowledged_at and sign*(pos-old) > .1:
                    progress[i] = (sample_at, pos)
                if abs(pos-e.target_motor_deg) > tolerances[i] and now-progress[i][0] > 1.25:
                    raise RuntimeError('no measured progress; not an endstop')
                if not evidence[i].observe(e, r, now):
                    complete = False
                elif abs(pos-target) > tolerances[i]:
                    raise RuntimeError('absolute endpoint mismatch; no retry')
            positions = [samples[i][0].measured_position for i in drivers]
            if all(v is not None for v in positions):
                spread = (max(positions)-min(positions))/360
                max_spread_mm = max(max_spread_mm, spread)
                if spread > .2:
                    raise RuntimeError('pair position spread exceeds 0.2 mm')
                print(f'FEEDBACK stage={stage} motor_deg={positions} spread_mm={spread:.5f}', flush=True)
            if complete:
                recorder.event('motor', 'bench.lift_cycle_waypoint', stage=stage, drivers=drivers,
                               target_mm=target_mm, positions_motor_deg=positions,
                               max_observed_spread_mm=max_spread_mm)
                print(f'WAYPOINT {stage}/4 target_mm={target_mm} motor_deg={positions}', flush=True)
                break
    return {'drivers': drivers, 'positions_motor_deg': {i: samples[i][0].measured_position for i in drivers},
            'max_observed_spread_mm': max_spread_mm}


def run_stop_check(session, recorder, drivers, *, confirmed=False):
    """One bounded extension interrupted deliberately; never automatic return."""
    samples, origins, tolerances = prepare(session, recorder, drivers, confirmed=confirmed)
    expected = {}
    for i in drivers:
        origin = samples[i][0].measured_position
        steps = cycle_steps(origin, 1980.)
        ack = session.request(f'X42S,{i},MOVE,{steps},{SPEED_PPS}', timeout=1)
        if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (i, steps, SPEED_PPS):
            raise RuntimeError('stop-test MOVE not precisely acknowledged')
        expected[i] = MoveExpectation(object(), quantized_target(origin, steps), time.monotonic(),
                                     origin_motor_deg=origin, tolerance_motor_deg=tolerances[i])
    deadline = time.monotonic()+1.5
    while True:
        samples = read_all(session, recorder, pending=drivers)
        check_passive(samples, origins, tolerances, drivers)
        if time.monotonic() > deadline:
            raise RuntimeError('no timely moving evidence; cannot certify dynamic stop')
        moving = True
        for i in drivers:
            r, now = samples[i]
            pos = r.measured_position
            if pos is None:
                if now-expected[i].acknowledged_at > .75:
                    raise RuntimeError('missing stop-test feedback')
                moving = False
                continue
            if not r.enabled or expected[i].outside_travel(pos) or not 4.99*360 <= pos < 5.45*360:
                raise RuntimeError('stop-test travel envelope exceeded')
            if not (pos-origins[i] >= .05*360 and not r.reached
                    and now-r.age_ms/1000 > expected[i].acknowledged_at):
                moving = False
        if moving:
            break
    observed = {i: samples[i][0].measured_position for i in drivers}
    sent = time.monotonic()
    ack = session.request_line('ESTOP', timeout=1, soft_timeout=True)
    if ack != 'OK,ESTOP' or not confirm_disabled_standstill(StopProbe(session), sent, disable_enabled=True):
        raise RuntimeError('dynamic stop not confirmed; no return')
    confirmed_after_s = time.monotonic()-sent
    final = read_all(session, recorder)
    if any(r.enabled or not stationary(r) for r, _ in final.values()):
        raise RuntimeError('post-stop four-driver standstill missing')
    check_passive(final, origins, tolerances, drivers)
    positions = {i: final[i][0].measured_position for i in drivers}
    if any(not origins[i]-tolerances[i] <= positions[i] < 5.45*360 for i in drivers):
        raise RuntimeError('did not park before the commanded endpoint')
    result = {'drivers': drivers, 'interrupted': True, 'pre_stop_motor_deg': observed,
              'positions_motor_deg': positions, 'stop_verification_elapsed_s': confirmed_after_s}
    recorder.event('motor', 'bench.dynamic_stop_verified', **result)
    print(f'DYNAMIC_STOP_VERIFIED {result}', flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--drivers', choices=('1', '3', 'both'), required=True)
    parser.add_argument('--supported-10mm-clearance', action='store_true', required=True)
    parser.add_argument('--stop-during-motion', action='store_true',
                        help='Interrupt one +0.5 mm command, verify standstill, do not return')
    args = parser.parse_args()
    drivers = (1, 3) if args.drivers == 'both' else (int(args.drivers),)
    runner = run_stop_check if args.stop_during_motion else run_cycle
    return execute_test(args.port, drivers, lambda s, r, d: runner(s, r, d, confirmed=True),
                        purpose=('bounded lift dynamic ESTOP verification' if args.stop_during_motion
                                 else 'bounded interior lift forward/return repetition; not full gait'))


if __name__ == '__main__':
    raise SystemExit(main())
