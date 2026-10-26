"""Operator-reviewed inward leg in the SAME 2026-10-26 recovery epoch.

One active axis or same-type pair, others disabled and monitored. Never substitutes inferred
historical coordinates for raw feedback. Requires the reviewed FOUR raw start
coordinates, unchanged-pose confirmation and independent support. No outward
motion, homing/ZERO/configuration writes, retries, or automatic next-axis move.
Lifts stay >=0.05 mm above the inferred old bottom; physical zero is NOT proven.
Another power cycle or manual movement requires a NEW reference review.
"""
import argparse
from dataclasses import dataclass
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
from scripts.x42s_micro_jog import quantized_target
from scripts.x42s_powercycle_return import HISTORICAL, estimated_motor_deg, validate_evidence
from scripts.x42s_suspended_gait import SCALES, STOP_FILE, TOL, SettledArrivalEvidence


@dataclass(frozen=True)
class RecoveryLeg:
    driver: int
    reviewed: dict
    target: float  # estimated historical mm or output degrees

    def validate(self):
        if (self.driver not in SCALES or set(self.reviewed) != set(SCALES)
                or not all(math.isfinite(v) for v in (*self.reviewed.values(), self.target))):
            raise ValueError('one known driver, four finite reviewed raw positions and target required')
        values = {i: estimated_motor_deg(i, v) / SCALES[i] for i, v in self.reviewed.items()}
        if any(not .045 <= values[i] <= 70.002 for i in (1, 3)):
            raise ValueError('lift reference outside reviewed recovery interval')
        if any(abs(values[i]) > 30 for i in (2, 4)):
            raise ValueError('rotary reference outside reviewed +/-30 output degrees')
        start = values[self.driver]
        if self.driver in (1, 3):
            if not .05 <= self.target < start or start - self.target > 60.002:
                raise ValueError('only inward lift travel to >=0.05 mm; no bottom homing')
        elif (abs(start) < .01 or self.target * start < 0
              or abs(self.target) >= abs(start) or abs(start - self.target) > 30):
            raise ValueError('only rotary return toward original zero, without crossing it')
        return start

    def waypoints(self):
        current = self.validate()
        points = []
        while abs(current - self.target) > 1e-8:
            if self.driver in (1, 3):
                if current > 2 + 1e-8:
                    step, speed, boundary = .5, 1.25, max(2., self.target)
                elif current > .5 + 1e-8:
                    step, speed, boundary = .25, .25, max(.5, self.target)
                else:
                    step, speed, boundary = .1, .05, self.target
                current = max(boundary, round(current - step, 9))
            else:
                speed = .5
                direction = 1 if self.target > current else -1
                current += direction * min(.5, abs(self.target - current))
            pps = max(1, round(speed * SCALES[self.driver] * 3200 / 360))
            points.append((current, pps))
        return tuple(points)


def steps_for(leg, raw, target):
    nominal = target * SCALES[leg.driver] - HISTORICAL[leg.driver]
    delta = nominal - raw
    direction = 1 if leg.target > leg.validate() else -1
    # Invert the FIRMWARE encoding floor(abs(pulses)*3600/3200)/10.
    # Merely flooring delta*3200/360 throws away an extra 0.1 degree in some
    # cases: 10.6589 degrees -> 94 pulses -> 10.5, while 95 -> 10.6 is still
    # inward. Pick the largest pulse count that does not cross the waypoint.
    # This fixes quantization, NOT endpoint tolerance or servo error handling.
    units = math.floor(abs(delta) * 10 + 1e-8)
    steps = direction * (((units + 1) * 3200 - 1) // 3600)
    maximum = .51 * SCALES[leg.driver] * 3200 / 360
    if direction * delta <= 0 or not 0 < abs(steps) <= maximum:
        raise RuntimeError('no reverse correction/retry; segment outside reviewed bound')
    return steps, nominal


def group_waypoints(legs):
    """Paired progress, not shared-clock/hardware synchronization.

    Keep every joint within its own <=0.5-unit segment and speed cap; choose
    proportional command speeds. Never assume equal physical progress between
    feedback samples. This is only for the independently supported clear fixture.
    """
    if tuple(sorted(l.driver for l in legs)) not in ((1,), (2,), (3,), (4,), (1, 3), (2, 4)):
        raise ValueError('only one axis or same-type pair may return')
    if any(l.reviewed != legs[0].reviewed for l in legs):
        raise ValueError('all legs must share the same four-axis reviewed snapshot')
    if len(legs) == 1:
        return tuple({legs[0].driver: point} for point in legs[0].waypoints())
    starts = {l.driver: l.validate() for l in legs}
    previous, result = dict(starts), []
    while any(abs(previous[l.driver] - l.target) > 1e-8 for l in legs):
        # Repartition the remaining fraction, avoiding a sub-pulse final tail
        # on the shorter-travel axis. Both lifts keep near-bottom SMALL segments,
        # not merely slow speeds on an otherwise 0.5 mm final command.
        counts = []
        for leg in legs:
            i = leg.driver
            step = .5
            if i in (1, 3):
                trial = max(leg.target, previous[i] - .5)
                step = .1 if trial <= .5 else .25 if trial <= 2 else .5
            counts.append(max(1, math.ceil(abs(leg.target - previous[i]) / step - 1e-9)))
        remaining_groups = max(counts)
        targets, caps = {}, {}
        for leg in legs:
            i = leg.driver
            targets[i] = previous[i] + (leg.target - previous[i]) / remaining_groups
            caps[i] = (.05 if min(previous[i], targets[i]) <= .5 else
                       .25 if min(previous[i], targets[i]) <= 2 else 1.25) if i in (1, 3) else .5
        duration = max(abs(targets[i] - previous[i]) / caps[i] for i in targets)
        speeds = {}
        for i, target in targets.items():
            proportional = math.floor(abs(target - previous[i]) * SCALES[i] * 3200 / 360 / duration + 1e-8)
            # Do not force equal arrival by slowing the short lift below the
            # previously exercised 0.05 mm/s. The real 2026-10-26 paired return
            # stalled its progress guard at 13 pps (~0.2 motor RPM); that is NOT
            # proof of a mechanical end stop. Let the short lift arrive earlier,
            # retaining its original position tube and settled evidence while
            # the other finishes. Never raise a leg's near-bottom speed cap.
            minimum = 160 if i in (1, 3) else 1
            speeds[i] = max(minimum, proportional)
            if speeds[i] > caps[i] * SCALES[i] * 3200 / 360 + 1e-7:
                raise ValueError('minimum speed incompatible with near-bottom cap')
        result.append({i: (target, speeds[i]) for i, target in targets.items()})
        previous = targets
        if len(result) > 4096:
            raise ValueError('excessively fragmented return; re-plan required')
    return tuple(result)


def run_leg(session, recorder, _, leg, *, confirmed=False, stop_file=STOP_FILE):
    return run_legs(session, recorder, (leg,), confirmed=confirmed, stop_file=stop_file)


def run_legs(session, recorder, legs, *, confirmed=False, stop_file=STOP_FILE):
    points = group_waypoints(legs)
    by_id = {l.driver: l for l in legs}
    reviewed = legs[0].reviewed
    if not confirmed or stop_file.exists():
        raise ValueError('fresh supported return confirmation required; stop must be clear')
    enabled, enabling, moves = set(), None, {}
    held = dict(reviewed)
    bounds = {i: sorted((l.validate() * SCALES[i], l.target * SCALES[i])) for i, l in by_id.items()}

    def check(i, r, now):
        if stop_file.exists():
            raise RuntimeError('operator stop')
        if i not in by_id:
            if r.enabled or not stationary(r) or abs(r.measured_position - reviewed[i]) > TOL[i]:
                raise RuntimeError(f'passive ID{i} enabled/moving/drifted')
        elif i not in moves:
            if r.measured_position is not None and abs(r.measured_position - held[i]) > TOL[i]:
                raise RuntimeError('reviewed raw position changed; no automatic re-anchor')
            if enabling != i and (r.enabled != (i in enabled) or not r.reached or not stationary(r)):
                raise RuntimeError('stationary held start required')
        elif (not r.enabled or moves[i].outside_travel(r.measured_position)
              or (r.measured_position is not None and not
                  bounds[i][0] - TOL[i] <= estimated_motor_deg(i, r.measured_position) <= bounds[i][1] + TOL[i])):
            raise RuntimeError(f'ID{i} outside reviewed recovery leg: {r.raw}')

    read_all(session, recorder, check_sample=check)
    recorder.event('motor', 'bench.recovery_leg_plan', driver_ids=list(by_id),
        reviewed_raw_motor_deg=reviewed, historical_offset_motor_deg=HISTORICAL,
        estimated_targets={i: l.target for i, l in by_id.items()}, waypoints=points,
        hardware_synchronized=False, independently_supported_clear_fixture=True,
        absolute_position_independently_verified=False, home_completed=False)
    for driver in by_id:
        if session.request_line(f'X42S,{driver},EN,1', timeout=1) != f'OK,X42S,{driver},EN,1':
            raise RuntimeError('enable not exactly acknowledged')
        enabled.add(driver)
        enabling, sent = driver, time.monotonic()
        while True:
            samples = read_all(session, recorder, pending=(driver,), check_sample=check)
            r, now = samples[driver]
            if r.enabled and r.reached and stationary(r) and now - r.age_ms / 1000 > sent:
                break
            if now - sent > 3:
                raise RuntimeError('fresh stationary enabled feedback unavailable')
        enabling = None
    for index, group in enumerate(points, 1):
        samples = read_all(session, recorder, check_sample=check)
        prepared = []
        for driver, (target, pps) in group.items():
            origin = samples[driver][0].measured_position
            steps, nominal = steps_for(by_id[driver], origin, target)
            prepared.append((driver, target, pps, origin, steps, nominal))
        evidence, progress, deadlines, nominals = {}, {}, {}, {}
        # Validate the WHOLE group before dispatch; then back-to-back requests.
        for driver, target, pps, origin, steps, nominal in prepared:
            if stop_file.exists():
                raise RuntimeError('operator stop before MOVE')
            ack = session.request(f'X42S,{driver},MOVE,{steps},{pps}', timeout=1)
            if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (driver, steps, pps):
                raise RuntimeError('MOVE not precisely acknowledged; stop group, no retry')
            move = MoveExpectation(object(), quantized_target(origin, steps), time.monotonic(),
                origin_motor_deg=origin, tolerance_motor_deg=TOL[driver])
            moves[driver] = move
            evidence[driver] = SettledArrivalEvidence()
            deadlines[driver] = move.acknowledged_at + abs(steps) / pps + 5
            progress[driver] = (move.acknowledged_at, origin, 1 if steps > 0 else -1)
            nominals[driver] = nominal
            recorder.event('motor', 'bench.recovery_leg_move', driver_id=driver, index=index,
                origin_raw_motor_deg=origin, target_raw_motor_deg=move.target_motor_deg,
                estimated_target=target, steps=steps, speed_pps=pps)
        while True:
            samples = read_all(session, recorder, pending=tuple(by_id), check_sample=check)
            done = set()
            for driver, move in moves.items():
                r, now = samples[driver]
                if now > deadlines[driver]:
                    raise RuntimeError('settled endpoint timeout; no retry')
                pos = r.measured_position
                if pos is None:
                    evidence[driver].reset()
                    if now - move.acknowledged_at > .75:
                        raise RuntimeError('position missing after start window')
                    continue
                sample_at = now - r.age_ms / 1000
                progress_at, progress_pos, direction = progress[driver]
                if sample_at > move.acknowledged_at and direction * (pos - progress_pos) > .1:
                    progress[driver] = (sample_at, pos, direction)
                elif now - progress_at > 1.25 and abs(pos - move.target_motor_deg) > TOL[driver]:
                    raise RuntimeError('no measured progress; NOT a physical limit')
                if evidence[driver].observe(move, r, now):
                    if abs(pos - nominals[driver]) > TOL[driver]:
                        raise RuntimeError('nominal endpoint mismatch')
                    done.add(driver)
            if done == set(by_id):
                inferred = {}
                for driver in by_id:
                    pos = samples[driver][0].measured_position
                    held[driver] = pos
                    inferred[driver] = estimated_motor_deg(driver, pos) / SCALES[driver]
                    recorder.event('motor', 'bench.recovery_leg_arrived', driver_id=driver,
                        index=index, measured_raw_motor_deg=pos, inferred_historical_position=inferred[driver])
                moves.clear()
                if index % 5 == 0 or index == len(points):
                    print(f'RECOVERY {list(by_id)} {index}/{len(points)} inferred={inferred}', flush=True)
                break
    samples = read_all(session, recorder, check_sample=check)
    raw = {i: r.measured_position for i, (r, _) in samples.items()}
    return dict(driver_ids=list(by_id), raw_motor_deg=raw,
        inferred_historical_position={i: estimated_motor_deg(i, v) / SCALES[i] for i, v in raw.items()},
        home_completed=False, absolute_position_independently_verified=False,
        no_zero_or_config_writes=True, completed_waypoints=len(points))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', type=int, choices=range(1, 5), required=True)
    parser.add_argument('--with-paired-axis', action='store_true',
                        help='Also return the other same-type axis; sequential bus dispatch, NOT hardware sync')
    parser.add_argument('--reviewed-raw-motor-deg', type=float, nargs=4, required=True)
    parser.add_argument('--target', type=float, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--same-epoch-supported-return-confirmed', action='store_true')
    args = parser.parse_args()
    leg = RecoveryLeg(args.driver, dict(zip(range(1, 5), args.reviewed_raw_motor_deg)), args.target)
    legs = (leg,)
    if args.with_paired_axis:
        other = {1: 3, 3: 1, 2: 4, 4: 2}[args.driver]
        legs += (RecoveryLeg(other, dict(leg.reviewed), args.target),)
    points = group_waypoints(legs)
    validate_evidence()
    print(f'PLAN IDs {[l.driver for l in legs]}: target {leg.target}; {len(points)} groups; NOT homing/hardware sync', flush=True)
    if not args.execute:
        return 0
    if not args.port or not args.same_epoch_supported_return_confirmed or STOP_FILE.exists():
        parser.error('port, same-epoch independent support confirmation and clear stop required')
    return execute_test(args.port, tuple(l.driver for l in legs), lambda s, r, d: run_legs(s, r, legs, confirmed=True),
        purpose='operator-reviewed inward recovery leg; historical position is inferred',
        event_prefix='bench.recovery_leg', diagnostic_max_bytes=32_000_000,
        test_context=dict(reviewed_raw_motor_deg=leg.reviewed, estimated_target=leg.target,
            historical_offset_motor_deg=HISTORICAL, operator_confirmed_no_movement=True,
            independent_support_confirmed=True, absolute_position_independently_verified=False,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))


if __name__ == '__main__':
    raise SystemExit(main())
