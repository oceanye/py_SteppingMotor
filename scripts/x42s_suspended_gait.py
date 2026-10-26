"""One explicitly authorized SUSPENDED full gait, not a field walking test.

Uses production planner, X42S rectangle certification and GaitExecutor. Slows
and subdivides the same path. No calibration/zero/PID writes, GUI config changes,
contact claims, retries or automatic return after failure. Fixed tested ID map,
1 mm lead, rotary ratio 21.5. Four fresh endpoint/hold checks at every segment.
Default is offline plan only; --execute needs explicit reviewed coordinates and
clearance/support confirmations. Creating logs/x42s-gait-stop.request cancels.
Host bounds are NOT hardware limits or a firmware communication watchdog.
The explicitly selected interior profile uses absolute lift coordinates 5..65
mm (60 mm travel), never a new zero; it is NOT standard 0..70 mm acceptance.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motor_control.gait_coordinates import action_values
from motor_control.gait_executor import GaitExecutor
from motor_control.gait_planner import parse_gait_params, plan_gait_stages, plan_swing_trajectory
from motor_control.gait_simulation import GaitSimulation
from motor_control.gait_x42s_plan import certify_x42s_stages
from motor_control.protocol import X42sMoveAck
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation, arrived
from scripts.x42s_lift_5mm import execute_test, stationary
from scripts.x42s_lift_cycle import read_all
from scripts.x42s_micro_jog import quantized_target

ROLES = {'Mup1': 1, 'Mr1': 2, 'Mup2': 3, 'Mr2': 4}
SCALES = {1: 360., 2: 21.5, 3: 360., 4: 21.5}
TOL = {1: .72, 2: .225, 3: .72, 4: .225}
STOP_FILE = ROOT / 'logs' / 'x42s-gait-stop.request'


def lift_bounds(profile):
    if profile not in ('standard', 'interior'):
        raise ValueError('unknown bench lift profile')
    return (0., 70.) if profile == 'standard' else (5., 65.)


class SettledArrivalEvidence:
    """Interior/2x bench: strengthen arrival, never waive a nonzero speed.

Retain fresh post-ACK actual endpoint/zero-speed requirements, plus >=1 second
of consecutively valid samples spanning <=the ORIGINAL position tolerance.
An invalid/nonzero-speed sample resets the window; no next MOVE is released.
Travel/missing-feedback/deadline and post-arrival hold guards remain separate.
"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.first = self.last = self.low = self.high = None
        self.count = 0

    def observe(self, move, reply, now, blind_s=.45):
        if not arrived(move, reply, now, blind_s):
            self.reset()
            return False
        at, pos = now-reply.age_ms/1000, reply.measured_position
        if self.last is not None and (at < self.last or at-self.last > .5):
            self.reset()
        if self.first is None:
            self.first = self.last = at
            self.low = self.high = pos
            self.count = 1
            return False
        self.low, self.high = min(self.low, pos), max(self.high, pos)
        if self.high-self.low > move.tolerance_motor_deg:
            self.reset()
            return False
        if at-self.last < .1:
            return False
        self.last = at
        self.count += 1
        return self.count >= 3 and at-self.first >= 1.


def validate_speed_scale(speed_scale):
    if isinstance(speed_scale, bool) or speed_scale not in (1., 2.):
        raise ValueError('bench speed scale must be explicitly 1 or 2')
    return float(speed_scale)


def build_plan(params, action, *, profile='standard', speed_scale=1., route=None,
               rotary_segment_deg=1., paired_lift_step_mm=2.):
    """Preserve stage semantics/endpoints; only reduce speed/subdivide groups."""
    if (params.trajectory_mode != 'two_mode_v1' or params.lift_mm != 70
            or params.mup1_lift_sign != 1 or params.mup2_lift_sign != 1
            or params.geometry.arm_length_mm != 190):
        raise ValueError('bench requires verified 190 mm/two-mode/70 mm/positive-extension profile')
    speed_scale = validate_speed_scale(speed_scale)
    if rotary_segment_deg not in (1., 4.) or paired_lift_step_mm not in (.4, 2.):
        raise ValueError('unreviewed bench subdivision')
    floor, ceiling = lift_bounds(profile)
    p = replace(params, lift_mm=ceiling-floor, swing_speed_deg_s=speed_scale/3,
                lift_speed_mm_s=1.25*speed_scale, settle_speed_mm_s=speed_scale)
    side, arc = action_values(action)
    report = (GaitSimulation(p).preview(side, arc) if route is None else
              plan_swing_trajectory(p, side=side, arc_deg=arc, route=route))
    if not report.feasible:
        raise ValueError('model rejected the path; no motion')
    stages = certify_x42s_stages(plan_gait_stages(p, side=side, arc_deg=arc,
                                route=report.route), p, report, .5)
    result, heights = [], {'Mup1': 0., 'Mup2': 0.}
    for stage in stages:
        if stage.stage_id == 'S4':
            groups, endpoints, durations, previous = [], [], [], (0., 0.)
            for group, end, duration in zip(stage.move_groups, stage.sync_endpoints, stage.group_durations):
                count = max(1, math.ceil(max(abs(b-a) for a, b in zip(previous, end))/rotary_segment_deg))
                for k in range(1, count+1):
                    endpoint = tuple(a+(b-a)*k/count for a, b in zip(previous, end))
                    groups.append(tuple(replace(m, delta=m.delta/count) for m in group))
                    endpoints.append(endpoint)
                    durations.append(duration/count)
                previous = end
            result.append(replace(stage, move_groups=tuple(groups), sync_endpoints=tuple(endpoints),
                                  group_durations=tuple(durations)))
        elif stage.is_motion_stage:
            if any(m.role not in heights for g in stage.move_groups for m in g):
                raise ValueError('unreviewed extra rotary phase stage')
            groups = []
            for group in stage.move_groups:
                if len({round(m.delta, 8) for m in group}) != 1:
                    raise ValueError('unequal linear travel')
                remaining = abs(group[0].delta)
                direction = 1 if group[0].delta > 0 else -1
                while remaining > 1e-8:
                    h = heights[group[0].role]
                    step, speed = 2., min(m.speed for m in group)
                    if len(group) == 2:
                        step = min(step, paired_lift_step_mm)
                    if direction < 0:
                        if h > 2+1e-8:
                            step = min(step, h-2)
                        elif h > .5+1e-8:
                            step, speed = min(.25, h-.5), .25
                        else:
                            step, speed = .1, .05
                    delta = direction * min(remaining, step)
                    groups.append(tuple(replace(m, delta=delta, speed=speed) for m in group))
                    for m in group:
                        heights[m.role] = round(heights[m.role]+delta, 9)
                        if not 0 <= heights[m.role] <= p.lift_mm:
                            raise ValueError('linear target outside selected lift profile')
                    remaining -= abs(delta)
            # One preflight for the whole linear pair, then back-to-back serial
            # requests. This is NOT atomic/shared-clock driver synchronization.
            durations = tuple(max(abs(m.delta)/m.speed for m in g) for g in groups)
            result.append(replace(stage, move_groups=tuple(groups), synchronized=True,
                                  group_durations=durations, duration_s=sum(durations),
                                  execution_profile='bench_sequential_linear'))
        else:
            result.append(stage)
    return p, report, tuple(result)


def checked_steps(driver, origin, target, rotary_origin, *, rotary_segment_deg=1.):
    if rotary_segment_deg not in (1., 4.):
        raise ValueError('unreviewed rotary subdivision')
    if driver not in SCALES or not all(math.isfinite(v) for v in (origin, target, rotary_origin)):
        raise ValueError('invalid axis or nonfinite target')
    scale = SCALES[driver]
    if driver in (1, 3):
        if not 0 <= target <= 70*scale or abs(target-origin) > 2.01*scale:
            raise ValueError('lift target/segment outside confirmed bounds')
    elif abs(target-rotary_origin) > 180*scale+1e-7 or abs(target-origin) > (rotary_segment_deg+.02)*scale:
        raise ValueError('rotary target/segment outside confirmed bounds')
    # Round inward: never command through the requested endpoint/bottom.
    steps = math.floor(abs(target-origin)*3200/360+1e-9)
    return steps if target >= origin else -steps


def lift_pair_observation(samples, zero_reference):
    """Describe asynchronous measurements, NOT a simultaneous physical spread.

    The timestamps are estimates from the bridge's age field, not a shared
    encoder capture clock. Do not extrapolate or relax the 0.5 mm stop guard.
    A stopped, equal-height endpoint cannot prove equality during motion.
    """
    heights = {i: (samples[i][0].measured_position-zero_reference[i])/360 for i in (1, 3)}
    measured_at = {i: samples[i][1]-samples[i][0].age_ms/1000 for i in (1, 3)}
    return dict(heights_mm=heights, sampled_spread_mm=abs(heights[1]-heights[3]),
                estimated_sample_skew_ms=abs(measured_at[1]-measured_at[3])*1000,
                ages_ms={i: samples[i][0].age_ms for i in (1, 3)},
                speeds_motor_rpm={i: samples[i][0].speed for i in (1, 3)},
                simultaneous_measurement=False, limit_mm=.5)


class SuspendedHost:
    def __init__(self, session, recorder, reviewed, *, stop_file=STOP_FILE, profile='standard', zero_reference=None, speed_scale=1., rotary_segment_deg=1., paired_progress_guard=False):
        self.lift_floor, self.lift_ceiling = lift_bounds(profile)
        self.profile = profile
        self.speed_scale = validate_speed_scale(speed_scale)
        if rotary_segment_deg not in (1., 4.):
            raise ValueError('unreviewed rotary subdivision')
        self.rotary_segment_deg = rotary_segment_deg
        self.paired_progress_guard = paired_progress_guard
        if set(reviewed) != {1, 2, 3, 4} or any(not math.isfinite(v) for v in reviewed.values()):
            raise ValueError('four finite explicitly reviewed motor coordinates required')
        # An explicitly operator-confirmed mechanical zero may have NONZERO raw
        # driver coordinates. Translate command/bounds only; never rewrite raw
        # telemetry, driver zero, or an old power-cycle/history offset.
        reference = {i: 0. for i in range(1, 5)} if zero_reference is None else dict(zero_reference)
        if set(reference) != {1, 2, 3, 4} or any(not math.isfinite(v) for v in reference.values()):
            raise ValueError('four finite manually confirmed zero references required')
        self.zero_reference = reference
        if any(abs(reviewed[i]-reference[i]-self.lift_floor*360) > 1.8 for i in (1, 3)) or any(abs((reviewed[i]-reference[i])/21.5) > .5 for i in (2, 4)):
            raise ValueError('only reviewed bottom/near-rotary-zero start; no automatic recovery')
        self.session, self.recorder, self.reviewed = session, recorder, dict(reviewed)
        self.stop_file = stop_file
        self.origins = dict(reviewed)
        self.held = dict(reviewed)
        self.heights = {1: self.lift_floor, 3: self.lift_floor}
        self.pending, self.evidence, self.progress = {}, {}, {}
        self.expected_enabled = set()
        self.stage, self.groups_completed, self.deadline = 'preflight', 0, 0.
        self.max_error = {i: 0. for i in reviewed}

    def cancelled(self):
        return self.stop_file.exists()

    def log(self, message):
        print(message, flush=True)
        self.recorder.event('gui', 'gui.message', message=message)

    def role_axis(self, role):
        return ROLES[role]

    def stop_axes(self, axes):
        # execute_test finally performs the full feedback-confirmed shutdown.
        self.session.request_line('ESTOP', timeout=1, soft_timeout=True)

    def snapshot(self):
        if self.cancelled():
            raise RuntimeError('operator stop file present')
        def check(i, r, now):
            enabling = i in self.pending and self.pending[i] is None
            if bool(r.enabled) != (i in self.expected_enabled) and not enabling:
                raise RuntimeError(f'ID{i} unexpected enable state')
            if i not in self.pending and (not r.reached or not stationary(r) or abs(r.measured_position-self.held[i]) > TOL[i]):
                raise RuntimeError(f'held ID{i} moved/lost stationarity: {r.raw}')
            move = self.pending.get(i)
            if move is not None and move.outside_travel(r.measured_position):
                raise RuntimeError(f'ID{i} outside commanded travel: {r.raw}')
        # Fail on the first bad sample, not after querying the remaining axes.
        return read_all(self.session, self.recorder, pending=tuple(self.pending), check_sample=check)

    def prepare(self):
        self.snapshot()
        for i in range(1, 5):
            if self.cancelled():
                raise RuntimeError('operator stop before enable')
            if self.session.request_line(f'X42S,{i},EN,1', timeout=1) != f'OK,X42S,{i},EN,1':
                raise RuntimeError('enable not exactly acknowledged')
            sent = time.monotonic()
            self.expected_enabled.add(i)
            # Only just-enabled axis may temporarily have invalid position.
            self.pending[i] = None
            while True:
                samples = self.snapshot()
                r, now = samples[i]
                if r.measured_position is not None and abs(r.measured_position-self.reviewed[i]) > TOL[i]:
                    raise RuntimeError(f'ID{i} drift during enable; no MOVE')
                if r.enabled and r.reached and stationary(r) and now-r.age_ms/1000 > sent:
                    break
                if now-sent > 3:
                    raise RuntimeError('fresh enabled stationary state unavailable')
            del self.pending[i]
        samples = self.snapshot()
        self.origins = {i: r.measured_position for i, (r, _) in samples.items()}
        self.held = dict(self.origins)
        self.recorder.event('motor', 'bench.gait_origins', positions_motor_deg=self.origins,
                            no_zero_written=True, nominal_lift_start_mm=self.lift_floor,
                            lift_profile=self.profile, lift_range_mm=[self.lift_floor, self.lift_ceiling],
                            mechanical_zero_raw_motor_deg=self.zero_reference,
                            raw_telemetry_preserved=True, speed_scale=self.speed_scale)

    def verify_hold(self, seconds=5.):
        """No movement, all four axes enabled, original strict hold checks."""
        if seconds != 5. or self.pending or self.expected_enabled != {1, 2, 3, 4}:
            raise ValueError('five-second prepared four-axis hold required')
        end, count = time.monotonic()+seconds, 0
        while time.monotonic() < end:
            self.snapshot()
            count += 1
        self.recorder.event('motor', 'bench.gait_hold_verified', duration_s=seconds, rounds=count)

    def dispatch(self, targets, speeds):
        if set(targets) != set(speeds) or set(targets) & set(self.pending):
            raise RuntimeError('invalid/pending dispatch')
        samples = self.snapshot()
        planned = []
        for i, target in targets.items():
            origin = samples[i][0].measured_position
            reference = self.zero_reference[i]
            if i in (1, 3) and not self.lift_floor*360 <= target-reference <= self.lift_ceiling*360:
                raise RuntimeError('absolute lift target outside selected profile')
            steps = checked_steps(i, origin-reference, target-reference, self.origins[i]-reference,
                                  rotary_segment_deg=self.rotary_segment_deg)
            cap = (1.25 if i in (1, 3) else 1.)*self.speed_scale
            # Keep the original slow approach even when bulk motion is 2x.
            if i in (1, 3) and target < origin:
                height = (origin-reference)/360-self.lift_floor
                if height <= .5+TOL[i]/360:
                    cap = .05
                elif height <= 2+TOL[i]/360:
                    cap = .25
            if not math.isfinite(speeds[i]) or not 0 < speeds[i] <= cap+1e-8:
                raise RuntimeError('bench speed exceeds verified conservative cap')
            pps = max(1, round(speeds[i]*SCALES[i]*3200/360))
            planned.append((i, origin, target, steps, pps))
        if self.paired_progress_guard and set(targets) == {1, 3}:
            intervals = {}
            for i, origin, target, _, _ in planned:
                lo, hi = sorted(((origin-self.zero_reference[i])/360,
                                 (target-self.zero_reference[i])/360))
                intervals[i] = (lo-TOL[i]/360, hi+TOL[i]/360)
            spread = max(abs(intervals[1][0]-intervals[3][1]), abs(intervals[1][1]-intervals[3][0]))
            self.recorder.event('motor', 'bench.gait_pair_envelope', stage=self.stage,
                                maximum_independent_spread_mm=spread, limit_mm=.5)
            if spread > .5:
                raise RuntimeError('paired lift progress rectangle exceeds 0.5 mm; no MOVE')
        # Validate the entire pair before the first MOVE; no retry on partial ACK.
        for i, origin, target, steps, pps in planned:
            if self.cancelled():
                raise RuntimeError('operator stop before MOVE')
            if steps:
                ack = self.session.request(f'X42S,{i},MOVE,{steps},{pps}', timeout=1)
                if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (i, steps, pps):
                    raise RuntimeError('MOVE not exactly acknowledged')
            now = time.monotonic()
            self.pending[i] = MoveExpectation(object(), quantized_target(origin, steps), now,
                no_motion=not steps, origin_motor_deg=origin, tolerance_motor_deg=TOL[i])
            self.evidence[i] = (SettledArrivalEvidence() if self.profile == 'interior' or self.speed_scale == 2
                                else ArrivalEvidence())
            self.progress[i] = (now, origin)
            self.deadline = max(self.deadline, now+abs(steps)/pps+5)
            self.recorder.event('motor', 'bench.gait_move', stage=self.stage, driver_id=i,
                origin_motor_deg=origin, nominal_target_motor_deg=target,
                target_motor_deg=self.pending[i].target_motor_deg, steps=steps, speed_pps=pps)

    def send_relative(self, axis, delta, speed):
        raise RuntimeError('bench requires whole-group preflight, not one-axis-at-a-time dispatch')

    def send_synchronized(self, moves, duration):
        axes = [m[0] for m in moves]
        if (not 1 <= len(moves) <= 2 or len(set(axes)) != len(axes)
                or not set(axes) <= {1, 3}):
            raise RuntimeError('unexpected linear group')
        targets = {}
        for i, delta, speed in moves:
            if not math.isfinite(delta):
                raise RuntimeError('nonfinite linear delta')
            self.heights[i] = round(self.heights[i]+delta, 9)
            targets[i] = self.zero_reference[i]+self.heights[i]*360
        self.dispatch(targets, {i: speed for i, _, speed in moves})
        return 'sent'

    def send_synchronized_endpoint(self, moves, duration, endpoints):
        if len(moves) != 2 or {m[0] for m in moves} != {2, 4} or len(endpoints) != 2:
            raise RuntimeError('invalid certified rotary pair')
        self.dispatch({m[0]: self.origins[m[0]]+end*SCALES[m[0]] for m, end in zip(moves, endpoints)},
                      {i: speed for i, _, speed in moves})
        return 'sent'

    def wait_group(self, axes, timeout_s):
        if set(axes) != set(self.pending):
            raise RuntimeError('pending group mismatch')
        while True:
            samples = self.snapshot()
            if set(self.pending) == {1, 3} and all(samples[i][0].measured_position is not None for i in (1, 3)):
                # These are asynchronous samples, not a synchronization claim.
                observation = lift_pair_observation(samples, self.zero_reference)
                if observation['sampled_spread_mm'] > .5:
                    self.recorder.event('motor', 'bench.gait_pair_spread_stop', level='ERROR',
                                        stage=self.stage, completed_groups=self.groups_completed,
                                        **observation)
                    raise RuntimeError('paired lift samples differ by over 0.5 mm; stop for review')
            complete = True
            for i, move in self.pending.items():
                r, now = samples[i]
                if r.measured_position is None:
                    if isinstance(self.evidence[i], SettledArrivalEvidence):
                        self.evidence[i].reset()
                    else:
                        self.evidence[i].previous = None
                    if now-move.acknowledged_at > .75:
                        raise RuntimeError('position missing past start blind window')
                    complete = False
                    continue
                if move.outside_travel(r.measured_position):
                    raise RuntimeError(f'ID{i} outside commanded travel: {r.raw}')
                at, position = self.progress[i]
                if abs(r.measured_position-position) >= .1:
                    self.progress[i] = (now, r.measured_position)
                elif now-at > 3 and abs(r.measured_position-move.target_motor_deg) > TOL[i]:
                    raise RuntimeError(f'ID{i} no measured progress, not a physical-limit measurement')
                complete = self.evidence[i].observe(move, r, now) and complete
            if complete:
                for i, move in self.pending.items():
                    self.held[i] = samples[i][0].measured_position
                    self.max_error[i] = max(self.max_error[i], abs(self.held[i]-move.target_motor_deg))
                self.groups_completed += 1
                self.recorder.event('motor', 'bench.gait_arrived', stage=self.stage,
                    group=self.groups_completed, positions_motor_deg={i: r.measured_position for i, (r, _) in samples.items()},
                    targets_motor_deg={i: m.target_motor_deg for i, m in self.pending.items()})
                self.pending.clear()
                self.deadline = 0.
                return 'DONE'
            if time.monotonic() > self.deadline:
                raise RuntimeError('fresh stable endpoint timeout; no retry')


def run_gait(session, recorder, _, params, report, stages, reviewed, *, profile='standard', zero_reference=None, speed_scale=1., prepare_only=False):
    floor, ceiling = lift_bounds(profile)
    if params.lift_mm != ceiling-floor:
        raise ValueError('plan and execution lift profile mismatch')
    speed_scale = validate_speed_scale(speed_scale)
    if (params.swing_speed_deg_s != speed_scale/3
            or params.lift_speed_mm_s != 1.25*speed_scale
            or params.settle_speed_mm_s != speed_scale):
        raise ValueError('plan and execution speed scale mismatch')
    host = SuspendedHost(session, recorder, reviewed, profile=profile,
                         zero_reference=zero_reference, speed_scale=speed_scale)
    host.prepare()
    if profile == 'interior' or speed_scale == 2 or prepare_only:
        host.verify_hold()
    if prepare_only:
        return dict(prepare_only=True, groups_completed=0, speed_scale=speed_scale,
                    positions_motor_deg=host.held,
                    mechanical_zero_raw_motor_deg=host.zero_reference)
    executor = GaitExecutor(host, params, stages, side=report.side)
    recorder.event('motor', 'bench.gait_plan', modality=report.modality, route=report.route,
                   params=params.as_document(), stages=[asdict(s) for s in stages],
                   suspended_only=True, lift_profile=profile, lift_range_mm=[floor, ceiling],
                   speed_scale=speed_scale, speed_is_command_cap_not_elapsed_time_guarantee=True,
                   contact_and_avoidance_not_physically_verified=True)
    while executor.current_stage() is not None:
        stage = executor.current_stage()
        host.stage = stage.stage_id
        if host.cancelled():
            raise RuntimeError('operator stop before stage')
        if stage.is_motion_stage:
            if not executor.execute_current_stage(progress=lambda sid, n, total:
                    host.log(f'BENCH {sid} {n}/{total}') if n % 5 == 0 or n == total else None):
                raise RuntimeError(executor.last_error)
        else:
            # Explicit suspended fixture only: NEVER claim contact/load confirmation.
            host.log(f'BENCH {stage.stage_id}: 架空流程检查点；不证明踩实/接触/负载')
            if not executor.advance_confirm():
                raise RuntimeError('confirmation stage failed')
    samples = host.snapshot()
    return dict(suspended_only=True, action_side=report.side, modality=report.modality,
                lift_profile=profile, lift_range_mm=[floor, ceiling], speed_scale=speed_scale,
                groups_completed=host.groups_completed,
                positions_motor_deg={i: r.measured_position for i, (r, _) in samples.items()},
                mechanical_zero_raw_motor_deg=host.zero_reference,
                max_endpoint_error_motor_deg=host.max_error,
                contact_and_avoidance_not_physically_verified=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=('L+', 'L-', 'R+', 'R-'), default='L+')
    parser.add_argument('--lift-profile', choices=('standard', 'interior'), default='standard')
    parser.add_argument('--interior-profile-confirmed', action='store_true')
    parser.add_argument('--speed-scale', type=float, choices=(1., 2.), default=1.)
    parser.add_argument('--double-speed-confirmed', action='store_true')
    parser.add_argument('--prepare-only', action='store_true',
                        help='Enable and verify a five-second hold only; no MOVE')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--reviewed-motor-deg', nargs=4, type=float, metavar='DEG')
    parser.add_argument('--manual-zero-motor-deg', nargs=4, type=float, metavar='DEG',
                        help='Raw readings at a NEW operator-confirmed four-axis mechanical zero; no driver ZERO')
    parser.add_argument('--manual-four-axis-zero-confirmed', action='store_true')
    parser.add_argument('--suspended-clearance-confirmed', action='store_true')
    parser.add_argument('--recorded-bottom-confirmed', action='store_true')
    args = parser.parse_args()
    params = parse_gait_params(json.loads((ROOT / '.gait_params.json').read_text(encoding='utf-8')))
    p, report, stages = build_plan(params, args.action, profile=args.lift_profile, speed_scale=args.speed_scale)
    print(json.dumps(dict(action=args.action, modality=report.modality, route=report.route,
        stage_groups={s.stage_id: len(s.move_groups) for s in stages},
        final_rotary_offsets=next(s.sync_endpoints[-1] for s in stages if s.stage_id == 'S4'),
        suspended_only=True, lift_profile=args.lift_profile, speed_scale=args.speed_scale,
        lift_range_mm=lift_bounds(args.lift_profile)), ensure_ascii=False), flush=True)
    if not args.execute:
        return 0
    if not (args.port and args.reviewed_motor_deg and args.suspended_clearance_confirmed
            and args.recorded_bottom_confirmed):
        parser.error('execution requires explicit reviewed positions, suspended clearance and recorded bottom')
    if args.lift_profile == 'interior' and not args.interior_profile_confirmed:
        parser.error('interior profile changes lift travel; explicit confirmation required')
    if args.speed_scale == 2 and not args.double_speed_confirmed:
        parser.error('2x bench speed requires explicit confirmation')
    reviewed = dict(zip(range(1, 5), args.reviewed_motor_deg))
    reference = None
    if args.manual_zero_motor_deg is not None:
        if not args.manual_four_axis_zero_confirmed:
            parser.error('manual software reference requires explicit FOUR-axis mechanical-zero confirmation')
        reference = dict(zip(range(1, 5), args.manual_zero_motor_deg))
    elif args.manual_four_axis_zero_confirmed:
        parser.error('manual zero confirmation requires all four raw reference readings')
    SuspendedHost(None, None, reviewed, profile=args.lift_profile, zero_reference=reference, speed_scale=args.speed_scale)
    if STOP_FILE.exists():
        parser.error('stop request exists; inspect it before any motion')
    return execute_test(args.port, (1, 2, 3, 4),
        lambda s, r, d: run_gait(s, r, d, p, report, stages, reviewed,
                               profile=args.lift_profile, zero_reference=reference,
                               speed_scale=args.speed_scale, prepare_only=args.prepare_only),
        purpose='operator-authorized suspended one full gait; not field acceptance',
        # Longer complete gaits must retain more than the short-jog raw window.
        # Still bounded/rotating; manifest records the actual retention limits.
        event_prefix='bench.gait', diagnostic_max_bytes=32_000_000,
        test_context=dict(action=args.action, speed_scale=args.speed_scale, prepare_only=args.prepare_only,
            reviewed_motor_deg=reviewed, lift_profile=args.lift_profile,
            manual_zero_raw_motor_deg=reference,
            lift_range_mm=lift_bounds(args.lift_profile), max_rotary_offset_deg=180,
            suspended_clearance_confirmed=True, no_zero_or_config_writes=True,
            bench_source_sha256={name: hashlib.sha256((ROOT / 'scripts' / name).read_bytes()).hexdigest()
                for name in ('x42s_suspended_gait.py', 'x42s_lift_5mm.py', 'x42s_lift_cycle.py')}))


if __name__ == '__main__':
    raise SystemExit(main())
