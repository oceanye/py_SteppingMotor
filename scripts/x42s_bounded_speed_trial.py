"""One explicitly reviewed single-axis speed trial, not an automatic ramp.

Retained operator-confirmed mechanical reference; lifts restricted to 5..15 mm,
rotaries 0..4 output degrees. One MOVE, then fresh settled endpoint verification
and four-driver stop/disable. Never retries, auto-returns, homes or writes ZERO.
Default offline. Speed is a command cap; short trapezoids may not reach it.
"""
import argparse
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.x42s_recovery_leg import RecoveryLeg, run_legs
from scripts.x42s_suspended_gait import SCALES, TOL, STOP_FILE
from scripts.x42s_lift_5mm import execute_test

LIFT_SPEEDS = (1.25, 2.5, 5.)
ROTARY_SPEEDS = (.5, 1., 2., 4.)


@dataclass(frozen=True)
class SpeedTrial(RecoveryLeg):
    speed: float = 0.

    def validate(self):
        if (self.driver not in SCALES or set(self.reviewed) != set(SCALES)
                or self.zero_reference is None or set(self.zero_reference) != set(SCALES)
                or not all(math.isfinite(v) for v in (*self.reviewed.values(), *self.zero_reference.values(), self.target, self.speed))):
            raise ValueError('explicit four raw readings, retained manual reference and finite trial required')
        positions = {i: self.relative_motor_deg(i, raw)/SCALES[i] for i, raw in self.reviewed.items()}
        for i, pos in positions.items():
            lo, hi = (5., 15.) if i in (1, 3) else (0., 4.)
            if not lo-TOL[i]/SCALES[i] <= pos <= hi+TOL[i]/SCALES[i]:
                raise ValueError('trial start outside reviewed small-range fixture')
        linear = self.driver in (1, 3)
        if self.speed not in (LIFT_SPEEDS if linear else ROTARY_SPEEDS):
            raise ValueError('unreviewed speed tier')
        lo, hi = (5., 15.) if linear else (0., 4.)
        if not lo <= self.target <= hi:
            raise ValueError('trial target outside bounded range')
        start = positions[self.driver]
        if abs(self.target-start)*SCALES[self.driver] <= TOL[self.driver]:
            raise ValueError('trial too small to evaluate; no correction jog')
        return start

    def segment_limit(self):
        return 10. if self.driver in (1, 3) else 4.

    def waypoints(self):
        self.validate()
        return ((self.target, round(self.speed*SCALES[self.driver]*3200/360)),)


def run_trial(session, recorder, _, trial, *, confirmed=False, stop_file=STOP_FILE, hold_other_axes=False):
    start = trial.validate()
    recorder.event('motor', 'bench.speed_trial_plan', driver_id=trial.driver,
                   start_position=start, target_position=trial.target, command_speed=trial.speed,
                   output_unit='mm' if trial.driver in (1, 3) else 'deg',
                   command_speed_not_measured_speed=True, independent_support_only=True,
                   hold_other_axes=hold_other_axes,
                   no_retry_or_automatic_return=True)
    result = run_legs(session, recorder, (trial,), confirmed=confirmed, stop_file=stop_file,
                      hold_other_axes=hold_other_axes)
    return {**result, 'trial_command_speed': trial.speed, 'trial_start': start,
            'trial_target': trial.target, 'speed_peak_verified': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', type=int, choices=range(1, 5), required=True)
    parser.add_argument('--target', type=float, required=True)
    parser.add_argument('--speed', type=float, required=True)
    parser.add_argument('--reviewed-raw-motor-deg', type=float, nargs=4, required=True)
    parser.add_argument('--manual-zero-motor-deg', type=float, nargs=4, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--supported-clear-range-confirmed', action='store_true')
    parser.add_argument('--hold-other-axes-confirmed', action='store_true',
                        help='Rotary only: enable other axes for holding; unchanged strict position/speed guards')
    args = parser.parse_args()
    trial = SpeedTrial(args.driver, dict(zip(range(1, 5), args.reviewed_raw_motor_deg)), args.target,
                       dict(zip(range(1, 5), args.manual_zero_motor_deg)), args.speed)
    start = trial.validate()
    if args.hold_other_axes_confirmed and args.driver not in (2, 4):
        parser.error('holding mode is only for one rotary trial')
    print(f'PLAN ID{trial.driver} {start:.6f} -> {trial.target:g}, command speed {trial.speed:g}; NOT measured peak', flush=True)
    if not args.execute:
        return 0
    if not args.port or not args.supported_clear_range_confirmed or STOP_FILE.exists():
        parser.error('explicit supported clear range, port and no stop request required')
    return execute_test(args.port, trial.driver,
        lambda s, r, d: run_trial(s, r, d, trial, confirmed=True,
                                hold_other_axes=args.hold_other_axes_confirmed),
        purpose='operator-authorized bounded single-axis speed characterization; NOT maximum speed certification',
        event_prefix='bench.speed_trial', diagnostic_max_bytes=32_000_000,
        test_context=dict(reviewed_raw_motor_deg=trial.reviewed, mechanical_zero_raw_motor_deg=trial.zero_reference,
                          target=trial.target, command_speed=trial.speed, independent_support=True,
                          hold_other_axes=args.hold_other_axes_confirmed,
                          speed_peak_verified=False, no_zero_or_config_writes=True,
                          source_sha256={n: hashlib.sha256((ROOT/'scripts'/n).read_bytes()).hexdigest()
                                         for n in ('x42s_bounded_speed_trial.py', 'x42s_recovery_leg.py', 'x42s_lift_5mm.py')}))


if __name__ == '__main__':
    raise SystemExit(main())
