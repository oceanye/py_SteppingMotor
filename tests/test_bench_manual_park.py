"""Offline tests; recorded coordinates do not constitute live admission."""
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from scripts.x42s_recovery_leg import RecoveryLeg, group_waypoints, run_legs, main
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock

REFERENCE = {1: -1768., 2: 436.3, 3: -23246.5, 4: -1995.2}
REVIEWED = {1: 5431.7, 2: 434.7, 3: -16049.8, 4: -1959.3}


class ManualParkTests(unittest.TestCase):
    def test_pair_lifts_then_small_rotary_returns_keep_raw_manual_reference(self):
        clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(REVIEWED)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
            for drivers, target in (((1, 3), 5), ((2,), 0), ((4,), 0)):
                rig.enabled = {i: False for i in rig.enabled}; rig.moving.clear()
                legs = tuple(RecoveryLeg(i, dict(rig.positions), target, REFERENCE) for i in drivers)
                result = run_legs(rig, Mock(), legs, confirmed=True)
                for i in drivers:
                    self.assertAlmostEqual(result['position_from_manual_zero'][i], target,
                                           delta=.002 if i in (1, 3) else .225/21.5)
                self.assertNotIn('inferred_historical_position', result)
                self.assertEqual(result['mechanical_zero_raw_motor_deg'], REFERENCE)
        self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
        self.assertFalse(result['home_completed'])

    def test_paired_lifts_entire_independent_progress_rectangle_under_original_limit(self):
        legs = tuple(RecoveryLeg(i, REVIEWED, 5, REFERENCE) for i in (1, 3))
        previous = {l.driver: l.validate() for l in legs}
        points = group_waypoints(legs)
        self.assertEqual(len(points), 38)
        for group in points:
            for i, (target, pps) in group.items():
                self.assertLessEqual(previous[i]-target, .4+1e-8)
                self.assertLessEqual(pps, 4000)
            self.assertLess(max(previous[1]-group[3][0], previous[3]-group[1][0])+.004, .5)
            previous = {i: v[0] for i, v in group.items()}

    def test_real_rotary_half_lsb_nominal_is_removed_without_relaxing_tolerance(self):
        from scripts.x42s_recovery_leg import steps_for
        from scripts.x42s_micro_jog import quantized_target
        from scripts.x42s_suspended_gait import TOL
        leg = RecoveryLeg(4, REVIEWED, 0, REFERENCE)
        raw = REVIEWED[4]
        for target, _ in leg.waypoints():
            steps, nominal = steps_for(leg, raw, target)
            self.assertAlmostEqual(nominal*10, round(nominal*10))
            encoded = quantized_target(raw, steps)
            # One measured motor-position LSB toward the start, as in real run.
            raw = round(encoded+.1, 1)
            self.assertLessEqual(abs(raw-nominal), TOL[4]+1e-8)
        self.assertEqual(TOL[4], .225)
        self.assertAlmostEqual(leg.raw_target(leg.waypoints()[-1][0]), REFERENCE[4])

    def test_manual_mode_rejects_bottom_outward_large_rotation_and_bad_references(self):
        for driver, target, reviewed, ref in (
                (1, 0, REVIEWED, REFERENCE), (3, 21, REVIEWED, REFERENCE),
                (2, .1, REVIEWED, REFERENCE), (4, 0, {**REVIEWED, 4: REFERENCE[4]+6*21.5}, REFERENCE),
                (1, 5, REVIEWED, {1: 0}), (1, 5, REVIEWED, {**REFERENCE, 1: float('nan')})):
            with self.assertRaises(ValueError): RecoveryLeg(driver, reviewed, target, ref).validate()
        legs = (RecoveryLeg(1, REVIEWED, 5, REFERENCE), RecoveryLeg(3, REVIEWED, 5))
        with self.assertRaises(ValueError): group_waypoints(legs)

    def test_reviewed_start_change_or_stop_prevents_motion(self):
        for reason in ('drift', 'stop', 'unconfirmed'):
            clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(REVIEWED)
            legs = tuple(RecoveryLeg(i, REVIEWED, 5, REFERENCE) for i in (1, 3))
            kwargs = dict(confirmed=reason != 'unconfirmed')
            if reason == 'drift': rig.positions[2] += 1
            if reason == 'stop': kwargs['stop_file'] = Path(__file__)
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), self.assertRaises((ValueError, RuntimeError)):
                run_legs(rig, Mock(), legs, **kwargs)
            self.assertFalse(rig.moves)

    def test_pair_ack_fault_and_passive_drift_still_abort(self):
        for failure in ('wrong_ack', 'fault', 'stall', 'missing', 'stale', 'overshoot', 'no_speed', 'held_drift'):
            clock = FakeClock(); rig = GaitRig(clock, failure); rig.positions = dict(REVIEWED)
            legs = tuple(RecoveryLeg(i, REVIEWED, 5, REFERENCE) for i in (1, 3))
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), self.assertRaises(RuntimeError):
                run_legs(rig, Mock(), legs, confirmed=True)
            self.assertLessEqual(len(rig.moves), 2)

    def test_envelope_failure_prevents_entire_pair(self):
        reviewed = {**REVIEWED, 3: REVIEWED[3]+180}
        clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(reviewed)
        legs = tuple(RecoveryLeg(i, reviewed, 5, REFERENCE) for i in (1, 3))
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                self.assertRaisesRegex(RuntimeError, 'rectangle'):
            run_legs(rig, Mock(), legs, confirmed=True)
        self.assertFalse(rig.moves)

    def test_manual_cli_never_uses_old_powercycle_evidence_or_implicit_confirmation(self):
        args = ['park', '--driver', '1', '--with-paired-axis', '--target', '5',
                '--reviewed-raw-motor-deg', *map(str, REVIEWED.values()),
                '--manual-zero-motor-deg', *map(str, REFERENCE.values())]
        for live in (False, True):
            argv = args+(['--execute', '--port', 'COM4', '--same-epoch-supported-return-confirmed'] if live else [])
            with patch('sys.argv', argv), patch('scripts.x42s_recovery_leg.validate_evidence') as old, \
                    patch('scripts.x42s_recovery_leg.execute_test') as execute, patch('builtins.print'), patch('sys.stderr'):
                if live:
                    with self.assertRaises(SystemExit): main()
                else:
                    self.assertEqual(main(), 0)
                old.assert_not_called(); execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
