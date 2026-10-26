import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch

from scripts.x42s_bounded_speed_trial import SpeedTrial, LIFT_SPEEDS, ROTARY_SPEEDS, run_trial, main
from scripts.x42s_suspended_gait import TOL, SCALES
from test_bench_manual_park import REFERENCE
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock

PARK = {1: 32.1, 2: 436.3, 3: -21446.301, 4: -1995.2}


class SpeedTrialTests(unittest.TestCase):
    def test_every_tier_out_and_back_preserves_original_zero_and_other_axes(self):
        for driver in range(1, 5):
            linear = driver in (1, 3)
            for speed in LIFT_SPEEDS if linear else ROTARY_SPEEDS:
                clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(PARK)
                for target in ((15., 5.) if linear else (4., 0.)):
                    rig.enabled = {i: False for i in rig.enabled}; rig.moving.clear()
                    before = dict(rig.positions)
                    trial = SpeedTrial(driver, before, target, REFERENCE, speed)
                    with self.subTest(driver=driver, speed=speed, target=target), \
                            patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
                        result = run_trial(rig, Mock(), driver, trial, confirmed=True)
                    self.assertAlmostEqual(result['position_from_manual_zero'][driver], target, delta=TOL[driver]/SCALES[driver])
                    self.assertFalse(result['speed_peak_verified'])
                    for i in set(PARK)-{driver}: self.assertEqual(rig.positions[i], before[i])
                self.assertEqual(len(rig.moves), 2)
                self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))

    def test_unreviewed_speed_range_and_reference_rejected(self):
        for driver, target, speed, ref in ((1, 16, 1.25, REFERENCE), (1, 0, 1.25, REFERENCE),
                (2, 5, .5, REFERENCE), (4, -1, 1, REFERENCE), (1, 15, 10, REFERENCE),
                (4, 4, 8, REFERENCE), (1, 15, 1.25, None), (1, 15, float('nan'), REFERENCE)):
            with self.assertRaises(ValueError): SpeedTrial(driver, PARK, target, ref, speed).validate()

    def test_original_abort_guards_no_retry(self):
        for failure in ('wrong_ack', 'fault', 'stall', 'missing', 'stale', 'overshoot', 'no_speed', 'held_drift'):
            clock = FakeClock(); rig = GaitRig(clock, failure); rig.positions = dict(PARK)
            trial = SpeedTrial(3, PARK, 15, REFERENCE, 2.5)
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), self.assertRaises(RuntimeError):
                run_trial(rig, Mock(), 3, trial, confirmed=True)
            self.assertEqual(len(rig.moves), 1)

    def test_stop_and_missing_confirmation_never_move(self):
        trial = SpeedTrial(1, PARK, 15, REFERENCE, 1.25)
        for kwargs in ({'confirmed': False}, {'confirmed': True, 'stop_file': Path(__file__)}):
            rig = Mock()
            with self.assertRaises(ValueError): run_trial(rig, Mock(), 1, trial, **kwargs)
            rig.request.assert_not_called()
        args = ['speed', '--driver', '1', '--target', '15', '--speed', '1.25',
                '--reviewed-raw-motor-deg', *map(str, PARK.values()),
                '--manual-zero-motor-deg', *map(str, REFERENCE.values())]
        with patch('sys.argv', args), patch('scripts.x42s_bounded_speed_trial.execute_test') as execute, patch('builtins.print'):
            self.assertEqual(main(), 0)
        execute.assert_not_called()

    def test_real_pretrial_id4_drift_blocks_enable_and_move(self):
        # 2026-10-26 16:59: command still -1995.2, actual -1951.3.
        # No inference about manual touch vs passive motion, and no re-zero.
        clock = FakeClock(); rig = GaitRig(clock)
        rig.positions = {**PARK, 3: -21446.699, 4: -1951.3}
        trial = SpeedTrial(1, PARK, 15, REFERENCE, 1.25)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                self.assertRaisesRegex(RuntimeError, 'passive ID4'):
            run_trial(rig, Mock(), 1, trial, confirmed=True)
        self.assertFalse(rig.moves)
        self.assertFalse(any(',EN,1' in c for c in rig.commands))

    def test_rotary_holding_mode_enables_others_without_moving_them(self):
        for driver in (2, 4):
            clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(PARK)
            trial = SpeedTrial(driver, PARK, 4, REFERENCE, .5)
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_trial(rig, Mock(), driver, trial, confirmed=True, hold_other_axes=True)
            self.assertEqual(result['held_enabled_driver_ids'], sorted(set(PARK)-{driver}))
            self.assertEqual(len(rig.moves), 1)
            self.assertTrue(rig.moves[0].startswith(f'X42S,{driver},MOVE,'))
            for i in set(PARK)-{driver}:
                self.assertTrue(rig.enabled[i])
                self.assertEqual(rig.positions[i], PARK[i])

    def test_holding_mode_preserves_strict_micro_velocity_stop(self):
        class MicroRig(GaitRig):
            def request(self, command, **kwargs):
                r = super().request(command, **kwargs)
                return replace(r, speed=.1) if self.moves and command == 'X42S,S,3' else r
        clock = FakeClock(); rig = MicroRig(clock); rig.positions = dict(PARK)
        trial = SpeedTrial(4, PARK, 4, REFERENCE, .5)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), self.assertRaises(RuntimeError):
            run_trial(rig, Mock(), 4, trial, confirmed=True, hold_other_axes=True)
        self.assertEqual(len(rig.moves), 1)
        with self.assertRaises(ValueError):
            run_trial(Mock(), Mock(), 1, SpeedTrial(1, PARK, 15, REFERENCE, 1.25), confirmed=True, hold_other_axes=True)


if __name__ == '__main__':
    unittest.main()
