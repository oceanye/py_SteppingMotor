import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from scripts.x42s_powercycle_return import (
    HISTORICAL, REBOOT_RAW, estimated_motor_deg, main, run_return, segment_steps, waypoints,
)
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class PowercycleReturnTests(unittest.TestCase):
    def rig(self, failure=None):
        clock = FakeClock()
        rig = GaitRig(clock, failure)
        rig.positions = dict(REBOOT_RAW)
        return clock, rig

    def test_complete_first_stage_keeps_raw_and_estimated_coordinates_separate(self):
        clock, rig = self.rig()
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
            result = run_return(rig, Mock(), 3, confirmed=True)
        self.assertEqual(len(rig.moves), 10)
        self.assertAlmostEqual(result['inferred_historical_motor_deg'][3] / 360, 60, delta=.002)
        self.assertLess(result['raw_motor_deg'][3], -1798)
        for i in (1, 2, 4):
            self.assertEqual(result['raw_motor_deg'][i], 0)
            self.assertEqual(result['inferred_historical_motor_deg'][i], HISTORICAL[i])
        self.assertFalse(result['home_completed'])
        self.assertFalse(result['absolute_position_independently_verified'])
        for cmd in rig.moves:
            f = cmd.split(',')
            self.assertEqual(f[1:3], ['3', 'MOVE'])
            self.assertTrue(-1632 <= int(f[3]) < 0)
            self.assertEqual(int(f[4]), 1600)
        self.assertFalse(any('RAW' in c or 'ZERO' in c for c in rig.commands))

    def test_faults_never_release_second_move(self):
        for failure in ('wrong_ack', 'stall', 'fault', 'missing', 'stale', 'overshoot', 'no_speed', 'held_drift'):
            clock, rig = self.rig(failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_return(rig, Mock(), 3, confirmed=True)
            self.assertEqual(len(rig.moves), 1)

    def test_wrong_start_or_unconfirmed_or_stop_never_moves(self):
        for reason in ('moved', 'unconfirmed', 'stop', 'enable_drift'):
            clock, rig = self.rig('enable_drift' if reason == 'enable_drift' else None)
            kwargs = dict(confirmed=True)
            if reason == 'moved': rig.positions[3] = -1800
            if reason == 'unconfirmed': kwargs['confirmed'] = False
            if reason == 'stop': kwargs['stop_file'] = Path(__file__)
            with self.subTest(reason=reason), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), self.assertRaises((ValueError, RuntimeError)):
                run_return(rig, Mock(), 3, **kwargs)
            self.assertFalse(rig.moves)

    def test_bounds_disallow_zero_rotary_motion_or_large_segment(self):
        self.assertEqual(waypoints()[-1], 60)
        for raw, target in ((0, 60), (0, 0), (1, 64.5), (float('nan'), 64.5), (-1800, 64.5)):
            with self.assertRaises(ValueError): segment_steps(raw, target)
        steps, raw = segment_steps(0, 64.5)
        self.assertLess(steps, 0)
        self.assertAlmostEqual(estimated_motor_deg(3, raw) / 360, 64.5)

    def test_offline_default_or_missing_confirmation_never_opens_port(self):
        for args, raises in (([], False), (['--execute', '--port', 'COM4'], True)):
            with patch('sys.argv', ['return'] + args), patch('scripts.x42s_powercycle_return.validate_evidence'), \
                    patch('scripts.x42s_powercycle_return.execute_test') as execute, patch('builtins.print'), patch('sys.stderr'):
                if raises:
                    with self.assertRaises(SystemExit): main()
                else:
                    self.assertEqual(main(), 0)
                execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
