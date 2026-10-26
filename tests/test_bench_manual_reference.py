import unittest
from unittest.mock import Mock, patch

from motor_control.gait_planner import default_gait_params
from scripts.x42s_suspended_gait import SuspendedHost, build_plan, run_gait, main
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class ManualReferenceTests(unittest.TestCase):
    def test_four_complete_offline_actions_keep_raw_feedback_and_use_new_zero(self):
        reference = {1: -1768., 2: 436.3, 3: -23246.5, 4: -1995.2}
        for action in ('L+', 'L-', 'R+', 'R-'):
            clock = FakeClock(); rig = GaitRig(clock)
            rig.positions = dict(reference)
            p, report, stages = build_plan(default_gait_params(), action)
            with self.subTest(action=action), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_gait(rig, Mock(), None, p, report, stages, dict(reference), zero_reference=reference)
            self.assertEqual(result['groups_completed'], sum(len(s.move_groups) for s in stages))
            self.assertEqual(result['mechanical_zero_raw_motor_deg'], reference)
            for i in (1, 3):
                self.assertAlmostEqual(result['positions_motor_deg'][i], reference[i], delta=.72)
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
            self.assertEqual(reference[1], -1768.)

    def test_no_implicit_reference_or_drift_admission(self):
        reference = {1: -1768., 2: 436.3, 3: -23246.5, 4: -1995.2}
        with self.assertRaises(ValueError): SuspendedHost(None, None, reference)
        for wrong in ({1: 0}, {**reference, 4: float('nan')}, {**reference, 1: 0}):
            with self.assertRaises(ValueError): SuspendedHost(None, None, reference, zero_reference=wrong)
        clock = FakeClock(); rig = GaitRig(clock)
        rig.positions = {**reference, 2: reference[2]+1}
        host = SuspendedHost(rig, Mock(), reference, zero_reference=reference)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), self.assertRaises(RuntimeError):
            host.prepare()
        self.assertFalse(rig.moves)

    def test_raw_bounds_are_translated_not_removed(self):
        reference = {1: -1768., 2: 436.3, 3: -23246.5, 4: -1995.2}
        clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(reference)
        host = SuspendedHost(rig, Mock(), reference, zero_reference=reference)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            host.prepare()
            for target in (reference[1]-1, reference[1]+71*360):
                with self.assertRaises(RuntimeError): host.dispatch({1: target}, {1: .25})
        self.assertFalse(rig.moves)

    def test_manual_reference_cli_requires_independent_confirmation(self):
        args = ['gait', '--execute', '--port', 'COM4', '--reviewed-motor-deg', '-1768', '436.3', '-23246.5', '-1995.2',
                '--manual-zero-motor-deg', '-1768', '436.3', '-23246.5', '-1995.2',
                '--suspended-clearance-confirmed', '--recorded-bottom-confirmed']
        with patch('sys.argv', args), patch('scripts.x42s_suspended_gait.execute_test') as execute, \
                patch('builtins.print'), patch('sys.stderr'), self.assertRaises(SystemExit): main()
        execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
