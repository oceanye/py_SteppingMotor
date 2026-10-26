import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from scripts.x42s_interior_return import run_return, validate, waypoints, main
from scripts.x42s_lift_return import ReturnPlan
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class InteriorReturnTests(unittest.TestCase):
    def rig(self, failure=None):
        clock = FakeClock()
        rig = GaitRig(clock, failure)
        rig.positions = {1: 2340.1, 2: 4.6, 3: 23399.9, 4: .2}
        return clock, rig

    def test_unequal_start_returns_only_selected_axis(self):
        for driver, plan in ((1, ReturnPlan(6.5, 5)), (3, ReturnPlan(65, 55))):
            clock, rig = self.rig()
            reviewed = dict(rig.positions)
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_return(rig, Mock(), driver, reviewed, plan, confirmed=True)
            self.assertAlmostEqual(result['positions_motor_deg'][driver], plan.target_mm*360, delta=.72)
            self.assertEqual(len(rig.moves), len(waypoints(plan)))
            for command in rig.moves:
                fields = command.split(',')
                self.assertEqual(int(fields[1]), driver)
                self.assertTrue(-1632 <= int(fields[3]) < 0)
                self.assertLessEqual(int(fields[4]), 4000)
            for i in set(reviewed)-{driver}:
                self.assertEqual(rig.positions[i], reviewed[i])
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))

    def test_slow_near_floor_and_bounded_plan(self):
        points = waypoints(ReturnPlan(15, 5))
        self.assertEqual(points[-1], (5., 160))
        self.assertIn((5.5, 800), points)
        self.assertIn((7., 4000), points)
        for start, target in ((65, 5), (6, 0), (71, 65), (5, 5), (5, 6)):
            with self.assertRaises(ValueError):
                waypoints(ReturnPlan(start, target))

    def test_faults_abort_without_second_move(self):
        for failure in ('wrong_ack', 'stall', 'fault', 'missing', 'stale', 'overshoot', 'no_speed', 'held_drift'):
            clock, rig = self.rig(failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_return(rig, Mock(), 3, dict(rig.positions), ReturnPlan(65, 55), confirmed=True)
            self.assertEqual(len(rig.moves), 1)

    def test_preflight_rejects_drift_unconfirmed_stop_without_enable(self):
        for reason in ('drift', 'stop', 'unconfirmed'):
            clock, rig = self.rig()
            reviewed = dict(rig.positions)
            kwargs = dict(confirmed=True)
            if reason == 'drift':
                rig.positions[2] += 1
            elif reason == 'stop':
                kwargs['stop_file'] = Path(__file__)
            else:
                kwargs['confirmed'] = False
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                    self.assertRaises((ValueError, RuntimeError)):
                run_return(rig, Mock(), 1, reviewed, ReturnPlan(6.5, 5), **kwargs)
            self.assertFalse(rig.moves)
            self.assertFalse(any(rig.enabled.values()))

    def test_reviewed_coordinates_validated_and_offline_never_opens_port(self):
        _, rig = self.rig()
        for driver, reviewed in ((2, rig.positions), (1, {**rig.positions, 1: 2400}),
                                (1, {**rig.positions, 3: 24000}), (1, {**rig.positions, 2: float('nan')})):
            with self.assertRaises(ValueError):
                validate(driver, reviewed, ReturnPlan(6.5, 5))
        argv = ['return', '--driver', '1', '--reviewed-motor-deg', '2340.1', '4.6', '23399.9', '.2',
                '--expected-start-mm', '6.5', '--target-mm', '5']
        with patch('sys.argv', argv), patch('scripts.x42s_interior_return.execute_test') as execute, patch('builtins.print'):
            self.assertEqual(main(), 0)
        execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
