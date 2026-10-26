import unittest
from unittest.mock import Mock, patch

from motor_control.protocol import parse_line
from scripts.x42s_lift_stage import StagePlan, stage_steps, run_stage, main
from test_bench_lift_cycle import CycleRig
from test_gait_realtime_fixes import FakeClock


class BenchLiftStageTests(unittest.TestCase):
    def make_rig(self, failure=None):
        clock = FakeClock()
        rig = CycleRig(clock, failure)
        rig.positions[1], rig.positions[3] = 1872., 1871.5
        return clock, rig

    def test_pair_advances_ten_mm_in_twenty_small_commands_without_rotating_or_zero(self):
        clock, rig = self.make_rig()
        plan = StagePlan(5.2, 15.2, 70)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
            result = run_stage(rig, Mock(), (1, 3), plan, confirmed=True)
        self.assertEqual(len(rig.moves), 40)
        for command in rig.moves:
            fields = command.split(',')
            self.assertIn(int(fields[1]), (1, 3))
            self.assertGreater(int(fields[3]), 0)
            self.assertLessEqual(int(fields[3]), 1632)
            self.assertEqual(int(fields[4]), 800)
        for position in result['positions_motor_deg'].values():
            self.assertAlmostEqual(position/360, 15.2, delta=.002)
        self.assertLessEqual(result['max_observed_spread_mm'], .2)
        self.assertEqual(rig.positions[2], 2.9)
        self.assertEqual(rig.positions[4], 0)

    def test_invalid_plan_cannot_open_port_or_enable(self):
        invalid = ((5.2, 70, 70), (55, 65.1, 70), (5.2, 15.21, 70), (5.2, 4, 70),
                   (0, 10, 70), (5.2, 5.2, 70), (5.2, 10, 76), (5.2, 10, 8),
                   (65, 70, 70), (65, 70, 74.9), (65, 70.01, 75), (65.1, 70, 75),
                   (float('nan'), 15, 70), (5, float('inf'), 70), (5, 10, float('nan')))
        for fields in invalid:
            session = Mock()
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                run_stage(session, Mock(), (1, 3), StagePlan(*fields), confirmed=True)
            session.request.assert_not_called()
            session.request_line.assert_not_called()
        args = ['stage', '--port', 'FAKE', '--expected-start-mm', '5.2', '--target-mm', '70',
                '--confirmed-safe-height-mm', '70', '--positive-extension-confirmed', '--support-confirmed']
        with patch('sys.argv', args), patch('scripts.x42s_lift_stage.execute_test') as execute, \
                patch('sys.stderr'), self.assertRaises(SystemExit):
            main()
        execute.assert_not_called()

    def test_stale_start_coordinate_or_unconfirmed_setup_never_enables(self):
        for bad_position, confirmed in ((0, True), (1800, True), (1872, False)):
            clock, rig = self.make_rig()
            rig.positions[1] = bad_position
            with self.subTest(position=bad_position, confirmed=confirmed), \
                    patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
                with self.assertRaises((ValueError, RuntimeError)):
                    run_stage(rig, Mock(), (1, 3), StagePlan(5.2, 15.2, 70), confirmed=confirmed)
            self.assertFalse(any(rig.enabled.values()))
            self.assertFalse(rig.moves)

    def test_any_fault_stops_dispatch_without_retry_or_return(self):
        for failure in ('wrong_second_ack', 'stall', 'fault', 'overshoot', 'missing', 'passive_drift'):
            clock, rig = self.make_rig(failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_stage(rig, Mock(), (1, 3), StagePlan(5.2, 15.2, 70), confirmed=True)
            self.assertEqual(len(rig.moves), 2)

    def test_unknown_endpoint_speed_is_not_success(self):
        clock, rig = self.make_rig()
        request = rig.request
        def no_speed(command, **kwargs):
            r = request(command, **kwargs)
            if rig.moves and command in ('X42S,S,1', 'X42S,S,3'):
                r = parse_line(r.raw.rsplit(',', 1)[0]+',7')
            return r
        rig.request = no_speed
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                self.assertRaisesRegex(RuntimeError, 'arrival timeout'):
            run_stage(rig, Mock(), (1, 3), StagePlan(5.2, 15.2, 70), confirmed=True)
        self.assertEqual(len(rig.moves), 2)

    def test_no_single_member_or_reverse_or_large_unsegmented_command(self):
        for drivers in ((1,), (3,), (2, 4), (3, 1)):
            with self.subTest(drivers=drivers), self.assertRaises(ValueError):
                run_stage(Mock(), Mock(), drivers, StagePlan(5.2, 15.2, 70), confirmed=True)
        plan = StagePlan(5.2, 15.2, 70)
        for origin, target in ((1872, 15.2), (1872, 5.2), (1872, 70), (0, 5.7), (float('nan'), 5.7)):
            with self.subTest(origin=origin, target=target), self.assertRaises(ValueError):
                stage_steps(origin, target, plan)
        self.assertEqual(stage_steps(1872, 5.7, plan), 1600)

    def test_short_final_segment_and_safety_margin(self):
        plan = StagePlan(55.2, 65, 70)
        self.assertEqual(len(plan.waypoints()), 20)
        self.assertEqual(plan.waypoints()[-2:], (64.7, 65))
        self.assertEqual(StagePlan(5.2, 5.3, 70).waypoints(), (5.3,))
        for start, end in ((5.2, 15.2), (15.2, 25.2), (25.2, 35.2), (35.2, 45.2), (45.2, 55.2)):
            points = StagePlan(start, end, 70).waypoints()
            self.assertEqual(len(points), 20)
            self.assertEqual(len(set(points)), 20)
            self.assertEqual(points[-1], end)

    def test_final_five_mm_requires_declared_margin_and_stops_at_seventy(self):
        clock, rig = self.make_rig()
        rig.positions[1] = rig.positions[3] = 23399.9
        plan = StagePlan(65, 70, 75)
        self.assertEqual(plan.waypoints(), tuple(65+.5*i for i in range(1, 11)))
        recorder = Mock()
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
            result = run_stage(rig, recorder, (1, 3), plan, confirmed=True)
        self.assertEqual(len(rig.moves), 20)
        self.assertTrue(all(0 < int(c.split(',')[3]) <= 1632 for c in rig.moves))
        self.assertTrue(all(c.startswith(('X42S,1,MOVE,', 'X42S,3,MOVE,')) for c in rig.moves))
        for position in result['positions_motor_deg'].values():
            self.assertAlmostEqual(position/360, 70, delta=.002)
        targets = [c.kwargs['target_mm'] for c in recorder.event.call_args_list
                   if c.args[1] == 'bench.lift_stage_waypoint']
        self.assertEqual(targets, list(plan.waypoints()))
        self.assertEqual(rig.positions[2], 2.9)
        self.assertEqual(rig.positions[4], 0)


if __name__ == '__main__':
    unittest.main()
