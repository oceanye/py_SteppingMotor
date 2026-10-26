import unittest
from unittest.mock import Mock, patch

from motor_control.protocol import parse_line
from scripts.x42s_lift_return import ReturnPlan, return_steps, run_return, main
from scripts.x42s_micro_jog import quantized_target
from test_gait_realtime_fixes import FakeClock


class ReturnRig:
    def __init__(self, clock, start=70, failure=None):
        self.clock, self.failure = clock, failure
        self.positions = {1: start*360-.1, 2: 3., 3: start*360-.1, 4: .1}
        self.enabled = {i: False for i in self.positions}
        self.commands, self.moves, self.moving = [], [], {}

    def request_line(self, command, **kwargs):
        self.commands.append(command)
        if command == 'ESTOP':
            self.enabled = {i: False for i in self.positions}
            self.moving.clear()
        else:
            self.enabled[int(command.split(',')[1])] = True
        return 'OK,'+command

    def request(self, command, **kwargs):
        self.commands.append(command)
        f = command.split(',')
        if f[1] != 'S':
            self.moves.append(command)
            i, steps, pps = int(f[1]), int(f[3]), int(f[4])
            self.moving[i] = (self.clock.now, self.positions[i], quantized_target(self.positions[i], steps), pps)
            if self.failure == 'wrong_ack' and i == 3:
                pps += 1
            return parse_line(f'ACK,X42S,{i},MOVE,{steps},{pps}')
        i = int(f[2])
        target, reached, speed, mask, age, fault = self.positions[i], 1, 0, 63, 1, 'none'
        if i in self.moving:
            at, origin, target, pps = self.moving[i]
            dt = self.clock.now-at
            self.positions[i] = max(target, origin-max(0, dt-.1)*pps*360/3200)
            reached = int(self.positions[i] == target)
            speed = 0 if reached else -pps*60/3200
            if dt < .4:
                mask = 1
            if i == 3:
                if self.failure == 'stall':
                    self.positions[i], reached, speed = origin, 0, 0
                elif self.failure == 'fault':
                    fault = 'stall'
                elif self.failure == 'missing':
                    mask = 1
                elif self.failure == 'stale':
                    age = 501
                elif self.failure == 'wrong_direction':
                    self.positions[i], mask = origin+1, 63
                elif self.failure == 'overshoot':
                    self.positions[i], mask = target-1, 63
                elif self.failure == 'no_speed':
                    mask = 7
                elif self.failure == 'bottom_stall' and target <= .72:
                    self.positions[i], reached, speed = origin, 0, 0
                elif self.failure == 'bottom_overshoot' and target <= .72:
                    # Field regression: -0.9 motor deg = -0.0025 mm at bottom;
                    # position-only mask must still trigger travel protection.
                    self.positions[i], mask = -.9, 2
        if self.moves and i == 2 and self.failure == 'passive_drift':
            self.positions[i] = 4
        return parse_line(f'X42S,{i},S,1,{int(self.enabled[i])},{reached},{fault},'
                          f'{target},{self.positions[i]},0,{speed},0,30,{age},0,{mask}')


class BenchLiftReturnTests(unittest.TestCase):
    def run_plan(self, start, end, failure=None):
        clock, recorder = FakeClock(), Mock()
        rig = ReturnRig(clock, start, failure)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
            result = run_return(rig, recorder, (1, 3), ReturnPlan(start, end), confirmed=True)
        return rig, recorder, result

    def test_bulk_is_five_times_speed_and_only_commands_negative_small_lift_moves(self):
        rig, _, result = self.run_plan(70, 60)
        self.assertEqual(len(rig.moves), 40)
        for c in rig.moves:
            f = c.split(',')
            self.assertIn(int(f[1]), (1, 3))
            self.assertTrue(-1632 <= int(f[3]) < 0)
            self.assertEqual(int(f[4]), 4000)
        for pos in result['positions_motor_deg'].values():
            self.assertAlmostEqual(pos/360, 60, delta=.002)
        self.assertEqual(rig.positions[2], 3)
        self.assertEqual(rig.positions[4], .1)
        self.assertFalse(any('ZERO' in c or 'RAW' in c or ',EN,0' in c for c in rig.commands))

    def test_slow_zones_and_zero_endpoint_without_zero_write_or_negative_command_target(self):
        rig, recorder, result = self.run_plan(3, 0)
        events = [c.kwargs for c in recorder.event.call_args_list if c.args[1] == 'bench.lift_return_waypoint']
        self.assertTrue(all(e['speed_pps'] == 4000 for e in events if e['target_mm'] >= 2))
        self.assertTrue(all(e['speed_pps'] == 800 for e in events if .5 <= e['target_mm'] < 2))
        self.assertTrue(all(e['speed_pps'] == 160 for e in events if e['target_mm'] < .5))
        self.assertEqual(events[-1]['target_mm'], 0)
        self.assertTrue(all(e['target_mm'] >= 0 for e in events))
        for pos in result['positions_motor_deg'].values():
            self.assertTrue(0 <= pos <= .72)
        self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))

    def test_failed_move_never_retries_or_reverses_outward(self):
        for failure in ('wrong_ack', 'stall', 'fault', 'missing', 'stale', 'wrong_direction',
                        'overshoot', 'no_speed', 'passive_drift'):
            clock = FakeClock()
            rig = ReturnRig(clock, failure=failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_return(rig, Mock(), (1, 3), ReturnPlan(70, 69), confirmed=True)
            self.assertEqual(len(rig.moves), 2)
            self.assertTrue(all(int(c.split(',')[3]) < 0 for c in rig.moves))

    def test_stall_at_bottom_or_negative_overshoot_is_not_success_and_never_clears_position(self):
        for failure in ('bottom_stall', 'bottom_overshoot'):
            clock = FakeClock()
            rig = ReturnRig(clock, start=.1, failure=failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_return(rig, Mock(), (1, 3), ReturnPlan(.1, 0), confirmed=True)
            self.assertEqual(len(rig.moves), 2)
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))

    def test_invalid_bounds_roles_confirmation_or_start_never_enables(self):
        for start, target in ((70, -1), (71, 69), (70, 0), (0, 0), (.05, 0),
                              (65, 70), (70, 70), (float('nan'), 65), (70, float('inf'))):
            session = Mock()
            with self.subTest(start=start, target=target), self.assertRaises(ValueError):
                run_return(session, Mock(), (1, 3), ReturnPlan(start, target), confirmed=True)
            session.request.assert_not_called()
            session.request_line.assert_not_called()
        for drivers, confirmed in (((1,), True), ((2, 4), True), ((1, 3), False)):
            with self.assertRaises(ValueError):
                run_return(Mock(), Mock(), drivers, ReturnPlan(70, 69), confirmed=confirmed)
        clock = FakeClock()
        rig = ReturnRig(clock, start=65)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), self.assertRaises(RuntimeError):
            run_return(rig, Mock(), (1, 3), ReturnPlan(70, 69), confirmed=True)
        self.assertEqual(rig.moves, [])
        self.assertFalse(any(rig.enabled.values()))

    def test_quantization_stays_above_target_and_caps_each_command(self):
        plan = ReturnPlan(1, 0)
        for origin, target in ((360.1, .5), (179.9, .25), (36.1, 0), (35.9, 0)):
            steps = return_steps(origin, target, plan)
            self.assertTrue(-1632 <= steps < 0)
            self.assertGreaterEqual(quantized_target(origin, steps), target*360-1e-8)
        for origin, target in ((360, -1), (360, 1), (360, 0), (0, 0), (float('nan'), 0)):
            with self.subTest(origin=origin, target=target), self.assertRaises(ValueError):
                return_steps(origin, target, plan)

    def test_cli_rejects_whole_seventy_mm_before_opening_port(self):
        args = ['return', '--port', 'FAKE', '--expected-start-mm', '70', '--target-mm', '0',
                '--recorded-bottom-confirmed', '--supported-return-confirmed']
        with patch('sys.argv', args), patch('scripts.x42s_lift_return.execute_test') as execute, \
                patch('sys.stderr'), self.assertRaises(SystemExit):
            main()
        execute.assert_not_called()

    def test_explicit_reviewed_positions_do_not_rezero_or_relax_enable_drift_guard(self):
        for moved_again in (False, True):
            clock = FakeClock()
            rig = ReturnRig(clock)
            rig.positions[1], rig.positions[3] = 25196.301, 25199.0
            request_line = rig.request_line
            def enable(command, **kwargs):
                result = request_line(command, **kwargs)
                if moved_again and command == 'X42S,1,EN,1':
                    rig.positions[1] += 1
                return result
            rig.request_line = enable
            with self.subTest(moved_again=moved_again), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'):
                if moved_again:
                    with self.assertRaisesRegex(RuntimeError, 'drift during enable'):
                        run_return(rig, Mock(), (1, 3), ReturnPlan(70, 69), confirmed=True,
                                   expected_positions={1: 25196.301, 3: 25199.0})
                    self.assertFalse(rig.moves)
                else:
                    result = run_return(rig, Mock(), (1, 3), ReturnPlan(70, 69), confirmed=True,
                                        expected_positions={1: 25196.301, 3: 25199.0})
                    self.assertEqual(len(rig.moves), 4)
                    self.assertTrue(all(abs(v/360-69) <= .002 for v in result['positions_motor_deg'].values()))
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
        for expected in ({1: 25196.301}, {1: 25000, 3: 25199}, {1: float('nan'), 3: 25199}):
            session = Mock()
            with self.assertRaises(ValueError):
                run_return(session, Mock(), (1, 3), ReturnPlan(70, 69), confirmed=True, expected_positions=expected)
            session.request.assert_not_called()


if __name__ == '__main__':
    unittest.main()
