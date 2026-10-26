import unittest
from unittest.mock import Mock, patch

from motor_control.protocol import parse_line
from scripts.x42s_lift_5mm import run_lift, segment_steps, MAX_SEGMENT_PULSES, main
from scripts.x42s_micro_jog import quantized_target
from test_gait_realtime_fixes import FakeClock


class LiftRig:
    def __init__(self, clock, failure=None, residual=0):
        self.clock, self.failure, self.residual = clock, failure, residual
        self.positions = {1: 0., 2: 2.8, 3: 0., 4: 0.}
        self.enabled = {i: False for i in self.positions}
        self.commands, self.moving, self.moved_at = [], None, None

    def request_line(self, command, **kwargs):
        self.commands.append(command)
        self.enabled[int(command.split(',')[1])] = True
        if self.failure == 'enable_drift':
            self.positions[1] = 2
        return 'OK,'+command

    def request(self, command, **kwargs):
        self.commands.append(command)
        f = command.split(',')
        if f[1] != 'S':
            driver, steps, speed = int(f[1]), int(f[3]), int(f[4])
            self.moved_at = self.clock.now
            target = quantized_target(self.positions[driver], steps)
            self.moving = (driver, self.positions[driver], target)
            if self.failure == 'wrong_ack':
                driver = 4
            return parse_line(f'ACK,X42S,{driver},MOVE,{steps},{speed}')
        driver = int(f[2])
        speed, reached, fault, age, mask = 0., 1, 'none', 1, 63
        if self.moving and driver == self.moving[0]:
            _, origin, target = self.moving
            dt = self.clock.now-self.moved_at
            if dt < .4 or self.failure == 'missing':
                mask = 1
            final = target+self.residual
            self.positions[driver] = min(final, origin+max(0, dt-.1)*90)
            reached = int(self.positions[driver] >= final)
            speed = 0 if reached else 15
            if self.failure == 'blocked':
                self.positions[driver], speed, reached = origin, 0, 0
            elif self.failure == 'fault':
                fault = 'stall'
            elif self.failure == 'stale':
                age = 501
            elif self.failure == 'overshoot':
                self.positions[driver] = target+1
                mask = 63
            elif self.failure == 'wrong_direction':
                self.positions[driver] = origin-1
                mask = 63
            elif self.failure == 'speed_at_endpoint':
                speed = 1
        if self.moving and driver == 2 and self.failure == 'other_drift':
            self.positions[2] = 4
        pos = self.positions[driver]
        target = self.moving[2] if self.moving and self.moving[0] == driver else pos
        return parse_line(f'X42S,{driver},S,1,{int(self.enabled[driver])},{reached},{fault},'
                          f'{target},{pos},0,{speed},0,30,{age},0,{mask}')

    def moves(self):
        return [c for c in self.commands if ',MOVE,' in c]


class BenchLift5mmTests(unittest.TestCase):
    def run_case(self, failure=None, residual=0):
        clock = FakeClock()
        rig = LiftRig(clock, failure, residual)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
            result = run_lift(rig, Mock(), 1, confirmed=True)
        return rig, result

    def test_ten_positive_segments_reach_5mm_without_zero_or_other_axis_moves(self):
        rig, result = self.run_case()
        self.assertAlmostEqual(result['measured_mm'], 5)
        self.assertEqual(rig.moves(), ['X42S,1,MOVE,1600,800']*10)
        self.assertEqual([c for c in rig.commands if ',EN,' in c], ['X42S,1,EN,1'])
        self.assertEqual(rig.positions[2], 2.8)
        self.assertFalse(any('ZERO' in c or ',RAW,' in c for c in rig.commands))

    def test_waypoints_correct_residual_without_accumulating_height_error(self):
        for residual in (-.3, .3):
            with self.subTest(residual=residual):
                rig, result = self.run_case(residual=residual)
                self.assertAlmostEqual(result['measured_mm'], 5, delta=.002)
                self.assertEqual(len(rig.moves()), 10)
                self.assertTrue(all(0 < int(c.split(',')[3]) <= MAX_SEGMENT_PULSES for c in rig.moves()))

    def test_any_failure_aborts_first_segment_without_retry_or_return(self):
        cases = {
            'wrong_ack': 'precisely acknowledged', 'missing': 'position missing',
            'blocked': 'no measured progress', 'fault': 'unhealthy',
            'stale': 'unhealthy', 'overshoot': 'outside travel',
            'wrong_direction': 'outside travel', 'other_drift': 'untested ID2',
            'speed_at_endpoint': 'arrival timeout',
        }
        for failure, message in cases.items():
            clock = FakeClock()
            rig = LiftRig(clock, failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'):
                with self.assertRaisesRegex(RuntimeError, message):
                    run_lift(rig, Mock(), 1, confirmed=True)
            self.assertEqual(len(rig.moves()), 1)

    def test_enable_position_change_cannot_lead_to_move(self):
        clock = FakeClock()
        rig = LiftRig(clock, 'enable_drift')
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, 'changed during enable'):
                run_lift(rig, Mock(), 1, confirmed=True)
        self.assertEqual(rig.moves(), [])

    def test_not_at_bottom_cannot_repeat_another_5mm(self):
        clock = FakeClock()
        rig = LiftRig(clock)
        rig.positions[1] = 1800
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, 'not near confirmed bottom'):
                run_lift(rig, Mock(), 1, confirmed=True)
        self.assertFalse(any(',EN,' in c or ',MOVE,' in c for c in rig.commands))

    def test_explicit_confirmations_and_lift_role_required_before_io(self):
        for driver, confirmed in ((1, False), (2, True), (4, True)):
            session = Mock()
            with self.subTest(driver=driver, confirmed=confirmed), self.assertRaises(ValueError):
                run_lift(session, Mock(), driver, confirmed=confirmed)
            session.request.assert_not_called()
            session.request_line.assert_not_called()

    def test_segment_bounds_quantize_toward_target_and_never_reverse(self):
        for origin in (0, .1, .3, 179.7, 180.3, 1619.7, 1620.3):
            target = min(1800, origin+180)
            steps = segment_steps(origin, target)
            self.assertGreater(steps, 0)
            self.assertLessEqual(steps, MAX_SEGMENT_PULSES)
            self.assertLessEqual(quantized_target(origin, steps), target+1e-9)
        for origin, target in ((0, 360), (-.1, 180), (180, 0), (1800, 1800),
                               (1700, 1801), (float('nan'), 180), (0, float('inf'))):
            with self.subTest(origin=origin, target=target), self.assertRaises(ValueError):
                segment_steps(origin, target)

    def test_cli_always_stops_and_closes_and_never_reports_unconfirmed_stop_as_success(self):
        module = 'scripts.x42s_lift_5mm.'
        args = ['probe', '--port', 'FAKE', '--driver', '1', '--positive-is-extension',
                '--bottom-zero-confirmed', '--supported-10mm-clearance']
        for mode in ('success', 'abort', 'keyboard', 'unconfirmed', 'poisoned'):
            session, port, recorder = Mock(), Mock(), Mock()
            stops = []
            def request_line(command, **kwargs):
                if command == 'X42S,INFO':
                    return 'OK,X42S,INFO,2,test'
                self.assertEqual(command, 'ESTOP')
                stops.append(command)
                if len(stops) == 2 and mode == 'poisoned':
                    raise RuntimeError('request matcher poisoned')
                return 'OK,ESTOP'
            session.request_line.side_effect = request_line
            error = (RuntimeError('motion failed') if mode in ('abort', 'poisoned')
                     else KeyboardInterrupt() if mode == 'keyboard' else None)
            with self.subTest(mode=mode), patch('sys.argv', args), patch('time.sleep'), \
                    patch('builtins.print'), patch(module+'serial.Serial', return_value=port), \
                    patch(module+'SerialSession', return_value=session), \
                    patch(module+'DiagnosticRecorder', return_value=recorder), \
                    patch(module+'environment_summary', return_value={}), \
                    patch(module+'run_lift', return_value={}, side_effect=error), \
                    patch(module+'confirm_disabled_standstill', side_effect=[True, mode != 'unconfirmed']):
                self.assertEqual(main(), 0 if mode == 'success' else 2)
            self.assertEqual(len(stops), 2)
            session.close.assert_called_once()
            recorder.close.assert_called_once()
            if mode == 'poisoned':
                port.write.assert_called_once_with(b'ESTOP\n')


if __name__ == '__main__':
    unittest.main()
