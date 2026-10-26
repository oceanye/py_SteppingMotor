import unittest
from unittest.mock import Mock, patch
from motor_control.protocol import parse_line
from scripts.x42s_micro_jog import run_cycle, lift_steps, quantized_target
from test_gait_realtime_fixes import FakeClock


class BenchMicroJogTests(unittest.TestCase):
    def rig(self, fail=False, residual=False, driver_id=1):
        positions = {i: 0.0 for i in range(1, 5)}
        commands = []
        def request(command, **kwargs):
            commands.append(command)
            fields = command.split(',')
            if fields[1] == 'S':
                driver = int(fields[2])
                fault = 'stall' if fail and any(',MOVE,' in c for c in commands) else 'none'
                return parse_line(f'X42S,{driver},S,1,1,1,{fault},0,{positions[driver]},0,0,0,28,1,0,63')
            driver, steps = int(fields[1]), int(fields[3])
            positions[driver] = quantized_target(positions[driver], steps)
            if residual:
                positions[driver] -= .1 if steps > 0 else .3
            return parse_line(f'ACK,X42S,{driver},MOVE,{steps},80')
        return Mock(request=request, request_line=lambda *a, **k:f'OK,X42S,{driver_id},EN,1'), commands

    def test_bounded_round_trip_uses_actual_feedback(self):
        clock = FakeClock()
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            session, commands = self.rig()
            result = run_cycle(session, Mock(), 1, lift_bottom_zero=True)
        self.assertAlmostEqual(result['returned_motor_deg'], 1.1)
        self.assertEqual(result['final_reference_motor_deg'], 1.)
        self.assertEqual([c for c in commands if ',MOVE,' in c], ['X42S,1,MOVE,71,80', 'X42S,1,MOVE,-61,80'])

    def test_fault_never_sends_return_or_retry(self):
        clock = FakeClock()
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            session, commands = self.rig(fail=True)
            with self.assertRaisesRegex(RuntimeError, 'unhealthy'):
                run_cycle(session, Mock(), 1, lift_bottom_zero=True)
        self.assertEqual([c for c in commands if ',MOVE,' in c], ['X42S,1,MOVE,71,80'])

    def test_reserved_lift_margin_handles_residual_without_weakening_rotary(self):
        for driver in (1, 2):
            clock = FakeClock()
            with self.subTest(driver=driver), patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
                session, _ = self.rig(residual=True, driver_id=driver)
                if driver == 1:
                    result = run_cycle(session, Mock(), driver, lift_bottom_zero=True)
                    self.assertGreater(result['returned_motor_deg'], 0)
                    self.assertLess(result['returned_motor_deg'], 9)
                else:
                    with self.assertRaisesRegex(RuntimeError, 'overshot'):
                        run_cycle(session, Mock(), driver)

    def test_lift_requires_explicit_bottom_before_any_io(self):
        session = Mock()
        with self.assertRaisesRegex(RuntimeError, 'bottom-zero'):
            run_cycle(session, Mock(), 1)
        session.request.assert_not_called()
        session.request_line.assert_not_called()

    def test_quantized_return_never_commands_below_zero(self):
        self.assertEqual(lift_steps(8.9, 0), -79)
        self.assertAlmostEqual(quantized_target(8.9, -79), .1)
        for tenth in range(91):
            pos = tenth/10
            for target in (0., 9.):
                steps = lift_steps(pos, target)
                actual_target = quantized_target(pos, steps)
                self.assertGreaterEqual(actual_target, -1e-9)
                self.assertLessEqual(actual_target, 9+1e-9)
                self.assertLessEqual(abs(steps), 80)

    def test_soft_limit_or_nonfinite_input_is_rejected(self):
        for pos, target in ((-.1,0),(9.1,0),(0,-.1),(0,9.1),(float('nan'),0),(0,float('inf'))):
            with self.subTest(pos=pos,target=target), self.assertRaises(ValueError):
                lift_steps(pos,target)

    def test_firmware_start_blind_window_is_not_an_immediate_telemetry_fault(self):
        for permanent in (False, True):
            clock = FakeClock()
            with self.subTest(permanent=permanent), patch('time.monotonic',clock.monotonic), patch('time.sleep',clock.sleep):
                session, commands = self.rig(driver_id=2)
                original = session.request
                moved_at = [None]
                def request(command, **kwargs):
                    reply = original(command, **kwargs)
                    if ',MOVE,' in command:
                        moved_at[0] = clock.now
                    if (command == 'X42S,S,2' and moved_at[0] is not None
                            and (permanent or clock.now-moved_at[0] < .4)):
                        # Firmware deliberately invalidates numeric cache after MOVE.
                        return parse_line('X42S,2,S,1,1,0,none,9,0,0,0,0,28,305,0,1')
                    return reply
                session.request = request
                if permanent:
                    with self.assertRaisesRegex(RuntimeError, 'missing after start'):
                        run_cycle(session, Mock(), 2)
                    self.assertEqual(sum(',MOVE,' in c for c in commands), 1)
                else:
                    self.assertEqual(run_cycle(session, Mock(), 2)['returned_motor_deg'], 0)
