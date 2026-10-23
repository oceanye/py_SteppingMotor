import unittest
from unittest.mock import Mock, patch
from motor_control.protocol import parse_line
from scripts.x42s_micro_jog import run_cycle
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
            positions[driver] += steps*360/3200
            if residual:
                positions[driver] -= .1 if steps > 0 else .3
            return parse_line(f'ACK,X42S,{driver},MOVE,{steps},80')
        return Mock(request=request, request_line=lambda *a, **k:f'OK,X42S,{driver_id},EN,1'), commands

    def test_bounded_round_trip_uses_actual_feedback(self):
        clock = FakeClock()
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            session, commands = self.rig()
            result = run_cycle(session, Mock(), 1)
        self.assertEqual(result['returned_motor_deg'], 0)
        self.assertEqual([c for c in commands if ',MOVE,' in c], ['X42S,1,MOVE,80,80', 'X42S,1,MOVE,-80,80'])

    def test_fault_never_sends_return_or_retry(self):
        clock = FakeClock()
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            session, commands = self.rig(fail=True)
            with self.assertRaisesRegex(RuntimeError, 'unhealthy'):
                run_cycle(session, Mock(), 1)
        self.assertEqual([c for c in commands if ',MOVE,' in c], ['X42S,1,MOVE,80,80'])

    def test_recorded_linear_residual_passes_but_rotary_does_not(self):
        for driver in (1, 2):
            clock = FakeClock()
            with self.subTest(driver=driver), patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
                session, _ = self.rig(residual=True, driver_id=driver)
                if driver == 1:
                    self.assertAlmostEqual(run_cycle(session, Mock(), driver)['returned_motor_deg'], -.4)
                else:
                    with self.assertRaisesRegex(RuntimeError, 'overshot'):
                        run_cycle(session, Mock(), driver)
