import unittest
from unittest.mock import Mock, patch

from motor_control.protocol import parse_line
from scripts.x42s_hold_probe import observe_hold, observe_fixture_lift
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class HoldTests(unittest.TestCase):
    def test_single_lsb_recorded_without_claiming_standstill_or_moving(self):
        for failure in (None, 'speed', 'position'):
            clock, recorder = FakeClock(), Mock()
            rig = GaitRig(clock)
            request = rig.request
            def read(command, **kwargs):
                reply = request(command, **kwargs)
                if command == 'X42S,S,2' and rig.enabled[2]:
                    f = reply.raw.split(',')
                    f[10] = '0.2' if failure == 'speed' else '0.1'
                    if failure == 'position':
                        f[8] = '5.0'
                    reply = parse_line(','.join(f))
                return reply
            rig.request = read
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
                if failure:
                    with self.assertRaises(RuntimeError):
                        observe_hold(rig, recorder, 2, dict(rig.positions), seconds=1)
                else:
                    result = observe_hold(rig, recorder, 2, dict(rig.positions), seconds=1)
                    self.assertTrue(result['not_a_standstill_certificate'])
                    self.assertEqual(result['position_min_motor_deg'], result['position_max_motor_deg'])
                    self.assertGreater(result['speed_rpm_histogram']['0.1'], 0)
            self.assertFalse(rig.moves)

    def test_linear_axis_cannot_be_enabled_by_probe(self):
        session = Mock()
        with self.assertRaises(ValueError):
            observe_hold(session, Mock(), 1, {1: 0, 2: 0, 3: 0, 4: 0})
        session.request.assert_not_called()

    def test_isolated_lift_diagnostic_does_not_turn_speed_into_arrival_or_move(self):
        clock, rig = FakeClock(), GaitRig(FakeClock())
        rig.clock = clock
        rig.positions = {1: 1800., 2: -409.8, 3: 23399.699, 4: -596.9}
        request = rig.request
        def read(c, **kw):
            r = request(c, **kw)
            if c == 'X42S,S,1' and rig.enabled[1]:
                fields = r.raw.split(',')
                fields[10] = '.3'
                return parse_line(','.join(fields))
            return r
        rig.request = read
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            result = observe_fixture_lift(rig, Mock(), 1, dict(rig.positions), seconds=1)
        self.assertTrue(result['not_a_standstill_certificate'])
        self.assertGreater(result['speed_rpm_histogram']['0.3'], 0)
        self.assertFalse(rig.moves)
        self.assertFalse(any(rig.enabled[i] for i in (2, 3, 4)))

    def test_isolated_lift_speed_drift_missing_and_passive_guards(self):
        for failure in ('speed', 'position', 'missing', 'passive'):
            clock = FakeClock()
            rig = GaitRig(clock)
            rig.positions = {1: 1800., 2: -409.8, 3: 23399.699, 4: -596.9}
            request = rig.request
            def read(c, **kw):
                r = request(c, **kw)
                if rig.enabled[1] and c == ('X42S,S,2' if failure == 'passive' else 'X42S,S,1'):
                    fields = r.raw.split(',')
                    if failure in ('speed', 'passive'):
                        fields[10] = '.4'
                    elif failure == 'position':
                        fields[8] = '1801'
                    else:
                        fields[-1] = '7'
                    r = parse_line(','.join(fields))
                return r
            rig.request = read
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), self.assertRaises(RuntimeError):
                observe_fixture_lift(rig, Mock(), 1, dict(rig.positions), seconds=1)
            self.assertFalse(rig.moves)


if __name__ == '__main__':
    unittest.main()
