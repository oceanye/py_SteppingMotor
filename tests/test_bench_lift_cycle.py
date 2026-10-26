import unittest
from unittest.mock import Mock, patch

from motor_control.protocol import parse_line
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation
from scripts.x42s_lift_cycle import run_cycle, cycle_steps, run_stop_check
from scripts.x42s_micro_jog import quantized_target
from test_gait_realtime_fixes import FakeClock


class CycleRig:
    def __init__(self, clock, failure=None):
        self.clock, self.failure = clock, failure
        self.positions = {1: 1800., 2: 2.9, 3: 1800., 4: 0.}
        self.enabled = {i: False for i in self.positions}
        self.moves, self.moving = [], {}

    def request_line(self, command, **kwargs):
        if command == 'ESTOP':
            self.enabled = {i: False for i in self.positions}
            self.moving.clear()
            return 'OK,ESTOP'
        self.enabled[int(command.split(',')[1])] = True
        return 'OK,'+command

    def request(self, command, **kwargs):
        f = command.split(',')
        if f[1] != 'S':
            self.moves.append(command)
            i, steps, speed = int(f[1]), int(f[3]), int(f[4])
            self.moving[i] = (self.clock.now, self.positions[i], quantized_target(self.positions[i], steps))
            if self.failure == 'wrong_second_ack' and i == 3:
                i = 4
            return parse_line(f'ACK,X42S,{i},MOVE,{steps},{speed}')
        i = int(f[2])
        target = self.positions[i]
        reached, speed, mask, fault = 1, 0, 63, 'none'
        if i in self.moving:
            at, origin, target = self.moving[i]
            dt = self.clock.now-at
            direction = 1 if target > origin else -1
            self.positions[i] = origin+direction*min(abs(target-origin), max(0, dt-.1)*90)
            reached = int(self.positions[i] == target)
            speed = 0 if reached else direction*15
            if dt < .4:
                mask = 1
            if i == 3:
                if self.failure == 'stall':
                    self.positions[i], reached, speed = origin, 0, 0
                elif self.failure == 'fault':
                    fault = 'stall'
                elif self.failure == 'overshoot':
                    self.positions[i], mask = target+direction*1, 63
                elif self.failure == 'missing':
                    mask = 1
        if self.moves and i == 2 and self.failure == 'passive_drift':
            self.positions[i] = 4
        return parse_line(f'X42S,{i},S,1,{int(self.enabled[i])},{reached},{fault},'
                          f'{target},{self.positions[i]},0,{speed},0,30,1,0,{mask}')


class BenchLiftCycleTests(unittest.TestCase):
    def test_single_and_pair_repeat_forward_return_with_measured_endpoints(self):
        for drivers in ((1,), (3,), (1, 3)):
            clock, recorder = FakeClock(), Mock()
            rig = CycleRig(clock)
            with self.subTest(drivers=drivers), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_cycle(rig, recorder, drivers, confirmed=True)
            self.assertEqual(result['positions_motor_deg'], {i: 1800 for i in drivers})
            self.assertEqual(len(rig.moves), 4*len(drivers))
            for stage in range(4):
                for command in rig.moves[stage*len(drivers):(stage+1)*len(drivers)]:
                    self.assertEqual(int(command.split(',')[3]), 1600 if stage%2 == 0 else -1600)

    def test_second_member_fault_or_bad_feedback_aborts_without_return_or_retry(self):
        for failure in ('wrong_second_ack', 'stall', 'fault', 'overshoot', 'missing', 'passive_drift'):
            clock = FakeClock()
            rig = CycleRig(clock, failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'):
                with self.assertRaises(RuntimeError):
                    run_cycle(rig, Mock(), (1, 3), confirmed=True)
            self.assertEqual(len(rig.moves), 2)
            self.assertTrue(all(int(c.split(',')[3]) > 0 for c in rig.moves))

    def test_wrong_height_or_missing_confirmation_never_moves(self):
        clock = FakeClock()
        rig = CycleRig(clock)
        rig.positions[1] = 0
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, '5 mm start'):
                run_cycle(rig, Mock(), (1,), confirmed=True)
        self.assertEqual(rig.moves, [])
        self.assertFalse(any(rig.enabled.values()))
        for drivers in ((2,), (3, 1), (1, 1)):
            with self.assertRaises(ValueError):
                run_cycle(Mock(), Mock(), drivers, confirmed=True)

    def test_reverse_quantization_stays_inside_interior_and_bounds_segments(self):
        for origin, target in ((1800, 1980), (1980.3, 1800), (1799.7, 1980)):
            steps = cycle_steps(origin, target)
            actual = quantized_target(origin, steps)
            self.assertGreaterEqual(actual, min(origin, target)-1e-9)
            self.assertLessEqual(actual, max(origin, target)+1e-9)
            self.assertLessEqual(abs(steps), 1632)
        for origin, target in ((0, 1800), (1800, 0), (1800, 3600), (1790, 1980)):
            with self.subTest(origin=origin, target=target), self.assertRaises(ValueError):
                cycle_steps(origin, target)

    def test_dynamic_stop_verifies_feedback_and_never_returns(self):
        for confirmed in (True, False):
            clock = FakeClock()
            rig = CycleRig(clock)
            with self.subTest(confirmed=confirmed), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), \
                    patch('scripts.x42s_lift_cycle.confirm_disabled_standstill', return_value=confirmed):
                if confirmed:
                    result = run_stop_check(rig, Mock(), (1, 3), confirmed=True)
                    self.assertTrue(result['interrupted'])
                    self.assertTrue(all(1800 < x < 1962 for x in result['positions_motor_deg'].values()))
                else:
                    with self.assertRaisesRegex(RuntimeError, 'dynamic stop not confirmed'):
                        run_stop_check(rig, Mock(), (1, 3), confirmed=True)
            self.assertEqual(len(rig.moves), 2)
            self.assertTrue(all(int(c.split(',')[3]) > 0 for c in rig.moves))
            self.assertFalse(any(rig.enabled.values()))

    def test_no_measured_motion_is_not_a_successful_dynamic_stop(self):
        clock = FakeClock()
        rig = CycleRig(clock, 'stall')
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            with self.assertRaisesRegex(RuntimeError, 'no timely moving evidence'):
                run_stop_check(rig, Mock(), (3,), confirmed=True)
        self.assertEqual(len(rig.moves), 1)


class ArrivalEvidenceUnitTests(unittest.TestCase):
    def test_requires_two_distinct_stationary_samples_and_resets_after_invalid(self):
        move = MoveExpectation(object(), 1800, 99)
        r = parse_line('X42S,1,S,1,1,1,none,1800,1800,0,0,0,30,0,0,63')
        evidence = ArrivalEvidence()
        self.assertFalse(evidence.observe(move, r, 100))
        self.assertFalse(evidence.observe(move, r, 100))
        self.assertTrue(evidence.observe(move, r, 100.2))
        unknown = parse_line('X42S,1,S,1,1,1,none,1800,1800,0,0,0,30,0,0,7')
        self.assertFalse(evidence.observe(move, unknown, 100.3))
        self.assertFalse(evidence.observe(move, r, 100.4))
        self.assertTrue(evidence.observe(move, r, 100.6))


if __name__ == '__main__':
    unittest.main()
