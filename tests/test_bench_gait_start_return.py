import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from motor_control.protocol import parse_line
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation
from scripts.x42s_gait_start_return import run_return, validate, main
from test_bench_lift_return import ReturnRig
from test_gait_realtime_fixes import FakeClock


class StartReturnTests(unittest.TestCase):
    def rig(self, failure=None):
        clock = FakeClock()
        rig = ReturnRig(clock, failure=failure)
        rig.positions = {1: 156.4, 2: 4.7, 3: 72.1, 4: .2}
        return clock, rig

    def test_both_unequal_start_positions_separately_return_without_zero_write(self):
        for driver in (1, 3):
            clock, rig = self.rig()
            reviewed = dict(rig.positions)
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_return(rig, Mock(), driver, reviewed, confirmed=True)
            self.assertTrue(0 < result['measured_motor_deg'] <= 1.8)
            for c in rig.moves:
                f = c.split(',')
                self.assertEqual(int(f[1]), driver)
                self.assertTrue(-327 <= int(f[3]) < 0)
                self.assertEqual(int(f[4]), 160)
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
            for i in set(reviewed)-{driver}:
                self.assertEqual(rig.positions[i], reviewed[i])

    def test_faults_abort_first_segment_no_retry(self):
        for failure in ('wrong_ack', 'stall', 'fault', 'missing', 'stale',
                        'wrong_direction', 'overshoot', 'no_speed', 'passive_drift'):
            clock, rig = self.rig(failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_return(rig, Mock(), 3, dict(rig.positions), confirmed=True)
            self.assertEqual(len(rig.moves), 1)

    def test_reviewed_drift_stop_and_confirmation_reject_before_enable(self):
        for kind in ('drift', 'stop', 'confirmation'):
            clock, rig = self.rig()
            reviewed = dict(rig.positions)
            kwargs = dict(confirmed=True)
            if kind == 'drift':
                rig.positions[2] += 1
            elif kind == 'stop':
                kwargs['stop_file'] = Path(__file__)
            else:
                kwargs['confirmed'] = False
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                    self.assertRaises((RuntimeError, ValueError)):
                run_return(rig, Mock(), 1, reviewed, **kwargs)
            self.assertFalse(rig.moves)
            self.assertFalse(any(rig.enabled.values()))

    def test_invalid_bounds_and_offline_default(self):
        for driver, value in ((2, 156), (1, -1), (1, 0), (1, 181), (1, float('nan'))):
            with self.assertRaises(ValueError):
                validate(driver, {1: value, 2: 4.7, 3: 72.1, 4: .2})
        with patch('sys.argv', ['tool', '--driver', '1', '--reviewed-motor-deg', '156.4', '4.7', '72.1', '.2']), \
                patch('scripts.x42s_gait_start_return.execute_test') as execute, patch('builtins.print'):
            self.assertEqual(main(), 0)
        execute.assert_not_called()

    def test_165150_actual_negative_endpoint_aborts_without_sixth_move(self):
        clock, rig = self.rig()
        reviewed, request = dict(rig.positions), rig.request
        def read(command, **kwargs):
            result = request(command, **kwargs)
            if command == 'X42S,S,1' and len(rig.moves) == 5:
                return parse_line('X42S,1,S,1,1,0,none,1.200,-0.300,1.560,0.000,0.198,31.000,59,0,63')
            return result
        rig.request = read
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                patch('builtins.print'), self.assertRaisesRegex(RuntimeError, 'positive return envelope'):
            run_return(rig, Mock(), 1, reviewed, confirmed=True)
        self.assertEqual(len(rig.moves), 5)
        self.assertTrue(all(c.startswith('X42S,1,MOVE,') for c in rig.moves))

    def test_actual_reached_frames_with_nonzero_speed_do_not_complete_endpoint(self):
        # Actual 16:51:49.364 .. 16:51:50.164 samples. Synthetic replay clock;
        # not fresh hardware telemetry. The reached flag flickers before settling.
        rows = (
            ('1.500,-0.310,-0.100,0.434', 1),
            ('1.300,-0.160,-0.100,0.460', 1),
            ('1.200,-0.030,-0.100,0.465', 1),
            ('1.000,0.170,-0.100,0.426', 1),
            ('-0.300,1.560,0.000,0.198', 0),
        )
        move = MoveExpectation(object(), 1.2, 0, origin_motor_deg=12.4, tolerance_motor_deg=.72)
        evidence = ArrivalEvidence()
        for index, (values, reached) in enumerate(rows):
            reply = parse_line(f'X42S,1,S,1,1,{reached},none,1.200,{values},31.000,60,0,63')
            self.assertFalse(evidence.observe(move, reply, 1+index*.2))
        self.assertTrue(move.outside_travel(reply.measured_position))


if __name__ == '__main__':
    unittest.main()
