import unittest
from unittest.mock import Mock, patch

from motor_control.gait_planner import default_gait_params
from motor_control.protocol import parse_line
from motor_control.x42s_motion import MoveExpectation
from scripts.x42s_suspended_gait import build_plan, run_gait, SuspendedHost, main, lift_pair_observation
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class DoubleSpeedTests(unittest.TestCase):
    def test_four_plans_preserve_geometry_and_slow_bottom_approach(self):
        for action in ('L+', 'L-', 'R+', 'R-'):
            one = build_plan(default_gait_params(), action)
            two = build_plan(default_gait_params(), action, speed_scale=2)
            self.assertEqual(one[1].route, two[1].route)
            self.assertEqual(one[1].modality, two[1].modality)
            heights = {'Mup1': 0., 'Mup2': 0.}
            for a, b in zip(one[2], two[2]):
                self.assertEqual(a.stage_id, b.stage_id)
                self.assertEqual(a.sync_endpoints, b.sync_endpoints)
                self.assertEqual(len(a.move_groups), len(b.move_groups))
                for g1, g2 in zip(a.move_groups, b.move_groups):
                    for m1, m2 in zip(g1, g2):
                        self.assertEqual((m1.role, m1.delta), (m2.role, m2.delta))
                        slow = m1.role in heights and m1.delta < 0 and heights[m1.role] <= 2+1e-8
                        self.assertAlmostEqual(m2.speed, m1.speed*(1 if slow else 2))
                        if m1.role in heights:
                            heights[m1.role] = round(heights[m1.role]+m1.delta, 9)
            self.assertEqual(heights, {'Mup1': 0., 'Mup2': 0.})

    def test_all_four_complete_at_2x_with_nonzero_raw_reference(self):
        reference = {1: -1768., 2: 436.3, 3: -23246.5, 4: -1995.2}
        for action in ('L+', 'L-', 'R+', 'R-'):
            p, report, stages = build_plan(default_gait_params(), action, speed_scale=2)
            clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(reference)
            with self.subTest(action=action), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_gait(rig, Mock(), None, p, report, stages, reference,
                                  zero_reference=reference, speed_scale=2)
            self.assertEqual(result['groups_completed'], sum(len(s.move_groups) for s in stages))
            self.assertEqual(result['speed_scale'], 2)
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))

    def test_prepare_only_never_sends_move(self):
        clock = FakeClock(); rig = GaitRig(clock)
        p, report, stages = build_plan(default_gait_params(), 'L+', speed_scale=2)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            result = run_gait(rig, Mock(), None, p, report, stages, dict(rig.positions),
                              speed_scale=2, prepare_only=True)
        self.assertTrue(result['prepare_only'])
        self.assertFalse(rig.moves)

    def test_invalid_or_mismatched_scale_rejected(self):
        for speed in (0, 1.5, 3, float('nan'), True):
            with self.assertRaises(ValueError): build_plan(default_gait_params(), 'L+', speed_scale=speed)
        p, report, stages = build_plan(default_gait_params(), 'L+', speed_scale=2)
        with self.assertRaises(ValueError): run_gait(None, None, None, p, report, stages, {i: 0 for i in range(1, 5)})

    def test_2x_does_not_waive_guard_or_allow_fast_bottom_return(self):
        for failure in ('fault', 'stale', 'overshoot', 'stall', 'held_drift'):
            clock = FakeClock(); rig = GaitRig(clock, failure)
            p, report, stages = build_plan(default_gait_params(), 'L+', speed_scale=2)
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                    patch('builtins.print'), self.assertRaises(RuntimeError):
                run_gait(rig, Mock(), None, p, report, stages, dict(rig.positions), speed_scale=2)
            self.assertLessEqual(len(rig.moves), 2)
        clock = FakeClock(); rig = GaitRig(clock)
        host = SuspendedHost(rig, Mock(), dict(rig.positions), speed_scale=2)
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            host.prepare()
            rig.positions[1] = host.held[1] = 36.
            with self.assertRaises(RuntimeError): host.dispatch({1: 0.}, {1: 2.5})
        self.assertFalse(rig.moves)

    def test_live_2x_requires_explicit_confirmation(self):
        args = ['gait', '--execute', '--port', 'COM4', '--speed-scale', '2',
                '--reviewed-motor-deg', '0', '0', '0', '0',
                '--suspended-clearance-confirmed', '--recorded-bottom-confirmed']
        with patch('sys.argv', args), patch('scripts.x42s_suspended_gait.execute_test') as execute, \
                patch('builtins.print'), patch('sys.stderr'), self.assertRaises(SystemExit): main()
        execute.assert_not_called()

    def test_real_2x_asynchronous_spread_is_not_silently_waived(self):
        # Actual last samples before 20261026T061353-4e384604fa aborted.
        # Time origins normalized to diagnostic elapsed seconds, not live data.
        r1 = parse_line('X42S,1,S,1,1,0,none,5431.900,5231.500,0.020,0.000,0.000,29.000,296,949901,2')
        r3 = parse_line('X42S,3,S,1,1,0,none,-16046.500,-16049.100,-0.020,0.000,0.001,29.000,46,949907,2')
        samples = {1: (r1, 38.580292), 3: (r3, 38.680768)}
        reference = {1: -1768., 2: 436.3, 3: -23246.5, 4: -1995.2}
        observation = lift_pair_observation(samples, reference)
        self.assertAlmostEqual(observation['sampled_spread_mm'], .5497222222)
        self.assertAlmostEqual(observation['estimated_sample_skew_ms'], 350.476)
        self.assertFalse(observation['simultaneous_measurement'])
        self.assertIsNone(observation['speeds_motor_rpm'][1])
        recorder = Mock()
        host = SuspendedHost(Mock(), recorder, reference, zero_reference=reference, speed_scale=2)
        host.pending = {i: MoveExpectation(object(), r.measured_position, 0) for i, (r, _) in samples.items()}
        with patch.object(host, 'snapshot', return_value=samples), self.assertRaisesRegex(RuntimeError, 'paired lift'):
            host.wait_group((1, 3), 10)
        event = recorder.event.call_args
        self.assertEqual(event.args, ('motor', 'bench.gait_pair_spread_stop'))
        self.assertGreater(event.kwargs['sampled_spread_mm'], .5)
        host.session.request.assert_not_called()


if __name__ == '__main__':
    unittest.main()
