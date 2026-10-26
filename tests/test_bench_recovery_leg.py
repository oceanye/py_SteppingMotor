import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch

from scripts.x42s_recovery_leg import RecoveryLeg, run_leg, run_legs, group_waypoints, steps_for, main
from scripts.x42s_micro_jog import quantized_target
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class RecoveryLegTests(unittest.TestCase):
    def test_four_roles_return_in_raw_coordinates_without_zero(self):
        for driver, target in ((1, .05), (3, 5), (2, 0), (4, 0)):
            clock = FakeClock()
            rig = GaitRig(clock)
            rig.positions = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
            initial = dict(rig.positions)
            leg = RecoveryLeg(driver, initial, target)
            with self.subTest(driver=driver), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_leg(rig, Mock(), driver, leg, confirmed=True)
            self.assertAlmostEqual(result['inferred_historical_position'][driver], target, delta=.011)
            for i in set(initial) - {driver}:
                self.assertEqual(rig.positions[i], initial[i])
            self.assertFalse(result['home_completed'])
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
            self.assertTrue(all(c.startswith(f'X42S,{driver},MOVE,') for c in rig.moves))

    def test_rejects_outward_bottom_or_unreviewed_geometry(self):
        raw = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
        for driver, target in ((1, 0), (3, 61), (2, -20), (4, 1), (1, float('nan'))):
            with self.assertRaises(ValueError): RecoveryLeg(driver, raw, target).validate()
        with self.assertRaises(ValueError): RecoveryLeg(1, {**raw, 2: -500}, 1).validate()

    def test_missing_speed_fault_stall_or_passive_drift_stops_first_segment(self):
        for failure in ('wrong_ack', 'fault', 'stall', 'missing', 'stale', 'overshoot', 'no_speed', 'held_drift'):
            clock = FakeClock()
            rig = GaitRig(clock, failure)
            rig.positions = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
            leg = RecoveryLeg(3, dict(rig.positions), 5)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_leg(rig, Mock(), 3, leg, confirmed=True)
            self.assertEqual(len(rig.moves), 1)

    def test_reboot_drift_stop_and_unconfirmed_never_move(self):
        for reason in ('reboot', 'stop', 'unconfirmed'):
            clock = FakeClock(); rig = GaitRig(clock)
            rig.positions = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
            leg = RecoveryLeg(3, dict(rig.positions), 5)
            kwargs = {'confirmed': True}
            if reason == 'reboot': rig.positions[3] = 0
            if reason == 'stop': kwargs['stop_file'] = Path(__file__)
            if reason == 'unconfirmed': kwargs['confirmed'] = False
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), self.assertRaises((RuntimeError, ValueError)):
                run_leg(rig, Mock(), 3, leg, **kwargs)
            self.assertFalse(rig.moves)

    def test_near_bottom_slow_and_offline_no_connection(self):
        raw = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
        points = RecoveryLeg(1, raw, .05).waypoints()
        self.assertIn((.5, 800), points)
        self.assertEqual(points[-1], (.05, 160))
        args = ['return', '--driver', '3', '--reviewed-raw-motor-deg', '0', '0', '-1799.5', '0', '--target', '5']
        with patch('sys.argv', args), patch('scripts.x42s_recovery_leg.validate_evidence'), \
                patch('scripts.x42s_recovery_leg.execute_test') as execute, patch('builtins.print'):
            self.assertEqual(main(), 0)
        execute.assert_not_called()

    def test_unequal_lifts_and_rotary_pairs_finish_without_changing_raw_reference(self):
        for drivers, target in (((1, 3), .05), ((2, 4), 0)):
            clock = FakeClock(); rig = GaitRig(clock)
            rig.positions = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
            legs = tuple(RecoveryLeg(i, dict(rig.positions), target) for i in drivers)
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_legs(rig, Mock(), legs, confirmed=True)
            for i in drivers:
                self.assertAlmostEqual(result['inferred_historical_position'][i], target, delta=.011)
            self.assertEqual(len(rig.moves), 2 * len(group_waypoints(legs)))
            first = next(n for n, cmd in enumerate(rig.commands) if ',MOVE,' in cmd)
            self.assertIn(',MOVE,', rig.commands[first + 1])

    def test_partial_pair_ack_and_passive_micro_velocity_abort_without_retry(self):
        clock = FakeClock(); rig = GaitRig(clock, 'wrong_ack')
        rig.positions = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
        legs = tuple(RecoveryLeg(i, dict(rig.positions), .05) for i in (1, 3))
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), self.assertRaises(RuntimeError):
            run_legs(rig, Mock(), legs, confirmed=True)
        self.assertEqual(len(rig.moves), 2)
        class PassiveMicroRig(GaitRig):
            def request(self, command, **kwargs):
                reply = super().request(command, **kwargs)
                if command == 'X42S,S,1' and self.moves:
                    reply = replace(reply, speed=.1)
                return reply
        clock = FakeClock(); rig = PassiveMicroRig(clock)
        rig.positions = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
        legs = tuple(RecoveryLeg(i, dict(rig.positions), 0) for i in (2, 4))
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), self.assertRaises(RuntimeError):
            run_legs(rig, Mock(), legs, confirmed=True)
        self.assertEqual(len(rig.moves), 2)

    def test_four_axis_mixed_group_rejected(self):
        raw = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
        for ids in ((1, 2), (1, 2, 3, 4)):
            legs = tuple(RecoveryLeg(i, raw, .05 if i in (1, 3) else 0) for i in ids)
            with self.assertRaises(ValueError): group_waypoints(legs)

    def test_inward_quantization_inverts_firmware_without_extra_truncation(self):
        raw = {1: 0., 2: 0., 3: -1799.5, 4: 0.}
        legs = tuple(RecoveryLeg(i, raw, 0) for i in (2, 4))
        target, _ = group_waypoints(legs)[0][4]
        steps, nominal = steps_for(legs[1], 0, target)
        self.assertEqual(steps, 95)  # old double-floor produced 94
        self.assertAlmostEqual(quantized_target(0, steps), 10.6)
        self.assertLessEqual(quantized_target(0, steps), nominal)
        self.assertGreater(quantized_target(0, steps + 1), nominal)
        # Replay the observed -0.1 degree start-origin difference: endpoint
        # remains within ORIGINAL .225 degree tube; no widening is necessary.
        self.assertLess(abs(quantized_target(-.1, steps) - nominal), .225)

    def test_pair_uses_small_slow_segments_near_either_bottom(self):
        raw = {1: -.1, 2: 9.3, 3: -1799.5, 4: 15.1}
        legs = tuple(RecoveryLeg(i, raw, .05) for i in (1, 3))
        previous = {l.driver: l.validate() for l in legs}
        for group in group_waypoints(legs):
            for i, (target, pps) in group.items():
                delta = previous[i] - target
                limit = .1 if target <= .5 else .25 if target <= 2 else .5
                cap = 160 if target <= .5 else 800 if target <= 2 else 4000
                self.assertLessEqual(delta, limit + 1e-8)
                self.assertLessEqual(pps, cap)
                self.assertGreaterEqual(pps, 160)
                self.assertGreater(delta * 3200, 1)
                previous[i] = target
        for value in previous.values(): self.assertAlmostEqual(value, .05)

    def test_short_lift_does_not_drop_to_failed_13_pps_to_force_simultaneous_arrival(self):
        raw = {1: -.1, 2: 9.3, 3: -1799.5, 4: 15.1}
        legs = tuple(RecoveryLeg(i, raw, .05) for i in (1, 3))
        points = group_waypoints(legs)
        self.assertTrue(all(160 <= g[1][1] <= 4000 for g in points))
        self.assertTrue(any(g[1][1] == g[3][1] == 160 for g in points))


if __name__ == '__main__':
    unittest.main()
