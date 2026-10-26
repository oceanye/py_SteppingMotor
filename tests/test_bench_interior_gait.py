import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from motor_control.gait_planner import default_gait_params
from motor_control.protocol import parse_line
from motor_control.x42s_motion import MoveExpectation
from scripts.x42s_interior_approach import run_approach, validate, main as approach_main
from scripts.x42s_suspended_gait import build_plan, SuspendedHost, SettledArrivalEvidence, run_gait, main
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class InteriorGaitTests(unittest.TestCase):
    def rig(self, failure=None, interior=False):
        clock = FakeClock()
        rig = GaitRig(clock, failure)
        rig.positions = {1: 1800. if interior else -.3, 2: 4.6,
                         3: 1800. if interior else 72.1, 4: .2}
        return clock, rig

    def test_approach_both_reviewed_origins_only_moves_positive_to_five(self):
        for driver in (1, 3):
            clock, rig = self.rig()
            reviewed = dict(rig.positions)
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_approach(rig, Mock(), driver, reviewed, confirmed=True)
            self.assertAlmostEqual(result['measured_motor_deg'], 1800, delta=.72)
            self.assertEqual(len(rig.moves), 10)
            for c in rig.moves:
                fields = c.split(',')
                self.assertEqual(int(fields[1]), driver)
                self.assertTrue(0 < int(fields[3]) <= 1632)
                self.assertEqual(int(fields[4]), 800)
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
            for i in set(reviewed)-{driver}:
                self.assertEqual(rig.positions[i], reviewed[i])

    def test_approach_faults_no_retry_or_rotary_motion(self):
        for failure in ('wrong_ack', 'stall', 'fault', 'missing', 'stale',
                        'overshoot', 'no_speed', 'held_drift'):
            clock, rig = self.rig(failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_approach(rig, Mock(), 3, dict(rig.positions), confirmed=True)
            self.assertEqual(len(rig.moves), 1)

    def test_approach_requires_explicit_review_support_and_stop_clear(self):
        for reason in ('drift', 'stop', 'unconfirmed'):
            clock, rig = self.rig()
            reviewed = dict(rig.positions)
            kwargs = dict(confirmed=True)
            if reason == 'drift':
                rig.positions[2] += 1
            elif reason == 'stop':
                kwargs['stop_file'] = Path(__file__)
            else:
                kwargs['confirmed'] = False
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                    self.assertRaises((ValueError, RuntimeError)):
                run_approach(rig, Mock(), 1, reviewed, **kwargs)
            self.assertFalse(rig.moves)
            self.assertFalse(any(rig.enabled.values()))
        for v in (-1, 181, float('nan')):
            with self.assertRaises(ValueError):
                validate(1, {1: v, 2: 4.6, 3: 72.1, 4: .2})

    def test_interior_four_actions_preserve_entire_rotary_curve_and_stage_sequence(self):
        for action in ('L+', 'L-', 'R+', 'R-'):
            _, _, original = build_plan(default_gait_params(), action)
            p, report, stages = build_plan(default_gait_params(), action, profile='interior')
            self.assertEqual(p.lift_mm, 60)
            self.assertEqual([s.stage_id for s in stages], [s.stage_id for s in original])
            self.assertEqual(next(s.sync_endpoints for s in stages if s.stage_id == 'S4'),
                             next(s.sync_endpoints for s in original if s.stage_id == 'S4'))
            h = dict(Mup1=5., Mup2=5.)
            for s in stages:
                for g in s.move_groups:
                    for m in g:
                        if m.role in h:
                            h[m.role] += m.delta
                            self.assertTrue(5-1e-8 <= h[m.role] <= 65+1e-8)
            self.assertAlmostEqual(h['Mup1'], 5)
            self.assertAlmostEqual(h['Mup2'], 5)
            clock, rig = self.rig(interior=True)
            recorder = Mock()
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_gait(rig, recorder, None, p, report, stages, dict(rig.positions), profile='interior')
            self.assertEqual(result['groups_completed'], sum(len(s.move_groups) for s in stages))
            self.assertEqual(result['lift_range_mm'], [5., 65.])
            self.assertTrue(any(c.args[1] == 'bench.gait_hold_verified' for c in recorder.event.call_args_list))
            for i in (1, 3):
                self.assertAlmostEqual(rig.positions[i], 1800, delta=.72)

    def test_profile_mismatch_old_start_and_outside_targets_do_not_move(self):
        current = {1: -.3, 2: 4.6, 3: 72.1, 4: .2}
        with self.assertRaises(ValueError):
            SuspendedHost(Mock(), Mock(), current, profile='interior')
        p, report, stages = build_plan(default_gait_params(), 'L+')
        session = Mock()
        with self.assertRaises(ValueError):
            run_gait(session, Mock(), None, p, report, stages, current, profile='interior')
        session.request_line.assert_not_called()
        for target in (4.999*360, 65.001*360):
            clock, rig = self.rig(interior=True)
            host = SuspendedHost(rig, Mock(), dict(rig.positions), profile='interior')
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
                host.prepare()
                with self.assertRaises(RuntimeError):
                    host.dispatch({1: target}, {1: .25})
            self.assertFalse(rig.moves)

    def test_bad_active_sample_stops_before_querying_remaining_axes(self):
        clock, rig = self.rig(interior=True)
        host = SuspendedHost(rig, Mock(), dict(rig.positions), profile='interior')
        host.expected_enabled = {1, 2, 3, 4}
        rig.enabled = {i: True for i in rig.positions}
        host.pending[1] = MoveExpectation(object(), 1801., 0, origin_motor_deg=1800., tolerance_motor_deg=.72)
        rig.positions[1] = 1802.
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                self.assertRaisesRegex(RuntimeError, 'outside commanded travel'):
            host.snapshot()
        self.assertEqual(rig.commands, ['X42S,S,1'])

    def test_one_lsb_hold_speed_still_aborts_in_interior_profile(self):
        clock, rig = self.rig(interior=True)
        host = SuspendedHost(rig, Mock(), dict(rig.positions), profile='interior')
        request = rig.request
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            host.prepare()
            def read(c, **kwargs):
                reply = request(c, **kwargs)
                if c == 'X42S,S,2':
                    return parse_line('X42S,2,S,1,1,1,none,4.6,4.6,0,0.1,0,30,1,0,63')
                return reply
            rig.request = read
            with self.assertRaisesRegex(RuntimeError, 'held ID2'):
                host.verify_hold()
        self.assertFalse(rig.moves)

    def test_missing_interior_confirmation_and_offline_approach_never_open_serial(self):
        argv = ['gait', '--execute', '--port', 'FAKE', '--lift-profile', 'interior',
                '--reviewed-motor-deg', '1800', '4.6', '1800', '.2',
                '--suspended-clearance-confirmed', '--recorded-bottom-confirmed']
        with patch('sys.argv', argv), patch('scripts.x42s_suspended_gait.execute_test') as execute, \
                patch('builtins.print'), patch('sys.stderr'), self.assertRaises(SystemExit):
            main()
        execute.assert_not_called()
        with patch('sys.argv', ['approach', '--driver', '1', '--reviewed-motor-deg', '-.3', '4.6', '72.1', '.2']), \
                patch('scripts.x42s_interior_approach.execute_test') as execute, patch('builtins.print'):
            self.assertEqual(approach_main(), 0)
        execute.assert_not_called()

    def test_recorded_172413_hold_frame_stops_after_61_verified_groups(self):
        # Recorded ID1 frame after S2(30) + S2B(31), not a fresh live sample.
        # The preceding two zero-speed endpoints do not waive the next hold check.
        clock, rig = self.rig(interior=True)
        p, report, stages = build_plan(default_gait_params(), 'L+', profile='interior')
        completed = []
        def event(channel, name, **data):
            if name == 'bench.gait_arrived':
                completed.append(data)
        request = rig.request
        def read(c, **kwargs):
            reply = request(c, **kwargs)
            if c == 'X42S,S,1' and len(completed) == 61:
                return parse_line('X42S,1,S,1,1,1,none,2340.100,2340.100,-0.030,0.100,0.000,31.000,35,0,63')
            return reply
        rig.request = read
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                patch('builtins.print'), self.assertRaisesRegex(RuntimeError, 'held ID1'):
            run_gait(rig, Mock(event=event), None, p, report, stages, dict(rig.positions), profile='interior')
        self.assertEqual(len(completed), 61)
        self.assertEqual(len(rig.moves), 91)
        self.assertTrue(all(c.startswith(('X42S,1,MOVE,', 'X42S,3,MOVE,')) for c in rig.moves))

    def test_real_two_zero_then_nonzero_sequence_cannot_complete_new_settling_window(self):
        move = MoveExpectation(object(), 2340.1, 0, origin_motor_deg=2430.1, tolerance_motor_deg=.72)
        evidence = SettledArrivalEvidence()
        for now, pos, rpm in ((1., 2339.9, 0), (1.2, 2340., 0), (1.4, 2340.1, .1)):
            reply = parse_line(f'X42S,1,S,1,1,1,none,2340.1,{pos},0,{rpm},0,31,35,0,63')
            self.assertFalse(evidence.observe(move, reply, now))
        # Following rows are SYNTHETIC; field run stopped before observing them.
        stable = parse_line('X42S,1,S,1,1,1,none,2340.1,2340.1,0,0,0,31,35,0,63')
        for now in (1.6, 1.8, 2., 2.2, 2.4):
            self.assertFalse(evidence.observe(move, stable, now))
        self.assertTrue(evidence.observe(move, stable, 2.61))
        missing_speed = parse_line('X42S,1,S,1,1,1,none,2340.1,2340.1,0,0,0,31,35,0,7')
        self.assertFalse(evidence.observe(move, missing_speed, 2.8))
        self.assertFalse(evidence.observe(move, stable, 3.))

    def test_settling_does_not_bridge_missing_sample_gaps_or_widen_position_tolerance(self):
        move = MoveExpectation(object(), 2340.1, 0, origin_motor_deg=2430.1, tolerance_motor_deg=.72)
        stable = parse_line('X42S,1,S,1,1,1,none,2340.1,2340.1,0,0,0,31,35,0,63')
        evidence = SettledArrivalEvidence()
        for now in (1., 1.2, 2.4, 2.6, 2.8):
            self.assertFalse(evidence.observe(move, stable, now))
        bad = parse_line('X42S,1,S,1,1,1,none,2340.1,2341.0,0,0,0,31,35,0,63')
        self.assertFalse(evidence.observe(move, bad, 3.))
        self.assertTrue(move.outside_travel(2339.0))

    def test_group_wait_resets_window_on_transient_missing_position(self):
        clock, rig = self.rig(interior=True)
        clock.now = 0.
        host = SuspendedHost(rig, Mock(), dict(rig.positions), profile='interior')
        host.pending[1] = MoveExpectation(object(), 1800., 0, no_motion=True,
            origin_motor_deg=1800., tolerance_motor_deg=.72)
        host.evidence[1] = SettledArrivalEvidence()
        host.progress[1] = (0., 1800.)
        host.deadline = 5.
        count = 0
        def snapshot():
            nonlocal count
            count += 1
            clock.sleep(.2)
            mask = 1 if count == 3 else 63
            r = parse_line(f'X42S,1,S,1,1,1,none,1800,1800,0,0,0,31,1,0,{mask}')
            return {1: (r, clock.monotonic())}
        with patch('time.monotonic', clock.monotonic), patch.object(host, 'snapshot', snapshot):
            self.assertEqual(host.wait_group((1,), 5), 'DONE')
        self.assertGreaterEqual(clock.monotonic(), 1.79)

    def test_recorded_180216_held_lift_speed_stops_pending_rotary_group_immediately(self):
        # Real 2026-10-23 S4 group 33 abort frame; surrounding rig is synthetic.
        clock, rig = self.rig(interior=True)
        host = SuspendedHost(rig, Mock(), dict(rig.positions), profile='interior')
        host.expected_enabled = {1, 2, 3, 4}
        host.held = {1: 1800., 2: -425.1, 3: 23399.9, 4: -594.6}
        host.groups_completed = 102
        for driver, origin, target in ((2, -425.1, -404.3), (4, -594.6, -598.2)):
            host.pending[driver] = MoveExpectation(object(), target, clock.monotonic(),
                origin_motor_deg=origin, tolerance_motor_deg=.225)
        def read(command, **kwargs):
            rig.commands.append(command)
            return parse_line('X42S,1,S,1,1,1,none,1800.000,1800.000,0.070,-0.100,0.000,31.000,60,0,63')
        rig.request = read
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                self.assertRaisesRegex(RuntimeError, 'held ID1'):
            host.snapshot()
        self.assertEqual(rig.commands, ['X42S,S,1'])
        self.assertEqual(host.groups_completed, 102)
        self.assertEqual(set(host.pending), {2, 4})
        self.assertFalse(rig.moves)


if __name__ == '__main__':
    unittest.main()
