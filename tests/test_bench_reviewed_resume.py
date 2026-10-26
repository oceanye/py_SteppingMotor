import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch

from motor_control.gait_planner import default_gait_params
from motor_control.protocol import parse_line
from scripts.x42s_reviewed_resume import (PARKED, ORIGINAL_ORIGINS, ReviewedResumeHost,
    validate_start, remaining_stages, run_resume, main)
from scripts.x42s_suspended_gait import build_plan
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class ReviewedResumeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = build_plan(default_gait_params(), 'L+', profile='interior')

    def rig(self):
        clock = FakeClock()
        rig = GaitRig(clock)
        rig.positions = dict(PARKED)
        rig.positions[3] = 23399.699
        return clock, rig

    def test_remaining_plan_keeps_exact_absolute_curve_and_speed(self):
        _, _, stages = self.plan
        original = next(s for s in stages if s.stage_id == 'S4')
        rest = remaining_stages(stages)
        self.assertEqual(rest[0].sync_endpoints, original.sync_endpoints[32:])
        self.assertEqual(rest[0].move_groups, original.move_groups[32:])
        self.assertEqual(sum(len(s.move_groups) for s in rest), 208)
        self.assertEqual([s.stage_id for s in rest], ['S4', 'S5', 'S5B', 'S6', 'S7'])

    def test_complete_fake_continuation_preserves_original_reference(self):
        clock, rig = self.rig()
        p, report, stages = self.plan
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
            result = run_resume(rig, Mock(), None, p, report, stages, dict(rig.positions))
        self.assertEqual(result['resumed_groups_completed'], 208)
        self.assertEqual(result['combined_verified_groups'], 310)
        self.assertFalse(result['continuous_full_step'])
        for i, end in ((1, 1800.), (3, 1800.), (2, 4.6+60*21.5), (4, .2-60*21.5)):
            self.assertAlmostEqual(rig.positions[i], end, delta=.72 if i in (1, 3) else .225)
        self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
        self.assertTrue(rig.moves[0].startswith('X42S,2,MOVE,'))
        self.assertTrue(rig.moves[1].startswith('X42S,4,MOVE,'))
        self.assertTrue(all(abs(int(c.split(',')[3])) <= 6407 for c in rig.moves))

    def test_changed_pose_wrong_plan_stop_and_enable_drift_rejected(self):
        for i in (1, 2, 3, 4):
            with self.assertRaises(ValueError):
                validate_start({**PARKED, i: PARKED[i]+1})
        with self.assertRaises(ValueError):
            validate_start({**PARKED, 2: float('nan')})
        _, _, low = build_plan(default_gait_params(), 'L-', profile='interior')
        with self.assertRaises(ValueError):
            remaining_stages(low)
        for failure in ('stop', 'enable_drift', 'live_drift'):
            clock, rig = self.rig()
            reviewed = dict(rig.positions)
            host = ReviewedResumeHost(rig, Mock(), reviewed,
                stop_file=Path(__file__) if failure == 'stop' else Path('nonexistent-unit-stop-file'))
            if failure == 'enable_drift':
                rig.failure = failure
            elif failure == 'live_drift':
                rig.positions[4] += 1
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                    self.assertRaises((ValueError, RuntimeError)):
                host.prepare()
            self.assertFalse(rig.moves)

    def test_original_held_speed_guard_still_aborts_without_any_move(self):
        clock, rig = self.rig()
        host = ReviewedResumeHost(rig, Mock(), dict(rig.positions))
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep):
            host.prepare()
            self.assertEqual(host.origins, ORIGINAL_ORIGINS)
            original_request = rig.request
            def read(c, **kw):
                if c == 'X42S,S,1':
                    return parse_line('X42S,1,S,1,1,1,none,1799.9,1799.9,0,-0.1,0,31,10,0,63')
                return original_request(c, **kw)
            rig.request = read
            with self.assertRaisesRegex(RuntimeError, 'held ID1'):
                host.verify_hold()
        self.assertFalse(rig.moves)

    def test_cli_is_offline_by_default_and_requires_new_authorization(self):
        argv = ['resume', '--reviewed-motor-deg', '1799.9', '-420.8', '23399.699', '-595.2']
        with patch('sys.argv', argv), patch('scripts.x42s_reviewed_resume.load_source', return_value=self.plan), \
                patch('scripts.x42s_reviewed_resume.execute_test') as execute, patch('builtins.print'):
            self.assertEqual(main(), 0)
        execute.assert_not_called()
        with patch('sys.argv', argv+['--execute', '--port', 'FAKE']), \
                patch('scripts.x42s_reviewed_resume.load_source', return_value=self.plan), \
                patch('scripts.x42s_reviewed_resume.execute_test') as execute, patch('builtins.print'), \
                patch('sys.stderr'), self.assertRaises(SystemExit):
            main()
        execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
