import math
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch

from motor_control.gait_planner import default_gait_params, plan_gait_stages
from motor_control.protocol import parse_line
from scripts.x42s_micro_jog import quantized_target
from scripts.x42s_suspended_gait import build_plan, checked_steps, SuspendedHost, run_gait, main
from test_gait_realtime_fixes import FakeClock


class GaitRig:
    def __init__(self, clock, failure=None, *, failure_driver=3):
        self.clock, self.failure = clock, failure
        self.failure_driver = failure_driver
        self.positions = {1: -.9, 2: 4.7, 3: 0., 4: .2}
        self.enabled = {i: False for i in self.positions}
        self.commands, self.moves, self.moving = [], [], {}

    def request_line(self, command, **kwargs):
        self.commands.append(command)
        if command == 'ESTOP':
            self.enabled = {i: False for i in self.positions}
            self.moving.clear()
        else:
            i = int(command.split(',')[1])
            self.enabled[i] = True
            if self.failure == 'enable_drift':
                self.positions[i] += 1
        return 'OK,'+command

    def request(self, command, **kwargs):
        self.commands.append(command)
        f = command.split(',')
        if f[1] != 'S':
            self.moves.append(command)
            i, steps, pps = int(f[1]), int(f[3]), int(f[4])
            self.moving[i] = (self.clock.now, self.positions[i], quantized_target(self.positions[i], steps), pps)
            if self.failure == 'wrong_ack' and i == self.failure_driver:
                pps += 1
            return parse_line(f'ACK,X42S,{i},MOVE,{steps},{pps}')
        i = int(f[2])
        target, reached, speed, mask, age, fault = self.positions[i], 1, 0, 63, 1, 'none'
        if i in self.moving:
            at, origin, target, pps = self.moving[i]
            dt = self.clock.now-at
            direction = 1 if target >= origin else -1
            self.positions[i] = origin+direction*min(abs(target-origin), max(0, dt-.1)*pps*360/3200)
            reached = int(abs(self.positions[i]-target) < 1e-8)
            speed = 0 if reached else direction*pps*60/3200
            if dt < .4:
                mask = 1
            if i == self.failure_driver:
                if self.failure == 'stall':
                    self.positions[i], reached, speed = origin, 0, 0
                elif self.failure == 'fault':
                    fault = 'stall'
                elif self.failure == 'missing':
                    mask = 1
                elif self.failure == 'stale':
                    age = 501
                elif self.failure == 'overshoot':
                    self.positions[i], mask = target+direction*1, 2
                elif self.failure == 'no_speed':
                    mask = 7
        if self.moves and self.failure == 'held_drift' and i == 2:
            self.positions[i] += 1
        return parse_line(f'X42S,{i},S,1,{int(self.enabled[i])},{reached},{fault},'
            f'{target},{self.positions[i]},0,{speed},0,30,{age},0,{mask}')


class SuspendedGaitTests(unittest.TestCase):
    def plan(self, action):
        return build_plan(default_gait_params(), action)

    def test_all_four_paths_keep_stage_order_and_exact_endpoints(self):
        for action in ('L+', 'L-', 'R+', 'R-'):
            p, report, stages = self.plan(action)
            base = plan_gait_stages(p, side=report.side, arc_deg=60 if action.endswith('-') else -60,
                                    route=report.route)
            self.assertEqual([s.stage_id for s in stages], [s.stage_id for s in base])
            for s, original in zip(stages, base):
                for role in ('Mup1', 'Mup2', 'Mr1', 'Mr2'):
                    total = lambda stage: sum(m.delta for g in stage.move_groups for m in g if m.role == role)
                    self.assertAlmostEqual(total(s), total(original))
                if s.stage_id == 'S4':
                    self.assertEqual(s.sync_endpoints[-1], original.sync_endpoints[-1])
                    self.assertGreater(s.certified_margin_mm, 0)
                for g in s.move_groups:
                    for m in g:
                        self.assertLessEqual(abs(m.delta), (2. if m.role.startswith('Mup') else 1.)+1e-8)
                        self.assertLessEqual(m.speed, (1.25 if m.role.startswith('Mup') else 1.)+1e-8)

    def test_complete_fake_feedback_four_actions_never_writes_zero_or_config(self):
        for action in ('L+', 'L-', 'R+', 'R-'):
            p, report, stages = self.plan(action)
            clock = FakeClock()
            rig = GaitRig(clock)
            reviewed = dict(rig.positions)
            with self.subTest(action=action), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'):
                result = run_gait(rig, Mock(), None, p, report, stages, reviewed)
            self.assertEqual(result['groups_completed'], sum(len(s.move_groups) for s in stages))
            self.assertTrue(result['suspended_only'])
            for i in (1, 3):
                self.assertLessEqual(abs(result['positions_motor_deg'][i]), .72)
            self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
            self.assertTrue(all(int(c.split(',')[4]) <= 4000 for c in rig.moves))
            # No full telemetry round inserted between paired lift commands.
            first = next(n for n, c in enumerate(rig.commands) if ',MOVE,' in c)
            self.assertIn('X42S,3,MOVE,', rig.commands[first+1])

    def test_unhealthy_group_aborts_without_later_segments(self):
        p, report, stages = self.plan('L+')
        for failure in ('wrong_ack', 'fault', 'missing', 'stale', 'overshoot', 'stall', 'no_speed', 'held_drift'):
            clock = FakeClock()
            rig = GaitRig(clock, failure)
            with self.subTest(failure=failure), patch('time.monotonic', clock.monotonic), \
                    patch('time.sleep', clock.sleep), patch('builtins.print'), self.assertRaises(RuntimeError):
                run_gait(rig, Mock(), None, p, report, stages, dict(rig.positions))
            self.assertLessEqual(len(rig.moves), 2)

    def test_enable_drift_and_wrong_start_never_move(self):
        clock = FakeClock()
        for failure in ('enable_drift', None):
            rig = GaitRig(clock, failure)
            reviewed = dict(rig.positions)
            if not failure:
                rig.positions[2] += 1
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                    self.assertRaises(RuntimeError):
                SuspendedHost(rig, Mock(), reviewed).prepare()
            self.assertFalse(rig.moves)

    def test_quantization_and_bounds(self):
        for i, origin, target in ((1, -.9, 720), (1, 36.1, 0), (2, 4.7, 26.2)):
            steps = checked_steps(i, origin, target, 4.7)
            self.assertLessEqual(abs(quantized_target(origin, steps)-target), .225)
            self.assertGreaterEqual(quantized_target(origin, steps), min(origin, target)-1e-9)
            self.assertLessEqual(quantized_target(origin, steps), max(origin, target)+1e-9)
        for values in ((1, 36, -1, 0), (1, 0, 1000, 0), (3, 25000, 25300, 0),
                       (2, 0, 22, 0), (4, 3870, 3871, 0), (1, 0, math.nan, 0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                checked_steps(*values)

    def test_stop_request_prevents_enable_and_move(self):
        session = Mock()
        host = SuspendedHost(session, Mock(), {1: 0, 2: 0, 3: 0, 4: 0}, stop_file=Path(__file__))
        with self.assertRaises(RuntimeError):
            host.prepare()
        session.request.assert_not_called()
        session.request_line.assert_not_called()

    def test_offline_default_and_missing_confirmation_do_not_open_port(self):
        for args, error in (([], False), (['--execute', '--port', 'COM4'], True)):
            with patch('sys.argv', ['tool']+args), patch('scripts.x42s_suspended_gait.execute_test') as execute, \
                    patch('builtins.print'), patch('sys.stderr'):
                if error:
                    with self.assertRaises(SystemExit):
                        main()
                else:
                    self.assertEqual(main(), 0)
                execute.assert_not_called()

    def test_unverified_profile_rejected(self):
        for p in (replace(default_gait_params(), lift_mm=71), replace(default_gait_params(), mup1_lift_sign=-1)):
            with self.assertRaises(ValueError):
                build_plan(p, 'L+')


if __name__ == '__main__':
    unittest.main()
