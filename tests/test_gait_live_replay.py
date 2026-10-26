"""Offline replay of 2026-10-23 real frames; never opens serial or moves axes.

Capture times are normalized for deterministic unit tests. These fixtures are
not fresh live feedback and never authorize motion or assert world calibration.
"""
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from motor_control import BindingSet, LOGICAL_ROLE_ORDER
from motor_control.actuator_model import X42sActuatorState
from motor_control.gait_planner import default_gait_params
from motor_control.gait_twin import build_twin_snapshot
from motor_control.protocol import parse_line
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation
from scripts.x42s_suspended_gait import SuspendedHost, run_gait, build_plan
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock
from test_x42s_pc import _snapshot_app, _x42s_snapshot


FRAMES = (
    'X42S,1,S,1,0,1,none,155.600,156.400,0.000,0.000,0.000,31.000,43,0,63',
    'X42S,2,S,1,0,1,none,4.700,4.700,0.000,0.000,0.000,30.000,85,0,63',
    'X42S,3,S,1,0,1,none,71.200,72.100,0.000,0.000,0.000,31.000,19,1,63',
    'X42S,4,S,1,0,1,none,0.200,0.200,0.000,0.000,0.000,30.000,60,0,63',
)


class RecordedFeedbackTests(unittest.TestCase):
    def test_real_frame_units_and_twin_never_substitute_command_for_position(self):
        bindings = BindingSet.from_driver_id_mapping({'Mup1': 1, 'Mr1': 2, 'Mup2': 3, 'Mr2': 4},
            gear_ratios={'Mup1': 1, 'Mr1': 21.5, 'Mup2': 1, 'Mr2': 21.5},
            lead_mms={'Mup1': 1, 'Mup2': 1})
        states = {}
        for role, raw in zip(LOGICAL_ROLE_ORDER, FRAMES):
            r = parse_line(raw)
            states[role.value] = X42sActuatorState(role, r.driver_id).with_telemetry(
                now=1-r.age_ms/1000, enabled=r.enabled, reached=r.reached,
                measured_position=r.measured_position, command_position=r.command_position,
                position_error=r.position_error, speed=r.speed, current_a=r.current_a,
                temperature_c=r.temperature_c, fault=r.fault)
        _, axes = _x42s_snapshot(_snapshot_app(), bindings, states, now=1)
        for name, expected in (('Mup1', 156.4/360), ('Mup2', 72.1/360),
                               ('Mr1', 4.7/21.5), ('Mr2', .2/21.5)):
            self.assertAlmostEqual(axes[name]['position'], expected)
        self.assertNotEqual(axes['Mup1']['target_position'], axes['Mup1']['position'])
        snapshot = build_twin_snapshot(default_gait_params(), list(axes.values()), calibration_valid=False)
        self.assertEqual(snapshot['source'], 'x42s_measured')
        self.assertTrue(snapshot['axes_measured'])
        self.assertFalse(snapshot['pose_measured'])
        self.assertIsNone(snapshot['pose'])
        # Disabled axes do not become an eligible calibrated walking stance.
        self.assertIn(snapshot['state'], ('uncalibrated', 'untrusted'))

    def test_actual_partial_lift_start_cannot_be_relabelled_bottom(self):
        with self.assertRaisesRegex(ValueError, 'reviewed bottom'):
            SuspendedHost(Mock(), Mock(), {1: 156.4, 2: 4.7, 3: 72.1, 4: .2})

    def test_recorded_one_lsb_hold_speed_reproduces_bench_abort_without_rotary_move(self):
        clock = FakeClock()
        rig = GaitRig(clock)
        request = rig.request
        def read(command, **kwargs):
            reply = request(command, **kwargs)
            if rig.moves and command == 'X42S,S,2':
                return parse_line('X42S,2,S,1,1,1,none,4.700,4.700,-0.020,0.100,0.000,30.000,10,0,63')
            return reply
        rig.request = read
        p, report, stages = build_plan(default_gait_params(), 'L+')
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                patch('builtins.print'), self.assertRaisesRegex(RuntimeError, 'held ID2'):
            run_gait(rig, Mock(), None, p, report, stages, dict(rig.positions))
        self.assertEqual(len(rig.moves), 2)
        self.assertTrue(all(c.startswith(('X42S,1,MOVE,', 'X42S,3,MOVE,')) for c in rig.moves))

    def test_one_lsb_is_not_zero_and_position_only_frame_is_not_arrival(self):
        moving = parse_line('X42S,2,S,1,1,1,none,4.700,4.700,-0.020,0.100,0.000,30.000,10,0,63')
        self.assertEqual(moving.speed, .1)
        expected = MoveExpectation(object(), 4.7, 0)
        evidence = ArrivalEvidence()
        self.assertFalse(evidence.observe(expected, moving, 1))
        idle = replace(moving, speed=0.)
        self.assertFalse(evidence.observe(expected, idle, 1.2))
        self.assertTrue(evidence.observe(expected, idle, 1.4))
        partial = parse_line('X42S,1,S,1,1,0,none,0.100,-0.900,0.080,0.000,0.012,31.000,36,0,2')
        self.assertIsNone(partial.speed)
        self.assertIsNone(partial.current_a)
        self.assertEqual(partial.measured_position, -.9)


if __name__ == '__main__':
    unittest.main()
