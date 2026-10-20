"""X42S migration boundary regressions; fake transport only, no hardware."""
import math
import time
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

from motor_control import LogicalRole
from motor_control import BindingSet
from motor_control import x42s_runtime
from motor_control.actuator_model import X42sActuatorState
from motor_control.gait_recovery import recovery_differences, recovery_reference
from motor_control.gait_recovery_ui import verify_pre_run_restore
from motor_control.gait_executor import GaitExecutorError
from motor_control.protocol import parse_line
from motor_control.gait_twin import axis_motion_active
from motor_control.web_adapter import DesktopWebController
from motor_control.x42s_units import motor_degrees_to_role, role_move_to_pulses
from test_x42s_pc import _x42s_gait_app
from test_web_adapter import FakeDesktopApplication


class UnitBoundaryTests(unittest.TestCase):
    def test_role_conversion_roundtrip_has_single_gear_and_lead_factor(self):
        for linear, lead, gear, delta, speed in (
                (True, 1, 1, -70, 1), (True, 8, 21.5, 5, 2),
                (False, 8, 21.5, -90, 8.4)):
            with self.subTest(linear=linear, lead=lead, gear=gear):
                steps, rate = role_move_to_pulses(
                    delta, speed, linear=linear, lead_mm=lead, gear_ratio=gear)
                units_per_pulse = (lead if linear else 360) / (3200 * gear)
                converted = motor_degrees_to_role(
                    steps * 360 / 3200, linear=linear, gear_ratio=gear, lead_mm=lead)
                self.assertAlmostEqual(converted, delta, delta=units_per_pulse / 2 + 1e-9)
                self.assertAlmostEqual(rate, speed / units_per_pulse, delta=0.5)

    def test_missing_zero_invalid_and_overflow_remain_distinct(self):
        kwargs = dict(linear=True, gear_ratio=1, lead_mm=8)
        self.assertIsNone(motor_degrees_to_role(None, **kwargs))
        self.assertEqual(motor_degrees_to_role(0, **kwargs), 0)
        for value in (True, float('nan'), float('inf'), '8'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                role_move_to_pulses(value, 1, **kwargs)
        for speed in (0, float('inf')):
            with self.assertRaises(ValueError):
                role_move_to_pulses(1, speed, **kwargs)
        with self.assertRaises(ValueError):
            role_move_to_pulses(1e308, 1, linear=True, lead_mm=.01, gear_ratio=1000)

    def test_nonfinite_telemetry_cannot_break_json_or_become_a_position(self):
        for name in ('measured_position', 'command_position', 'position_error',
                     'speed', 'current_a', 'temperature_c', 'last_update_monotonic'):
            for value in (math.nan, math.inf, True, '0'):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    X42sActuatorState(LogicalRole.MR1, 2, **{name: value})

    def test_internal_position_reader_rejects_old_faulted_or_rebound_samples(self):
        for fields in ({'last_update_monotonic': time.monotonic()-4},
                       {'fault': 'stall'}, {'driver_id': 9}, {'measured_position': None}):
            app = _x42s_gait_app()
            app._x42s_states['Mr1'] = replace(app._x42s_states['Mr1'], **fields)
            self.assertIsNone(app._x42s_role_output_units(LogicalRole.MR1))


class ManualBatchTests(unittest.TestCase):
    def make_app(self, responder=None):
        calls = []
        def send(command, *args):
            calls.append(command)
            return (True, 'OK') if responder is None else responder(command)
        app = SimpleNamespace(_send_manual_x42s=send, _post_ui=lambda callback: callback())
        return app, calls

    def test_bad_second_target_cannot_move_first_target(self):
        app, calls = self.make_app()
        with patch.object(x42s_runtime, 'messagebox') as dialog:
            x42s_runtime.X42sRuntimeMixin._x42s_manual_worker(
                app, 'plus', [('Mup1', 1, 1), ('Mup2', 3, 1000)], 90, 3600)
        self.assertEqual(calls, [])
        self.assertIn('整组未下发', dialog.showwarning.call_args.args[1])

    def test_partial_or_uncertain_ack_stops_all_attempted_and_skips_later_targets(self):
        app, calls = self.make_app(lambda command: (
            (False, 'timeout') if command.startswith('X42S,3,MOVE') else (True, 'OK')))
        with patch.object(x42s_runtime, 'messagebox'):
            x42s_runtime.X42sRuntimeMixin._x42s_manual_worker(
                app, 'plus', [('Mup1', 1, 1), ('Mup2', 3, 1), ('Mr1', 2, 1)], 90, 8)
        self.assertEqual(calls, ['X42S,1,MOVE,800,71','X42S,3,MOVE,800,71',
                                'X42S,1,STOP','X42S,3,STOP'])

    def test_wrong_subcommand_ack_is_not_stop_confirmation(self):
        app = _x42s_gait_app()
        app._send_and_read = lambda *_a, **_k: 'OK,X42S,2,EN,1'
        self.assertFalse(app._send_manual_x42s('X42S,2,STOP', 'Mr1', 2)[0])
        app._send_and_read = lambda *_a, **_k: 'OK,X42S,2,STOP'
        self.assertTrue(app._send_manual_x42s('X42S,2,STOP', 'Mr1', 2)[0])


class RecoveryTests(unittest.TestCase):
    def make_app(self):
        app = _x42s_gait_app()
        app._move_reservation = {axis: None for axis in range(len(app.axis_runtime))}
        app._gait_needs_recovery = True
        app._gait_recovery_snapshot = {
            'positions': {role.value: 0 for role in LogicalRole},
            'supports': ('A','B'), 'beta':180,
            'reference':recovery_reference(app)}
        return app

    def test_recovery_requires_fresh_stationary_enabled_fault_free_all_four(self):
        app = self.make_app()
        for fields in ({'reached': False}, {'online': False}, {'enabled': False},
                       {'speed': 2}, {'fault': 'stall'}, {'measured_position': None},
                       {'last_update_monotonic': time.monotonic()-4}):
            states = dict(app._x42s_states)
            states['Mr1'] = replace(states['Mr1'], **fields)
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                recovery_differences(app.control_bindings, states,
                                     app._gait_recovery_snapshot['positions'], now=time.monotonic())

    def test_recovery_never_sends_reverse_move_or_clears_lock_on_different_position(self):
        app = self.make_app()
        app._gait_recovery_snapshot['positions']['Mr1'] = 60
        with patch('motor_control.gait_recovery_ui.messagebox') as dialog:
            verify_pre_run_restore(app)
        self.assertTrue(app._gait_needs_recovery)
        self.assertEqual(app.commands, [])
        self.assertIn('未发送运动命令', dialog.showwarning.call_args.args[1])

    def test_correct_arrival_and_operator_confirmation_commit_without_move_or_zero(self):
        app = self.make_app()
        app._x42s_states['Mr1'] = replace(app._x42s_states['Mr1'], speed=None)
        with patch('motor_control.gait_recovery_ui.messagebox') as dialog:
            dialog.askyesno.return_value = True
            verify_pre_run_restore(app)
        self.assertFalse(app._gait_needs_recovery)
        self.assertIsNone(app._gait_recovery_snapshot)
        self.assertEqual(app.commands, [])
        dialog.askyesno.assert_called_once()

    def test_changed_reference_decline_or_dialog_race_preserves_lock(self):
        for scenario in ('reference', 'decline', 'race'):
            app = self.make_app()
            with patch('motor_control.gait_recovery_ui.messagebox') as dialog:
                if scenario == 'reference':
                    app._gait_world_epoch = 1
                elif scenario == 'decline':
                    dialog.askyesno.return_value = False
                else:
                    def move_during_dialog(*_a):
                        app._x42s_states['Mr1'] = replace(app._x42s_states['Mr1'], measured_position=21.5)
                        return True
                    dialog.askyesno.side_effect = move_during_dialog
                verify_pre_run_restore(app)
            self.assertTrue(app._gait_needs_recovery, scenario)
            self.assertEqual(app.commands, [])


class PendingManualTests(unittest.TestCase):
    def app_with_move(self):
        app = _x42s_gait_app()
        app._hardware_estop_active = False
        app._estop_in_progress = False
        app._estop_unconfirmed = False
        app.pico_node_health = {}
        app._move_reservation = {axis: None for axis in range(len(app.axis_runtime))}
        app.state_store = Mock()
        self.assertTrue(app._send_manual_x42s('X42S,2,MOVE,800,80','Mr1',2,800,80)[0])
        return app

    def test_ack_blocks_binding_baseline_and_gait_until_new_measured_arrival(self):
        app = self.app_with_move()
        candidate = BindingSet.from_driver_id_mapping({'Mup1':1,'Mr1':5,'Mup2':3,'Mr2':4})
        with self.assertRaises(RuntimeError):
            app._apply_control_bindings(candidate)
        with self.assertRaises(GaitExecutorError):
            app._gait_capture_baseline(app.gait_params)
        with self.assertRaises(GaitExecutorError):
            app._gait_begin_run('left')
        app.state_store.save_coordinated_bindings.assert_not_called()
        app.state_store.save_gait_params.assert_not_called()
        control, cards = app._snapshot_coordinated_control_locked()
        card = next(c for c in cards if c['role']=='Mr1')
        self.assertTrue(card['command_pending'])
        self.assertTrue(axis_motion_active(card))

    def test_old_reached_and_wrong_endpoint_cannot_clear_pending_but_arrival_can(self):
        app = self.app_with_move()
        reply = lambda angle: parse_line(f'X42S,2,S,1,1,1,none,90,{angle},0,0,0,25,0,0,7')
        app._apply_x42s_reply(LogicalRole.MR1,2,reply(90))
        self.assertIn(2,app._x42s_manual_pending)  # still in command-front blind window
        app._x42s_manual_pending[2]['sent_at'] -= 1
        app._apply_x42s_reply(LogicalRole.MR1,2,reply(0))
        self.assertIn(2,app._x42s_manual_pending)  # old endpoint despite reached=1
        app._apply_x42s_reply(LogicalRole.MR1,2,reply(90))
        self.assertFalse(app._x42s_manual_pending)

    def test_timeout_cannot_clear_pending_from_old_telemetry_but_exact_stop_can(self):
        app = self.app_with_move()
        app._x42s_manual_pending[2]['acknowledged'] = False
        app._x42s_manual_pending[2]['sent_at'] -= 1
        app._apply_x42s_reply(LogicalRole.MR1,2,parse_line('X42S,2,S,1,1,1,none,90,90,0,0,0,25,0,0,7'))
        self.assertIn(2,app._x42s_manual_pending)
        self.assertTrue(app._send_manual_x42s('X42S,2,STOP','Mr1',2)[0])
        self.assertFalse(app._x42s_manual_pending)

    def test_worker_or_actual_motion_blocks_binding_without_legacy_axis(self):
        for active_worker in (True,False):
            app = _x42s_gait_app()
            app.state_store = Mock()
            if active_worker:
                app._active_control_workers=1
            else:
                app._x42s_states['Mr1']=replace(app._x42s_states['Mr1'],reached=False)
            candidate = BindingSet.from_driver_id_mapping({'Mup1':1,'Mr1':5,'Mup2':3,'Mr2':4})
            with self.assertRaises(RuntimeError):
                app._apply_control_bindings(candidate)
            app.state_store.save_coordinated_bindings.assert_not_called()


class WebMigrationTests(unittest.TestCase):
    def test_x42s_default_has_no_legacy_topology_and_all_legacy_writes_reject(self):
        app = FakeDesktopApplication()
        app.legacy_stepdir_enabled = False
        controller = DesktopWebController(app)
        status = controller.web_get_status()
        self.assertEqual(status['topology']['total_axes'], 0)
        self.assertEqual(status['topology']['axes'], [])
        self.assertEqual(status['stepper_axes'], [])
        for call in (lambda:controller.web_stepper_stop(0),
                     lambda:controller.web_stepper_move(0,'forward',1),
                     lambda:controller.web_stepper_config(0,mode='linear')):
            with self.assertRaisesRegex(RuntimeError, 'STEP/DIR'):
                call()
        self.assertEqual(app.serial_commands, [])


if __name__ == '__main__':
    unittest.main()
