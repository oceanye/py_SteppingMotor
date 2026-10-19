"""Current X42S defaults and retired STEP/DIR isolation; no hardware I/O."""
import configparser
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from motor_control import BindingSet, BACKEND_STEPDIR, BACKEND_X42S_UART
from motor_control.actuator_model import X42sActuatorState
from motor_control.coordinated_control import LOGICAL_ROLE_ORDER
from motor_control.desktop_app import StepperGUI
from motor_control.gait_twin import axis_motion_active, build_twin_snapshot
from motor_control.web_adapter import DesktopWebController
from test_x42s_pc import _snapshot_app, _x42s_snapshot
from test_gait_twin import linked_app


class CurrentHardwareTests(unittest.TestCase):
    def test_default_binding_is_x42s_unbound_and_legacy_is_explicit(self):
        current = BindingSet.empty()
        self.assertEqual(current.backend, BACKEND_X42S_UART)
        self.assertFalse(current.is_complete)
        self.assertTrue(all(b is None for b in current.assignments))
        self.assertEqual(BindingSet.empty(backend=BACKEND_STEPDIR).backend, BACKEND_STEPDIR)

    def test_load_preserves_existing_bindings_and_never_saves_automatically(self):
        for bindings in (BindingSet.suggested(), BindingSet.suggested_x42s(), None):
            app = StepperGUI.__new__(StepperGUI)
            app.legacy_stepdir_enabled = False
            app._startup_warnings = []
            app.state_store = Mock()
            app.state_store.load_coordinated_bindings.return_value = (
                bindings.as_document() if bindings else None)
            app._load_control_bindings()
            self.assertEqual(app.control_bindings, bindings or BindingSet.empty())
            app.state_store.save_coordinated_bindings.assert_not_called()
            self.assertEqual(bool(app._startup_warnings),
                             bindings is not None and bindings.backend == BACKEND_STEPDIR)

    def test_retired_commands_block_before_serial_or_pulse_ledger_access(self):
        app = StepperGUI.__new__(StepperGUI)
        app.legacy_stepdir_enabled = False
        app.log = Mock()
        app.serial_session = Mock()
        for command in ('MOVE,0,100,1,1000', 'SYNC,S', 'ENA,2,0', 'STDIAG,0', 'ENC,0,S', 'NODE,1,S'):
            self.assertIn('legacy stepdir disabled', app._send_and_read(command))
        self.assertFalse(app._send_pulses(0, 100, 1, 1))
        app.serial_session.request_line.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, '退役'):
            app._apply_control_bindings(BindingSet.suggested())

    def test_web_legacy_move_and_config_rejected_before_queueing(self):
        app = SimpleNamespace(legacy_stepdir_enabled=False)
        adapter = DesktopWebController(app)
        with self.assertRaisesRegex(RuntimeError, '停用'):
            adapter.web_stepper_move(0, 'FWD', 1)
        with self.assertRaisesRegex(RuntimeError, '停用'):
            adapter.web_stepper_config(0, mode='linear')

    def snapshot(self, **overrides):
        bindings = BindingSet.suggested_x42s()
        states = {}
        for role in LOGICAL_ROLE_ORDER:
            fields = dict(enabled=True, reached=True, speed=0.0,
                          measured_position=90.0, command_position=90.0)
            fields.update(overrides)
            states[role.value] = X42sActuatorState(
                role, bindings.for_role(role).driver_id).with_telemetry(now=0.9, **fields)
        return _x42s_snapshot(_snapshot_app(), bindings, states, now=1.0)

    def test_idle_target_is_retained_but_does_not_claim_motion(self):
        control, motors = self.snapshot()
        self.assertEqual(control['position_source'], 'x42s_driver')
        self.assertTrue(control['measured'])
        self.assertNotIn('pico_node_health_incomplete', control['blockers'])
        for card in motors.values():
            self.assertIsNotNone(card['target_position'])
            self.assertFalse(card['in_progress'])
            self.assertFalse(axis_motion_active(card))
        self.assertEqual(motors['Mup1']['unit'], 'mm')
        self.assertEqual(motors['Mr1']['unit'], 'deg')

    def test_unknown_moving_or_stale_feedback_never_counts_as_idle(self):
        # 2026-10-19 实机定案：X42S 静止时驱动器常不置速度 valid 位
        # （四台遥测 speed 恒 None）——速度未知不构成"在运动"；静止
        # 证据=位置可信+新鲜+reached。速度报了数值时才判非零/非有限。
        for fields in ({'reached': False}, {'reached': None}, {'speed': 1.0},
                       {'speed': math.nan}):
            _, motors = self.snapshot(**fields)
            self.assertTrue(axis_motion_active(motors['Mr1']), fields)
        _, motors = self.snapshot(speed=None)
        self.assertFalse(axis_motion_active(motors['Mr1']))
        _, motors = self.snapshot()
        card = dict(motors['Mr1'], stale=True)
        self.assertTrue(axis_motion_active(card))
        self.assertFalse(axis_motion_active(dict(card, state='DISCONNECTED')))

    def test_missing_position_is_not_measured_and_pose_is_always_inferred(self):
        control, _ = self.snapshot(measured_position=None)
        self.assertFalse(control['measured'])
        _, motors = self.snapshot()
        snapshot = build_twin_snapshot(linked_app().gait_params, list(motors.values()))
        self.assertTrue(snapshot['measured'])
        self.assertFalse(snapshot['pose_measured'])

    def test_default_firmware_environment_is_x42s(self):
        config = configparser.ConfigParser(interpolation=None)
        config.read(Path(__file__).resolve().parents[1] / 'esp32_stepper/platformio.ini', encoding='utf-8')
        self.assertEqual(config['platformio']['default_envs'], 'esp32s3_gear_x42s')
        self.assertIn('-DX42S_UART_ENABLED=1', config['env:esp32s3_gear_x42s']['build_flags'])


class CurrentGuiTests(unittest.TestCase):
    def test_real_gui_defaults_hide_legacy_controls_without_touching_hardware(self):
        import tkinter as tk
        if not hasattr(tk, 'Tk'):
            self.skipTest('Tk unavailable')
        from motor_control import desktop_app
        root = tk.Tk()
        root.withdraw()
        errors = []
        root.report_callback_exception = lambda *args: errors.append(args)
        recorder = Mock()
        recorder.health.return_value = {'dropped': 0, 'write_errors': 0}
        with tempfile.TemporaryDirectory() as workspace, \
                patch.dict('os.environ', {'STEPPER_LEGACY_STEPDIR': '0'}), \
                patch.object(desktop_app, 'PROJECT_DIR', Path(workspace)), \
                patch.object(desktop_app, 'install_exception_hooks'), \
                patch.object(StepperGUI, 'refresh_ports'), \
                patch.object(StepperGUI, '_load_axis_config') as old_config, \
                patch.object(StepperGUI, '_load_calib') as old_calibration, \
                patch.object(desktop_app.serial, 'Serial') as serial:
            try:
                app = StepperGUI(root, diagnostics=recorder)
                root.update_idletasks()
                self.assertFalse(app.legacy_stepdir_enabled)
                self.assertEqual(app.control_bindings.backend, BACKEND_X42S_UART)
                self.assertEqual(app.binding_backend_var.get(), BACKEND_X42S_UART)
                self.assertEqual(str(app.notebook.tab(app.tab_index_step[0], 'state')), 'hidden')
                self.assertFalse(app.gait_widgets['twin']['start_busy'])
                app._jump_to_binding_axis('Mr2')
                self.assertEqual(app.x42s_manual_role_var.get(), 'Mr2')
                self.assertEqual(str(app.notebook.tab(app.tab_index_step[0], 'state')), 'hidden')
                old_config.assert_not_called()
                old_calibration.assert_not_called()
                status = app.web_get_status()
                self.assertFalse(status['legacy_stepdir_enabled'])
                serial.assert_not_called()
                self.assertEqual(errors, [])
            finally:
                for pending in root.tk.splitlist(root.tk.call('after', 'info')):
                    root.after_cancel(pending)
                root.destroy()


if __name__ == '__main__':
    unittest.main()
