"""Baseline workflow regressions: fake serial/state store only, never hardware."""
import sys
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_x42s_pc import _x42s_gait_app
from test_gait_linkage import mechanism
from motor_control.gait_executor import GaitExecutorError
from motor_control.gait_planner import default_gait_params
from motor_control.state_store import StateStoreError


def baseline_app():
    app = _x42s_gait_app(measured=(12., -107.8, 18., -495.3))
    app.gait_params = default_gait_params()
    app.state_store = Mock()
    app._move_reservation = {a: None for a in range(len(app.axis_runtime))}
    app._gait_needs_recovery = True
    app._gait_world_epoch = 4
    return app


class BaselineWorkflowTests(unittest.TestCase):
    def test_pair_is_saved_once_with_nonzero_measured_offsets_and_no_motor_writes(self):
        app = baseline_app()
        params = app._gait_capture_baseline(app.gait_params)
        self.assertAlmostEqual(params.mr1_zero_deg, -107.8 / 21.5)
        self.assertAlmostEqual(params.mr2_zero_deg, -495.3 / 21.5)
        self.assertTrue(params.calibration_confirmed)
        self.assertEqual(params.calibration_fingerprint, app._gait_hardware_fingerprint())
        for role in ('Mr1', 'Mr2'):
            self.assertEqual(getattr(params, role.lower() + '_zero_signature'),
                             app._gait_zero_signature(role, params))
        app.state_store.save_gait_params.assert_called_once_with(params.as_document())
        self.assertFalse(app._gait_needs_recovery)
        self.assertFalse(app._gait_baseline_recording)
        self.assertEqual(app._gait_world_epoch, 5)
        self.assertEqual(app.commands, [])

    def test_any_bad_role_preserves_whole_baseline_and_recovery(self):
        for role in ('Mup1', 'Mr1', 'Mup2', 'Mr2'):
            for fields in ({'online': False}, {'enabled': False}, {'fault': 'stall'},
                           {'measured_position': None}, {'reached': False, 'speed': 0.},
                           {'reached': True, 'speed': 2.}, {'speed': float('nan')},
                           {'measured_position': float('nan')}):
                with self.subTest(role=role, fields=fields):
                    app = baseline_app()
                    original = app.gait_params
                    if any(isinstance(v, float) and v != v for v in fields.values()):
                        # The data model itself rejects NaN. Also force corrupt
                        # data here to keep testing the baseline's second guard.
                        with self.assertRaises(ValueError):
                            replace(app._x42s_states[role], **fields)
                        for name, value in fields.items():
                            object.__setattr__(app._x42s_states[role], name, value)
                    else:
                        app._x42s_states[role] = replace(app._x42s_states[role], **fields)
                    app._x42s_preflight_state = lambda role, driver: app._x42s_states[role.value]
                    with self.assertRaises(GaitExecutorError):
                        app._gait_capture_baseline(original)
                    self.assertIs(app.gait_params, original)
                    self.assertTrue(app._gait_needs_recovery)
                    self.assertEqual(app._gait_world_epoch, 4)
                    app.state_store.save_gait_params.assert_not_called()
                    self.assertFalse(getattr(app, '_gait_baseline_recording', False))

    def test_expired_sample_is_not_accepted_when_status_refresh_fails(self):
        app = baseline_app()
        app._x42s_states['Mr2'] = replace(app._x42s_states['Mr2'],
                                         last_update_monotonic=time.monotonic()-10)
        app._send_and_read = Mock(return_value='')
        with self.assertRaises(GaitExecutorError):
            app._gait_capture_baseline(app.gait_params)
        self.assertEqual(app._send_and_read.call_args.args[0], 'X42S,S,4')
        app.state_store.save_gait_params.assert_not_called()

    def test_save_failure_keeps_old_stance_zeros_and_recovery(self):
        app = baseline_app()
        original = app.gait_params
        app._gait_supports = ('C', 'B')
        app.state_store.save_gait_params.side_effect = StateStoreError('disk full')
        with self.assertRaises(StateStoreError):
            app._gait_capture_baseline(original)
        self.assertIs(app.gait_params, original)
        self.assertEqual(app._gait_supports, ('C', 'B'))
        self.assertTrue(app._gait_needs_recovery)
        self.assertFalse(app._gait_baseline_recording)

    def test_context_change_during_read_cannot_commit(self):
        app = baseline_app()
        app._control_generation = 1
        original_read = app._x42s_preflight_state
        def read(role, driver):
            state = original_read(role, driver)
            if role.value == 'Mr2':
                app._control_generation += 1
            return state
        app._x42s_preflight_state = read
        with self.assertRaisesRegex(GaitExecutorError, '读取期间'):
            app._gait_capture_baseline(app.gait_params)
        app.state_store.save_gait_params.assert_not_called()

    def test_busy_estop_release_disconnect_and_owned_reject_without_writes(self):
        for field, value in (('_hardware_estop_active', True), ('_estop_unconfirmed', True),
                             ('_rot_release_active', True), ('_active_control_workers', 1),
                             ('_gait_owned', {102: object()}), ('_closing', True)):
            with self.subTest(field=field):
                app = baseline_app()
                setattr(app, field, value)
                with self.assertRaises(GaitExecutorError):
                    app._gait_capture_baseline(app.gait_params)
                app.state_store.save_gait_params.assert_not_called()
                self.assertEqual(app.commands, [])
        app = baseline_app()
        app._is_serial_connected = lambda: False
        with self.assertRaises(GaitExecutorError):
            app._gait_capture_baseline(app.gait_params)
        app.state_store.save_gait_params.assert_not_called()

    def test_pulse_backend_does_not_fake_untrusted_origin(self):
        app = mechanism()
        app.state_store = Mock()
        app.axis_runtime[2].position_trusted = False
        with self.assertRaisesRegex(GaitExecutorError, '位置不可信'):
            app._gait_capture_baseline(app.gait_params)
        app.state_store.save_gait_params.assert_not_called()
        self.assertEqual(app.commands, [])

    def test_ui_cancel_keeps_existing_baseline(self):
        app = baseline_app()
        app._gait_params_from_ui = lambda: app.gait_params
        app._gait_capture_baseline = Mock()
        with patch('motor_control.desktop_app.messagebox.askokcancel', return_value=False):
            app._gait_reestablish_baseline()
        app._gait_capture_baseline.assert_not_called()

    def test_prepared_190_baseline_passes_all_four_action_preflights_without_zero_or_move(self):
        for side in ('left', 'right'):
            for arc in (-60., 60.):
                with self.subTest(side=side, arc=arc):
                    app = baseline_app()
                    app._gait_capture_baseline(app.gait_params)
                    run, report = app._gait_begin_run(side, arc_deg=arc)
                    self.assertTrue(report.feasible)
                    self.assertEqual(run.state, 'ready')
                    self.assertFalse(any(',MOVE,' in c or ',ZERO' in c for c in app.commands))

    def test_x42s_start_rejects_motion_and_inflight_commands_without_rezero_request(self):
        for fields in ({'reached': False}, {'speed': 2.}):
            app = _x42s_gait_app()
            app._x42s_states['Mup1'] = replace(app._x42s_states['Mup1'], **fields)
            with self.assertRaisesRegex(GaitExecutorError, '尚未到位静止'):
                app._gait_begin_run('left')
        app = _x42s_gait_app()
        app._active_control_workers = 1
        with self.assertRaisesRegex(GaitExecutorError, '手动控制命令'):
            app._gait_begin_run('left')

    def test_ui_confirmation_records_both_once_without_extra_checkbox_or_save(self):
        app = baseline_app()
        app._gait_params_from_ui = lambda: app.gait_params
        with patch('motor_control.desktop_app.messagebox.askokcancel', return_value=True) as ask, \
                patch('motor_control.desktop_app.stop_preview_animation'), \
                patch('motor_control.desktop_app.load_gait_fields') as load:
            app._gait_reestablish_baseline()
        ask.assert_called_once()
        load.assert_called_once_with(app)
        self.assertTrue(app.gait_params.calibration_confirmed)
        app.state_store.save_gait_params.assert_called_once()
        self.assertEqual(app.commands, [])


class PlanCancellationTests(unittest.TestCase):
    def test_cancel_at_s0_s1_or_before_queuing_s2_preserves_baseline(self):
        for confirms in (0, 1, 2):
            with self.subTest(confirms=confirms):
                app = mechanism()
                run, _ = app._gait_begin_run('left')
                for _ in range(confirms):
                    self.assertTrue(run.advance_confirm())
                original = app.gait_params
                app.commands.clear()
                app._gait_abort_run()
                self.assertEqual(run.state, 'aborted')
                self.assertFalse(app._gait_needs_recovery)
                self.assertIs(app.gait_params, original)
                self.assertEqual(app._gait_owned, {})
                self.assertEqual(app.stopped, [])
                self.assertEqual(app.commands, [])
                self.assertEqual(app._gait_supports, ('A', 'B'))
                app._gait_abort_run()  # Repeated cancellation must be idempotent.
                self.assertFalse(app._gait_needs_recovery)
                self.assertEqual(app.stopped, [])
                self.assertIsNotNone(app._gait_begin_run('left')[0])

    def test_queued_or_completed_motion_still_requires_stop_and_recovery(self):
        for completed in (False, True):
            with self.subTest(completed=completed):
                app = mechanism()
                run, _ = app._gait_begin_run('left')
                run.advance_confirm()
                run.advance_confirm()
                run.begin_stage_execution()
                if completed:
                    run.execute_current_stage()
                app._gait_abort_run()
                self.assertTrue(app._gait_needs_recovery)
                self.assertTrue(app.stopped)
                with self.assertRaises(GaitExecutorError):
                    app._gait_begin_run('left')

    def test_fault_or_failed_run_never_takes_early_cancel_path(self):
        for failed in (False, True):
            app = mechanism()
            run, _ = app._gait_begin_run('left')
            if failed:
                run._fail('failed', 'synthetic failure')
            else:
                app._hardware_estop_active = True
            app._gait_abort_run()
            self.assertTrue(app._gait_needs_recovery)
            self.assertTrue(app.stopped)


class BaselineWorkerExclusionTests(unittest.TestCase):
    def app(self):
        app = baseline_app()
        app._control_context = threading.local()
        app._control_generation = 0
        for key in ('_estop_in_progress', '_hardware_estop_active',
                    '_estop_unconfirmed', '_closing', '_disconnecting'):
            setattr(app, key, False)
        return app

    def test_active_worker_blocks_capture_until_finished(self):
        app = self.app()
        release = threading.Event()
        worker = app._start_control_worker(lambda: release.wait(2))
        try:
            with self.assertRaises(GaitExecutorError):
                app._gait_capture_baseline(app.gait_params)
        finally:
            release.set()
            worker.join(2)
        self.assertEqual(app._active_control_workers, 0)

    def test_capture_blocks_new_worker_and_thread_start_failure_releases_count(self):
        app = self.app()
        app._gait_baseline_recording = True
        target = Mock()
        self.assertIsNone(app._start_control_worker(target))
        target.assert_not_called()
        app._gait_baseline_recording = False
        with patch('motor_control.desktop_app.threading.Thread.start', side_effect=RuntimeError('no thread')):
            with self.assertRaises(RuntimeError):
                app._start_control_worker(target)
        self.assertEqual(app._active_control_workers, 0)

    def test_manual_command_and_driver_zero_cannot_destroy_baseline_during_run(self):
        app = self.app()
        app._send_and_read = Mock()
        app._gait_owned = {102: object()}
        ok, reason = app._send_manual_x42s('X42S,2,MOVE,100,50', 'Mr1', 2, 100, 50)
        self.assertFalse(ok)
        app._send_and_read.assert_not_called()
        app._gait_owned = {}
        ok, reason = app._send_manual_x42s('X42S,2,ZERO', 'Mr1', 2)
        self.assertFalse(ok)
        self.assertIn('双侧基准', reason)
        app._send_and_read.assert_not_called()

    def test_manual_guard_rechecks_ownership_at_actual_serial_write(self):
        app = self.app()
        def transport(command, **kwargs):
            app._gait_owned = {102: object()}
            self.assertFalse(kwargs['guard']())
            return ''
        app._send_and_read = transport
        self.assertFalse(app._send_manual_x42s('X42S,2,MOVE,100,50', 'Mr1', 2, 100, 50)[0])


if __name__ == '__main__':
    unittest.main()
