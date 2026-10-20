"""Full application regressions: sim/live modes, selecting cells and timers."""
import unittest
from unittest.mock import Mock, patch

from scripts.gui_smoke import isolated_gui, finish_simulation, run_smoke
from motor_control.gait_map import map_label, map_pads
from motor_control.ui import gait_twin as twin
from motor_control.ui.gait_curve import curve_payload
from motor_control.ui.gait_twin_window import open_twin_window


class GaitGuiWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.app, self.errors = self.enterContext(isolated_gui())
        self.root = self.app.root
        self.view = self.app.gait_widgets['twin']
        self.original = self.app.gait_params
        self.app._save_gait_params = Mock(side_effect=AssertionError('display wrote calibration'))
        self.app._send_and_read = Mock(side_effect=AssertionError('display sent command'))
        self.addCleanup(self.verify_offline)

    def verify_offline(self):
        self.app._save_gait_params.assert_not_called()
        self.app._send_and_read.assert_not_called()
        self.assertEqual(self.errors, [])
        self.assertIs(self.app.gait_params, self.original)

    def test_four_actions_full_gui_smoke(self):
        run_smoke(self.app)

    def test_sim_start_apply_modes_and_rotated_large_window_selection(self):
        app, view = self.app, self.view
        view['start_left'].set(map_label('C'))
        view['start_right'].set(map_label('A'))
        with patch.object(twin.messagebox, 'askokcancel', side_effect=AssertionError('virtual edit asked hardware confirmation')):
            view['apply_button'].invoke()
        self.assertEqual(view['simulation'].supports, ('C', 'A'))
        self.assertEqual(app._gait_supports, ('A', 'B'))
        app.gait_op_mode_var.set('live')
        self.assertEqual(view['mode_var'].get(), 'live')
        self.assertEqual(view['start_left'].get(), map_label('A'))
        self.assertIn('实机', view['apply_button'].cget('text'))
        view['mode_controls'][0].invoke()
        self.assertEqual(app.gait_op_mode_var.get(), 'sim')
        self.assertEqual(view['start_left'].get(), map_label('C'))
        open_twin_window(app)
        detail = view['large_window']
        detail['window'].attributes('-alpha', 0)
        self.root.update()
        view['rotate_map_control'].invoke()
        view['select_side'].set('left')
        x, y = detail['project'](map_pads()['B'])
        detail['canvas'].event_generate('<Button-1>', x=round(x), y=round(y))
        self.assertEqual(view['start_left'].get(), map_label('B'))
        view['apply_button'].invoke()
        self.assertEqual(view['simulation'].supports, ('B', 'A'))
        view['start_right'].set(map_label('B'))
        view['apply_button'].invoke()
        self.assertIn('未重置', app.gait_widgets['sim_status'].cget('text'))
        self.assertEqual(view['simulation'].supports, ('B', 'A'))

    def test_continuous_curve_pause_mode_switch_pending_start_and_clear(self):
        app, view = self.app, self.view
        app.gait_widgets['sim_btn'].invoke()
        sim = view['simulation']
        app.gait_op_mode_var.set('live')
        self.assertTrue(sim.paused)
        self.assertNotIn('simulation_job', view)
        app.gait_op_mode_var.set('sim')
        app.gait_widgets['sim_btn'].invoke()
        finish_simulation(app)
        for _ in range(5):
            app.gait_widgets['sim_btn'].invoke()
            finish_simulation(app)
        self.assertEqual(sim.completed_steps, 6)
        payload = curve_payload(app, view)
        self.assertGreater(payload['start'], 0)
        self.assertEqual(payload['end']-payload['start'], 90)
        self.assertGreater(len({point[2] for point in payload['data']}), 1)
        self.assertTrue(view['curve_canvas'].find_withtag('curve_zero'))
        stance = sim.supports
        view['start_left'].set(map_label('C'))
        app.gait_widgets['sim_btn'].invoke()
        self.assertFalse(sim.active)
        self.assertIn('尚未应用', app.gait_widgets['sim_status'].cget('text'))
        self.assertEqual(sim.supports, stance)
        view['clear_button'].invoke()
        self.assertFalse(sim.history.points)
        self.assertFalse(sim.curve)
        self.assertEqual(sim.supports, stance)

    def test_stale_telemetry_only_blocks_physical_edit_but_active_run_blocks_sim(self):
        app, view = self.app, self.view
        snapshot = {'axes': {'Mr1': {'position_source': 'x42s_driver', 'state': 'X42S_ONLINE',
                        'position_trusted': False, 'stale': True, 'x42s': {'reached': None}}},
                    'pose': None, 'state': 'stale', 'message': 'stale'}
        app._gait_twin_snapshot = lambda: snapshot
        twin.refresh_twin_panel(app)
        self.assertTrue(view['start_busy'])
        self.assertFalse(view['interaction_busy'])
        self.assertEqual(view['mode_var'].get(), 'sim')
        app.gait_widgets['sim_btn'].invoke()
        self.assertTrue(view['simulation'].active)
        twin.refresh_twin_panel(app)
        self.assertTrue(view['simulation'].active)
        self.assertEqual(view['mode_var'].get(), 'sim')
        app.gait_op_mode_var.set('live')
        self.assertEqual(str(view['apply_button'].cget('state')), 'disabled')
        app._gait_owned = {102: object()}
        twin.refresh_twin_panel(app)
        self.assertFalse(view['simulation'].active)
        self.assertEqual(view['mode_var'].get(), 'live')
        app.gait_op_mode_var.set('sim')
        self.assertEqual(view['mode_var'].get(), 'live')
        self.assertEqual(str(app.gait_widgets['sim_btn'].cget('state')), 'disabled')
        app._gait_owned.clear()

    def test_periodic_refresh_survives_callback_error_and_close_never_reschedules(self):
        with patch.object(self.app, '_refresh_coordinated_ui_once', side_effect=ValueError('injected display error')), \
                patch.object(self.root, 'after') as after:
            with self.assertRaisesRegex(ValueError, 'injected'):
                self.app._refresh_coordinated_ui()
            after.assert_called_once()
            after.reset_mock()
            self.app._closing = True
            with self.assertRaises(ValueError):
                self.app._refresh_coordinated_ui()
            after.assert_not_called()
            self.app._closing = False


if __name__ == '__main__':
    unittest.main()
