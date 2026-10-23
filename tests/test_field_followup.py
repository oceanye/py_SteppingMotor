"""2026-10-23 field log regressions; fake transport, no connected equipment."""
import json
import tempfile
import time
import tkinter as tk
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
from unittest.mock import Mock, patch
from tkinter import ttk

from motor_control import BindingSet, LogicalRole
from motor_control.desktop_app import StepperGUI, NUM_MOTOR_AXES, NUM_STEPPER_AXES
from motor_control.diagnostics import DiagnosticRecorder
from motor_control.diagnostic_export import export_bundle
from motor_control.protocol import parse_line, reply_matcher_for
from motor_control.ui.scrollable import ScrollableFrame
from motor_control.x42s_motion import MoveExpectation, arrived, endpoint_tolerance
from motor_control.x42s_safety import confirm_disabled_standstill
from test_gait_realtime_fixes import FakeClock
from test_x42s_pc import _x42s_gait_app


class StopEvidenceTests(unittest.TestCase):
    def app(self, bad=None):
        app = _x42s_gait_app()
        app._control_generation = 1
        app._track_lease_generation = 0
        app._foc_enabled_ui = [False]*NUM_MOTOR_AXES
        app._move_reservation = {a: None for a in range(NUM_STEPPER_AXES)}
        app._force_relock_rot_axes = Mock()
        app.v_track_status = SimpleNamespace(set=Mock())
        app._x42s_manual_pending = {1: {"target": 70}}
        app._estop_unconfirmed = True
        def send(command, **kwargs):
            app.commands.append(command)
            if command == "ESTOP": return "OK,ESTOP"
            i = int(command.split(',')[-1])
            if bad == 'mismatch': i = 9
            enabled = int(bad == 'enabled')
            online = int(bad != 'offline')
            age = 3000 if bad == 'old' else 5
            measured = time.monotonic()*10 if bad == 'drifting' else 0
            speed = 1 if bad == 'moving' else 0
            mask = 0 if bad == 'unknown' else 63
            return f'X42S,{i},S,{online},{enabled},0,none,0,{measured},0,{speed},0,25,{age},0,{mask}'
        app._send_and_read = send
        return app

    def test_bridge_ack_alone_does_not_clear_pending_or_latch(self):
        for bad in ('enabled','offline','old','drifting','moving','unknown','mismatch'):
            clock = FakeClock()
            with self.subTest(bad=bad), patch('time.monotonic',clock.monotonic), patch('time.sleep',clock.sleep):
                app=self.app(bad)
                self.assertFalse(app._stop_all_outputs())
                self.assertTrue(app._estop_unconfirmed)
                self.assertTrue(app._x42s_manual_pending)
                self.assertIn('停车未确认', app.v_track_status.set.call_args.args[0])

    def test_two_new_disabled_stable_samples_per_axis_confirm(self):
        clock=FakeClock()
        with patch('time.monotonic',clock.monotonic), patch('time.sleep',clock.sleep):
            app=self.app()
            self.assertTrue(app._stop_all_outputs())
            self.assertFalse(app._estop_unconfirmed)
            self.assertFalse(app._x42s_manual_pending)
            for i in range(1,5): self.assertGreaterEqual(app.commands.count(f'X42S,S,{i}'),2)
            self.assertFalse(any(',MOVE,' in c or ',EN,' in c for c in app.commands))

    def test_connect_keeps_diagnostics_available_but_motion_locked(self):
        clock=FakeClock()
        with patch('time.monotonic',clock.monotonic), patch('time.sleep',clock.sleep):
            app=self.app('offline')
            self.assertTrue(app._stop_all_outputs(bridge_handshake=True))
            self.assertTrue(app._estop_unconfirmed)

    def test_rebinding_or_new_control_generation_cannot_confirm_old_stop(self):
        clock=FakeClock()
        with patch('time.monotonic',clock.monotonic), patch('time.sleep',clock.sleep):
            app=self.app()
            send=app._send_and_read
            def changed(*args,**kwargs):
                app._control_generation += 1
                return send(*args,**kwargs)
            app._send_and_read=changed
            self.assertFalse(confirm_disabled_standstill(app,99,timeout_s=1))

    def test_failed_disconnect_stays_connected_but_allows_stop_retry(self):
        app = self.app()
        app._closing = app._disconnect_teardown_in_progress = False
        app._disconnecting = False
        app._is_serial_connected = lambda: True
        app._stop_all_outputs = Mock(return_value=False)
        app._disconnect_serial = Mock()
        with patch('motor_control.desktop_app.messagebox.showerror'):
            app.toggle_connect()
        self.assertFalse(app._disconnecting)
        self.assertTrue(app._estop_unconfirmed)
        app._disconnect_serial.assert_not_called()


class RoleToleranceTests(unittest.TestCase):
    def test_recorded_half_degree_lift_error_not_a_rotary_waiver(self):
        b=SimpleNamespace(gear_ratio=1,lead_mm=1)
        linear=endpoint_tolerance('Mup1',b)
        rotary=endpoint_tolerance('Mr1',b)
        self.assertAlmostEqual(linear,.72)
        self.assertAlmostEqual(rotary,.225)
        r=parse_line('X42S,1,S,1,1,1,none,25200.199,25200.699,-.49,0,0,30,35,0,63')
        e=MoveExpectation(object(),25200.2,990,origin_motor_deg=.2,tolerance_motor_deg=linear)
        self.assertTrue(arrived(e,r,1000,.45))
        self.assertFalse(e.outside_travel(r.measured_position))
        self.assertFalse(arrived(replace(e,tolerance_motor_deg=rotary),r,1000,.45))
        self.assertTrue(e.outside_travel(25202))

    def test_linear_bound_cannot_grow_unlimited_with_gearing(self):
        self.assertEqual(endpoint_tolerance('Mup1',SimpleNamespace(gear_ratio=100,lead_mm=.1)),1)


class RetiredFocAndPopupTests(unittest.TestCase):
    def test_no_background_poll_or_raw_send_when_gear_is_disabled(self):
        app=_x42s_gait_app()
        app.gear_enabled=False
        app._serial_source_is_current=Mock(side_effect=AssertionError('must not poll'))
        app._foc_poll_loop(1,None)
        self.assertEqual(app.commands,[])
        self.assertIn('FOC retired',StepperGUI._send_and_read(app,'FOC,0,EN,1'))
        self.assertEqual(app.commands,[])

    def test_retired_foc_entry_points_stay_off_even_in_gear_maintenance(self):
        app = _x42s_gait_app()
        app.gear_enabled = True
        app._start_control_worker = Mock(side_effect=AssertionError('must not start'))
        app._send_foc(0, 'EN,1')
        app._apply_foc_tune_to_firmware()
        app._foc_autotune(0)
        app._foc_autotune_worker(0)
        self.assertEqual(app.commands, [])

    def test_tcl_only_popdown_never_crashes_or_scrolls_form(self):
        root=tk.Tk();root.withdraw()
        try:
            form=ScrollableFrame(root)
            combo=ttk.Combobox(form.content,values=('left','right'))
            popup=str(root.tk.call('ttk::combobox::PopdownWindow',str(combo)))
            form.winfo_containing=lambda *_:root.nametowidget(popup)
            form.canvas.yview_scroll=Mock()
            event=SimpleNamespace(x_root=0,y_root=0,widget=combo,delta=120)
            self.assertIsNone(form._wheel(event))
            form.canvas.yview_scroll.assert_not_called()
        finally:root.destroy()

    def test_default_gui_has_no_foc_or_gear_but_retains_track(self):
        from scripts.gui_smoke import isolated_gui
        with patch.dict('os.environ',{'STEPPER_GEAR_ENABLED':'0'}), isolated_gui() as (app,errors):
            names=[app.notebook.tab(t,'text') for t in app.notebook.tabs()]
            self.assertFalse(any('FOC' in s or '减速' in s for s in names))
            self.assertTrue(any('轨道 D' in s for s in names))
            app._disconnect_serial('offline regression', is_error=False)
            app._drain_ui_actions()
            self.assertEqual(errors,[])

    def test_opted_in_gear_status_does_not_access_retired_foc_widgets(self):
        from scripts.gui_smoke import isolated_gui
        with patch.dict('os.environ', {'STEPPER_GEAR_ENABLED':'1'}), isolated_gui() as (app, errors):
            app.ser = SimpleNamespace(is_open=True)
            app._update_foc_display(0, '2', '12.5', '20', '0')
            self.assertEqual(app._motor_status[0]['current_deg'], 12.5)
            self.assertEqual(app.fw, [{}, {}])
            self.assertTrue(app.gw[0])
            app.ser = None
            app._disconnect_serial('offline regression', is_error=False)
            app._drain_ui_actions()
            self.assertEqual(errors, [])

    def test_firmware_identity_is_precise_and_unknown_is_visible(self):
        match=reply_matcher_for('X42S,INFO')
        self.assertTrue(match(parse_line('OK,X42S,INFO,2,Oct 23 2026 12:00:00')))
        self.assertFalse(match(parse_line('OK,X42S,INFO')))
        self.assertFalse(match(parse_line('OK,X42S,2,STOP')))
        app=_x42s_gait_app()
        app._send_and_read=lambda *a,**k:'ERR:bad driver id'
        app._query_x42s_firmware_info()
        self.assertFalse(app._x42s_firmware_info['verified_revision'])
        self.assertIn('已烧录版本',app._logs[-1])


class EvidenceRetentionTests(unittest.TestCase):
    def test_poll_rotation_cannot_evict_action_journal_and_bundle_contains_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder=DiagnosticRecorder(tmp,max_bytes=1800,backups=1)
            try:
                recorder.event('motor','serial.tx_written',line='X42S,2,MOVE,100,50')
                recorder.event('telemetry','x42s.telemetry',role='Mr1',online=True,enabled=True,fault=None)
                for i in range(60):
                    recorder.event('motor','serial.tx_written',line='X42S,S,2',i=i)
                recorder.flush()
                files=recorder.snapshot_files()
                journal=''.join(v for k,v in files.items() if k.startswith('events.'))
                self.assertIn('X42S,2,MOVE',journal)
                self.assertNotIn('X42S,S,2',journal)
                self.assertGreater(recorder.health()['rotations']['motor'],0)
                self.assertEqual(recorder.health()['dropped'],0)
                archive=export_bundle(tmp,recorder=recorder)
                with zipfile.ZipFile(archive) as z:
                    self.assertTrue(any('events.jsonl' in name for name in z.namelist()))
                    self.assertEqual(json.loads(z.read('manifest.json'))['telemetry_rows'], 1)
            finally:recorder.close()

    def test_fault_window_keeps_pre_and_post_and_only_transitions_in_journal(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder=DiagnosticRecorder(tmp)
            try:
                recorder.event('motor','serial.tx_written',line='X42S,S,2')
                for i in range(5):
                    recorder.event('telemetry','x42s.telemetry',role='Mr2',online=False,enabled=True,fault=None)
                recorder.event('motor','serial.tx_written',line='X42S,S,4')
                recorder.flush()
                journal=[json.loads(s) for s in (recorder.directory/'events.jsonl').read_text(encoding='utf-8').splitlines()]
                self.assertEqual(sum(e['event']=='x42s.telemetry' for e in journal),1)
                flight=(recorder.directory/'flight.jsonl').read_text(encoding='utf-8')
                self.assertIn('X42S,S,2',flight)
                self.assertIn('X42S,S,4',flight)
            finally:recorder.close()


if __name__=='__main__': unittest.main()
