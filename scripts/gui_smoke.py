"""Real Tk offline smoke: current gait controls, isolated config, no hardware.

python scripts/gui_smoke.py [--show]
Uses shipped 190 mm defaults; never reads/writes site calibration or opens ports.
"""
import os
import sys
import tempfile
import tkinter as tk
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from motor_control import desktop_app
from motor_control.ui import gait_simulation as simulation, gait_twin as twin


@contextmanager
def isolated_gui(*, show=False):
    root = tk.Tk()
    root.withdraw()
    if not show:
        root.attributes('-alpha', 0)
    errors = []
    root.report_callback_exception = lambda *args: errors.append(args)
    recorder = Mock()
    recorder.health.return_value = {'dropped': 0, 'write_errors': 0}
    with tempfile.TemporaryDirectory() as workspace, \
            patch.dict(os.environ, {'STEPPER_LEGACY_STEPDIR': '0'}), \
            patch.object(desktop_app, 'PROJECT_DIR', Path(workspace)), \
            patch.object(desktop_app, 'install_exception_hooks'), \
            patch.object(desktop_app.StepperGUI, 'refresh_ports'), \
            patch.object(desktop_app.serial, 'Serial', side_effect=AssertionError('offline test opened serial')) as serial:
        try:
            app = desktop_app.StepperGUI(root, diagnostics=recorder)
            root.geometry('1480x900')
            app.notebook.select(app.tab_index_gait)
            root.deiconify()
            root.update()
            yield app, errors
            serial.assert_not_called()
        finally:
            for job in root.tk.splitlist(root.tk.call('after', 'info')):
                root.after_cancel(job)
            root.destroy()


def finish_simulation(app):
    """Exercise the real tick/curve accumulator without waiting on wall time."""
    view = app.gait_widgets['twin']
    count = 0
    while view['simulation'].active:
        job = view.pop('simulation_job', None)
        if job:
            app.root.after_cancel(job)
        with patch.object(twin, 'draw_twin'):
            simulation._tick(app)
        count += 1
        if count % 100 == 0:
            twin.draw_twin(app)
        if count > 1000:
            raise AssertionError('simulation stalled')
    twin.draw_twin(app)


def run_smoke(app):
    widgets, view = app.gait_widgets, app.gait_widgets['twin']
    assert view['mode_var'] is app.gait_op_mode_var
    assert view['mode_var'].get() == 'sim'
    for choice in widgets['action_choices']:
        simulation.reset_simulation(app)
        choice.invoke()
        report = view['candidate']['report']
        assert report.feasible, report.message
        assert view['canvas'].find_withtag('candidate_ghost')
        assert view['curve_display']['forecast']
        widgets['sim_btn'].invoke()
        widgets['sim_btn'].invoke()
        assert view['simulation'].paused
        widgets['sim_btn'].invoke()
        finish_simulation(app)
        assert view['simulation'].completed_steps == 1
        assert view['curve_canvas'].find_withtag('curve_model')
        print(f"{report.side} {report.samples[-1].phi_deg:+g}: {report.modality}, simulation/curves OK")
    assert not app._gait_owned
    assert app._gait_supports == app.gait_params.initial_supports


if __name__ == '__main__':
    with isolated_gui(show='--show' in sys.argv) as (app, errors):
        run_smoke(app)
        assert not errors, errors
    print('SMOKE OK (offline, no serial or site configuration writes)')
