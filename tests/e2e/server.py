"""Loopback-only offline fixture using the REAL HTTP server/desktop adapter.

FakeDesktopApplication contains no serial object, Tk window, or config store.
Only firmware I/O is simulated. There are no test-only production API routes.
"""
import argparse
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tests')]
from test_web_adapter import FakeDesktopApplication
from motor_control.actuator_model import X42sActuatorState
from motor_control.coordinated_control import (
    BindingSet, LOGICAL_ROLE_ORDER, build_coordinated_snapshot,
)
from motor_control.web_adapter import DesktopWebController
from web_control import WebControlServer


class OfflineApplication(FakeDesktopApplication):
    def __init__(self):
        super().__init__()
        self.legacy_stepdir_enabled = False
        self._fw_mode_cache = 'GEAR'
        self.control_bindings = BindingSet.from_driver_id_mapping(
            {'Mup1': 1, 'Mr1': 2, 'Mup2': 3, 'Mr2': 4},
            gear_ratios={'Mup1': 1, 'Mr1': 21.5, 'Mup2': 1, 'Mr2': 21.5},
            lead_mms={'Mup1': 8, 'Mup2': 8})

    def _snapshot_coordinated_control_locked(self):
        # Deliberately different command/measured values prove no target-as-position.
        states = {}
        for role in LOGICAL_ROLE_ORDER:
            binding = self.control_bindings.for_role(role)
            motor_deg = 360 if role.value.startswith('Mup') else 1935  # 8mm / 90deg
            states[role.value] = X42sActuatorState(role, binding.driver_id).with_telemetry(
                now=time.monotonic(), enabled=True, reached=True,
                measured_position=motor_deg, command_position=motor_deg * 2,
                position_error=0, speed=None)
        return build_coordinated_snapshot(
            bindings=self.control_bindings, profiles=self.axis_profiles,
            runtimes=self.axis_runtime, axis_param_valid=self.axis_param_valid,
            running=self.running, stepper_in_progress=self.stepper_in_progress,
            move_dispatching=self._move_dispatching, move_reservations=self._move_reservation,
            web_step_pending=self._web_step_pending, pending_steps=self._pending_step,
            telemetry=self.axis_motion_telemetry, connected=self._connected,
            x42s_states=states)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=18765)
    args = parser.parse_args()
    server = WebControlServer(DesktopWebController(OfflineApplication()), '127.0.0.1', args.port)
    server.start()
    print(f'OFFLINE ONLY: http://127.0.0.1:{server.bound_port}', flush=True)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
