"""Bounded live X42S observation. Read-only unless --verify-stop is explicit.

No MOVE/enable/ZERO/raw frame API; never changes binding or calibration files.
Explicit --verify-stop may send EN,0 if a fresh reply still reports enabled.
Run only with exclusive access to the specified ESP32 serial port.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import serial
from motor_control.coordinated_control import BindingSet
from motor_control.diagnostics import DiagnosticRecorder, environment_summary
from motor_control.protocol import X42sStatusReply
from motor_control.serial_session import SerialSession
from motor_control.x42s_safety import confirm_disabled_standstill


class StopProbe:
    def __init__(self, session):
        self.session = session
        self.state_lock = threading.RLock()
        self._control_generation = 1
        self.control_bindings = BindingSet.from_driver_id_mapping(
            {'Mup1': 1, 'Mr1': 2, 'Mup2': 3, 'Mr2': 4},
            gear_ratios={'Mup1': 1, 'Mr1': 21.5, 'Mup2': 1, 'Mr2': 21.5},
            lead_mms={'Mup1': 1, 'Mup2': 1})

    def _send_and_read(self, command, **kwargs):
        kwargs.pop('allow_closing', None)
        return self.session.request_line(command, **kwargs)

    def _apply_x42s_reply(self, *_args):
        pass

    def log(self, message):
        print(message, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--verify-stop', action='store_true',
                        help='Explicitly send ESTOP (disables motors), then verify ID1..4')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('--seconds must be 1..120')
    recorder = DiagnosticRecorder(ROOT, metadata={
        'purpose': 'live X42S read-only probe' if not args.verify_stop else 'live ESTOP verification',
        'environment': environment_summary(ROOT)})
    print(f'DIAGNOSTICS {recorder.directory}', flush=True)
    port = serial.Serial(port=None, baudrate=115200, timeout=.1, write_timeout=1)
    port.port = args.port
    port.dtr = port.rts = False
    session = None
    stats = {i: {'samples': 0, 'valid': 0, 'faults': set(), 'min_deg': None,
                 'max_deg': None, 'max_age_ms': 0, 'timeouts': None} for i in range(1, 5)}
    success = True
    try:
        port.open()
        session = SerialSession(port, diagnostics=recorder,
            unsolicited_handler=lambda msg: print('EVENT '+msg.raw, flush=True),
            event_handler=lambda msg: print('EVENT '+msg.raw, flush=True))
        session.start()
        time.sleep(1.5)
        info = session.request_line('X42S,INFO', timeout=2, soft_timeout=True)
        print(f'FIRMWARE {info}', flush=True)
        recorder.event('motor', 'firmware.x42s_info', reply=info)
        if args.verify_stop:
            sent = time.monotonic()
            ack = session.request_line('ESTOP', timeout=2, soft_timeout=True)
            confirmed = ack == 'OK,ESTOP' and confirm_disabled_standstill(
                StopProbe(session), sent, disable_enabled=True)
            print(f'STOP bridge_ack={ack!r} drivers_confirmed={confirmed}', flush=True)
            recorder.event('motor', 'safety.estop_result', bridge_ack=ack,
                           drivers_confirmed=confirmed, level='INFO' if confirmed else 'ERROR')
            success = success and confirmed
        deadline, index = time.monotonic()+args.seconds, 0
        while time.monotonic() < deadline:
            driver = index % 4 + 1
            index += 1
            reply = session.request(f'X42S,S,{driver}', timeout=1, soft_timeout=True)
            print(reply.raw if reply else f'NO_REPLY ID{driver}', flush=True)
            if isinstance(reply, X42sStatusReply):
                row = stats[driver]
                row['samples'] += 1
                row['max_age_ms'] = max(row['max_age_ms'], reply.age_ms)
                row['timeouts'] = reply.timeout_count
                row['faults'].add(reply.fault)
                valid = reply.online and reply.age_ms <= 1500 and reply.measured_position is not None
                if valid:
                    row['valid'] += 1
                    pos = reply.measured_position
                    row['min_deg'] = pos if row['min_deg'] is None else min(row['min_deg'], pos)
                    row['max_deg'] = pos if row['max_deg'] is None else max(row['max_deg'], pos)
                recorder.event('telemetry', 'x42s.telemetry', role=f'ID{driver}', **asdict(reply))
            time.sleep(.25)
        for driver in range(1, 5):
            reply = session.request_line(f'X42S,DIAG,{driver}', timeout=2, soft_timeout=True)
            print(f'DIAG {reply}', flush=True)
        for row in stats.values():
            row['faults'] = sorted(str(f) for f in row['faults'])
        print('SUMMARY '+json.dumps(stats, ensure_ascii=False), flush=True)
        recorder.event('gui', 'bench.probe_summary', drivers=stats)
        success = success and all(r['valid'] >= 2 and r['faults'] == ['None'] for r in stats.values())
    finally:
        if session is not None:
            session.close()
        elif port.is_open:
            port.close()
        recorder.close()
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
