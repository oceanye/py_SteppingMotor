"""Explicit bench test: ONE driver, +80/-80 pulses at 80pps, then ESTOP.

80 pulses = 9 motor degrees. Not a gait, calibration, PID tune or recovery.
Requires operator-confirmed clearance/support. Never returns after a failure.
"""
import argparse
from dataclasses import asdict
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import serial
from scripts.x42s_bench_probe import StopProbe
from motor_control.coordinated_control import LOGICAL_ROLE_ORDER
from motor_control.diagnostics import DiagnosticRecorder, environment_summary
from motor_control.protocol import X42sStatusReply, X42sMoveAck
from motor_control.serial_session import SerialSession
from motor_control.x42s_motion import MoveExpectation, arrived, endpoint_tolerance
from motor_control.x42s_safety import confirm_disabled_standstill


def snapshot(session, recorder):
    result = {}
    for driver in range(1, 5):
        r = session.request(f'X42S,S,{driver}', timeout=1, soft_timeout=True)
        now = time.monotonic()
        if not isinstance(r, X42sStatusReply) or r.driver_id != driver:
            raise RuntimeError(f'ID{driver}: missing/mismatched reply')
        recorder.event('telemetry', 'x42s.telemetry', role=f'ID{driver}', **asdict(r))
        if not r.online or r.fault or r.age_ms > 1500:
            raise RuntimeError(f'ID{driver}: unhealthy feedback: {r.raw}')
        if r.measured_position is not None and not math.isfinite(r.measured_position):
            raise RuntimeError(f'ID{driver}: nonfinite position')
        result[driver] = (r, now)
        time.sleep(.05)
    return result


def stable_origins(samples):
    origins = {}
    for driver, (r, _) in samples.items():
        if r.measured_position is None or (r.speed is not None and abs(r.speed) > .01):
            raise RuntimeError(f'ID{driver}: stationary origin unavailable')
        origins[driver] = r.measured_position
    return origins


def run_cycle(session, recorder, driver):
    """Fail-closed fixed travel; caller MUST issue/verify ESTOP in finally."""
    origins = stable_origins(snapshot(session, recorder))
    bindings = StopProbe(session).control_bindings
    tolerances = {b.driver_id: endpoint_tolerance(role, b)
                  for role in LOGICAL_ROLE_ORDER for b in [bindings.for_role(role)]}
    if session.request_line(f'X42S,{driver},EN,1', timeout=1) != f'OK,X42S,{driver},EN,1':
        raise RuntimeError('enable not confirmed')
    enabled_at = time.monotonic()
    deadline = enabled_at+3
    while True:
        samples = snapshot(session, recorder)
        r, now = samples[driver]
        if r.enabled and r.measured_position is not None and now-r.age_ms/1000 > enabled_at:
            break
        if time.monotonic() > deadline:
            raise RuntimeError('fresh enabled feedback missing')
    enabled_origins = stable_origins(samples)
    if any(abs(enabled_origins[i]-origins[i]) > tolerances[i] for i in origins):
        raise RuntimeError('position changed during enable; abort, no MOVE')
    # Fixed motor travel; role tolerances match the shipped ID1..4 binding.
    for steps in (80, -80):
        origin = samples[driver][0].measured_position
        command = f'X42S,{driver},MOVE,{steps},80'
        ack = session.request(command, timeout=1)
        if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (driver, steps, 80):
            raise RuntimeError('MOVE not precisely acknowledged; no retry/return')
        expected = MoveExpectation(object(), origin+steps*360/3200, time.monotonic(),
                                   origin_motor_deg=origin, tolerance_motor_deg=tolerances[driver])
        deadline, previous_arrival = time.monotonic()+8, None
        while True:
            samples = snapshot(session, recorder)
            for other, (r, now) in samples.items():
                if other != driver:
                    if r.measured_position is None or abs(r.measured_position-origins[other]) > tolerances[other]:
                        raise RuntimeError(f'untested ID{other} moved/lost position')
                    continue
                if not r.enabled or expected.outside_travel(r.measured_position):
                    raise RuntimeError(f'ID{driver} disabled/overshot: {r.raw}')
                print(r.raw, flush=True)
            r, now = samples[driver]
            if arrived(expected, r, now, .45):
                sample_at = now-r.age_ms/1000
                if previous_arrival is not None and sample_at-previous_arrival >= .1:
                    print(f'ARRIVED ID{driver} steps={steps} expected={expected.target_motor_deg:.3f}'
                          f' measured={r.measured_position:.3f}', flush=True)
                    recorder.event('gui', 'bench.jog_arrived', driver_id=driver, steps=steps,
                                   target_motor_deg=expected.target_motor_deg,
                                   measured_motor_deg=r.measured_position)
                    break
                if previous_arrival is None:
                    previous_arrival = sample_at
            else:
                previous_arrival = None
            if time.monotonic() > deadline:
                raise RuntimeError('arrival timeout; no retry/return')
    returned = samples[driver][0].measured_position
    if abs(returned-origins[driver]) > tolerances[driver]:
        raise RuntimeError('round-trip origin mismatch; no corrective move')
    return {'driver_id': driver, 'origin_motor_deg': origins[driver], 'returned_motor_deg': returned}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--driver', required=True, type=int, choices=range(1, 5))
    parser.add_argument('--operator-confirmed-safe', required=True, action='store_true')
    args = parser.parse_args()
    recorder = DiagnosticRecorder(ROOT, metadata={'purpose': 'operator-authorized bounded micro-jog',
                                                 'environment': environment_summary(ROOT)})
    print(f'DIAGNOSTICS {recorder.directory}', flush=True)
    port = serial.Serial(port=None, baudrate=115200, timeout=.1, write_timeout=1)
    port.port = args.port
    port.dtr = port.rts = False
    session, success = None, False
    try:
        port.open()
        session = SerialSession(port, diagnostics=recorder)
        session.start()
        time.sleep(1.5)
        info = session.request_line('X42S,INFO', timeout=1)
        if not info or not info.startswith('OK,X42S,INFO,2,'):
            raise RuntimeError('unidentified firmware; refusing motion')
        start = time.monotonic()
        if session.request_line('ESTOP', timeout=1) != 'OK,ESTOP' or not confirm_disabled_standstill(StopProbe(session), start):
            raise RuntimeError('initial four-driver stop not confirmed')
        result = run_cycle(session, recorder, args.driver)
        print(f'PASS {result}', flush=True)
        success = True
    except Exception as exc:
        print(f'ABORT {exc}', flush=True)
        recorder.exception('motor', 'bench.jog_aborted', exc)
    finally:
        if session is not None:
            try:
                start = time.monotonic()
                ack = session.request_line('ESTOP', timeout=1, soft_timeout=True)
                stopped = ack == 'OK,ESTOP' and confirm_disabled_standstill(StopProbe(session), start)
                print(f'FINAL_STOP drivers_confirmed={stopped}', flush=True)
                recorder.event('motor', 'safety.estop_result', drivers_confirmed=stopped,
                               level='INFO' if stopped else 'ERROR')
                success = success and stopped
            except Exception as exc:
                success = False
                # A poisoned request matcher must not prevent a best-effort stop.
                # This text write is NOT a stop confirmation.
                try:
                    port.write(b'ESTOP\n')
                except Exception:
                    pass
                print(f'FINAL_STOP UNCONFIRMED: {exc}; CHECK PHYSICAL SAFETY', flush=True)
                recorder.exception('motor', 'safety.estop_unconfirmed', exc)
            finally:
                session.close()
                recorder.close()
        else:
            if port.is_open:
                port.close()
            recorder.close()
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
