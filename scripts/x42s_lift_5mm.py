"""Operator-observed +5 mm lift test; NOT homing or automatic stroke discovery.

Fixed bench mapping: ID1=Mup1, ID3=Mup2, direct drive, lead 1 mm/rev.
Requires confirmed bottom zero, positive extension, support and 10 mm clearance.
Ten absolute waypoints, 0.5 mm apart, at 0.25 mm/s. Other axes stay disabled.
No reverse, ZERO, PID writes, automatic retry or return. Stall is an ABORT,
never evidence of a mechanical endpoint. The original micro-jog stays unchanged.
PC failure may still let the one outstanding <=0.51 mm segment finish;
these host checks are NOT a mechanical limit or a firmware communication lease.
"""
import argparse
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import serial
from scripts.x42s_bench_probe import StopProbe
from scripts.x42s_micro_jog import snapshot, quantized_target
from motor_control.coordinated_control import LOGICAL_ROLE_ORDER
from motor_control.diagnostics import DiagnosticRecorder, environment_summary
from motor_control.protocol import X42sMoveAck
from motor_control.serial_session import SerialSession
from motor_control.x42s_motion import MoveExpectation, arrived, endpoint_tolerance
from motor_control.x42s_safety import confirm_disabled_standstill

DEG_PER_MM = 360.0  # confirmed lead=1, direct drive
SPEED_PPS = 800  # 0.25 mm/s, 15 motor RPM
MAX_SEGMENT_PULSES = 1632  # 0.51 mm, including waypoint residual correction
MAX_POSITION_DEG = 5.01 * DEG_PER_MM  # bench envelope, not physical upper limit
WAYPOINTS = tuple(180.0 * i for i in range(1, 11))


def segment_steps(position, target):
    if not all(math.isfinite(x) for x in (position, target)):
        raise ValueError('nonfinite position/target')
    if not 0 <= position < target <= WAYPOINTS[-1]:
        raise ValueError('only positive travel inside bottom..5 mm is permitted')
    steps = math.floor((target-position)*3200/360 + 1e-9)
    if not 0 < steps <= MAX_SEGMENT_PULSES:
        raise ValueError('segment exceeds 0.51 mm or is below command resolution')
    return steps


def stationary(reply):
    return (reply.measured_position is not None and reply.speed is not None
            and math.isfinite(reply.speed) and abs(reply.speed) <= .01)


def check_others(samples, origins, tolerances, driver):
    for other, (reply, _) in samples.items():
        if other == driver:
            continue
        if (reply.enabled or not stationary(reply)
                or abs(reply.measured_position-origins[other]) > tolerances[other]):
            raise RuntimeError(f'untested ID{other} enabled/moving/position changed')


def run_lift(session, recorder, driver, *, confirmed=False):
    """Caller MUST issue and verify ESTOP in finally, including all failures."""
    if not confirmed or driver not in (1, 3):
        raise ValueError('explicit lift direction, bottom, clearance/support confirmation required')
    samples = snapshot(session, recorder)
    if any(r.enabled or not stationary(r) for r, _ in samples.values()):
        raise RuntimeError('all four drivers must be disabled and stationary before enable')
    origins = {i: r.measured_position for i, (r, _) in samples.items()}
    if not 0 <= origins[driver] <= 1.5:
        raise RuntimeError('lift not near confirmed bottom zero; no enable/MOVE')
    bindings = StopProbe(session).control_bindings
    tolerances = {b.driver_id: endpoint_tolerance(role, b)
                  for role in LOGICAL_ROLE_ORDER for b in [bindings.for_role(role)]}
    if session.request_line(f'X42S,{driver},EN,1', timeout=1) != f'OK,X42S,{driver},EN,1':
        raise RuntimeError('enable not precisely acknowledged')
    enabled_at = time.monotonic()
    while True:
        samples = snapshot(session, recorder, pending_driver=driver)
        check_others(samples, origins, tolerances, driver)
        r, now = samples[driver]
        if r.measured_position is not None:
            if abs(r.measured_position-origins[driver]) > tolerances[driver]:
                raise RuntimeError('lift position changed during enable; no MOVE')
            if r.enabled and stationary(r) and now-r.age_ms/1000 > enabled_at:
                break
        if time.monotonic()-enabled_at > 3:
            raise RuntimeError('fresh stationary enabled feedback unavailable')

    for index, waypoint in enumerate(WAYPOINTS, 1):
        origin = samples[driver][0].measured_position
        steps = segment_steps(origin, waypoint)
        ack = session.request(f'X42S,{driver},MOVE,{steps},{SPEED_PPS}', timeout=1)
        if not isinstance(ack, X42sMoveAck) or (ack.driver_id, ack.steps, ack.speed) != (driver, steps, SPEED_PPS):
            raise RuntimeError('MOVE not precisely acknowledged; no retry/return')
        expected = MoveExpectation(object(), quantized_target(origin, steps), time.monotonic(),
                                   origin_motor_deg=origin, tolerance_motor_deg=tolerances[driver])
        deadline = expected.acknowledged_at + steps/SPEED_PPS + 4
        progress_at, progress_position = expected.acknowledged_at, origin
        previous_arrival = None
        while True:
            samples = snapshot(session, recorder, pending_driver=driver)
            check_others(samples, origins, tolerances, driver)
            r, now = samples[driver]
            if time.monotonic() > deadline:
                raise RuntimeError('arrival timeout; no retry/return')
            if r.measured_position is None:
                if now-expected.acknowledged_at > .75:
                    raise RuntimeError('lift position missing after start window')
                continue
            position = r.measured_position
            if (not r.enabled or expected.outside_travel(position)
                    or not 0 <= position <= MAX_POSITION_DEG):
                raise RuntimeError(f'lift disabled/outside travel envelope: {r.raw}')
            sample_at = now-r.age_ms/1000
            if sample_at > expected.acknowledged_at and position > progress_position+.1:
                progress_at, progress_position = sample_at, position
            if (abs(position-expected.target_motor_deg) > tolerances[driver]
                    and now-progress_at > 1.25):
                raise RuntimeError('no measured progress; NOT a confirmed mechanical limit')
            print(f'FEEDBACK ID{driver} height_mm={position/DEG_PER_MM:.5f} {r.raw}', flush=True)
            if stationary(r) and arrived(expected, r, now, .45):
                if previous_arrival is not None and sample_at-previous_arrival >= .1:
                    if abs(position-waypoint) > tolerances[driver]:
                        raise RuntimeError('absolute waypoint residual too large; no retry')
                    recorder.event('motor', 'bench.lift_waypoint', driver_id=driver,
                                   index=index, target_mm=waypoint/DEG_PER_MM,
                                   measured_mm=position/DEG_PER_MM, steps=steps)
                    print(f'WAYPOINT {index}/10 ID{driver} measured_mm={position/DEG_PER_MM:.5f}', flush=True)
                    break
                if previous_arrival is None:
                    previous_arrival = sample_at
            else:
                previous_arrival = None
    return {'driver_id': driver, 'origin_motor_deg': origins[driver],
            'measured_motor_deg': samples[driver][0].measured_position,
            'measured_mm': samples[driver][0].measured_position/DEG_PER_MM}


def execute_test(port_name, driver, runner, *, purpose='operator-authorized +5 mm segmented lift observation',
                 test_context=None, event_prefix='bench.lift', diagnostic_max_bytes=2_000_000):
    """Shared live lifecycle: identified firmware, initial/final verified ESTOP."""
    recorder = DiagnosticRecorder(ROOT, max_bytes=diagnostic_max_bytes, metadata={
        'purpose': purpose,
        'driver_id': driver,
        'units': {1: {'role': 'Mup1', 'lead_mm': 1, 'gear_ratio': 1},
                  2: {'role': 'Mr1', 'gear_ratio': 21.5},
                  3: {'role': 'Mup2', 'lead_mm': 1, 'gear_ratio': 1},
                  4: {'role': 'Mr2', 'gear_ratio': 21.5}},
        'positive_is_extension': True,
        'test_context': test_context if test_context is not None else {'confirmed_clearance_mm': 10},
        'environment': environment_summary(ROOT)})
    print(f'DIAGNOSTICS {recorder.directory}', flush=True)
    port = serial.Serial(port=None, baudrate=115200, timeout=.1, write_timeout=1)
    port.port = port_name
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
        sent = time.monotonic()
        if session.request_line('ESTOP', timeout=1) != 'OK,ESTOP' or not confirm_disabled_standstill(
                StopProbe(session), sent, disable_enabled=True):
            raise RuntimeError('initial four-driver stop not confirmed')
        result = runner(session, recorder, driver)
        recorder.event('motor', event_prefix+'_completed', **result)
        print(f'MOTION_COMPLETE {result}', flush=True)
        success = True
    except (Exception, KeyboardInterrupt) as exc:
        print(f'ABORT {exc}', flush=True)
        recorder.exception('motor', event_prefix+'_aborted', exc)
    finally:
        if session is not None:
            try:
                sent = time.monotonic()
                ack = session.request_line('ESTOP', timeout=1, soft_timeout=True)
                stopped = ack == 'OK,ESTOP' and confirm_disabled_standstill(
                    StopProbe(session), sent, disable_enabled=True)
                print(f'FINAL_STOP drivers_confirmed={stopped}', flush=True)
                recorder.event('motor', 'safety.estop_result', drivers_confirmed=stopped,
                               level='INFO' if stopped else 'ERROR')
                success = success and stopped
            except Exception as exc:
                success = False
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--driver', required=True, type=int, choices=(1, 3))
    for flag in ('positive-is-extension', 'bottom-zero-confirmed', 'supported-10mm-clearance'):
        parser.add_argument('--'+flag, required=True, action='store_true')
    args = parser.parse_args()
    return execute_test(args.port, args.driver,
                        lambda s, r, d: run_lift(s, r, d, confirmed=True))


if __name__ == '__main__':
    raise SystemExit(main())
