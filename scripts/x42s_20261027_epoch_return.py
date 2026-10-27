"""Pinned, operator-reviewed 2026-10-27 restart transfer and ID4 return only.

NOT generic automatic power-loss recovery or absolute-position retention.
Old mechanical reference and the last stationary pose are preserved. The user
confirmed no further touch across restart. A fixed offset maps NEW raw readings
to ESTIMATED physical coordinates; no ZERO/config writes or GUI calibration.
Only ID4 returns ~10.05 output degrees, <=0.5-degree segments at 0.5 deg/s.
Other three axes hold; original feedback, endpoint and stationarity guards apply.
The default return never moves lifts or starts a gait. Separately selected
--sequence needs a newly reviewed start and operator visual alignment/support
confirmation. No auto-retry, implicit next operation or bottom seeking. Default offline.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.x42s_lift_5mm import execute_test
from scripts.x42s_recovery_leg import RecoveryLeg, run_legs
from scripts.x42s_suspended_gait import STOP_FILE, SCALES, SuspendedHost

ARCHIVE = ROOT/'docs/diagnostics/diagnostics-2026-10-27-powercycle-reference-review.zip'
ARCHIVE_SHA256 = 'fe16ee5b3e27125d86336afa3d036cb483fd46fb707dd2a7adb1d35579d0da87'
BEFORE_SESSION = '20261026T101742-85106c02b8'
AFTER_SESSION = '20261027T003954-522a1e35ab'
ORIGINAL_ZERO = {1: -1768., 2: 436.3, 3: -23246.5, 4: -1995.2}
BEFORE_RAW = {1: 31.9, 2: 436.3, 3: -21447., 4: -2211.3}
REBOOT_RAW = {i: 0. for i in SCALES}
# new_raw - new_zero == old_raw - original_zero, IF the pose did not move.
TRANSFERRED_ZERO = {i: REBOOT_RAW[i]-(BEFORE_RAW[i]-ORIGINAL_ZERO[i]) for i in SCALES}
REFERENCE_KIND = 'operator_reviewed_powercycle_transfer'


def validate_evidence(path=ARCHIVE):
    if hashlib.sha256(path.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise ValueError('pinned restart evidence changed; fresh review required')
    with zipfile.ZipFile(path) as archive:
        manifest = json.loads(archive.read('manifest.json'))
        if manifest['omitted_files'] or manifest['truncated_files']:
            raise ValueError('incomplete restart evidence')
        for session, expected in ((BEFORE_SESSION, BEFORE_RAW), (AFTER_SESSION, REBOOT_RAW)):
            base = f'sessions/{session}/'
            meta = json.loads(archive.read(base+'session.json'))
            health = meta['health']
            if (not meta['closed'] or health['dropped'] or health['write_errors']
                    or health['rotations']['telemetry']):
                raise ValueError('incomplete stationary reference')
            positions, counts = {}, {i: 0 for i in SCALES}
            for line in archive.read(base+'telemetry.jsonl').decode('utf8').splitlines():
                record = json.loads(line)
                if record['event'] != 'x42s.telemetry':
                    continue
                r = record['data']
                i = r['driver_id']
                if (i not in expected or not r['online'] or r['fault'] or not r['reached']
                        or r['age_ms'] > 500 or r['speed'] != 0
                        or r['measured_position'] != expected[i]):
                    raise ValueError('stationary reference differs from reviewed evidence')
                positions[i] = r['measured_position']
                counts[i] += 1
            if positions != expected or min(counts.values()) < 2:
                raise ValueError('insufficient four-axis reference evidence')


def provenance():
    return dict(reference_kind=REFERENCE_KIND, original_zero_raw_motor_deg=ORIGINAL_ZERO,
        before_raw_motor_deg=BEFORE_RAW, reboot_raw_motor_deg=REBOOT_RAW,
        inferred_zero_raw_motor_deg=TRANSFERRED_ZERO,
        before_session=BEFORE_SESSION, after_session=AFTER_SESSION,
        archive_sha256=ARCHIVE_SHA256, operator_confirmed_no_further_touch=True,
        absolute_position_independently_verified=False, home_completed=False)


class RestartRotaryReturn(RecoveryLeg):
    def rotary_start_limit(self):
        # Narrow exception: this class ONLY admits the pinned ID4 pose below.
        # Generic manual parking retains +/-5 degrees; not a configurable bypass.
        return 15.

    def reference_kind(self):
        return REFERENCE_KIND

    def validate(self):
        if (self.driver != 4 or self.target != 0 or self.reviewed != REBOOT_RAW
                or self.zero_reference != TRANSFERRED_ZERO):
            raise ValueError('only the pinned restart ID4 return is reviewed; no reuse/retry')
        return super().validate()


class TransferredRecoveryLeg(RecoveryLeg):
    """Reviewed current-epoch lift parking; never re-label inferred zero as measured."""
    def reference_kind(self):
        return REFERENCE_KIND

    def validate(self):
        if self.zero_reference != TRANSFERRED_ZERO or self.driver not in (1,3):
            raise ValueError('only retained transferred-reference lift parking')
        return super().validate()


def run_short_lift_trial(session, recorder, _, reviewed, *, confirmed=False):
    from scripts.x42s_four_gait_sequence import validate_start
    if not confirmed:
        raise ValueError('explicit supported no-extra-dwell trial required')
    validate_start(reviewed, TRANSFERRED_ZERO)
    host=SuspendedHost(session,recorder,reviewed,profile='interior',zero_reference=TRANSFERRED_ZERO,
        speed_scale=2,paired_progress_guard=True,reference_kind=REFERENCE_KIND,short_endpoint_checks=True)
    host.prepare(); host.verify_hold()
    for direction in (1,-1):
        host.stage='short_endpoint_lift_out' if direction>0 else 'short_endpoint_lift_back'
        for index in range(1,11):
            target=5+.4*index if direction>0 else 9-.4*index
            host.dispatch({i:TRANSFERRED_ZERO[i]+target*360 for i in (1,3)},
                          {i:2.5 if direction>0 or target>=7 else .05 if target<=5.5 else .25 for i in (1,3)})
            if not host.wait_group((1,3),10):
                raise RuntimeError('short endpoint lift trial failed')
            host.heights={1:target,3:target}
            host.log(f'SHORT ENDPOINT {host.stage} {index}/10 target_mm={target:g}')
        host.verify_stage_boundary()
    samples=host.ready_snapshot()
    return dict(groups_completed=host.groups_completed,reference_transfer=provenance(),
        short_endpoint_checks=True,full_gaits_completed=0,lift_range_mm=[5,9],
        raw_motor_deg={i:r.measured_position for i,(r,_) in samples.items()})


def run_return(session, recorder, _, *, confirmed=False, stop_file=STOP_FILE):
    if not confirmed or stop_file.exists():
        raise ValueError('explicit unchanged-pose supported return confirmation required')
    validate_evidence()
    recorder.event('motor', 'bench.restart_reference_transfer', **provenance())
    leg = RestartRotaryReturn(4, REBOOT_RAW, 0., TRANSFERRED_ZERO)
    result = run_legs(session, recorder, (leg,), confirmed=True, stop_file=stop_file, hold_other_axes=True)
    return {**result, 'reference_transfer': provenance(), 'no_automatic_gait': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--unchanged-pose-supported-return-confirmed', action='store_true')
    operation=parser.add_mutually_exclusive_group()
    operation.add_argument('--sequence', action='store_true', help='Separate four-gait operation, never automatic after return')
    operation.add_argument('--park-lifts', action='store_true')
    operation.add_argument('--short-lift-trial', action='store_true')
    parser.add_argument('--reviewed-raw-motor-deg', nargs=4, type=float)
    parser.add_argument('--visually-aligned-supported-sequence-confirmed', action='store_true')
    parser.add_argument('--no-extra-intermediate-dwell-confirmed', action='store_true',
        help='Two fresh zero-speed endpoint samples; 1s stability at stage boundaries, not every segment')
    args = parser.parse_args()
    validate_evidence()
    if args.park_lifts or args.short_lift_trial:
        if not args.reviewed_raw_motor_deg:
            parser.error('explicit four-axis current raw snapshot required')
        reviewed=dict(zip(range(1,5),args.reviewed_raw_motor_deg))
        if args.park_lifts:
            legs=tuple(TransferredRecoveryLeg(i,reviewed,5.,TRANSFERRED_ZERO) for i in (1,3))
            for leg in legs:leg.validate()
            runner=lambda s,r,d:run_legs(s,r,legs,confirmed=True)
            prefix='bench.restart_lift_park'
        else:
            from scripts.x42s_four_gait_sequence import validate_start
            validate_start(reviewed,TRANSFERRED_ZERO)
            runner=lambda s,r,d:run_short_lift_trial(s,r,d,reviewed,confirmed=True)
            prefix='bench.short_endpoint_trial'
        print(f'PLAN {prefix}: retained inferred reference, explicit standalone operation',flush=True)
        if not args.execute:return 0
        if (not args.port or not args.visually_aligned_supported_sequence_confirmed or STOP_FILE.exists()
                or (args.short_lift_trial and not args.no_extra_intermediate_dwell_confirmed)):
            parser.error('explicit supported operation, no-extra-dwell approval for trial, and clear stop required')
        return execute_test(args.port,(1,3),runner,purpose=prefix+'; not full gait acceptance',
            event_prefix=prefix,diagnostic_max_bytes=32_000_000,
            test_context=dict(provenance(),reviewed_raw_motor_deg=reviewed,
                no_extra_intermediate_dwell=args.short_lift_trial,
                source_sha256={n:hashlib.sha256((ROOT/'scripts'/n).read_bytes()).hexdigest()
                    for n in ('x42s_20261027_epoch_return.py','x42s_recovery_leg.py','x42s_suspended_gait.py')}))
    if args.sequence:
        from motor_control.gait_planner import parse_gait_params
        from scripts.x42s_four_gait_sequence import build_sequence, run_sequence, validate_start
        params = parse_gait_params(json.loads((ROOT/'.gait_params.json').read_text(encoding='utf8')))
        plans = build_sequence(params)
        context = dict(provenance(), rotary_alignment_visually_confirmed=True)
        print('PLAN: four suspended actions, 5..65 mm, restart-derived reference; NOT standard70mm', flush=True)
        if not args.execute:
            return 0
        if (not args.port or not args.reviewed_raw_motor_deg
                or not args.visually_aligned_supported_sequence_confirmed or STOP_FILE.exists()):
            parser.error('fresh reviewed pose, visual alignment, support and clear range required')
        reviewed = dict(zip(range(1,5),args.reviewed_raw_motor_deg))
        validate_start(reviewed, TRANSFERRED_ZERO)
        return execute_test(args.port, (1,2,3,4),
            lambda s,r,d: run_sequence(s,r,d,plans,reviewed,TRANSFERRED_ZERO,confirmed=True,
                                      reference_provenance=context,
                                      short_endpoint_checks=args.no_extra_intermediate_dwell_confirmed),
            purpose='four suspended interior gaits with operator-reviewed restart reference; NOT standard70mm',
            event_prefix='bench.sequence', diagnostic_max_bytes=64_000_000,
            test_context=dict(context, reviewed_raw_motor_deg=reviewed, lift_range_mm=[5,65],
                speed_scale=2, independently_supported_clear_fixture=True,
                no_extra_intermediate_dwell=args.no_extra_intermediate_dwell_confirmed,
                source_sha256={n:hashlib.sha256((ROOT/'scripts'/n).read_bytes()).hexdigest()
                    for n in ('x42s_20261027_epoch_return.py','x42s_four_gait_sequence.py',
                              'x42s_suspended_gait.py','x42s_lift_5mm.py')}))
    leg = RestartRotaryReturn(4, REBOOT_RAW, 0., TRANSFERRED_ZERO)
    print(json.dumps(dict(provenance(), driver=4, inferred_start_output_deg=leg.validate(),
        groups=len(leg.waypoints()), command_speed_output_deg_s=.5), ensure_ascii=False), flush=True)
    if not args.execute:
        return 0
    if not args.port or not args.unchanged_pose_supported_return_confirmed or STOP_FILE.exists():
        parser.error('port, explicit unchanged pose/support and clear stop required')
    return execute_test(args.port, 4, lambda s, r, d: run_return(s, r, d, confirmed=True),
        purpose='pinned restart coordinate transfer; bounded ID4 return, physical zero remains inferred',
        event_prefix='bench.restart_return', diagnostic_max_bytes=32_000_000,
        test_context=dict(provenance(), independently_supported_clear_fixture=True,
            source_sha256={n:hashlib.sha256((ROOT/'scripts'/n).read_bytes()).hexdigest()
                for n in ('x42s_20261027_epoch_return.py','x42s_recovery_leg.py','x42s_lift_5mm.py')}))


if __name__ == '__main__':
    raise SystemExit(main())
