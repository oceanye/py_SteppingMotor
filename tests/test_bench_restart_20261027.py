"""Offline replay only: restart zeros are not mechanical zeros."""
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from scripts.x42s_20261027_epoch_return import (
    BEFORE_RAW, ORIGINAL_ZERO, REBOOT_RAW, TRANSFERRED_ZERO, REFERENCE_KIND,
    RestartRotaryReturn, validate_evidence, run_return, main,
)
from scripts.x42s_recovery_leg import RecoveryLeg
from scripts.x42s_suspended_gait import SCALES, TOL
from scripts.x42s_four_gait_sequence import validate_start
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock


class RestartReturnTests(unittest.TestCase):
    def test_pinned_evidence_and_translation_do_not_rezero(self):
        validate_evidence()
        for i in SCALES:
            self.assertAlmostEqual(REBOOT_RAW[i]-TRANSFERRED_ZERO[i], BEFORE_RAW[i]-ORIGINAL_ZERO[i])
        self.assertAlmostEqual(TRANSFERRED_ZERO[4], 216.1)
        self.assertAlmostEqual(TRANSFERRED_ZERO[1], -1799.9)
        self.assertAlmostEqual(TRANSFERRED_ZERO[3], -1799.5)
        with self.assertRaises(ValueError):
            validate_start(REBOOT_RAW, ORIGINAL_ZERO)
        with self.assertRaises(ValueError):
            validate_start(REBOOT_RAW, TRANSFERRED_ZERO)  # ID4 still not aligned!

    def test_complete_return_preserves_other_axes_and_strict_bounds(self):
        clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(REBOOT_RAW)
        recorder = Mock()
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep),patch('builtins.print'):
            result = run_return(rig, recorder, None, confirmed=True)
        self.assertEqual(result['completed_waypoints'], 21)
        self.assertEqual(result['held_enabled_driver_ids'], [1,2,3])
        self.assertEqual(result['reference_kind'], REFERENCE_KIND)
        self.assertNotIn('position_from_manual_zero', result)
        self.assertFalse(result['absolute_position_independently_verified'])
        self.assertFalse(result['home_completed'])
        self.assertAlmostEqual(result['raw_motor_deg'][4], TRANSFERRED_ZERO[4], delta=TOL[4])
        self.assertAlmostEqual(result['inferred_epoch_position'][4], 0, delta=TOL[4]/SCALES[4])
        for i in (1,2,3):
            self.assertEqual(rig.positions[i], 0)
        for cmd in rig.moves:
            f=cmd.split(',')
            self.assertEqual(f[1:3], ['4','MOVE'])
            self.assertTrue(0<int(f[3])<=97)
            self.assertEqual(int(f[4]), 96)
        self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
        rows=[c.kwargs for c in recorder.event.call_args_list if len(c.args)>1 and c.args[1]=='bench.recovery_leg_plan']
        self.assertEqual(rows[0]['reference_kind'], REFERENCE_KIND)

    def test_exception_does_not_relax_generic_parking_or_accept_changed_reference(self):
        with self.assertRaises(ValueError):
            RecoveryLeg(4,REBOOT_RAW,0,TRANSFERRED_ZERO).validate()
        for driver, reviewed, target, ref in (
            (2, REBOOT_RAW, 0, TRANSFERRED_ZERO),
            (4, {**REBOOT_RAW,4:1}, 0, TRANSFERRED_ZERO),
            (4, REBOOT_RAW, 1, TRANSFERRED_ZERO),
            (4, REBOOT_RAW, 0, ORIGINAL_ZERO),
        ):
            with self.assertRaises(ValueError):
                RestartRotaryReturn(driver,reviewed,target,ref).validate()

    def test_existing_fault_guards_abort_without_second_move(self):
        for failure in ('wrong_ack','fault','stall','missing','stale','overshoot','no_speed','held_drift'):
            clock=FakeClock(); rig=GaitRig(clock,failure,failure_driver=4); rig.positions=dict(REBOOT_RAW)
            with self.subTest(failure=failure),patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep), \
                    patch('builtins.print'),self.assertRaises(RuntimeError):
                run_return(rig,Mock(),None,confirmed=True)
                self.assertEqual(len(rig.moves), 1)

    def test_fresh_pose_mismatch_unconfirmed_and_stop_never_move(self):
        for reason in ('drift','unconfirmed','stop'):
            clock=FakeClock();rig=GaitRig(clock);rig.positions=dict(REBOOT_RAW)
            if reason=='drift':rig.positions[4]=1
            kwargs=dict(confirmed=reason!='unconfirmed')
            if reason=='stop':kwargs['stop_file']=Path(__file__)
            with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep),self.assertRaises((ValueError,RuntimeError)):
                run_return(rig,Mock(),None,**kwargs)
            self.assertFalse(rig.moves)
            self.assertFalse(any(',EN,1' in c for c in rig.commands))

    def test_default_is_offline_and_live_requires_confirmation(self):
        for live in (False,True):
            argv=['restart']+(['--execute','--port','COM4'] if live else [])
            with patch('sys.argv',argv),patch('scripts.x42s_20261027_epoch_return.execute_test') as execute, \
                    patch('builtins.print'),patch('sys.stderr'):
                if live:
                    with self.assertRaises(SystemExit):main()
                else:self.assertEqual(main(),0)
            execute.assert_not_called()


if __name__=='__main__':
    unittest.main()
