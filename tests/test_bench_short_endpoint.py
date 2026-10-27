import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from scripts.x42s_20261027_epoch_return import TRANSFERRED_ZERO, REFERENCE_KIND, run_short_lift_trial
from scripts.x42s_suspended_gait import SuspendedHost, TOL
from motor_control.x42s_motion import ArrivalEvidence, MoveExpectation
from motor_control.protocol import parse_line
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock

START={1:0.,2:0.,3:0.,4:216.1}


class MicroSpeedRig(GaitRig):
    until = -1
    bad_speed = .1

    def request(self,command,**kwargs):
        result=super().request(command,**kwargs)
        if command=='X42S,S,1' and self.clock.now<self.until:
            result=replace(result,speed=self.bad_speed)
        return result


class ShortEndpointTests(unittest.TestCase):
    def make_host(self,clock,rig,recorder,short=True):
        rig.positions=dict(START)
        return SuspendedHost(rig,recorder,START,profile='interior',zero_reference=TRANSFERRED_ZERO,
            speed_scale=2,paired_progress_guard=True,reference_kind=REFERENCE_KIND,short_endpoint_checks=short)

    def test_small_out_and_back_two_sample_trial_preserves_endpoint_and_other_axes(self):
        clock=FakeClock();rig=GaitRig(clock);rig.positions=dict(START)
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep),patch('builtins.print'):
            result=run_short_lift_trial(rig,Mock(),None,START,confirmed=True)
        self.assertEqual(result['groups_completed'],20)
        self.assertEqual(result['full_gaits_completed'],0)
        for i in (1,3):
            self.assertAlmostEqual(rig.positions[i],TRANSFERRED_ZERO[i]+1800,delta=TOL[i])
        for i in (2,4):self.assertEqual(rig.positions[i],START[i])
        self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))

    def test_one_idle_lsb_defers_next_move_until_new_one_second_zero_window(self):
        clock=FakeClock();rig=MicroSpeedRig(clock);recorder=Mock()
        host=self.make_host(clock,rig,recorder)
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep):
            host.prepare()
            began=clock.now;rig.until=began+.4
            host.dispatch({1:TRANSFERRED_ZERO[1]+5.4*360},{1:2.5})
            self.assertGreaterEqual(clock.now-began,1.4)
            self.assertIsInstance(host.evidence[1],ArrivalEvidence)
        self.assertEqual(len(rig.moves),1)
        events=[c.args[1] for c in recorder.event.call_args_list if len(c.args)>1]
        self.assertIn('bench.idle_qualification_started',events)
        self.assertIn('bench.idle_qualified',events)

    def test_persistent_lsb_missing_speed_or_larger_speed_prevents_any_move(self):
        for speed in (.1,.2,None):
            clock=FakeClock();rig=MicroSpeedRig(clock);host=self.make_host(clock,rig,Mock())
            with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep):
                host.prepare();rig.until=clock.now+100;rig.bad_speed=speed
                with self.assertRaises(RuntimeError):host.dispatch({1:TRANSFERRED_ZERO[1]+5.4*360},{1:2.5})
            self.assertFalse(rig.moves)

    def test_lsb_on_held_axis_during_other_axis_motion_still_aborts_immediately(self):
        clock=FakeClock();rig=MicroSpeedRig(clock);host=self.make_host(clock,rig,Mock())
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep):
            host.prepare();host.dispatch({2:10.7},{2:.5});rig.until=clock.now+100
            with self.assertRaises(RuntimeError):host.wait_group((2,),10)
        self.assertEqual(len(rig.moves),1)

    def test_original_strict_mode_and_position_tolerance_are_not_relaxed(self):
        for short,drift in ((False,0),(True,1)):
            clock=FakeClock();rig=MicroSpeedRig(clock);host=self.make_host(clock,rig,Mock(),short=short)
            with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep):
                host.prepare();rig.until=clock.now+.4;rig.positions[1]+=drift
                with self.assertRaises(RuntimeError):host.dispatch({1:TRANSFERRED_ZERO[1]+5.4*360},{1:2.5})
            self.assertFalse(rig.moves)

    def test_faults_in_fast_trial_never_release_later_move(self):
        for failure in ('wrong_ack','fault','missing','stale','overshoot','stall','no_speed','held_drift'):
            clock=FakeClock();rig=GaitRig(clock,failure);rig.positions=dict(START)
            with self.subTest(failure=failure),patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep), \
                    patch('builtins.print'),self.assertRaises(RuntimeError):
                run_short_lift_trial(rig,Mock(),None,START,confirmed=True)
            self.assertLessEqual(len(rig.moves),2)

    def test_endpoint_handoff_is_one_shot_and_expires_with_host_elapsed_time(self):
        for delay in (0,.31):
            clock=FakeClock();rig=GaitRig(clock);recorder=Mock();host=self.make_host(clock,rig,recorder)
            with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep):
                host.prepare();host.dispatch({1:TRANSFERRED_ZERO[1]+5.4*360},{1:2.5})
                host.wait_group((1,),10)
                self.assertIsNotNone(host._ready_group_samples)
                before=len(rig.commands);clock.sleep(delay)
                host.dispatch_snapshot()
                self.assertEqual(len(rig.commands)==before,delay==0)
                self.assertIsNone(host._ready_group_samples)
                before=len(rig.commands);host.dispatch_snapshot()
                self.assertGreater(len(rig.commands),before)

    def test_endpoint_handoff_invalid_speed_or_intervening_read_is_not_reused(self):
        clock=FakeClock();rig=GaitRig(clock);recorder=Mock();host=self.make_host(clock,rig,recorder)
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep):
            host.prepare();host.dispatch({1:TRANSFERRED_ZERO[1]+5.4*360},{1:2.5});host.wait_group((1,),10)
            samples=dict(host._ready_group_samples);r,at=samples[1];samples[1]=(replace(r,speed=None),at)
            host._ready_group_samples=samples
            before=len(rig.commands);host.dispatch_snapshot();self.assertGreater(len(rig.commands),before)
            host._ready_group_samples=samples;host.snapshot();self.assertIsNone(host._ready_group_samples)

    def test_recorded_s4_motor_half_degree_excursion_still_aborts(self):
        # 20261027T014056-093e2e55b8, S4 group31: ACK target -297.8,
        # actual -297.3. This is NOT the idle one-LSB speed case.
        class RecordedRig(GaitRig):
            inject=False
            def request(self,command,**kwargs):
                if self.inject and command=='X42S,S,2':
                    self.commands.append(command)
                    return parse_line('X42S,2,S,1,1,1,none,-297.800,-297.300,-0.540,0.000,0.000,29.000,85,1160780,63')
                return super().request(command,**kwargs)
        clock=FakeClock();rig=RecordedRig(clock);host=self.make_host(clock,rig,Mock())
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep):
            host.prepare()
            host.pending[2]=MoveExpectation(host,-297.8,clock.now,origin_motor_deg=-343.,tolerance_motor_deg=TOL[2])
            rig.inject=True
            with self.assertRaisesRegex(RuntimeError,'ID2 outside commanded travel'):
                host.snapshot()
        self.assertEqual(TOL[2],.225)
        self.assertFalse(rig.moves)


if __name__=='__main__':unittest.main()
