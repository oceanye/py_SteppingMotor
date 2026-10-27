import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from motor_control.gait_planner import default_gait_params
from scripts.x42s_four_gait_sequence import build_sequence, run_sequence, validate_start, main, ACTIONS
from scripts.x42s_suspended_gait import SuspendedHost, SCALES, TOL
from test_bench_manual_park import REFERENCE
from test_bench_suspended_gait import GaitRig
from test_gait_realtime_fixes import FakeClock

START = {1: 31.9, 2: 436.3, 3: -21446.801, 4: -1995.1}


class FourGaitSequenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plans = build_sequence(default_gait_params())

    def test_dynamic_routes_both_modes_and_original_frame_are_preserved(self):
        plans = self.plans
        self.assertEqual(tuple(s.action for s in plans), ACTIONS)
        self.assertEqual([s.report.modality for s in plans], ['high_junction']*2+['low_junction']*2)
        self.assertEqual(plans[0].supports_before, plans[-1].supports_after)
        self.assertEqual(plans[-1].end_rotary, {2: 0, 4: 0})
        for previous, following in zip(plans, plans[1:]):
            self.assertEqual(previous.supports_after, following.supports_before)
            self.assertEqual(previous.end_rotary, following.start_rotary)
        for step in plans:
            self.assertEqual(step.params.lift_mm, 60)
            heights = {'Mup1': 5., 'Mup2': 5.}
            for stage in step.stages:
                for group in stage.move_groups:
                    for m in group:
                        if m.role.startswith('Mr'):
                            self.assertLessEqual(abs(m.delta), 4+1e-8)
                        else:
                            heights[m.role] = round(heights[m.role]+m.delta, 8)
                            self.assertTrue(5 <= heights[m.role] <= 65)
                            if len(group)==2:self.assertLessEqual(abs(m.delta), .4+1e-8)
            self.assertEqual(heights, {'Mup1': 5., 'Mup2': 5.})

    def test_full_fake_feedback_continuous_sequence_no_rezero(self):
        clock = FakeClock(); rig = GaitRig(clock); rig.positions = dict(START)
        recorder = Mock()
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), patch('builtins.print'):
            result = run_sequence(rig, recorder, None, self.plans, START, REFERENCE, confirmed=True)
        self.assertEqual(result['completed_actions'], list(ACTIONS))
        self.assertEqual(result['groups_completed'], sum(len(s.move_groups) for p in self.plans for s in p.stages))
        self.assertFalse(result['standard_70mm_acceptance'])
        self.assertFalse(result['field_contact_avoidance_verified'])
        for i, pos in result['raw_motor_deg'].items():
            self.assertAlmostEqual(pos, REFERENCE[i]+(1800 if i in (1,3) else 0), delta=TOL[i])
        self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
        self.assertEqual(len([c for c in rig.commands if ',EN,1' in c]), 4)
        envelopes=[c.kwargs for c in recorder.event.call_args_list if len(c.args)>1 and c.args[1]=='bench.gait_pair_envelope']
        self.assertTrue(envelopes)
        self.assertTrue(all(e['maximum_independent_spread_mm']<=.5 for e in envelopes))

    def test_failure_aborts_without_next_step_or_zero_reset(self):
        for failure in ('wrong_ack', 'overshoot', 'held_drift'):
            clock = FakeClock(); rig = GaitRig(clock,failure); rig.positions = dict(START)
            recorder = Mock()
            with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), \
                    patch('builtins.print'), self.assertRaises(RuntimeError):
                run_sequence(rig, recorder, None, self.plans, START, REFERENCE, confirmed=True)
            self.assertLessEqual(len(rig.moves), 2)
            self.assertFalse(any(len(c.args)>1 and c.args[1]=='bench.sequence_step_completed' for c in recorder.event.call_args_list))

    def test_strict_original_start_does_not_admit_manual_touch_or_rezero(self):
        validate_start(START,REFERENCE)
        for wrong in ({**START, 4: START[4]+1}, {**START, 1: START[1]+2}, {1:0}, {**START,3:float('nan')}):
            with self.assertRaises(ValueError): validate_start(wrong,REFERENCE)
        with patch('sys.argv',['sequence','--execute','--port','COM4']), \
                patch('scripts.x42s_four_gait_sequence.execute_test') as execute, patch('builtins.print'), \
                patch('sys.stderr'), self.assertRaises(SystemExit):main()
        execute.assert_not_called()

    def test_reviewed_restart_sequence_keeps_epoch_provenance_and_physical_bounds(self):
        from scripts.x42s_20261027_epoch_return import TRANSFERRED_ZERO, provenance, REFERENCE_KIND
        start = {1:0.,2:0.,3:0.,4:216.1}
        context = dict(provenance(), rotary_alignment_visually_confirmed=True)
        clock=FakeClock();rig=GaitRig(clock);rig.positions=dict(start)
        recorder=Mock()
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep),patch('builtins.print'):
            result=run_sequence(rig,recorder,None,self.plans,start,TRANSFERRED_ZERO,confirmed=True,
                                reference_provenance=context)
        self.assertEqual(result['completed_actions'],list(ACTIONS))
        self.assertEqual(result['reference_kind'],REFERENCE_KIND)
        self.assertFalse(result['reference_provenance']['absolute_position_independently_verified'])
        for i,raw in result['raw_motor_deg'].items():
            self.assertAlmostEqual(raw,TRANSFERRED_ZERO[i]+(1800 if i in (1,3) else 0),delta=TOL[i])
        self.assertFalse(any('ZERO' in c or 'RAW' in c for c in rig.commands))
        for wrong in (dict(context,rotary_alignment_visually_confirmed=False),
                      dict(context,inferred_zero_raw_motor_deg=REFERENCE)):
            session=Mock()
            with self.assertRaises(ValueError):
                run_sequence(session,Mock(),None,self.plans,start,TRANSFERRED_ZERO,confirmed=True,
                             reference_provenance=wrong)
            session.request.assert_not_called();session.request_line.assert_not_called()

    def test_pair_envelope_is_enforced_before_first_move(self):
        clock=FakeClock();rig=GaitRig(clock);rig.positions=dict(START)
        host=SuspendedHost(rig,Mock(),START,profile='interior',zero_reference=REFERENCE,
                           speed_scale=2,rotary_segment_deg=4,paired_progress_guard=True)
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep):
            host.prepare()
            with self.assertRaisesRegex(RuntimeError,'rectangle'):
                host.dispatch({i:REFERENCE[i]+7*360 for i in (1,3)},{1:2.5,3:2.5})
        self.assertFalse(rig.moves)

    def test_no_extra_dwell_keeps_four_actions_and_qualifies_each_stage_boundary(self):
        clock=FakeClock();rig=GaitRig(clock);rig.positions=dict(START);recorder=Mock()
        with patch('time.monotonic',clock.monotonic),patch('time.sleep',clock.sleep),patch('builtins.print'):
            result=run_sequence(rig,recorder,None,self.plans,START,REFERENCE,confirmed=True,short_endpoint_checks=True)
        self.assertEqual(result['completed_actions'],list(ACTIONS))
        self.assertTrue(result['short_endpoint_checks'])
        boundaries=[c.kwargs for c in recorder.event.call_args_list
                    if len(c.args)>1 and c.args[1]=='bench.idle_qualified' and c.kwargs['reason']=='stage_boundary']
        self.assertEqual(len(boundaries),sum(s.is_motion_stage for p in self.plans for s in p.stages))
        self.assertTrue(all(r['duration_s']>=1 for r in boundaries))
        for i,raw in result['raw_motor_deg'].items():
            self.assertAlmostEqual(raw,REFERENCE[i]+(1800 if i in (1,3) else 0),delta=TOL[i])


if __name__ == '__main__':
    unittest.main()
