"""World-map and multi-step twin checks; no serial, GUI or physical motors."""
import copy
import math
import os
import sys
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_gait_twin import linked_app
from motor_control.gait_map import map_pads, map_label, start_pair_reference, in_map
from motor_control.gait_twin import TwinHistory, pad_center
from motor_control.gait_planner import GaitParams, GaitGeometry, parse_gait_params, gait_pads
from motor_control.gait_avoidance import TWO_MODE, lattice_coordinates
from motor_control.gait_executor import GaitExecutorError
from motor_control.state_store import StateStoreError


class MapGeometryTests(unittest.TestCase):
    def test_exactly_five_offset_rows_and_columns_same_world_frame(self):
        pads = map_pads()
        self.assertEqual(len(pads), 25)
        self.assertEqual(len({map_label(n) for n in pads}), 25)
        self.assertEqual(pads['B'], (0, 0))
        for row in range(-2, 3):
            names = [n for n in pads if lattice_coordinates(n)[1] == row]
            self.assertEqual(len(names), 5)
            self.assertEqual(sorted(lattice_coordinates(n)[0] + row//2 for n in names), list(range(-2, 3)))
        for name, xy in pads.items():
            self.assertTrue(in_map(name))
            self.assertEqual(pad_center(name), xy)

    def test_all_ordered_neighbor_pairs_roundtrip_in_six_orientations(self):
        pads = map_pads()
        angles = set()
        for left in pads:
            for right in pads:
                if not math.isclose(math.dist(pads[left], pads[right]), 1):
                    with self.assertRaises(ValueError):
                        start_pair_reference((left, right))
                    continue
                beta, placement = start_pair_reference((left, right))
                params = GaitParams(initial_pad_pair=(left, right),
                                    initial_placement=placement, beam_reference_deg=beta).validated()
                self.assertEqual(parse_gait_params(params.as_document()), params)
                self.assertEqual(params.initial_supports, (left, right))
                self.assertAlmostEqual(params.initial_beam_deg, beta)
                angles.add(beta)
                dx, dy = pads[left][0] - pads[right][0], pads[left][1] - pads[right][1]
                self.assertAlmostEqual(math.cos(math.radians(beta)), dx)
                self.assertAlmostEqual(math.sin(math.radians(beta)), dy)
        self.assertEqual(angles, {0, 60, 120, 180, 240, 300})

    def test_bad_config_never_silently_falls_back_to_ab(self):
        doc = GaitParams().as_document()
        for pair in ('AB', [], ['A'], ['A', 'A'], ['B', '邻座(99,0)'], [None, 'B'],
                     ['邻座(0,0)', 'A']):
            with self.subTest(pair=pair), self.assertRaises(ValueError):
                parse_gait_params(dict(doc, initial_pad_pair=pair))
        with self.assertRaises(ValueError):
            parse_gait_params(dict(doc, initial_pad_pair=['B', 'A']))

    def test_old_configs_keep_ab_and_obstacles_extend_past_map_edge(self):
        p = parse_gait_params({'schema': GaitParams.SCHEMA})
        self.assertEqual(p.initial_supports, ('A', 'B'))
        self.assertIsNone(p.initial_pad_pair)
        edge = next(n for n in map_pads() if lattice_coordinates(n)[1] == 2)
        obstacles = gait_pads(replace(p, trajectory_mode=TWO_MODE), edge)
        self.assertTrue(any(not in_map(n) for n in obstacles))


class StartReferenceTests(unittest.TestCase):
    def app(self):
        app = linked_app()
        app.saved = []
        app.state_store = SimpleNamespace(save_gait_params=app.saved.append)
        return app

    def test_apply_changes_world_not_motor_coordinates_and_requires_recalibration(self):
        app = self.app()
        app.axis_runtime[0].position_steps = 712
        before = [r.position_steps for r in app.axis_runtime]
        app._gait_apply_start_pair('C', 'A')
        self.assertEqual(app._gait_supports, ('C', 'A'))
        self.assertEqual(app._gait_beta_deg, 60)
        self.assertEqual(before, [r.position_steps for r in app.axis_runtime])
        self.assertEqual(app.commands, [])
        self.assertEqual(len(app.saved), 1)
        self.assertIsNone(app.gait_params.mr1_zero_deg)
        self.assertIsNone(app.gait_params.mr2_zero_signature)
        self.assertFalse(app.gait_params.calibration_confirmed)
        self.assertEqual(app._gait_world_epoch, 1)
        with self.assertRaisesRegex(GaitExecutorError, '尚未确认'):
            app._gait_begin_run('left')

    def test_owned_pending_manual_and_release_all_block_start_edit(self):
        for fault in ('owned', 'moving', 'reservation', 'release', 'stop_pending'):
            with self.subTest(fault=fault):
                app = self.app()
                if fault == 'owned':
                    app._gait_owned = {0: object()}
                elif fault == 'moving':
                    app.running[0] = True
                elif fault == 'reservation':
                    app._move_reservation[2] = object()
                elif fault == 'release':
                    app._rot_release_active = True
                else:
                    app._estop_unconfirmed = True
                original = app.gait_params
                with self.assertRaises(GaitExecutorError):
                    app._gait_apply_start_pair('C', 'A')
                self.assertIs(app.gait_params, original)
                self.assertEqual(app._gait_supports, ('A', 'B'))
                self.assertEqual(app.saved, [])

    def test_save_failure_leaves_pose_and_params_unchanged(self):
        app = self.app()
        def fail(_):
            raise StateStoreError('disk unavailable')
        app.state_store.save_gait_params = fail
        original = app.gait_params
        with self.assertRaises(StateStoreError):
            app._gait_apply_start_pair('C', 'A')
        self.assertIs(app.gait_params, original)
        self.assertEqual(app._gait_supports, ('A', 'B'))

    def test_rebase_does_not_restore_untrusted_motor(self):
        app = self.app()
        app.axis_runtime[0] = replace(app.axis_runtime[0], position_trusted=False)
        app._gait_apply_start_pair('C', 'A')
        self.assertFalse(app.axis_runtime[0].position_trusted)

    def test_startup_retains_selected_pair_but_demands_confirmation(self):
        app = self.app()
        app._gait_apply_start_pair('C', 'A')
        app.state_store.load_gait_params = lambda: dict(app.saved[-1], calibration_confirmed=True)
        app._startup_warnings = []
        app._load_gait_params()
        self.assertEqual(app.gait_params.initial_supports, ('C', 'A'))
        self.assertFalse(app.gait_params.calibration_confirmed)


def sample(ref=1, x=0, done=False, state='estimated', world='w'):
    return {'reference_key': ref, 'world_key': world, 'step_done': done,
            'state': state, 'target_pose': {'fake': 'must never be recorded'},
            'pose': {'feet': {'left': {'center': (x, 0)}, 'right': {'center': (x+1, 0)}}}}


class HistoryTests(unittest.TestCase):
    def test_multiple_steps_keep_paths_and_count_done_once(self):
        h = TwinHistory()
        for ref in (1, 2, 3):
            h.observe(sample(ref, ref))
            h.observe(sample(ref, ref+.2, True))
            h.observe(sample(ref, ref+.2, True))
        self.assertEqual(h.completed_steps, 3)
        self.assertEqual(len(h.points), 6)
        self.assertEqual(len(h.segments()), 3)

    def test_clear_does_not_recount_completed_run_or_modify_input(self):
        h = TwinHistory()
        snap = sample(done=True)
        original = copy.deepcopy(snap)
        h.observe(snap)
        h.clear(snap)
        self.assertFalse(h.points)
        h.observe(snap)
        self.assertEqual(h.completed_steps, 0)
        self.assertEqual(len(h.points), 1)
        self.assertEqual(snap, original)

    def test_new_world_clears_but_invalid_progress_only_breaks_line(self):
        h = TwinHistory()
        h.observe(sample())
        h.observe(dict(sample(state='stale'), pose=None))
        h.observe(sample(x=.5))
        self.assertEqual(len(h.points), 2)
        self.assertEqual(len(h.segments()), 2)
        h.observe(sample(world='new'))
        self.assertEqual(len(h.points), 1)

    def test_invalid_states_never_add_positions_and_history_is_bounded(self):
        h = TwinHistory(max_points=5)
        for state in ('untrusted', 'uncalibrated', 'axis_only', 'stale'):
            h.observe(sample(state=state))
        self.assertFalse(h.points)
        for i in range(25):
            h.observe(sample(x=i))
        self.assertEqual(len(h.points), 5)
        self.assertEqual(h.points[0][1][0], 20)

    def test_points_carry_world_psi_for_tip_tracks(self):
        # 2026-09-30 实机轨迹要与评估层同款（中心+三爪端折线）：历史点
        # 必须带上世界 ψ；旧测试替身不带 psi_deg 时容忍为 None。
        h = TwinHistory()
        snap = sample()
        snap['pose']['feet']['left']['psi_deg'] = 30.0
        snap['pose']['feet']['right']['psi_deg'] = 57.5
        h.observe(snap)
        self.assertEqual(h.points[-1][4], (30.0, 57.5))
        h.observe(sample(x=.5))
        self.assertEqual(h.points[-1][4], (None, None))


class MapExecutionTests(unittest.TestCase):
    def app(self):
        app = linked_app()
        app.gait_params = replace(app.gait_params, trajectory_mode=TWO_MODE,
            geometry=GaitGeometry(arm_length_mm=40, arm_radius_mm=.2,
                                 node_radius_mm=.2, safety_margin_mm=.1))
        return app

    def complete(self, app, side, arc, history):
        run, report = app._gait_begin_run(side, arc_deg=arc)
        self.assertTrue(report.feasible)
        history.observe(app._gait_twin_snapshot())
        with patch('motor_control.desktop_app.messagebox.askokcancel', return_value=True):
            while run.current_stage() is not None:
                if run.current_stage().is_motion_stage:
                    self.assertTrue(run.execute_current_stage())
                else:
                    app._gait_stage_confirmed()
                history.observe(app._gait_twin_snapshot())
        self.assertEqual(run.state, 'done')
        return report.modality

    def test_four_steps_accumulate_and_fifth_is_blocked_before_any_command(self):
        app, history = self.app(), TwinHistory()
        modes = [self.complete(app, side, arc, history)
                 for side, arc in [('left', 60), ('right', -60)] * 2]
        self.assertEqual(len(set(modes)), 2)
        self.assertEqual(history.completed_steps, 4)
        pose = app._gait_twin_snapshot()['pose']
        self.assertAlmostEqual(pose['feet']['left']['center'][1], math.sqrt(3))
        self.assertGreater(len(history.points), 4)
        before = list(app.commands)
        with self.assertRaisesRegex(GaitExecutorError, '5×5'):
            app._gait_begin_run('left', arc_deg=60)
        self.assertEqual(app.commands, before)
        self.assertIsNone(app._gait_landing_route('left', app.gait_params, arc_deg=60))
        # Boundary is not a dead end: inverse of the last move is still legal.
        self.complete(app, 'right', 60, history)
        self.assertEqual(history.completed_steps, 5)
        # Revoking calibration freezes history; it is not a physical rebase.
        before_points = list(history.points)
        app.gait_params = replace(app.gait_params, calibration_confirmed=False,
                                  calibration_fingerprint=None)
        history.observe(app._gait_twin_snapshot())
        self.assertEqual(list(history.points), before_points)
        self.assertEqual(history.completed_steps, 5)

    def test_rotated_start_uses_the_same_world_pose_and_mode_classifier(self):
        app, history = self.app(), TwinHistory()
        beta, placement = start_pair_reference(('C', 'A'))
        app.gait_params = replace(app.gait_params, initial_pad_pair=('C', 'A'),
                                 initial_placement=placement, beam_reference_deg=beta)
        app._gait_supports, app._gait_beta_deg = ('C', 'A'), beta
        self.complete(app, 'left', 60, history)
        self.assertEqual(app._gait_supports, ('B', 'A'))
        self.assertAlmostEqual(app._gait_twin_snapshot()['pose']['feet']['right']['center'][0], -1)


if __name__ == '__main__':
    unittest.main()
