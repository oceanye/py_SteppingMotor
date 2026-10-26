import unittest
from dataclasses import replace

from scripts.gait_full_simulation_check import check_step, audit
from motor_control.gait_planner import default_gait_params
from motor_control.gait_simulation import GaitSimulation


class FullSimulationTests(unittest.TestCase):
    def test_four_full_steps_on_current_geometry_and_both_installation_signs(self):
        for sign in (1, -1):
            p = replace(default_gait_params(), mr1_sign=sign, mr2_sign=-sign)
            for action in ('L+', 'L-', 'R+', 'R-'):
                with self.subTest(sign=sign, action=action):
                    sim = GaitSimulation(p)
                    row = check_step(sim, action)
                    self.assertEqual(len(row['stage_sequence']), 9)
                    self.assertEqual(sim.completed_steps, 1)
                    self.assertGreater(row['x42s_rectangle_lower_bound_mm'], 0)

    def test_current_profile_24_step_round_trip_and_four_curve_exports(self):
        result, rows = audit(default_gait_params())
        self.assertFalse(result['hardware_motion'])
        self.assertEqual(result['summary']['failures'], 0)
        self.assertEqual(result['summary']['legal_actions_passed'], 4)
        self.assertEqual(result['summary']['cumulative_steps'], 24)
        self.assertEqual(result['summary']['cumulative_winding_peak_deg'], 180)
        self.assertEqual({r['action'] for r in rows}, {'L+', 'L-', 'R+', 'R-'})
        self.assertTrue(all(r['source'] == 'offline_model_not_feedback' for r in rows))


if __name__ == '__main__':
    unittest.main()
