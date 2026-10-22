"""Digital twin tests exercise the real command/progress/terminal ledger."""
import math
import time
import unittest
from dataclasses import replace

from motor_control import AxisMotionTelemetry
from motor_control.gait_twin import ROLES, build_twin_snapshot
import test_gait_linkage as linkage


def linked_app():
    app = linkage.mechanism()
    app._hardware_estop_active = app._estop_in_progress = app._estop_unconfirmed = False
    return app


def x42s_axis(role, **overrides):
    """build_coordinated_snapshot 的 X42S 卡片形状（位置已换算输出轴）。"""
    card = {
        "role": role,
        "binding_valid": True,
        "state": "X42S_ONLINE",
        "position_source": "x42s_driver",
        "position_trusted": True,
        "position": 90.0,
        "target_position": 90.0,
        "stale": False,
        "continuous": False,
        "executed_steps": None,
    }
    card.update(overrides)
    return card


class X42sTwinSourceTests(unittest.TestCase):
    """2026-10-10 X42S 实测接入：来源标注为实测，在线态不判 untrusted。"""

    def test_online_x42s_axes_report_measured_source(self):
        params = linked_app().gait_params
        axes = [x42s_axis(role) for role in ROLES]
        snapshot = build_twin_snapshot(
            params, axes, context=None, calibration_valid=True
        )
        self.assertEqual(snapshot["source"], "x42s_measured")
        self.assertTrue(snapshot["measured"])
        # X42S_ONLINE 在白名单内：正常显示轴坐标，而不是 untrusted。
        self.assertEqual(snapshot["state"], "axis_only")
        self.assertIn("输出轴", snapshot["message"])

    def test_offline_x42s_axis_is_untrusted(self):
        params = linked_app().gait_params
        axes = [x42s_axis(role) for role in ROLES]
        axes[1] = x42s_axis(
            "Mr1", state="X42S_OFFLINE", position_trusted=False, position=None
        )
        snapshot = build_twin_snapshot(
            params, axes, context=None, calibration_valid=True
        )
        self.assertEqual(snapshot["state"], "untrusted")


class DigitalTwinTests(unittest.TestCase):
    def begin(self):
        app = linked_app()
        run, _ = app._gait_begin_run("left")
        return app, run

    def send_sync_without_terminal(self, app):
        def ack(command, **kwargs):
            app.commands.append(command)
            parts = command.split(",")
            return f"OK,SYNC,{parts[1]},{parts[3]}"
        app._send_and_read = ack
        self.assertEqual(app._gait_send_synchronized([(2, 180, 18), (3, 60, 6)], 18.75), "sent")

    def test_command_target_is_not_current_position(self):
        app, _ = self.begin()
        self.send_sync_without_terminal(app)
        snapshot = app._gait_twin_snapshot()
        self.assertFalse(snapshot["measured"])
        self.assertEqual(snapshot["state"], "waiting")
        self.assertEqual(snapshot["pose"]["phi_deg"], 0)
        self.assertAlmostEqual(snapshot["target_pose"]["phi_deg"], 60.075)
        self.assertAlmostEqual(snapshot["target_pose"]["feet"]["left"]["psi_deg"], 149.925)

    def test_real_progress_moves_world_pose_and_done_does_not_double_count(self):
        app, _ = self.begin()
        self.send_sync_without_terminal(app)
        original = tuple(r.position_steps for r in app.axis_runtime)
        app._on_step_progress(2, 400, 800)
        app._on_step_progress(3, 133, 267)
        snapshot = app._gait_twin_snapshot()
        pose = snapshot["pose"]
        self.assertAlmostEqual(pose["phi_deg"], 29.925)
        self.assertAlmostEqual(pose["beta_deg"], 150.075)
        self.assertAlmostEqual(pose["feet"]["left"]["psi_deg"], 90.075)
        self.assertAlmostEqual(pose["feet"]["right"]["psi_deg"], 30)
        x, y = pose["feet"]["left"]["center"]
        self.assertAlmostEqual(x, math.cos(math.radians(150.075)))
        self.assertAlmostEqual(y, -math.sin(math.radians(150.075)))  # 镜像
        self.assertEqual(original, tuple(r.position_steps for r in app.axis_runtime))
        app._on_step_done(2, 800, 800)
        app._on_step_done(3, 267, 267)
        done = app._gait_twin_snapshot()
        self.assertIsNone(done["target_pose"])
        self.assertAlmostEqual(done["pose"]["phi_deg"], 60.075)
        self.assertAlmostEqual(done["pose"]["feet"]["left"]["psi_deg"], 149.925)

    def test_lift_uses_real_progress_and_initial_axis_coordinate(self):
        app = linked_app()
        app.axis_runtime[0].position_steps = 1200
        app._gait_begin_run("left")
        baseline = app.axis_profiles[0].units_from_steps(1200)
        app._pending_step[0] = 800
        app.stepper_in_progress[0] = True
        app.axis_motion_telemetry[0] = AxisMotionTelemetry.starting(0, 800)
        app._on_step_progress(0, 200, 800)
        snapshot = app._gait_twin_snapshot()
        self.assertAlmostEqual(snapshot["axes"]["Mup1"]["position"], baseline + app.axis_profiles[0].units_from_steps(200))
        self.assertAlmostEqual(snapshot["pose"]["feet"]["left"]["z_mm"], app.axis_profiles[0].units_from_steps(200))

    def test_stale_disconnected_or_aborted_never_creates_fresh_pose(self):
        for fault in ("stale", "disconnect", "abort"):
            with self.subTest(fault=fault):
                app, _ = self.begin()
                self.send_sync_without_terminal(app)
                if fault == "stale":
                    app.axis_motion_telemetry[2] = replace(app.axis_motion_telemetry[2],
                                                         updated_monotonic=time.monotonic()-2)
                elif fault == "disconnect":
                    app._is_serial_connected = lambda: False
                else:
                    app._on_step_aborted(2, 40, 800)
                snapshot = app._gait_twin_snapshot()
                self.assertIsNone(snapshot["pose"])
                self.assertIsNone(snapshot["target_pose"])
                self.assertIn(snapshot["state"], ("stale", "untrusted"))

    def test_wrong_progress_is_ignored(self):
        app, _ = self.begin()
        self.send_sync_without_terminal(app)
        app._on_step_progress(2, 500, 999)
        self.assertEqual(app._gait_twin_snapshot()["pose"]["feet"]["left"]["psi_deg"], 30)

    def test_left_then_right_run_keeps_reference_frames_and_height_origin(self):
        app = linked_app()
        helper = linkage.PhysicalExecutionBridgeTests()
        helper.complete(app, "left")
        left = app._gait_twin_snapshot()["pose"]
        self.assertAlmostEqual(left["beta_deg"], 119.925)
        helper.complete(app, "right")
        right = app._gait_twin_snapshot()["pose"]
        self.assertAlmostEqual(right["beta_deg"], 59.85)
        self.assertAlmostEqual(right["feet"]["left"]["psi_deg"], left["feet"]["left"]["psi_deg"])
        self.assertEqual(right["feet"]["right"]["z_mm"], 0)

    def test_manual_move_after_gait_shows_axes_without_inventing_pivot(self):
        app = linked_app()
        linkage.PhysicalExecutionBridgeTests().complete(app, "left")
        app.axis_runtime[2].position_steps += 100
        snapshot = app._gait_twin_snapshot()
        self.assertEqual(snapshot["state"], "axis_only")
        self.assertIsNone(snapshot["pose"])
        self.assertIsNotNone(snapshot["axes"]["Mr1"]["position"])

    def test_rebinding_cannot_reinterpret_old_rotation_start(self):
        app, _ = self.begin()
        self.send_sync_without_terminal(app)
        app.axis_profiles[2] = replace(app.axis_profiles[2], pulse_per_rev=3200)
        snapshot = app._gait_twin_snapshot()
        self.assertIsNone(snapshot["pose"])
        self.assertEqual(snapshot["state"], "uncalibrated")

    def test_startup_and_manual_axis_values_work_without_a_run(self):
        app = linked_app()
        snapshot = app._gait_twin_snapshot()
        self.assertEqual(snapshot["state"], "axis_only")
        self.assertEqual(snapshot["axes"]["Mr1"]["position"], 0)
        self.assertIsNone(snapshot["pose"])


if __name__ == "__main__":
    unittest.main()
