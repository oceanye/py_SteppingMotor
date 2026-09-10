import unittest

from motor_control.gait_executor import (
    GaitExecutor,
    GaitExecutorError,
    GROUP_TIMEOUT_MARGIN_S,
)
from motor_control.gait_planner import (
    GaitParams,
    GaitGeometry,
    plan_gait_stages,
)


class FakeHost:
    """记录事件序列的可编程宿主：验证组间顺序/组内并列等契约。"""

    def __init__(self):
        self.events = []            # ("send"/"wait"/"stop", axis, extra)
        self.role_axes = {"Mr1": 2, "Mr2": 3, "Mup1": 0, "Mup2": 1}
        self.wait_results = []      # 每次 wait 的返回值队列
        self.cancelled_flag = False
        self.fail_send_axes = set()
        self.noop_axes = set()

    def role_axis(self, role_name):
        axis = self.role_axes.get(role_name)
        if axis is None:
            raise GaitExecutorError(f"{role_name} 未绑定")
        return axis

    def send_relative(self, axis, delta, speed):
        self.events.append(("send", axis, (delta, speed)))
        if axis in self.fail_send_axes:
            return "failed"
        if axis in self.noop_axes:
            return "noop"
        return "sent"

    def wait_terminal(self, axis, timeout_s):
        self.events.append(("wait", axis, timeout_s))
        return self.wait_results.pop(0) if self.wait_results else "DONE"

    def stop_axes(self, axes):
        self.events.append(("stop", tuple(sorted(set(axes))), None))

    def cancelled(self):
        return self.cancelled_flag

    def log(self, message):
        self.events.append(("log", None, message))

    def sends(self):
        return [event for event in self.events if event[0] == "send"]

    def waits(self):
        return [event for event in self.events if event[0] == "wait"]


def _params():
    return GaitParams(
        geometry=GaitGeometry(d_mm=220.0),
        swing_segments=3,
    ).validated()


def _left_stages(params, psi_start=30.0):
    return plan_gait_stages(params, side="left", swing_psi_start_deg=psi_start)


class HappyPathTests(unittest.TestCase):
    def test_full_left_run_sequence(self):
        host = FakeHost()
        params = _params()
        run = GaitExecutor(host, params, _left_stages(params), side="left")
        by_id = {stage.stage_id: stage for stage in run.stages}

        # S0/S1：确认型，只推进不产生运动
        self.assertTrue(run.advance_confirm())
        self.assertTrue(run.advance_confirm())
        self.assertEqual(host.sends(), [])

        # S2 抬足：单组单轴（Mup1 抬起）
        self.assertTrue(run.execute_current_stage())
        sends = host.sends()
        self.assertEqual(len(sends), 1)
        axis, (delta, speed) = sends[0][1], sends[0][2]
        self.assertEqual(host.role_axes["Mup1"], axis)
        self.assertAlmostEqual(delta, params.lift_mm)
        self.assertAlmostEqual(speed, params.lift_speed_mm_s)
        self.assertEqual(run.stage_index, 3)

        # S3 在基准相位时被省略
        self.assertNotIn("S3", by_id)

        # S4：3 组，每组先全部下发再等待（组内并列、组间顺序）
        s4_wait_start = len(host.events)
        self.assertTrue(run.execute_current_stage())
        group_events = host.events[s4_wait_start:]
        send_indices = [i for i, e in enumerate(group_events) if e[0] == "send"]
        wait_indices = [i for i, e in enumerate(group_events) if e[0] == "wait"]
        self.assertEqual(len(send_indices), 6)   # 3 组 × 摆动+支撑
        self.assertEqual(len(wait_indices), 6)
        # 第 1 组的两条 wait 都在第 2 组的两条 send 之前
        self.assertLess(max(wait_indices[:2]), min(send_indices[2:]))
        # 摆动侧总量 +60°（θ=0 解绕小步，sign=+1），支撑侧 +60°（sign=+1）
        swing_total = sum(e[2][0] for e in group_events
                          if e[0] == "send" and e[1] == host.role_axes["Mr1"])
        support_total = sum(e[2][0] for e in group_events
                            if e[0] == "send" and e[1] == host.role_axes["Mr2"])
        self.assertAlmostEqual(swing_total, 60.0, places=6)
        self.assertAlmostEqual(support_total, 60.0, places=6)

        # S5 确认、S6 落脚（速度为 settle）、S7 确认 → done
        self.assertTrue(run.advance_confirm())
        self.assertTrue(run.execute_current_stage())
        settle = [e for e in host.sends()
                  if e[1] == host.role_axes["Mup1"]][-1]
        self.assertAlmostEqual(settle[2][0], -params.lift_mm)
        self.assertAlmostEqual(settle[2][1], params.settle_speed_mm_s)
        self.assertTrue(run.advance_confirm())
        self.assertEqual(run.state, "done")
        self.assertIsNone(run.current_stage())

    def test_group_timeout_scales_with_expected_duration(self):
        host = FakeHost()
        params = _params()
        run = GaitExecutor(host, params, _left_stages(params), side="left")
        run.advance_confirm()
        run.advance_confirm()
        run.execute_current_stage()          # S2
        run.execute_current_stage()          # S4 第一组即可观察超时
        timeout = host.waits()[0][2]
        longest_delta = max(abs(e[2][0]) for e in host.sends()
                            if e[1] in (host.role_axes["Mr1"],
                                        host.role_axes["Mr2"]))
        expected_min = longest_delta / params.swing_side_speed_deg_s
        self.assertGreaterEqual(timeout, 3.0 * expected_min)
        self.assertGreaterEqual(timeout, GROUP_TIMEOUT_MARGIN_S)


class FailureTests(unittest.TestCase):
    def _run_until_s4(self, host, params):
        run = GaitExecutor(host, params, _left_stages(params), side="left")
        run.advance_confirm()
        run.advance_confirm()
        run.execute_current_stage()   # S2
        return run

    def test_dispatch_failure_stops_axes_and_blocks(self):
        host = FakeHost()
        params = _params()
        host.fail_send_axes.add(host.role_axes["Mr2"])
        run = self._run_until_s4(host, params)
        self.assertFalse(run.execute_current_stage())
        self.assertEqual(run.state, "failed")
        stops = [e for e in host.events if e[0] == "stop"]
        self.assertTrue(stops)
        self.assertIn(host.role_axes["Mr2"], stops[0][1])
        with self.assertRaises(GaitExecutorError):
            run.execute_current_stage()

    def test_abort_terminal_marks_aborted(self):
        host = FakeHost()
        params = _params()
        run = self._run_until_s4(host, params)
        host.wait_results = ["DONE", "ABORTED"]
        self.assertFalse(run.execute_current_stage())
        self.assertEqual(run.state, "aborted")
        self.assertIn("不可信", run.last_error)

    def test_timeout_marks_failed(self):
        host = FakeHost()
        params = _params()
        run = self._run_until_s4(host, params)
        host.wait_results = ["TIMEOUT"]
        self.assertFalse(run.execute_current_stage())
        self.assertEqual(run.state, "failed")
        self.assertTrue([e for e in host.events if e[0] == "stop"])

    def test_request_stop_between_groups_aborts(self):
        host = FakeHost()
        params = _params()
        run = self._run_until_s4(host, params)
        original_wait = host.wait_terminal

        def wait_then_stop(axis, timeout_s):
            result = original_wait(axis, timeout_s)
            run.request_stop()   # 第一组完成后用户按中止
            return result

        host.wait_terminal = wait_then_stop
        waits_before_s4 = len(host.waits())
        self.assertFalse(run.execute_current_stage())
        self.assertEqual(run.state, "aborted")
        self.assertTrue([e for e in host.events if e[0] == "stop"])
        # S4 只执行了第一组（2 次等待），后续组不再下发
        self.assertEqual(len(host.waits()) - waits_before_s4, 2)

    def test_cancelled_host_aborts_before_dispatch(self):
        host = FakeHost()
        params = _params()
        run = self._run_until_s4(host, params)
        host.cancelled_flag = True
        sends_before = len(host.sends())
        self.assertFalse(run.execute_current_stage())
        self.assertEqual(run.state, "aborted")
        self.assertEqual(len(host.sends()), sends_before)

    def test_noop_move_skips_wait(self):
        host = FakeHost()
        params = _params()
        host.noop_axes.add(host.role_axes["Mr2"])   # 支撑侧每段都不足 1 脉冲
        run = self._run_until_s4(host, params)
        self.assertTrue(run.execute_current_stage())
        waits = {e[1] for e in host.waits()}
        self.assertNotIn(host.role_axes["Mr2"], waits)

    def test_missing_role_binding_fails_closed(self):
        host = FakeHost()
        params = _params()
        del host.role_axes["Mr2"]
        run = self._run_until_s4(host, params)
        self.assertFalse(run.execute_current_stage())
        self.assertEqual(run.state, "failed")
        self.assertIn("Mr2", run.last_error)


class StageGateTests(unittest.TestCase):
    def test_confirm_advance_rejected_for_motion_stage(self):
        host = FakeHost()
        params = _params()
        run = GaitExecutor(host, params, _left_stages(params), side="left")
        run.advance_confirm()
        run.advance_confirm()
        stage = run.current_stage()
        self.assertEqual(stage.stage_id, "S2")
        self.assertTrue(stage.is_motion_stage)
        self.assertFalse(run.advance_confirm())

    def test_execute_rejected_for_confirm_stage(self):
        host = FakeHost()
        params = _params()
        run = GaitExecutor(host, params, _left_stages(params), side="left")
        with self.assertRaises(GaitExecutorError):
            run.execute_current_stage()   # S0 是确认型

    def test_describe_snapshot(self):
        host = FakeHost()
        params = _params()
        run = GaitExecutor(host, params, _left_stages(params), side="left")
        snapshot = run.describe()
        self.assertEqual(snapshot["side"], "left")
        self.assertEqual(snapshot["state"], "ready")
        self.assertEqual(snapshot["stage_id"], "S0")
        self.assertFalse(snapshot["is_motion_stage"])
        self.assertEqual(snapshot["stage_count"], len(run.stages))
        run.advance_confirm()
        self.assertEqual(run.describe()["stage_id"], "S1")


if __name__ == "__main__":
    unittest.main()
