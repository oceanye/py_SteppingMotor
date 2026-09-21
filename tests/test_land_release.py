"""落地纠偏：悬空腿落地（标高差收敛）自动释放/重锁旋转电机。"""

import sys
import threading
import types
import unittest


def _ensure_tkinter_import_surface():
    try:
        import tkinter  # noqa: F401
        return
    except ImportError:
        pass
    tkinter_module = types.ModuleType("tkinter")
    tkinter_module.__path__ = []

    class TclError(Exception):
        pass

    tkinter_module.TclError = TclError
    ttk_module = types.ModuleType("tkinter.ttk")
    messagebox_module = types.ModuleType("tkinter.messagebox")
    for name in ("showerror", "showwarning", "showinfo", "askyesno", "askokcancel"):
        setattr(messagebox_module, name, lambda *_args, **_kwargs: None)
    tkinter_module.ttk = ttk_module
    tkinter_module.messagebox = messagebox_module
    sys.modules["tkinter"] = tkinter_module
    sys.modules["tkinter.ttk"] = ttk_module
    sys.modules["tkinter.messagebox"] = messagebox_module


def _ensure_serial_import_surface():
    try:
        import serial.tools.list_ports  # noqa: F401
        return
    except ImportError:
        pass
    serial_module = types.ModuleType("serial")
    serial_module.__path__ = []
    tools_module = types.ModuleType("serial.tools")
    tools_module.__path__ = []
    ports_module = types.ModuleType("serial.tools.list_ports")
    ports_module.comports = lambda: []
    tools_module.list_ports = ports_module
    serial_module.tools = tools_module
    serial_module.Serial = object
    sys.modules["serial"] = serial_module
    sys.modules["serial.tools"] = tools_module
    sys.modules["serial.tools.list_ports"] = ports_module


_ensure_tkinter_import_surface()
_ensure_serial_import_surface()

from motor_control import (
    AxisMotionTelemetry,
    AxisProfile,
    AxisRuntime,
    BindingSet,
    NUM_STEPPER_AXES,
)
from motor_control.desktop_app import LAND_RELOCK_DELAY_S, StepperGUI
from motor_control.gait_executor import GaitExecutorError
from motor_control.protocol import (
    ErrorReply,
    OkReply,
    build_ena_command,
    reply_matcher_for,
)


class Value:
    """Tk 变量的最小替身。"""

    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeRoot:
    """root.after 的最小替身：记录延迟回调，由测试手动触发。"""

    def __init__(self):
        self.after_calls = []

    def after(self, ms, callback):
        self.after_calls.append((ms, callback))
        return len(self.after_calls)

    def after_cancel(self, _token):
        pass


def _land_app():
    app = StepperGUI.__new__(StepperGUI)
    app.paired_axes = (0, 1)
    app.axis_profiles = [AxisProfile() for _ in range(NUM_STEPPER_AXES)]
    app.axis_runtime = [
        AxisRuntime(position_trusted=True) for _ in range(NUM_STEPPER_AXES)
    ]
    app.running = [False] * NUM_STEPPER_AXES
    app.stepper_in_progress = [False] * NUM_STEPPER_AXES
    app._move_dispatching = [False] * NUM_STEPPER_AXES
    app._move_reservation = [None] * NUM_STEPPER_AXES
    app._web_step_pending = [None] * NUM_STEPPER_AXES
    app._pending_step = [None] * NUM_STEPPER_AXES
    app._axis_motion_generation = [0] * NUM_STEPPER_AXES
    app.axis_motion_telemetry = [
        AxisMotionTelemetry() for _ in range(NUM_STEPPER_AXES)
    ]
    app.control_bindings = BindingSet.from_axis_mapping(
        {"Mup1": 0, "Mr1": 2, "Mup2": 1, "Mr2": 3})
    app.axis_param_valid = [True] * NUM_STEPPER_AXES
    app.state_lock = threading.RLock()
    app.sw = [{} for _ in range(NUM_STEPPER_AXES)]
    app.paired_widgets = {}
    app._logs = []
    app.log = app._logs.append
    app._save_calib = lambda: None
    app._update_pos_label = lambda _axis: None
    app._is_serial_connected = lambda: True
    app._post_ui = lambda callback: callback()
    app.pair_land_release_enabled = Value(True)
    app.pair_land_release_threshold_mm = Value(3.0)
    app._rot_release_active = False
    app._rot_release_axes = ()
    app._rot_relock_after = None
    app._rot_relock_generation = 0
    app._rot_release_settled = threading.Event()
    app._rot_release_settled.set()
    app._last_land_gap_mm = None
    app._gait_owned = {}
    app.root = FakeRoot()
    app.commands = []
    app._send_and_read = lambda command, **_kwargs: (
        app.commands.append(command) or "OK,ENA,2,0")
    app.queued = []

    def queue_worker(target, *args, **kwargs):
        app.queued.append((target, args, kwargs))
        return object()

    app._start_control_worker = queue_worker
    return app


def _run_queued(app):
    """执行被 _start_control_worker 捕获的全部 worker（同步）。"""
    while app.queued:
        target, args, kwargs = app.queued.pop(0)
        target(*args, **kwargs)


class LandReleaseTests(unittest.TestCase):
    def test_estimated_position_includes_pending_progress(self):
        app = _land_app()
        app.axis_runtime[1].position_steps = 400        # 已结算 2.0 mm
        app._pending_step[1] = 600
        app.axis_motion_telemetry[1] = (
            AxisMotionTelemetry.starting(0, 600).with_progress(300, 600))
        # 轴0 = 0 mm；轴1 估算 = 400 + 600×0.5 = 700 steps = 3.5 mm
        self.assertAlmostEqual(app._land_gap_mm(), 3.5)

    def test_gap_falling_edge_triggers_release(self):
        app = _land_app()
        app.axis_runtime[0].position_steps = 0          # 站立腿基准
        app.axis_runtime[1].position_steps = 1000       # 悬空腿 5.0 mm
        app.stepper_in_progress[1] = True               # 运动中触发
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)
        app.axis_runtime[1].position_steps = 500        # 收敛到 2.5 mm
        app._land_release_tick(1)
        self.assertTrue(app._rot_release_active)
        self.assertEqual(app._rot_release_axes, (2, 3))
        _run_queued(app)
        self.assertEqual(app.commands, ["ENA,2,0", "ENA,3,0"])
        # 释放期间直线轴仍忙 → 不会同 tick 立即重锁
        self.assertTrue(any("落地纠偏" in message for message in app._logs))

    def test_standing_side_downward_also_triggers(self):
        # 等价路径：站立腿（读数高的一侧）“向下”同样令差值收敛。
        app = _land_app()
        app.axis_runtime[1].position_steps = 0          # 悬空腿基准
        app.axis_runtime[0].position_steps = 1000       # 站立腿 5.0 mm
        app.stepper_in_progress[0] = True
        app._land_release_tick(0)
        app.axis_runtime[0].position_steps = 400        # 2.0 mm ≤ 阈值
        app._land_release_tick(0)
        self.assertTrue(app._rot_release_active)
        _run_queued(app)
        self.assertEqual(app.commands, ["ENA,2,0", "ENA,3,0"])

    def test_paired_motion_does_not_trigger(self):
        # 联动整体升降（两轴同时运动）不是落地场景：启动时间差造成的
        # 瞬时差值既不评估也不记录基准（2026-09-17 18:31 误触发回归）。
        app = _land_app()
        app.stepper_in_progress[0] = True
        app.stepper_in_progress[1] = True
        app._last_land_gap_mm = 30.0                    # 残留的大基准
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 100        # 差值 0.5 mm
        app._land_release_tick(0)
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)
        self.assertEqual(app.commands, [])
        # 双轴运动期间不更新基准：仍停留在运动前的值
        self.assertEqual(app._last_land_gap_mm, 30.0)

    def test_set_home_and_calibrate_clear_gap_baseline(self):
        # 设原点/校准使读数跳变，差值基准必须作废，否则跨跳变比较
        # 会把"归零后首次运动"伪造成下降沿。
        app = _land_app()
        app._last_land_gap_mm = 30.0
        StepperGUI.set_home(app, 0)
        self.assertIsNone(app._last_land_gap_mm)
        app._last_land_gap_mm = 12.0
        app.v_goto = [Value(0.0) for _ in range(NUM_STEPPER_AXES)]
        StepperGUI.calibrate_position(app, 1)
        self.assertIsNone(app._last_land_gap_mm)

    def test_release_worker_tolerates_unconfirmed_ena(self):
        # ENA 用软超时：运动中固件响应慢时返回 None，只记"未确认"，
        # 不把超时升级成会话失步断连。
        app = _land_app()
        app._send_and_read = lambda command, **_kwargs: None
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 1000
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 500
        app._land_release_tick(1)
        self.assertTrue(app._rot_release_active)
        _run_queued(app)
        self.assertTrue(any("未确认" in message for message in app._logs))

    def test_gap_widening_never_triggers(self):
        app = _land_app()
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 500        # 2.5 mm
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 1000       # 抬腿：差值扩大
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)
        self.assertEqual(app.commands, [])

    def test_gap_already_within_threshold_no_trigger(self):
        app = _land_app()
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 400        # 2.0 mm
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 500        # 2.5 mm，始终低于阈值
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)

    def test_disabled_never_triggers_or_tracks(self):
        app = _land_app()
        app.pair_land_release_enabled = Value(False)
        app.axis_runtime[1].position_steps = 1000
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 500
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)
        # 禁用期间不记录 gap：重新启用后不会用陈旧差值误触发
        self.assertIsNone(app._last_land_gap_mm)

    def test_untrusted_position_does_not_trigger(self):
        app = _land_app()
        app.axis_runtime[1].position_trusted = False
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 1000
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 500
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)
        self.assertEqual(app.commands, [])

    def test_partial_mr_binding_skips_release(self):
        app = _land_app()
        app.control_bindings = BindingSet.from_axis_mapping(
            {"Mup1": 0, "Mr1": 2, "Mup2": 1, "Mr2": None})
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 1000
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 500
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)
        self.assertTrue(any("未绑定" in message for message in app._logs))

    def test_released_axis_move_rejected(self):
        app = _land_app()
        app._rot_release_active = True
        app._rot_release_axes = (2, 3)
        self.assertFalse(app._send_pulses(2, 200, 0, 1.0))
        self.assertTrue(any("已释放" in message for message in app._logs))
        # 未被释放的直线轴不受拦截，照常下发 MOVE
        app._send_pulses(0, 200, 0, 1.0)
        self.assertTrue(any(cmd.startswith("MOVE,0,") for cmd in app.commands))

    def test_relock_after_both_linear_idle(self):
        app = _land_app()
        app._rot_release_active = True
        app._rot_release_axes = (2, 3)
        app._land_release_tick(0)
        # 落地后不再立即重锁：先安排延迟定时器
        self.assertTrue(app._rot_release_active)
        self.assertEqual(len(app.root.after_calls), 1)
        delay_ms, fire = app.root.after_calls[0]
        self.assertEqual(delay_ms, LAND_RELOCK_DELAY_S * 1000)
        fire()
        self.assertFalse(app._rot_release_active)
        _run_queued(app)
        self.assertEqual(app.commands, ["ENA,2,1", "ENA,3,1"])

    def test_busy_linear_axis_defers_relock(self):
        app = _land_app()
        app._rot_release_active = True
        app._rot_release_axes = (2, 3)
        app.stepper_in_progress[1] = True
        app._land_release_tick(0)
        self.assertTrue(app._rot_release_active)
        self.assertEqual(app.commands, [])
        self.assertEqual(app.root.after_calls, [])

    def test_relaunched_linear_motion_cancels_pending_relock(self):
        # 延迟等待期间直线运动重启 = 落地未完成：作废本次定时器，
        # 待再次落地后重新计 5 秒。
        app = _land_app()
        app._rot_release_active = True
        app._rot_release_axes = (2, 3)
        app._land_release_tick(0)
        self.assertEqual(len(app.root.after_calls), 1)
        _delay_ms, stale_fire = app.root.after_calls[0]
        app.stepper_in_progress[1] = True
        app._land_release_tick(0)
        self.assertIsNone(app._rot_relock_after)
        stale_fire()                       # 已被作废的旧回调：不得重锁
        self.assertTrue(app._rot_release_active)
        self.assertEqual(app.commands, [])
        app.stepper_in_progress[1] = False
        app._land_release_tick(0)          # 再次落地 → 重新调度
        self.assertEqual(len(app.root.after_calls), 2)
        app.root.after_calls[1][1]()
        _run_queued(app)
        self.assertEqual(app.commands, ["ENA,2,1", "ENA,3,1"])

    def test_gait_release_keeps_position_trusted(self):
        # 步态占用期间同样触发，但旋转轴位置继续沿用（不标记不可信），
        # 换位流程才能继续。
        app = _land_app()
        app._gait_owned = {0: object(), 1: object(), 2: object(), 3: object()}
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 1000
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 500
        app._land_release_tick(1)
        self.assertTrue(app._rot_release_active)
        _run_queued(app)
        self.assertEqual(app.commands, ["ENA,2,0", "ENA,3,0"])
        self.assertTrue(app.axis_runtime[2].position_trusted)
        self.assertTrue(app.axis_runtime[3].position_trusted)
        self.assertTrue(any("步态落脚纠偏" in m for m in app._logs))

    def test_single_axis_release_marks_untrusted(self):
        app = _land_app()
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 1000
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 500
        app._land_release_tick(1)
        self.assertTrue(app._rot_release_active)
        _run_queued(app)
        self.assertFalse(app.axis_runtime[2].position_trusted)
        self.assertFalse(app.axis_runtime[3].position_trusted)

    def test_rot_axis_pulsing_skips_release(self):
        # 旋转轴正在脉冲（如步态 SYNC 摆动段）时绝不失能。
        app = _land_app()
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 1000
        app.stepper_in_progress[1] = True
        app.stepper_in_progress[2] = True     # Mr1 轴在脉冲
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 500
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)
        self.assertEqual(app.commands, [])

    def test_ensure_locked_before_gait_rotation_stage(self):
        # 5 秒延迟未到就要执行下一旋转阶段：提前重锁并确认，旧定时器作废。
        app = _land_app()
        app._rot_release_active = True
        app._rot_release_axes = (2, 3)
        app._land_release_tick(0)             # 调度延迟重锁
        self.assertEqual(len(app.root.after_calls), 1)
        StepperGUI._ensure_rot_axes_locked_blocking(app)
        self.assertFalse(app._rot_release_active)
        self.assertEqual(app.commands, ["ENA,2,1", "ENA,3,1"])
        app.root.after_calls[0][1]()          # 旧回调到点：已被作废
        self.assertEqual(app.commands, ["ENA,2,1", "ENA,3,1"])
        self.assertTrue(any("步态继续" in m for m in app._logs))

    def test_ensure_locked_raises_without_confirmation(self):
        app = _land_app()
        app._send_and_read = lambda command, **_kwargs: None
        app._rot_release_active = True
        app._rot_release_axes = (2, 3)
        with self.assertRaises(GaitExecutorError):
            StepperGUI._ensure_rot_axes_locked_blocking(app)

    def _manual_app(self):
        """手动释放验证的替身：Mr1/Mr2 绑定旋转轴 + 弹窗一律放行。

        本机有真 tkinter 时 messagebox 是真模块：必须 mock 掉全部
        弹窗函数，否则 showwarning 会弹出真对话框阻塞测试。
        """
        from unittest import mock

        from motor_control import MODE_ROTARY
        from motor_control import desktop_app as desktop_app_module

        app = _land_app()
        app.axis_profiles[2] = AxisProfile(mode=MODE_ROTARY)
        app.axis_profiles[3] = AxisProfile(mode=MODE_ROTARY)
        for name in ("showwarning", "showerror", "showinfo"):
            patcher = mock.patch.object(
                desktop_app_module.messagebox, name, lambda *_a, **_k: None)
            patcher.start()
            self.addCleanup(patcher.stop)
        confirm = mock.patch.object(
            desktop_app_module.messagebox, "askokcancel", return_value=True)
        confirm.start()
        self.addCleanup(confirm.stop)
        return app

    def test_manual_release_schedules_auto_relock(self):
        # 手动释放走同一链路：ENA,0 ×2 + 5 秒兜底自动锁定；
        # 不把旋转轴位置标记为不可信（验证用途，便于恢复）。
        app = self._manual_app()
        StepperGUI._toggle_manual_rot_release(app)
        self.assertTrue(app._rot_release_active)
        _run_queued(app)
        self.assertEqual(app.commands, ["ENA,2,0", "ENA,3,0"])
        self.assertTrue(app.axis_runtime[2].position_trusted)
        self.assertTrue(app.axis_runtime[3].position_trusted)
        self.assertEqual(len(app.root.after_calls), 1)
        app.root.after_calls[0][1]()          # 5 秒到点
        _run_queued(app)
        self.assertEqual(
            app.commands, ["ENA,2,0", "ENA,3,0", "ENA,2,1", "ENA,3,1"])

    def test_manual_release_then_immediate_relock(self):
        # 释放态再点一次：取消兜底定时器，立即锁定。
        app = self._manual_app()
        StepperGUI._toggle_manual_rot_release(app)     # 释放
        _run_queued(app)
        StepperGUI._toggle_manual_rot_release(app)     # 立即锁定
        self.assertFalse(app._rot_release_active)
        self.assertIsNone(app._rot_relock_after)
        _run_queued(app)
        self.assertEqual(
            app.commands, ["ENA,2,0", "ENA,3,0", "ENA,2,1", "ENA,3,1"])
        app.root.after_calls[0][1]()          # 已作废的兜底回调：不得重复发
        self.assertEqual(
            app.commands, ["ENA,2,0", "ENA,3,0", "ENA,2,1", "ENA,3,1"])

    def test_manual_release_blocked_while_gait_owns_axes(self):
        app = self._manual_app()
        app._gait_owned = {0: object()}
        StepperGUI._toggle_manual_rot_release(app)
        self.assertFalse(app._rot_release_active)
        self.assertEqual(app.commands, [])

    def test_force_relock_sends_lock_immediately(self):
        app = _land_app()
        app._rot_release_active = True
        app._rot_release_axes = (2, 3)
        app._force_relock_rot_axes()
        self.assertFalse(app._rot_release_active)
        self.assertEqual(app.commands, ["ENA,2,1", "ENA,3,1"])

    def test_threshold_is_effective(self):
        app = _land_app()
        app.pair_land_release_threshold_mm = Value(2.0)
        app.axis_runtime[0].position_steps = 0
        app.axis_runtime[1].position_steps = 700         # 3.5 mm
        app.stepper_in_progress[1] = True
        app._land_release_tick(1)
        app.axis_runtime[1].position_steps = 580         # 2.9 mm > 2.0
        app._land_release_tick(1)
        self.assertFalse(app._rot_release_active)
        app.axis_runtime[1].position_steps = 300         # 1.5 mm ≤ 2.0
        app._land_release_tick(1)
        self.assertTrue(app._rot_release_active)


class EnaCommandProtocolTests(unittest.TestCase):
    def test_build_ena_command_forms(self):
        self.assertEqual(build_ena_command(2, False), "ENA,2,0")
        self.assertEqual(build_ena_command(2, True), "ENA,2,1")
        self.assertEqual(build_ena_command(3), "ENA,3,S")

    def test_matcher_requires_family_and_axis(self):
        matcher = reply_matcher_for("ENA,2,0")
        self.assertTrue(matcher(OkReply("OK,ENA,2,0", ("ENA", "2", "0"))))
        self.assertFalse(matcher(OkReply("OK,ENA,3,0", ("ENA", "3", "0"))))
        self.assertFalse(matcher(OkReply("OK,ESTOP", ("ESTOP",))))
        # 固件错误回复永远允许完成请求
        self.assertTrue(matcher(ErrorReply("ERR:busy", "busy")))


if __name__ == "__main__":
    unittest.main()
