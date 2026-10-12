"""2026-09-29 X42S UART PC 侧（feature/x42s-uart-actuator-binding 第二轮）。

- 协议层：X42S 遥测/诊断/扫描/掉线行的保守解析（valid 位还原 None）、
  回复匹配器与命令构造（0=广播禁入）；
- 快照合并：x42s 遥测并入 build_coordinated_snapshot（未知不冒充 0、
  掉线/故障/超期/断连分层，旧驱动器状态不投到新绑定）；
- 桌面集成：步态后端闸（§19）、点动/零位对 X42S 绑定拒绝、
  遥测并入状态缓存、落地纠偏在 X42S 后端整体跳过。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # 裸名互导
from types import SimpleNamespace

from motor_control import (
    BACKEND_STEPDIR,
    BACKEND_X42S_UART,
    LOGICAL_ROLE_ORDER,
    BindingSet,
)
from motor_control.actuator_model import (
    KNOWN_X42S_FAULTS,
    X42sActuatorState,
    x42s_state_issues,
)
from motor_control.coordinated_control import build_coordinated_snapshot
from motor_control.desktop_app import StepperGUI
from motor_control.gait_executor import GaitExecutorError
from motor_control.protocol import (
    OkReply,
    ProtocolEncodingError,
    UnknownMessage,
    X42S_FAULT_TAGS,
    X42sDiagReply,
    X42sMoveAck,
    X42sOfflineEvent,
    X42sScanReply,
    X42sStatusReply,
    build_x42s_diag_command,
    build_x42s_enable_command,
    build_x42s_move_command,
    build_x42s_scan_command,
    build_x42s_status_command,
    build_x42s_stop_command,
    build_x42s_sync_command,
    build_x42s_zero_command,
    parse_line,
    reply_matcher_for,
)

from test_desktop_axis_integration import _headless_app


def _snapshot_app():
    """最小 stepper 状态夹具（快照合并测试只关心 x42s 分支）。"""

    from motor_control import (
        AxisMotionTelemetry,
        AxisProfile,
        AxisRuntime,
        NUM_STEPPER_AXES,
    )

    return SimpleNamespace(
        profiles=[AxisProfile() for _ in range(NUM_STEPPER_AXES)],
        runtimes=[AxisRuntime() for _ in range(NUM_STEPPER_AXES)],
        axis_param_valid=[True] * NUM_STEPPER_AXES,
        running=[False] * NUM_STEPPER_AXES,
        stepper_in_progress=[False] * NUM_STEPPER_AXES,
        move_dispatching=[False] * NUM_STEPPER_AXES,
        move_reservations=[None] * NUM_STEPPER_AXES,
        web_step_pending=[None] * NUM_STEPPER_AXES,
        pending_steps=[None] * NUM_STEPPER_AXES,
        telemetry=[AxisMotionTelemetry() for _ in range(NUM_STEPPER_AXES)],
    )


def _x42s_snapshot(fixture, bindings, states=None, *, connected=True,
                   safety_state="normal", now=1.0):
    control, motors = build_coordinated_snapshot(
        bindings=bindings,
        profiles=fixture.profiles,
        runtimes=fixture.runtimes,
        axis_param_valid=fixture.axis_param_valid,
        running=fixture.running,
        stepper_in_progress=fixture.stepper_in_progress,
        move_dispatching=fixture.move_dispatching,
        move_reservations=fixture.move_reservations,
        web_step_pending=fixture.web_step_pending,
        pending_steps=fixture.pending_steps,
        telemetry=fixture.telemetry,
        connected=connected,
        safety_state=safety_state,
        x42s_states=states,
        now=now,
    )
    return control, {motor["role"]: motor for motor in motors}


class X42sProtocolTests(unittest.TestCase):
    def test_fault_tag_set_locked_to_model(self):
        # 三处标签集（协议/模型/固件）必须一致，否则遥测会被静默丢弃。
        self.assertEqual(X42S_FAULT_TAGS, ("none",) + KNOWN_X42S_FAULTS)

    def test_status_line_parses_with_valid_mask(self):
        # 16 字段：valid_mask=0b111000 ⇒ 只有 speed/current/temp 有效。
        line = ("X42S,3,S,1,1,0,none,0.000,0.000,0.000,12.500,1.800,42.000,"
                "35,0,56")
        message = parse_line(line)
        self.assertIsInstance(message, X42sStatusReply)
        self.assertEqual(message.driver_id, 3)
        self.assertTrue(message.online)
        self.assertTrue(message.enabled)
        self.assertFalse(message.reached)
        self.assertIsNone(message.fault)
        self.assertIsNone(message.command_position)
        self.assertIsNone(message.measured_position)
        self.assertIsNone(message.position_error)
        self.assertEqual(message.speed, 12.5)
        self.assertEqual(message.current_a, 1.8)
        self.assertEqual(message.temperature_c, 42.0)
        self.assertEqual(message.age_ms, 35)
        self.assertEqual(message.timeout_count, 0)

    def test_status_line_full_validity(self):
        line = ("X42S,1,S,1,0,1,stall,-100.500,-101.250,0.750,0.000,0.000,"
                "0.000,120,3,63")
        message = parse_line(line)
        self.assertIsInstance(message, X42sStatusReply)
        self.assertEqual(message.fault, "stall")
        self.assertFalse(message.enabled)
        self.assertTrue(message.reached)
        self.assertEqual(message.command_position, -100.5)
        self.assertEqual(message.measured_position, -101.25)
        self.assertEqual(message.position_error, 0.75)

    def test_malformed_status_lines_stay_unknown(self):
        for line in (
            "X42S,0,S,1,1,0,none,0,0,0,0,0,0,0,0,0",        # 广播 ID
            "X42S,3,S,1,1,0,vapor_lock,0,0,0,0,0,0,0,0,63",  # 未知故障标签
            "X42S,3,S,1,1,0,none,abc,0,0,0,0,0,0,0,63",      # 非数值
            "X42S,3,S,1,1,0,none,0,0,0,0,0,0,0,0",           # 15 字段（缺 mask）
            "X42S,3,S,2,1,0,none,0,0,0,0,0,0,0,0,63",        # 三态 flag 非法
        ):
            self.assertIsInstance(parse_line(line), UnknownMessage, line)

    def test_scan_diag_and_offline_lines(self):
        scan = parse_line("X42S,2,SCAN,ONLINE")
        self.assertIsInstance(scan, X42sScanReply)
        self.assertEqual((scan.driver_id, scan.online), (2, True))
        self.assertIsInstance(
            parse_line("X42S,4,SCAN,OFFLINE"), X42sScanReply)
        diag = parse_line("X42S,1,DIAG,10,8,1,0,1,2")
        self.assertIsInstance(diag, X42sDiagReply)
        self.assertEqual(
            (diag.frames_sent, diag.timeouts, diag.checksum_errors),
            (10, 2, 1),
        )
        self.assertIsInstance(
            parse_line("X42S,1,DIAG,10,8,1,0,1,x"), UnknownMessage)
        offline = parse_line("X42S,2,OFFLINE")
        self.assertIsInstance(offline, X42sOfflineEvent)
        self.assertEqual(offline.driver_id, 2)
        self.assertIsInstance(parse_line("X42S,9,WHAT"), UnknownMessage)

    def test_reply_matcher_binds_status_to_driver_id(self):
        matcher = reply_matcher_for(build_x42s_status_command(3))
        good = parse_line(
            "X42S,3,S,1,1,0,none,0,0,0,0,0,0,0,0,0")
        other = parse_line(
            "X42S,4,S,1,1,0,none,0,0,0,0,0,0,0,0,0")
        self.assertTrue(matcher(good))
        self.assertFalse(matcher(other))
        self.assertTrue(matcher(parse_line("ERR:driver timeout")))
        self.assertTrue(reply_matcher_for(build_x42s_scan_command())(
            parse_line("X42S,1,SCAN,ONLINE")))
        self.assertTrue(reply_matcher_for(build_x42s_diag_command(2))(
            parse_line("X42S,2,DIAG,0,0,0,0,0,0")))
        self.assertTrue(reply_matcher_for(build_x42s_sync_command())(
            parse_line("OK,X42S,SYNC")))
        # X42S,<id>,EN 的 OK 回复按台号匹配，防他台迟到行误配。
        en_matcher = reply_matcher_for("X42S,5,EN,1")
        self.assertTrue(en_matcher(parse_line("OK,X42S,5,EN,1")))
        self.assertFalse(en_matcher(parse_line("OK,X42S,6,EN,1")))

    def test_builders_reject_broadcast_and_out_of_range(self):
        self.assertEqual(build_x42s_status_command(7), "X42S,S,7")
        self.assertEqual(build_x42s_diag_command(1), "X42S,DIAG,1")
        self.assertEqual(build_x42s_scan_command(), "X42S,SCAN")
        self.assertEqual(build_x42s_sync_command(), "X42S,SYNC")
        for bad in (0, -1, 256, True):
            with self.assertRaises(ProtocolEncodingError):
                build_x42s_status_command(bad)

    def test_motion_builders_and_move_ack_round_trip(self):
        # 2026-10-12 手动调试命令族：EN/MOVE/STOP/ZERO 与固件分发一一对齐。
        self.assertEqual(build_x42s_enable_command(1, True), "X42S,1,EN,1")
        self.assertEqual(build_x42s_enable_command(2, False), "X42S,2,EN,0")
        # 台架基准值：输出轴 90° = 17200 脉冲 @1600pps。
        self.assertEqual(
            build_x42s_move_command(1, 17200, 1600), "X42S,1,MOVE,17200,1600")
        self.assertEqual(
            build_x42s_move_command(1, -17200, 1600), "X42S,1,MOVE,-17200,1600")
        self.assertEqual(build_x42s_stop_command(3), "X42S,3,STOP")
        self.assertEqual(build_x42s_zero_command(4), "X42S,4,ZERO")
        for bad in (0, -1, 256, True):
            with self.assertRaises(ProtocolEncodingError):
                build_x42s_enable_command(bad, True)
            with self.assertRaises(ProtocolEncodingError):
                build_x42s_stop_command(bad)
            with self.assertRaises(ProtocolEncodingError):
                build_x42s_zero_command(bad)
        # steps/speed 校验：0、负速、bool、浮点全拒（固件同规则）。
        for bad_steps in (0, True, 17200.0, "17200"):
            with self.assertRaises(ProtocolEncodingError):
                build_x42s_move_command(1, bad_steps, 1600)
        for bad_speed in (0, -1, True, 1600.5, "1600"):
            with self.assertRaises(ProtocolEncodingError):
                build_x42s_move_command(1, 17200, bad_speed)

    def test_move_ack_parses_and_rejects_malformed(self):
        ack = parse_line("ACK,X42S,1,MOVE,17200,1600")
        self.assertIsInstance(ack, X42sMoveAck)
        self.assertEqual((ack.driver_id, ack.steps, ack.speed),
                         (1, 17200, 1600))
        reverse = parse_line("ACK,X42S,1,MOVE,-17200,1600")
        self.assertEqual(reverse.steps, -17200)
        # 固件不会 ACK 零步/零速：此类行按未知处理，不得冒充成功。
        for line in (
            "ACK,X42S,1,MOVE,0,1600",
            "ACK,X42S,1,MOVE,17200,0",
            "ACK,X42S,0,MOVE,17200,1600",
            "ACK,X42S,1,MOVE,abc,1600",
            "ACK,X42S,1,MOVE,17200",
            "ACK,X42S,1,HOME,17200,1600",
        ):
            self.assertIsInstance(parse_line(line), UnknownMessage, line)
        # EN/STOP/ZERO 的应答走通用 OK 行（回归：现有解析路径不变）。
        ok = parse_line("OK,X42S,1,EN,1")
        self.assertIsInstance(ok, OkReply)
        self.assertEqual(ok.values, ("X42S", "1", "EN", "1"))

    def test_reply_matcher_binds_move_ack_to_driver(self):
        # 2026-10-12 实机回归：matcher 不认 ACK,X42S,..,MOVE 时，成功应答
        # 被当迟到行丢弃，GUI 弹"未被接受"但电机照转——必须按台号认领。
        move = reply_matcher_for(build_x42s_move_command(1, 17200, 1605))
        self.assertTrue(move(parse_line("ACK,X42S,1,MOVE,17200,1605")))
        self.assertFalse(move(parse_line("ACK,X42S,2,MOVE,17200,1605")))
        self.assertTrue(move(parse_line("ERR:driver timeout")))
        self.assertFalse(move(parse_line("OK,X42S,1,EN,1")))
        # EN/STOP/ZERO 的 OK 应答同样按台号匹配（回归）。
        self.assertTrue(reply_matcher_for("X42S,3,STOP")(
            parse_line("OK,X42S,3,STOP")))
        self.assertFalse(reply_matcher_for("X42S,3,STOP")(
            parse_line("OK,X42S,4,STOP")))
        self.assertTrue(reply_matcher_for("X42S,2,ZERO")(
            parse_line("OK,X42S,2,ZERO")))


class SnapshotMergeTests(unittest.TestCase):
    def setUp(self):
        self.fixture = _snapshot_app()
        self.bindings = BindingSet.suggested_x42s()

    def _live_state(self, role_name="Mr1", **kwargs):
        role = next(r for r in LOGICAL_ROLE_ORDER if r.value == role_name)
        defaults = dict(
            enabled=True, reached=True,
            command_position=1200.0, measured_position=1198.0,
            position_error=-2.0,
        )
        defaults.update(kwargs)
        return X42sActuatorState(
            role=role, driver_id=2).with_telemetry(now=0.9, **defaults)

    def test_trusted_telemetry_projects_measured_position(self):
        states = {"Mr1": self._live_state()}
        control, motors = _x42s_snapshot(self.fixture, self.bindings, states,
                                         now=1.0)
        card = motors["Mr1"]
        self.assertEqual(card["state"], "X42S_ONLINE")
        # 驱动器遥测是电机轴角度；快照统一 ÷gear_ratio（默认 21.5，
        # 2026-10-10 实机确认）换算为输出轴角度（§12 语义不变：仍是实测）。
        self.assertEqual(card["gear_ratio"], 21.5)
        self.assertAlmostEqual(card["position"], 1198.0 / 21.5)
        self.assertAlmostEqual(card["target_position"], 1200.0 / 21.5)
        # 原始电机轴遥测完整保留在 x42s 子字典，不做有损覆盖。
        self.assertEqual(card["x42s"]["measured_position"], 1198.0)
        self.assertTrue(card["position_trusted"])
        self.assertTrue(card["measured"])
        self.assertEqual(card["position_source"], "x42s_driver")
        self.assertEqual(card["unit"], "deg")
        self.assertEqual(card["driver_id"], 2)
        self.assertIn("x42s", card)
        self.assertEqual(control["backend"], BACKEND_X42S_UART)

    def test_gear_ratio_converts_motor_shaft_to_output(self):
        # 实机验收值：输出轴 90° = 电机轴 1935°（1:21.5）。自定义减速比
        # 绑定换算 1935/21.5=90.0；直驱（gear_ratio=1）原样透传。
        role = next(r for r in LOGICAL_ROLE_ORDER if r.value == "Mr1")
        state = X42sActuatorState(
            role=role, driver_id=2).with_telemetry(
            now=0.9, measured_position=1935.0, command_position=1935.0)
        bindings = BindingSet.from_driver_id_mapping(
            {"Mup1": None, "Mr1": 2, "Mup2": None, "Mr2": None},
            gear_ratios={"Mr1": 21.5},
        )
        _, motors = _x42s_snapshot(self.fixture, bindings,
                                   {"Mr1": state}, now=1.0)
        card = motors["Mr1"]
        self.assertEqual(card["gear_ratio"], 21.5)
        self.assertAlmostEqual(card["position"], 90.0)
        self.assertAlmostEqual(card["target_position"], 90.0)

        direct = BindingSet.from_driver_id_mapping(
            {"Mup1": None, "Mr1": 2, "Mup2": None, "Mr2": None},
            gear_ratios={"Mr1": 1.0},
        )
        _, motors = _x42s_snapshot(self.fixture, direct,
                                   {"Mr1": state}, now=1.0)
        self.assertAlmostEqual(motors["Mr1"]["position"], 1935.0)

    def test_partial_validity_does_not_invent_numbers(self):
        # 只有 enabled/reached，无任何位置字段（valid 位全未置）。
        role = next(r for r in LOGICAL_ROLE_ORDER if r.value == "Mr1")
        state = X42sActuatorState(
            role=role, driver_id=2).with_telemetry(now=0.95, enabled=True)
        _, motors = _x42s_snapshot(self.fixture, self.bindings,
                                   {"Mr1": state}, now=1.0)
        card = motors["Mr1"]
        self.assertIsNone(card["position"])
        self.assertFalse(card["measured"])
        self.assertFalse(card["position_trusted"])
        self.assertEqual(card["state"], "X42S_ONLINE")  # 在线但无位置

    def test_state_layering(self):
        offline = {"Mr1": X42sActuatorState.offline(
            next(r for r in LOGICAL_ROLE_ORDER if r.value == "Mr1"), 2)}
        _, motors = _x42s_snapshot(self.fixture, self.bindings, offline)
        self.assertEqual(motors["Mr1"]["state"], "X42S_OFFLINE")
        faulted = {"Mr1": self._live_state(fault="over_temp")}
        _, motors = _x42s_snapshot(self.fixture, self.bindings, faulted)
        self.assertEqual(motors["Mr1"]["state"], "X42S_FAULT")
        stale = {"Mr1": self._live_state()}  # now=1.7 ⇒ age=800ms > 500
        _, motors = _x42s_snapshot(self.fixture, self.bindings, stale,
                                   now=1.7)
        self.assertEqual(motors["Mr1"]["state"], "X42S_STALE")
        self.assertFalse(motors["Mr1"]["position_trusted"])
        live = {"Mr1": self._live_state()}
        _, motors = _x42s_snapshot(self.fixture, self.bindings, live,
                                   connected=False)
        self.assertEqual(motors["Mr1"]["state"], "DISCONNECTED")
        _, motors = _x42s_snapshot(self.fixture, self.bindings, live,
                                   safety_state="software_estop")
        self.assertEqual(motors["Mr1"]["state"], "SOFTWARE_ESTOP")

    def test_stale_driver_state_for_rebound_role_is_ignored(self):
        # 角色换绑到台 5 后，缓存里台 2 的旧状态不得投影到该角色。
        rebound = BindingSet.from_driver_id_mapping(
            {"Mup1": 1, "Mr1": 5, "Mup2": 3, "Mr2": 4})
        states = {"Mr1": self._live_state()}  # driver_id=2
        _, motors = _x42s_snapshot(self.fixture, rebound, states, now=1.0)
        self.assertEqual(motors["Mr1"]["state"], "X42S_BACKEND_PENDING")
        self.assertIsNone(motors["Mr1"]["position"])

    def test_missing_state_keeps_pending_projection(self):
        _, motors = _x42s_snapshot(self.fixture, self.bindings, states={})
        for role in ("Mup1", "Mr1", "Mup2", "Mr2"):
            self.assertEqual(motors[role]["state"], "X42S_BACKEND_PENDING")
            self.assertIsNone(motors[role]["position"])

    def test_state_issues_flag_addr_conflict_and_offline(self):
        mup1 = next(r for r in LOGICAL_ROLE_ORDER if r.value == "Mup1")
        mr1 = next(r for r in LOGICAL_ROLE_ORDER if r.value == "Mr1")
        issues = x42s_state_issues([
            X42sActuatorState(role=mup1, driver_id=3),
            X42sActuatorState(role=mr1, driver_id=3),
        ])
        self.assertTrue(any("地址冲突" in text for text in issues))
        self.assertTrue(any("offline" in text for text in issues))


class X42sPendingRefreshTests(unittest.TestCase):
    """2026-10-12 实机回归：保存 x42s 绑定后重启 GUI，启动期后端待接入
    （states 为空、快照无 "x42s" 键）时 refresh_coordinated_tab 落进了
    步进轴分支取 binding["axis"]，KeyError 启动即崩。"""

    class _Widget:
        def __init__(self):
            self.configured = {}

        def configure(self, **kwargs):
            self.configured.update(kwargs)

    def test_pending_x42s_binding_renders_waiting_without_crash(self):
        from motor_control.ui.coordinated_tab import refresh_coordinated_tab

        fixture = _snapshot_app()
        control, motors = _x42s_snapshot(
            fixture, BindingSet.suggested_x42s(), states={})
        roles, status = {}, {}
        for role in ("Mup1", "Mr1", "Mup2", "Mr2"):
            roles[role] = {name: self._Widget() for name in
                           ("state", "position", "detail", "progress", "canvas")}
            status[role] = self._Widget()
        app = SimpleNamespace(
            coordinated_widgets={
                "roles": roles,
                "binding_status": status,
                "ready_label": self._Widget(),
                "blocker_label": self._Widget(),
            },
            _binding_editor_dirty=False,
        )
        refresh_coordinated_tab(app, control, list(motors.values()))
        for role in ("Mup1", "Mr1", "Mup2", "Mr2"):
            self.assertIn("后端待接入", roles[role]["detail"].configured["text"])
            self.assertIn("连接串口", roles[role]["progress"].configured["text"])


class _Var:
    def __init__(self, value):
        self._value = value

    def get(self):
        return self._value

    def set(self, value):
        self._value = value


class DesktopIntegrationTests(unittest.TestCase):
    def _app(self):
        app = _headless_app()
        app._x42s_states = {}
        app._binding_editor_dirty = False
        app._binding_editor_revision = 0
        app.coordinated_widgets = {}
        return app

    def test_gait_role_axis_rejects_x42s_binding(self):
        app = self._app()
        app.control_bindings = BindingSet.suggested_x42s()
        with self.assertRaises(GaitExecutorError) as ctx:
            StepperGUI._gait_role_axis(app, "Mr1")
        self.assertIn("X42S", str(ctx.exception))

    def test_apply_x42s_reply_merges_and_keeps_unknown_none(self):
        app = self._app()
        app.control_bindings = BindingSet.suggested_x42s()
        role = next(r for r in LOGICAL_ROLE_ORDER if r.value == "Mr1")
        reply = parse_line("X42S,2,S,1,1,1,none,500.0,499.0,-1.0,0,0,0,10,0,7")
        StepperGUI._apply_x42s_reply(app, role, 2, reply)
        state = app._x42s_states["Mr1"]
        self.assertTrue(state.online)
        self.assertEqual(state.measured_position, 499.0)
        self.assertIsNone(state.speed)  # valid 位未置 ⇒ None，不冒充 0
        # 错台号回复不得并入该角色。
        wrong = parse_line("X42S,9,S,1,1,1,none,1,1,0,0,0,0,0,0,7")
        StepperGUI._apply_x42s_reply(app, role, 2, wrong)
        self.assertEqual(app._x42s_states["Mr1"].measured_position, 499.0)

    def test_binding_draft_backend_aware(self):
        app = self._app()
        app.binding_backend_var = _Var(BACKEND_X42S_UART)
        app.x42s_binding_vars = {
            "Mup1": _Var("1"), "Mr1": _Var("2"),
            "Mup2": _Var("3"), "Mr2": _Var("4"),
        }
        app.control_binding_vars = {}
        app._binding_axis_by_option = {}
        draft = StepperGUI._binding_draft(app)
        self.assertEqual(draft.backend, BACKEND_X42S_UART)
        self.assertEqual(draft.as_driver_id_mapping(),
                         {"Mup1": 1, "Mr1": 2, "Mup2": 3, "Mr2": 4})
        # 非整数 / 广播号 / 重复号都要在构造层被拒。
        app.x42s_binding_vars["Mr2"].set("x")
        from motor_control import BindingValidationError
        with self.assertRaises(BindingValidationError):
            StepperGUI._binding_draft(app)
        app.x42s_binding_vars["Mr2"].set("4")
        app.x42s_binding_vars["Mup2"].set("4")  # 重复台号
        with self.assertRaises(BindingValidationError) as ctx:
            StepperGUI._binding_draft(app)
        self.assertEqual(ctx.exception.code, "duplicate_driver_id")

    def test_gait_begin_rejects_x42s_backend(self):
        from test_gait_linkage import mechanism
        app = mechanism()
        app.control_bindings = BindingSet.suggested_x42s()
        with self.assertRaises(GaitExecutorError) as ctx:
            app._gait_begin_run("left", arc_deg=-60.0)
        self.assertIn("X42S", str(ctx.exception))
        self.assertEqual(app._gait_owned, {})

    def test_land_release_skips_without_stepper_binding(self):
        app = self._app()
        app.control_bindings = BindingSet.suggested_x42s()
        app.pair_land_release_enabled = _Var(True)
        StepperGUI._trigger_land_release(app)
        self.assertFalse(app._rot_release_active)
        self.assertTrue(any("未绑定物理步进轴" in line for line in app._logs))


if __name__ == "__main__":
    unittest.main()
