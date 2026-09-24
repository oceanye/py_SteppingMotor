import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import os
import math
import socket
import webbrowser
import sys
import traceback
import hashlib
import json
from dataclasses import replace

from motor_control import (
    AXIS_LABEL,
    DEFAULT_GEAR_RATIO,
    DEFAULT_LEAD_MM,
    DEFAULT_PULSE_PER_REV,
    MODE_LINEAR,
    MODE_ROTARY,
    MOTOR_AXIS_LABELS,
    NUM_LOCAL_STEPPER_AXES,
    NUM_PICO_NODES,
    NUM_STEPPER_AXES,
    PICO_AXES_PER_NODE,
    SPEED_DEFAULT,
    STEPPER_PINS,
    AxisProfile,
    AxisRuntime,
    AxisMotionTelemetry,
    BindingSet,
    BindingValidationError,
    LOGICAL_ROLE_ORDER,
    LogicalRole,
    ROLE_SPECS,
    binding_compatibility_issues,
    build_coordinated_snapshot,
    clamp_step_delay_us,
    convert_axis_value,
    direction_label_parts,
    outward_position_sign,
    speed_to_delay_ms,
    step_delay_ms_to_pulse_rate,
    step_speed_clamps_delay,
    step_speed_to_delay_ms,
    stepper_axis_topology,
    parse_binding_document,
    projected_step_position,
)
from motor_control.axis_math import (
    PULSE_RATE_WARN_PPS,
    coerce_finite_in_range,
)
from motor_control.autotune import AutotuneRunner
from motor_control.protocol import (
    HardwareEstopEvent,
    HardwareEstopState,
    MotorFaultEvent,
    NodeOfflineEvent,
    StepProgress,
    StepResult,
    StepTerminal,
    TrackTimeout,
    UnknownMessage,
    ProtocolEncodingError,
    build_ena_command,
    build_move_command,
    build_sync_command,
    build_stop_command,
    build_track_command,
)
from motor_control.serial_session import (
    RequestCancelled,
    RequestTimeout,
    SerialSession,
    SerialSessionError,
)
from motor_control.state_store import StateStore, StateStoreError
from motor_control.gait_executor import GaitExecutor, GaitExecutorError
from motor_control.gait_avoidance import TWO_MODE, leg_clearance
from motor_control.gait_planner import (
    LOW_NODE_PHASE_DEG,
    SWING_ARC_DEG,
    GaitParams,
    parse_gait_params,
    plan_gait_stages,
    plan_swing_trajectory,
    unwrap_swing_joint_delta,
    clearance_margin_mm,
    effective_geometry,
    gait_pads,
)
from motor_control.ui_dispatch import UiDispatcher
from motor_control.ui import (
    build_coordinated_tab as build_coordinated_tab_view,
    build_foc_tab as build_foc_tab_view,
    build_gait_tab as build_gait_tab_view,
    build_gear_tab as build_gear_tab_view,
    build_paired_tab as build_paired_tab_view,
    build_stepper_tab as build_stepper_tab_view,
    build_track_tab as build_track_tab_view,
    build_ui as build_main_ui,
    collect_gait_params,
    draw_gait_preview,
    load_gait_fields,
    play_preview_animation,
    reset_preview_view,
    stop_preview_animation,
    toggle_preview_animation,
    refresh_coordinated_tab as refresh_coordinated_tab_view,
    refresh_gait_panel as refresh_gait_panel_view,
    sync_binding_editor as sync_binding_editor_view,
)
from motor_control.web_adapter import DesktopWebController

# ============== 轴配置 ==============
# 步进拓扑与单位模型位于 motor_control；这里仅保留闭环电机兼容常量。
NUM_MOTOR_AXES = 2
NUM_AXES = NUM_STEPPER_AXES       # 向后兼容
NUM_GEAR_AXES = NUM_MOTOR_AXES    # 向后兼容

# ============== 步进标定 ==============
# 兼容：默认 200 微步/圈 ÷ 1.0mm 导程 = 200 pulse/mm（28HD140GT81-200LR 贯通式步进）。
PULSES_PER_MM = [DEFAULT_PULSE_PER_REV / DEFAULT_LEAD_MM] * NUM_STEPPER_AXES
# 速度档位（单位/s：直线=mm/s, 旋转=°/s）。
# 2026-08-21 应用户要求扩充：低速加 0.05/0.1/0.2（减速箱微动），
# 高速加 15/20/30（丝杆快移；>1000pps 仍会弹确认窗、>4000pps 钳位提醒）。
SPEED_PRESETS = [0.05, 0.1, 0.2, 0.3, 0.5, 0.6, 1, 1.5, 2, 2.5, 3,
                 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 8, 10, 15, 20, 30]
_speed_to_delay_ms = speed_to_delay_ms

DELAY_DEFAULT_MS = [
    step_speed_to_delay_ms(a, SPEED_DEFAULT, PULSES_PER_MM[a])
    for a in range(NUM_STEPPER_AXES)
]
DIR_OUTWARD = 1
DIR_INWARD  = 0
# 每轴方向翻转标志（电机安装方向不同时用）
# 0 = 不翻转, 1 = 翻转 DIR 信号。两轴原始方向均正确，不翻转。
DIR_INVERT = [0] * NUM_STEPPER_AXES
CONTINUOUS_BURST_MM = 0.2

# 落地纠偏：落地（两直轴回到空闲）后延迟重锁的秒数——留出机构在承载
# 重力下自正中的时间。急停/断开/关闭路径不等待，立即恢复锁定。
# 2026-09-21 应用户要求：由"空闲即锁"改为落地后延迟 5 秒。
LAND_RELOCK_DELAY_S = 5.0


def _continuous_burst_units(profile):
    """Return a responsive continuous-jog burst that is at least one pulse."""

    return max(CONTINUOUS_BURST_MM, profile.units_from_steps(1))

# ============== FOC ==============
FOC_STATE_NAMES = {"0": "失能", "1": "对齐中", "2": "运行", "3": "故障"}
GEAR_STATE_NAMES = {"0": "失能", "1": "—", "2": "运行", "3": "故障"}  # GEAR 没有"对齐"阶段
FOC_POLL_INTERVAL_S = 0.1

# ============== 模式 ==============
MODE_FOC  = "FOC"
MODE_GEAR = "GEAR"

PROJECT_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR        = os.path.join(PROJECT_DIR, "logs")
os.makedirs(LOG_DIR, exist_ok=True)


def _empty_axis_dict():
    return {i: None for i in range(NUM_STEPPER_AXES)}


class _GaitHostAdapter:
    """把 StepperGUI 的运动链路适配成 GaitExecutor 需要的宿主协议。"""

    def __init__(self, app):
        self._app = app

    def role_axis(self, role_name):
        return self._app._gait_role_axis(role_name)

    def send_relative(self, axis, delta, speed):
        return self._app._gait_send_relative(axis, delta, speed)

    def send_synchronized(self, moves, duration_s):
        return self._app._gait_send_synchronized(moves, duration_s)

    def send_synchronized_endpoint(self, moves, duration_s, endpoints):
        return self._app._gait_send_synchronized(moves, duration_s,
                                                cumulative_targets=endpoints)

    def wait_terminal(self, axis, timeout_s):
        return self._app._gait_wait_terminal(axis, timeout_s)

    def stop_axes(self, axes):
        self._app._gait_stop_axes(axes)

    def cancelled(self):
        return self._app._control_worker_cancelled()

    def log(self, message):
        self._app.log(message)


class StepperGUI:
    def __init__(self, root, web_server_factory=None):
        self.root = root
        self.state_store = StateStore(PROJECT_DIR)
        self._startup_warnings = []
        self.root.title("30轴步进（ESP32 + Pico）+ FOC/GEAR + 轨道 D 控制器")
        self.root.resizable(True, True)

        # ── 共享：串口 ──
        self.ser = None
        self.serial_session = None
        self.state_lock = threading.RLock()
        self._disconnect_lock = threading.Lock()
        self._estop_lock = threading.Lock()
        self._control_context = threading.local()
        self._ui_dispatcher = UiDispatcher(self._handle_ui_dispatch_error)
        self.foc_poll_running = False
        self._last_serial_error_log = 0.0
        self._serial_failure_handled = False
        self._serial_generation = 0
        self._control_generation = 0
        self._closing = False
        self._disconnecting = False
        self._disconnect_teardown_in_progress = False
        self._estop_in_progress = False
        self._hardware_estop_active = False
        self._estop_unconfirmed = False

        # ── 每轴状态（list 按 axis 索引）──
        self.running            = [False] * NUM_STEPPER_AXES  # 连续运动 flag
        # 位置与限位使用与显示模式无关的脉冲坐标；mm/° 只在边界换算。
        self.axis_runtime = [
            AxisRuntime(position_trusted=False) for _ in range(NUM_STEPPER_AXES)
        ]
        self.stepper_in_progress= [False] * NUM_STEPPER_AXES
        self._pending_step      = [None]  * NUM_STEPPER_AXES
        self._move_dispatching  = [False] * NUM_STEPPER_AXES
        self._move_reservation  = [None]  * NUM_STEPPER_AXES
        self._web_step_pending  = [None]  * NUM_STEPPER_AXES
        self._axis_motion_generation = [0] * NUM_STEPPER_AXES
        self.axis_motion_telemetry = [
            AxisMotionTelemetry() for _ in range(NUM_STEPPER_AXES)
        ]
        self.control_bindings = BindingSet.empty()
        self.pico_node_health = {
            node: "unknown" for node in range(1, NUM_PICO_NODES + 1)
        }
        self._binding_editor_dirty = False
        self._binding_editor_revision = 0
        self._coordinated_refresh_after = None
        self._foc_enabled_ui    = [False] * NUM_MOTOR_AXES
        self.foc_trace_buf      = [[] for _ in range(NUM_MOTOR_AXES)]
        self._motor_status      = [
            {"state": "?", "current_deg": None, "target_deg": None, "fault": None}
            for _ in range(NUM_MOTOR_AXES)
        ]
        # goto watcher 代数：每次新 goto +1，旧 watcher 检测到 gen 变了就退出
        self._goto_watcher_gen  = [0] * NUM_MOTOR_AXES

        # ── 每轴 Tk 变量 ──
        self.v_dist   = [tk.DoubleVar(value=10.0) for _ in range(NUM_STEPPER_AXES)]
        self.v_dir    = [tk.IntVar   (value=DIR_OUTWARD) for _ in range(NUM_STEPPER_AXES)]
        self.v_delay  = [tk.DoubleVar(value=DELAY_DEFAULT_MS[a]) for a in range(NUM_STEPPER_AXES)]
        self.v_speed_str = [tk.StringVar(value=str(SPEED_DEFAULT)) for _ in range(NUM_STEPPER_AXES)]
        self.v_goto   = [tk.DoubleVar(value=0.0)  for _ in range(NUM_STEPPER_AXES)]
        self.v_foctgt = [tk.DoubleVar(value=0.0)  for _ in range(NUM_MOTOR_AXES)]
        self.v_focstate   = [tk.StringVar(value="未连接") for _ in range(NUM_MOTOR_AXES)]
        self.v_focfault   = [tk.StringVar(value="--")    for _ in range(NUM_MOTOR_AXES)]
        self.v_foccur     = [tk.StringVar(value="--")    for _ in range(NUM_MOTOR_AXES)]
        self.v_focvlimit  = [tk.DoubleVar(value=10.0)    for _ in range(NUM_MOTOR_AXES)]
        self.v_focpangle  = [tk.DoubleVar(value=25.0)    for _ in range(NUM_MOTOR_AXES)]
        self.v_focvp      = [tk.DoubleVar(value=0.2)     for _ in range(NUM_MOTOR_AXES)]
        self.v_focpp      = [tk.IntVar   (value=7)       for _ in range(NUM_MOTOR_AXES)]

        # ── GEAR 模式调参变量（和 FOC 共享 v_foctgt / v_focstate / v_foccur / v_focfault）──
        self.v_gearpwm = [tk.DoubleVar(value=100.0)  for _ in range(NUM_MOTOR_AXES)]   # PWM duty cap %（0-100）
        self.v_gearkp  = [tk.DoubleVar(value=1.0)    for _ in range(NUM_MOTOR_AXES)]
        self.v_gearki  = [tk.DoubleVar(value=0.0)    for _ in range(NUM_MOTOR_AXES)]
        self.v_gearkd  = [tk.DoubleVar(value=0.05)   for _ in range(NUM_MOTOR_AXES)]
        self.v_geargr  = [tk.DoubleVar(value=1000.0) for _ in range(NUM_MOTOR_AXES)]   # 齿轮比（NVS）

        # 轨道 D（独立 DRV8871）状态。命令采用 800 ms 租约，GUI 按住时续租。
        self.v_track_duty = tk.IntVar(value=60)
        self.v_track_status = tk.StringVar(value="已停止")
        self._track_direction = "STOP"
        self._track_lease_ms = 800
        self._track_lease_generation = 0
        self._track_last_response = ""

        # ── 左右直线联动页（左侧直+右侧直，同命令同时发两轴）──
        self.paired_axes = (0, 1)
        self.pair_dist = tk.DoubleVar(value=10.0)
        self.pair_dir = tk.IntVar(value=DIR_OUTWARD)
        self.pair_speed_str = tk.StringVar(value=str(SPEED_DEFAULT))
        self.paired_widgets = {}
        self.pair_speed_str.trace_add(
            "write", self._on_pair_speed_change)

        # ── 落地纠偏：悬空腿落地（两直轴标高差收敛到阈值内）时，
        #    自动释放左右旋转电机（Mr1/Mr2 绑定的轴），让机构在逐渐
        #    承载的重力下扭正一次；落地（两直轴回到空闲）后延迟
        #    LAND_RELOCK_DELAY_S 秒自动重新锁定，留足自正中时间。
        #    释放是纯机械动作：旋转轴不发脉冲，软件位置继续沿用。──
        self.pair_land_release_enabled = tk.BooleanVar(value=True)
        # 2026-09-21 应用户要求：落地阈值 3mm → 6mm（更早进入释放）。
        self.pair_land_release_threshold_mm = tk.DoubleVar(value=6.0)
        self._rot_release_active = False
        self._rot_release_axes: tuple[int, ...] = ()   # 触发时解析的 Mr 绑定轴
        self._rot_relock_after = None        # 延迟重锁定时器句柄（仅 UI 线程碰）
        self._rot_relock_generation = 0      # 递增即可作废未到点的延迟重锁
        self._rot_release_settled = threading.Event()  # ENA,0 已落到串口上
        self._rot_release_settled.set()
        self._last_land_gap_mm: float | None = None

        # ── 模式：
        #    mode_select_var: 用户选择 (Auto/FOC/GEAR)；Auto 时从固件 MODE 命令读
        #    fw_mode_var:      当前实际生效的模式显示 (FOC/GEAR/?)
        self.mode_select_var = tk.StringVar(value="Auto")
        self.fw_mode_var = tk.StringVar(value="?")
        self._fw_mode_cache = "?"

        # ── 每轴 widget refs（dict-per-axis）──
        self.sw = [dict() for _ in range(NUM_STEPPER_AXES)]  # stepper widgets
        self.fw = [dict() for _ in range(NUM_MOTOR_AXES)]    # FOC widgets
        self.gw = [dict() for _ in range(NUM_MOTOR_AXES)]    # GEAR widgets
        self.tw = {}
        self.web_server = None
        self._web_controller = DesktopWebController(self)
        self._autotune_runner = AutotuneRunner(
            send=self._send_and_read,
            trace_provider=lambda axis: list(self.foc_trace_buf[axis]),
            log=self.log,
            sleep=time.sleep,
            axis_label=lambda axis: AXIS_LABEL[axis],
            cancelled=self._control_worker_cancelled,
        )
        self.web_status_var = tk.StringVar(value="● 服务未启动")
        self.web_address_var = tk.StringVar(value="—")
        self.web_port_var = tk.StringVar(value="—")
        self.web_url_var = tk.StringVar(value="网页服务未启动")

        # ── 每轴步进配置：控制模式 + 电机参数（脉冲每转/减速比/导程）──
        self.axis_profiles = [AxisProfile() for _ in range(NUM_STEPPER_AXES)]
        self._axis_profile_load_fallback = set()
        self.axis_param_valid   = [True] * NUM_STEPPER_AXES
        self._speed_clamp_warned = [False] * NUM_STEPPER_AXES
        self.axis_mode_var = [tk.StringVar(value=MODE_LINEAR) for _ in range(NUM_STEPPER_AXES)]
        self.axis_ppr_var  = [tk.DoubleVar(value=DEFAULT_PULSE_PER_REV) for _ in range(NUM_STEPPER_AXES)]
        self.axis_gr_var   = [tk.DoubleVar(value=DEFAULT_GEAR_RATIO) for _ in range(NUM_STEPPER_AXES)]
        self.axis_lead_var = [tk.DoubleVar(value=DEFAULT_LEAD_MM) for _ in range(NUM_STEPPER_AXES)]
        self._load_axis_config()
        self._load_control_bindings()

        # ── 三足轮换步态：参数（标定向导写入 .gait_params.json）+ 当前执行实例 ──
        self.gait_params = GaitParams(trajectory_mode=TWO_MODE)
        self._gait_run: GaitExecutor | None = None
        self._gait_owned = {}
        self._gait_supports = ("A", "B")
        self._gait_beta_deg = 180.0
        self._gait_needs_recovery = False
        self._load_gait_params()

        self._build_ui()
        for warning in self._startup_warnings:
            self.log(warning)
        self._startup_warnings.clear()
        # 参数框手输（不点上下箭头）也必须生效：Spinbox 的 command= 只在点箭头时触发，
        # 因此对三个参数 var 加写入监听。放在 _build_ui 之后绑定，避免 _load_axis_config
        # 初始化 set 时触发回调访问未建成的控件。
        for axis in range(NUM_STEPPER_AXES):
            _param_cb = lambda *_args, _a=axis: self._on_axis_param_change(_a)
            self.axis_ppr_var[axis].trace_add("write", _param_cb)
            self.axis_gr_var[axis].trace_add("write", _param_cb)
            self.axis_lead_var[axis].trace_add("write", _param_cb)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(25, self._drain_ui_actions)
        self._load_calib()
        self._refresh_coordinated_ui()
        self._load_foc_tune()
        self._load_gear_tune()
        if web_server_factory is not None:
            self._start_web_server(web_server_factory)

    # ═════════════ 顶层 UI ═════════════
    def _build_ui(self):
        build_main_ui(self, LOG_DIR, NUM_MOTOR_AXES)

    # ═════════════ 步进 Tab（参数化）═════════════
    def _build_stepper_tab(self, parent, axis):
        build_stepper_tab_view(
            self, parent, axis, SPEED_PRESETS, DIR_OUTWARD, DIR_INWARD)

    def _build_paired_tab(self, parent):
        build_paired_tab_view(
            self, parent, self.paired_axes, SPEED_PRESETS,
            DIR_OUTWARD, DIR_INWARD)

    def _build_coordinated_tab(self, parent):
        build_coordinated_tab_view(self, parent)

    # ═════════════ 四逻辑电机：绑定与只读状态 ══════════════
    def _load_control_bindings(self):
        """Load one complete binding snapshot; malformed data fails safe."""

        try:
            data = self.state_store.load_coordinated_bindings()
        except StateStoreError as exc:
            self._startup_warnings.append(f"⚠️ 逻辑电机绑定读取失败: {exc}")
            self.control_bindings = BindingSet.empty()
            return
        if data is None:
            self.control_bindings = BindingSet.empty()
            return
        try:
            self.control_bindings = parse_binding_document(data)
        except BindingValidationError as exc:
            self.control_bindings = BindingSet.empty()
            self._startup_warnings.append(
                f"⚠️ 逻辑电机绑定配置无效（{exc.code}）: {exc}；已安全回退为未绑定"
            )

    def _safety_state_locked(self):
        if self._hardware_estop_active:
            return "hardware_estop"
        if self._estop_in_progress:
            return "software_estop"
        if self._estop_unconfirmed:
            return "estop_unconfirmed"
        return "normal"

    def _snapshot_coordinated_control_locked(self, *, now=None):
        """Build the desktop/Web shared snapshot while ``state_lock`` is held."""

        return build_coordinated_snapshot(
            bindings=self.control_bindings,
            profiles=self.axis_profiles,
            runtimes=self.axis_runtime,
            axis_param_valid=self.axis_param_valid,
            running=self.running,
            stepper_in_progress=self.stepper_in_progress,
            move_dispatching=self._move_dispatching,
            move_reservations=self._move_reservation,
            web_step_pending=self._web_step_pending,
            pending_steps=self._pending_step,
            telemetry=self.axis_motion_telemetry,
            connected=self._is_serial_connected(),
            safety_state=self._safety_state_locked(),
            pico_node_health=self.pico_node_health,
            now=now,
        )

    def _apply_control_bindings(self, candidate):
        """Atomically validate, persist and publish a complete binding set."""

        if not isinstance(candidate, BindingSet):
            candidate = BindingSet.from_axis_mapping(candidate)
        with self.state_lock:
            if candidate.assignments == self.control_bindings.assignments:
                return self.control_bindings
            issues = binding_compatibility_issues(
                candidate, self.axis_profiles, self.axis_param_valid
            )
            if issues:
                raise BindingValidationError(issues[0].code, issues[0].message)
            affected_axes = set(self.control_bindings.bound_axes)
            affected_axes.update(candidate.bound_axes)
            active = [
                axis for axis in sorted(affected_axes)
                if self._axis_motion_active_locked(axis)
            ]
            if active:
                labels = ", ".join(
                    f"步进轴 {axis}（{AXIS_LABEL[axis]}）" for axis in active
                )
                raise RuntimeError(f"{labels} 正在运动或已预约，停止后才能换绑")
            if (self._rot_release_active
                    and affected_axes.intersection(self._rot_release_axes)):
                raise RuntimeError("旋转电机处于落地纠偏释放中，锁定后才能换绑")

            # The binding file is tiny and local.  Keep the same RLock held
            # through os.replace so no MOVE/config path can enter between the
            # idle check, persistence and in-memory publication.
            reservation = object()
            for axis in affected_axes:
                self._move_reservation[axis] = reservation
            applied = candidate.with_revision(self.control_bindings.revision + 1)
            try:
                self.state_store.save_coordinated_bindings(applied.as_document())
                self.control_bindings = applied
            finally:
                for axis in affected_axes:
                    if self._move_reservation[axis] is reservation:
                        self._move_reservation[axis] = None
        return applied

    def _binding_draft(self):
        values = {}
        for role in LOGICAL_ROLE_ORDER:
            selected = self.control_binding_vars[role.value].get()
            if selected == "未绑定":
                values[role.value] = None
                continue
            if selected not in self._binding_axis_by_option:
                raise BindingValidationError(
                    "invalid_axis_option", f"{role.value} 选择了未知物理轴"
                )
            values[role.value] = self._binding_axis_by_option[selected]
        return BindingSet.from_axis_mapping(values)

    def _show_binding_draft_validation(self):
        statuses = self.coordinated_widgets.get("binding_status", {})
        try:
            candidate = self._binding_draft()
            issues = binding_compatibility_issues(
                candidate, self.axis_profiles, self.axis_param_valid
            )
            issue_by_role = {
                issue.role.value: issue for issue in issues if issue.role is not None
            }
            for role in LOGICAL_ROLE_ORDER:
                binding = candidate.for_role(role)
                issue = issue_by_role.get(role.value)
                if issue is not None:
                    message, color = issue.message, "#b42318"
                elif binding is None:
                    message, color = "未绑定", "#b26a00"
                else:
                    message, color = "草稿有效", "#1565c0"
                statuses[role.value].configure(text=message, foreground=color)
        except BindingValidationError as exc:
            for widget in statuses.values():
                widget.configure(text=str(exc), foreground="#b42318")

    def _on_binding_editor_change(self, _event=None):
        self._binding_editor_dirty = True
        dirty = self.coordinated_widgets.get("dirty_label")
        if dirty is not None:
            dirty.configure(text="草稿未应用", foreground="#b26a00")
        self._show_binding_draft_validation()

    def _fill_suggested_bindings(self):
        suggested = BindingSet.suggested()
        for role in LOGICAL_ROLE_ORDER:
            binding = suggested.for_role(role)
            self.control_binding_vars[role.value].set(
                self._binding_option_by_axis[binding.axis]
            )
        self._on_binding_editor_change()

    def _clear_binding_editor(self):
        if not messagebox.askyesno(
            "解除逻辑绑定",
            "将解除 Mup1/Mr1/Mup2/Mr2 的全部物理步进轴绑定。\n"
            "此操作不会移动电机，也不会清除轴校准。是否继续？",
            icon="warning",
        ):
            return
        for role in LOGICAL_ROLE_ORDER:
            self.control_binding_vars[role.value].set("未绑定")
        self._on_binding_editor_change()
        self._save_binding_editor()

    def _save_binding_editor(self):
        try:
            candidate = self._binding_draft()
            applied = self._apply_control_bindings(candidate)
        except (BindingValidationError, RuntimeError, StateStoreError) as exc:
            self.log(f"⛔ 逻辑电机绑定未保存: {exc}")
            messagebox.showerror("绑定未保存", str(exc))
            self._show_binding_draft_validation()
            return False
        sync_binding_editor_view(self, applied)
        self.log(
            "✓ 逻辑电机绑定已保存: "
            + ", ".join(
                f"{role.value}→"
                + (
                    "未绑定"
                    if applied.for_role(role) is None
                    else f"步进轴 {applied.for_role(role).axis}"
                )
                for role in LOGICAL_ROLE_ORDER
            )
        )
        self._refresh_coordinated_ui(reschedule=False)
        return True

    def _jump_to_binding_axis(self, role=None):
        if role is None:
            roles = LOGICAL_ROLE_ORDER
        else:
            roles = (role,)
        selected_axis = None
        for selected_role in roles:
            option = self.control_binding_vars[selected_role.value].get()
            if option in self._binding_axis_by_option:
                selected_axis = self._binding_axis_by_option[option]
                break
        if selected_axis is None:
            messagebox.showinfo("未选择物理轴", "请先在任意角色行选择一个物理步进轴")
            return
        self.notebook.select(self.tab_index_step[selected_axis])
        group_index = self.stepper_axis_group_indices[selected_axis]
        self.stepper_group_book.select(group_index)
        axis_book = self.stepper_axis_books[selected_axis]
        axis_book.select(self.stepper_axis_tabs[selected_axis])

    def _refresh_coordinated_ui(self, *, reschedule=True):
        if self._closing or not getattr(self, "coordinated_widgets", None):
            return
        with self.state_lock:
            control, snapshots = self._snapshot_coordinated_control_locked()
        refresh_coordinated_tab_view(self, control, snapshots)
        if self._binding_editor_dirty:
            self._show_binding_draft_validation()
        self._refresh_gait_ui()
        if reschedule and not self._closing:
            self._coordinated_refresh_after = self.root.after(
                200, self._refresh_coordinated_ui
            )

    # ═════════════ 三足轮换步态：参数 / 标定 / 分阶段执行 ═════════════
    def _load_gait_params(self):
        """启动时读 .gait_params.json；坏数据安全回退为占位默认。"""

        try:
            data = self.state_store.load_gait_params()
        except StateStoreError as exc:
            self._startup_warnings.append(f"⚠️ 步态参数读取失败: {exc}")
            return
        if data is None:
            return
        try:
            # Pulse coordinates do not establish physical support locations on restart.
            self.gait_params = replace(parse_gait_params(data), calibration_confirmed=False)
        except ValueError as exc:
            self._startup_warnings.append(
                f"⚠️ 步态参数无效（{exc}）；已回退为默认占位参数")
            self.gait_params = GaitParams()

    def _save_gait_params(self, params):
        """校验、原子写盘并发布到内存（UI 线程调用）。"""

        params = params.validated()
        if getattr(self, "_gait_owned", {}):
            raise GaitExecutorError("步态执行中禁止改参数/标定，请先中止或完成")
        self.state_store.save_gait_params(params.as_document())
        self.gait_params = params
        return params

    def _gait_hardware_fingerprint(self):
        with self.state_lock:
            value = [(role.value, self._gait_role_axis(role.value),
                      repr(self.axis_profiles[self._gait_role_axis(role.value)]))
                     for role in LOGICAL_ROLE_ORDER]
        return hashlib.sha256(json.dumps(value).encode("utf-8")).hexdigest()

    def _gait_zero_signature(self, role, params):
        axis = self._gait_role_axis(role)
        sign = params.mr1_sign if role == "Mr1" else params.mr2_sign
        return repr((axis, self.axis_profiles[axis], sign))

    def _gait_reestablish_baseline(self):
        if getattr(self, "_gait_owned", {}):
            messagebox.showwarning("步态占用", "请先中止并确认四轴停止")
            return
        if not messagebox.askokcancel("重建物理基准", "必须人工确认：左足在A、右足在B，三爪均踩低节点；四轴静止、位置已校准。\n此按钮不移动电机。确认后须重新记两侧旋转零位并确认标定。"):
            return
        with self.state_lock:
            if any(self._axis_motion_active_locked(a) for a in self.control_bindings.bound_axes):
                return
            self._gait_supports, self._gait_beta_deg = ("A", "B"), 180.0
            self._gait_needs_recovery = False
        self._gait_run = None
        self._save_gait_params(replace(self.gait_params, mr1_zero_deg=None, mr2_zero_deg=None,
                                      calibration_confirmed=False, beam_reference_deg=180.0))
        load_gait_fields(self)
        self._refresh_gait_ui()

    def _gait_role_axis(self, role_name):
        """逻辑角色 → 物理轴；未绑定/模式不符抛 GaitExecutorError。"""

        try:
            role = LogicalRole(role_name)
        except ValueError as exc:
            raise GaitExecutorError(f"未知逻辑角色 {role_name}") from exc
        binding = self.control_bindings.for_role(role)
        if binding is None:
            raise GaitExecutorError(f"{role_name} 未绑定物理步进轴")
        axis = binding.axis
        spec = ROLE_SPECS[role]
        with self.state_lock:
            mode = self.axis_profiles[axis].mode
            param_valid = self.axis_param_valid[axis]
        if mode != spec.required_mode:
            required = "旋转" if spec.required_mode == MODE_ROTARY else "直线"
            actual = "旋转" if mode == MODE_ROTARY else "直线"
            raise GaitExecutorError(
                f"{role_name} 需要{required}模式，"
                f"轴{AXIS_LABEL[axis]} 当前为{actual}模式")
        if not param_valid:
            raise GaitExecutorError(
                f"{role_name} 绑定的轴{AXIS_LABEL[axis]} 参数无效")
        return axis

    def _gait_record_role_zero(self, role_name):
        """UI 线程：把绑定 Mr 轴当前软件位置记为 ψ=30° 基准零位。"""

        if role_name not in ("Mr1", "Mr2"):
            raise GaitExecutorError(f"{role_name} 不是旋转角色，无零位标定")
        axis = self._gait_role_axis(role_name)
        if getattr(self, "_gait_owned", {}):
            raise GaitExecutorError("步态拥有四轴时不能修改零位；请先中止/完成")
        with self.state_lock:
            if self._axis_motion_active_locked(axis):
                raise GaitExecutorError(f"{role_name} 正在运动，不能记零")
            runtime = self.axis_runtime[axis]
            if not runtime.position_trusted:
                raise GaitExecutorError(
                    f"{role_name} 位置不可信；先设原点或校准后再记零")
            position = self.axis_profiles[axis].units_from_steps(
                runtime.position_steps)
        updates = {}
        if role_name == "Mr1":
            updates["mr1_zero_deg"] = position
            updates["mr1_zero_signature"] = self._gait_zero_signature(role_name, self.gait_params)
        else:
            updates["mr2_zero_deg"] = position
            updates["mr2_zero_signature"] = self._gait_zero_signature(role_name, self.gait_params)
        self._save_gait_params(replace(self.gait_params, calibration_confirmed=False, **updates))
        self.log(f"✓ {role_name} 零位已记录：轴坐标 {position:+.3f}"
                 f"（对应 ψ 基准 {LOW_NODE_PHASE_DEG:g}°）")
        return position

    def _gait_swing_theta_deg(self, side, params):
        """摆动侧 Mr 累计角 θ = 轴坐标 − 零位（≈线缆缠绕量，度）。

        绑定缺失、零位未标或位置不可信时返回 None（调用方按基准位 0 处理）。
        """

        role = "Mr1" if side == "left" else "Mr2"
        try:
            axis = self._gait_role_axis(role)
        except GaitExecutorError:
            return None
        zero = (params.mr1_zero_deg if role == "Mr1"
                else params.mr2_zero_deg)
        with self.state_lock:
            runtime = self.axis_runtime[axis]
            if not runtime.position_trusted:
                return None
            position = self.axis_profiles[axis].units_from_steps(
                runtime.position_steps)
        return position - (zero if zero is not None else 0.0)

    def _gait_landing_route(self, side, params, *, pads=None,
                            arc_deg=SWING_ARC_DEG):
        """按当前站位与横梁角匹配该侧几何落点（默认顺向 +60°）。

        2026-09-21 按用户要求放开区域限制：目标座不再固定取 A/B/C 的
        第三块，而是在全部支座（含邻座）中按方位/半径匹配；初始站位
        (A,B) 的右侧换位因此落到邻座。干跑预览与实机执行共用本方法，
        预览图与实际动作才是同一几何。arc_deg 是换位方向（+60 顺向 /
        -60 逆向），预览与执行传同一值。匹配不到返回 None（调用方决定
        拦截还是回退）。
        """

        supports = getattr(self, "_gait_supports", ("A", "B"))
        beta_now = getattr(self, "_gait_beta_deg", params.beam_reference_deg)
        start, pivot = supports if side == "left" else supports[::-1]
        bearing = beta_now + (180.0 if side == "right" else 0.0)
        geometry = effective_geometry(params)
        if pads is None:
            pads = gait_pads(params, pivot)
        landing_bearing = (bearing - arc_deg + 180.0) % 360.0 - 180.0
        for name, pad in pads.items():
            if name in (start, pivot):
                continue
            pad_bearing = math.degrees(math.atan2(
                pad.center[1] - pads[pivot].center[1],
                pad.center[0] - pads[pivot].center[0]))
            bearing_err = abs((pad_bearing - landing_bearing + 180) % 360 - 180)
            radius_err = abs(math.dist(pad.center, pads[pivot].center)
                             - geometry.d_mm)
            if bearing_err <= 0.5 and radius_err <= 0.5:
                return (start, name, pivot, bearing)
        return None

    def _gait_begin_run(self, side, params=None, arc_deg=None):
        """UI 线程：前置检查 + 干跑 + 按当前相位建立执行器。

        arc_deg 显式传入时按传入方向执行（四个执行按钮各携带方向）；
        None 时读步态页方向选择。返回 (executor, dry_run_report)；
        任何门槛不满足抛 GaitExecutorError。
        """

        params = (params if params is not None else self.gait_params).validated()
        auto_avoidance = params.trajectory_mode == TWO_MODE
        geometry = effective_geometry(params)
        if side not in ("left", "right"):
            raise GaitExecutorError("side 必须是 left 或 right")
        # 2026-09-22 按用户要求：实机移动与预览的四种换位方式一致。
        # 方向优先取调用方显式传入；未传时读步态页选择。值无效直接
        # 拦截，绝不静默换向。（AttributeError 回退顺向仅服务无控件
        # 的测试 harness。）
        if arc_deg is None:
            try:
                arc_selection = float(self.gait_arc_var.get())
            except AttributeError:
                arc_selection = SWING_ARC_DEG
            except (tk.TclError, ValueError, TypeError):
                arc_selection = None
        else:
            arc_selection = arc_deg
        if (arc_selection is None or not math.isfinite(arc_selection)
                or not -360.0 < arc_selection < 360.0 or arc_selection == 0):
            raise GaitExecutorError("换位方向无效；请用对应的【开始…移】按钮")
        arc_deg = arc_selection
        if auto_avoidance and not math.isclose(abs(arc_deg), SWING_ARC_DEG):
            raise GaitExecutorError("两模态避杆只支持相邻支座 ±60° 换位")
        if not self._is_serial_connected():
            raise GaitExecutorError("串口未连接")
        with self.state_lock:
            if any(getattr(self, key, False) for key in ("_closing", "_disconnecting",
                   "_estop_in_progress", "_hardware_estop_active", "_estop_unconfirmed")):
                raise GaitExecutorError("控制器正在关闭/急停或停车未确认")
        if getattr(self, "_gait_owned", {}):
            raise GaitExecutorError("上一次步态尚未完成/中止，不能覆盖执行器")
        if not params.calibration_confirmed:
            raise GaitExecutorError("尚未确认电机方向、PPR、零位及现场支撑条件；请在步态页的【标定】中确认")
        if getattr(self, "_gait_needs_recovery", False):
            raise GaitExecutorError("上一次动作未完整完成；必须人工重建 A/B 物理基准、重新标定")
        if params.calibration_fingerprint != self._gait_hardware_fingerprint():
            raise GaitExecutorError("绑定或PPR/减速比已变化，旧零位标定失效，请重新标定")
        if self._rot_release_active:
            raise GaitExecutorError("旋转轴释放状态未解除，不能执行步态")
        if not params.geometry.surrounding_pads:
            raise GaitExecutorError("实际步态不能关闭邻接六边形高点检查")
        # 1. 四个逻辑角色全部绑定且模式正确
        role_axes = {}
        for role in LOGICAL_ROLE_ORDER:
            role_axes[role.value] = self._gait_role_axis(role.value)
        with self.state_lock:
            for role, a in role_axes.items():
                if self._axis_motion_active_locked(a) or not self.axis_runtime[a].position_trusted:
                    raise GaitExecutorError(f"{role} 忙碌/位置不可信，禁止开始")
        if role_axes["Mr1"] >= 6 or role_axes["Mr2"] >= 6:
            raise GaitExecutorError("同步旋转必须绑定 ESP32 本地轴；Pico 未实现公共时基，不允许降级")
        if self._send_and_read("SYNC,S") != "OK,SYNC,V1,6":
            raise GaitExecutorError("固件缺少六轴SYNC V1能力；先更新主线固件，不能抬足后才发现不支持")
        for a in (role_axes["Mr1"], role_axes["Mr2"]):
            if self._send_and_read(build_ena_command(a)) != f"OK,ENA,{a},1":
                raise GaitExecutorError("旋转驱动器未确认使能；禁止抬足/联动")
        # 2. 两侧旋转轴都已记零（相位换算的基准）
        for role_name, zero in (("Mr1", params.mr1_zero_deg),
                                ("Mr2", params.mr2_zero_deg)):
            if zero is None:
                raise GaitExecutorError(
                    f"{role_name} 尚未做零位标定；请先把三足摆到基准位"
                    f"（ψ=30°）并【记零】")
            saved_signature = params.mr1_zero_signature if role_name == "Mr1" else params.mr2_zero_signature
            if saved_signature != self._gait_zero_signature(role_name, params):
                raise GaitExecutorError(f"{role_name} 零位与绑定/方向/PPR不匹配，必须重新记零")
        # 3. 摆动侧旋转轴当前相位与累计角（决定 S3 相位调整和解绕小步）
        swing_role = "Mr1" if side == "left" else "Mr2"
        axis = role_axes[swing_role]
        with self.state_lock:
            if self._axis_motion_active_locked(axis):
                raise GaitExecutorError(
                    f"{swing_role}（轴{AXIS_LABEL[axis]}）正在运动")
            runtime = self.axis_runtime[axis]
            if not runtime.position_trusted:
                raise GaitExecutorError(
                    f"{swing_role} 位置不可信；请先重新校准")
            position = self.axis_profiles[axis].units_from_steps(
                runtime.position_steps)
        sign = params.mr1_sign if swing_role == "Mr1" else params.mr2_sign
        zero = params.mr1_zero_deg if swing_role == "Mr1" else params.mr2_zero_deg
        beta_now = getattr(self, "_gait_beta_deg", params.beam_reference_deg)
        psi_now = (LOW_NODE_PHASE_DEG + sign * (position - zero)
                   + beta_now - params.beam_reference_deg)
        support_role = "Mr2" if side == "left" else "Mr1"
        support_axis = role_axes[support_role]
        support_zero = params.mr2_zero_deg if side == "left" else params.mr1_zero_deg
        support_sign = params.mr2_sign if side == "left" else params.mr1_sign
        with self.state_lock:
            support_pos = self.axis_profiles[support_axis].units_from_steps(
                self.axis_runtime[support_axis].position_steps)
        support_psi = (LOW_NODE_PHASE_DEG + support_sign * (support_pos - support_zero)
                       + beta_now - params.beam_reference_deg)
        if abs((support_psi - LOW_NODE_PHASE_DEG + 60) % 120 - 60) > 0.5:
            raise GaitExecutorError("支撑足世界相位未对准低节点；不能把电机相对横梁角当作绝对姿态")
        supports = getattr(self, "_gait_supports", ("A", "B"))
        start, pivot = supports if side == "left" else supports[::-1]
        bearing = beta_now + (180.0 if side == "right" else 0.0)
        pads = gait_pads(params, pivot)
        expected_bearing = math.degrees(math.atan2(
            pads[start].center[1] - pads[pivot].center[1],
            pads[start].center[0] - pads[pivot].center[0]))
        if abs((bearing - expected_bearing + 180) % 360 - 180) > 0.5:
            raise GaitExecutorError("横梁角与当前支座位置不一致；请重新建立 A/B 标定基准")
        # 2026-09-21 按用户要求放开区域限制：目标座按所选方向几何落点
        # 在全部支座（含邻座）中匹配（与干跑预览共用 _gait_landing_route）。
        route = self._gait_landing_route(side, params, pads=pads, arc_deg=arc_deg)
        if route is None:
            raise GaitExecutorError(
                f"该侧{'顺' if arc_deg > 0 else '逆'}向{abs(arc_deg):g}°落点处"
                "没有六边形支座；请核实站位与横梁角，或先交换摆动/支撑侧")
        report = plan_swing_trajectory(
            params, side=side, route=route, arc_deg=arc_deg)
        if auto_avoidance and not report.feasible:
            raise GaitExecutorError("两模态腿—高杆避让未通过，禁止抬足：" + report.message)
        self.log(f"步态角度联动校验（{route[0]}→{route[1]}，支点{route[2]}，"
                 f"{'顺' if arc_deg > 0 else '逆'}向{abs(arc_deg):g}°）：{report.message}")
        # legacy 保留历史碰撞仅提示行为；两模态的腿—高杆门槛已在上方
        # 拦截，额外结构诊断仍提示。行程、固件、标定等操作门槛仍拦截。
        clearance_warnings: list[str] = []
        if not report.feasible:
            clearance_warnings.append(f"干跑校验不可行：{report.message}")
        stages = plan_gait_stages(params, side=side, swing_psi_start_deg=psi_now,
                                  arc_deg=arc_deg, route=route)
        # S3 也有扫掠风险，按相同包络检查整个原地相位调整，不只检查 S4。
        phase_stage = next((s for s in stages if s.stage_id == "S3"), None)
        if phase_stage is not None:
            delta_psi = phase_stage.move_groups[0][0].delta * sign
            phase_margin = min(clearance_margin_mm(
                pads[start].center, psi_now + delta_psi * i / 360.0,
                params.geometry, report.hexagons, lift_mm=params.lift_mm,
                pivot=pads[pivot].center) for i in range(361))
            phase_margin -= params.geometry.arm_length_mm * math.radians(abs(delta_psi)) / 720.0
            if phase_margin <= 0:
                clearance_warnings.append(
                    f"S3 原地相位调整可能扫掠高点（最小间隙 {phase_margin:.2f}mm）")
            if auto_avoidance:
                phase_margin = min(leg_clearance(
                    pads[start].center, psi_now+delta_psi*i/360,
                    geometry, report.hexagons)[0] for i in range(361))
                phase_margin -= geometry.arm_length_mm*math.radians(abs(delta_psi))/720
        p_swing, p_support = self.axis_profiles[axis], self.axis_profiles[support_axis]
        # One shared master tick, rounded total counts and DDA rounding contribute
        # at most two pulse quanta per joint. Reserve their geometric displacement.
        phi_error = 2.0 / p_support.pulses_per_unit
        psi_error = phi_error + 2.0 / p_swing.pulses_per_unit
        pulse_envelope = math.radians(phi_error) * geometry.d_mm + math.radians(psi_error) * geometry.arm_length_mm
        # 2026-09-21 起 S2 两直轴同时抬升，量化包络须覆盖两条直轴。
        for lift_axis in (role_axes["Mup1"], role_axes["Mup2"]):
            pulse_envelope += 0.5 / self.axis_profiles[lift_axis].pulses_per_unit
        phase_residual = abs((psi_now - LOW_NODE_PHASE_DEG + 60) % 120 - 60)
        if phase_stage is not None:
            phase_residual = 0.5 / p_swing.pulses_per_unit
        pulse_envelope += geometry.arm_length_mm * math.radians(phase_residual)
        pulse_envelope += geometry.d_mm * math.radians(abs((bearing - expected_bearing + 180) % 360 - 180))
        if report.min_margin_mm <= pulse_envelope or (phase_stage is not None and phase_margin <= pulse_envelope):
            if auto_avoidance:
                raise GaitExecutorError(
                    f"腿—高杆净间隙不足以覆盖脉冲/标定误差包络 {pulse_envelope:.3f}mm；"
                    "禁止执行，请核实实测半径、间隙及脉冲分辨率")
            clearance_warnings.append(
                f"避障余量（最小间隙 {report.min_margin_mm:.2f}mm）可能不足以覆盖"
                f"脉冲量化包络 {pulse_envelope:.2f}mm；建议提高细分/减速比或修正几何")
        # 预检整个行程与两侧线缆角限制；不允许发出抬足后才发现公转越界。
        with self.state_lock:
            if self._gait_hardware_fingerprint() != params.calibration_fingerprint:
                raise GaitExecutorError("预检期间配置已变化")
            if any(self._axis_motion_active_locked(a) or not self.axis_runtime[a].position_trusted
                   for a in role_axes.values()):
                raise GaitExecutorError("预检期间轴状态已变化")
            targets = {a: self.axis_runtime[a].position_steps for a in role_axes.values()}
            for role, a in role_axes.items():
                p, r = self.axis_profiles[a], self.axis_runtime[a]
                if ((r.min_steps is not None and r.position_steps < r.min_steps)
                        or (r.max_steps is not None and r.position_steps > r.max_steps)):
                    raise GaitExecutorError(f"{role} 当前坐标已在行程外，请先恢复标定")
                if role.startswith("Mr"):
                    z = params.mr1_zero_deg if role == "Mr1" else params.mr2_zero_deg
                    if abs(p.units_from_steps(r.position_steps)-z) > params.rotation_limit_deg:
                        raise GaitExecutorError(f"{role} 当前已在线缆窗口外")
            for stage in stages:
                stage_origin = dict(targets)
                for group_index, group in enumerate(stage.move_groups):
                    sync_counts = []
                    for move_index, move in enumerate(group):
                        a = role_axes[move.role]
                        p, r = self.axis_profiles[a], self.axis_runtime[a]
                        signed_delta = math.copysign(p.command_steps_from_units(abs(move.delta)), move.delta)
                        if stage.sync_endpoints:
                            endpoint = stage.sync_endpoints[group_index][move_index]
                            target = stage_origin[a] + math.copysign(p.command_steps_from_units(abs(endpoint)), endpoint)
                            signed_delta = target-targets[a]
                        count = int(abs(signed_delta))
                        if (count < 1 and not stage.sync_endpoints) or move.speed * p.pulses_per_unit > PULSE_RATE_WARN_PPS:
                            raise GaitExecutorError(f"{move.role} {stage.stage_id} 脉冲分辨率/速度不符合安全要求")
                        if count > 20000000:
                            raise GaitExecutorError("步态运动超过固件单段脉冲数限制")
                        targets[a] += signed_delta
                        direction = DIR_OUTWARD if signed_delta * outward_position_sign(a) > 0 else DIR_INWARD
                        sync_counts.append((a, count if direction ^ DIR_INVERT[a] else -count))
                        if ((r.min_steps is not None and targets[a] < r.min_steps)
                                or (r.max_steps is not None and targets[a] > r.max_steps)):
                            raise GaitExecutorError(f"{move.role} {stage.stage_id} 超过已标定行程")
                        if move.role.startswith("Mr"):
                            z = params.mr1_zero_deg if move.role == "Mr1" else params.mr2_zero_deg
                            if abs(p.units_from_steps(targets[a]) - z) > params.rotation_limit_deg + 1e-8:
                                raise GaitExecutorError(f"{move.role} 角度轨迹超过线缆窗口；必须另行校验悬空解绕，不能缩短公转自转轨迹")
                    if stage.synchronized:
                        try:
                            build_sync_command(sync_counts[0][0], sync_counts[0][1],
                                               sync_counts[1][0], sync_counts[1][1], round(
                                                   (stage.group_durations[group_index] if stage.group_durations
                                                    else stage.duration_s)*1000000))
                        except ProtocolEncodingError as exc:
                            raise GaitExecutorError(f"同步轨迹时间/脉冲数超出固件能力：{exc}") from exc
            token = object()
            for a in role_axes.values():
                self._move_reservation[a] = token
            self._gait_owned = {a: token for a in role_axes.values()}
            twin_lift_start = {role: self.axis_profiles[role_axes[role]].units_from_steps(
                self.axis_runtime[role_axes[role]].position_steps) for role in ("Mup1", "Mup2")}
        self._gait_run = GaitExecutor(
            _GaitHostAdapter(self), params, stages, side=side)
        self._gait_run.route = route
        self._gait_run.arc_deg = arc_deg
        self._gait_run.rotation_start = None
        self._gait_run.twin_lift_start = twin_lift_start
        self._gait_run.twin_final_positions = None
        self._gait_run.twin_manual_invalid = False
        self.log(f"步态执行器就绪（{side}）：{len(stages)} 个阶段，"
                 f"当前 ψ={psi_now:.1f}°")
        if clearance_warnings:
            text = "\n".join(clearance_warnings)
            self.log(f"⚠️ 碰撞/避障校验未通过（仅提示，不拦截）：{text}")
            messagebox.showwarning(
                "碰撞/避障校验未通过（仅提示）",
                text + "\n\n按要求不再拦截执行；请确认现场安全，"
                "运动中随时可【⛔ 中止】。")
        return self._gait_run, report

    def _gait_release_ownership(self):
        with self.state_lock:
            for a, token in getattr(self, "_gait_owned", {}).items():
                if self._move_reservation[a] is token:
                    self._move_reservation[a] = None
            self._gait_owned = {}

    def _gait_angle_snapshot(self):
        """Estimated (not measured) world/beam/joint angles from shared pulse progress."""
        run = getattr(self, "_gait_run", None)
        if run is None or run.rotation_start is None:
            return None
        try:
            if run.params.calibration_fingerprint != self._gait_hardware_fingerprint():
                return None
        except GaitExecutorError:
            return None
        swing, support = ("Mr1", "Mr2") if run.side == "left" else ("Mr2", "Mr1")
        offsets = {}
        with self.state_lock:
            for role in (swing, support):
                a = self._gait_role_axis(role)
                r, p = self.axis_runtime[a], self.axis_profiles[a]
                position = projected_step_position(r.position_steps, self._pending_step[a],
                                                   self.axis_motion_telemetry[a])
                position = r.position_steps if position is None else position
                sign = run.params.mr1_sign if role == "Mr1" else run.params.mr2_sign
                offsets[role] = sign * p.units_from_steps(position - run.rotation_start[a])
            phi = offsets[support]
            beta_start = run.route[3] - (180 if run.side == "right" else 0)
        return {"phi_deg": phi, "beta_deg": beta_start - phi,
                "psi_delta_deg": offsets[swing] - phi,
                "swing_q_delta_deg": offsets[swing], "support_q_delta_deg": offsets[support],
                "measured": False}

    def _gait_twin_snapshot(self):
        """Read-only visual model; its output never enters motion/safety decisions."""
        from motor_control.gait_twin import build_twin_snapshot

        with self.state_lock:
            _control, axes = self._snapshot_coordinated_control_locked()
            params = self.gait_params
            try:
                reference_valid = (params.calibration_fingerprint == self._gait_hardware_fingerprint()
                         and params.mr1_zero_deg is not None and params.mr2_zero_deg is not None
                         and params.mr1_zero_signature == self._gait_zero_signature("Mr1", params)
                         and params.mr2_zero_signature == self._gait_zero_signature("Mr2", params))
            except GaitExecutorError:
                reference_valid = False
            valid = params.calibration_confirmed and reference_valid
            run = getattr(self, "_gait_run", None)
            context = None
            if (run is not None and hasattr(run, "twin_lift_start") and reference_valid
                    and run.params.calibration_fingerprint == params.calibration_fingerprint
                    and all(getattr(run.params, key) == getattr(params, key) for key in (
                        "mr1_zero_deg", "mr2_zero_deg", "mr1_sign", "mr2_sign",
                        "mup1_lift_sign", "mup2_lift_sign", "beam_reference_deg"))):
                if run.twin_final_positions is not None and any(
                        a.get("target_position") is not None
                        or a.get("position") != run.twin_final_positions.get(a["role"])
                        for a in axes):
                    # After independent manual/Web moves the grounded pivot is
                    # unknown. Keep axis readouts, do not invent a world pose.
                    run.twin_manual_invalid = True
                rotation_start = (None if run.rotation_start is None else {
                    role: self.axis_profiles[self._gait_role_axis(role)].units_from_steps(
                        run.rotation_start[self._gait_role_axis(role)]) for role in ("Mr1", "Mr2")})
                context = {"side": run.side, "route": run.route,
                           "rotation_start": rotation_start, "lift_start": run.twin_lift_start,
                           "reference_key": id(run), "manual_invalid": run.twin_manual_invalid}
            return build_twin_snapshot(params, axes, context=context,
                                       calibration_valid=valid,
                                       needs_recovery=getattr(self, "_gait_needs_recovery", False))

    def _gait_execute_stage(self):
        """UI 线程：启动控制工作线程执行当前运动阶段。"""

        run = self._gait_run
        if run is None:
            messagebox.showwarning("步态未开始", "请先选择方向并【开始】一次摆动")
            return
        stage = run.current_stage()
        if stage is None:
            return
        if not stage.is_motion_stage:
            # 确认型阶段由 UI 层直接 advance；这里只处理运动阶段。
            return
        run.begin_stage_execution()   # 抢占状态，防重复点击
        worker = self._start_control_worker(self._gait_stage_worker)
        if worker is None:
            run.revert_stage_execution()
            self.log("⛔ 控制器正在停止或急停，阶段未执行")
        self._refresh_gait_ui()

    def _gait_stage_worker(self):
        run = self._gait_run
        if run is None:
            return

        def progress(stage_id, done_groups, total_groups):
            self._post_ui(
                lambda: self._refresh_gait_ui(
                    f"{stage_id} 第 {done_groups}/{total_groups} 组"))

        try:
            # 步态中的落脚释放可能还处在延迟重锁窗口内：旋转阶段开始前
            # 先恢复锁定并等固件确认，绝不带着失能的旋转轴走 SYNC。
            self._ensure_rot_axes_locked_blocking()
            run.execute_current_stage(progress)
        except Exception as exc:
            self.log(f"⛔ 步态阶段未执行：{exc}")
            run.request_stop()
            run._fail("failed", f"执行异常：{exc}")
            self._gait_stop_axes(self.control_bindings.bound_axes)
        finally:
            if run.state in ("aborted", "failed"):
                self._gait_needs_recovery = True
                self.gait_params = replace(self.gait_params, calibration_confirmed=False)
                self._gait_release_ownership()
            self._post_ui(lambda: self._refresh_gait_ui())

    def _gait_abort_run(self):
        """UI 线程：中止按钮——请求退出 + 立即停掉四个绑定轴。"""

        run = self._gait_run
        if run is None:
            return
        if run.state == "done":
            # 2026-09-24 走完一步后随手点【中止】不触发"须重建基准"：
            # 流程已完成，站位/横梁角已更新、四轴已释放，没有东西可停。
            self.log("步态流程已完成，无需中止；可直接开始下一次换位（或点【重置】清理显示）")
            return
        run.request_stop()
        self._gait_needs_recovery = True
        run._fail("aborted", "步态已中止，须人工重建物理基准")
        axes = []
        for role in LOGICAL_ROLE_ORDER:
            try:
                axes.append(self._gait_role_axis(role.value))
            except GaitExecutorError:
                continue
        self._gait_stop_axes(axes)
        self._gait_release_ownership()
        self.log("⛔ 步态已中止：四个逻辑轴都已发 STOP；"
                 "如有轴被 ABORT，位置不可信，需重新校准后才能继续")

    def _refresh_gait_ui(self, progress_text=None):
        """刷新步态面板（面板在协调页构建后存在）。"""

        if self._closing:
            return
        refresh = getattr(self, "_refresh_gait_panel", None)
        if refresh is not None:
            refresh(progress_text)

    # ── GaitExecutor 宿主协议的实现（控制工作线程调用） ──
    def _gait_send_relative(self, axis, delta, speed):
        """下发一段相对运动（ACK 即返回）；sent / noop / failed。"""

        with self.state_lock:
            profile = self.axis_profiles[axis]
        try:
            steps = profile.command_steps_from_units(abs(delta))
        except (TypeError, ValueError, OverflowError):
            return "failed"
        if steps <= 0:
            return "noop"
        ppu = profile.pulses_per_unit
        pps = abs(speed) * ppu
        if pps > PULSE_RATE_WARN_PPS:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]} 步态速度 {speed:g}"
                     f"{profile.speed_unit} 折合 {pps:,.0f} pps，超过安全参考 "
                     f"{PULSE_RATE_WARN_PPS:.0f} pps，已拒绝；请调低步态速度")
            return "failed"
        delay_ms = step_speed_to_delay_ms(axis, abs(speed), ppu)
        direction = (
            DIR_OUTWARD
            if delta * outward_position_sign(axis) > 0
            else DIR_INWARD
        )
        token = getattr(self, "_gait_owned", {}).get(axis)
        if token is None:
            return "failed"
        guard = lambda: self._move_reservation[axis] is token and not self._control_worker_cancelled()
        return "sent" if self._send_mm_reserved(
            axis, abs(delta), direction, delay_ms, profile=profile, guard=guard) else "failed"

    def _gait_send_synchronized(self, moves, duration_s, *, cumulative_targets=None):
        """Preflight/reserve BOTH axes before one SYNC write; no serial motion fallback."""
        if len(moves) != 2 or self._control_worker_cancelled():
            return "failed"
        prepared = []
        with self.state_lock:
            run = getattr(self, "_gait_run", None)
            if cumulative_targets is not None and (run is None or len(cumulative_targets) != 2):
                return "failed"
            origin = (getattr(run, "rotation_start", None) or
                      {a: self.axis_runtime[a].position_steps for a, _, _ in moves})
            for move_index, (axis, delta, speed) in enumerate(moves):
                if not 0 <= axis < 6 or not math.isfinite(delta) or not math.isfinite(speed) or speed <= 0:
                    return "failed"
                p, r = self.axis_profiles[axis], self.axis_runtime[axis]
                token = getattr(self, "_gait_owned", {}).get(axis)
                if (token is None or self._move_reservation[axis] is not token
                        or not r.position_trusted or not self.axis_param_valid[axis]
                        or p.mode != MODE_ROTARY or self._pending_step[axis] is not None
                        or self.stepper_in_progress[axis] or self._move_dispatching[axis]):
                    return "failed"
                count = p.command_steps_from_units(abs(delta))
                signed = count if delta > 0 else -count
                if cumulative_targets is not None:
                    endpoint = cumulative_targets[move_index]
                    if not math.isfinite(endpoint):
                        return "failed"
                    signed = int(origin[axis] + math.copysign(p.command_steps_from_units(abs(endpoint)), endpoint)
                                 - r.position_steps)
                    count = abs(signed)
                if count <= 0 and cumulative_targets is None:
                    return "failed"
                target = r.position_steps + signed
                if ((r.min_steps is not None and target < r.min_steps)
                        or (r.max_steps is not None and target > r.max_steps)):
                    return "failed"
                if speed * p.pulses_per_unit > min(5000, PULSE_RATE_WARN_PPS):
                    return "failed"
                direction = DIR_OUTWARD if signed * outward_position_sign(axis) > 0 else DIR_INWARD
                wire_count = count if direction ^ DIR_INVERT[axis] else -count
                prepared.append((axis, signed, wire_count, token))
            a, b = prepared
            try:
                command = build_sync_command(a[0], a[2], b[0], b[2], round(duration_s * 1_000_000))
            except (ProtocolEncodingError, TypeError, ValueError, OverflowError):
                return "failed"
            for axis, signed, _, _ in prepared:
                self._move_dispatching[axis] = True
                self.stepper_in_progress[axis] = True
                self._pending_step[axis] = signed
                self.axis_motion_telemetry[axis] = (AxisMotionTelemetry.starting(
                    self._axis_motion_generation[axis], abs(signed)) if signed else
                    AxisMotionTelemetry(generation=self._axis_motion_generation[axis],
                                        state="STARTING", requested_steps=0, executed_steps=0,
                                        started_monotonic=time.monotonic(), updated_monotonic=time.monotonic()))
            if run is not None and run.rotation_start is None:
                self._gait_run.rotation_start = {axis: self.axis_runtime[axis].position_steps
                                                for axis, _, _, _ in prepared}

        def guard():
            with self.state_lock:
                return (not self._control_worker_cancelled()
                        and all(self._move_reservation[axis] is token for axis, _, _, token in prepared))

        uncertain = False
        try:
            response = self._send_and_read(command, guard=guard, propagate_request_error=True)
        except RequestCancelled:
            response = ""
        except (RequestTimeout, SerialSessionError):
            response, uncertain = "", True
        except Exception as exc:
            self.log(f"SYNC 下发异常：{exc}")
            response, uncertain = "", True
        accepted = response == f"OK,SYNC,{a[0]},{b[0]}"
        # Generic ERR lacks a transaction id; after a write, even a stale ERR
        # must not leave the host claiming a trusted position.
        uncertain = uncertain or not accepted
        with self.state_lock:
            for axis, signed, _, _ in prepared:
                self._move_dispatching[axis] = False
                if not accepted:
                    if uncertain:
                        self.axis_runtime[axis].position_trusted = False
                    if self._pending_step[axis] is not None:
                        self._pending_step[axis] = None
                        self.stepper_in_progress[axis] = False
                        self.axis_motion_telemetry[axis] = self.axis_motion_telemetry[axis].reset(
                            result="UNKNOWN" if uncertain else "REJECTED")
        self.log(f"角度同步轨迹 {command} → {response or '未确认'}")
        return "sent" if accepted else "failed"

    def _gait_wait_terminal(self, axis, timeout_s):
        """轮询等待一轴到达终态；DONE / ABORTED / TIMEOUT / CANCELLED。"""

        deadline = time.monotonic() + timeout_s
        while True:
            if (self._control_worker_cancelled()
                    or (getattr(self, "_gait_run", None) is not None and self._gait_run._stop_requested)):
                return "CANCELLED"
            with self.state_lock:
                busy = (
                    self.stepper_in_progress[axis]
                    or self._pending_step[axis] is not None
                    or self._move_dispatching[axis]
                )
                result = self.axis_motion_telemetry[axis].last_result
            if not busy:
                return result if result in ("DONE", "ABORTED") else "TIMEOUT"
            if time.monotonic() >= deadline:
                return "TIMEOUT"
            time.sleep(0.02)

    def _gait_stop_axes(self, axes):
        """立即失效并停轴（UI/工作线程都可调用；发送放独立线程）。"""

        for axis in sorted(set(axes)):
            stop_reservation = self._begin_axis_stop(axis)
            if self._is_serial_connected():
                threading.Thread(
                    target=self._send_axis_stop,
                    args=(axis, stop_reservation),
                    daemon=True,
                ).start()
            else:
                with self.state_lock:
                    if self._move_reservation[axis] is stop_reservation:
                        self._move_reservation[axis] = None
            self.log(f"⛔ 轴{AXIS_LABEL[axis]} 步态中止停止")

    # ── 步态页 UI 回调（Tk 线程） ──
    def _build_gait_tab(self, parent):
        build_gait_tab_view(self, parent)

    def _refresh_gait_panel(self, progress_text=None):
        if self._closing:
            return
        refresh_gait_panel_view(self, progress_text)

    def _gait_params_from_ui(self):
        """收集输入框 → 校验后的 GaitParams；非法时弹窗并返回 None。"""

        if getattr(self, "_gait_owned", {}):
            messagebox.showwarning("步态占用", "执行期间不能保存/更换参数，请先完成或中止")
            return None
        try:
            return collect_gait_params(self, self.gait_params)
        except (ValueError, GaitExecutorError) as exc:
            messagebox.showerror("步态参数无效", str(exc))
            return None

    def _gait_save_params_clicked(self):
        params = self._gait_params_from_ui()
        if params is None:
            return
        self._save_gait_params(params)
        self.log("✓ 步态参数已保存到 .gait_params.json")
        self._refresh_gait_ui()

    def _gait_reload_params_clicked(self):
        load_gait_fields(self)
        self.log("步态输入框已重置为已保存值")
        self._refresh_gait_ui()

    def _gait_record_zero_clicked(self, role_name):
        params = self._gait_params_from_ui()
        if params is None:
            return
        try:
            self._save_gait_params(replace(params, calibration_confirmed=False))
            self._gait_record_role_zero(role_name)
        except GaitExecutorError as exc:
            messagebox.showerror("记零失败", str(exc))
            return
        self.gait_calibrated_var.set(False)
        self._refresh_gait_ui()

    def _gait_run_dry_run(self, interactive=True):
        params = self._gait_params_from_ui()
        if params is None:
            return
        self._save_gait_params(params)
        side = self.gait_side_var.get()
        # 2026-09-22 应用户要求预览/执行支持顺/逆双向：四种换位方式
        # （左/右 × 顺/逆）与实机执行一致。另算反向落点，底图把
        # 起点/支点/顺逆两个落点都画出（共边相连）。
        try:
            arc_deg = float(self.gait_arc_var.get())
        except (AttributeError, tk.TclError, ValueError):
            arc_deg = SWING_ARC_DEG
        if not math.isfinite(arc_deg) or arc_deg == 0 or abs(arc_deg) > 360.0:
            arc_deg = SWING_ARC_DEG
        route = self._gait_landing_route(side, params, arc_deg=arc_deg)
        alt_route = self._gait_landing_route(side, params, arc_deg=-arc_deg)
        self._gait_preview_alt_pad = (alt_route[1] if alt_route else None)
        if route is None:
            if params.trajectory_mode == TWO_MODE:
                stop_preview_animation(self)
                self._gait_last_report = None
                self.gait_report_var.set("当前站位该方向没有目标支座；未生成替代轨迹")
                draw_gait_preview(self)
                return
            self.log(f"⚠️ 干跑：当前站位该侧{'顺' if arc_deg > 0 else '逆'}向"
                     f"{abs(arc_deg):g}°落点没有六边形支座，"
                     "预览退回固定 A→C/B→A 示意路由")
        report = plan_swing_trajectory(
            params, side=side, route=route, arc_deg=arc_deg)
        # 新轨迹新包围盒：滚轮缩放/中键平移视图复位，避免旧视图卡住新图
        reset_preview_view(self)
        self._gait_last_report = report
        self._gait_last_report_key = (side, arc_deg)
        self._gait_preview_stance = (getattr(self, "_gait_supports", ("A", "B")),
                                    getattr(self, "_gait_beta_deg", 180.0))
        self.gait_report_var.set(report.message)
        start, target, pivot, _bearing = report.route
        direction = "顺向" if arc_deg > 0 else "逆向"
        alt_text = (f"，反向落点 {self._gait_preview_alt_pad}"
                    if self._gait_preview_alt_pad else "")
        self.log(
            f"步态角度预览（{side}·{direction}，{start}→{target} 绕{pivot}"
            f"{alt_text}，轨迹模式 {params.trajectory_mode}）：{report.message}")
        stop_preview_animation(self)
        draw_gait_preview(self)
        if not report.feasible and interactive:
            messagebox.showwarning(
                "干跑不可行",
                report.message + "\n\n请修正几何/半径/间隙参数后再试。")

    def _gait_play_preview_clicked(self):
        """▶ 模拟动作：逐帧回放（只动视图）；播放中点=暂停，暂停点=继续。"""

        # 2026-09-23 应用户要求："停止模拟"会整页闪回静态预览，改为
        # 暂停——画面停在当前帧，再点继续；只有轨迹/方向变化才彻底停。
        anim = self.gait_widgets.get("preview_anim")
        if anim is not None and (anim.get("playing") or anim.get("paused")):
            toggle_preview_animation(self)
            return
        # 2026-09-22 修复：切换左/右或顺/逆后直接点播放会重播旧报告。
        # 播放前核对报告与当前选择，不一致先按当前选择重新干跑。
        try:
            selection = (self.gait_side_var.get(),
                         float(self.gait_arc_var.get()))
        except (AttributeError, tk.TclError, ValueError):
            selection = None
        stance = (getattr(self, "_gait_supports", ("A", "B")),
                  getattr(self, "_gait_beta_deg", 180.0))
        if (getattr(self, "_gait_last_report", None) is None
                or selection is None
                or getattr(self, "_gait_last_report_key", None) != selection
                or getattr(self, "_gait_preview_stance", stance) != stance):
            self._gait_run_dry_run(interactive=False)
        play_preview_animation(self)

    def _gait_start_run_clicked(self, side, arc_deg=None):
        params = self._gait_params_from_ui()
        if params is None:
            return
        self._save_gait_params(params)
        try:
            _run, report = self._gait_begin_run(side, params, arc_deg=arc_deg)
        except GaitExecutorError as exc:
            messagebox.showerror("不能开始步态", str(exc))
            return
        # 预览与执行共用画面：把预览选择同步到本次实际执行的换位方式，
        # 图上显示的就是即将/正在走的动作。
        run = self._gait_run
        self.gait_side_var.set(side)
        self.gait_arc_var.set(f"{run.arc_deg:g}")
        mode_var = getattr(self, "gait_mode_var", None)
        if mode_var is not None:
            mode_var.set(("L" if side == "left" else "R")
                         + ("+" if run.arc_deg > 0 else "-"))
        self._gait_last_report = report
        self._gait_last_report_key = (side, run.arc_deg)
        self._gait_preview_stance = (self._gait_supports, self._gait_beta_deg)
        self.gait_report_var.set(report.message)
        reset_preview_view(self)
        stop_preview_animation(self)
        draw_gait_preview(self)
        self._refresh_gait_ui()

    def _gait_stage_confirmed(self):
        run = self._gait_run
        if run is None:
            return
        if run.state == "running":
            return
        if run._stop_requested:
            self._gait_abort_run()
            self._refresh_gait_ui()
            return
        stage = run.current_stage()
        if stage is None:
            self._refresh_gait_ui()
            return
        verb = "执行" if stage.is_motion_stage else "确认完成"
        if not messagebox.askokcancel(
                f"{stage.stage_id} {stage.title}",
                stage.confirm_text + f"\n\n确定{verb}本阶段？"):
            return
        if stage.is_motion_stage:
            self._gait_execute_stage()
        elif run.advance_confirm():
            self.log(f"✓ {stage.stage_id} {stage.title} 人工确认通过")
            if run.state == "done":
                start, target, pivot, _bearing = run.route
                self._gait_supports = (target, pivot) if run.side == "left" else (pivot, target)
                support_role = "Mr2" if run.side == "left" else "Mr1"
                a = self._gait_role_axis(support_role)
                support_sign = run.params.mr2_sign if run.side == "left" else run.params.mr1_sign
                actual_phi = support_sign * self.axis_profiles[a].units_from_steps(
                    self.axis_runtime[a].position_steps - run.rotation_start[a])
                self._gait_beta_deg -= actual_phi
                run.twin_final_positions = {
                    role.value: self.axis_profiles[self._gait_role_axis(role.value)].units_from_steps(
                        self.axis_runtime[self._gait_role_axis(role.value)].position_steps)
                    for role in LOGICAL_ROLE_ORDER}
                self._gait_release_ownership()
        self._refresh_gait_ui()

    def _gait_abort_clicked(self):
        self._gait_abort_run()
        self._refresh_gait_ui()

    def _gait_reset_run(self):
        if getattr(self, "_gait_owned", {}):
            self._gait_abort_run()
        self._gait_run = None
        self.log("步态流程已重置（执行器丢弃，重新开始前会重新干跑）")
        self._refresh_gait_ui()

    # ═════════════ FOC Tab（参数化）═════════════
    def _build_foc_tab(self, parent, axis):
        build_foc_tab_view(self, parent, axis)

    # ═════════════ 轨道 D（独立 DRV8871）═════════════
    def _build_track_tab(self, parent):
        build_track_tab_view(self, parent)


    def _set_track_controls(self, state):
        for key in ("duty", "fwd_btn", "rev_btn", "stop_btn"):
            widget = self.tw.get(key)
            if widget is not None:
                widget.config(state=state)

    def _track_press(self, direction):
        try:
            duty = max(1, min(100, int(self.v_track_duty.get())))
        except (TypeError, ValueError, tk.TclError):
            duty = 60
            self.v_track_duty.set(duty)
        self._start_track_lease(direction, duty)

    def _start_track_lease(self, direction, duty):
        if direction not in ("FWD", "REV"):
            return False
        if not self._is_serial_connected():
            self.v_track_status.set("未连接")
            return False
        with self.state_lock:
            if (self._hardware_estop_active or self._estop_in_progress
                    or self._estop_unconfirmed or self._closing
                    or self._disconnecting):
                self.v_track_status.set("控制器正在停止或急停")
                return False
            self._track_lease_generation += 1
            generation = self._track_lease_generation
            control_generation = self._control_generation
            self._track_direction = direction
            self._track_lease_ms = 800
        self.v_track_status.set(f"{direction} · PWM {duty}% · 租约续租中")
        self.tw['status_label'].config(foreground="#1565c0")
        self._track_lease_tick(
            generation, direction, duty, control_generation
        )
        return True

    def _track_lease_tick(
        self, generation, direction, duty, control_generation
    ):
        with self.state_lock:
            if (generation != self._track_lease_generation
                    or self._track_direction != direction
                    or control_generation != self._control_generation
                    or self._hardware_estop_active):
                return

        def lease_is_current():
            with self.state_lock:
                return (generation == self._track_lease_generation
                        and self._track_direction == direction
                        and control_generation == self._control_generation
                        and not self._hardware_estop_active)

        def worker():
            response = self._send_and_read(
                build_track_command(direction, duty, 800),
                timeout=0.25,
                guard=lease_is_current,
            )
            with self.state_lock:
                current = (generation == self._track_lease_generation
                           and self._track_direction == direction
                           and control_generation == self._control_generation)
                if current:
                    self._track_last_response = response
                    if response != "OK,TRACK,D":
                        self._track_direction = "STOP"
                        current = False
            if response == "OK,TRACK,D" and current:
                self._post_ui(lambda: self.root.after(
                    250, lambda: self._track_lease_tick(
                        generation, direction, duty, control_generation
                    )))
            elif response and response != "OK,TRACK,D":
                def show_failure():
                    with self.state_lock:
                        if (generation != self._track_lease_generation
                                or self._track_direction != "STOP"
                                or self._track_last_response != response):
                            return
                    self.v_track_status.set(f"轨道命令失败: {response}")

                self._post_ui(show_failure)

        threading.Thread(target=worker, daemon=True).start()

    def _track_release(self):
        with self.state_lock:
            self._track_lease_generation += 1
            generation = self._track_lease_generation
            self._track_direction = "STOP"
        self.v_track_status.set("已停止")
        if self.tw.get('status_label'):
            self.tw['status_label'].config(foreground="#555")
        threading.Thread(target=self._send_track_stop,
                         args=(generation,), daemon=True).start()

    def _send_track_stop(self, generation=None, control_guard=None):
        def guard():
            with self.state_lock:
                track_ok = (generation is None or
                            (generation == self._track_lease_generation
                             and self._track_direction == "STOP"))
                return track_ok and (control_guard is None or control_guard())
        response = self._send_and_read(
            build_track_command("STOP"), timeout=0.6, guard=guard)
        with self.state_lock:
            if (generation is None or
                    (generation == self._track_lease_generation
                     and self._track_direction == "STOP")):
                self._track_last_response = response
        return response

    # ═════════════ 共享辅助 ═════════════
    def _post_ui(self, callback):
        """从工作线程安全投递 Tk 操作；HTTP/串口线程不得直接调用 Tk。"""
        self._ui_dispatcher.post(callback)

    def _call_ui(self, callback, timeout=2.0):
        """在 Tk 线程执行操作并等待结果，供需要确认已生效的 API 使用。"""
        return self._ui_dispatcher.call(callback, timeout=timeout)

    def _handle_ui_dispatch_error(self, exc):
        """异步 UI 更新失败必须可诊断，不能静默表现为按钮无响应。"""
        message = f"⚠️ UI 更新失败: {exc}"
        try:
            self.log(message)
        except Exception:
            traceback.print_exception(type(exc), exc, exc.__traceback__, file=sys.stderr)

    def _start_control_worker(self, target, *args, **kwargs):
        """启动绑定当前控制代数的写命令线程；ESTOP 后旧代数自动失效。"""
        with self.state_lock:
            if (self._estop_in_progress or self._hardware_estop_active
                    or self._estop_unconfirmed or self._closing
                    or self._disconnecting):
                self.log("控制器正在停止或急停，已拒绝新的 GUI 控制命令")
                return None
            generation = self._control_generation

        def runner():
            self._control_context.generation = generation
            try:
                target(*args, **kwargs)
            finally:
                try:
                    del self._control_context.generation
                except AttributeError:
                    pass

        thread = threading.Thread(target=runner, daemon=True)
        thread.start()
        return thread

    def _control_worker_cancelled(self):
        generation = getattr(self._control_context, "generation", None)
        with self.state_lock:
            return (
                generation is not None
                and (
                    generation != self._control_generation
                    or self._closing
                    or self._disconnecting
                    or self._estop_in_progress
                    or self._hardware_estop_active
                    or self._estop_unconfirmed
                )
            )

    def _drain_ui_actions(self):
        self._ui_dispatcher.drain(limit=100)
        try:
            self.root.after(25, self._drain_ui_actions)
        except tk.TclError:
            pass

    def _on_speed_select(self, axis):
        try:
            speed = float(self.v_speed_str[axis].get())
        except ValueError:
            return
        if speed <= 0: return
        self._set_delay_from_speed(axis, speed)
        self._update_speed_label(axis)

    def _on_root_mousewheel(self, event):
        """鼠标位于日志区时滚动日志，不改变当前键盘焦点。"""
        try:
            target = self.root.winfo_containing(event.x_root, event.y_root)
            if target is not self.log_text or event.delta == 0:
                return None
            units = -1 if event.delta > 0 else 1
            self.log_text.yview_scroll(units, "units")
            return "break"
        except tk.TclError:
            return None

    def _update_speed_label(self, axis):
        d = self.v_delay[axis].get()
        self.sw[axis]['delay_label'].config(text=f"({d:.2f} ms/脉冲)")

    def _on_step_progress(self, axis, done, total):
        if not 0 <= axis < NUM_STEPPER_AXES:
            return
        with self.state_lock:
            pending = self._pending_step[axis]
            if (
                pending is not None
                and total > 0
                and total == int(abs(pending))
            ):
                self.axis_motion_telemetry[axis] = (
                    self.axis_motion_telemetry[axis].with_progress(done, total)
                )
            elif pending == 0 and total == 0 and done == 0 and axis in getattr(self, "_gait_owned", {}):
                # Stationary partner still belongs to the atomic SYNC pair.
                self.axis_motion_telemetry[axis] = replace(
                    self.axis_motion_telemetry[axis], state="MOVING",
                    executed_steps=0, updated_monotonic=time.monotonic())
        if 0 <= axis < len(self.sw) and 'progress' in self.sw[axis]:
            pct = int(done * 100 / total) if total > 0 else 0
            self.sw[axis]['progress']['value'] = pct
            self.sw[axis]['progress_label'].config(text=f"{pct}%  ({done}/{total} 步)")
        self._land_release_tick(axis)

    def _reset_progress(self, axis):
        if axis < len(self.sw) and 'progress' in self.sw[axis]:
            self.sw[axis]['progress']['value'] = 0
            self.sw[axis]['progress_label'].config(text="")

    # ═════════════ 步进：每轴配置（模式/减速比/导程）═════════════
    def _pulses_per_unit(self, axis):
        """当前模式下每单位（直线=mm, 旋转=度）对应的脉冲数。"""
        return self.axis_profiles[axis].pulses_per_unit

    def _unit_label(self, axis):
        return self.axis_profiles[axis].unit

    def _unit_per_s(self, axis):
        return self.axis_profiles[axis].speed_unit

    @staticmethod
    def _convert_axis_value(value, old_pulses_per_unit, new_pulses_per_unit):
        """按等效脉冲数在 mm/° 表示之间转换位置或软件限位。"""
        return convert_axis_value(value, old_pulses_per_unit, new_pulses_per_unit)

    def _on_axis_mode_change(self, axis):
        mode = self.axis_mode_var[axis].get()
        old_profile = self.axis_profiles[axis]
        old_mode = old_profile.mode
        if mode not in (MODE_LINEAR, MODE_ROTARY):
            return False
        if mode == old_mode:
            return True

        with self.state_lock:
            bindings = getattr(self, "control_bindings", BindingSet.empty())
            bound_role = bindings.role_for_axis(axis)
        if (
            bound_role is not None
            and mode != ROLE_SPECS[bound_role].required_mode
        ):
            self.axis_mode_var[axis].set(old_mode)
            required = (
                "直线" if ROLE_SPECS[bound_role].required_mode == MODE_LINEAR else "旋转"
            )
            message = (
                f"步进轴 {axis} 已绑定 {bound_role.value}，必须保持{required}模式；"
                "请先在“电机绑定与状态”页解除绑定"
            )
            self.log(f"⛔ {message}")
            messagebox.showwarning("绑定模式受保护", message)
            return False

        with self.state_lock:
            moving = (self.running[axis] or self.stepper_in_progress[axis]
                      or self._move_dispatching[axis]
                      or self._move_reservation[axis] is not None
                      or self._web_step_pending[axis] is not None
                      or self._pending_step[axis] is not None)
        if moving:
            self.axis_mode_var[axis].set(old_mode)
            message = f"轴{AXIS_LABEL[axis]}正在运动，停止后才能切换直线/旋转模式"
            self.log(f"⛔ {message}")
            messagebox.showwarning("暂不能切换模式", message)
            return False

        # 参数必须先在旧模式下完成 UI 线程校验，避免用无效值换算坐标。
        if not self._require_axis_params(axis):
            self.axis_mode_var[axis].set(old_mode)
            return False
        with self.state_lock:
            # _require_axis_params may run long enough for a web MOVE to
            # reserve this axis.  Recheck and commit under the reservation
            # lock, and keep the freshly validated PPR/GR/lead values.
            moving = (self.running[axis] or self.stepper_in_progress[axis]
                      or self._move_dispatching[axis]
                      or self._move_reservation[axis] is not None
                      or self._web_step_pending[axis] is not None
                      or self._pending_step[axis] is not None)
            if moving:
                applied_profile = None
                trusted = self.axis_runtime[axis].position_trusted
            else:
                current_profile = self.axis_profiles[axis]
                applied_profile = current_profile.with_mode(mode)
                self.axis_profiles[axis] = applied_profile
                trusted = self.axis_runtime[axis].position_trusted
        if applied_profile is None:
            self.axis_mode_var[axis].set(old_mode)
            message = f"轴{AXIS_LABEL[axis]}正在运动，停止后才能切换直线/旋转模式"
            self.log(f"⛔ {message}")
            messagebox.showwarning("暂不能切换模式", message)
            return False

        old_unit = old_profile.unit
        new_unit = applied_profile.unit

        # AxisRuntime 始终保存 steps；切换模式只替换显示配置，位置/限位
        # 不再先换算成另一种浮点单位再写回。
        self._apply_axis_param_ui(axis)
        self._refresh_axis_unit_labels(axis)
        self._update_pos_label(axis)
        self._update_range_display(axis)
        self._on_axis_param_change(axis)
        self._save_axis_config()
        self._save_calib()
        trust_text = "位置可信状态已保留" if trusted else "原位置本就不可信，仍需校准"
        self.log(f"轴{AXIS_LABEL[axis]} {old_unit}→{new_unit}，已按等效脉冲换算坐标/限位；{trust_text}")
        return True

    @staticmethod
    def _coerce_axis_param(value, minimum, maximum, label):
        try:
            return coerce_finite_in_range(value, minimum, maximum, label)
        except ValueError as exc:
            raise ValueError(f"{label}必须在 {minimum:g}–{maximum:g} 范围内") from exc

    def _on_axis_param_change(self, axis):
        try:
            ppr = self._coerce_axis_param(
                self.axis_ppr_var[axis].get(), 1.0, 10000.0, "脉冲/转")
            gear_ratio = self._coerce_axis_param(
                self.axis_gr_var[axis].get(), 0.001, 1000.0, "减速比")
            lead_mm = self._coerce_axis_param(
                self.axis_lead_var[axis].get(), 0.01, 100.0, "导程")
        except (tk.TclError, TypeError, ValueError):
            with self.state_lock:
                self.axis_param_valid[axis] = False
            return False
        with self.state_lock:
            current_mode = self.axis_profiles[axis].mode
            updated_profile = AxisProfile(
                mode=current_mode,
                pulse_per_rev=ppr,
                gear_ratio=gear_ratio,
                lead_mm=lead_mm,
            )
            moving = self._axis_motion_active_locked(axis)
            if moving and updated_profile != self.axis_profiles[axis]:
                # Keep the active/queued command's immutable unit contract.
                # The edited Tk value remains visible and will be applied by
                # the next explicit action after the axis is idle.
                self.axis_param_valid[axis] = False
                return False
            self.axis_profiles[axis] = updated_profile
            self.axis_param_valid[axis] = True
        try:
            speed = float(self.v_speed_str[axis].get())
        except (tk.TclError, ValueError):
            speed = SPEED_DEFAULT
        self._set_delay_from_speed(axis, speed)
        self._update_speed_label(axis)
        self._update_pos_label(axis)
        self._update_range_display(axis)
        self._save_axis_config()
        self._save_calib()
        return True

    def _require_axis_params(self, axis):
        """仅从 UI 线程调用；运动前拒绝无效或尚未输入完成的参数。"""
        if self._on_axis_param_change(axis):
            return True
        message = (f"轴{AXIS_LABEL[axis]}参数无效：脉冲/转 1–10000，"
                   "减速比 0.001–1000，导程 0.01–100")
        self.log(f"⛔ {message}")
        messagebox.showerror("轴参数无效", message)
        return False

    def _speed_safety_ok(self, axis):
        """仅从 UI 线程调用；换算脉冲率超过安全参考值时弹窗确认。"""
        try:
            delay_ms = float(self.v_delay[axis].get())
        except (tk.TclError, TypeError, ValueError):
            return True  # 数值异常交由参数校验/发送路径处理
        if not math.isfinite(delay_ms) or delay_ms <= 0:
            return True
        pps = step_delay_ms_to_pulse_rate(axis, delay_ms)
        if pps <= PULSE_RATE_WARN_PPS:
            return True
        message = (
            f"轴{AXIS_LABEL[axis]}当前速度折合约 {pps:,.0f} 脉冲/秒，"
            f"超过安全参考值 {PULSE_RATE_WARN_PPS:.0f} 脉冲/秒。\n\n"
            "无加速曲线的开环步进在此速率下极易失步（啸叫、行程远小于设定），\n"
            "带减速箱的电机尤其如此。请检查该轴的模式/减速比/导程\n"
            "是否与电机的实际机械结构匹配（例如减速旋转轴误用直线模式）。\n\n仍要执行吗？")
        self.log(f"⚠️ 轴{AXIS_LABEL[axis]} 脉冲率 {pps:,.0f} pps 超过安全参考 "
                 f"{PULSE_RATE_WARN_PPS:.0f} pps，已弹窗确认")
        return messagebox.askokcancel("速度过高确认", message)

    def _set_delay_from_speed(self, axis, speed):
        """速度→固件延时；请求速度被固件上限钳位时给出一次性日志提醒。"""
        if not math.isfinite(speed) or speed <= 0:
            return
        ppu = self._pulses_per_unit(axis)
        delay_ms = step_speed_to_delay_ms(axis, speed, ppu)
        self.v_delay[axis].set(delay_ms)
        clamped = step_speed_clamps_delay(axis, speed, ppu)
        if clamped and not self._speed_clamp_warned[axis]:
            self.log(
                f"⚠️ 轴{AXIS_LABEL[axis]} 速度 {speed:g} {self._unit_per_s(axis)} 折合 "
                f"{speed * ppu:,.0f} pps，超出固件上限，实际将按 "
                f"{step_delay_ms_to_pulse_rate(axis, delay_ms):,.0f} pps "
                "钳位执行（比设定更慢）")
        self._speed_clamp_warned[axis] = clamped

    def _apply_axis_param_ui(self, axis):
        state = "normal" if self.axis_profiles[axis].mode == MODE_LINEAR else "disabled"
        sw = self.sw[axis]
        if 'lead_spin' in sw:
            sw['lead_spin'].config(state=state)
            sw['lead_label'].config(state=state)

    def _refresh_axis_unit_labels(self, axis):
        unit = self._unit_label(axis)
        sw = self.sw[axis]
        noun = "角度" if self.axis_profiles[axis].mode == MODE_ROTARY else "距离"
        if 'dist_label' in sw: sw['dist_label'].config(text=f"{noun} ({unit}):")
        if 'speed_unit_label' in sw: sw['speed_unit_label'].config(text=self._unit_per_s(axis))
        if 'goto_label' in sw: sw['goto_label'].config(text=f"前往位置 ({unit}):")
        if 'set_home_btn' in sw: sw['set_home_btn'].config(text=f"⌂ 设为原点 (0 {unit})")

    def _save_axis_config(self):
        data = {
            "mode": [profile.mode for profile in self.axis_profiles],
            "pulse_per_rev": [profile.pulse_per_rev for profile in self.axis_profiles],
            "gear_ratio": [profile.gear_ratio for profile in self.axis_profiles],
            "lead_mm": [profile.lead_mm for profile in self.axis_profiles],
        }
        try:
            self.state_store.save_axis_config(data)
        except StateStoreError as exc:
            self.log(f"⚠️ 轴配置保存失败: {exc}")

    def _load_axis_config(self):
        try:
            data = self.state_store.load_axis_config()
        except StateStoreError as exc:
            self._startup_warnings.append(f"⚠️ 轴配置读取失败: {exc}")
            return
        if data is None:
            return
        mode_list = data.get("mode", [])
        ppr_list = data.get("pulse_per_rev", [])
        gr_list = data.get("gear_ratio", [])
        lead_list = data.get("lead_mm", [])
        loaded_lists = {
            "mode": mode_list,
            "pulse_per_rev": ppr_list,
            "gear_ratio": gr_list,
            "lead_mm": lead_list,
        }
        for field, value in loaded_lists.items():
            if not isinstance(value, list):
                self._startup_warnings.append(
                    f"⚠️ 轴配置字段 {field} 不是数组，已忽略"
                )
                self._axis_profile_load_fallback.update(
                    range(NUM_STEPPER_AXES)
                )
                loaded_lists[field] = []
        mode_list = loaded_lists["mode"]
        ppr_list = loaded_lists["pulse_per_rev"]
        gr_list = loaded_lists["gear_ratio"]
        lead_list = loaded_lists["lead_mm"]
        for axis in range(NUM_STEPPER_AXES):
            profile = self.axis_profiles[axis]
            mode = profile.mode
            ppr = profile.pulse_per_rev
            gear_ratio = profile.gear_ratio
            lead_mm = profile.lead_mm
            if axis < len(mode_list) and mode_list[axis] in (MODE_LINEAR, MODE_ROTARY):
                mode = mode_list[axis]
            elif axis < len(mode_list):
                self._axis_profile_load_fallback.add(axis)
            if axis < len(ppr_list):
                try:
                    ppr = self._coerce_axis_param(
                        ppr_list[axis], 1.0, 10000.0, "脉冲/转")
                except (ValueError, TypeError):
                    self._axis_profile_load_fallback.add(axis)
            if axis < len(gr_list):
                try:
                    gear_ratio = self._coerce_axis_param(
                        gr_list[axis], 0.001, 1000.0, "减速比")
                except (ValueError, TypeError):
                    self._axis_profile_load_fallback.add(axis)
            if axis < len(lead_list):
                try:
                    lead_mm = self._coerce_axis_param(
                        lead_list[axis], 0.01, 100.0, "导程")
                except (ValueError, TypeError):
                    self._axis_profile_load_fallback.add(axis)
            self.axis_profiles[axis] = AxisProfile(
                mode=mode,
                pulse_per_rev=ppr,
                gear_ratio=gear_ratio,
                lead_mm=lead_mm,
            )
            self.axis_mode_var[axis].set(mode)
            self.axis_ppr_var[axis].set(ppr)
            self.axis_gr_var[axis].set(gear_ratio)
            self.axis_lead_var[axis].set(lead_mm)

    def _update_pos_label(self, axis):
        runtime = self.axis_runtime[axis]
        trusted = runtime.position_trusted
        position = runtime.position_in(self.axis_profiles[axis])
        unit = self._unit_label(axis)
        text = f"{position:.1f} {unit}" if trusted else f"≈ {position:.1f} {unit}（需校准）"
        color = "blue" if trusted else "#d84315"
        self.sw[axis]['pos_label'].config(text=text, foreground=color)
        pair_label = self.paired_widgets.get(f'pos_label_{axis}')
        if pair_label is not None:
            pair_label.config(text=text, foreground=color)

    def _enable_stepper_buttons(self, axis, state):
        sw = self.sw[axis]
        keys = ['move_btn', 'jog_out_btn', 'jog_in_btn', 'cont_out_btn', 'cont_in_btn',
                'stop_btn', 'set_home_btn', 'go_home_btn', 'goto_btn', 'calib_btn',
                'set_min_btn', 'set_max_btn', 'clear_range_btn']
        for k in keys:
            sw[k].config(state=state)

    # ═════════════ 串口 ═════════════
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.set(ports[0])

    def toggle_connect(self):
        with self.state_lock:
            teardown_in_progress = self._disconnect_teardown_in_progress
        if teardown_in_progress:
            messagebox.showinfo("正在断开", "串口仍在回收，请稍后再连接")
            return
        if self._is_serial_connected():
            # 正常断开前停止所有输出；否则断开串口后电机仍可能继续执行或保持使能。
            with self.state_lock:
                self._disconnecting = True
            if not self._stop_all_outputs(allow_closing=True):
                messagebox.showerror(
                    "停车未确认",
                    "控制器没有确认 ESTOP。串口仍可用时将保持连接并封锁新命令；"
                    "请立即使用物理急停或断电，确认安全后重试断开。",
                )
                return
            self._disconnect_serial("用户断开串口", is_error=False)
            return
        try:
            self._serial_failure_handled = False
            with self.state_lock:
                self._disconnecting = False
                self._hardware_estop_active = False
                self._estop_unconfirmed = True
                self.pico_node_health = {
                    node: "unknown" for node in range(1, NUM_PICO_NODES + 1)
                }
            opened_serial = serial.Serial(
                self.port_var.get(), int(self.baud_var.get()), timeout=0.2
            )
            with self.state_lock:
                self.ser = opened_serial
                self._serial_generation += 1
                generation = self._serial_generation
            time.sleep(1.0)
            while opened_serial.in_waiting:
                opened_serial.readline()
            new_session = SerialSession(
                opened_serial,
                event_handler=lambda event: self._handle_serial_event(
                    event, generation, new_session
                ),
                unsolicited_handler=lambda message: self._handle_serial_unsolicited(
                    message, generation, new_session
                ),
                error_handler=lambda exc: self._handle_serial_failure(
                    exc, "会话", generation, new_session
                ),
                trace_handler=lambda direction, line: self._trace_serial_line(
                    direction, line, generation, new_session
                ),
            )
            with self.state_lock:
                self.serial_session = new_session
            new_session.start()
            if not self._stop_all_outputs():
                raise RuntimeError(
                    "连接安全握手失败：固件未精确确认 OK,ESTOP"
                )
            self.conn_status.config(text="● 已连接", foreground="green")
            self.conn_btn.config(text="断开")
            for a in range(NUM_STEPPER_AXES):
                self._enable_stepper_buttons(a, "normal")
                self._update_pos_label(a)
            self._enable_paired_buttons("normal")
            self._set_track_controls("normal")
            # 新会话位置从校准文件重载，旧差值基准跨会话无意义。
            self._last_land_gap_mm = None
            self.log(f"已连接 {self.port_var.get()} @ {self.baud_var.get()}")
            self.foc_poll_running = True
            threading.Thread(
                target=self._foc_poll_loop,
                args=(generation, new_session),
                daemon=True,
            ).start()
            # 自动查固件模式，并根据模式灰掉另一组 tab + 下发对应模式的 tune
            self._query_mode_and_apply()
        except Exception as e:
            self._disconnect_serial(str(e), is_error=True)
            messagebox.showerror("连接失败", str(e))

    def _is_serial_connected(self):
        session = self.serial_session
        return bool(session and session.is_open)

    def _disconnect_serial(
        self,
        reason="",
        is_error=False,
        expected_generation=None,
        expected_session=None,
    ):
        """统一停止读写/轮询并关闭串口；可安全地从任意线程重复调用。"""
        # 关会话前先把落地纠偏释放的旋转轴恢复锁定（best-effort）。
        # 必须放在 _disconnect_lock 外：发送失败会经 _handle_serial_failure
        # 重入本方法，持锁自递归会死锁（_disconnect_lock 不可重入）。
        self._force_relock_rot_axes(allow_closing=True)
        with self._disconnect_lock:
            with self.state_lock:
                if expected_generation is not None and (
                    expected_generation != self._serial_generation
                    or expected_session is not self.serial_session
                ):
                    return
            if self._serial_failure_handled and is_error:
                return
            if is_error:
                self._serial_failure_handled = True
            self.foc_poll_running = False
            with self.state_lock:
                self._serial_generation += 1
                disconnect_generation = self._serial_generation
                self._control_generation += 1
                self._disconnecting = True
                self._disconnect_teardown_in_progress = True
                self._estop_unconfirmed = True
                self._track_lease_generation += 1
                self._track_direction = "STOP"
                self._fw_mode_cache = "?"
                self._motor_status = [
                    {"state": "?", "current_deg": None,
                     "target_deg": None, "fault": None}
                    for _ in range(NUM_MOTOR_AXES)
                ]
                invalidated_axes = []
                for axis in range(NUM_STEPPER_AXES):
                    self._axis_motion_generation[axis] += 1
                    active = (
                        self.running[axis]
                        or self.stepper_in_progress[axis]
                        or self._move_dispatching[axis]
                        or self._move_reservation[axis] is not None
                        or self._web_step_pending[axis] is not None
                        or self._pending_step[axis] is not None
                    )
                    if active:
                        self.axis_runtime[axis].position_trusted = False
                        invalidated_axes.append(axis)
                    self.running[axis] = False
                    self.stepper_in_progress[axis] = False
                    self._move_dispatching[axis] = False
                    self._move_reservation[axis] = None
                    self._web_step_pending[axis] = None
                    self._pending_step[axis] = None
                    self.axis_motion_telemetry[axis] = (
                        self.axis_motion_telemetry[axis].reset(
                            result="DISCONNECTED"
                        )
                    )
                session, self.serial_session = self.serial_session, None
                ser, self.ser = self.ser, None

        # close() may join the reader, whose error callback can re-enter this
        # method.  Never hold the desktop disconnect lock across that join.
        if session is not None:
            try:
                session.close()
            except Exception:
                pass
        elif ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        with self.state_lock:
            self._disconnect_teardown_in_progress = False
        self._save_calib()

        def update_ui():
            with self.state_lock:
                if (
                    disconnect_generation != self._serial_generation
                    or self.serial_session is not None
                ):
                    return
            self.v_track_status.set("连接中断，已停止")
            self._set_track_controls("disabled")
            if self._raw_log_fh:
                try:
                    self._raw_log_fh.close()
                except Exception:
                    pass
                self._raw_log_fh = None
                self.raw_log_var.set(False)
            self.conn_status.config(text="● 未连接", foreground="red")
            self.conn_btn.config(text="连接")
            self.fw_mode_var.set("?")
            self.mode_label.config(foreground="gray")
            for axis in range(NUM_STEPPER_AXES):
                if axis in invalidated_axes:
                    self._update_pos_label(axis)
                self._enable_stepper_buttons(axis, "disabled")
            self._enable_paired_buttons("disabled")
            for axis in range(NUM_MOTOR_AXES):
                self._apply_foc_gating(axis, state="?", fault="?")
                self._apply_gear_gating(axis, state="?", fault="?")
                self.v_focstate[axis].set("未连接")
                self.v_foccur[axis].set("--")
                self.v_focfault[axis].set("--")
                self.notebook.tab(self.tab_index_foc[axis], state="normal")
                self.notebook.tab(self.tab_index_gear[axis], state="normal")

        self._post_ui(update_ui)
        now = time.monotonic()
        if not is_error or now - self._last_serial_error_log >= 5.0:
            self._last_serial_error_log = now
            prefix = "串口 I/O 异常，已停止轮询并断开" if is_error else "串口已断开"
            self.log(f"{prefix}: {reason}" if reason else prefix)

    def _serial_source_is_current(self, generation=None, session=None):
        if generation is None:
            return True
        with self.state_lock:
            return (
                generation == self._serial_generation
                and session is self.serial_session
            )

    def _handle_serial_failure(
        self, exc, source, generation=None, session=None
    ):
        self._disconnect_serial(
            f"{source}: {exc}",
            is_error=True,
            expected_generation=generation,
            expected_session=session,
        )

    def _trace_serial_line(
        self, direction, line, generation=None, session=None
    ):
        """SerialSession 的唯一原始流记录入口。"""
        if not self._serial_source_is_current(generation, session):
            return
        handle = self._raw_log_fh
        if handle is None:
            return
        try:
            handle.write(f"{time.time():.3f}  {direction}: {line}\n")
            handle.flush()
        except Exception:
            pass

    def _handle_serial_event(self, event, generation=None, session=None):
        """把 typed 串口异步事件转成控制状态和 UI 更新。"""
        if not self._serial_source_is_current(generation, session):
            return

        def post_ui(callback):
            self._post_ui(
                lambda: callback()
                if self._serial_source_is_current(generation, session)
                else None
            )

        if isinstance(event, StepProgress):
            if 0 <= event.axis < NUM_STEPPER_AXES:
                post_ui(lambda e=event: self._on_step_progress(
                    e.axis, e.executed_steps, e.requested_steps))
            return
        if isinstance(event, StepTerminal):
            if not 0 <= event.axis < NUM_STEPPER_AXES:
                return
            callback = self._on_step_done if event.result == StepResult.DONE else self._on_step_aborted
            post_ui(lambda e=event, cb=callback: cb(
                e.axis, e.executed_steps, e.requested_steps))
            return
        if isinstance(event, TrackTimeout):
            with self.state_lock:
                if not self._serial_source_is_current(generation, session):
                    return
                self._track_lease_generation += 1
                self._track_direction = "STOP"
                self._track_last_response = event.raw
            post_ui(lambda: self.v_track_status.set("轨道租约到期，已停止"))
            post_ui(lambda: self.log("轨道 D 固件租约到期，已自动停止"))
            return
        if isinstance(event, HardwareEstopEvent):
            if event.state == HardwareEstopState.TRIGGERED:
                with self.state_lock:
                    if not self._serial_source_is_current(generation, session):
                        return
                    self._control_generation += 1
                    self._hardware_estop_active = True
                    self._track_lease_generation += 1
                    self._track_direction = "STOP"
                    self._track_last_response = event.raw
                    for axis in range(NUM_STEPPER_AXES):
                        if self._axis_motion_active_locked(axis):
                            self.axis_motion_telemetry[axis] = (
                                self.axis_motion_telemetry[axis].stopping()
                            )
                        self.running[axis] = False
                    for axis in range(NUM_MOTOR_AXES):
                        self._foc_enabled_ui[axis] = False
                post_ui(lambda: self.v_track_status.set("硬件急停触发，已停止"))
                post_ui(lambda: self.log(
                    "⚠️ 硬件急停触发：固件已停止全部步进/轨道并失能闭环轴；"
                    "释放急停按钮后可重新操作"))
            else:
                with self.state_lock:
                    if not self._serial_source_is_current(generation, session):
                        return
                    self._hardware_estop_active = False
                post_ui(lambda: self.v_track_status.set("硬件急停已释放"))
                post_ui(lambda: self.log(
                    "硬件急停回路已恢复；请确认现场安全后再手动发起运动"))
            return
        if isinstance(event, MotorFaultEvent):
            if 0 <= event.axis < NUM_MOTOR_AXES:
                post_ui(lambda e=event: self.log(
                    f"⚠️ 轴{AXIS_LABEL[e.axis]}: {e.raw}"))
            return
        if isinstance(event, NodeOfflineEvent):
            with self.state_lock:
                if not self._serial_source_is_current(generation, session):
                    return
                if 1 <= event.node <= NUM_PICO_NODES:
                    self.pico_node_health[event.node] = "offline"
            post_ui(lambda e=event: self.log(
                f"⚠️ Pico 节点 {e.node} 离线；对应远端轴暂不可用"))
            return
        post_ui(lambda e=event: self.log(f"⚠️ 未处理串口事件: {e.raw}"))

    def _handle_serial_unsolicited(
        self, message, generation=None, session=None
    ):
        """显示启动诊断、畸形帧和迟到回复，但绝不让它们完成新请求。"""
        if (not message.raw
                or not self._serial_source_is_current(generation, session)):
            return
        prefix = "⚠️ 串口未识别" if isinstance(message, UnknownMessage) else "⚠️ 串口迟到/未匹配"
        self._post_ui(
            lambda p=prefix, line=message.raw: self.log(f"{p}: {line}")
            if self._serial_source_is_current(generation, session)
            else None
        )

    def _send_and_read(
        self,
        cmd,
        timeout=1.0,
        guard=None,
        allow_closing=False,
        propagate_request_error=False,
        soft_timeout=False,
    ):
        context_generation = getattr(self._control_context, "generation", None)
        with self.state_lock:
            session = self.serial_session
            session_generation = self._serial_generation

        def can_send():
            if not self._serial_source_is_current(
                session_generation, session
            ):
                return False
            if ((self._closing or self._disconnecting) and not allow_closing):
                return False
            if (context_generation is not None
                    and context_generation != self._control_generation):
                return False
            return guard is None or guard()

        if session is None or not session.is_open or not can_send():
            if propagate_request_error:
                raise RequestCancelled(f"request cancelled before send: {cmd!r}")
            return ""
        try:
            return session.request_line(
                cmd, timeout=timeout, guard=can_send,
                soft_timeout=soft_timeout)
        except RequestCancelled:
            if propagate_request_error:
                raise
            return ""
        except RequestTimeout as exc:
            # A timed-out reply can be indistinguishable from a later command's
            # OK/ACK.  SerialSession therefore retires the stream; force the
            # same visible disconnect path instead of continuing unsafely.
            if self._serial_source_is_current(session_generation, session):
                self._handle_serial_failure(
                    exc, "请求超时", session_generation, session
                )
            if propagate_request_error:
                raise
            return ""
        except SerialSessionError as exc:
            if self._serial_source_is_current(session_generation, session):
                self._handle_serial_failure(
                    exc, "请求", session_generation, session
                )
            if propagate_request_error:
                raise
            return ""

    # ═════════════ 步进：发送 ═════════════
    def _send_pulses(
        self, axis, steps, direction, delay_ms, guard=None, profile=None
    ):
        if not 0 <= axis < NUM_STEPPER_AXES:
            return False
        if self._rot_release_active and axis in self._rot_release_axes:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]} 旋转电机已释放（落地纠偏中），"
                     "拒绝运动；两直线轴停止后会自动重新锁定")
            return False
        if steps <= 0 or not self._is_serial_connected():
            return False
        with self.state_lock:
            if profile is None:
                profile = self.axis_profiles[axis]
            position_trusted = self.axis_runtime[axis].position_trusted
        if not position_trusted:
            self.log(f"⚠️ 轴 {AXIS_LABEL[axis]} 位置不可信；请先设置原点或校准位置")
            return False
        # 不在 PC 侧排队第二段 MOVE：等待期间当前位置可能因上一段 DONE
        # 改变，令先前的限位预检失效。连续运动由自己的循环等待 terminal。
        with self.state_lock:
            busy = (
                self.stepper_in_progress[axis]
                or self._move_dispatching[axis]
                or self._pending_step[axis] is not None
            )
        if busy:
            self.log(f"轴{AXIS_LABEL[axis]} 正在运动，已拒绝排队 MOVE")
            return False
        delay_us = max(1, int(round(delay_ms * 1000)))
        # RP2040 远端节点的 PIO/RS485 契约要求至少 100 us。
        delay_us = clamp_step_delay_us(axis, delay_us)
        actual_dir = direction ^ DIR_INVERT[axis]   # 按轴翻转DIR信号
        outward_sign = outward_position_sign(axis)
        sign = outward_sign if direction == DIR_OUTWARD else -outward_sign
        signed_steps = sign * steps
        with self.state_lock:
            if self._pending_step[axis] is not None:
                self.log(f"轴{AXIS_LABEL[axis]} 已有待结算运动，拒绝覆盖")
                return False
            self._move_dispatching[axis] = True
            # MOVE 写出后 DONE/ABORT 可能先于 ACK 到达。发送前预挂账，
            # terminal 事件即可立即、且只结算一次。
            self._pending_step[axis] = signed_steps
            self.stepper_in_progress[axis] = True
            self.axis_motion_telemetry[axis] = AxisMotionTelemetry.starting(
                self._axis_motion_generation[axis], steps
            )
        try:
            command = build_move_command(axis, steps, actual_dir, delay_us)
        except ProtocolEncodingError as exc:
            with self.state_lock:
                self._move_dispatching[axis] = False
                if self._pending_step[axis] == signed_steps:
                    self._pending_step[axis] = None
                    self.stepper_in_progress[axis] = False
                    self.axis_motion_telemetry[axis] = (
                        self.axis_motion_telemetry[axis].reset(result="REJECTED")
                    )
            self.log(f"⛔ 轴{AXIS_LABEL[axis]} MOVE 参数无效: {exc}")
            return False
        request_error = None
        try:
            resp = self._send_and_read(
                command, guard=guard, propagate_request_error=True
            )
        except RequestCancelled as exc:
            request_error = exc
            resp = ""
        except (RequestTimeout, SerialSessionError) as exc:
            request_error = exc
            resp = ""
        dist_mm = profile.units_from_steps(steps)
        out_txt, in_txt, _out_arrow, _in_arrow = direction_label_parts(axis)
        dir_txt = out_txt if direction == DIR_OUTWARD else in_txt
        effective_delay_ms = delay_us / 1000.0
        mm_s = (
            step_delay_ms_to_pulse_rate(axis, effective_delay_ms)
            / profile.pulses_per_unit
        )
        self.log(f"轴{AXIS_LABEL[axis]} {dist_mm:.1f}{profile.unit} {dir_txt} "
                 f"@ {effective_delay_ms:.2f}ms ({mm_s:.1f}{profile.speed_unit}) → {resp}")
        with self.state_lock:
            self._move_dispatching[axis] = False
            accepted = resp == f"ACK,{axis}"
            terminal_already_settled = self._pending_step[axis] is None
            if not accepted and not terminal_already_settled:
                self._pending_step[axis] = None
                self.stepper_in_progress[axis] = False
                result = (
                    "UNKNOWN"
                    if request_error is not None
                    and not isinstance(request_error, RequestCancelled)
                    else "REJECTED"
                )
                self.axis_motion_telemetry[axis] = (
                    self.axis_motion_telemetry[axis].reset(result=result)
                )
                if request_error is not None and not isinstance(
                    request_error, RequestCancelled
                ):
                    # Bytes may have reached the controller even though the
                    # reply was lost.  Never keep a trusted host position.
                    self.axis_runtime[axis].position_trusted = False
        if accepted and not terminal_already_settled:
            if 'progress' in self.sw[axis]:
                self._post_ui(lambda a=axis: self._show_step_started(a))
        return accepted

    def _show_step_started(self, axis):
        with self.state_lock:
            if not self.stepper_in_progress[axis]:
                return
        self.sw[axis]['progress']['value'] = 0
        self.sw[axis]['progress_label'].config(text="运动中...")

    def _reserve_axis_move(self, axis):
        """Synchronously pin one GUI MOVE to its validated axis profile."""

        with self.state_lock:
            if self._axis_motion_active_locked(axis):
                return None
            reservation = object()
            self._move_reservation[axis] = reservation
            return (
                reservation,
                self._axis_motion_generation[axis],
                self.axis_profiles[axis],
            )

    def _axis_move_denied_reason(self, axis) -> str:
        """2026-09-21：步态占用与轴自身运动中的拒绝原因分开说清，
        避免步态 S6 下降期间手动点动被"正在运动"误导为轴卡住。"""

        with self.state_lock:
            gait_owned = axis in self._gait_owned
        if gait_owned:
            return (f"轴{AXIS_LABEL[axis]} 步态运行中，禁止手动操作"
                    "（可点步态页【中止】后单独调整）")
        return f"轴{AXIS_LABEL[axis]} 正在运动，MOVE 未排队"

    def _release_axis_move(self, axis, reservation):
        with self.state_lock:
            if self._move_reservation[axis] is reservation:
                self._move_reservation[axis] = None

    def _start_reserved_axis_worker(
        self, axis, worker_reservation, target, *args, **kwargs
    ):
        try:
            worker = self._start_control_worker(target, *args, **kwargs)
        except BaseException:
            self._release_axis_move(axis, worker_reservation)
            raise
        if worker is None:
            self._release_axis_move(axis, worker_reservation)
        return worker

    def _begin_axis_stop(self, axis):
        """Invalidate queued MOVE work and reserve the axis until STOP replies."""

        with self.state_lock:
            if axis in getattr(self, "_gait_owned", {}):
                self._gait_needs_recovery = True
                if self._gait_run is not None:
                    self._gait_run.request_stop()
            self._axis_motion_generation[axis] += 1
            stop_reservation = object()
            self.running[axis] = False
            self._move_reservation[axis] = stop_reservation
            # A Web MOVE worker holding the previous token will observe the
            # generation/reservation mismatch before SerialSession writes it.
            self._web_step_pending[axis] = None
            self.axis_motion_telemetry[axis] = (
                self.axis_motion_telemetry[axis].stopping()
            )
        return stop_reservation

    def _send_axis_stop(self, axis, stop_reservation=None, guard=None):
        """Send one per-axis STOP while keeping later MOVE work behind it."""

        if stop_reservation is None:
            stop_reservation = self._begin_axis_stop(axis)
        try:
            return self._send_and_read(
                build_stop_command(axis), timeout=0.6, guard=guard
            )
        finally:
            with self.state_lock:
                if self._move_reservation[axis] is stop_reservation:
                    self._move_reservation[axis] = None

    def _send_mm(
        self,
        axis,
        distance_mm,
        direction,
        delay_ms,
        profile=None,
        guard=None,
        reservation=None,
        motion_generation=None,
    ):
        with self.state_lock:
            if motion_generation is None:
                motion_generation = self._axis_motion_generation[axis]
            if motion_generation != self._axis_motion_generation[axis]:
                return False
            if reservation is None:
                if self._move_reservation[axis] is not None:
                    return False
                reservation = object()
                self._move_reservation[axis] = reservation
            elif self._move_reservation[axis] is not reservation:
                return False

        def move_guard():
            with self.state_lock:
                current = (
                    motion_generation == self._axis_motion_generation[axis]
                    and self._move_reservation[axis] is reservation
                )
            return current and (guard is None or guard())

        try:
            return self._send_mm_reserved(
                axis,
                distance_mm,
                direction,
                delay_ms,
                profile=profile,
                guard=move_guard,
            )
        finally:
            with self.state_lock:
                if self._move_reservation[axis] is reservation:
                    self._move_reservation[axis] = None

    def _send_mm_reserved(
        self, axis, distance_mm, direction, delay_ms, profile=None, guard=None
    ):
        # profile 是请求被接受时的不可变快照；配置与工作线程交错时也不能
        # 把旧模式的距离/速度和新模式的脉冲换算混在同一条 MOVE 中。
        with self.state_lock:
            if profile is None:
                profile = self.axis_profiles[axis]
            runtime = self.axis_runtime[axis]
            position_trusted = runtime.position_trusted
            position_steps = runtime.position_steps
            minimum_steps = runtime.min_steps
            maximum_steps = runtime.max_steps
        if not position_trusted:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]} 位置不可信，禁止运动；请先重新校准")
            return False
        try:
            steps = profile.command_steps_from_units(distance_mm)
        except (TypeError, ValueError, OverflowError):
            self.log(f"⛔ 轴{AXIS_LABEL[axis]} 距离必须是有限数值")
            return False
        if steps <= 0:
            self.log(f"轴{AXIS_LABEL[axis]}: 忽略距离过小 ({distance_mm})")
            return False
        outward_sign = float(outward_position_sign(axis))
        sign = outward_sign if direction == DIR_OUTWARD else -outward_sign
        target_steps = position_steps + sign * steps
        tolerance_steps = profile.exact_steps_from_units(0.05)
        target = profile.units_from_steps(target_steps)
        if minimum_steps is not None and target_steps < minimum_steps - tolerance_steps:
            minimum = profile.units_from_steps(minimum_steps)
            self.log(
                f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target:.1f}{profile.unit}"
                f"<下限{minimum:.1f}"
            )
            return False
        if maximum_steps is not None and target_steps > maximum_steps + tolerance_steps:
            maximum = profile.units_from_steps(maximum_steps)
            self.log(
                f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target:.1f}{profile.unit}"
                f">上限{maximum:.1f}"
            )
            return False
        return self._send_pulses(
            axis, steps, direction, delay_ms, guard=guard, profile=profile
        )

    def _send_to_position(
        self,
        axis,
        target_units,
        delay_ms,
        profile=None,
        reservation=None,
        motion_generation=None,
    ):
        """Resolve an absolute target only after owning the per-axis reservation."""

        try:
            target_units = float(target_units)
        except (TypeError, ValueError, OverflowError):
            return False
        if not math.isfinite(target_units):
            return False
        with self.state_lock:
            if motion_generation is None:
                motion_generation = self._axis_motion_generation[axis]
            if motion_generation != self._axis_motion_generation[axis]:
                return False
            if reservation is None:
                if self._move_reservation[axis] is not None:
                    return False
                reservation = object()
                self._move_reservation[axis] = reservation
            elif self._move_reservation[axis] is not reservation:
                return False
            current_profile = self.axis_profiles[axis]
            profile_changed = profile is not None and profile != current_profile
            if profile is None:
                profile = current_profile
            busy = (
                self.running[axis]
                or self.stepper_in_progress[axis]
                or self._move_dispatching[axis]
                or self._web_step_pending[axis] is not None
                or self._pending_step[axis] is not None
            )
            runtime = self.axis_runtime[axis]
            position_trusted = runtime.position_trusted
            position_steps = runtime.position_steps
            minimum_steps = runtime.min_steps
            maximum_steps = runtime.max_steps
            target_steps = profile.exact_steps_from_units(target_units)
            tolerance_steps = profile.exact_steps_from_units(0.05)

        try:
            if profile_changed:
                self.log(
                    f"轴{AXIS_LABEL[axis]} 配置已变化，绝对定位已取消；请重试"
                )
                return False
            if busy:
                self.log(f"轴{AXIS_LABEL[axis]} 正在运动，绝对定位未排队")
                return False
            if not position_trusted:
                self.log(
                    f"⛔ 轴{AXIS_LABEL[axis]} 位置不可信，禁止运动；请先重新校准"
                )
                return False
            if (
                minimum_steps is not None
                and target_steps < minimum_steps - tolerance_steps
            ):
                minimum = profile.units_from_steps(minimum_steps)
                self.log(
                    f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target_units:.1f}{profile.unit}"
                    f"<下限{minimum:.1f}"
                )
                return False
            if (
                maximum_steps is not None
                and target_steps > maximum_steps + tolerance_steps
            ):
                maximum = profile.units_from_steps(maximum_steps)
                self.log(
                    f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target_units:.1f}{profile.unit}"
                    f">上限{maximum:.1f}"
                )
                return False

            delta_steps = target_steps - position_steps
            if abs(delta_steps) < 1.0:
                self.log(f"轴{AXIS_LABEL[axis]} 已在目标附近")
                return False
            direction = (
                DIR_OUTWARD
                if delta_steps * outward_position_sign(axis) > 0
                else DIR_INWARD
            )
            distance = abs(profile.units_from_steps(delta_steps))
            self.log(
                f"轴{AXIS_LABEL[axis]} 前往 {target_units:.1f}{profile.unit} "
                f"(移动 {distance:.1f}{profile.unit})"
            )
            return self._send_mm(
                axis,
                distance,
                direction,
                delay_ms,
                profile=profile,
                reservation=reservation,
                motion_generation=motion_generation,
            )
        finally:
            with self.state_lock:
                if self._move_reservation[axis] is reservation:
                    self._move_reservation[axis] = None

    @staticmethod
    def _executed_fraction(executed_steps, requested_steps, default):
        if executed_steps is None or requested_steps is None or requested_steps <= 0:
            return default
        return max(0.0, min(1.0, float(executed_steps) / float(requested_steps)))

    def _on_step_done(self, axis, executed_steps=None, requested_steps=None):
        # A gait may lower a foot only after the full commanded path completes.
        # Legacy/mismatched/partial DONE is not sufficient evidence, even if
        # the controller labels it DONE. Keep ordinary legacy MOVE behavior.
        with self.state_lock:
            expected = self._pending_step[axis]
            gait_owned = axis in getattr(self, "_gait_owned", {})
        if (gait_owned and expected is not None
                and (requested_steps != abs(expected)
                     or executed_steps != abs(expected))):
            self.log(f"步态轴{AXIS_LABEL[axis]} DONE脉冲数异常，禁止继续落脚")
            if requested_steps != abs(expected):
                executed_steps = requested_steps = None
            self._on_step_aborted(axis, executed_steps, requested_steps)
            return
        with self.state_lock:
            pending_step = self._pending_step[axis]
            if pending_step is not None:
                fraction = self._executed_fraction(
                    executed_steps, requested_steps, 1.0
                )
                self.axis_runtime[axis].apply_completed_steps(
                    pending_step * fraction
                )
                self._pending_step[axis] = None
                self.axis_motion_telemetry[axis] = self.axis_motion_telemetry[
                    axis
                ].terminal("DONE", executed_steps, requested_steps)
            self.stepper_in_progress[axis] = False
        if pending_step is not None:
            self._update_pos_label(axis)
        if 'progress' in self.sw[axis]:
            self.sw[axis]['progress']['value'] = 100
            self.sw[axis]['progress_label'].config(text="✓ 完成")
            self.root.after(2000, lambda a=axis: self._reset_progress(a))
        self._save_calib()
        self.log(f"轴{AXIS_LABEL[axis]} 步进完成")
        self._land_release_tick(axis)

    def _on_step_aborted(self, axis, executed_steps=None, requested_steps=None):
        """按固件回报记入部分位移；旧协议不回报脉冲时绝不记入整段位移。"""
        fraction = self._executed_fraction(executed_steps, requested_steps, 0.0)
        with self.state_lock:
            pending_step = self._pending_step[axis]
            partial_steps = 0.0
            if pending_step is not None:
                partial_steps = pending_step * fraction
                self.axis_runtime[axis].apply_completed_steps(partial_steps)
            self._pending_step[axis] = None
            self.stepper_in_progress[axis] = False
            self.running[axis] = False
            self.axis_runtime[axis].position_trusted = False
            self.axis_motion_telemetry[axis] = self.axis_motion_telemetry[
                axis
            ].terminal("ABORTED", executed_steps, requested_steps)
            profile = self.axis_profiles[axis]
        self._update_pos_label(axis)
        if 'progress' in self.sw[axis]:
            self.sw[axis]['progress_label'].config(text="⚠ 已中止 · 位置需重新校准")
        self._save_calib()
        partial_units = profile.units_from_steps(partial_steps)
        detail = (f"，已按固件回报记入 {partial_units:+.3f} {profile.unit}"
                  if executed_steps is not None and requested_steps else
                  "，旧固件未回报实际脉冲，未记入待执行位移")
        self.log(f"⚠️ 轴{AXIS_LABEL[axis]} 运动已中止{detail}；软件位置已标记为不可信")
        self._land_release_tick(axis)

    # ═════════════ 步进：命令 ═════════════
    # ═════════════ 左右直线联动（同命令发两轴）═════════════
    # 不重写运动逻辑：全部复用单轴链路（_quick_move/_press_continuous/
    # stop_continuous 各自带忙检查、软限位与日志），本组方法只做
    # “两轴都空闲才启动”的预检查，避免只动一边。
    def _on_pair_speed_change(self, *_args):
        for axis in self.paired_axes:
            self.v_speed_str[axis].set(self.pair_speed_str.get())
            self._on_speed_select(axis)

    def _enable_paired_buttons(self, state):
        for key in ('move_btn', 'jog_out_btn', 'jog_in_btn',
                    'cont_out_btn', 'cont_in_btn', 'stop_btn'):
            widget = self.paired_widgets.get(key)
            if widget is not None:
                widget.config(state=state)

    def _paired_all_idle(self):
        with self.state_lock:
            return all(not self._axis_motion_active_locked(axis)
                       for axis in self.paired_axes)

    def _paired_start(self, distance_mm, direction):
        """预检查后对两轴发同一命令；返回是否已下发。"""

        for axis in self.paired_axes:
            if not self._require_axis_params(axis):
                return False
            if not self._speed_safety_ok(axis):
                return False
        if not self._paired_all_idle():
            self.log("⛔ 联动：左侧直/右侧直 任一轴正在运动或已预约，"
                     "拒绝启动（防止只动一边）")
            return False
        for axis in self.paired_axes:
            self._quick_move(axis, distance_mm, direction)
        return True

    def _paired_send_move(self):
        try:
            distance = float(self.pair_dist.get())
        except (tk.TclError, TypeError, ValueError, OverflowError):
            distance = math.nan
        if not math.isfinite(distance) or distance <= 0:
            messagebox.showerror("输入无效", "运动距离必须是大于 0 的有限数值")
            return
        self._paired_start(distance, self.pair_dir.get())

    def _paired_quick_move(self, direction):
        self._paired_start(1.0, direction)

    def _paired_press_continuous(self, direction):
        for axis in self.paired_axes:
            self._press_continuous(axis, direction)

    def _paired_release_continuous(self):
        for axis in self.paired_axes:
            self._release_continuous(axis)

    def _paired_stop(self):
        for axis in self.paired_axes:
            self.stop_continuous(axis)

    # ═════════════ 落地纠偏：悬空腿落地时自动释放旋转电机 ═════════════
    # 依据：两直轴标高差 ≈ 悬空腿离地间隙。差值从大于阈值收敛到阈值内
    # （两条等价路径：悬空侧“向上”或站立侧“向下”）= 悬空腿即将落地承载，
    # 此时释放 Mr1/Mr2 让机构在重力下扭正一次；两直轴回到空闲后自动锁定。
    # 释放是纯机械动作：旋转轴不发脉冲，软件位置继续沿用（用户决定，
    # 纠偏量小且有界，不重新校准）。抬腿（差值扩大）绝不触发。
    def _axis_estimated_position_units(self, axis):
        """已结算位置 + 进行中运动按进度的实时估算（mm/°）。"""

        with self.state_lock:
            runtime = self.axis_runtime[axis]
            steps = runtime.position_steps
            pending = self._pending_step[axis]
            telemetry = self.axis_motion_telemetry[axis]
        if (pending is not None and telemetry.requested_steps
                and telemetry.executed_steps is not None):
            steps += pending * telemetry.executed_steps / telemetry.requested_steps
        return self.axis_profiles[axis].units_from_steps(steps)

    def _land_gap_mm(self):
        """两直轴标高差；任一侧位置不可信时返回 None（不触发）。"""

        with self.state_lock:
            if not all(self.axis_runtime[a].position_trusted
                       for a in self.paired_axes):
                return None
        return abs(self._axis_estimated_position_units(self.paired_axes[0])
                   - self._axis_estimated_position_units(self.paired_axes[1]))

    def _land_release_tick(self, axis):
        """直线轴运动事件钩子（UI 线程）：监测落地收敛 + 尝试重新锁定。

        单轴日常运动与三足步态的直线阶段（S2 抬足 / S5 接近 / S6 落脚）
        共用本钩子：步态占用期间同样触发释放，只是不把旋转轴位置标记
        为不可信，换位流程才能继续走完。
        """

        if axis not in self.paired_axes:
            return
        try:
            enabled = self.pair_land_release_enabled.get()
            threshold = self.pair_land_release_threshold_mm.get()
        except (tk.TclError, TypeError, ValueError):
            return
        if not math.isfinite(threshold) or threshold <= 0:
            return
        if not self._rot_release_active and enabled:
            # 落地收敛必然发生在单轴运动（悬空侧向上或站立侧向下）。
            # 两轴同时运动是顶部整体升降：两轴启动时间差会让差值瞬时
            # 窜动，既不是落地也不能当下降沿，故不评估也不记录基准。
            with self.state_lock:
                moving = [a for a in self.paired_axes
                          if self._axis_motion_active_locked(a)]
            if len(moving) == 1:
                gap = self._land_gap_mm()
                if gap is not None:
                    last = self._last_land_gap_mm
                    self._last_land_gap_mm = gap
                    if last is not None and last > threshold >= gap:
                        self._trigger_land_release(
                            gap,
                            mark_untrusted=not bool(getattr(self, "_gait_owned", {})))
        self._maybe_relock_rot_axes()

    def _trigger_land_release(self, gap_mm=None, *, mark_untrusted=True):
        """释放 Mr1/Mr2 绑定的旋转轴（有一个未绑定就整体跳过并提示）。

        gap_mm 为 None 表示手动验证释放（复用同一释放/延迟重锁链路）。
        mark_untrusted：单轴日常路径释放后把旋转轴位置标记为不可信
        （自由转子不受主机脉冲监控，需重新校准）；三足步态路径与手动
        验证路径为让流程继续/便于恢复，软件位置继续沿用（与"纠偏量
        小且有界、不重新校准"的既有决定一致）。
        """

        axes = []
        for role_name in ("Mr1", "Mr2"):
            binding = self.control_bindings.for_role(LogicalRole(role_name))
            if binding is None:
                self.log(f"⚠️ 落地纠偏：{role_name} 未绑定物理轴，本次跳过释放"
                         "（纠偏需要两侧旋转轴同时释放）")
                return
            axes.append(binding.axis)
        with self.state_lock:
            # 旋转轴正在脉冲（如步态 SYNC 摆动段）时绝不失能：释放只
            # 允许发生在旋转空闲的落脚窗口。
            if any(self._axis_motion_active_locked(a) for a in axes):
                return

        def worker():
            try:
                for axis in self._rot_release_axes:
                    # 释放命令在直线轴脉冲运动中发出，固件串口响应会变慢；
                    # ENA 是幂等设置命令，超时按"未确认"处理而不是判定会话
                    # 失步断连（硬超时会误杀整个连接）。
                    resp = self._send_and_read(
                        build_ena_command(axis, False), timeout=1.2,
                        soft_timeout=True)
                    if resp:
                        self.log(f"🔓 落地纠偏：释放旋转轴{AXIS_LABEL[axis]} → {resp}")
                    else:
                        self.log(f"⚠️ 落地纠偏：释放旋转轴{AXIS_LABEL[axis]}"
                                 " 固件未确认（运动中响应慢），纠偏可能未执行")
            finally:
                # 让提前重锁的等待方知道 ENA,0 已经落到串口上。
                self._rot_release_settled.set()

        self._rot_release_active = True
        self._rot_release_axes = tuple(sorted(set(axes)))
        self._rot_release_settled.clear()
        if mark_untrusted:
            # Free rotor motion is not observed by host pulse accounting.
            with self.state_lock:
                for released_axis in self._rot_release_axes:
                    self.axis_runtime[released_axis].position_trusted = False
            self._save_calib()
        elif gap_mm is not None:
            self.log("🔓 步态落脚纠偏：旋转轴软件位置继续沿用，不标记不可信")
        if self._start_control_worker(worker) is None:
            self._rot_release_active = False
            self._rot_release_axes = ()
            self._rot_release_settled.set()
            return
        if gap_mm is None:
            self.log("🔓 手动验证：释放左右旋转电机（转子可用手扭动）；"
                     f"{LAND_RELOCK_DELAY_S:.0f} 秒后自动锁定，或再点一次按钮立即锁定")
        else:
            self.log(f"🔓 落地纠偏：两轴标高差 {gap_mm:.1f}mm 已进入阈值，"
                     "释放左右旋转电机，机构自正中；落地后 "
                     f"{LAND_RELOCK_DELAY_S:.0f} 秒自动重新锁定")
        self._update_land_release_status()

    def _maybe_relock_rot_axes(self):
        """两直轴都空闲（含连动/预约/网页）= 落地完成，调度延迟重锁。"""

        if not self._rot_release_active:
            return
        if not self._is_serial_connected():
            return
        with self.state_lock:
            busy = any(self._axis_motion_active_locked(a)
                       for a in self.paired_axes)
            pending = self._rot_relock_after is not None
        if busy:
            # 落地尚未完成（直线运动重启）：作废本次延迟，等再次落地。
            self._cancel_rot_relock_timer()
            return
        if pending:
            return
        self._schedule_rot_relock()

    def _schedule_rot_relock(self):
        """UI 线程：安排 LAND_RELOCK_DELAY_S 秒后的重锁定时器。"""

        with self.state_lock:
            self._rot_relock_generation += 1
            generation = self._rot_relock_generation
        self._rot_relock_after = self.root.after(
            int(LAND_RELOCK_DELAY_S * 1000),
            lambda: self._rot_relock_timer_fired(generation))

    def _rot_relock_timer_fired(self, generation):
        """延迟到点（UI 线程）：仍处于释放态且机构空闲时恢复锁定。"""

        self._rot_relock_after = None
        with self.state_lock:
            if (generation != self._rot_relock_generation
                    or not self._rot_release_active):
                return
            # 直线轴又在运动 → 交回 _maybe_relock 在下一个直线事件重排；
            # 旋转轴正在脉冲（防御，正常流程不会走到）→ 绝不失能中使能。
            if (any(self._axis_motion_active_locked(a) for a in self.paired_axes)
                    or any(self._axis_motion_active_locked(a)
                           for a in self._rot_release_axes)):
                return
            self._rot_release_active = False
            axes = self._rot_release_axes
            self._rot_release_axes = ()

        self._start_control_worker(
            self._relock_rot_axes_worker(axes, reason="纠偏完成"))
        self._update_land_release_status()

    def _relock_rot_axes_worker(self, axes, *, reason):
        """构造重锁 worker（控制线程执行）：向各旋转轴发 ENA,1。"""

        def worker():
            for axis in axes:
                # 重锁在直线轴刚停的窗口发出，同样用软超时防误断连。
                resp = self._send_and_read(
                    build_ena_command(axis, True), timeout=1.2,
                    soft_timeout=True)
                if resp:
                    self.log(f"🔒 {reason}：重新锁定旋转轴{AXIS_LABEL[axis]} → {resp}")
                else:
                    self.log(f"⚠️ {reason}：重新锁定旋转轴{AXIS_LABEL[axis]}"
                             " 固件未确认，请目视确认旋转电机已锁定")

        return worker

    def _toggle_manual_rot_release(self):
        """验证用双态按钮：手动释放 / 立即锁定左右旋转电机。

        手动释放不改变软件位置（与步态路径一致），5 秒后自动锁定
        兜底，期间再点一次立即锁定。步态占用四轴期间禁用。
        """

        if self._rot_release_active:
            self._cancel_rot_relock_timer()
            with self.state_lock:
                self._rot_release_active = False
                axes = self._rot_release_axes
                self._rot_release_axes = ()
            if axes:
                def worker():
                    # 与提前重锁同理：先等 ENA,0 落到串口再发 ENA,1。
                    self._rot_release_settled.wait(timeout=2.5)
                    self._relock_rot_axes_worker(axes, reason="手动锁定")()
                self._start_control_worker(worker)
            self.log("🔒 手动锁定：恢复左右旋转电机锁定")
            self._update_land_release_status()
            return
        if getattr(self, "_gait_owned", {}):
            messagebox.showwarning(
                "步态执行中", "步态占用四轴期间禁止手动释放；请先完成或中止")
            return
        if not self._is_serial_connected():
            messagebox.showwarning("未连接", "请先连接串口再手动释放")
            return
        try:
            mr_axes = (self._gait_role_axis("Mr1"), self._gait_role_axis("Mr2"))
        except GaitExecutorError as exc:
            messagebox.showerror(
                "无法释放", f"需要 Mr1/Mr2 绑定旋转模式轴：{exc}")
            return
        with self.state_lock:
            if any(self._axis_motion_active_locked(a) for a in mr_axes):
                messagebox.showwarning(
                    "旋转轴运动中", "旋转轴正在运动，停止后才能手动释放")
                return
        if not messagebox.askokcancel(
                "手动释放旋转电机",
                "将向 Mr1/Mr2 绑定的旋转轴发送失能（ENA,0）。\n\n"
                "释放后转子可用手扭动验证；软件位置继续沿用、不会自动\n"
                "更新，请勿大力扭动。"
                f"{LAND_RELOCK_DELAY_S:.0f} 秒后自动锁定，或再点一次按钮\n"
                "立即锁定。\n\n继续？"):
            return
        self._trigger_land_release(mark_untrusted=False)
        if self._rot_release_active:
            # 直线轴此刻空闲，不会有运动事件来调度重锁：手动安排
            # 延迟自动锁定兜底。
            self._schedule_rot_relock()

    def _cancel_rot_relock_timer(self):
        """UI 线程：作废未到点的延迟重锁。"""

        with self.state_lock:
            self._rot_relock_generation += 1
            after_id = self._rot_relock_after
            self._rot_relock_after = None
        if after_id is not None:
            try:
                self.root.after_cancel(after_id)
            except tk.TclError:
                pass

    def _ensure_rot_axes_locked_blocking(self):
        """控制线程：步态旋转阶段开始前确保旋转轴已重新锁定。

        5 秒延迟未到时终止等待立即重锁（换位流程不因延迟卡住）；
        重锁未获固件确认时抛 GaitExecutorError，由阶段 worker 的
        失败路径接管。
        """

        with self.state_lock:
            if not self._rot_release_active:
                return
            self._rot_relock_generation += 1   # 未到点的延迟重锁立即失效
        # 先等 ENA,0 落到串口（保证失能→再使能的因果顺序）。
        self._rot_release_settled.wait(timeout=2.5)
        with self.state_lock:
            if not self._rot_release_active:
                return   # 延迟回调或其它路径已抢先完成重锁
            self._rot_release_active = False
            axes = self._rot_release_axes
            self._rot_release_axes = ()
        unconfirmed = []
        for axis in axes:
            resp = self._send_and_read(
                build_ena_command(axis, True), timeout=1.2, soft_timeout=True)
            if resp:
                self.log(f"🔒 步态继续：提前重新锁定旋转轴{AXIS_LABEL[axis]} → {resp}")
            else:
                unconfirmed.append(AXIS_LABEL[axis])
        self._update_land_release_status()
        if unconfirmed:
            raise GaitExecutorError(
                "旋转轴重锁未获固件确认：" + "、".join(unconfirmed)
                + "；禁止在失能状态下执行步态旋转运动")

    def _force_relock_rot_axes(self, allow_closing=False):
        """急停/断开/关闭路径：立即恢复锁定，绝不留在失能态。"""

        if not self._rot_release_active:
            return
        with self.state_lock:
            # 作废未到点的延迟重锁（回调核对 generation 后自行退出）。
            # 本方法可能在任意线程调用：绝不跨线程碰 Tk 的 after 句柄。
            self._rot_relock_generation += 1
        self._rot_release_active = False
        axes = self._rot_release_axes
        self._rot_release_axes = ()
        for axis in axes:
            try:
                # 断开路径 best-effort：软超时不炸断开流程，未确认仅记日志。
                self._send_and_read(
                    build_ena_command(axis, True), timeout=0.8,
                    allow_closing=allow_closing, soft_timeout=True)
            except Exception:
                pass
        self.log("🔒 已重新锁定旋转电机（停止/断开路径）")
        self._update_land_release_status()

    def _update_land_release_status(self):
        def apply():
            label = self.paired_widgets.get('land_release_status')
            button = self.paired_widgets.get('land_release_button')
            if label is None and button is None:
                return
            if self._rot_release_active:
                if label is not None:
                    label.config(
                        text="🔓 旋转电机已释放 · 机构纠偏中"
                             f"（落地后 {LAND_RELOCK_DELAY_S:.0f} 秒自动锁定）",
                        foreground="#b42318")
                if button is not None:
                    button.config(text="🔒 立即锁定旋转电机")
            else:
                if label is not None:
                    label.config(text="🔒 旋转电机已锁定", foreground="#2e7d32")
                if button is not None:
                    button.config(text="🔓 手动释放旋转电机（验证用）")
        # 急停/断开路径可能从工作线程调用；Tk 控件操作必须回到 UI 线程。
        self._post_ui(apply)

    def send_move(self, axis):
        if not self._require_axis_params(axis):
            return
        if not self._speed_safety_ok(axis):
            return
        try:
            distance = float(self.v_dist[axis].get())
        except (tk.TclError, TypeError, ValueError, OverflowError):
            distance = math.nan
        if not math.isfinite(distance) or distance <= 0:
            messagebox.showerror("输入无效", "运动距离必须是大于 0 的有限数值")
            return
        move = self._reserve_axis_move(axis)
        if move is None:
            self.log(self._axis_move_denied_reason(axis))
            return
        reservation, motion_generation, profile = move
        self._start_reserved_axis_worker(
            axis, reservation, self._send_mm,
            axis, distance, self.v_dir[axis].get(),
            self.v_delay[axis].get(), profile,
            reservation=reservation,
            motion_generation=motion_generation)

    def _quick_move(self, axis, distance_mm, direction):
        if not self._require_axis_params(axis):
            return
        if not self._speed_safety_ok(axis):
            return
        move = self._reserve_axis_move(axis)
        if move is None:
            self.log(self._axis_move_denied_reason(axis))
            return
        reservation, motion_generation, profile = move
        self._start_reserved_axis_worker(
            axis, reservation, self._send_mm,
            axis, distance_mm, direction,
            self.v_delay[axis].get(), profile,
            reservation=reservation,
            motion_generation=motion_generation)

    def _press_continuous(self, axis, direction):
        if self.running[axis] or not self.ser or not self.ser.is_open: return
        if not self._require_axis_params(axis):
            return
        delay_ms = self.v_delay[axis].get()
        # 按住连发是重复手势，不适合每次弹窗；仅记录风险日志。
        try:
            pulse_rate = step_delay_ms_to_pulse_rate(axis, delay_ms)
        except (TypeError, ValueError):
            pulse_rate = 0.0
        if pulse_rate > PULSE_RATE_WARN_PPS:
            self.log(f"⚠️ 轴{AXIS_LABEL[axis]} 连续模式脉冲率 {pulse_rate:,.0f} pps "
                     f"超过安全参考 {PULSE_RATE_WARN_PPS:.0f} pps，注意失步风险")
        with self.state_lock:
            if self.running[axis]:
                return
            self.running[axis] = True
            profile = self.axis_profiles[axis]
            motion_generation = self._axis_motion_generation[axis]
        burst_units = _continuous_burst_units(profile)
        out_txt, in_txt, _out_arrow, _in_arrow = direction_label_parts(axis)
        dir_txt = out_txt if direction == DIR_OUTWARD else in_txt
        self.log(f"轴{AXIS_LABEL[axis]} 按住连续{dir_txt}")
        def worker():
            while self.running[axis]:
                while self.stepper_in_progress[axis] and self.running[axis]:
                    time.sleep(0.02)
                if not self.running[axis]: break
                if not self._send_mm(
                    axis, burst_units, direction, delay_ms, profile,
                    motion_generation=motion_generation,
                ):
                    with self.state_lock:
                        self.running[axis] = False
                    break
            self.log(f"轴{AXIS_LABEL[axis]} 连续运动停止")
        if self._start_control_worker(worker) is None:
            self.running[axis] = False

    def _release_continuous(self, axis):
        with self.state_lock:
            if self.running[axis]:
                self.running[axis] = False
                self._axis_motion_generation[axis] += 1
        self._land_release_tick(axis)

    def stop_continuous(self, axis):
        stop_reservation = self._begin_axis_stop(axis)
        if self._is_serial_connected():
            threading.Thread(
                target=self._send_axis_stop,
                args=(axis, stop_reservation),
                daemon=True,
            ).start()
        else:
            with self.state_lock:
                if self._move_reservation[axis] is stop_reservation:
                    self._move_reservation[axis] = None
        self.log(f"⛔ 轴{AXIS_LABEL[axis]} 紧急停止")

    # ═════════════ 步进：位置/原点 ═════════════
    def _axis_motion_active_locked(self, axis):
        return (
            self.running[axis]
            or self.stepper_in_progress[axis]
            or self._move_dispatching[axis]
            or self._move_reservation[axis] is not None
            or self._web_step_pending[axis] is not None
            or self._pending_step[axis] is not None
        )

    def _warn_calibration_while_moving(self, axis):
        message = f"轴{AXIS_LABEL[axis]}正在运动，停止后才能修改位置或限位"
        self.log(f"⛔ {message}")
        messagebox.showwarning("暂不能校准", message)

    def set_home(self, axis):
        with self.state_lock:
            blocked = self._axis_motion_active_locked(axis)
            if not blocked:
                self.axis_runtime[axis].set_position(
                    self.axis_profiles[axis], 0.0, trusted=True)
                # 位置读数跳变：落地纠偏的差值基准必须作废，
                # 否则跨跳变比较会伪造一个"收敛"下降沿误触发。
                self._last_land_gap_mm = None
        if blocked:
            self._warn_calibration_while_moving(axis)
            return
        self._update_pos_label(axis)
        self._save_calib()
        self.log(f"✓ 轴{AXIS_LABEL[axis]} 当前位置设为原点")

    def calibrate_position(self, axis):
        try: val = float(self.v_goto[axis].get())
        except (tk.TclError, TypeError, ValueError, OverflowError):
            messagebox.showerror("输入无效", "请先填目标位置"); return
        if not math.isfinite(val):
            messagebox.showerror("输入无效", "校准位置必须是有限数值"); return
        with self.state_lock:
            blocked = self._axis_motion_active_locked(axis)
            if not blocked:
                self.axis_runtime[axis].set_position(
                    self.axis_profiles[axis], val, trusted=True)
                # 同 set_home：校准使读数跳变，差值基准必须作废。
                self._last_land_gap_mm = None
        if blocked:
            self._warn_calibration_while_moving(axis)
            return
        self._update_pos_label(axis)
        self._save_calib()
        self.log(f"✓ 轴{AXIS_LABEL[axis]} 位置校准为 {val:.1f} {self._unit_label(axis)}")

    def go_home(self, axis):
        self._goto(axis, 0.0)

    def goto_target_position(self, axis):
        try: target = float(self.v_goto[axis].get())
        except (tk.TclError, TypeError, ValueError, OverflowError):
            messagebox.showerror("输入无效", "请输入有效目标位置"); return
        if not math.isfinite(target):
            messagebox.showerror("输入无效", "目标位置必须是有限数值"); return
        self._goto(axis, target)

    def _goto(self, axis, target_mm):
        try:
            target_mm = float(target_mm)
        except (TypeError, ValueError, OverflowError):
            target_mm = math.nan
        if not math.isfinite(target_mm):
            messagebox.showerror("输入无效", "目标位置必须是有限数值")
            return
        if not self._require_axis_params(axis):
            return
        if not self._speed_safety_ok(axis):
            return
        move = self._reserve_axis_move(axis)
        if move is None:
            self.log(f"轴{AXIS_LABEL[axis]} 正在运动，绝对定位未排队")
            return
        reservation, motion_generation, profile = move
        self._start_reserved_axis_worker(
            axis, reservation, self._send_to_position,
            axis, target_mm,
            self.v_delay[axis].get(), profile,
            reservation=reservation,
            motion_generation=motion_generation)

    # ═════════════ 行程校准 ═════════════
    def _mark_min(self, axis):
        with self.state_lock:
            blocked = self._axis_motion_active_locked(axis)
            runtime = self.axis_runtime[axis]
            profile = self.axis_profiles[axis]
            trusted = runtime.position_trusted
            invalid = (
                runtime.max_steps is not None
                and runtime.position_steps >= runtime.max_steps
            )
            if not blocked and trusted and not invalid:
                runtime.min_steps = runtime.position_steps
                value = runtime.position_in(profile)
        if blocked:
            self._warn_calibration_while_moving(axis); return
        if not trusted:
            messagebox.showerror("位置不可信", "请先设置原点或校准当前位置"); return
        if invalid:
            messagebox.showerror("范围无效", "最小不能 ≥ 最大"); return
        self._update_range_display(axis); self._save_calib()
        self.log(f"⊖ 轴{AXIS_LABEL[axis]} 最小 = {value:.1f}{profile.unit}")

    def _mark_max(self, axis):
        with self.state_lock:
            blocked = self._axis_motion_active_locked(axis)
            runtime = self.axis_runtime[axis]
            profile = self.axis_profiles[axis]
            trusted = runtime.position_trusted
            invalid = (
                runtime.min_steps is not None
                and runtime.position_steps <= runtime.min_steps
            )
            if not blocked and trusted and not invalid:
                runtime.max_steps = runtime.position_steps
                value = runtime.position_in(profile)
        if blocked:
            self._warn_calibration_while_moving(axis); return
        if not trusted:
            messagebox.showerror("位置不可信", "请先设置原点或校准当前位置"); return
        if invalid:
            messagebox.showerror("范围无效", "最大不能 ≤ 最小"); return
        self._update_range_display(axis); self._save_calib()
        self.log(f"⊕ 轴{AXIS_LABEL[axis]} 最大 = {value:.1f}{profile.unit}")

    def _clear_range(self, axis):
        with self.state_lock:
            blocked = self._axis_motion_active_locked(axis)
            if not blocked:
                self.axis_runtime[axis].min_steps = None
                self.axis_runtime[axis].max_steps = None
        if blocked:
            self._warn_calibration_while_moving(axis)
            return
        self._update_range_display(axis); self._save_calib()
        self.log(f"轴{AXIS_LABEL[axis]} 行程限位已清除")

    def _update_range_display(self, axis):
        mn, mx = self.axis_runtime[axis].limits_in(self.axis_profiles[axis])
        label = self.sw[axis]['range_label']
        if mn is None and mx is None:
            label.config(text="未校准（无软件限位）", foreground="gray")
        else:
            mn_s = f"{mn:.1f}" if mn is not None else "?"
            mx_s = f"{mx:.1f}" if mx is not None else "?"
            travel = f"  (行程 {mx-mn:.1f}{self._unit_label(axis)})" if (mn is not None and mx is not None) else ""
            label.config(text=f"min={mn_s} max={mx_s}{travel}", foreground="black")

    def _check_range(self, axis, target_mm):
        runtime = self.axis_runtime[axis]
        profile = self.axis_profiles[axis]
        if not runtime.position_trusted:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]} 位置不可信，禁止运动；请先重新校准")
            return False
        mn, mx = runtime.limits_in(profile)
        target_steps = profile.exact_steps_from_units(target_mm)
        tolerance_steps = profile.exact_steps_from_units(0.05)
        if runtime.min_steps is not None and target_steps < runtime.min_steps - tolerance_steps:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target_mm:.1f}{self._unit_label(axis)}<下限{mn:.1f}"); return False
        if runtime.max_steps is not None and target_steps > runtime.max_steps + tolerance_steps:
            self.log(f"⛔ 轴{AXIS_LABEL[axis]}: 目标{target_mm:.1f}{self._unit_label(axis)}>上限{mx:.1f}"); return False
        return True

    @staticmethod
    def _profile_metadata(profile):
        return {
            "mode": profile.mode,
            "pulse_per_rev": profile.pulse_per_rev,
            "gear_ratio": profile.gear_ratio,
            "lead_mm": profile.lead_mm,
        }

    @staticmethod
    def _profile_metadata_matches(profile, value):
        if not isinstance(value, dict):
            return False
        try:
            stored = AxisProfile(
                mode=value["mode"],
                pulse_per_rev=value["pulse_per_rev"],
                gear_ratio=value["gear_ratio"],
                lead_mm=value["lead_mm"],
            )
        except (KeyError, TypeError, ValueError):
            return False
        return stored == profile

    def _save_calib(self):
        with self.state_lock:
            data = {}
            for axis in range(NUM_STEPPER_AXES):
                profile = self.axis_profiles[axis]
                entry = self.axis_runtime[axis].as_legacy_units(profile)
                # Additive metadata lets the next startup detect a crash or
                # failed write between axis-config and calibration files.
                entry["_profile"] = self._profile_metadata(profile)
                data[str(axis)] = entry
        try:
            self.state_store.save_calibration(data)
        except StateStoreError as e:
            self.log(f"⚠️ 校准保存失败: {e}")

    def _load_calib(self):
        try:
            data = self.state_store.load_calibration()
            if data is None:
                return
            mismatched_axes = []
            for a in range(NUM_STEPPER_AXES):
                d = data.get(str(a), {})
                if not isinstance(d, dict):
                    self.axis_runtime[a] = AxisRuntime(position_trusted=False)
                    mismatched_axes.append(a)
                    self._update_pos_label(a)
                    self._update_range_display(a)
                    continue
                profile = self.axis_profiles[a]
                metadata = d.get("_profile")
                profile_mismatch = (
                    a in self._axis_profile_load_fallback
                    or (
                        metadata is not None
                        and not self._profile_metadata_matches(profile, metadata)
                    )
                )
                try:
                    runtime = AxisRuntime.from_legacy_units(
                        profile,
                        minimum=d.get("min"),
                        maximum=d.get("max"),
                        position=d.get("position", 0.0),
                        trusted=(
                            bool(d.get("trusted", True))
                            and not profile_mismatch
                        ),
                    )
                except (TypeError, ValueError):
                    runtime = AxisRuntime(position_trusted=False)
                    profile_mismatch = True
                self.axis_runtime[a] = runtime
                if profile_mismatch:
                    mismatched_axes.append(a)
                self._update_pos_label(a)
                self._update_range_display(a)
            if mismatched_axes:
                labels = ", ".join(AXIS_LABEL[a] for a in mismatched_axes)
                self.log(
                    f"⚠️ 轴配置与校准不一致或损坏，位置已标记为不可信: {labels}"
                )
            any_set = any(
                runtime.min_steps is not None or runtime.max_steps is not None
                for runtime in self.axis_runtime
            )
            if any_set:
                self.log("已加载行程校准")
        except (StateStoreError, TypeError, ValueError) as e:
            self.log(f"⚠️ 校准读取失败: {e}")

    # ═════════════ FOC 调参持久化 ═════════════
    def _save_foc_tune(self):
        """把每轴 V/PA/VP/PP 当前值写到 .foc_tune.json。"""
        try:
            data = {str(a): {
                "V":  round(self.v_focvlimit[a].get(), 2),
                "PA": round(self.v_focpangle[a].get(), 2),
                "VP": round(self.v_focvp[a].get(),     3),
                "PP": int(self.v_focpp[a].get()),
            } for a in range(NUM_MOTOR_AXES)}
            self.state_store.save_foc_tune(data)
        except (StateStoreError, tk.TclError, TypeError, ValueError) as e:
            self.log(f"⚠️ FOC 调参保存失败: {e}")

    def _load_foc_tune(self):
        """启动时从 json 读回 Tk 变量 + 更新滑条标签。不自动下发到固件，
        要等 toggle_connect 成功后 _apply_foc_tune_to_firmware 再推。"""
        try:
            data = self.state_store.load_foc_tune()
            if data is None:
                return
            for a in range(NUM_MOTOR_AXES):
                d = data.get(str(a), {})
                if not isinstance(d, dict):
                    raise TypeError(f"FOC 轴 {a} 调参项必须是对象")
                if "V"  in d: self.v_focvlimit[a].set(float(d["V"]))
                if "PA" in d: self.v_focpangle[a].set(float(d["PA"]))
                if "VP" in d: self.v_focvp[a].set(float(d["VP"]))
                if "PP" in d: self.v_focpp[a].set(int(d["PP"]))
                # 同步标签
                self.fw[a]['vlimit_label'].config(text=f"{self.v_focvlimit[a].get():.1f} V (扭矩)")
                self.fw[a]['pangle_label'].config(text=f"{self.v_focpangle[a].get():.1f} (刚度)")
                self.fw[a]['vp_label'].config(text=f"{self.v_focvp[a].get():.2f} (阻尼)")
            self.log("已加载 FOC 调参")
        except (StateStoreError, tk.TclError, TypeError, ValueError, KeyError) as e:
            self.log(f"⚠️ FOC 调参读取失败: {e}")

    def _apply_foc_tune_to_firmware(self):
        """连接成功后将已加载的 V/PA/VP 下发给固件；PP 在固件 NVS 里，不需要重发。"""
        values = [
            (self.v_focvlimit[axis].get(), self.v_focpangle[axis].get(),
             self.v_focvp[axis].get())
            for axis in range(NUM_MOTOR_AXES)
        ]
        def worker():
            time.sleep(0.3)
            for axis, (v, pa, vp) in enumerate(values):
                self._send_and_read(f"FOC,{axis},V,{v:.1f}")
                self._send_and_read(f"FOC,{axis},PA,{pa:.1f}")
                self._send_and_read(f"FOC,{axis},VP,{vp:.2f}")
            self.log("FOC 调参已下发固件")
        self._start_control_worker(worker)

    # ═════════════ FOC：命令 ═════════════
    def _send_foc(self, axis, sub_and_arg):
        """例：_send_foc(0, 'EN,1') → 发 FOC,0,EN,1 → 日志记响应"""
        cmd = f"FOC,{axis},{sub_and_arg}"
        def worker():
            resp = self._send_and_read(cmd)
            self.log(f"{cmd} → {resp}")
        self._start_control_worker(worker)

    def _foc_goto(self, axis):
        self._send_foc(axis, f"A,{self.v_foctgt[axis].get():.1f}")

    def _foc_quick(self, axis, deg):
        self.v_foctgt[axis].set(deg); self._foc_goto(axis)

    def _foc_increment(self, axis, delta):
        self.v_foctgt[axis].set(self.v_foctgt[axis].get() + delta); self._foc_goto(axis)

    def _foc_home(self, axis):
        self._send_foc(axis, "H")

    def _foc_toggle_enable(self, axis):
        self._foc_enabled_ui[axis] = not self._foc_enabled_ui[axis]
        v = 1 if self._foc_enabled_ui[axis] else 0
        self._send_foc(axis, f"EN,{v}")

    def _foc_on_vlimit(self, axis, _):
        v = self.v_focvlimit[axis].get()
        if v > 12.0:
            txt = f"{v:.1f} V ⚠️ 超额定"
            self.fw[axis]['vlimit_label'].config(text=txt, foreground="red")
        else:
            self.fw[axis]['vlimit_label'].config(text=f"{v:.1f} V (扭矩)", foreground="black")
        self._send_foc(axis, f"V,{v:.1f}")

    def _foc_on_pangle(self, axis, _):
        p = self.v_focpangle[axis].get()
        self.fw[axis]['pangle_label'].config(text=f"{p:.1f} (刚度)")
        self._send_foc(axis, f"PA,{p:.1f}")

    def _foc_on_vp(self, axis, _):
        p = self.v_focvp[axis].get()
        self.fw[axis]['vp_label'].config(text=f"{p:.2f} (阻尼)")
        self._send_foc(axis, f"VP,{p:.2f}")

    def _foc_save_pp(self, axis):
        n = int(self.v_focpp[axis].get())
        self._send_foc(axis, f"PP,{n}")
        self._save_foc_tune()

    def _foc_clear_fault(self, axis):
        self._send_foc(axis, "CLR")
        self._foc_enabled_ui[axis] = False

    # ═════════════ FOC：轮询 + 显示 ═════════════
    def _any_stepper_axis_busy(self):
        """任一步进轴正在运动（含按住连动）→ 暂停 FOC 状态轮询。

        双轴同时脉冲输出时固件对查询命令的响应会显著变慢；此时继续
        轮询既挤占串口又会拿到迟到数据，等轴停下来再恢复。
        """

        with self.state_lock:
            return any(
                self.stepper_in_progress[a]
                or self._pending_step[a] is not None
                or self.running[a]
                for a in range(NUM_STEPPER_AXES)
            )

    def _foc_poll_loop(self, generation, session):
        while (self.foc_poll_running
               and self._serial_source_is_current(generation, session)):
            if self._any_stepper_axis_busy():
                time.sleep(0.1)
                continue
            for axis in range(NUM_MOTOR_AXES):
                if not (self.foc_poll_running
                        and self._serial_source_is_current(generation, session)):
                    break
                if self._any_stepper_axis_busy():
                    break
                # 只读状态轮询用软超时：固件瞬时忙不过来时跳过本轮，
                # 绝不把“查询无响应”升级成断开整个串口。
                resp = self._send_and_read(
                    f"FOC,{axis},S",
                    guard=lambda: self._serial_source_is_current(
                        generation, session
                    ),
                    soft_timeout=True,
                )
                if not resp:
                    continue
                prefix = f"FOC,{axis},S,"
                if resp.startswith(prefix):
                    parts = resp.split(",")
                    # 格式: FOC,<axis>,S,<state>,<cur>,<tgt>,<fault>
                    if len(parts) == 7:
                        s, c, t, f = parts[3], parts[4], parts[5], parts[6]
                        self._post_ui(
                            lambda a=axis, s=s, c=c, t=t, f=f:
                            self._update_foc_display(a, s, c, t, f)
                            if self._serial_source_is_current(generation, session)
                            else None
                        )
                time.sleep(FOC_POLL_INTERVAL_S / NUM_MOTOR_AXES)  # 总周期仍 ~100ms

    def _update_foc_display(self, axis, state, cur, tgt, fault):
        self.v_focstate[axis].set(FOC_STATE_NAMES.get(state, "?"))
        try:
            cur_f = float(cur); tgt_f = float(tgt)
        except ValueError:
            cur_f = tgt_f = None
        with self.state_lock:
            self._motor_status[axis] = {
                "state": state,
                "current_deg": cur_f,
                "target_deg": tgt_f,
                "fault": fault == "1",
            }
        cur_text = f"{cur_f:.1f}°" if cur_f is not None else "--"
        if self.stepper_in_progress[axis]: cur_text += " ⏸"
        self.v_foccur[axis].set(cur_text)
        self.v_focfault[axis].set("报警" if fault == "1" else "正常")
        self.fw[axis]['fault_label'].config(foreground="red" if fault == "1" else "green")
        self.gw[axis]['fault_label'].config(foreground="red" if fault == "1" else "green")
        # 两个模式都按响应门控（只激活当前模式的会真正生效）
        self._apply_foc_gating(axis, state, fault)
        self._apply_gear_gating(axis, state, fault)

        if cur_f is not None and state == "2":
            now = time.time()
            self.foc_trace_buf[axis].append((now, tgt_f, cur_f))
            cutoff = now - 10.0
            self.foc_trace_buf[axis] = [x for x in self.foc_trace_buf[axis] if x[0] >= cutoff]
            self._redraw_scope(axis)

    def _apply_gear_gating(self, axis, state, fault):
        gw = self.gw[axis]
        if not self.ser or not self.ser.is_open:
            for w in gw['motion_btns'] + gw['cfg_widgets']:
                w.config(state="disabled")
            gw['enable_btn'].config(state="disabled")
            gw['clear_btn'].config(state="disabled")
            return
        is_fault    = (fault == "1" or state == "3")
        is_disabled = (state == "0")
        is_running  = (state == "2")
        motion_state = "normal" if is_running else "disabled"
        for b in gw['motion_btns']: b.config(state=motion_state)
        cfg_state = "normal" if (is_disabled or is_running) else "disabled"
        for w in gw['cfg_widgets']: w.config(state=cfg_state)
        en_state = "normal" if (is_disabled or is_running) else "disabled"
        gw['enable_btn'].config(state=en_state)
        if is_running:    gw['enable_btn'].config(text="■ 失能 PID")
        elif is_fault:    gw['enable_btn'].config(text="(故障，先清除)")
        else:             gw['enable_btn'].config(text="▶ 使能 PID")
        gw['clear_btn'].config(state="normal" if is_fault else "disabled")

    def _apply_foc_gating(self, axis, state, fault):
        fw = self.fw[axis]
        if not self.ser or not self.ser.is_open:
            for w in fw['motion_btns'] + fw['cfg_btns']:
                w.config(state="disabled")
            fw['enable_btn'].config(state="disabled")
            fw['clear_btn'].config(state="disabled")
            return
        is_fault    = (fault == "1" or state == "3")
        is_disabled = (state == "0")
        is_running  = (state == "2")
        is_aligning = (state == "1")
        motion_state = "normal" if is_running else "disabled"
        for b in fw['motion_btns']: b.config(state=motion_state)
        cfg_state = "normal" if (is_disabled or is_running) else "disabled"
        for w in fw['cfg_btns']: w.config(state=cfg_state)
        en_state = "normal" if (is_disabled or is_running) else "disabled"
        fw['enable_btn'].config(state=en_state)
        if is_running:    fw['enable_btn'].config(text="■ 失能 FOC")
        elif is_aligning: fw['enable_btn'].config(text="… 对齐中")
        elif is_fault:    fw['enable_btn'].config(text="(故障，先清除)")
        else:             fw['enable_btn'].config(text="▶ 使能 FOC")
        fw['clear_btn'].config(state="normal" if is_fault else "disabled")

    def _redraw_scope(self, axis):
        # 在 FOC tab 和 GEAR tab 的画布上都画（共用同一份 trace 缓冲）
        canvases = []
        if axis < len(self.fw) and 'scope' in self.fw[axis]: canvases.append(self.fw[axis]['scope'])
        if axis < len(self.gw) and 'scope' in self.gw[axis]: canvases.append(self.gw[axis]['scope'])
        for cv in canvases:
            self._draw_scope_on(cv, axis)

    def _draw_scope_on(self, cv, axis):
        W, H = 420, 420
        cv.delete("all")
        buf = self.foc_trace_buf[axis]
        if len(buf) < 2:
            cv.create_text(W/2, H/2, text="(等待数据，使能 + 设目标后开始)", fill="#888")
            return
        t0 = buf[0][0]; t_span = max(0.1, buf[-1][0] - t0)
        ys = [p[1] for p in buf] + [p[2] for p in buf]
        y_min, y_max = min(ys), max(ys)
        if y_max - y_min < 10:
            c = (y_min + y_max) / 2; y_min, y_max = c - 5, c + 5
        y_pad = (y_max - y_min) * 0.1
        y_min -= y_pad; y_max += y_pad
        for frac in (0.25, 0.5, 0.75):
            y = H * frac; cv.create_line(0, y, W, y, fill="#e5e5e5")
        if y_min < 0 < y_max:
            y0 = H * (y_max - 0) / (y_max - y_min)
            cv.create_line(0, y0, W, y0, fill="#aaa", dash=(3, 3))
        cv.create_text(3, 3, text=f"{y_max:.0f}°", anchor="nw", fill="#555", font=("Arial", 8))
        cv.create_text(3, H-3, text=f"{y_min:.0f}°", anchor="sw", fill="#555", font=("Arial", 8))
        pts_t, pts_c = [], []
        for t, tgt, cur in buf:
            x = W * (t - t0) / t_span
            yt = H * (y_max - tgt) / (y_max - y_min)
            yc = H * (y_max - cur) / (y_max - y_min)
            pts_t.extend([x, yt]); pts_c.extend([x, yc])
        if len(pts_t) >= 4: cv.create_line(*pts_t, fill="#1565c0", width=1)
        if len(pts_c) >= 4: cv.create_line(*pts_c, fill="#d32f2f", width=2)

    # ═════════════ GEAR Tab（参数化）═════════════
    def _build_gear_tab(self, parent, axis):
        build_gear_tab_view(self, parent, axis)

    # ═════════════ GEAR：命令 ═════════════
    def _send_gear(self, axis, sub_and_arg):
        """和 _send_foc 一样格式，GEAR 固件也用 FOC,<axis>,... 命名空间。"""
        cmd = f"FOC,{axis},{sub_and_arg}"
        def worker():
            resp = self._send_and_read(cmd)
            self.log(f"{cmd} → {resp}")
        self._start_control_worker(worker)

    def _gear_goto(self, axis):
        target = self.v_foctgt[axis].get()
        self._send_gear(axis, f"A,{target:.1f}")
        self._start_goto_watcher(axis, target)

    def _start_goto_watcher(self, axis, target_deg, tol_deg=2.0, timeout_s=15.0):
        """后台监视 cur → tgt 的逼近，每 1.5s 打印进度。新 goto / 失能 自动取消旧 watcher。"""
        self._goto_watcher_gen[axis] += 1
        my_gen = self._goto_watcher_gen[axis]
        def worker():
            t0 = time.time()
            last_log = 0.0
            self.log(f"→ 闭环{MOTOR_AXIS_LABELS[axis]} 前往 {target_deg:.1f}°")
            while self.ser and self.ser.is_open:
                # 1. 被新 goto 取代 → 退出（不打日志）
                if self._goto_watcher_gen[axis] != my_gen:
                    return
                # 2. 使用线程安全状态快照，不从后台线程读取 Tk 变量。
                with self.state_lock:
                    motor = dict(self._motor_status[axis])
                if motor["state"] != "2":
                    self.log(f"  闭环{MOTOR_AXIS_LABELS[axis]} watcher 退出（PID 状态={motor['state']}）")
                    return
                # 3. 读当前角度
                cur = motor["current_deg"]
                if cur is None:
                    time.sleep(0.3); continue
                err = target_deg - cur
                dt = time.time() - t0
                # 4. 到位
                if abs(err) < tol_deg:
                    self.log(f"✓ 闭环{MOTOR_AXIS_LABELS[axis]} 到位 cur={cur:.1f}° (用时 {dt:.1f}s)")
                    return
                # 5. 超时
                if dt > timeout_s:
                    self.log(f"⚠️ 闭环{MOTOR_AXIS_LABELS[axis]} {timeout_s:.0f}s 未到位 "
                             f"cur={cur:.1f}° 差 {err:+.1f}° (Kp 太小？发 DIAG,0 看 PWM)")
                    return
                # 6. 周期进度
                if time.time() - last_log > 1.5:
                    self.log(f"  闭环{MOTOR_AXIS_LABELS[axis]} cur={cur:.1f}° 差 {err:+.1f}° (t={dt:.1f}s)")
                    last_log = time.time()
                time.sleep(0.3)
        threading.Thread(target=worker, daemon=True).start()

    def _gear_quick(self, axis, deg):
        self.v_foctgt[axis].set(deg); self._gear_goto(axis)

    def _gear_increment(self, axis, delta):
        self.v_foctgt[axis].set(self.v_foctgt[axis].get() + delta); self._gear_goto(axis)

    def _gear_home(self, axis):
        self._send_gear(axis, "H")

    def _gear_toggle_enable(self, axis):
        self._foc_enabled_ui[axis] = not self._foc_enabled_ui[axis]
        v = 1 if self._foc_enabled_ui[axis] else 0
        self._send_gear(axis, f"EN,{v}")

    def _gear_on_pwm(self, axis, _):
        v = self.v_gearpwm[axis].get()
        self.gw[axis]['pwm_label'].config(text=f"{v:.1f} %")
        self._send_gear(axis, f"V,{v:.1f}")   # GEAR 固件 V 是 PWM 百分比

    def _gear_on_kp(self, axis, _):
        v = self.v_gearkp[axis].get()
        self.gw[axis]['kp_label'].config(text=f"{v:.2f}")
        self._send_gear(axis, f"PA,{v:.2f}")

    def _gear_on_ki(self, axis, _):
        v = self.v_gearki[axis].get()
        self.gw[axis]['ki_label'].config(text=f"{v:.2f}")
        self._send_gear(axis, f"PI,{v:.2f}")

    def _gear_on_kd(self, axis, _):
        v = self.v_gearkd[axis].get()
        self.gw[axis]['kd_label'].config(text=f"{v:.3f}")
        self._send_gear(axis, f"PD,{v:.3f}")

    def _gear_save_gr(self, axis):
        gr = float(self.v_geargr[axis].get())
        self._send_gear(axis, f"GR,{gr:.1f}")
        self._save_gear_tune()

    def _gear_clear_fault(self, axis):
        self._send_gear(axis, "CLR")
        self._foc_enabled_ui[axis] = False

    # ═════════════ GEAR 调参持久化 ═════════════
    def _save_gear_tune(self):
        try:
            data = {str(a): {
                "PWM": round(self.v_gearpwm[a].get(), 2),
                "Kp":  round(self.v_gearkp[a].get(),  3),
                "Ki":  round(self.v_gearki[a].get(),  3),
                "Kd":  round(self.v_gearkd[a].get(),  4),
                "GR":  round(self.v_geargr[a].get(),  1),
            } for a in range(NUM_MOTOR_AXES)}
            self.state_store.save_gear_tune(data)
            # 简短日志反馈 axis 0 当前值（最常用）
            d = data["0"]
            self.log(f"💾 GEAR 调参已保存到 {self.state_store.paths.gear_tune.name} "
                     f"(L: PWM={d['PWM']}% Kp={d['Kp']} Ki={d['Ki']} Kd={d['Kd']} GR={d['GR']})")
        except (StateStoreError, tk.TclError, TypeError, ValueError, KeyError) as e:
            self.log(f"⚠️ GEAR 调参保存失败: {e}")

    def _load_gear_tune(self):
        try:
            data = self.state_store.load_gear_tune()
            if data is None:
                return
            for a in range(NUM_MOTOR_AXES):
                d = data.get(str(a), {})
                if not isinstance(d, dict):
                    raise TypeError(f"GEAR 轴 {a} 调参项必须是对象")
                if "PWM" in d: self.v_gearpwm[a].set(float(d["PWM"]))
                if "Kp"  in d: self.v_gearkp[a].set(float(d["Kp"]))
                if "Ki"  in d: self.v_gearki[a].set(float(d["Ki"]))
                if "Kd"  in d: self.v_gearkd[a].set(float(d["Kd"]))
                if "GR"  in d: self.v_geargr[a].set(float(d["GR"]))
                self.gw[a]['pwm_label'].config(text=f"{self.v_gearpwm[a].get():.1f} %")
                self.gw[a]['kp_label'].config(text=f"{self.v_gearkp[a].get():.2f}")
                self.gw[a]['ki_label'].config(text=f"{self.v_gearki[a].get():.2f}")
                self.gw[a]['kd_label'].config(text=f"{self.v_gearkd[a].get():.3f}")
            self.log("已加载 GEAR 调参")
        except (StateStoreError, tk.TclError, TypeError, ValueError, KeyError) as e:
            self.log(f"⚠️ GEAR 调参读取失败: {e}")

    def _apply_gear_tune_to_firmware(self):
        values = [
            (self.v_gearpwm[axis].get(), self.v_gearkp[axis].get(),
             self.v_gearki[axis].get(), self.v_gearkd[axis].get())
            for axis in range(NUM_MOTOR_AXES)
        ]
        def worker():
            time.sleep(0.3)
            for axis, (pwm, kp, ki, kd) in enumerate(values):
                self._send_and_read(f"FOC,{axis},V,{pwm:.1f}")
                self._send_and_read(f"FOC,{axis},PA,{kp:.2f}")
                self._send_and_read(f"FOC,{axis},PI,{ki:.2f}")
                self._send_and_read(f"FOC,{axis},PD,{kd:.3f}")
            self.log("GEAR 调参已下发固件")
        self._start_control_worker(worker)

    # ═════════════ 模式自动检测 + tab 灰显 ═════════════
    def _query_mode_and_apply(self):
        """连接成功后调一次。用户选 Auto 时发 MODE 命令自动检测；
        用户选 FOC/GEAR 时直接强制应用，不查询固件。"""
        sel = self.mode_select_var.get()
        with self.state_lock:
            generation = self._serial_generation
            session = self.serial_session

        def apply_if_current(mode):
            if self._serial_source_is_current(generation, session):
                self._apply_mode_to_tabs(mode)

        if sel in (MODE_FOC, MODE_GEAR):
            self.log(f"模式选择 = 强制 {sel}（跳过固件 MODE 查询）")
            self._post_ui(lambda: apply_if_current(sel))
            return
        # Auto
        def worker():
            time.sleep(0.4)
            if not self._serial_source_is_current(generation, session):
                return
            resp = self._send_and_read(
                "MODE",
                timeout=1.5,
                guard=lambda: self._serial_source_is_current(
                    generation, session
                ),
            )
            if not self._serial_source_is_current(generation, session):
                return
            mode = MODE_FOC
            if resp.startswith("MODE,"):
                m = resp.split(",", 1)[1].strip().upper()
                if m in (MODE_FOC, MODE_GEAR): mode = m
            else:
                self.log(f"⚠️ 固件未返回 MODE（响应={resp!r}）。请烧含 MODE 命令的固件，"
                         f"或用'强制 FOC/GEAR'手动选。先按 FOC 处理。")
            self._post_ui(lambda: apply_if_current(mode))
        threading.Thread(target=worker, daemon=True).start()

    def _on_mode_select_change(self):
        """用户切换 Auto/FOC/GEAR radio 时调用。如果已连接，立即重新应用模式。"""
        sel = self.mode_select_var.get()
        self.log(f"模式选择 → {sel}")
        if self.ser and self.ser.is_open:
            self._query_mode_and_apply()

    def _apply_mode_to_tabs(self, mode):
        """灰掉非当前模式的 tab，更新 mode 标签，下发对应模式的 tune。"""
        self.fw_mode_var.set(mode)
        with self.state_lock:
            self._fw_mode_cache = mode
        if mode == MODE_FOC:
            self.mode_label.config(foreground="#1565c0")
            for a in range(NUM_MOTOR_AXES):
                self.notebook.tab(self.tab_index_foc[a],  state="normal")
                self.notebook.tab(self.tab_index_gear[a], state="disabled")
            # 自动跳到第一个 FOC tab
            self.notebook.select(self.tab_index_foc[0])
            self._apply_foc_tune_to_firmware()
        elif mode == MODE_GEAR:
            self.mode_label.config(foreground="#2e7d32")
            for a in range(NUM_MOTOR_AXES):
                self.notebook.tab(self.tab_index_foc[a],  state="disabled")
                self.notebook.tab(self.tab_index_gear[a], state="normal")
            self.notebook.select(self.tab_index_gear[0])
            self._apply_gear_tune_to_firmware()
        self.log(f"固件模式 = {mode}")

    # ═════════════ 网页服务公开接口（允许从非 Tk 线程调用）═════════════
    @staticmethod
    def _lan_ipv4():
        """尽量取得手机可访问的局域网 IPv4；失败时安全回退到本机地址。"""
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("8.8.8.8", 80))
            address = probe.getsockname()[0]
            if address and not address.startswith("127."):
                return address
        except OSError:
            pass
        finally:
            probe.close()
        try:
            address = socket.gethostbyname(socket.gethostname())
            return address if address else "127.0.0.1"
        except OSError:
            return "127.0.0.1"

    def _start_web_server(self, factory):
        """启动随 GUI 生命周期运行的局域网网页服务。"""
        try:
            server = factory(self._web_controller)
            server.start()
            self.attach_web_server(server)
            port = server.bound_port
            if port is None:
                raise RuntimeError("网页服务未取得监听端口")
            address = self._lan_ipv4()
            url = f"http://{address}:{port}/"
            self.web_address_var.set(address)
            self.web_port_var.set(str(port))
            self.web_url_var.set(url)
            if address.startswith("127."):
                self.web_status_var.set("● 已启动（仅检测到本机地址）")
                self.web_status_label.configure(foreground="#b26a00")
                self.log("⚠️ 网页服务已启动，但只检测到本机地址；请联网后重启 GUI")
            else:
                self.web_status_var.set("● 服务已启动")
                self.web_status_label.configure(foreground="#16803a")
                self.log(f"网页服务已启动：{url}（固定地址，无控制令牌）")
        except Exception as exc:
            self.web_server = None
            self.web_status_var.set("● 服务启动失败")
            self.web_address_var.set("—")
            self.web_port_var.set("—")
            self.web_url_var.set(f"网页服务启动失败：{exc}")
            self.web_status_label.configure(foreground="#b42318")
            self.log(f"⚠️ 网页服务启动失败：{exc}")

    def _copy_web_url(self):
        url = self.web_url_var.get()
        if not url.startswith("http://"):
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(url)
        self.root.update_idletasks()
        self.log("手机控制固定网址已复制")

    def _open_web_url(self):
        url = self.web_url_var.get()
        if url.startswith("http://"):
            webbrowser.open(url)

    def _on_close(self):
        """尽力停车，然后回收串口和网页服务器。"""
        with self.state_lock:
            if self._closing:
                return
            self._closing = True
            active_axes = [
                axis for axis in range(NUM_STEPPER_AXES)
                if self._axis_motion_active_locked(axis)
            ]
        if self._coordinated_refresh_after is not None:
            try:
                self.root.after_cancel(self._coordinated_refresh_after)
            except tk.TclError:
                pass
            self._coordinated_refresh_after = None
        if self._is_serial_connected():
            confirmed = self._stop_all_outputs(allow_closing=True)
            if not confirmed and not messagebox.askyesno(
                "停车未确认",
                "控制器没有确认 ESTOP，电机可能仍在运动或保持使能。\n\n"
                "请立即使用物理急停或断电。是否仍要强制退出？",
                icon="warning",
            ):
                with self.state_lock:
                    self._closing = False
                self._refresh_coordinated_ui()
                return
        # 确认停车或操作者明确选择强制退出后，再关闭 HTTP 入口。
        if self.web_server is not None:
            try:
                self.web_server.stop()
            except Exception:
                pass
            self.web_server = None
        if self._is_serial_connected():
            self._disconnect_serial("GUI 已关闭", is_error=False)
        # _disconnect_serial 的 UI 更新来不及在 destroy 前执行；关闭路径必须同步持久化失信状态。
        with self.state_lock:
            for axis in active_axes:
                self.axis_runtime[axis].position_trusted = False
                self.running[axis] = False
                self.stepper_in_progress[axis] = False
                self._move_dispatching[axis] = False
                self._move_reservation[axis] = None
                self._pending_step[axis] = None
                self.axis_motion_telemetry[axis] = (
                    self.axis_motion_telemetry[axis].reset(result="CLOSED")
                )
        self._save_calib()
        self.root.destroy()

    def _ensure_web_control_available(self, *, allow_estop_retry=False):
        with self.state_lock:
            if self._closing:
                raise RuntimeError("控制器正在关闭")
            if self._disconnecting:
                raise RuntimeError("串口正在断开")
            if self._estop_in_progress:
                raise RuntimeError("软件急停正在执行")
            if self._hardware_estop_active:
                raise RuntimeError("硬件急停已触发")
            if self._estop_unconfirmed and not allow_estop_retry:
                raise RuntimeError(
                    "ESTOP 未获确认，普通运动已锁定；请重试急停或物理断电"
                )
            generation = self._control_generation
        if not self._is_serial_connected():
            raise RuntimeError("串口未连接")
        return generation

    def _stop_all_outputs(self, allow_closing=False):
        """发送固件原子软件急停；返回是否收到精确确认。"""
        with self.state_lock:
            if getattr(self, "_gait_owned", {}):
                self._gait_needs_recovery = True
                if self._gait_run is not None:
                    self._gait_run.request_stop()
                self._gait_release_ownership()
            self._control_generation += 1
            self._track_lease_generation += 1
            for axis in range(NUM_STEPPER_AXES):
                self._axis_motion_generation[axis] += 1
                if self._axis_motion_active_locked(axis):
                    self.axis_motion_telemetry[axis] = (
                        self.axis_motion_telemetry[axis].stopping()
                    )
                self.running[axis] = False
                self._move_reservation[axis] = None
                self._web_step_pending[axis] = None
            for axis in range(NUM_MOTOR_AXES):
                self._foc_enabled_ui[axis] = False
            self._track_direction = "STOP"
        response = self._send_and_read("ESTOP", timeout=0.8, allow_closing=allow_closing)
        confirmed = response == "OK,ESTOP"
        # 新固件 ESTOP 内部已全轴恢复锁定；这里再补发一次，兼容旧固件
        # 与 ESTOP 未确认场景，绝不让旋转电机停留在落地纠偏的失能态。
        self._force_relock_rot_axes(allow_closing=True)
        with self.state_lock:
            self._estop_unconfirmed = not confirmed
        status = "已停止" if confirmed else "⚠ 停车未确认"
        self._post_ui(lambda value=status: self.v_track_status.set(value))
        return confirmed

    def attach_web_server(self, server):
        """保存 WebControlServer 实例，供外部启动/关闭流程统一管理。"""
        self.web_server = server
        return server

    def web_get_status(self):
        """兼容旧调用方；HTTP 服务直接使用 ``DesktopWebController``。"""
        return self._web_controller.web_get_status()

    @staticmethod
    def _web_direction(direction):
        return DesktopWebController.parse_direction(direction)

    def web_stepper_move(self, axis, direction, distance_mm, speed_mm_s=SPEED_DEFAULT):
        return self._web_controller.web_stepper_move(
            axis, direction, distance_mm, speed_mm_s)

    def web_stepper_stop(self, axis):
        return self._web_controller.web_stepper_stop(axis)

    def web_stepper_config(self, axis, mode=None, pulse_per_rev=None, gear_ratio=None, lead_mm=None):
        return self._web_controller.web_stepper_config(
            axis,
            mode=mode,
            pulse_per_rev=pulse_per_rev,
            gear_ratio=gear_ratio,
            lead_mm=lead_mm,
        )

    def web_motor_command(self, mode, axis, action, target_deg=None):
        return self._web_controller.web_motor_command(
            mode, axis, action, target_deg=target_deg)

    def web_track_command(self, action, pwm=0, lease_ms=0):
        return self._web_controller.web_track_command(
            action, pwm=pwm, lease_ms=lease_ms)

    def web_emergency_stop(self):
        return self._web_controller.web_emergency_stop()

    # ═════════════ FOC：自动调参（单轴）═════════════
    def _foc_autotune(self, axis):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("未连接", "请先连接串口"); return
        if not messagebox.askokcancel(
            "自动调参确认",
            f"闭环 {MOTOR_AXIS_LABELS[axis]} 两阶段扫描 PA + VP，约 2 分钟。\n电机会来回转动，请先固定好。"):
            return
        self.fw[axis]['autotune_btn'].config(state="disabled")
        if self._start_control_worker(self._foc_autotune_worker, axis) is None:
            self.fw[axis]['autotune_btn'].config(state="normal")

    def _step_response_test(self, axis, target, pre_settle=2.5, duration=5.0):
        metrics = self._autotune_runner.step_response_test(
            axis, target, pre_settle_s=pre_settle, duration_s=duration)
        if metrics is None:
            return None
        return (
            metrics.overshoot_deg,
            metrics.steady_state_error_deg,
            metrics.rise_time_s,
            metrics.jitter_deg,
        )

    def _foc_autotune_worker(self, axis):
        generation = getattr(self._control_context, "generation", None)
        try:
            result = self._autotune_runner.run_foc(axis)
            if not result.completed:
                return
            best_pa = result.best_pa
            best_vp = result.best_vp
            def apply_result():
                with self.state_lock:
                    if (
                        generation != self._control_generation
                        or self._closing
                        or self._disconnecting
                        or self._estop_in_progress
                        or self._hardware_estop_active
                        or self._estop_unconfirmed
                    ):
                        return
                self.v_focpangle[axis].set(float(best_pa))
                self.fw[axis]['pangle_label'].config(text=f"{best_pa:.1f} (刚度)")
                self.v_focvp[axis].set(float(best_vp))
                self.fw[axis]['vp_label'].config(text=f"{best_vp:.2f} (阻尼)")
                self._save_foc_tune()
            self._post_ui(apply_result)
        finally:
            self._post_ui(lambda: self.fw[axis]['autotune_btn'].config(state="normal"))

    # ═════════════ GEAR：自动调参（三阶段 Kp → Kd → Ki）═════════════
    def _gear_autotune(self, axis):
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("未连接", "请先连接串口"); return
        if not messagebox.askokcancel(
            "GEAR 自动调参",
            f"闭环 {MOTOR_AXIS_LABELS[axis]} 三阶段扫描 Kp → Kd → Ki，约 3 分钟。\n"
            f"电机会反复在 0° ↔ 60° 之间走，请确认机械空间够。"):
            return
        self.gw[axis]['autotune_btn'].config(state="disabled")
        if self._start_control_worker(self._gear_autotune_worker, axis) is None:
            self.gw[axis]['autotune_btn'].config(state="normal")

    def _gear_autotune_worker(self, axis):
        generation = getattr(self._control_context, "generation", None)
        try:
            result = self._autotune_runner.run_gear(axis)
            if not result.completed:
                return
            best_kp = result.best_kp
            best_kd = result.best_kd
            best_ki = result.best_ki
            def apply_result():
                with self.state_lock:
                    if (
                        generation != self._control_generation
                        or self._closing
                        or self._disconnecting
                        or self._estop_in_progress
                        or self._hardware_estop_active
                        or self._estop_unconfirmed
                    ):
                        return
                self.v_gearkp[axis].set(float(best_kp))
                self.gw[axis]['kp_label'].config(text=f"{best_kp:.2f}")
                self.v_gearkd[axis].set(float(best_kd))
                self.gw[axis]['kd_label'].config(text=f"{best_kd:.3f}")
                self.v_gearki[axis].set(float(best_ki))
                self.gw[axis]['ki_label'].config(text=f"{best_ki:.2f}")
                self._save_gear_tune()
            self._post_ui(apply_result)
        finally:
            self._post_ui(lambda: self.gw[axis]['autotune_btn'].config(state="normal"))

    # ═════════════ 日志 ═════════════
    def _classify_log(self, msg):
        if any(k in msg for k in ("⛔", "ERR", "FAULT", "失败", "异常", "拒绝")):
            return "err"
        if any(k in msg for k in ("⚠️", "warn")):
            return "warn"
        if "轴L" in msg or "axis 0" in msg or "FOC,0," in msg or "MOVE,0" in msg:
            return "axisL"
        if "轴R" in msg or "axis 1" in msg or "FOC,1," in msg or "MOVE,1" in msg:
            return "axisR"
        if any(k in msg for k in ("✓", "OK", "DONE", "完成", "通过")):
            return "ok"
        if msg.startswith("MOT:") or msg.startswith("[FOC"):
            return "rx"
        return ""

    def _log_file_path(self):
        return os.path.join(LOG_DIR, f"gui_{time.strftime('%Y-%m-%d')}.log")

    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        full = f"{ts}  {msg}"
        # 写日期分割的日志文件（追加）
        try:
            with open(self._log_file_path(), "a", encoding="utf-8") as f:
                f.write(full + "\n")
        except Exception:
            pass
        # UI 上色显示
        tag = self._classify_log(msg)
        def _append():
            self.log_text.config(state="normal")
            if tag:
                self.log_text.insert("end", full + "\n", tag)
            else:
                self.log_text.insert("end", full + "\n")
            self.log_text.see("end")
            self.log_text.config(state="disabled")
        self._post_ui(_append)

    def clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _open_log_dir(self):
        try:
            os.startfile(LOG_DIR)  # Windows
        except Exception:
            messagebox.showinfo("日志目录", LOG_DIR)

    def _open_user_guide(self):
        """在默认浏览器打开本地使用说明 docs/user_guide.html。"""
        from pathlib import Path

        path = Path(PROJECT_DIR) / "docs" / "user_guide.html"
        if not path.is_file():
            messagebox.showerror("使用说明缺失", f"未找到使用说明文件：\n{path}")
            return
        try:
            webbrowser.open(path.resolve().as_uri())
            self.log(f"已打开使用说明：{path}")
        except Exception as exc:
            messagebox.showerror(
                "打开失败", f"无法打开使用说明：{exc}\n\n文件位置：{path}")

    def _toggle_raw_log(self):
        """打开/关闭串口原始流文件记录（每条 RX 行写到独立 log）。"""
        if self.raw_log_var.get():
            try:
                path = os.path.join(LOG_DIR, f"raw_{time.strftime('%Y%m%d_%H%M%S')}.log")
                self._raw_log_fh = open(path, "w", encoding="utf-8")
                self.log(f"串口原始流 → {os.path.basename(path)}")
            except Exception as e:
                self.log(f"⚠️ 原始流打开失败: {e}")
                self.raw_log_var.set(False)
        else:
            if self._raw_log_fh:
                try: self._raw_log_fh.close()
                except Exception: pass
                self._raw_log_fh = None
                self.log("串口原始流已停止")
