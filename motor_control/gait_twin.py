"""Read-only gait pose from axis feedback, never control feedback.

XY is normalized by the beam length (d=1), so the live schematic needs no
additional geometry inputs. Z is displacement from the current step's starting
axis coordinate, not a contact/ground-height measurement.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Mapping, Sequence

from motor_control.gait_planner import GaitParams, LOW_NODE_PHASE_DEG
from motor_control.gait_avoidance import lattice_coordinates


PAD_CENTERS = {"A": (-1.0, 0.0), "B": (0.0, 0.0),
               "C": (-0.5, math.sqrt(3) / 2)}
ROLES = ("Mup1", "Mr1", "Mup2", "Mr2")


def axis_motion_active(axis):
    """X42S retains its last target at rest; a target alone is not motion.

    Missing/stale status is not permission to edit a live physical reference.
    Legacy pulse axes retain their original pending-target semantics.
    """
    if str(axis.get("position_source", "")).startswith("x42s"):
        # No live actuator: allow offline simulation. Physical reference edits
        # still have their own online/enabled/fresh-feedback preflight.
        if (axis.get("binding_valid") is False or axis.get("state") in (
                "UNBOUND", "INVALID", "DISCONNECTED", "X42S_OFFLINE", "X42S_BACKEND_PENDING")):
            return False
        live = axis.get("x42s") or {}
        speed = live.get("speed")
        # 2026-10-19 实机定案：X42S 静止时驱动器常不置速度 valid 位
        # （speed=None 是常态，四台实测全部如此）——速度未知不构成
        # "在运动"；静止证据=位置可信+遥测新鲜+reached。速度仅在
        # 驱动器上报了数值时才判非零。
        return (not axis.get("position_trusted") or bool(axis.get("stale"))
                or live.get("reached") is not True
                or (speed is not None
                    and (not _finite(speed) or abs(speed) > 0.01)))
    return (axis.get("target_position") is not None
            or axis.get("state") in ("STARTING", "MOVING", "CONTINUOUS"))


class TwinHistory:
    """Bounded display-only path; invalid telemetry always breaks the line.

    A new step does not clear world history. Only a new physical reference or
    an explicit clear does. No targets, wall-clock interpolation or motor IO.
    """

    def __init__(self, max_points=6000):
        self.points = deque(maxlen=max_points)
        self.landings = deque(maxlen=250)
        self.world_key = None
        self.clear()

    def clear(self, snapshot=None):
        self.points.clear()
        self.landings.clear()
        self.completed_steps = 0
        self._segment = 0
        self.current_step = 0
        self.segment_steps = {}
        self._last_reference = None
        self._healthy = False
        self._last_done = (snapshot.get("reference_key")
                           if snapshot and snapshot.get("step_done") else None)
        if snapshot:
            self.world_key = snapshot.get("world_key")

    def observe(self, snapshot):
        world = snapshot.get("world_key")
        if world != self.world_key:
            self.clear()
            self.world_key = world
        pose = snapshot.get("pose")
        if pose is None or snapshot.get("state") not in ("estimated", "waiting"):
            self._healthy = False
            return
        left, right = (tuple(pose["feet"][s]["center"]) for s in ("left", "right"))
        center = tuple((a+b)/2 for a, b in zip(left, right))
        if not all(math.isfinite(v) for point in (left, right) for v in point):
            self._healthy = False
            return
        # 世界 ψ（左右）随点记录：实机轨迹要画三爪端折线（与评估层同款），
        # 只有中心点不够。旧测试替身不带 psi_deg → None，绘制时跳过爪线。
        psis = tuple(pose["feet"][s].get("psi_deg") for s in ("left", "right"))
        ref = snapshot.get("reference_key")
        if not self.current_step or ref != self._last_reference:
            self.current_step += 1
        if not self._healthy or ref != self._last_reference:
            self._segment += 1
            self.segment_steps[self._segment] = self.current_step
            # Match the bounded point history; a long session must not leak metadata.
            retained = {p[0] for p in self.points} | {self._segment}
            self.segment_steps = {k: v for k, v in self.segment_steps.items() if k in retained}
        point = (self._segment, left, right, center, psis)
        if not self.points or point != self.points[-1]:
            self.points.append(point)
        self._healthy, self._last_reference = True, ref
        if snapshot.get("step_done") and ref != self._last_done:
            self._last_done = ref
            self.completed_steps += 1
            self.landings.append((self.completed_steps, left, right))

    def segments(self):
        """Split on steps and telemetry gaps; never draw an invented connector."""
        chunks = []
        for point in self.points:
            if not chunks or chunks[-1][-1][0] != point[0]:
                chunks.append([])
            chunks[-1].append(point)
        return chunks

    def recent_segments(self):
        """Return (age, segment) for current/previous STEP, not telemetry chunk.

        A disconnect splits a line but must not make the same physical step
        look like a previous step. All older paths stay out of the display.
        """
        chunks = self.segments()
        steps = [self.segment_steps.get(chunk[0][0], chunk[0][0]) for chunk in chunks]
        current = self.current_step or (steps[-1] if steps else 0)
        previous = next((s for s in reversed(steps) if s != current), None)
        return [(0 if step == current else 1, chunk) for step, chunk in zip(steps, chunks)
                if step == current or step == previous]


def pad_center(name):
    if name in PAD_CENTERS:
        return PAD_CENTERS[name]
    i, j = lattice_coordinates(name)
    return i+j/2, math.sqrt(3)*j/2


def build_twin_snapshot(
    params: GaitParams,
    axes: Sequence[Mapping],
    *,
    context: Mapping | None = None,
    calibration_valid: bool = False,
    needs_recovery: bool = False,
) -> dict:
    """Use measured X42S axes or legacy pulse accounting, never command targets.

    Axis feedback is measured; world pose is kinematic inference with assumed
    support/contact and calibrated geometry, not externally measured XYZ.
    """
    motors = {str(a["role"]): dict(a) for a in axes}
    # X42S 后端：位置为驱动器实测（快照层已 ÷gear_ratio 换算输出轴），
    # 与步进轴的脉冲估算分列标注，UI 不得混淆两种来源。
    all_x42s = bool(axes) and all(
        str(a.get("position_source", "")).startswith("x42s") for a in axes
    )
    result = {"source": "x42s_measured" if all_x42s else "commanded_pulses",
              "measured": (all_x42s and len(motors) == len(ROLES)
                           and all(a.get("position_trusted") and _finite(a.get("position"))
                                   for a in motors.values())),
              "pose_measured": False,
              "xy_unit": "beam_length", "axes": motors,
              "pose": None, "target_pose": None, "state": "unavailable",
              "message": "等待四个电机绑定", "reference_key": None}
    if any(r not in motors or not motors[r].get("binding_valid") for r in ROLES):
        return result
    if context is not None:
        result["reference_key"] = context.get("reference_key")
    # X42S_ONLINE 与步进轴的运行态同级放行（2026-10-10 接入真实遥测）：
    # X42S 后端的 position 已在快照层 ÷gear_ratio 换算为输出轴角度；
    # X42S_OFFLINE/FAULT/STALE/BACKEND_PENDING/DISCONNECTED 不在白名单，
    # 走 untrusted 保留最近有效估算。
    bad_states = {str(a.get("state", "")) for a in motors.values()} - {
        "IDLE", "STARTING", "MOVING", "CONTINUOUS", "X42S_ONLINE"}
    if bad_states or any(not motors[r].get("position_trusted") for r in ROLES):
        result.update(state="untrusted", message="断连 / 停止 / 位置失信；保留最近有效估算")
        return result
    if needs_recovery:
        result.update(state="untrusted", message="流程中止，需重建基准；保留最近有效估算")
        return result
    if not calibration_valid:
        result.update(state="uncalibrated", reference_key=None,
                      message="轴坐标可查看；世界姿态需完成零位及方向标定")
        return result
    if context is None or context.get("manual_invalid", False):
        result.update(
            state="axis_only",
            message=("显示输出轴坐标；开始步态后按已确认支点推算公转"
                     if all_x42s else
                     "显示电机轴坐标；开始步态后按已确认支点推算公转"),
        )
        return result
    pending = [motors[r] for r in ROLES
               if motors[r].get("target_position") is not None
               and (not all_x42s or axis_motion_active(motors[r]))]
    if any(a.get("stale") for a in pending):
        result.update(state="stale", message="脉冲进度超过1秒未更新；保留最近有效估算")
        return result
    if any(a.get("continuous") for a in motors.values()):
        result.update(state="axis_only", message="独立连续运动中，支点未确认；只显示轴坐标")
        return result
    if not all(_finite(motors[r].get("position")) for r in ROLES):
        return result

    positions = {r: float(motors[r]["position"]) for r in ROLES}
    result["pose"] = _pose(params, positions, context)
    if pending:
        targets = {r: (float(motors[r]["target_position"])
                       if _finite(motors[r].get("target_position")) else positions[r]) for r in ROLES}
        result["target_pose"] = _pose(params, targets, context)
    waiting = (not all_x42s) and any(
        a.get("executed_steps") is None for a in pending
    )
    if all_x42s:
        # 实测位置无需"脉冲进度"确认，直接进入实测态。
        result.update(state="estimated",
                      message="X42S 轴位反馈（mm/°）· 整机姿态按支点与机械模型推算")
    else:
        result.update(state="waiting" if waiting else "estimated",
                      message=("指令目标已显示，等待首帧脉冲进度" if waiting else
                               "脉冲估算 · 无编码器 · 不包含失步或机械滑动"))
    return result


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _pose(params, positions, context):
    side = context["side"]
    start, target, pivot, bearing = context["route"]
    support_role = "Mr2" if side == "left" else "Mr1"
    support_sign = params.mr2_sign if side == "left" else params.mr1_sign
    rotation_start = context.get("rotation_start")
    phi = (0.0 if rotation_start is None else support_sign *
           (positions[support_role] - rotation_start[support_role]))
    beta = bearing - (180.0 if side == "right" else 0.0) - phi
    px, py = pad_center(pivot)
    theta = math.radians(bearing - phi)
    swing_center = (px + math.cos(theta), py + math.sin(theta))
    feet = {}
    for foot, rotation, lift, sign, zero, lift_sign in (
        ("left", "Mr1", "Mup1", params.mr1_sign, params.mr1_zero_deg, params.mup1_lift_sign),
        ("right", "Mr2", "Mup2", params.mr2_sign, params.mr2_zero_deg, params.mup2_lift_sign),
    ):
        q = sign * (positions[rotation] - zero)
        feet[foot] = {"center": swing_center if foot == side else (px, py),
                      "psi_deg": LOW_NODE_PHASE_DEG + q + beta - params.beam_reference_deg,
                      "q_deg": q,
                      "z_mm": lift_sign * (positions[lift] - context["lift_start"][lift]),
                      "support_assumed": foot != side}
    return {"phi_deg": phi, "beta_deg": beta, "feet": feet,
            "route": f"{start}→{target}，支点{pivot}", "side": side,
            "route_pads": {name: pad_center(name) for name in (start, target, pivot)}}
