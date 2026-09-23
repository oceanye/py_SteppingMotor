"""三足轮换步态的分阶段执行器（干跑通过后、按阶段人工确认执行）.

``gait_planner`` 给出纯数学（S0..S7 阶段、组内并列的相对运动）；本模块
把它变成对既有运动链路的顺序调用：

- 组序 = 时间序：上一组全部到达终态（DONE）才发下一组；
- S4同步组：一条SYNC原子下发，两轴共用五次进度；宿主不支持则拒绝，
  不通过独立MOVE近似同步；
- 任何一轴 ABORT/TIMEOUT/下发失败 → 停掉本次用到的全部轴，状态
  置为 aborted/failed，后续阶段拒绝执行（ABORT 后位置不可信，运动
  链路本身也会拒绝）。

宿主协议（desktop_app 实现，测试用替身替换）：

- ``role_axis(role_name) -> int``          绑定解析；缺失抛 GaitExecutorError
- ``send_relative(axis, delta, speed) -> str``   "sent" / "noop" / "failed"
- ``send_synchronized(moves, duration_s) -> str``  原子同步组，仅允许"sent"
- ``send_synchronized_endpoint(moves, duration_s, endpoints) -> str``
  分段组使用同一 S4 原点的累计终点，段间等待双轴 DONE，不累计相对取整误差
- ``wait_terminal(axis, timeout_s) -> str``      "DONE"/"ABORTED"/"TIMEOUT"/"CANCELLED"
- ``stop_axes(axes)``                            中止时停轴
- ``cancelled() -> bool``                        急停/关闭/控制代数失效
- ``log(message)``

本模块不 import Tk、不碰串口。
"""

from __future__ import annotations

import threading
from typing import Callable, Sequence

from .gait_planner import GaitParams, GaitStage

# 每组的等待超时 = 3×理论时长 + 15s（首段 ACK/进度帧延迟留足余量）。
GROUP_TIMEOUT_MARGIN_S = 15.0
GROUP_TIMEOUT_FACTOR = 3.0
WAIT_POLL_S = 0.02


class GaitExecutorError(RuntimeError):
    """执行器拒绝继续（绑定缺失、状态非法等）。"""


class GaitExecutor:
    """一次摆动（A→C 或 B→A）的阶段推进状态机。"""

    def __init__(
        self,
        host,
        params: GaitParams,
        stages: Sequence[GaitStage],
        *,
        side: str,
    ):
        if side not in ("left", "right"):
            raise GaitExecutorError("side 必须是 left 或 right")
        self._host = host
        self.params = params
        self.stages = tuple(stages)
        if not self.stages:
            raise GaitExecutorError("阶段计划为空")
        self.side = side
        self.state = "ready"  # ready / running / done / aborted / failed
        self.last_error: str | None = None
        self._stage_index = 0
        self._lock = threading.Lock()
        self._stop_requested = False

    # ── UI/状态读取 ─────────────────────────────────────────

    @property
    def stage_index(self) -> int:
        with self._lock:
            return self._stage_index

    def current_stage(self) -> GaitStage | None:
        with self._lock:
            if self._stage_index >= len(self.stages):
                return None
            return self.stages[self._stage_index]

    def describe(self) -> dict:
        """当前推进状态的 UI 快照（可在任意线程调用）。"""

        with self._lock:
            stage = (
                None
                if self._stage_index >= len(self.stages)
                else self.stages[self._stage_index]
            )
            return {
                "side": self.side,
                "state": self.state,
                "stage_index": self._stage_index,
                "stage_count": len(self.stages),
                "stage_id": None if stage is None else stage.stage_id,
                "title": None if stage is None else stage.title,
                "confirm_text": None if stage is None else stage.confirm_text,
                "is_motion_stage": False if stage is None else stage.is_motion_stage,
                "move_group_count":
                    0 if stage is None else len(stage.move_groups),
                "last_error": self.last_error,
            }

    def request_stop(self) -> None:
        """UI 中止按钮：请求停止并让正在等待的循环尽快退出。"""

        self._stop_requested = True

    def begin_stage_execution(self) -> None:
        """UI 在启动工作线程前抢占状态，防止重复点击执行按钮。"""

        with self._lock:
            if self.state == "ready":
                self.state = "running"

    def revert_stage_execution(self) -> None:
        """工作线程没能启动时撤销抢占，让按钮恢复可用。"""

        with self._lock:
            if self.state == "running":
                self.state = "ready"

    # ── 推进 ────────────────────────────────────────────────

    def advance_confirm(self) -> bool:
        """推进一个确认型阶段（S0/S1/S5/S7，无运动）。"""

        with self._lock:
            if self.state not in ("ready", "running"):
                return False
            stage = (
                None
                if self._stage_index >= len(self.stages)
                else self.stages[self._stage_index]
            )
            if stage is None or stage.is_motion_stage:
                return False
            self._stage_index += 1
            if self._stage_index >= len(self.stages):
                self.state = "done"
            return True

    def execute_current_stage(
        self,
        progress: Callable[[str, int, int], None] | None = None,
    ) -> bool:
        """执行当前运动阶段（S2/S3/S4/S6）的全部组；成功后推进指针。

        只能从控制工作线程调用（阻塞等待各轴终态）。
        """

        with self._lock:
            if self.state not in ("ready", "running"):
                raise GaitExecutorError(
                    f"执行器状态为 {self.state}，不能执行新阶段"
                )
            stage = (
                None
                if self._stage_index >= len(self.stages)
                else self.stages[self._stage_index]
            )
            if stage is None:
                raise GaitExecutorError("所有阶段已完成")
            if not stage.is_motion_stage:
                raise GaitExecutorError(
                    f"{stage.stage_id} 是确认型阶段，请走确认推进"
                )
            self.state = "running"

        self._host.log(
            f"▶ 步态 {stage.stage_id} {stage.title}："
            f"{len(stage.move_groups)} 组运动开始"
        )
        ok = self._run_stage_groups(stage, progress)
        if not ok:
            return False

        with self._lock:
            self._stage_index += 1
            finished = self._stage_index >= len(self.stages)
            if finished:
                self.state = "done"
            else:
                # 运动阶段完成后回到 ready 语义：等待下一次人工确认。
                self.state = "ready"
        self._host.log(f"✓ 步态 {stage.stage_id} {stage.title} 完成")
        if finished:
            self._host.log("本次摆动全部阶段完成")
        return True

    # ── 内部 ────────────────────────────────────────────────

    def _fail(self, state: str, message: str) -> None:
        with self._lock:
            self.state = state
            self.last_error = message
        self._host.log(f"⛔ 步态中止：{message}")

    def _abort_check(self, stage: GaitStage) -> bool:
        if self._stop_requested:
            self._stop_stage_axes(stage, "UI 中止请求")
            self._fail("aborted", "用户按了【中止】")
            return True
        if self._host.cancelled():
            self._stop_stage_axes(stage, "控制器急停/关闭")
            self._fail("aborted", "控制器急停或正在关闭")
            return True
        return False

    def _stop_stage_axes(self, stage: GaitStage, reason: str) -> None:
        axes: list[int] = []
        for role in ("Mup1", "Mr1", "Mup2", "Mr2"):
            try:
                axes.append(self._host.role_axis(role))
            except GaitExecutorError:
                continue
        if axes:
            self._host.log(f"停止步态用轴（{reason}）")
            self._host.stop_axes(axes)

    def _run_stage_groups(
        self,
        stage: GaitStage,
        progress: Callable[[str, int, int], None] | None,
    ) -> bool:
        total = len(stage.move_groups)
        for group_index, group in enumerate(stage.move_groups):
            if self._abort_check(stage):
                return False
            if not self._run_group(stage, group, group_index):
                return False
            if progress is not None:
                progress(stage.stage_id, group_index + 1, total)
        return True

    def _run_group(self, stage: GaitStage, group, group_index=0) -> bool:
        dispatched: list[tuple[int, str]] = []  # (axis, role)
        expected_s = stage.duration_s if stage.synchronized else 0.0
        if stage.group_durations:
            expected_s = stage.group_durations[group_index]
        if stage.synchronized:
            try:
                moves = [(self._host.role_axis(m.role), m.delta, m.speed) for m in group]
                if stage.sync_endpoints:
                    sender = getattr(self._host, "send_synchronized_endpoint", None)
                    result = (None if sender is None else sender(
                        moves, expected_s, stage.sync_endpoints[group_index]))
                else:
                    sender = getattr(self._host, "send_synchronized", None)
                    result = None if sender is None else sender(moves, expected_s)
                if result != "sent":
                    raise GaitExecutorError("固件/宿主不支持原子同步轨迹或预检未通过；不允许独立 MOVE 降级")
                dispatched = [(axis, m.role) for (axis, _, _), m in zip(moves, group)]
            except Exception as exc:
                self._stop_stage_axes(stage, "同步下发失败")
                self._fail("failed", str(exc))
                return False
        else:
            return self._run_independent_group(stage, group)

        timeout_s = GROUP_TIMEOUT_FACTOR * expected_s + GROUP_TIMEOUT_MARGIN_S
        for axis, role in dispatched:
            terminal = self._host.wait_terminal(axis, timeout_s)
            if terminal != "DONE":
                self._stop_stage_axes(stage, "同步轨迹未完整完成")
                self._fail("aborted" if terminal in ("ABORTED", "CANCELLED") else "failed",
                           f"{role} 同步轨迹 {terminal}；位置可能不可信，停止全部四轴并重新校准")
                return False
        if self._abort_check(stage):
            return False
        return True

    def _run_independent_group(self, stage, group) -> bool:
        dispatched = []
        expected_s = 0.0
        for move in group:
            if self._abort_check(stage):
                return False
            try:
                axis = self._host.role_axis(move.role)
            except GaitExecutorError as exc:
                self._fail("failed", f"{move.role} 绑定解析失败: {exc}")
                self._stop_stage_axes(stage, "绑定缺失")
                return False
            result = self._host.send_relative(axis, move.delta, move.speed)
            if result == "failed":
                self._fail(
                    "failed",
                    f"{move.role}（轴{axis}）下发失败：{move.delta:+.3f} @ "
                    f"{move.speed:g}/s",
                )
                self._stop_stage_axes(stage, "下发失败")
                return False
            if result == "noop":
                self._host.log(
                    f"{move.role} 本段位移过小（{move.delta:+.3f}），"
                    "折算不足 1 脉冲，已跳过"
                )
                continue
            dispatched.append((axis, move.role))
            expected_s = max(expected_s, abs(move.delta) / move.speed)

        timeout_s = (
            GROUP_TIMEOUT_FACTOR * expected_s + GROUP_TIMEOUT_MARGIN_S
        )
        for axis, role in dispatched:
            terminal = self._host.wait_terminal(axis, timeout_s)
            if terminal == "DONE":
                continue
            if terminal == "ABORTED":
                self._stop_stage_axes(stage, "轴中止")
                self._fail(
                    "aborted", f"{role}（轴{axis}）运动被中止；位置已不可信"
                )
            elif terminal == "CANCELLED":
                self._stop_stage_axes(stage, "控制代数失效")
                self._fail("aborted", "控制器急停或正在关闭")
            else:
                self._stop_stage_axes(stage, "等待超时")
                self._fail(
                    "failed",
                    f"{role}（轴{axis}）{timeout_s:.0f}s 内未收到完成回报",
                )
            return False
        return True


__all__ = [
    "GaitExecutor",
    "GaitExecutorError",
    "GROUP_TIMEOUT_FACTOR",
    "GROUP_TIMEOUT_MARGIN_S",
]
