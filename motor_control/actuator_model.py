"""X42S UART 执行器遥测模型（feature/x42s-uart-actuator-binding，2026-09-28）.

按《REQUIREMENTS_2026-09-28_X42S_UART_ACTUATOR_BINDING》§12：遥测必须区分
command（指令位置）/ measured（驱动器回报的实测位置）/ error（跟随误差），
轮询起步 10–20Hz。本模块只做纯数据模型与新鲜度/可信度判定，不含串口、
线程或 Tk 依赖——数据来源（X42S 后端）自里程碑 M4 起接入。
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import time
from typing import Sequence

from .coordinated_control import (
    LOGICAL_ROLE_ORDER,
    LogicalRole,
    X42S_DRIVER_ID_MAX,
    X42S_DRIVER_ID_MIN,
)

# 10–20Hz 轮询 ⇒ 两个周期（200ms）内视为新鲜；500ms 起视为失联。
TELEMETRY_FRESH_MAX_AGE_MS = 1500
TELEMETRY_STALE_AGE_MS = 3000
# 2026-10-20 实机定案：四台轮转全量轮询的每台刷新周期≈700ms
# （25ms tick × 4 台 × 7 帧事务），运动中挂起探测≈300ms——旧值
# 200/500ms 意味着常态遥测永远"不新鲜"：孪生态势图恒显示"已冻结
# （断连/失信）"，姿态/柱状图不画（分阶段执行画面全静止的根因）。
# 掉线检测由固件层 OFFLINE 判定承担，age 阈值只管缓存新鲜度。

# 驱动器可上报的异常标签（§9 安全集；具体与 X42S 状态字的映射待手册核对）。
KNOWN_X42S_FAULTS: tuple[str, ...] = (
    "stall",          # 堵转/失步
    "over_current",   # 过流
    "over_temp",      # 过热
    "under_voltage",  # 母线低压
    "follow_error",   # 跟随误差超限
    "comm_error",     # 通讯错误（校验/超时/错配计数累计）
    "addr_conflict",  # 总线上检测到重复 ID（M2 扫描发现）
)


@dataclass(frozen=True, slots=True)
class X42sActuatorState:
    """单台 X42S 驱动器的瞬时状态投影（只读、可 JSON 化）。

    所有 float 字段为 None 表示"未知/本轮未上报"，绝不回退成 0——
    显示层不得把未知画成 0 位置。
    """

    role: LogicalRole
    driver_id: int
    online: bool = False
    enabled: bool = False
    command_position: float | None = None
    measured_position: float | None = None
    position_error: float | None = None
    speed: float | None = None
    current_a: float | None = None
    temperature_c: float | None = None
    reached: bool = False
    fault: str | None = None
    last_update_monotonic: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.role, LogicalRole):
            raise ValueError("role 必须是 LogicalRole")
        if self.role not in LOGICAL_ROLE_ORDER:
            raise ValueError("role 必须是四个机构角色之一")
        if (
            isinstance(self.driver_id, bool)
            or not isinstance(self.driver_id, int)
            or not X42S_DRIVER_ID_MIN <= self.driver_id <= X42S_DRIVER_ID_MAX
        ):
            raise ValueError("driver_id 必须在 1..255（0 为广播地址，禁止绑定）")
        if self.fault is not None and self.fault not in KNOWN_X42S_FAULTS:
            raise ValueError(f"未知异常标签：{self.fault}")

    @classmethod
    def offline(cls, role: LogicalRole, driver_id: int) -> "X42sActuatorState":
        """掉线投影：online=False 且不携带任何位置数据。"""

        return cls(role=role, driver_id=driver_id, online=False)

    def with_telemetry(
        self,
        *,
        now: float | None = None,
        **fields: object,
    ) -> "X42sActuatorState":
        """记录一次成功轮询：online=True 并刷新时间戳（字段不得为 None 值）。"""

        stamp = time.monotonic() if now is None else float(now)
        updates = dict(fields)
        updates.update({"online": True, "last_update_monotonic": stamp})
        return replace(self, **updates)

    def age_ms(self, *, now: float | None = None) -> int | None:
        if self.last_update_monotonic is None:
            return None
        stamp = time.monotonic() if now is None else float(now)
        return max(0, int(round((stamp - self.last_update_monotonic) * 1000)))

    @property
    def telemetry_trusted(self) -> bool:
        """位置可信 = 在线 + 实测位置已上报 + 时间戳在新鲜窗口内。

        注意：属性形式无 now 参数，按调用时刻的 time.monotonic() 计算；
        精确判定请用 telemetry_trusted_at(now=...)。
        """

        return self.telemetry_trusted_at(now=time.monotonic())

    def telemetry_trusted_at(self, *, now: float) -> bool:
        if not self.online or self.measured_position is None:
            return False
        age = self.age_ms(now=now)
        if age is None:
            return False
        return age <= TELEMETRY_FRESH_MAX_AGE_MS

    def as_snapshot_dict(self, *, now: float | None = None) -> dict[str, object]:
        """桌面/Web 共用的 JSON 兼容投影；label 统一为 "X42S ID N"（§5.4）。"""

        stamp = time.monotonic() if now is None else float(now)
        age = self.age_ms(now=stamp)
        return {
            "role": self.role.value,
            "driver_id": self.driver_id,
            "label": f"X42S ID{self.driver_id}",
            "online": self.online,
            "enabled": self.enabled,
            "command_position": self.command_position,
            "measured_position": self.measured_position,
            "position_error": self.position_error,
            "speed": self.speed,
            "current_a": self.current_a,
            "temperature_c": self.temperature_c,
            "reached": self.reached,
            "fault": self.fault,
            "last_update_age_ms": age,
            "stale": age is None or age > TELEMETRY_STALE_AGE_MS,
            "telemetry_trusted": self.telemetry_trusted_at(now=stamp),
        }


def x42s_state_issues(states: Sequence["X42sActuatorState"]) -> tuple[str, ...]:
    """对一组执行器状态做跨台一致性检查（§10 掉线定位、地址冲突）。

    返回人类可读的问题描述；空元组 = 无问题。仅做纯数据检查，
    通讯重试策略属 X42S 总线层（M2）。
    """

    issues: list[str] = []
    seen_ids: dict[int, str] = {}
    for state in states:
        if state.driver_id in seen_ids:
            issues.append(
                f"X42S ID{state.driver_id} 同时绑定 {seen_ids[state.driver_id]} 与"
                f" {state.role.value}（地址冲突）"
            )
        else:
            seen_ids[state.driver_id] = state.role.value
        if not state.online:
            issues.append(
                f"{state.role.value} / X42S ID{state.driver_id} offline"
            )
    return tuple(issues)
