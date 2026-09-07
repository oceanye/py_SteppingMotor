"""三足轮换步态的纯规划数学（干跑 + 分阶段执行的离线核心）.

依据 ``docs/handoff control method.txt``（A→C 绕 B 公转方案）实现：

- 五次平滑曲线 ``S(s)=10s³-15s⁴+6s⁵``；
- 左三足中心 ``O1(φ) = B + d·dir(180°-φ)``，φ∈[0°,60°]；
- 同步自转 ``ψ1 = ψ10 + 2φ``（ψ10=30°，低节点相位）；
- 横梁角 ``β = 180° - 60°·S(s)``，关节角 ``q = ψ - β + c``；
- 一个摆动循环里：摆动侧电机 Δq=+180°、支撑侧电机 Δq=+60°；
- 高点避让：爪臂（胶囊体）对每个六边形高节点（带半径圆）的连续
  轨迹间隙 ``D_jk = dist - r_爪 - r_高点 - δ > 0``；壳体正下方的
  "飞越节点"除外（2D 模型判不了垂向间隙，上报为需目视确认项）。

本模块不 import Tk、不碰串口：所有几何/轨迹/碰撞/阶段计划都可以离线
单元测试。执行与标定向导在 ``desktop_app`` 里基于这里的纯函数搭建。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Sequence

# ── 角度约定 ──────────────────────────────────────────────
# 对外 API 一律用"度"；内部三角函数换算弧度。
# 世界系俯视图：B 在原点，A 在 180° 方向，C 在 120° 方向（ABC 等边）。
SWING_ARC_DEG = 60.0        # 一次公转角
SWING_SPIN_DEG = 120.0      # 摆动侧自转角（= 2×公转角）
LOW_NODE_PHASE_DEG = 30.0   # 低节点起始相位 ψ10（同时是三根爪臂之一的方向）
HIGH_NODE_PHASE_DEG = 90.0  # 高节点相位（与低节点相间 60°）

SWING_JOINT_DELTA_DEG = 180.0  # 摆动侧电机关节角总变化 Δq = 120-(-60)
SUPPORT_JOINT_DELTA_DEG = 60.0  # 支撑侧电机补偿角总变化 Δq = 0-(-60)


def smoothstep5(s: float) -> float:
    """五次平滑曲线：S(0)=0，S(1)=1，两端一阶导为 0。"""

    s = min(1.0, max(0.0, float(s)))
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def _dir(deg: float) -> tuple[float, float]:
    rad = math.radians(deg)
    return (math.cos(rad), math.sin(rad))


def _dot(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _sub(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    return (a[0] - b[0], a[1] - b[1])


def _scale(a: tuple[float, float], k: float) -> tuple[float, float]:
    return (a[0] * k, a[1] * k)


def _add(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    return (a[0] + b[0], a[1] + b[1])


def point_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    """点到线段的最近距离（mm）。"""

    seg = _sub(end, start)
    seg_len_sq = _dot(seg, seg)
    if seg_len_sq <= 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    t = _dot(_sub(point, start), seg) / seg_len_sq
    t = min(1.0, max(0.0, t))
    closest = _add(start, _scale(seg, t))
    delta = _sub(point, closest)
    return math.hypot(delta[0], delta[1])


# ══════════════════════════════════════════════════════════
# 参数模型（几何 + 标定 + 执行节拍）
# ══════════════════════════════════════════════════════════

@dataclass(frozen=True)
class GaitGeometry:
    """俯视几何。所有长度 mm，占位默认值需现场实测后覆盖。"""

    d_mm: float = 220.0             # AB = BC 六边形中心距（占位默认，需实测覆盖）
    arm_length_mm: float = 40.0     # 爪臂长度 = 六边形节点环半径（落脚一致性）
    hub_radius_mm: float = 12.0     # 三足中心壳体等效半径
    arm_radius_mm: float = 4.0      # 爪臂等效半径（胶囊粗细）
    node_radius_mm: float = 5.0     # 高节点等效半径
    safety_margin_mm: float = 2.0   # δ：要求的最小安全间隙

    def validated(self) -> "GaitGeometry":
        positive = (
            "d_mm", "arm_length_mm", "hub_radius_mm", "arm_radius_mm",
            "node_radius_mm", "safety_margin_mm",
        )
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"几何参数 {name} 必须是大于 0 的有限数值")
        if self.hub_radius_mm >= self.arm_length_mm:
            raise ValueError("壳体半径必须小于爪臂长度")
        return self


@dataclass(frozen=True)
class GaitParams:
    """干跑与分阶段执行的全部可调参数（schema v1，JSON 持久化）。"""

    geometry: GaitGeometry = field(default_factory=GaitGeometry)
    # 执行节拍
    swing_segments: int = 12          # S4 公转拆成的 MOVE 段数
    lift_mm: float = 10.0             # z_clear：抬足高度（轴行程单位）
    swing_speed_deg_s: float = 6.0    # 支撑侧（横梁驱动）电机速度
    lift_speed_mm_s: float = 2.0      # 抬足速度
    settle_speed_mm_s: float = 1.0    # 落足速度（更慢）
    feasibility_samples: int = 120    # 干跑碰撞校验采样密度
    # 标定：电机方向符号与基准零位（向导写入；+1 表示轴坐标增大 = q 增大）
    mr1_sign: int = 1
    mr2_sign: int = 1
    mup1_lift_sign: int = 1           # +1：轴坐标增大 = 抬升
    mup2_lift_sign: int = 1
    # 标定基准：记零时各 Mr 轴的软件坐标（度）；ψ 基准固定为 30°
    mr1_zero_deg: float | None = None
    mr2_zero_deg: float | None = None

    SCHEMA = "gait-params-v1"

    def validated(self) -> "GaitParams":
        self.geometry.validated()
        if not 1 <= int(self.swing_segments) <= 200:
            raise ValueError("swing_segments 必须在 1..200")
        if not 1 <= int(self.feasibility_samples) <= 5000:
            raise ValueError("feasibility_samples 必须在 1..5000")
        for name in ("lift_mm", "swing_speed_deg_s", "lift_speed_mm_s",
                     "settle_speed_mm_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"执行参数 {name} 必须是大于 0 的有限数值")
        for name in ("mr1_sign", "mr2_sign", "mup1_lift_sign", "mup2_lift_sign"):
            if getattr(self, name) not in (1, -1):
                raise ValueError(f"方向符号 {name} 必须是 +1 或 -1")
        for name in ("mr1_zero_deg", "mr2_zero_deg"):
            value = getattr(self, name)
            if value is not None and not math.isfinite(value):
                raise ValueError(f"零位 {name} 必须是有限数值")
        return self

    # 摆动侧与支撑侧按 3:1 同步，保证每段两者同时完成。
    @property
    def swing_side_speed_deg_s(self) -> float:
        return 3.0 * self.swing_speed_deg_s

    def as_document(self) -> dict[str, Any]:
        geometry = self.geometry
        return {
            "schema": self.SCHEMA,
            "geometry": {
                "d_mm": geometry.d_mm,
                "arm_length_mm": geometry.arm_length_mm,
                "hub_radius_mm": geometry.hub_radius_mm,
                "arm_radius_mm": geometry.arm_radius_mm,
                "node_radius_mm": geometry.node_radius_mm,
                "safety_margin_mm": geometry.safety_margin_mm,
            },
            "swing_segments": int(self.swing_segments),
            "lift_mm": self.lift_mm,
            "swing_speed_deg_s": self.swing_speed_deg_s,
            "lift_speed_mm_s": self.lift_speed_mm_s,
            "settle_speed_mm_s": self.settle_speed_mm_s,
            "feasibility_samples": int(self.feasibility_samples),
            "mr1_sign": self.mr1_sign,
            "mr2_sign": self.mr2_sign,
            "mup1_lift_sign": self.mup1_lift_sign,
            "mup2_lift_sign": self.mup2_lift_sign,
            "mr1_zero_deg": self.mr1_zero_deg,
            "mr2_zero_deg": self.mr2_zero_deg,
        }


def parse_gait_params(value: Mapping[str, Any] | None) -> GaitParams:
    """从 JSON 文档恢复参数；缺失字段用默认值，非法整体失败。"""

    if value is None:
        return GaitParams()
    if not isinstance(value, Mapping):
        raise ValueError("步态参数文档必须是对象")
    schema = value.get("schema")
    if schema != GaitParams.SCHEMA:
        raise ValueError(f"步态参数 schema 不支持: {schema!r}")

    def _number(key: str, default: float) -> float:
        raw = value.get(key, default)
        try:
            result = float(raw)
        except (TypeError, ValueError):
            raise ValueError(f"步态参数 {key} 不是数值") from None
        return result

    def _opt_number(key: str) -> float | None:
        raw = value.get(key)
        if raw is None:
            return None
        try:
            return float(raw)
        except (TypeError, ValueError):
            raise ValueError(f"步态参数 {key} 不是数值") from None

    def _sign(key: str) -> int:
        raw = value.get(key, 1)
        if raw not in (1, -1):
            raise ValueError(f"步态参数 {key} 必须是 +1 或 -1")
        return int(raw)

    geometry_raw = value.get("geometry", {})
    if not isinstance(geometry_raw, Mapping):
        raise ValueError("geometry 必须是对象")
    geometry = GaitGeometry(
        d_mm=float(geometry_raw.get("d_mm", GaitGeometry.d_mm)),
        arm_length_mm=float(
            geometry_raw.get("arm_length_mm", GaitGeometry.arm_length_mm)),
        hub_radius_mm=float(
            geometry_raw.get("hub_radius_mm", GaitGeometry.hub_radius_mm)),
        arm_radius_mm=float(
            geometry_raw.get("arm_radius_mm", GaitGeometry.arm_radius_mm)),
        node_radius_mm=float(
            geometry_raw.get("node_radius_mm", GaitGeometry.node_radius_mm)),
        safety_margin_mm=float(
            geometry_raw.get("safety_margin_mm",
                             GaitGeometry.safety_margin_mm)),
    )
    params = GaitParams(
        geometry=geometry,
        swing_segments=int(_number("swing_segments",
                                   GaitParams.swing_segments)),
        lift_mm=_number("lift_mm", GaitParams.lift_mm),
        swing_speed_deg_s=_number("swing_speed_deg_s",
                                  GaitParams.swing_speed_deg_s),
        lift_speed_mm_s=_number("lift_speed_mm_s",
                                GaitParams.lift_speed_mm_s),
        settle_speed_mm_s=_number("settle_speed_mm_s",
                                  GaitParams.settle_speed_mm_s),
        feasibility_samples=int(_number("feasibility_samples",
                                        GaitParams.feasibility_samples)),
        mr1_sign=_sign("mr1_sign"),
        mr2_sign=_sign("mr2_sign"),
        mup1_lift_sign=_sign("mup1_lift_sign"),
        mup2_lift_sign=_sign("mup2_lift_sign"),
        mr1_zero_deg=_opt_number("mr1_zero_deg"),
        mr2_zero_deg=_opt_number("mr2_zero_deg"),
    )
    return params.validated()


# ══════════════════════════════════════════════════════════
# 几何：六边形节点、圆弧、爪臂
# ══════════════════════════════════════════════════════════

@dataclass(frozen=True)
class HexPad:
    """一个固定六边形：中心 + 朝向角（度）。低/高节点相间 60°。"""

    name: str
    center: tuple[float, float]
    orientation_deg: float = 0.0

    def node_center(
        self,
        phase_deg: float,
        index: int,
        ring_radius_mm: float,
    ) -> tuple[float, float]:
        angle = self.orientation_deg + phase_deg + 120.0 * (index % 3)
        return _add(self.center, _scale(_dir(angle), ring_radius_mm))

    def low_nodes(self, ring_radius_mm: float) -> list[tuple[float, float]]:
        return [
            self.node_center(LOW_NODE_PHASE_DEG, k, ring_radius_mm)
            for k in range(3)
        ]

    def high_nodes(self, ring_radius_mm: float) -> list[tuple[float, float]]:
        return [
            self.node_center(HIGH_NODE_PHASE_DEG, k, ring_radius_mm)
            for k in range(3)
        ]


def default_hex_pads(geometry: GaitGeometry) -> dict[str, HexPad]:
    """A/B/C 三个固定六边形：B 为原点，A 在 180°，C 在 120°，间距 d。"""

    d = geometry.d_mm
    return {
        "B": HexPad("B", (0.0, 0.0)),
        "A": HexPad("A", _scale(_dir(180.0), d)),
        "C": HexPad("C", _scale(_dir(120.0), d)),
    }


def swing_center(
    pivot: HexPad, radius_mm: float, phi_deg: float, start_bearing_deg: float
) -> tuple[float, float]:
    """摆动三足中心：绕 pivot 半径 radius 的圆弧上，方位角 start_bearing-φ。"""

    return _add(
        pivot.center, _scale(_dir(start_bearing_deg - phi_deg), radius_mm)
    )


# 左摆动（A→C，支点 B）：中心从 A(180°) 顺 60° 到 C(120°)。
LEFT_SWING = ("A", "C", "B", 180.0)
# 右摆动（B→A，支点 C）：中心从 B(C→B 方位 300°) 到 A(C→A 方位 240°)。
RIGHT_SWING = ("B", "A", "C", 300.0)


@dataclass(frozen=True)
class SwingSample:
    """轨迹上一步的完整状态（干跑与执行共用）。"""

    s: float
    phi_deg: float
    psi_deg: float            # 摆动三足绝对姿态
    beta_deg: float           # 横梁方位（pivot→摆动端）
    center: tuple[float, float]
    margin_mm: float | None   # 该采样处最小间隙（None = 未做碰撞校验）


@dataclass(frozen=True)
class DryRunReport:
    feasible: bool
    min_margin_mm: float
    min_margin_sample: SwingSample | None
    samples: tuple[SwingSample, ...]
    hexagons: tuple[HexPad, ...]
    message: str
    # 轨迹中从壳体正上方越过的高节点（2D 干跑不校验，执行时目视确认）
    hub_passover_nodes: tuple[str, ...] = ()

    def as_summary(self) -> dict[str, Any]:
        worst = self.min_margin_sample
        return {
            "feasible": self.feasible,
            "min_margin_mm": self.min_margin_mm,
            "worst_phi_deg": None if worst is None else worst.phi_deg,
            "worst_psi_deg": None if worst is None else worst.psi_deg,
            "sample_count": len(self.samples),
            "hub_passover_nodes": list(self.hub_passover_nodes),
            "message": self.message,
        }


def arm_segment(
    center: tuple[float, float],
    psi_deg: float,
    arm_index: int,
    geometry: GaitGeometry,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """第 arm_index 根爪臂的胶囊中线段：从壳体边缘到爪臂末端。"""

    direction = _dir(psi_deg + 120.0 * (arm_index % 3))
    start = _add(center, _scale(direction, geometry.hub_radius_mm))
    end = _add(center, _scale(direction, geometry.arm_length_mm))
    return start, end


def node_under_hub(
    node: tuple[float, float],
    center: tuple[float, float],
    geometry: GaitGeometry,
) -> bool:
    """该高节点是否位于抬起三足壳体的正下方（2D 模型判不了的"飞越事件"）。

    中心圆弧路径与起点/目标六边形的邻弧高节点恒定贴近（最近距离
    ≈ ring²/(2d)），任何 d 都躲不开。这类节点由抬升的壳体从正上方
    越过，水平间隙无意义，只能靠执行时的垂向间隙确认（文档 §7 把
    壳体列为需 3D 数据的检查项）。
    """

    distance = math.hypot(node[0] - center[0], node[1] - center[1])
    return distance <= geometry.hub_radius_mm + geometry.node_radius_mm


def clearance_margin_mm(
    center: tuple[float, float],
    psi_deg: float,
    geometry: GaitGeometry,
    hexagons: Sequence[HexPad],
    *,
    include_hub: bool = False,
) -> float:
    """当前 (φ,ψ) 位形下：3 根爪臂（可选含壳体）对所有高节点的最小间隙。

    返回值已扣除爪臂/节点等效半径与安全裕度 δ；> 0 才安全。
    不检查低节点（落脚/支撑足与它们同层接触，不是障碍）。

    ``include_hub`` 默认关：壳体位于抬升高度上，2D 零高度代理会把
    "末段中心必然进入目标六边形节点环"的几何事实误报成碰撞（文档 §7
    的壳体检查需要真实高度数据，留待实测后做 3D 校验）。
    壳体正下方（见 :func:`node_under_hub`）的节点跳过臂检查——它们
    由抬升壳体越过，2D 零高度代理给不出有效结论。
    """

    worst = math.inf
    high_nodes: list[tuple[str, tuple[float, float]]] = []
    for hexagon in hexagons:
        for node in hexagon.high_nodes(geometry.arm_length_mm):
            high_nodes.append((hexagon.name, node))
    for arm_index in range(3):
        start, end = arm_segment(center, psi_deg, arm_index, geometry)
        for _name, node in high_nodes:
            if node_under_hub(node, center, geometry):
                continue
            distance = point_segment_distance(node, start, end)
            worst = min(worst, distance - geometry.arm_radius_mm)
    if include_hub:
        for _name, node in high_nodes:
            worst = min(
                worst,
                math.hypot(node[0] - center[0], node[1] - center[1])
                - geometry.hub_radius_mm,
            )
    return worst - geometry.node_radius_mm - geometry.safety_margin_mm


def plan_swing_trajectory(
    params: GaitParams,
    *,
    side: str,
    extra_hexagons: Iterable[HexPad] = (),
) -> DryRunReport:
    """干跑：生成整条摆动轨迹并做连续碰撞校验。

    ``side`` 是 "left"（A→C 绕 B）或 "right"（B→A 绕 C）。
    """

    params = params.validated()
    geometry = params.geometry
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    _start, _target, pivot_name, start_bearing = (
        LEFT_SWING if side == "left" else RIGHT_SWING
    )
    hexagons = dict(default_hex_pads(geometry))
    for extra in extra_hexagons:
        hexagons[extra.name] = extra
    pivot = hexagons[pivot_name]

    samples: list[SwingSample] = []
    high_nodes: list[tuple[str, tuple[float, float]]] = []
    for hexagon in hexagons.values():
        for k, node in enumerate(hexagon.high_nodes(geometry.arm_length_mm)):
            high_nodes.append(
                (f"{hexagon.name}·{HIGH_NODE_PHASE_DEG + 120.0 * k:g}°高节点",
                 node)
            )
    hub_passovers: list[str] = []
    for index in range(int(params.feasibility_samples) + 1):
        s = index / float(params.feasibility_samples)
        shaping = smoothstep5(s)
        phi_deg = SWING_ARC_DEG * shaping
        psi_deg = (LOW_NODE_PHASE_DEG
                   + SWING_SPIN_DEG * shaping)
        beta_deg = start_bearing - SWING_ARC_DEG * shaping
        center = swing_center(pivot, geometry.d_mm, phi_deg, start_bearing)
        for label, node in high_nodes:
            if (label not in hub_passovers
                    and node_under_hub(node, center, geometry)):
                hub_passovers.append(label)
        margin = clearance_margin_mm(center, psi_deg, geometry,
                                     list(hexagons.values()))
        samples.append(
            SwingSample(s=s, phi_deg=phi_deg, psi_deg=psi_deg,
                        beta_deg=beta_deg, center=center, margin_mm=margin)
        )
    worst = min(samples, key=lambda item: item.margin_mm or math.inf)
    feasible = (worst.margin_mm is not None and worst.margin_mm > 0.0)
    if feasible:
        message = (f"可行：全轨迹最小间隙 {worst.margin_mm:.2f} mm "
                   f"(φ={worst.phi_deg:.1f}°, ψ={worst.psi_deg:.1f}°)")
    else:
        message = (f"不可行：最小间隙 {worst.margin_mm:.2f} mm ≤ 0 "
                   f"(φ={worst.phi_deg:.1f}°, ψ={worst.psi_deg:.1f}°)；"
                   "先增大抬足高度/间隙参数或修正几何后重试")
    if hub_passovers:
        message += ("；" + "、".join(hub_passovers)
                    + " 从壳体正上方越过（2D 干跑不校验，"
                    "执行时需目视确认垂向间隙）")
    return DryRunReport(
        feasible=feasible,
        min_margin_mm=worst.margin_mm or 0.0,
        min_margin_sample=worst,
        samples=tuple(samples),
        hexagons=tuple(hexagons.values()),
        message=message,
        hub_passover_nodes=tuple(hub_passovers),
    )


# ══════════════════════════════════════════════════════════
# 阶段计划：S0..S7（分阶段人工确认执行）
# ══════════════════════════════════════════════════════════

@dataclass(frozen=True)
class RoleMove:
    """一个逻辑角色的一次相对运动（轴坐标已含方向符号）。"""

    role: str          # "Mr1" / "Mr2" / "Mup1" / "Mup2"
    delta: float       # 轴单位（Mr 为度，Mup 为 mm），已乘方向符号
    speed: float       # 轴单位/秒


@dataclass(frozen=True)
class GaitStage:
    """一个阶段：说明 + 顺序的运动组序列；每组内并列下发、全部完成才进下一组。

    组序 = 时间序（S4 的第 k 段必须等第 k-1 段完成），组内 = 同时下发
    （摆动侧与支撑侧按 3:1 速度比近似同步完成）。
    """

    stage_id: str
    title: str
    confirm_text: str  # 人工确认清单（接触/姿态检查）
    move_groups: tuple[tuple[RoleMove, ...], ...] = ()

    @property
    def is_motion_stage(self) -> bool:
        return bool(self.move_groups)


def _segment_progress(index: int, total: int) -> tuple[float, float]:
    """第 index 段（0 基）的 [S(s_i), S(s_{i+1})] 平滑进度区间。"""

    s0 = index / float(total)
    s1 = (index + 1) / float(total)
    return smoothstep5(s0), smoothstep5(s1)


def plan_gait_stages(
    params: GaitParams,
    *,
    side: str,
    swing_psi_start_deg: float | None = None,
) -> list[GaitStage]:
    """生成一次完整摆动的 S0..S7 阶段计划。

    ``swing_psi_start_deg`` 是摆动三足当前绝对姿态（由执行器按标定零位
    换算）；等于 30°（低节点基准）时 S3 为空并自动跳过。None 表示
    只生成计划不做相位修正（用于纯展示）。
    """

    params = params.validated()
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    swing_role, support_role = (
        ("Mr1", "Mr2") if side == "left" else ("Mr2", "Mr1")
    )
    lift_role = "Mup1" if side == "left" else "Mup2"
    swing_sign = params.mr1_sign if swing_role == "Mr1" else params.mr2_sign
    support_sign = (params.mr2_sign if support_role == "Mr2"
                    else params.mr1_sign)
    lift_sign = (params.mup1_lift_sign if lift_role == "Mup1"
                 else params.mup2_lift_sign)

    side_text = "左三足 A→C（支点 B）" if side == "left" else "右三足 B→A（支点 C）"

    stages: list[GaitStage] = [
        GaitStage(
            stage_id="S0",
            title="前置检查",
            confirm_text=(
                f"开始 {side_text}：确认绑定完整、位置可信、干跑校验通过，\n"
                "左/右三足分别踩在起始低节点上。"
            ),
        ),
        GaitStage(
            stage_id="S1",
            title="锁定支撑足",
            confirm_text=(
                "确认支撑三足完全踩实低节点、无松动；\n"
                "抬起过程中支撑侧就是公转支点。"
            ),
        ),
    ]

    # S2 抬起摆动足
    stages.append(
        GaitStage(
            stage_id="S2",
            title="抬起摆动足",
            confirm_text="确认摆动三足三个接触点全部离地，与高节点有垂向间隙。",
            move_groups=(
                (RoleMove(lift_role, lift_sign * params.lift_mm,
                          params.lift_speed_mm_s),),
            ),
        )
    )

    # S3 相位调整（目标 ψ=30° 基准；通常在基准位则跳过）
    if swing_psi_start_deg is not None:
        phase_delta = swing_psi_start_deg - LOW_NODE_PHASE_DEG
        # 归一化到 (-180, 180]，走最短转向
        phase_delta = (phase_delta + 180.0) % 360.0 - 180.0
        if abs(phase_delta) > 0.5:
            stages.append(
                GaitStage(
                    stage_id="S3",
                    title="摆动足相位调整",
                    confirm_text="爪臂进入扫掠状态：确认无任何爪臂指向高节点方向。",
                    move_groups=(
                        (RoleMove(swing_role,
                                  -swing_sign * phase_delta,
                                  params.swing_side_speed_deg_s),),
                    ),
                )
            )

    # S4 公转 + 自转（段间顺序、段内并列：摆动侧与支撑侧按 3:1 速度同时完成）
    segment_groups: list[tuple[RoleMove, ...]] = []
    for index in range(int(params.swing_segments)):
        progress0, progress1 = _segment_progress(
            index, int(params.swing_segments))
        swing_delta = (progress1 - progress0) * SWING_JOINT_DELTA_DEG
        support_delta = (progress1 - progress0) * SUPPORT_JOINT_DELTA_DEG
        segment_groups.append(
            (
                RoleMove(swing_role, swing_sign * swing_delta,
                         params.swing_side_speed_deg_s),
                RoleMove(support_role, support_sign * support_delta,
                         params.swing_speed_deg_s),
            )
        )
    stages.append(
        GaitStage(
            stage_id="S4",
            title="公转 + 同步自转",
            confirm_text=(
                f"共 {int(params.swing_segments)} 段同步运动；观察爪臂始终从"
                "高点间隙中扫过。任何异常立即点【中止】。"
            ),
            move_groups=tuple(segment_groups),
        )
    )

    stages.extend(
        [
            GaitStage(
                stage_id="S5",
                title="接近目标位",
                confirm_text="目视检查爪臂相位与目标六边形低节点一一对应。",
            ),
            GaitStage(
                stage_id="S6",
                title="落脚",
                confirm_text="低速下放；只有三个低节点接触均有效后才允许进入锁定。",
                move_groups=(
                    (RoleMove(lift_role, -lift_sign * params.lift_mm,
                              params.settle_speed_mm_s),),
                ),
            ),
            GaitStage(
                stage_id="S7",
                title="锁定完成",
                confirm_text="确认三足踩实、姿态正确；本次摆动结束。",
            ),
        ]
    )
    return stages


def swing_phase_correction_deg(
    params: GaitParams, role: str, current_axis_deg: float
) -> float:
    """按标定零位换算当前绝对姿态 ψ，返回到 30° 基准的最短修正角。"""

    zero = params.mr1_zero_deg if role == "Mr1" else params.mr2_zero_deg
    sign = params.mr1_sign if role == "Mr1" else params.mr2_sign
    if zero is None:
        raise ValueError(f"{role} 尚未做零位标定")
    psi = LOW_NODE_PHASE_DEG + sign * (current_axis_deg - zero)
    # 修正角 = 目标 ψ − 当前 ψ（归一化到 (-180,180] 走最短路径）
    return (LOW_NODE_PHASE_DEG - psi + 180.0) % 360.0 - 180.0
