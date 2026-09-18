"""三足轮换步态的纯规划数学（干跑 + 分阶段执行的离线核心）.

依据 ``docs/handoff control method.txt``（A→C 绕 B 公转方案）实现：

- 五次平滑曲线 ``S(s)=10s³-15s⁴+6s⁵``；
- 左三足中心 ``O1(φ) = B + d·dir(180°-φ)``，φ∈[0°,60°]；
- 同步自转 ``ψ1 = ψ10 + 2φ``（ψ10=30°，低节点相位）；
- 横梁角 ``β = 180° - 60°·S(s)``，关节角 ``q = ψ - β + c``；
- 一个摆动循环里：摆动侧电机 Δq=+180°、支撑侧电机 Δq=+60°；
- 高点避让：邻接六边形、爪臂、中心结构、横梁与实测垂向包络；
  采样间运动距离界给出连续间隙保守下界，不排除壳体下的高点。

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
# 终点三重对称不意味着中途轨迹等效。下方legacy候选函数不能用于公转
# 的同步避障轨迹，只能供将来独立验证的悬空原地解绕规划参考。


def unwrap_swing_joint_delta(
    theta_deg: float,
    rotation_limit_deg: float,
    *,
    support_delta_deg: float = SUPPORT_JOINT_DELTA_DEG,
) -> float:
    """按摆动侧累计角 θ 选位形等效的摆动关节增量 Δq（度）。

    约束：摆动后 θ+Δq 与随后作支撑 θ+Δq+support 都必须落在
    ±rotation_limit_deg 内（支撑侧着地不能解绕，必须在它自己悬空时
    就留好余量）。候选 Δq = 60+120k 中取 |θ+Δq| 最小者（让累计角贴着
    窗口中央徘徊，线缆缠绕量最小）；并列取 |Δq| 小、再并列取正。
    θ 已在窗外时仍可一步拉回（Δq 无界），但 |Δq| 超过两圈即认为累计角
    失真，拒绝并要求人工处理。
    """

    limit = float(rotation_limit_deg)
    base = SWING_JOINT_DELTA_DEG - SWING_SPIN_DEG  # = 60
    lo = -limit + support_delta_deg
    hi = limit - support_delta_deg
    if lo > hi:
        raise ValueError("rotation_limit_deg 太小：摆动后无法留出支撑余量")
    # Δq = base + SWING_SPIN_DEG·k 且 θ+Δq ∈ [lo, hi] 的整数 k 范围
    k_lo = math.ceil((lo - theta_deg - base) / SWING_SPIN_DEG)
    k_hi = math.floor((hi - theta_deg - base) / SWING_SPIN_DEG)
    # 目标 |θ+Δq| 最小 → k* ≈ -(θ+base)/120；在合法范围内取最近的候选
    target_k = round(-(theta_deg + base) / SWING_SPIN_DEG)
    best = None
    for k in sorted({max(k_lo, min(k_hi, target_k) + d) for d in (-1, 0, 1)}):
        if not k_lo <= k <= k_hi:
            continue
        delta = base + SWING_SPIN_DEG * k
        landed = theta_deg + delta
        key = (abs(landed), abs(delta), -delta)
        if best is None or key < best[0]:
            best = (key, delta)
    assert best is not None
    if abs(best[1]) > 720.0:
        raise ValueError(
            f"摆动侧累计角 {theta_deg:.1f}° 偏离解绕窗口 ±{limit:g}° "
            "超过两圈：请先把该侧手动转回窗口内（或重新记零）再开始")
    return best[1]


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
    high_node_height_mm: float = 12.0  # 相对低节点，必须实测
    body_drop_mm: float = 0.0       # 壳体/电机/轴承最低点低于爪臂中心线的量
    beam_height_mm: float = 30.0    # 横梁中心线相对低节点高度，必须实测
    beam_radius_mm: float = 4.0    # 横梁/连接件保守胶囊包络
    surrounding_pads: bool = True  # 检查紧邻的六边形，不只 A/B/C

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
        if self.d_mm + 1e-8 < math.sqrt(3) * self.arm_length_mm:
            raise ValueError("六边形中心距小于紧贴正六边形要求，节点环模型重叠；请核实几何")
        for name in ("high_node_height_mm", "body_drop_mm"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} 必须是非负有限数值")
        for name in ("beam_height_mm", "beam_radius_mm"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} 必须是正有限数值")
        if not isinstance(self.surrounding_pads, bool):
            raise ValueError("surrounding_pads 必须是布尔值")
        return self


@dataclass(frozen=True)
class GaitParams:
    """统一预览与实际分阶段执行参数；旧v1文档须重新建立零位签名。"""

    geometry: GaitGeometry = field(default_factory=GaitGeometry)
    # 执行节拍
    swing_segments: int = 12          # 展示密度；不会切分实际同步指令
    lift_mm: float = 10.0             # z_clear：抬足高度（轴行程单位）
    swing_speed_deg_s: float = 6.0    # 支撑侧（横梁驱动）电机速度
    lift_speed_mm_s: float = 2.0      # 抬足速度
    settle_speed_mm_s: float = 1.0    # 落足速度（更慢）
    feasibility_samples: int = 120    # 干跑碰撞校验采样密度
    # 每侧Mr线缆角度限制；轨迹不能放入窗口时拒绝，不替换中途角度关系。
    rotation_limit_deg: float = 180.0
    # 标定：电机方向符号与基准零位（向导写入；+1 表示轴坐标增大 = q 增大）
    mr1_sign: int = 1
    mr2_sign: int = 1
    mup1_lift_sign: int = 1           # +1：轴坐标增大 = 抬升
    mup2_lift_sign: int = 1
    # 标定基准：记零时各 Mr 轴的软件坐标（度）；ψ 基准固定为 30°
    mr1_zero_deg: float | None = None
    mr2_zero_deg: float | None = None
    phase_gain: float = 2.0          # Δψ/φ；不受中心距/升降行程影响
    beam_reference_deg: float = 180.0  # 两个 Mr 同时记零时横梁世界角
    calibration_confirmed: bool = False  # 实测几何、方向、PPR、反力闭合已确认
    calibration_fingerprint: str | None = None
    mr1_zero_signature: str | None = None
    mr2_zero_signature: str | None = None

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
        if (not math.isfinite(self.rotation_limit_deg)
                or not 120.0 < self.rotation_limit_deg <= 720.0):
            raise ValueError("rotation_limit_deg 必须在 (120, 720] 度"
                             "（窗口小于 120 时摆动小步无处安放）")
        for name in ("mr1_sign", "mr2_sign", "mup1_lift_sign", "mup2_lift_sign"):
            if getattr(self, name) not in (1, -1):
                raise ValueError(f"方向符号 {name} 必须是 +1 或 -1")
        for name in ("mr1_zero_deg", "mr2_zero_deg"):
            value = getattr(self, name)
            if value is not None and not math.isfinite(value):
                raise ValueError(f"零位 {name} 必须是有限数值")
        if (not math.isfinite(self.phase_gain) or not 0 < self.phase_gain <= 10
                or abs(self.phase_gain * SWING_ARC_DEG % 120.0) > 1e-8):
            raise ValueError("phase_gain 必须为 2/4/6/8/10；终点须对准三重对称低节点")
        if not math.isfinite(self.beam_reference_deg):
            raise ValueError("beam_reference_deg 必须是有限数值")
        if not isinstance(self.calibration_confirmed, bool):
            raise ValueError("calibration_confirmed 必须是布尔值")
        if self.calibration_fingerprint is not None and not isinstance(self.calibration_fingerprint, str):
            raise ValueError("calibration_fingerprint 必须是字符串或 null")
        for value in (self.mr1_zero_signature, self.mr2_zero_signature):
            if value is not None and not isinstance(value, str):
                raise ValueError("旋转零位签名必须是字符串或 null")
        return self

    # 传统摆动侧速度（3×支撑速度）。S4 解绕后按增量比另算段速；
    # 此值现用于 S3 相位调整等单轴整段运动。
    @property
    def swing_side_speed_deg_s(self) -> float:
        return (self.phase_gain + 1.0) * self.swing_speed_deg_s

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
                "high_node_height_mm": geometry.high_node_height_mm,
                "body_drop_mm": geometry.body_drop_mm,
                "beam_height_mm": geometry.beam_height_mm,
                "beam_radius_mm": geometry.beam_radius_mm,
                "surrounding_pads": geometry.surrounding_pads,
            },
            "swing_segments": int(self.swing_segments),
            "lift_mm": self.lift_mm,
            "swing_speed_deg_s": self.swing_speed_deg_s,
            "lift_speed_mm_s": self.lift_speed_mm_s,
            "settle_speed_mm_s": self.settle_speed_mm_s,
            "feasibility_samples": int(self.feasibility_samples),
            "rotation_limit_deg": self.rotation_limit_deg,
            "mr1_sign": self.mr1_sign,
            "mr2_sign": self.mr2_sign,
            "mup1_lift_sign": self.mup1_lift_sign,
            "mup2_lift_sign": self.mup2_lift_sign,
            "mr1_zero_deg": self.mr1_zero_deg,
            "mr2_zero_deg": self.mr2_zero_deg,
            "phase_gain": self.phase_gain,
            "beam_reference_deg": self.beam_reference_deg,
            "calibration_confirmed": self.calibration_confirmed,
            "calibration_fingerprint": self.calibration_fingerprint,
            "mr1_zero_signature": self.mr1_zero_signature,
            "mr2_zero_signature": self.mr2_zero_signature,
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
        high_node_height_mm=float(geometry_raw.get("high_node_height_mm", 12.0)),
        body_drop_mm=float(geometry_raw.get("body_drop_mm", 0.0)),
        beam_height_mm=float(geometry_raw.get("beam_height_mm", 30.0)),
        beam_radius_mm=float(geometry_raw.get("beam_radius_mm", 4.0)),
        surrounding_pads=geometry_raw.get("surrounding_pads", True),
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
        rotation_limit_deg=_number("rotation_limit_deg",
                                   GaitParams.rotation_limit_deg),
        mr1_sign=_sign("mr1_sign"),
        mr2_sign=_sign("mr2_sign"),
        mup1_lift_sign=_sign("mup1_lift_sign"),
        mup2_lift_sign=_sign("mup2_lift_sign"),
        mr1_zero_deg=_opt_number("mr1_zero_deg"),
        mr2_zero_deg=_opt_number("mr2_zero_deg"),
        phase_gain=_number("phase_gain", 2.0),
        beam_reference_deg=_number("beam_reference_deg", 180.0),
        calibration_confirmed=value.get("calibration_confirmed", False),
        calibration_fingerprint=value.get("calibration_fingerprint"),
        mr1_zero_signature=value.get("mr1_zero_signature"),
        mr2_zero_signature=value.get("mr2_zero_signature"),
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

    def __post_init__(self):
        if (len(self.center) != 2 or not all(math.isfinite(v) for v in self.center)
                or not math.isfinite(self.orientation_deg)):
            raise ValueError("六边形中心及方向必须是有限数值")

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
    pads = {
        "B": HexPad("B", (0.0, 0.0)),
        "A": HexPad("A", _scale(_dir(180.0), d)),
        "C": HexPad("C", _scale(_dir(120.0), d)),
    }
    if geometry.surrounding_pads:
        # 三角晶格：覆盖 A/B/C 外的一圈邻座。与爪臂半径无关的 d 只参与几何。
        for i in range(-2, 2):
            for j in range(-1, 3):
                center = (d * (i + j / 2.0), d * math.sqrt(3) * j / 2.0)
                if not any(math.dist(center, p.center) < 1e-8 for p in pads.values()):
                    pads[f"邻座({i},{j})"] = HexPad(f"邻座({i},{j})", center)
    return pads


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
    swing_q_delta_deg: float = 0.0
    support_q_delta_deg: float = 0.0


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
    """仅诊断水平投影重叠；这不允许跳过高点或放行未校验的飞越。"""

    distance = math.hypot(node[0] - center[0], node[1] - center[1])
    return distance <= geometry.hub_radius_mm + geometry.node_radius_mm


def clearance_margin_mm(
    center: tuple[float, float],
    psi_deg: float,
    geometry: GaitGeometry,
    hexagons: Sequence[HexPad],
    *,
    include_hub: bool = True,
    lift_mm: float = 0.0,
    pivot: tuple[float, float] | None = None,
) -> float:
    """爪胶囊、中心结构圆盘、横梁胶囊对有高度的高节点的包络间隙。

    >0才安全；不排除任何节点。升降位移、最低点下伸和横梁最低中心线
    高度必须实测；低节点触地仍须独立接触确认，不由此模型判定。
    """

    worst = math.inf
    high_nodes: list[tuple[str, tuple[float, float]]] = []
    for hexagon in hexagons:
        for node in hexagon.high_nodes(geometry.arm_length_mm):
            high_nodes.append((hexagon.name, node))
    for arm_index in range(3):
        start, end = arm_segment(center, psi_deg, arm_index, geometry)
        for _name, node in high_nodes:
            distance = point_segment_distance(node, start, end)
            vertical = max(0.0, lift_mm - geometry.high_node_height_mm)
            margin = (math.hypot(max(0.0, distance - geometry.node_radius_mm), vertical)
                      - geometry.arm_radius_mm - geometry.safety_margin_mm)
            worst = min(worst, margin)
    if include_hub:
        for _name, node in high_nodes:
            lateral = max(0.0, math.dist(node, center)
                          - geometry.hub_radius_mm - geometry.node_radius_mm)
            vertical = max(0.0, lift_mm - geometry.body_drop_mm
                           - geometry.high_node_height_mm)
            worst = min(worst, math.hypot(lateral, vertical) - geometry.safety_margin_mm)
    if pivot is not None:
        for _name, node in high_nodes:
            lateral = max(0.0, point_segment_distance(node, pivot, center)
                          - geometry.node_radius_mm)
            vertical = max(0.0, geometry.beam_height_mm - geometry.high_node_height_mm)
            worst = min(worst, math.hypot(lateral, vertical)
                        - geometry.beam_radius_mm - geometry.safety_margin_mm)
    return worst


def angular_targets(params: GaitParams, phi_deg: float) -> tuple[float, float, float]:
    """Δψ, Δq_swing, Δq_support；旋转关系只取决于角度与相位增益。

    同一横梁参考系 Δβ=-φ，安装符号在转换为轴坐标时再乘。
    不允许用终点三重对称来替换中途避障路径。
    """
    return (params.phase_gain * phi_deg,
            (params.phase_gain + 1.0) * phi_deg, phi_deg)


def plan_swing_trajectory(
    params: GaitParams,
    *,
    side: str,
    extra_hexagons: Iterable[HexPad] = (),
    swing_joint_delta_deg: float | None = None,
    route: tuple[str, str, str, float] | None = None,
) -> DryRunReport:
    """干跑：生成整条摆动轨迹并做连续碰撞校验。

    ``side`` 是 "left"（A→C 绕 B）或 "right"（B→A 绕 C）。
    legacy参数 ``swing_joint_delta_deg`` 仅接受与当前角度模型一致的值。
    不能传入终点等效解绕增量来改变中途自转。route可用于后续换位。
    """

    params = params.validated()
    geometry = params.geometry
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    spin_deg, joint_delta, _support_delta = angular_targets(params, SWING_ARC_DEG)
    if swing_joint_delta_deg is not None and not math.isclose(swing_joint_delta_deg, joint_delta):
        raise ValueError("不能用终点等效解绕增量替换同步避障角度轨迹")
    _start, _target, pivot_name, start_bearing = (
        route if route is not None else (LEFT_SWING if side == "left" else RIGHT_SWING)
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
                   + spin_deg * shaping)
        beta_deg = start_bearing - (180.0 if side == "right" else 0.0) - phi_deg
        center = swing_center(pivot, geometry.d_mm, phi_deg, start_bearing)
        for label, node in high_nodes:
            if (label not in hub_passovers
                    and node_under_hub(node, center, geometry)):
                hub_passovers.append(label)
        margin = clearance_margin_mm(center, psi_deg, geometry,
                                     list(hexagons.values()), lift_mm=params.lift_mm,
                                     pivot=pivot.center)
        samples.append(
            SwingSample(s=s, phi_deg=phi_deg, psi_deg=psi_deg,
                        beta_deg=beta_deg, center=center, margin_mm=margin,
                        swing_q_delta_deg=joint_delta * shaping,
                        support_q_delta_deg=phi_deg)
        )
    # Lipschitz 下界：每对采样之间，任一臂端移动不超过
    # (d + k*R)*Δφ。最近距离是 1-Lipschitz；扣除半区间运动界，
    # 防止采样点都通过但中间穿过节点。稀采样只会更保守，不会误放行。
    worst = min(samples, key=lambda item: float(item.margin_mm))
    interval_bound = min(
        min(float(a.margin_mm), float(b.margin_mm))
        - (geometry.d_mm + params.phase_gain * geometry.arm_length_mm)
        * math.radians(b.phi_deg - a.phi_deg) / 2.0
        for a, b in zip(samples, samples[1:]))
    minimum = min(float(worst.margin_mm), interval_bound)
    feasible = minimum > 0.0
    if feasible:
        message = (f"模型可行：连续间隙保守下界 {minimum:.2f} mm "
                   f"(φ={worst.phi_deg:.1f}°, ψ={worst.psi_deg:.1f}°)")
    else:
        message = (f"不可行/未证实：连续间隙下界 {minimum:.2f} mm ≤ 0 "
                   f"(φ={worst.phi_deg:.1f}°, ψ={worst.psi_deg:.1f}°)；"
                   "请实测并修正高度/包络/相位增益；必要时提高校验密度")
    if hub_passovers:
        message += ("；" + "、".join(hub_passovers)
                    + " 存在壳体投影重叠，已计入垂向间隙校验")
    return DryRunReport(
        feasible=feasible,
        min_margin_mm=minimum,
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
    """阶段及运动组；synchronized组须原子下发、共用五次进度，无MOVE降级。"""

    stage_id: str
    title: str
    confirm_text: str  # 人工确认清单（接触/姿态检查）
    move_groups: tuple[tuple[RoleMove, ...], ...] = ()
    synchronized: bool = False
    duration_s: float = 0.0

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

    # S3 相位调整（目标 ψ≡30° (mod 120°) 基准；在基准位则跳过）
    if swing_psi_start_deg is not None:
        phase_delta = swing_psi_start_deg - LOW_NODE_PHASE_DEG
        # 三足 120° 对称：归一化到 (-60, 60]，按等效周期走最短转向
        phase_delta = ((phase_delta + SWING_SPIN_DEG / 2) % SWING_SPIN_DEG
                       - SWING_SPIN_DEG / 2)
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

    # S4 整条五次轨迹一次原子下发。固件共享时基/主进度，不再用两条
    # 独立梯形 MOVE 近似同步，也不把 Δq=180 替换成终点等效的 60。
    _spin, swing_delta_total, support_delta_total = angular_targets(params, SWING_ARC_DEG)
    duration_s = 1.875 * SWING_ARC_DEG / params.swing_speed_deg_s
    segment_groups = [(RoleMove(swing_role, swing_sign * swing_delta_total,
                               (params.phase_gain + 1) * params.swing_speed_deg_s),
                       RoleMove(support_role, support_sign * support_delta_total,
                                params.swing_speed_deg_s))]
    stages.append(
        GaitStage(
            stage_id="S4",
            title="公转 + 同步自转",
            confirm_text=(
                "同一五次进度同步运动；观察爪臂始终从"
                "高点间隙中扫过。任何异常立即点【中止】。\n"
                f"φ=60°，Δψ={params.phase_gain * 60:g}°，"
                f"Δq摆={swing_delta_total:g}°，Δq支=60°，"
                f"计划时长 {duration_s:.2f}s；禁止轨迹内等效解绕。"
            ),
            move_groups=tuple(segment_groups),
            synchronized=True,
            duration_s=duration_s,
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
    # 修正角 = 目标 ψ − 当前 ψ；三足 120° 对称，按等效周期归一到 (-60,60]
    return ((LOW_NODE_PHASE_DEG - psi + SWING_SPIN_DEG / 2) % SWING_SPIN_DEG
            - SWING_SPIN_DEG / 2)
