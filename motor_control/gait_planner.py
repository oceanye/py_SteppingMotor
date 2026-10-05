"""三足轮换步态的纯规划数学（干跑 + 分阶段执行的离线核心）.

依据 ``docs/handoff control method.txt``（A→C 绕 B 公转方案）实现：

- 五次平滑曲线 ``S(s)=10s³-15s⁴+6s⁵``；
- 左三足中心 ``O1(φ) = B + d·dir(180°-φ)``，φ∈[0°,60°]；
- 横梁角 ``β = β_start - φ``，关节角 ``q = ψ - β + c``；
- legacy 自转 ``ψ = ψ_start + k·φ``（默认 k=-2），保留旧配置；
- two_mode_v1 按当前晶格扇区自动选择低节点侧比例／高节点侧变比例自转；
  ``gait_avoidance`` 的同一角度折线用于预览、连续间隙下界与实际分段 SYNC；
- 新模式检查完整径向腿扫掠邻接高杆，不用抬升高度豁免平面冲突。
  横梁、壳体和实测垂向包络另作结构诊断，不与腿避杆证明混淆。

本模块不 import Tk、不碰串口：所有几何/轨迹/碰撞/阶段计划都可以离线
单元测试。执行与标定向导在 ``desktop_app`` 里基于这里的纯函数搭建。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Iterable, Mapping, Sequence
from .gait_avoidance import LEGACY, TWO_MODE, MODE_NAMES, avoidance_path, leg_clearance, lattice_coordinates
from .gait_map import start_pair_reference

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

    d_mm: float = 220.0             # 旧策略中心距；两模态仅由投影边长派生，不使用此值
    arm_length_mm: float = 40.0     # 正六边形投影边长 a = 中心到顶点 R = 腿投影长度
    hub_radius_mm: float = 12.0     # 三足中心壳体等效半径
    arm_radius_mm: float = 4.0      # 爪臂等效半径（胶囊粗细）
    node_radius_mm: float = 5.0     # 高节点等效半径
    safety_margin_mm: float = 2.0   # δ：要求的最小安全间隙
    high_node_height_mm: float = 12.0  # 相对低节点，必须实测
    body_drop_mm: float = 0.0       # 壳体/电机/轴承最低点低于爪臂中心线的量
    beam_height_mm: float = 30.0    # 横梁中心线相对低节点高度，必须实测
    beam_radius_mm: float = 4.0    # 横梁/连接件保守胶囊包络
    surrounding_pads: bool = True  # 检查紧邻的六边形，不只 A/B/C

    @property
    def touching_center_distance_mm(self) -> float:
        """Adjacent touching hexagons: projected center spacing d = sqrt(3)*a."""
        return math.sqrt(3) * self.arm_length_mm

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
        if self.d_mm + 1e-8 < self.touching_center_distance_mm:
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


# 初始摆放 → (左足/右足支座, 横梁基准角)。红杆在横梁左/右侧的两种
# 镜像摆法；程序账本原点由它派生（无传感器，全靠人工按实际摆放选择）。
# 2026-09-28 实机校正左右：判据是"紧挨横梁的红杆"在从左足看向右足的
# 哪只手边。左足A·右足B（原有默认，β₀180）两支座共享棱上北端是低节点、
# 南端是红杆，从左足(西)看向右足(东)红杆在右手；镜像摆法左足B·右足A
# （β₀0°）红杆在左手。初版误用"第三支座C在哪侧"判左右（恰相反），已对调。
INITIAL_PLACEMENTS: dict[str, tuple[tuple[str, str], float]] = {
    "red_left": (("B", "A"), 0.0),
    "red_right": (("A", "B"), 180.0),
}

# eb308f4（对调前）按旧表写入摆放键：red_left↔180° / red_right↔0°。
# c56f588 对调后同键要求相反的β₀；parse_gait_params 据此识别旧配对并
# 自动换键（β₀、零位等原样保留——换的只是标签，不是几何）。
LEGACY_PLACEMENT_PAIRING: dict[str, float] = {
    "red_left": 180.0,
    "red_right": 0.0,
}


@dataclass(frozen=True)
class GaitParams:
    """统一预览与实际分阶段执行参数；旧v1文档须重新建立零位签名。"""

    geometry: GaitGeometry = field(default_factory=GaitGeometry)
    # 执行节拍
    swing_segments: int = 12          # 展示密度；不会切分实际同步指令
    lift_mm: float = 70.0             # 起步两直线轴同时抬升量（轴行程单位）
                                       # 2026-09-21 按用户实测 50 → 70：
                                       # 单腿硬顶 ~30mm 即卡顿（摩擦），两轴
                                       # 分摊后 70 才能保证移位腿收起后
                                       # 爪与节点不打架
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
    # Δψ/φ；不受中心距/升降行程影响。2026-09-21 应用户要求默认 2 → -2：
    # 公转 +60°、摆动电机 (k+1)φ = -60°（与公转反向），世界自转 -120°
    # （三重对称下与 +120° 终点等效，但路径短、方向相反，线缆缠绕少）。
    phase_gain: float = -2.0
    # Preserve legacy programmatic callers/configurations. New desktop sessions
    # explicitly select TWO_MODE; existing saved gains are never reinterpreted.
    trajectory_mode: str = LEGACY
    # 2026-09-24 初始摆放两种镜像：红杆在横梁哪一侧决定同一按钮走出
    # 的模态序列（2026-09-28 实机校正左右标注，初版判据反了已对调）。
    # "red_left" = 左足B、右足A、基准 0°（俯视、从左足看向右足，红杆
    # 在左手边）；"red_right" = 左足A、右足B、基准 180°（红杆在右手
    # 边）——原有唯一摆法，故仍是默认。切换即更换坐标基准：账本重置、
    # 零位作废，必须按新摆放重新记零标定。
    initial_placement: str = "red_right"
    initial_pad_pair: tuple[str, str] | None = None  # None: legacy A/B placement
    beam_reference_deg: float = 180.0  # 两个 Mr 同时记零时横梁世界角
    calibration_confirmed: bool = False  # 实测几何、方向、PPR、反力闭合已确认
    calibration_fingerprint: str | None = None
    mr1_zero_signature: str | None = None
    mr2_zero_signature: str | None = None

    SCHEMA = "gait-params-v1"

    def validated(self) -> "GaitParams":
        if self.trajectory_mode not in (LEGACY, TWO_MODE):
            raise ValueError("未知步态轨迹模式")
        # Validate the SAME spacing used by two-mode planning. The saved d is
        # legacy-only and must not reject a measured projected edge (e.g. 190
        # with an old d=220). Preserve the document and all other checks,
        # including surrounding_pads type; do not silently migrate legacy.
        geometry = (replace(self.geometry, d_mm=self.geometry.touching_center_distance_mm)
                    if self.trajectory_mode == TWO_MODE else self.geometry)
        geometry.validated()
        if self.initial_placement not in INITIAL_PLACEMENTS:
            raise ValueError("initial_placement 必须是 red_left 或 red_right")
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
        # 2026-09-21 放开负增益：k=-2 表示摆动电机与公转反向转 60°。
        # 终点仍须落在三重对称低节点（k×60° 必须是 120° 的整数倍）。
        if (not math.isfinite(self.phase_gain)
                or not -10.0 <= self.phase_gain <= 10.0
                or self.phase_gain == 0
                or abs(self.phase_gain * SWING_ARC_DEG % 120.0) > 1e-8):
            raise ValueError("phase_gain 必须为 ±2/±4/±6/±8/±10；终点须对准三重对称低节点")
        if not math.isfinite(self.beam_reference_deg):
            raise ValueError("beam_reference_deg 必须是有限数值")
        expected_beam = self.initial_beam_deg
        if self.initial_pad_pair is not None:
            if not isinstance(self.initial_pad_pair, tuple):
                raise ValueError("initial_pad_pair 必须是两个支座组成的 tuple")
            _, placement = start_pair_reference(self.initial_pad_pair)
            if self.initial_placement != placement:
                raise ValueError("初始摆放与选定左右足支座不一致，请重新应用起步位置")
        if not math.isclose(self.beam_reference_deg, expected_beam):
            raise ValueError("beam_reference_deg 必须与起步支座及摆放一致；"
                             "请用【应用起步位置】或【初始摆放】设置")
        if not isinstance(self.calibration_confirmed, bool):
            raise ValueError("calibration_confirmed 必须是布尔值")
        if self.calibration_fingerprint is not None and not isinstance(self.calibration_fingerprint, str):
            raise ValueError("calibration_fingerprint 必须是字符串或 null")
        for value in (self.mr1_zero_signature, self.mr2_zero_signature):
            if value is not None and not isinstance(value, str):
                raise ValueError("旋转零位签名必须是字符串或 null")
        return self

    # 传统摆动侧速度（|k+1|×支撑速度）。S4 解绕后按增量比另算段速；
    # 此值现用于 S3 相位调整等单轴整段运动。k=-1 已被 validated 拒绝，
    # 结果恒为正；方向由 RoleMove.delta 的符号承载。
    @property
    def swing_side_speed_deg_s(self) -> float:
        return abs(self.phase_gain + 1.0) * self.swing_speed_deg_s

    # 初始摆放派生的账本原点（启动/重建基准时写入 _gait_supports/_gait_beta_deg）。
    @property
    def initial_supports(self) -> tuple[str, str]:
        return (self.initial_pad_pair if self.initial_pad_pair is not None
                else INITIAL_PLACEMENTS[self.initial_placement][0])

    @property
    def initial_beam_deg(self) -> float:
        if self.initial_pad_pair is not None:
            return start_pair_reference(self.initial_pad_pair)[0]
        return INITIAL_PLACEMENTS[self.initial_placement][1]

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
            "trajectory_mode": self.trajectory_mode,
            "initial_placement": self.initial_placement,
            "initial_pad_pair": (list(self.initial_pad_pair)
                                 if self.initial_pad_pair is not None else None),
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
    beam_reference_deg = _number("beam_reference_deg", 180.0)
    initial_placement = value.get("initial_placement")
    pair = value.get("initial_pad_pair")
    if pair is not None:
        _, derived_placement = start_pair_reference(pair)
        pair = tuple(pair)
        if initial_placement is None:
            initial_placement = derived_placement
    elif initial_placement is None:
        # 旧文档无摆放键：按已保存的β₀查配对摆放，避免缺键默认与旧β₀
        # 矛盾导致加载失败。
        matched = [name for name, (_sup, beam) in INITIAL_PLACEMENTS.items()
                   if math.isclose(beam_reference_deg, beam)]
        initial_placement = matched[0] if matched else "red_right"
    elif initial_placement in LEGACY_PLACEMENT_PAIRING:
        # 2026-09-28 左右对调校正的旧文档迁移：摆放键与β₀按 eb308f4 旧表
        # 配对时自动换键（β₀不动），零位/标定原样保留，免于重新记零；
        # 无法识别的组合仍交 validated 拒绝。
        if (math.isclose(beam_reference_deg,
                         LEGACY_PLACEMENT_PAIRING[initial_placement])
                and not math.isclose(
                    beam_reference_deg,
                    INITIAL_PLACEMENTS[initial_placement][1])):
            initial_placement = ("red_right" if initial_placement == "red_left"
                                 else "red_left")
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
        phase_gain=_number("phase_gain", GaitParams.phase_gain),
        trajectory_mode=value.get("trajectory_mode", LEGACY),
        initial_placement=initial_placement,
        initial_pad_pair=pair,
        beam_reference_deg=beam_reference_deg,
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
    # 本次轨迹实际使用的 (start, target, pivot, start_bearing)。
    # 碰撞校验仍覆盖 hexagons 全部支座；视图只画 route 涉及的支座。
    route: tuple[str, str, str, float] | None = None
    # "left"/"right"：哪一侧腿是摆动足（视图按左右足固定配色）。
    side: str = ""
    modality: str = ""
    worst_rod: str = ""
    worst_leg: int = 0
    structure_margin_mm: float | None = None

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
            "modality": self.modality,
            "worst_rod": self.worst_rod,
            "worst_leg": self.worst_leg,
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


def effective_geometry(params: GaitParams) -> GaitGeometry:
    """Two-mode paths are certified on the requested touching-hexagon lattice.

    Projected hexagon edge = leg reach = node ring radius is the only scale.
    Old d is retained for legacy, but never read by two-mode validation or
    planning. Projected leg/rod widths and the safety gap remain independent.
    """
    if params.trajectory_mode == TWO_MODE:
        return replace(params.geometry, d_mm=params.geometry.touching_center_distance_mm,
                       surrounding_pads=True)
    return params.geometry


def gait_pads(params: GaitParams, pivot_name="B") -> dict[str, HexPad]:
    geometry = effective_geometry(params)
    pads = default_hex_pads(geometry)
    if params.trajectory_mode == TWO_MODE or params.initial_pad_pair is not None:
        # Recenter the obstacle neighborhood after EVERY step; never lose an
        # outer rod just because the support left the original A/B/C patch.
        pi, pj = lattice_coordinates(pivot_name)
        d = geometry.d_mm
        for i in range(pi-2, pi+3):
            for j in range(pj-2, pj+3):
                center = (d*(i+j/2), d*math.sqrt(3)*j/2)
                if not any(math.dist(center, p.center) < 1e-8 for p in pads.values()):
                    name = f"邻座({i},{j})"
                    pads[name] = HexPad(name, center)
    return pads


def plan_swing_trajectory(
    params: GaitParams,
    *,
    side: str,
    extra_hexagons: Iterable[HexPad] = (),
    swing_joint_delta_deg: float | None = None,
    route: tuple[str, str, str, float] | None = None,
    arc_deg: float | None = None,
) -> DryRunReport:
    """干跑：生成整条摆动轨迹并做连续碰撞校验。

    ``side`` 是 "left"（A→C 绕 B）或 "right"（B→A 绕 C）。
    legacy参数 ``swing_joint_delta_deg`` 仅接受与当前角度模型一致的值。
    不能传入终点等效解绕增量来改变中途自转。route可用于后续换位。
    ``arc_deg`` 是本次换位方向：+SWING_ARC_DEG（默认顺向）或负值逆向。
    预览与实机执行（begin_run）传同一方向，四种换位方式所见即所得。
    """

    params = params.validated()
    geometry = effective_geometry(params)
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    if arc_deg is None:
        arc_deg = SWING_ARC_DEG
    if not math.isfinite(arc_deg) or not -360.0 < arc_deg < 360.0 or arc_deg == 0:
        raise ValueError("arc_deg 必须是 ±360°内的非零有限角度")
    spin_deg, joint_delta, _support_delta = angular_targets(params, arc_deg)
    if swing_joint_delta_deg is not None and not math.isclose(swing_joint_delta_deg, joint_delta):
        raise ValueError("不能用终点等效解绕增量替换同步避障角度轨迹")
    _start, _target, pivot_name, start_bearing = (
        route if route is not None else (LEFT_SWING if side == "left" else RIGHT_SWING)
    )
    path = (avoidance_path(start_bearing, arc_deg)
            if params.trajectory_mode == TWO_MODE else None)
    hexagons = gait_pads(params, pivot_name)
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
    # Include every execution knot. Linear angle samples give an explicit
    # interval motion bound even when the world-spin/phi ratio is variable.
    count = max(600, int(params.feasibility_samples)) if path else int(params.feasibility_samples)
    grid = {i/count for i in range(count+1)}
    if path:
        grid.update(k[0] for k in path.knots)
    worst_rod, worst_leg, worst_leg_margin = "", 0, math.inf
    structural = math.inf
    for s in sorted(grid):
        shaping = s if path else smoothstep5(s)
        phi_deg = arc_deg * shaping
        psi_deg = (LOW_NODE_PHASE_DEG
                   + spin_deg * shaping)
        sample_joint_delta = joint_delta * shaping
        if path:
            spin, sample_joint_delta, phi_deg = path.angles(shaping)
            psi_deg = LOW_NODE_PHASE_DEG + spin
        beta_deg = start_bearing - (180.0 if side == "right" else 0.0) - phi_deg
        center = swing_center(pivot, geometry.d_mm, phi_deg, start_bearing)
        for label, node in high_nodes:
            if (label not in hub_passovers
                    and node_under_hub(node, center, geometry)):
                hub_passovers.append(label)
        margin = clearance_margin_mm(center, psi_deg, geometry,
                                     list(hexagons.values()), lift_mm=params.lift_mm,
                                     pivot=pivot.center)
        structural = min(structural, margin)
        if path:
            margin, rod, leg = leg_clearance(center, psi_deg, geometry, list(hexagons.values()))
            if margin < worst_leg_margin:
                worst_leg_margin, worst_rod, worst_leg = margin, rod, leg
        samples.append(
            SwingSample(s=s, phi_deg=phi_deg, psi_deg=psi_deg,
                        beta_deg=beta_deg, center=center, margin_mm=margin,
                        swing_q_delta_deg=sample_joint_delta,
                        support_q_delta_deg=phi_deg)
        )
    # Lipschitz 下界：每对采样之间，任一臂端移动不超过
    # d*|Δφ| + R*|Δψ|。最近距离是 1-Lipschitz；扣除半区间运动界，
    # 防止采样点都通过但中间穿过节点。稀采样只会更保守，不会误放行。
    # 逆向弧（arc_deg<0）时 Δφ 为负，取绝对值保持同一保守方向。
    worst = min(samples, key=lambda item: float(item.margin_mm))
    interval_bound = min(
        min(float(a.margin_mm), float(b.margin_mm))
        - (geometry.d_mm * math.radians(abs(b.phi_deg-a.phi_deg))
           + geometry.arm_length_mm * math.radians(abs(b.psi_deg-a.psi_deg))) / 2.0
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
    if path:
        message = (f"{MODE_NAMES[path.modality]}；腿—高杆平面净间隙保守下界 "
                   f"{minimum:.3f}mm（已扣腿/杆半径及δ）；"
                   f"最紧腿{worst_leg} / {worst_rod}，|φ|={abs(worst.phi_deg):.2f}°。"
                   + ("模型通过，实机还需覆盖脉冲误差。" if feasible else
                      "未通过：禁止新模式实机执行；抬高不替代平面避杆。")
                   + " 紧贴晶格参考；非实测碰撞传感器。")
    return DryRunReport(
        feasible=feasible,
        min_margin_mm=minimum,
        min_margin_sample=worst,
        samples=tuple(samples),
        hexagons=tuple(hexagons.values()),
        message=message,
        hub_passover_nodes=tuple(hub_passovers),
        route=(_start, _target, pivot_name, start_bearing),
        side=side,
        modality="" if path is None else path.modality,
        worst_rod=worst_rod, worst_leg=worst_leg,
        structure_margin_mm=structural if path else None,
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
    group_durations: tuple[float, ...] = ()
    # Ideal absolute offsets from S4 start, rounded cumulatively at the host.
    sync_endpoints: tuple[tuple[float, float], ...] = ()

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
    arc_deg: float | None = None,
    route: tuple[str, str, str, float] | None = None,
) -> list[GaitStage]:
    """生成一次完整摆动的 S0..S7 阶段计划。

    ``swing_psi_start_deg`` 是摆动三足当前绝对姿态（由执行器按标定零位
    换算）；等于 30°（低节点基准）时 S3 为空并自动跳过。None 表示
    只生成计划不做相位修正（用于纯展示）。
    ``arc_deg`` 是本次换位方向：+SWING_ARC_DEG 顺向（默认）、负值逆向。
    2026-09-22 起实机执行与预览传同一方向，四种换位方式所见即所得。
    """

    params = params.validated()
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")
    if arc_deg is None:
        arc_deg = SWING_ARC_DEG
    if not math.isfinite(arc_deg) or not -360.0 < arc_deg < 360.0 or arc_deg == 0:
        raise ValueError("arc_deg 必须是 ±360°内的非零有限角度")
    swing_role, support_role = (
        ("Mr1", "Mr2") if side == "left" else ("Mr2", "Mr1")
    )
    lift_role = "Mup1" if side == "left" else "Mup2"
    swing_sign = params.mr1_sign if swing_role == "Mr1" else params.mr2_sign
    support_sign = (params.mr2_sign if support_role == "Mr2"
                    else params.mr1_sign)
    lift_sign = (params.mup1_lift_sign if lift_role == "Mup1"
                 else params.mup2_lift_sign)

    side_text = ("左三足换位（左侧腿摆动，右侧腿支撑）" if side == "left"
                 else "右三足换位（右侧腿摆动，左侧腿支撑）")

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

    # S2 顶部抬升（两直线轴同时撑起）。2026-09-21 按用户澄清的机构语义：
    # 轴"向上" = 该腿撑起顶部，单腿硬顶实测 ~30mm 即卡顿；两轴同时抬升
    # 分摊负载，并为移位腿收起留出爪-节点垂向间隙。
    stages.append(
        GaitStage(
            stage_id="S2",
            title="两轴同时抬升顶部",
            confirm_text=(
                f"确认顶部整体抬升 {params.lift_mm:g}mm：两直线轴同步向上、"
                "速度一致，两爪仍在原低节点；机构无卡顿、横梁无倾斜。"
            ),
            move_groups=(
                (RoleMove("Mup1", params.mup1_lift_sign * params.lift_mm,
                          params.lift_speed_mm_s),
                 RoleMove("Mup2", params.mup2_lift_sign * params.lift_mm,
                          params.lift_speed_mm_s)),
            ),
        )
    )
    # S2B 收起移位腿：GUI"向下"= 腿收起。顶部由站立腿撑住，移位腿收回后
    # 爪离地间隙 = 抬升量（此时两直轴标高差 = lift_mm）。
    stages.append(
        GaitStage(
            stage_id="S2B",
            title="收起移位腿",
            confirm_text=(
                "确认移位腿已收起：三爪全部离地、与高节点有垂向间隙；"
                "站立腿仍稳定撑住顶部，无侧倾。"
            ),
            move_groups=(
                (RoleMove(lift_role, -lift_sign * params.lift_mm,
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

    # S4 每条角度折线段一次原子下发。固件共享时基/主进度，不用两条
    # 独立梯形 MOVE 近似同步，也不把 Δq=180 替换成终点等效的 60。
    _spin, swing_delta_total, support_delta_total = angular_targets(params, arc_deg)
    duration_s = 1.875 * abs(arc_deg) / params.swing_speed_deg_s
    segment_groups = [(RoleMove(swing_role, swing_sign * swing_delta_total,
                               abs(params.phase_gain + 1) * params.swing_speed_deg_s),
                       RoleMove(support_role, support_sign * support_delta_total,
                                params.swing_speed_deg_s))]
    group_durations, endpoints = (), ()
    modality_text = ""
    if params.trajectory_mode == TWO_MODE:
        path = avoidance_path((route or (LEFT_SWING if side == "left" else RIGHT_SWING))[3], arc_deg)
        modality_text = MODE_NAMES[path.modality] + "；分段原子SYNC，段间静止换向。\n"
        segment_groups, durations, targets = [], [], []
        for a, b in zip(path.knots, path.knots[1:]):
            qa, qb = a[1]+a[2], b[1]+b[2]
            ds, dp = qb-qa, b[1]-a[1]
            # Cap swing joint peak at 3x the configured orbit peak. The
            # high-mode rational curve is not assumed to have constant gain.
            duration = max(0.05, 1.875*abs(dp)/params.swing_speed_deg_s,
                           1.875*abs(ds)/(3*params.swing_speed_deg_s))
            durations.append(duration)
            targets.append((swing_sign*qb, support_sign*b[1]))
            segment_groups.append((RoleMove(swing_role, swing_sign*ds,
                                             max(0.001, 1.875*abs(ds)/duration)),
                                   RoleMove(support_role, support_sign*dp,
                                            max(0.001, 1.875*abs(dp)/duration))))
        group_durations, endpoints = tuple(durations), tuple(targets)
        duration_s = sum(durations)
        _spin, swing_delta_total, support_delta_total = path.angles(1)
    stages.append(
        GaitStage(
            stage_id="S4",
            title="公转 + 同步自转",
            confirm_text=(
                modality_text +
                "同一五次进度同步运动；观察爪臂始终从"
                "高点间隙中扫过。任何异常立即点【中止】。\n"
                f"φ={arc_deg:g}°，Δψ={_spin:g}°，"
                f"Δq摆={swing_delta_total:g}°，Δq支={support_delta_total:g}°，"
                f"计划时长 {duration_s:.2f}s；禁止轨迹内等效解绕。"
            ),
            move_groups=tuple(segment_groups),
            synchronized=True,
            duration_s=duration_s,
            group_durations=group_durations,
            sync_endpoints=endpoints,
        )
    )

    # S6 落脚：2026-09-24 应用户实机反馈重构——单靠站立腿下降时重心
    # 已偏向移位侧，单轴硬顶明显卡顿。改为两段：S5B 先把移位腿伸出
    # 踩实新支座（此时两腿同高、共同承载），S6 再两轴同步降回原标高。
    # 落地纠偏的触发点随之从"站立侧向下"移到 S5B 的"悬空侧向上"
    # （两条等价路径，desktop_app._land_release_tick 只评估单轴运动）。
    stages.extend(
        [
            GaitStage(
                stage_id="S5",
                title="接近目标位",
                confirm_text="目视检查移位腿爪臂相位与目标六边形低节点一一对应。",
            ),
            GaitStage(
                stage_id="S5B",
                title="移位腿先落脚",
                confirm_text=(
                    "移位腿低速伸出到新支座低节点踩实站好；顶部暂由站立腿"
                    "保持高度。标高差收敛到阈值时，落地纠偏自动释放旋转电机"
                    "并延迟重锁——属正常纠偏，不是故障。"
                ),
                move_groups=(
                    (RoleMove(lift_role, lift_sign * params.lift_mm,
                              params.settle_speed_mm_s),),
                ),
            ),
            GaitStage(
                stage_id="S6",
                title="顶部下降（两腿同时）",
                confirm_text=(
                    "两条腿已共同承载，两直轴同步低速降回原标高；确认无卡顿、"
                    "无侧倾、移位腿三个低节点保持接触有效后才允许进入锁定。"
                ),
                move_groups=(
                    (RoleMove("Mup1", -params.mup1_lift_sign * params.lift_mm,
                              params.settle_speed_mm_s),
                     RoleMove("Mup2", -params.mup2_lift_sign * params.lift_mm,
                              params.settle_speed_mm_s)),
                ),
            ),
            GaitStage(
                stage_id="S7",
                title="锁定完成",
                confirm_text=(
                    "确认移位腿三足踩实、两直轴回到抬升前标高（无标高差）、"
                    "姿态正确；本次摆动结束。"
                ),
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
