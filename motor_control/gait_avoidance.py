"""Angle-only two-mode leg/high-rod paths (no IO, no hardware assumptions).

The executable path is the polyline, NOT the rational generating curve. Each
edge is sent as one rest-to-rest, common-clock SYNC; preview and certification
use these same edges. All three radial legs, including their roots, are checked.
"""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
import re

LEGACY = "legacy_gain"
TWO_MODE = "two_mode_v1"
LOW = "low_junction"
HIGH = "high_junction"
# 2026-09-24 用户实机连续行走后确认：转动逻辑不绑定"左顺移/左逆移"
# 这类按钮名（那只说动哪条腿、往哪边转）。模态每一步按当前站位单独
# 判定：连续前进（左顺移/右逆移交替选一）时横梁依次从低节点扇区和
# 高节点扇区扫过，模态逐步交替；原路返回则与来时相同。名称直接写
# 横梁几何，操作员看一眼就知道本步会不会跨红杆。
MODE_NAMES = {LOW: "低节点侧·反向比例自转（本步横梁不跨红杆）",
              HIGH: "高节点侧·同向变比例自转（本步横梁跨红杆，S4 分段慢速）"}
SEGMENTS = 60
# 2026-09-23/24 应用户观察"爪-红杆最大富余不在公转30°处"做的
# (自转中心c, 斜率s) 全网格扫描：30° 是唯一安全缝隙的几何中心，
# 中心偏移 ±1.2° 整腿净间隙即转负；瓶颈是起飞前爪端贴起点座高杆
# （φ≈28°），推后/提前均单调变差，解为对称陡化。指数 18 是整腿口径
# 的整数最优（+1.81mm / 爪端 9.98mm）；调研细网格的族内微优在
# 19.6（采样口径 +2.08mm，未部署）。指数 24（爪端优先：整腿 +1.47 /
# 爪端 10.88，见 gitea 分支 gait-exponent-24 = e7a4837）曾短暂部署，
# 2026-09-24 应用户决定回退 18：放行余量优先于爪端目测距离，实测
# 半径/δ 替换后若不可行也应回收指数而非调小半径硬放行。峰段
# |Δψ/Δφ| 折线比约 32×，执行端按 3× 公转限幅自动拉长该段时长。
HIGH_EXPONENT = 18


def lattice_coordinates(name):
    base = {"A": (-1, 0), "B": (0, 0), "C": (-1, 1)}
    if name in base:
        return base[name]
    match = re.fullmatch(r"邻座\((-?\d+),(-?\d+)\)", name)
    if match is None:
        raise ValueError("未知支座坐标")
    return tuple(map(int, match.groups()))


@dataclass(frozen=True)
class AvoidancePath:
    modality: str
    arc_deg: float
    # (normalized orbit progress, signed orbit angle, world spin delta)
    knots: tuple[tuple[float, float, float], ...]

    def angles(self, u: float) -> tuple[float, float, float]:
        """Return world spin, swing joint, support joint on the executed edge."""
        u = max(0.0, min(1.0, u))
        i = min(len(self.knots)-2, max(0, bisect_right(
            [k[0] for k in self.knots], u)-1))
        a, b = self.knots[i:i+2]
        t = (u-a[0])/(b[0]-a[0])
        phi = a[1] + t*(b[1]-a[1])
        spin = a[2] + t*(b[2]-a[2])
        return spin, spin+phi, phi


def avoidance_path(bearing_deg: float, arc_deg: float) -> AvoidancePath:
    """Classify the CURRENT route, not the button/leg name.

    Mid-sector bearings 30 mod120 face a shared LOW node; 90 mod120 face
    HIGH. +/-0.5deg permits the existing pulse-quantized stance tolerance.
    Only one adjacent-cell step (+/-60deg) is supported and validated.

    No sensor is needed: the modality follows from the tracked stance and
    beam angle alone (walking forward flips LOW/HIGH every step; stepping
    back along the arrival path keeps the previous modality).
    """
    if not math.isfinite(bearing_deg) or not math.isfinite(arc_deg) or not math.isclose(abs(arc_deg), 60.0):
        raise ValueError("两模态避杆只支持相邻支座 ±60° 换位")
    midpoint = bearing_deg - arc_deg/2
    low_error = abs((midpoint-30+60) % 120-60)
    high_error = abs((midpoint-90+60) % 120-60)
    if min(low_error, high_error) > 0.5:
        raise ValueError("当前支点方位未对准高低杆晶格；请重建基准")
    modality = LOW if low_error <= high_error else HIGH
    n = 1 if modality == LOW else SEGMENTS
    sign = 1 if arc_deg > 0 else -1
    knots = []
    for i in range(n+1):
        u = i/n
        f = u if modality == LOW else (
            u**HIGH_EXPONENT/(u**HIGH_EXPONENT+(1-u)**HIGH_EXPONENT))
        spin = sign*120*f*(1 if modality == LOW else -1)
        knots.append((u, arc_deg*u, spin))
    return AvoidancePath(modality, arc_deg, tuple(knots))


def leg_clearance(center, psi_deg, geometry, pads):
    """Planar capsule clearance; never waive a swept rod using lift height.

    Returns (net margin, rod label, leg index). Rigid radial legs span 0..R;
    this is conservative for the existing hub-to-tip model. Does not model
    a different robot having only vertical posts at its three tips.
    """
    from .gait_planner import point_segment_distance
    worst = (math.inf, "", 0)
    for arm in range(3):
        angle = math.radians(psi_deg+120*arm)
        tip = (center[0]+geometry.arm_length_mm*math.cos(angle),
               center[1]+geometry.arm_length_mm*math.sin(angle))
        for pad in pads:
            for j, node in enumerate(pad.high_nodes(geometry.arm_length_mm)):
                margin = (point_segment_distance(node, center, tip)
                          - geometry.arm_radius_mm-geometry.node_radius_mm
                          - geometry.safety_margin_mm)
                if margin < worst[0]:
                    worst = (margin, f"{pad.name}·{90+120*j}°高杆", arm+1)
    return worst
