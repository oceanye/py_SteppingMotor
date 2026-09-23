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
MODE_NAMES = {LOW: "低节点侧·反向比例自转", HIGH: "高节点侧·同向变比例自转"}
SEGMENTS = 60
# 2026-09-23 应用户观察"爪-红杆最大富余不在公转30°处"做的形状扫描：
# 把自转过渡集中在 φ=−30°±3° 的安全缝隙里快速完成。指数 6（缓变，
# 峰速 0.75×公转）在缝隙两肩各穿一个浅坑；对称陡化到 18（峰速
# 2.25×公转，仍在执行端 3× 限幅内）后整腿口径净间隙由 −3.99mm
# 转正为 +1.8mm。不对称移动加速点（提前或推后）经网格扫描均单调
# 变差：瓶颈是 (φ,ψ) 平面里缝隙两侧的固定杆位，不是速度分配。
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
