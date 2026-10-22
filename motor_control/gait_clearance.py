"""Shared planar whole-leg capsule metric, independent of Tk and motor IO.

gap_mm is physical surface separation (negative means projected overlap).
margin_mm additionally deducts the configured safety margin delta. Sampling
a pose does not certify the continuous swept path between poses.
"""
import math
from .gait_coordinates import direction


def leg_gaps(center, psi_deg, geometry, rods):
    """Three legs; rods are (label, x_mm, y_mm). Include the complete 0..R arm."""
    result = []
    for leg in range(3):
        dx, dy = direction(psi_deg + 120 * leg)
        best = None
        for label, x, y in rods:
            along = max(0., min(geometry.arm_length_mm,
                                (x-center[0])*dx + (y-center[1])*dy))
            point = (center[0]+along*dx, center[1]+along*dy)
            gap = math.hypot(x-point[0], y-point[1]) - geometry.arm_radius_mm - geometry.node_radius_mm
            if best is None or gap < best["gap_mm"]:
                best = dict(gap_mm=gap, margin_mm=gap-geometry.safety_margin_mm,
                            rod=label, leg=leg+1, closest_point=point)
        if best is None:
            raise ValueError("整腿间隙计算缺少高杆")
        result.append(best)
    return result
