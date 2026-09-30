"""Fixed 5x5 walking map; same lattice/world axes as the gait planner.

Rows use an offset layout (not an axial parallelogram). No hardware IO.
The map limits selectable/landable pads, NOT the obstacle neighborhood.
"""
import math

from .gait_avoidance import lattice_coordinates

MAP_SIZE = 5
_BASE_NAMES = {(-1, 0): "A", (0, 0): "B", (-1, 1): "C"}


def pad_name(i, j):
    return _BASE_NAMES.get((i, j), f"邻座({i},{j})")


def map_pads():
    """25 centers in beam-length units, with B at the world origin."""
    return {pad_name(col - row // 2, row):
            (col - row // 2 + row / 2, math.sqrt(3) * row / 2)
            for row in range(-2, 3) for col in range(-2, 3)}


def map_label(name):
    i, j = lattice_coordinates(name)
    return f"{j + 3}行{i + j // 2 + 3}列" + (f"·{name}" if name in ("A", "B", "C") else "")


def in_map(name):
    try:
        i, j = lattice_coordinates(name)
    except (ValueError, TypeError):
        return False
    return -2 <= j <= 2 and -2 <= i + j // 2 <= 2


def start_pair_reference(pair):
    """Validate ordered (left, right) pads, return (beam angle, placement).

    beta points from right foot to left foot. Shared-edge high rods fall on
    the left for beta=0/120/240 and on the right for beta=60/180/300.
    """
    if (not isinstance(pair, (tuple, list)) or len(pair) != 2
            or any(not isinstance(p, str) or not in_map(p) for p in pair)):
        raise ValueError("起步位置必须是 5×5 地图内的两个六边形")
    if any(pad_name(*lattice_coordinates(p)) != p for p in pair):
        raise ValueError("起步支座请使用地图标准名称（原点为 B，其左邻为 A）")
    left, right = (lattice_coordinates(p) for p in pair)
    di, dj = left[0] - right[0], left[1] - right[1]
    if (di, dj) not in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, -1), (-1, 1)):
        raise ValueError("左右足必须选择两个不同且共边相邻的六边形")
    beta = float((round(math.degrees(math.atan2(math.sqrt(3)*dj/2, di+dj/2))/60) % 6)*60)
    return beta, "red_left" if beta % 120 == 0 else "red_right"
