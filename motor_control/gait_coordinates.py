"""One world convention for planning, feedback, drawing and action selection.

World x is right and y is up. Model angles increase CLOCKWISE (the 2026-10-22
mirrored lattice): dir(a)=(cos(a),-sin(a)); orbit phi changes beta by -phi.
View rotation is display-only. Motor installation signs are applied separately.
The UI '+' action means clockwise orbit, hence phi=-60, NOT numeric phi>0.
"""
import math


def direction(degrees):
    angle = math.radians(degrees)
    return math.cos(angle), -math.sin(angle)


def polar_point(center, radius, degrees):
    dx, dy = direction(degrees)
    return center[0] + radius * dx, center[1] + radius * dy


def bearing(vector):
    return math.degrees(math.atan2(-vector[1], vector[0]))


def action_key(side, arc):
    if side not in ("left", "right") or arc not in (-60, 60):
        raise ValueError("动作需要左右足及 ±60° 弧向")
    return ("L" if side == "left" else "R") + ("+" if arc < 0 else "-")


def action_values(key):
    if key not in ("L+", "L-", "R+", "R-"):
        raise ValueError("未知四动作选项")
    return ("left" if key[0] == "L" else "right", -60.0 if key[1] == "+" else 60.0)


def orbit_label(arc):
    return "顺时针" if arc < 0 else "逆时针"


def projection_warning(geometry):
    if min(geometry.arm_radius_mm, geometry.node_radius_mm) < 1.0 or geometry.safety_margin_mm < .5:
        return ("投影包络/间隙含亚毫米值：腿半宽 "
                f"{geometry.arm_radius_mm:g}mm、高杆半径 {geometry.node_radius_mm:g}mm、"
                f"δ={geometry.safety_margin_mm:g}mm；请核实是否临时调试值，模型通过不证明实物安全")
    return ""


def rod_side_for_pair(pair):
    """Actual shared-edge high rod side, looking from LEFT foot to RIGHT foot.

Keep legacy persisted placement keys/coordinates unchanged; those keys no
longer describe handedness after mirroring. UI labels must use this geometry.
"""
    from .gait_map import map_pads
    left, right = (map_pads()[name] for name in pair)
    vx, vy = right[0]-left[0], right[1]-left[1]
    mid = ((left[0]+right[0])/2, (left[1]+right[1])/2)
    rods = [polar_point(left, 1/math.sqrt(3), 90+120*k) for k in range(3)]
    rod = min(rods, key=lambda p: math.dist(p, mid))
    cross = vx*(rod[1]-left[1])-vy*(rod[0]-left[0])
    return "左" if cross > 0 else "右"
