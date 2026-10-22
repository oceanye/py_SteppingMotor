"""Certify X42S independent-axis progress instead of assuming atomic SYNC.

FD MOVE uses immediate, independent trapezoids. For each rest-to-rest pair we
certify the ENTIRE joint-angle rectangle, including either axis finishing
first. Refine along the existing angle polyline until the rectangle is clear.
The capsule distance is 1-Lipschitz in point displacement, giving the bound:
  d * |half support span| + R * (|half swing span| + |half support span|).
Angles in that expression are radians. A supplied endpoint/reference error
budget is subtracted too. This does not certify unbounded servo overshoot,
structural flex, incorrect dimensions or slipping supports: bench checks remain.
"""
from dataclasses import replace
import math

from .gait_avoidance import leg_clearance
from .gait_coordinates import polar_point
from .gait_planner import RoleMove, effective_geometry


def certify_x42s_stages(stages, params, report, error_budget_mm):
    if not math.isfinite(error_budget_mm) or error_budget_mm < 0:
        raise ValueError("无效的 X42S 执行误差包络")
    geometry = effective_geometry(params)
    pivot = next(pad.center for pad in report.hexagons if pad.name == report.route[2])
    bearing = report.route[3]
    swing_sign, support_sign = ((params.mr1_sign, params.mr2_sign) if report.side == "left"
                                else (params.mr2_sign, params.mr1_sign))
    result = []
    for stage in stages:
        if stage.stage_id != "S4":
            result.append(stage)
            continue
        if not stage.sync_endpoints:
            raise ValueError("X42S 两模态执行需要累计终点轨迹，不能独立 MOVE 降级")
        groups, endpoints, durations, bounds = [], [], [], []

        def refine(a, b, duration, template, depth=0):
            qs = (a[0]+b[0])/2 * swing_sign
            qp = (a[1]+b[1])/2 * support_sign
            center = polar_point(pivot, geometry.d_mm, bearing-qp)
            swing_half, support_half = (abs(y-x)/2 for x, y in zip(a, b))
            displacement = math.radians(support_half)*geometry.d_mm + math.radians(
                swing_half+support_half)*geometry.arm_length_mm
            margin = leg_clearance(center, 30+qs-qp, geometry, report.hexagons)[0]
            lower = margin - displacement - error_budget_mm
            if lower <= 0:
                if depth >= 12 or max(abs(y-x) for x, y in zip(a, b)) < 1e-5:
                    raise ValueError(f"X42S 异步双轴扫掠包络不可行（下界 {lower:.3f}mm），未下发运动")
                middle = tuple((x+y)/2 for x, y in zip(a, b))
                refine(a, middle, duration/2, template, depth+1)
                refine(middle, b, duration/2, template, depth+1)
                return
            if len(groups) >= 4096:
                raise ValueError("X42S 避让分段超过上限，需核实尺寸/路径，未下发运动")
            groups.append(tuple(RoleMove(move.role, y-x, move.speed)
                                for move, x, y in zip(template, a, b)))
            endpoints.append(b)
            durations.append(duration)
            bounds.append(lower)

        previous = (0., 0.)
        for group, endpoint, duration in zip(stage.move_groups, stage.sync_endpoints, stage.group_durations):
            refine(previous, endpoint, duration, group)
            previous = endpoint
        result.append(replace(stage, move_groups=tuple(groups), sync_endpoints=tuple(endpoints),
                              group_durations=tuple(durations), execution_profile="x42s_bounded_progress",
                              certified_margin_mm=min(bounds),
                              confirm_text=(f"X42S 双轴进度独立；{len(groups)} 段整腿扫掠包络已校验，"
                                            f"保守余量 {min(bounds):.3f}mm。段间核验双轴实测终点。\n"
                                            "灰线为名义角度路径，不代表实机五次同步或实际耗时。"
                                            "观察高杆间隙，异常立即中止。")))
    return result
