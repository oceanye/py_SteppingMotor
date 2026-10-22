"""Two-step map trails. Pure display, independent of the full world ledger."""
import math
from motor_control.gait_coordinates import polar_point

FOOT_COLORS = {"left": "#2563eb", "right": "#ea580c"}
PAST_COLORS = {"left": "#bfdbfe", "right": "#fed7aa"}


def draw_recent_paths(canvas, history, project, arm, prefix):
    """Previous step pale/thin, current step saturated/solid; no older paths.

    Retain foot centre and tip traces for sweep analysis, but omit stationary
    traces, the redundant body-centre line and the accumulated landing labels.
    Telemetry gaps remain separate polylines even within the same step.
    """
    def line(points, color, width, tags):
        distinct = []
        for point in points:
            if not distinct or math.dist(point, distinct[-1]) > 1e-8:
                distinct.append(point)
        if len(distinct) >= 2:
            canvas.create_line(*(v for point in distinct for v in project(point)),
                               fill=color, width=width, tags=tags)

    for age, segment in history.recent_segments():
        if len(segment) < 2:
            continue
        age_tag = "path_previous" if age else "path_current"
        step = history.segment_steps.get(segment[0][0], segment[0][0])
        step_tag = f"path_step_{step}"
        for index, side in ((1, "left"), (2, "right")):
            color = (PAST_COLORS if age else FOOT_COLORS)[side]
            line([point[index] for point in segment], color, 1.2 if age else 2.2,
                 (f"{prefix}_{side}", age_tag, step_tag))
            for k in range(3):
                tips = []
                for point in segment:
                    psi = point[4][index-1]
                    if psi is None:
                        # Missing phase is a gap, not permission to join tips.
                        line(tips, color, 0.8 if age else 1.3,
                             (f"{prefix}_tips_{side}_{k}", age_tag, step_tag))
                        tips = []
                        continue
                    cx, cy = point[index]
                    tips.append(polar_point((cx, cy), arm, psi+120*k))
                line(tips, color, 0.8 if age else 1.3,
                     (f"{prefix}_tips_{side}_{k}", age_tag, step_tag))
