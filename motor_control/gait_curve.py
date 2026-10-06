"""Display-only step curves. Never used as a motion or safety input."""
import math
from bisect import bisect_left
from collections import deque

CURVE_Y_MIN_MM = -20.0
CURVE_Y_MAX_MM = 200.0
CURVE_WINDOW_S = 90.0


def rolling_window(current_s):
    """Constant-width window, reset to zero for each new step."""
    end = max(CURVE_WINDOW_S, current_s)
    return end-CURVE_WINDOW_S, end


def clip_series(data, start, end):
    """Clip with boundary interpolation, never bridge a telemetry/step gap."""
    result = []
    for point in data:
        if start <= point[0] <= end:
            result.append(point)
        if not result and point[0] > end:
            break
    # Preserve lines crossing a boundary even when neither sample is inside.
    for a, b in zip(data, data[1:]):
        if a[2] != b[2] or b[0] <= a[0]:
            continue
        for boundary in (start, end):
            if a[0] < boundary < b[0]:
                ratio = (boundary-a[0])/(b[0]-a[0])
                values = tuple(x+(y-x)*ratio for x, y in zip(a[1], b[1]))
                result.append((boundary, values, a[2]))
    return sorted(result, key=lambda p: p[0])


def time_at_phi(report, times, phi):
    """Map reported orbit progress to the S4 model clock, not wall time.

    Confirmation/network waits cannot advance this clock. Both signed arcs
    work; sensor/quantisation overshoot clamps to the planned interval.
    """
    sign = 1 if report.samples[-1].phi_deg >= 0 else -1
    angles = [sign*s.phi_deg for s in report.samples]
    progress = sign*phi
    if progress <= angles[0]:
        return times[0]
    if progress >= angles[-1]:
        return times[-1]
    index = bisect_left(angles, progress)
    fraction = (progress-angles[index-1])/(angles[index]-angles[index-1])
    return times[index-1]+fraction*(times[index]-times[index-1])


class EncoderCurveBuffer:
    """Future UI-thread adapter boundary, empty unless genuine data arrives.

    Adapter must supply calibrated six-tip distances, source='encoder', and
    the current step token. X is S4 model-progress time, NOT encoder wall time.
    No adapter/serial acquisition is provided here and pulse data never feeds
    this buffer. New step/world binds invalidate all previous measurements.
    """
    def __init__(self):
        self.step_key = None
        self.samples = deque(maxlen=1800)

    def bind(self, step_key):
        if step_key != self.step_key:
            self.step_key = step_key
            self.samples.clear()

    def append(self, step_key, s4_model_time_s, distances_mm, *, source):
        if source != "encoder" or step_key is None or step_key != self.step_key:
            raise ValueError("编码器曲线的数据源/当前步标识不匹配")
        values = tuple(distances_mm)
        numbers = (s4_model_time_s, *values)
        if len(values) != 6 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                   or not math.isfinite(v) for v in numbers):
            raise ValueError("编码器曲线需要有限时间和六爪距离")
        if s4_model_time_s < 0 or (self.samples and s4_model_time_s < self.samples[-1][0]):
            raise ValueError("编码器模型进度时间不能倒退")
        self.samples.append((s4_model_time_s, values, 0))
