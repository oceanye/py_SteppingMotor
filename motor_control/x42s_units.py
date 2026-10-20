"""X42S unit boundary: motor degrees/pulses <-> role mm/output degrees.

No GUI, transport or binding imports. Keep PPR aligned with firmware config.h.
The sign is the controller coordinate sign; installation signs belong to gait.
"""
import math

X42S_PULSE_PER_REV = 3200
X42S_SPEED_PPS_MAX = 100000
X42S_AXIS_HANDLE_BASE = 100


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} 必须为有限数值")
    return float(value)


def _positive(value, label):
    result = _finite(value, label)
    if result <= 0:
        raise ValueError(f"{label} 必须大于 0")
    return result


def role_to_output_degrees(value, *, linear, lead_mm):
    value = _finite(value, "位移/速度")
    return value * 360.0 / _positive(lead_mm, "导程") if linear else value


def motor_degrees_to_role(value, *, linear, gear_ratio, lead_mm):
    """Missing telemetry stays unknown, including zero versus missing distinction."""
    if value is None:
        return None
    out = _finite(value, "电机轴位置") / _positive(gear_ratio, "减速比")
    return out * _positive(lead_mm, "导程") / 360.0 if linear else out


def output_degrees_to_pulses(delta, speed, gear_ratio):
    scale = _positive(gear_ratio, "减速比") * X42S_PULSE_PER_REV / 360.0
    steps = _finite(delta, "位移") * scale
    rate = _positive(speed, "速度") * scale
    if not math.isfinite(steps) or not math.isfinite(rate):
        raise ValueError("X42S 位移/速度换算溢出")
    return round(steps), max(1, round(rate))


def role_move_to_pulses(delta, speed, *, linear, gear_ratio, lead_mm):
    return output_degrees_to_pulses(
        role_to_output_degrees(delta, linear=linear, lead_mm=lead_mm),
        role_to_output_degrees(abs(_finite(speed, "速度")), linear=linear, lead_mm=lead_mm),
        gear_ratio,
    )
