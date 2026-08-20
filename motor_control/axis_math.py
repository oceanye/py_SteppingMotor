"""Pure stepper-axis validation and unit conversion helpers."""

from __future__ import annotations

import math
from typing import Final


MODE_LINEAR: Final = "linear"
MODE_ROTARY: Final = "rotary"
AXIS_MODES: Final = (MODE_LINEAR, MODE_ROTARY)

DEFAULT_PULSE_PER_REV: Final = 200.0
DEFAULT_GEAR_RATIO: Final = 1.0
DEFAULT_LEAD_MM: Final = 1.0

PULSE_PER_REV_MIN: Final = 1.0
PULSE_PER_REV_MAX: Final = 10_000.0
GEAR_RATIO_MIN: Final = 0.001
GEAR_RATIO_MAX: Final = 1_000.0
LEAD_MM_MIN: Final = 0.01
LEAD_MM_MAX: Final = 100.0

SPEED_DEFAULT: Final = 3.0
DELAY_OVERHEAD_US: Final = 200.0
MIN_DELAY_MS: Final = 0.05


def coerce_finite_in_range(
    value: object,
    minimum: float,
    maximum: float,
    label: str,
) -> float:
    """Convert a numeric value and require a finite, inclusive range."""

    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be a number") from exc
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(f"{label} must be in the range {minimum:g}-{maximum:g}")
    return number


def coerce_pulse_per_rev(value: object) -> float:
    return coerce_finite_in_range(
        value, PULSE_PER_REV_MIN, PULSE_PER_REV_MAX, "pulse_per_rev"
    )


def coerce_gear_ratio(value: object) -> float:
    return coerce_finite_in_range(
        value, GEAR_RATIO_MIN, GEAR_RATIO_MAX, "gear_ratio"
    )


def coerce_lead_mm(value: object) -> float:
    return coerce_finite_in_range(value, LEAD_MM_MIN, LEAD_MM_MAX, "lead_mm")


def validate_axis_mode(mode: str) -> str:
    if mode not in AXIS_MODES:
        raise ValueError("mode must be 'linear' or 'rotary'")
    return mode


def _positive_finite(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be a number") from exc
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{label} must be finite and greater than zero")
    return number


def _finite(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def pulses_per_unit(
    mode: str,
    pulse_per_rev: object = DEFAULT_PULSE_PER_REV,
    gear_ratio: object = DEFAULT_GEAR_RATIO,
    lead_mm: object = DEFAULT_LEAD_MM,
) -> float:
    """Return pulses/mm for linear mode or pulses/degree for rotary mode."""

    mode = validate_axis_mode(mode)
    ppr = coerce_pulse_per_rev(pulse_per_rev)
    ratio = coerce_gear_ratio(gear_ratio)
    # The GUI validates lead even while its rotary-mode input is disabled.
    lead = coerce_lead_mm(lead_mm)
    if mode == MODE_ROTARY:
        return ppr * ratio / 360.0
    return ppr * ratio / lead


def convert_axis_value(
    value: object | None,
    old_pulses_per_unit: object,
    new_pulses_per_unit: object,
) -> float | None:
    """Convert a displayed position/limit while preserving equivalent pulses."""

    if value is None:
        return None
    numeric_value = _finite(value, "axis value")
    old_ppu = _positive_finite(old_pulses_per_unit, "old pulses_per_unit")
    new_ppu = _positive_finite(new_pulses_per_unit, "new pulses_per_unit")
    return numeric_value * old_ppu / new_ppu


def units_to_exact_steps(value: object, pulses_per_unit_value: object) -> float:
    """Map a calibration/display value to the canonical (possibly fractional) step coordinate."""

    return _finite(value, "axis value") * _positive_finite(
        pulses_per_unit_value, "pulses_per_unit"
    )


def units_to_steps(value: object, pulses_per_unit_value: object) -> int:
    """Map a commanded distance to whole pulses using the legacy rounding rule."""

    return int(round(units_to_exact_steps(value, pulses_per_unit_value)))


def steps_to_units(steps: object, pulses_per_unit_value: object) -> float:
    """Map a canonical step coordinate back to the current display unit."""

    return _finite(steps, "steps") / _positive_finite(
        pulses_per_unit_value, "pulses_per_unit"
    )


def speed_to_delay_ms(speed: object, pulses_per_unit_value: object) -> float:
    """Convert units/s to the compensated firmware pulse delay in milliseconds.

    This preserves the existing ESP32 timing rule: delays of at least 2 ms are
    sent without compensation; faster moves subtract the measured 200 us
    execution overhead and clamp the command delay to 0.05 ms.
    """

    speed_value = _positive_finite(speed, "speed")
    ppu = _positive_finite(pulses_per_unit_value, "pulses_per_unit")
    target_us = 1_000_000.0 / (ppu * speed_value)
    if target_us >= 2_000.0:
        return target_us / 1_000.0
    return max(MIN_DELAY_MS, (target_us - DELAY_OVERHEAD_US) / 1_000.0)


def unit_label(mode: str) -> str:
    return "\N{DEGREE SIGN}" if validate_axis_mode(mode) == MODE_ROTARY else "mm"


def speed_unit_label(mode: str) -> str:
    return f"{unit_label(mode)}/s"
