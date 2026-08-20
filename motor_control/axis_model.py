"""Typed configuration and step-based runtime state for a stepper axis."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Any

from .axis_math import (
    DEFAULT_GEAR_RATIO,
    DEFAULT_LEAD_MM,
    DEFAULT_PULSE_PER_REV,
    MODE_LINEAR,
    coerce_gear_ratio,
    coerce_lead_mm,
    coerce_pulse_per_rev,
    convert_axis_value,
    pulses_per_unit,
    speed_to_delay_ms,
    speed_unit_label,
    steps_to_units,
    unit_label,
    units_to_exact_steps,
    units_to_steps,
    validate_axis_mode,
)


def _finite_step_coordinate(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


@dataclass(frozen=True, slots=True)
class AxisProfile:
    """Validated physical parameters used to interpret one axis' pulses."""

    mode: str = MODE_LINEAR
    pulse_per_rev: float = DEFAULT_PULSE_PER_REV
    gear_ratio: float = DEFAULT_GEAR_RATIO
    lead_mm: float = DEFAULT_LEAD_MM

    def __post_init__(self) -> None:
        object.__setattr__(self, "mode", validate_axis_mode(self.mode))
        object.__setattr__(
            self, "pulse_per_rev", coerce_pulse_per_rev(self.pulse_per_rev)
        )
        object.__setattr__(self, "gear_ratio", coerce_gear_ratio(self.gear_ratio))
        object.__setattr__(self, "lead_mm", coerce_lead_mm(self.lead_mm))

    @property
    def pulses_per_unit(self) -> float:
        return pulses_per_unit(
            self.mode, self.pulse_per_rev, self.gear_ratio, self.lead_mm
        )

    @property
    def unit(self) -> str:
        return unit_label(self.mode)

    @property
    def speed_unit(self) -> str:
        return speed_unit_label(self.mode)

    def with_mode(self, mode: str) -> "AxisProfile":
        return replace(self, mode=mode)

    def exact_steps_from_units(self, value: object) -> float:
        return units_to_exact_steps(value, self.pulses_per_unit)

    def command_steps_from_units(self, value: object) -> int:
        return units_to_steps(value, self.pulses_per_unit)

    def units_from_steps(self, steps: object) -> float:
        return steps_to_units(steps, self.pulses_per_unit)

    def delay_ms_for_speed(self, speed: object) -> float:
        return speed_to_delay_ms(speed, self.pulses_per_unit)

    def equivalent_value_in(self, value: object | None, other: "AxisProfile") -> float | None:
        return convert_axis_value(value, self.pulses_per_unit, other.pulses_per_unit)


@dataclass(slots=True)
class AxisRuntime:
    """Runtime position and limits in a mode-independent step coordinate.

    Fractional values are allowed so importing legacy user-entered calibration
    values never loses precision.  Actual completed moves still add integral
    pulse counts.
    """

    position_steps: float = 0.0
    min_steps: float | None = None
    max_steps: float | None = None
    position_trusted: bool = False

    def __post_init__(self) -> None:
        self.position_steps = _finite_step_coordinate(
            self.position_steps, "position_steps"
        )
        if self.min_steps is not None:
            self.min_steps = _finite_step_coordinate(self.min_steps, "min_steps")
        if self.max_steps is not None:
            self.max_steps = _finite_step_coordinate(self.max_steps, "max_steps")
        if (
            self.min_steps is not None
            and self.max_steps is not None
            and self.min_steps > self.max_steps
        ):
            raise ValueError("min_steps must not exceed max_steps")
        self.position_trusted = bool(self.position_trusted)

    @classmethod
    def from_legacy_units(
        cls,
        profile: AxisProfile,
        *,
        position: object = 0.0,
        minimum: object | None = None,
        maximum: object | None = None,
        trusted: bool = False,
    ) -> "AxisRuntime":
        """Load the physical-unit fields used by ``.stepper_calib.json``."""

        return cls(
            position_steps=profile.exact_steps_from_units(position),
            min_steps=(
                None if minimum is None else profile.exact_steps_from_units(minimum)
            ),
            max_steps=(
                None if maximum is None else profile.exact_steps_from_units(maximum)
            ),
            position_trusted=trusted,
        )

    def as_legacy_units(self, profile: AxisProfile) -> dict[str, Any]:
        """Serialize with the existing calibration JSON keys and current units."""

        return {
            "min": (
                None
                if self.min_steps is None
                else profile.units_from_steps(self.min_steps)
            ),
            "max": (
                None
                if self.max_steps is None
                else profile.units_from_steps(self.max_steps)
            ),
            "position": profile.units_from_steps(self.position_steps),
            "trusted": self.position_trusted,
        }

    def position_in(self, profile: AxisProfile) -> float:
        return profile.units_from_steps(self.position_steps)

    def limits_in(self, profile: AxisProfile) -> tuple[float | None, float | None]:
        minimum = (
            None
            if self.min_steps is None
            else profile.units_from_steps(self.min_steps)
        )
        maximum = (
            None
            if self.max_steps is None
            else profile.units_from_steps(self.max_steps)
        )
        return minimum, maximum

    def set_position(self, profile: AxisProfile, value: object, *, trusted: bool) -> None:
        self.position_steps = profile.exact_steps_from_units(value)
        self.position_trusted = bool(trusted)

    def set_limits(
        self,
        profile: AxisProfile,
        minimum: object | None,
        maximum: object | None,
    ) -> None:
        min_steps = (
            None if minimum is None else profile.exact_steps_from_units(minimum)
        )
        max_steps = (
            None if maximum is None else profile.exact_steps_from_units(maximum)
        )
        if min_steps is not None and max_steps is not None and min_steps > max_steps:
            raise ValueError("minimum must not exceed maximum")
        self.min_steps = min_steps
        self.max_steps = max_steps

    def apply_completed_steps(self, signed_steps: object) -> None:
        self.position_steps += _finite_step_coordinate(signed_steps, "signed_steps")

    def target_steps(self, profile: AxisProfile, signed_distance: object) -> float:
        return self.position_steps + profile.exact_steps_from_units(signed_distance)

    def target_is_within_limits(self, target_steps: object, *, tolerance_steps: float = 0.0) -> bool:
        target = _finite_step_coordinate(target_steps, "target_steps")
        tolerance = _finite_step_coordinate(tolerance_steps, "tolerance_steps")
        if tolerance < 0:
            raise ValueError("tolerance_steps must not be negative")
        if self.min_steps is not None and target < self.min_steps - tolerance:
            return False
        if self.max_steps is not None and target > self.max_steps + tolerance:
            return False
        return True
