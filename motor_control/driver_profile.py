"""Hardware contract for the project's step/direction driver migration."""

from __future__ import annotations

import math
from typing import Final


STEPPER_DRIVER_PROFILE_ID: Final = "mks-tmc2209-v2-standalone-step-dir-v1"
STEPPER_DRIVER_DISPLAY_NAME: Final = "MKS TMC2209 V2.0"
STEPPER_DRIVER_CONTROL_MODE: Final = "standalone_step_dir"

# The phase-1 migration deliberately does not use PDN_UART or DIAG.  With the
# module's MS1 and MS2 inputs both low, TMC2209 selects 1/8 input microsteps.
TMC2209_DEFAULT_MICROSTEPS: Final = 8
TMC2209_STANDALONE_MICROSTEPS: Final = (8, 16, 32, 64)
DEFAULT_MOTOR_FULL_STEPS_PER_REV: Final = 200


def tmc2209_input_pulses_per_rev(
    full_steps_per_rev: object = DEFAULT_MOTOR_FULL_STEPS_PER_REV,
    microsteps: object = TMC2209_DEFAULT_MICROSTEPS,
) -> int:
    """Return STEP rising edges per motor revolution for standalone mode."""

    try:
        full_steps_value = float(full_steps_per_rev)
        microsteps_value = float(microsteps)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("full steps and microsteps must be integers") from exc
    if (
        not math.isfinite(full_steps_value)
        or not full_steps_value.is_integer()
        or full_steps_value <= 0
    ):
        raise ValueError("full_steps_per_rev must be a positive integer")
    if (
        not math.isfinite(microsteps_value)
        or not microsteps_value.is_integer()
        or int(microsteps_value) not in TMC2209_STANDALONE_MICROSTEPS
    ):
        raise ValueError(
            "standalone TMC2209 microsteps must be one of 8, 16, 32, 64"
        )
    return int(full_steps_value) * int(microsteps_value)
