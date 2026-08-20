"""Compatibility launcher for the modular desktop motor controller.

Field scripts and handoff documents continue to use ``python pc_gui.py``.
The implementation lives under :mod:`motor_control`; this file intentionally
stays small and re-exports the historically useful names.
"""

from __future__ import annotations

from pathlib import Path

from motor_control.bootstrap import configure_standard_streams


_PROJECT_PATH = Path(__file__).resolve().parent
configure_standard_streams(_PROJECT_PATH)

from motor_control import (  # noqa: E402 - bootstrap must run first under pythonw
    AXIS_LABEL,
    DEFAULT_GEAR_RATIO,
    DEFAULT_LEAD_MM,
    DEFAULT_PULSE_PER_REV,
    DELAY_OVERHEAD_US,
    LOCAL_AXIS_LABELS,
    MODE_LINEAR,
    MODE_ROTARY,
    NUM_LOCAL_STEPPER_AXES,
    NUM_PICO_NODES,
    NUM_STEPPER_AXES,
    PICO_AXES_PER_NODE,
    SPEED_DEFAULT,
    STEPPER_PINS,
    stepper_axis_topology,
)
from motor_control.desktop_app import (  # noqa: E402
    CONTINUOUS_BURST_MM,
    DELAY_DEFAULT_MS,
    DIR_INVERT,
    DIR_INWARD,
    DIR_OUTWARD,
    FOC_POLL_INTERVAL_S,
    FOC_STATE_NAMES,
    GEAR_STATE_NAMES,
    MODE_FOC,
    MODE_GEAR,
    NUM_AXES,
    NUM_GEAR_AXES,
    NUM_MOTOR_AXES,
    PULSES_PER_MM,
    SPEED_PRESETS,
    StepperGUI,
    _empty_axis_dict,
    _speed_to_delay_ms,
)


# Preserve the file/path constants exposed by the historical single-file GUI.
# They stay strings because some field-side helper scripts import them directly.
PROJECT_DIR = str(_PROJECT_PATH)
CALIB_FILE = str(_PROJECT_PATH / ".stepper_calib.json")
FOC_TUNE_FILE = str(_PROJECT_PATH / ".foc_tune.json")
GEAR_TUNE_FILE = str(_PROJECT_PATH / ".gear_tune.json")
LOG_DIR = str(_PROJECT_PATH / "logs")


def main() -> None:
    import tkinter as tk

    from web_control import WebControlServer

    root = tk.Tk()
    StepperGUI(root, lambda controller: WebControlServer(controller))
    root.mainloop()


if __name__ == "__main__":
    main()
