"""Tk view builders for the desktop motor-control application."""

from .main_window import build_ui
from .coordinated_tab import (
    build_coordinated_tab,
    refresh_coordinated_tab,
    sync_binding_editor,
)
from .motor_tabs import build_foc_tab, build_gear_tab
from .gait_tab import (
    build_gait_tab,
    collect_gait_params,
    draw_gait_preview,
    load_gait_fields,
    refresh_gait_panel,
)
from .paired_tab import build_paired_tab
from .stepper_tab import build_stepper_tab
from .track_tab import build_track_tab

__all__ = [
    "build_ui",
    "build_coordinated_tab",
    "build_stepper_tab",
    "build_paired_tab",
    "build_foc_tab",
    "build_track_tab",
    "build_gear_tab",
    "build_gait_tab",
    "collect_gait_params",
    "draw_gait_preview",
    "load_gait_fields",
    "refresh_gait_panel",
    "refresh_coordinated_tab",
    "sync_binding_editor",
]
