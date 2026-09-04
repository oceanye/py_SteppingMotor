"""Stable controller contract consumed by the dependency-free HTTP server."""

from __future__ import annotations

from typing import Any, Protocol


class WebController(Protocol):
    """Operations exposed by :class:`web_control.WebControlServer`.

    Implementations may be backed by the desktop application or a headless
    controller, but must be safe to call from HTTP worker threads.
    """

    def web_get_status(self) -> dict[str, Any]: ...

    def web_stepper_move(
        self,
        axis: int,
        direction: str | int,
        distance_mm: float,
        speed_mm_s: float = 3.0,
        confirm_high_rate: bool = False,
    ) -> dict[str, Any]: ...

    def web_stepper_stop(self, axis: int) -> dict[str, Any]: ...

    def web_stepper_config(
        self,
        axis: int,
        mode: str | None = None,
        pulse_per_rev: float | None = None,
        gear_ratio: float | None = None,
        lead_mm: float | None = None,
    ) -> dict[str, Any]: ...

    def web_motor_command(
        self,
        mode: str,
        axis: int,
        action: str,
        target_deg: float | None = None,
    ) -> dict[str, Any]: ...

    def web_track_command(
        self,
        action: str,
        pwm: int = 0,
        lease_ms: int = 0,
    ) -> dict[str, Any]: ...

    def web_emergency_stop(self) -> dict[str, Any]: ...
