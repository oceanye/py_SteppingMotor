"""Fixed stepper-axis topology shared by the desktop controller.

The public ``stepper_axis_topology`` helper intentionally returns the same
dictionary shape that ``pc_gui.py`` historically exposed.  New code can use
the typed, immutable :class:`StepperAxisTopology` representation instead.
"""

from __future__ import annotations

from dataclasses import dataclass
import operator
from typing import Literal, TypedDict


NUM_LOCAL_STEPPER_AXES = 6
NUM_PICO_NODES = 6
PICO_AXES_PER_NODE = 4
NUM_STEPPER_AXES = NUM_LOCAL_STEPPER_AXES + NUM_PICO_NODES * PICO_AXES_PER_NODE
LOCAL_MIN_DELAY_US = 1
REMOTE_MIN_DELAY_US = 100
NUM_AXES = NUM_STEPPER_AXES  # Legacy desktop-GUI alias.

# The local ESP32-S3 PUL/DIR pins mirror esp32_stepper/src/config.h.
STEPPER_PINS: tuple[tuple[int, int], ...] = (
    (5, 6),
    (7, 15),
    (1, 2),
    (4, 8),
    (9, 10),
    (38, 39),
)
LOCAL_AXIS_LABELS: tuple[str, ...] = ("L", "R", "A", "B", "C", "D")
AXIS_LABEL: tuple[str, ...] = LOCAL_AXIS_LABELS + tuple(
    f"P{node}-{local_axis}"
    for node in range(1, NUM_PICO_NODES + 1)
    for local_axis in range(1, PICO_AXES_PER_NODE + 1)
)

ControllerKind = Literal["esp32", "pico"]


class PinAssignment(TypedDict):
    pulse: int
    direction: int


class LegacyAxisTopology(TypedDict):
    axis: int
    label: str
    controller: ControllerKind
    node: int | None
    local_axis: int
    pins: PinAssignment | None


@dataclass(frozen=True, slots=True)
class StepperAxisTopology:
    """Routing metadata for one global stepper-axis number."""

    axis: int
    label: str
    controller: ControllerKind
    node: int | None
    local_axis: int
    pulse_pin: int | None = None
    direction_pin: int | None = None

    @property
    def is_local(self) -> bool:
        return self.controller == "esp32"

    @property
    def is_remote(self) -> bool:
        return self.controller == "pico"

    def as_legacy_dict(self) -> LegacyAxisTopology:
        pins: PinAssignment | None = None
        if self.pulse_pin is not None and self.direction_pin is not None:
            pins = {"pulse": self.pulse_pin, "direction": self.direction_pin}
        return {
            "axis": self.axis,
            "label": self.label,
            "controller": self.controller,
            "node": self.node,
            "local_axis": self.local_axis,
            "pins": pins,
        }


def validate_stepper_axis(axis: int) -> int:
    """Return an integer axis index or raise a precise validation error."""

    if isinstance(axis, bool):
        raise TypeError("stepper axis must be an integer")
    try:
        normalized = operator.index(axis)
    except TypeError as exc:
        raise TypeError("stepper axis must be an integer") from exc
    if not 0 <= normalized < NUM_STEPPER_AXES:
        raise ValueError("stepper axis out of range")
    return normalized


def get_stepper_axis_topology(axis: int) -> StepperAxisTopology:
    """Return typed ESP32/Pico routing metadata for a global axis."""

    axis = validate_stepper_axis(axis)
    if axis < NUM_LOCAL_STEPPER_AXES:
        pulse_pin, direction_pin = STEPPER_PINS[axis]
        return StepperAxisTopology(
            axis=axis,
            label=AXIS_LABEL[axis],
            controller="esp32",
            node=None,
            local_axis=axis,
            pulse_pin=pulse_pin,
            direction_pin=direction_pin,
        )

    offset = axis - NUM_LOCAL_STEPPER_AXES
    return StepperAxisTopology(
        axis=axis,
        label=AXIS_LABEL[axis],
        controller="pico",
        node=offset // PICO_AXES_PER_NODE + 1,
        local_axis=offset % PICO_AXES_PER_NODE,
    )


def stepper_axis_topology(axis: int) -> LegacyAxisTopology:
    """Return routing metadata in the legacy ``pc_gui.py`` dictionary shape."""

    return get_stepper_axis_topology(axis).as_legacy_dict()


def pico_axis_to_global(node: int, local_axis: int) -> int:
    """Map a one-based Pico node and zero-based local axis to a global axis."""

    if isinstance(node, bool) or not isinstance(node, int):
        raise TypeError("Pico node must be an integer")
    if isinstance(local_axis, bool) or not isinstance(local_axis, int):
        raise TypeError("Pico local axis must be an integer")
    if not 1 <= node <= NUM_PICO_NODES:
        raise ValueError("Pico node out of range")
    if not 0 <= local_axis < PICO_AXES_PER_NODE:
        raise ValueError("Pico local axis out of range")
    return NUM_LOCAL_STEPPER_AXES + (node - 1) * PICO_AXES_PER_NODE + local_axis


def clamp_step_delay_us(axis: int, requested_delay_us: int) -> int:
    """Apply the controller-specific minimum pulse delay for one global axis."""

    topology = get_stepper_axis_topology(axis)
    if isinstance(requested_delay_us, bool) or not isinstance(requested_delay_us, int):
        raise TypeError("requested delay must be an integer")
    minimum = LOCAL_MIN_DELAY_US if topology.is_local else REMOTE_MIN_DELAY_US
    return max(minimum, requested_delay_us)
