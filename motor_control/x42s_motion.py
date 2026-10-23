"""Per-command endpoint evidence. ACK and reached alone are not completion."""
from dataclasses import dataclass
import math

from .actuator_model import TELEMETRY_FRESH_MAX_AGE_MS
from .x42s_units import X42S_PULSE_PER_REV

MOTOR_ENDPOINT_TOLERANCE_DEG = 720 / X42S_PULSE_PER_REV
# Linear feedback in field logs settled ~0.5 motor degrees away from target
# (=0.00139 mm at lead=1). Keep rotary unchanged; bound linear acceptance in
# mechanism units AND motor units. Not a user calibration or a general overshoot waiver.
LINEAR_ENDPOINT_TOLERANCE_MM = .002


def endpoint_tolerance(role, binding):
    name = getattr(role, "value", role)
    if name not in ("Mup1", "Mup2"):
        return MOTOR_ENDPOINT_TOLERANCE_DEG
    return max(MOTOR_ENDPOINT_TOLERANCE_DEG, min(1.0,
        LINEAR_ENDPOINT_TOLERANCE_MM * 360 * binding.gear_ratio / binding.lead_mm))


@dataclass(frozen=True)
class MoveExpectation:
    owner: object
    target_motor_deg: float
    acknowledged_at: float | None = None
    no_motion: bool = False
    origin_motor_deg: float | None = None
    tolerance_motor_deg: float = MOTOR_ENDPOINT_TOLERANCE_DEG

    def outside_travel(self, position):
        if self.origin_motor_deg is None or position is None:
            return False
        low, high = sorted((self.origin_motor_deg, self.target_motor_deg))
        return position < low-self.tolerance_motor_deg or position > high+self.tolerance_motor_deg


def arrived(move, reply, now, blind_s):
    """A post-ACK, post-blind, stationary, fresh actual endpoint is mandatory.

    The driver has no per-MOVE telemetry sequence ID. Host ownership, a new
    sample timestamp and the quantized absolute target are therefore checked
    together; never accept an unsolicited reached flag without a live command.
    """
    if move is None or move.acknowledged_at is None:
        return False
    position, speed = reply.measured_position, reply.speed
    sample_at = now - reply.age_ms / 1000
    return (reply.online and reply.enabled and reply.fault is None and reply.reached
            and position is not None and math.isfinite(position)
            and reply.age_ms <= TELEMETRY_FRESH_MAX_AGE_MS
            and sample_at > move.acknowledged_at + (0 if move.no_motion else blind_s)
            and (speed is None or (math.isfinite(speed) and abs(speed) <= .01))
            and abs(position-move.target_motor_deg) <= move.tolerance_motor_deg)
