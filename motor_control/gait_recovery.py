"""Read-only recovery verification, never an unplanned reverse motor path.

A MOVE ACK proves command acceptance, not arrival. An interrupted avoidance
path cannot be reversed safely by individually returning the four motor angles.
This module checks a manually restored pose before restoring its world ledger.
"""
from .coordinated_control import LOGICAL_ROLE_ORDER
from .x42s_units import motor_degrees_to_role

RECOVERY_TOLERANCE = 0.05  # mm for Mup, output degrees for Mr (not motor degrees)


def recovery_differences(bindings, states, target_positions, *, now):
    """All four axes must have fresh, fault-free, stationary feedback."""
    differences = {}
    for role in LOGICAL_ROLE_ORDER:
        binding = bindings.for_role(role)
        state = states.get(role.value)
        if (binding is None or binding.kind != "x42s" or state is None
                or state.driver_id != binding.driver_id
                or not state.telemetry_trusted_at(now=now) or state.fault
                or not state.enabled or state.reached is not True
                or (state.speed is not None and abs(state.speed) > 0.01)):
            raise ValueError(f"{role.value} 未确认静止/使能/新鲜实测反馈，请先确认现场及遥测")
        target = target_positions.get(role.value)
        # Reuse the strict numerical boundary; no None/NaN or bool as zero.
        target = motor_degrees_to_role(target, linear=False, gear_ratio=1, lead_mm=1)
        if target is None:
            raise ValueError(f"{role.value} 缺少执行前目标位置")
        measured = motor_degrees_to_role(
            state.measured_position, linear=role.value.startswith("Mup"),
            gear_ratio=binding.gear_ratio, lead_mm=binding.lead_mm)
        differences[role.value] = target - measured
    return differences


def recovery_reference(app):
    """Binding/connection/reference changes invalidate the old snapshot."""
    p = app.gait_params
    return (app._gait_hardware_fingerprint(),
            getattr(app, "_control_generation", None),
            getattr(app, "_gait_world_epoch", 0),
            p.mr1_zero_deg, p.mr2_zero_deg, p.mr1_sign, p.mr2_sign,
            p.mup1_lift_sign, p.mup2_lift_sign, p.beam_reference_deg)
