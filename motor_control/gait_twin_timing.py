"""Read-only planned S4 time axis. No Tk, IO or changes to motor commands."""
from .gait_planner import plan_gait_stages, smoothstep5


def inverse_progress(progress):
    if progress <= 0:
        return 0.0
    if progress >= 1:
        return 1.0
    lo, hi = 0.0, 1.0
    for _ in range(36):
        mid = (lo+hi)/2
        if smoothstep5(mid) < progress:
            lo = mid
        else:
            hi = mid
    return (lo+hi)/2


def plan_sample_times(report, params):
    """Nominal segment clocks; excludes host/ACK waits and non-S4 stages.

    Equal-angle samples are not equally spaced in time. Compute stages once,
    with this route/direction, then invert each segment's quintic progress.
    """
    arc = report.samples[-1].phi_deg
    s4 = next(s for s in plan_gait_stages(
        params, side=report.side, route=report.route, arc_deg=arc,
        swing_psi_start_deg=report.samples[0].psi_deg) if s.stage_id == "S4")
    durations = s4.group_durations or (s4.duration_s,)
    ends = [abs(pair[1]/arc) for pair in s4.sync_endpoints] if s4.sync_endpoints else [1.0]
    segment, elapsed, start = 0, 0.0, 0.0
    times = []
    for sample in report.samples:
        progress = max(0.0, min(1.0, sample.phi_deg/arc))
        while segment < len(ends)-1 and progress > ends[segment]+1e-12:
            elapsed += durations[segment]
            start = ends[segment]
            segment += 1
        fraction = (progress-start)/(ends[segment]-start)
        times.append(elapsed+durations[segment]*inverse_progress(fraction))
    return times
