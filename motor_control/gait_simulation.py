"""Independent virtual walking ledger. No Tk, serial, files or real axis state."""
import math
from collections import deque
from dataclasses import dataclass

from .gait_map import map_pads, start_pair_reference
from .gait_planner import effective_geometry, plan_gait_stages, plan_swing_trajectory
from .gait_twin import TwinHistory, pad_center
from .gait_twin_timing import plan_sample_times


def simulation_key(params):
    """Calibration/axis zeros are deliberately not part of the virtual world."""
    return (effective_geometry(params), params.trajectory_mode, params.phase_gain,
            params.lift_mm, params.lift_speed_mm_s, params.settle_speed_mm_s,
            params.swing_speed_deg_s, params.feasibility_samples)


@dataclass(frozen=True)
class SimulationFrame:
    time_s: float
    stage: str
    pose: dict


class GaitSimulation:
    """A step commits only at S7; previews, pause and cancel never move stance.

    Linear stages are ideal constant-speed illustrations; S4 uses the shared
    planner's nominal segment clock. Neither includes physical/ACK delays.
    """
    def __init__(self, params, supports=None):
        self.params = params.validated()
        self.origin = tuple(supports or params.initial_supports)
        self.beta, _ = start_pair_reference(self.origin)
        self.supports = self.origin
        self.psis = (30.0, 30.0)
        self.completed_steps = 0
        self.elapsed_s = 0.0
        self.history = TwinHistory()
        self.curve = deque(maxlen=6000)
        self.segment = 0
        self.frames = ()
        self.index = -1
        self.report = None
        self.paused = False
        self.frame = SimulationFrame(0.0, "起步", self._rest_pose())

    @property
    def active(self):
        return bool(self.frames)

    def _rest_pose(self):
        return {"phi_deg": 0.0, "beta_deg": self.beta, "route_pads": {},
                "feet": {side: {"center": pad_center(name), "psi_deg": psi, "z_mm": 0.0}
                         for side, name, psi in zip(("left", "right"), self.supports, self.psis)}}

    def route(self, side, arc):
        if side not in ("left", "right") or arc not in (-60.0, 60.0):
            raise ValueError("模拟仅支持左/右 × 顺/逆 60°换位")
        start, pivot = self.supports if side == "left" else self.supports[::-1]
        bearing = self.beta + (180.0 if side == "right" else 0.0)
        px, py = pad_center(pivot)
        theta = math.radians(bearing-arc)
        target = (px+math.cos(theta), py+math.sin(theta))
        for name, point in map_pads().items():
            if name not in (start, pivot) and math.dist(point, target) < 1e-6:
                return start, name, pivot, bearing
        raise ValueError("本方向落点超出 5×5 地图；模拟站位未改变")

    def preview(self, side, arc):
        return plan_swing_trajectory(self.params, side=side, arc_deg=arc, route=self.route(side, arc))

    def begin(self, side, arc):
        if self.active:
            raise ValueError("当前模拟未完成，请暂停/继续或取消本步")
        report = self.preview(side, arc)
        if not report.feasible:
            raise ValueError("避让校验未通过，不能累计本步；可用单步回放检查。" + report.message)
        self.report = report
        self.frames = self._frames(report, arc)
        self.index = -1
        self.segment += 1
        self.paused = False
        return report

    def _frames(self, report, arc):
        params, side = self.params, report.side
        d = effective_geometry(params).d_mm
        stages = plan_gait_stages(params, side=side, arc_deg=arc, route=report.route)
        times = plan_sample_times(report, params)
        elapsed, frames, z = self.elapsed_s, [], {"left": 0.0, "right": 0.0}
        sample = report.samples[0]

        def pose(sample):
            feet = {s: {"center": pad_center(n), "psi_deg": psi, "z_mm": z[s]}
                    for s, n, psi in zip(("left", "right"), self.supports, self.psis)}
            center = (sample.center[0]/d, sample.center[1]/d)
            # Exact ideal landings prevent floating-point drift between steps.
            if sample is report.samples[0]:
                center = pad_center(report.route[0])
            elif sample is report.samples[-1]:
                center = pad_center(report.route[1])
            feet[side].update(center=center,
                              psi_deg=feet[side]["psi_deg"]+sample.psi_deg-30.0)
            return {"phi_deg": sample.phi_deg, "beta_deg": sample.beta_deg, "feet": feet,
                    "route_pads": {n: pad_center(n) for n in report.route[:3]}}

        for stage in stages:
            if stage.stage_id == "S4":
                indices = sorted({round(i*(len(report.samples)-1)/180) for i in range(181)})
                for index in indices:
                    sample = report.samples[index]
                    frames.append(SimulationFrame(elapsed+times[index], "S4 公转+自转", pose(sample)))
                elapsed += times[-1]
            elif stage.move_groups:
                for group in stage.move_groups:
                    duration = max(abs(move.delta)/move.speed for move in group)
                    initial = dict(z)
                    for i in range(1, 21):
                        for move in group:
                            foot = "left" if move.role == "Mup1" else "right"
                            sign = params.mup1_lift_sign if foot == "left" else params.mup2_lift_sign
                            z[foot] = initial[foot]+move.delta*sign*i/20
                        frames.append(SimulationFrame(elapsed+duration*i/20,
                                                       f"{stage.stage_id} {stage.title}", pose(sample)))
                    elapsed += duration
            else:
                frames.append(SimulationFrame(elapsed, f"{stage.stage_id} {stage.title}", pose(sample)))
        return tuple(frames)

    def advance(self):
        if not self.active or self.paused:
            return False
        self.index += 1
        self.frame = self.frames[self.index]
        feet = self.frame.pose["feet"]
        left, right = (feet[s]["center"] for s in ("left", "right"))
        mid = tuple((a+b)/2 for a, b in zip(left, right))
        point = (self.segment, left, right, mid, tuple(feet[s]["psi_deg"] for s in ("left", "right")))
        if not self.history.points or point != self.history.points[-1]:
            self.history.points.append(point)
        if self.index == len(self.frames)-1:
            _, target, pivot, _ = self.report.route
            self.supports = (target, pivot) if self.report.side == "left" else (pivot, target)
            self.beta = self.frame.pose["beta_deg"]
            self.psis = tuple(feet[s]["psi_deg"] for s in ("left", "right"))
            self.elapsed_s = self.frame.time_s
            self.completed_steps += 1
            self.history.landings.append((self.completed_steps, left, right))
            self.frames = ()
            return True
        return False

    def clear_path(self):
        self.history.clear()
        self.curve.clear()

    def cancel(self):
        if not self.active:
            return
        self.history.points = deque((p for p in self.history.points if p[0] != self.segment), maxlen=6000)
        self.curve = deque((p for p in self.curve if p[2] != self.segment), maxlen=6000)
        self.frames, self.paused = (), False
        self.frame = SimulationFrame(self.elapsed_s, "已取消本步", self._rest_pose())
