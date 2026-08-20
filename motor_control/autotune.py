"""Tk-free FOC and geared-motor autotuning algorithms.

The scan values, delays, stop conditions, scoring weights, and operator-facing
log messages mirror the original ``pc_gui.StepperGUI`` workers.  Hardware I/O
and time are dependencies, making the runner usable from a worker thread and
fully deterministic in offline tests.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Generic, Iterable, Optional, TypeVar, Union


TraceSample = tuple[float, float, float]  # timestamp, target_deg, current_deg
SendCommand = Callable[[str], object]
TraceProvider = Callable[[int], Iterable[TraceSample]]
LogMessage = Callable[[str], None]
Sleep = Callable[[float], None]
Clock = Callable[[], float]
AxisLabel = Callable[[int], str]
Cancelled = Callable[[], bool]

Setting = TypeVar("Setting", int, float)


@dataclass(frozen=True, slots=True)
class StepResponseMetrics:
    overshoot_deg: float
    steady_state_error_deg: float
    rise_time_s: float
    jitter_deg: float


@dataclass(frozen=True, slots=True)
class TuneTrial(Generic[Setting]):
    setting: Setting
    metrics: StepResponseMetrics


@dataclass(frozen=True, slots=True)
class FocTuneResult:
    completed: bool
    best_pa: Optional[int]
    best_vp: Optional[float]
    pa_trials: tuple[TuneTrial[int], ...]
    vp_trials: tuple[TuneTrial[float], ...]
    failure_reason: Optional[str] = None


@dataclass(frozen=True, slots=True)
class GearTuneResult:
    completed: bool
    best_kp: Optional[int]
    best_kd: Optional[float]
    best_ki: Optional[float]
    kp_trials: tuple[TuneTrial[int], ...]
    kd_trials: tuple[TuneTrial[float], ...]
    ki_trials: tuple[TuneTrial[float], ...]
    verification: Optional[StepResponseMetrics]
    failure_reason: Optional[str] = None


class _AutotuneAborted(RuntimeError):
    def __init__(self, message: str, *, cleanup_allowed: bool) -> None:
        super().__init__(message)
        self.cleanup_allowed = cleanup_allowed


def compute_step_response_metrics(
    trace: Iterable[TraceSample],
    target_deg: float,
    started_at: float,
    duration_s: float,
) -> Optional[StepResponseMetrics]:
    """Compute the same response metrics used by the legacy GUI worker.

    Samples are included only after ``started_at`` and when the firmware's
    reported target is within 0.5 degrees of the requested target.  At least
    five matching samples are required.  The final two seconds determine
    steady-state error and jitter when at least three tail samples exist.
    """

    data = [
        (timestamp - started_at, current)
        for timestamp, reported_target, current in trace
        if timestamp >= started_at and abs(reported_target - target_deg) < 0.5
    ]
    if len(data) < 5:
        return None

    currents = [current for _, current in data]
    if target_deg > 0:
        overshoot = max(0.0, max(currents) - target_deg)
    else:
        overshoot = max(0.0, target_deg - min(currents))

    rise_time: Optional[float] = None
    threshold = target_deg * 0.9
    for relative_time, current in data:
        if current >= threshold:
            rise_time = relative_time
            break

    tail = [current for relative_time, current in data if relative_time >= duration_s - 2.0]
    if len(tail) >= 3:
        tail_average = sum(tail) / len(tail)
        steady_state_error = abs(tail_average - target_deg)
        jitter = math.sqrt(
            sum((value - tail_average) ** 2 for value in tail) / len(tail)
        )
    else:
        steady_state_error = abs(currents[-1] - target_deg)
        jitter = 0.0

    # Keep the original ``rt or 99.0`` behavior, including a crossing at t=0.
    return StepResponseMetrics(
        overshoot_deg=overshoot,
        steady_state_error_deg=steady_state_error,
        rise_time_s=rise_time or 99.0,
        jitter_deg=jitter,
    )


class AutotuneRunner:
    """Run the existing FOC or GEAR scans without depending on Tk."""

    def __init__(
        self,
        *,
        send: SendCommand,
        trace_provider: TraceProvider,
        log: LogMessage,
        sleep: Sleep,
        clock: Clock = time.time,
        axis_label: AxisLabel = str,
        cancelled: Cancelled = lambda: False,
    ) -> None:
        self._send_command = send
        self._trace_provider = trace_provider
        self._log = log
        self._sleep_delay = sleep
        self._clock = clock
        self._axis_label = axis_label
        self._cancelled = cancelled

    def _checkpoint(self) -> None:
        if self._cancelled():
            raise _AutotuneAborted(
                "control generation was cancelled", cleanup_allowed=False
            )

    def _send(self, command: str) -> None:
        self._checkpoint()
        response = self._send_command(command)
        self._checkpoint()
        fields = command.split(",")
        expected = f"OK,{fields[1]}" if len(fields) > 1 else "OK"
        if response != expected:
            detail = str(response) if response else "no reply"
            raise _AutotuneAborted(
                f"{command!r} was not confirmed ({detail})",
                cleanup_allowed=True,
            )

    def _sleep(self, delay: float) -> None:
        self._checkpoint()
        self._sleep_delay(delay)
        self._checkpoint()

    def _safe_cleanup(self, axis: int) -> None:
        """Best-effort return-to-zero and disable after an unsuccessful scan."""

        if self._cancelled():
            return
        expected = f"OK,{axis}"
        # Disable first: commanding A=0 while still enabled could itself cause
        # a full reverse move after an unstable/failed trial.
        for command in (f"FOC,{axis},EN,0", f"FOC,{axis},A,0"):
            response = self._send_command(command)
            if self._cancelled() or response != expected:
                self._log(
                    f"⚠️ 自动调参清理未确认 ({command} → {response or '无响应'})；"
                    "请使用物理急停/断电确认电机已停止"
                )
                return
    def step_response_test(
        self,
        axis: int,
        target_deg: float,
        pre_settle_s: float = 2.5,
        duration_s: float = 5.0,
    ) -> Optional[StepResponseMetrics]:
        self._send(f"FOC,{axis},A,0")
        self._sleep(pre_settle_s)
        started_at = self._clock()
        self._send(f"FOC,{axis},A,{target_deg}")
        self._sleep(duration_s)
        return compute_step_response_metrics(
            self._trace_provider(axis), target_deg, started_at, duration_s
        )

    def run_foc(self, axis: int) -> FocTuneResult:
        try:
            return self._run_foc(axis)
        except _AutotuneAborted as exc:
            if exc.cleanup_allowed:
                self._safe_cleanup(axis)
            self._log(f"⛔ 自动调参已中止: {exc}")
            return FocTuneResult(False, None, None, (), (), str(exc))

    def _run_foc(self, axis: int) -> FocTuneResult:
        label = self._axis_label(axis)
        self._log(f"🤖 轴{label} 自动调参开始")
        self._send(f"FOC,{axis},EN,1")
        self._sleep(5.5)
        self._send(f"FOC,{axis},H")
        self._sleep(0.5)
        self._send(f"FOC,{axis},VP,0.15")

        pa_trials: list[TuneTrial[int]] = []
        for pa in [5, 10, 15, 20, 25, 30]:
            self._send(f"FOC,{axis},PA,{pa}")
            self._sleep(0.3)
            metrics = self.step_response_test(axis, 60.0)
            if metrics is None:
                continue
            self._log(
                f"  PA={pa}: 过冲={metrics.overshoot_deg:.1f}° "
                f"误差={metrics.steady_state_error_deg:.1f}° "
                f"上升={metrics.rise_time_s:.2f}s 抖={metrics.jitter_deg:.2f}°"
            )
            pa_trials.append(TuneTrial(pa, metrics))
            if metrics.overshoot_deg > 30:
                break

        if not pa_trials:
            self._log("❌ 无数据")
            self._safe_cleanup(axis)
            return FocTuneResult(False, None, None, (), (), "no PA response data")

        good_pa = [trial for trial in pa_trials if trial.metrics.overshoot_deg <= 15]
        pa_pool = good_pa if good_pa else pa_trials
        best_pa = min(pa_pool, key=self._foc_pa_score).setting
        self._send(f"FOC,{axis},PA,{best_pa}")
        self._sleep(0.3)

        vp_trials: list[TuneTrial[float]] = []
        for vp in [0.10, 0.15, 0.20, 0.30, 0.40, 0.55, 0.70]:
            self._send(f"FOC,{axis},VP,{vp}")
            self._sleep(0.3)
            metrics = self.step_response_test(axis, 60.0)
            if metrics is None:
                continue
            self._log(
                f"  VP={vp:.2f}: 过冲={metrics.overshoot_deg:.1f}° "
                f"上升={metrics.rise_time_s:.2f}s 抖={metrics.jitter_deg:.2f}°"
            )
            vp_trials.append(TuneTrial(vp, metrics))
            if metrics.jitter_deg > 3.0:
                break

        best_vp = 0.15
        if vp_trials:
            good_vp = [trial for trial in vp_trials if trial.metrics.jitter_deg <= 2.0]
            vp_pool = good_vp if good_vp else vp_trials
            best_vp = min(vp_pool, key=self._foc_vp_score).setting

        self._send(f"FOC,{axis},PA,{best_pa}")
        self._sleep(0.2)
        self._send(f"FOC,{axis},VP,{best_vp}")
        self._sleep(0.2)
        self._send(f"FOC,{axis},A,0")
        self._log(f"✅ 轴{label} 推荐：PA={best_pa}  VP={best_vp:.2f}")
        return FocTuneResult(
            True,
            best_pa,
            best_vp,
            tuple(pa_trials),
            tuple(vp_trials),
        )

    def run_gear(self, axis: int) -> GearTuneResult:
        try:
            return self._run_gear(axis)
        except _AutotuneAborted as exc:
            if exc.cleanup_allowed:
                self._safe_cleanup(axis)
            self._log(f"⛔ GEAR 自动调参已中止: {exc}")
            return GearTuneResult(
                False, None, None, None, (), (), (), None, str(exc)
            )

    def _run_gear(self, axis: int) -> GearTuneResult:
        target = 60.0
        duration = 8.0
        label = self._axis_label(axis)
        self._log(f"🤖 轴{label} GEAR 自动调参开始 (目标 ±{target:.0f}°)")

        self._send(f"FOC,{axis},V,100")
        self._sleep(0.1)
        self._send(f"FOC,{axis},PI,0")
        self._sleep(0.1)
        self._send(f"FOC,{axis},PD,0.5")
        self._sleep(0.1)
        self._send(f"FOC,{axis},PA,5")
        self._sleep(0.1)
        self._send(f"FOC,{axis},EN,1")
        self._sleep(0.5)
        self._send(f"FOC,{axis},H")
        self._sleep(0.3)

        self._log("--- 阶段 1/4: Kp 扫描 ---")
        kp_trials: list[TuneTrial[int]] = []
        for kp in [5, 10, 15, 20, 25, 30]:
            self._send(f"FOC,{axis},PA,{kp}")
            self._sleep(0.3)
            metrics = self.step_response_test(axis, target, pre_settle_s=2.0, duration_s=duration)
            if metrics is None:
                self._log(f"  Kp={kp}: 无数据")
                continue
            self._log(
                f"  Kp={kp}: 过冲={metrics.overshoot_deg:.1f}° "
                f"稳态误差={metrics.steady_state_error_deg:.1f}° "
                f"上升={metrics.rise_time_s:.2f}s 抖={metrics.jitter_deg:.2f}°"
            )
            kp_trials.append(TuneTrial(kp, metrics))
            if metrics.overshoot_deg > 40:
                self._log("  ⚠️ 过冲太大，停止")
                break

        if not kp_trials:
            self._log("❌ Kp 阶段无有效数据，中止")
            self._safe_cleanup(axis)
            return GearTuneResult(
                False, None, None, None, (), (), (), None, "no Kp response data"
            )

        good_kp = [trial for trial in kp_trials if trial.metrics.overshoot_deg <= 15.0]
        kp_pool = good_kp if good_kp else kp_trials
        best_kp = min(kp_pool, key=self._gear_kp_score).setting
        self._log(f"→ 选 Kp = {best_kp}")
        self._send(f"FOC,{axis},PA,{best_kp}")
        self._sleep(0.3)

        self._log("--- 阶段 2/4: Kd 扫描 ---")
        kd_trials: list[TuneTrial[float]] = []
        for kd in [0.3, 0.5, 0.8, 1.0, 1.5, 2.0]:
            self._send(f"FOC,{axis},PD,{kd}")
            self._sleep(0.3)
            metrics = self.step_response_test(axis, target, pre_settle_s=2.0, duration_s=duration)
            if metrics is None:
                continue
            self._log(
                f"  Kd={kd:.1f}: 过冲={metrics.overshoot_deg:.1f}° "
                f"稳态误差={metrics.steady_state_error_deg:.1f}° 抖={metrics.jitter_deg:.2f}°"
            )
            kd_trials.append(TuneTrial(kd, metrics))
            if metrics.jitter_deg > 5.0:
                self._log("  ⚠️ 抖动太大，停止")
                break

        best_kd = 0.5
        if kd_trials:
            good_kd = [
                trial
                for trial in kd_trials
                if trial.metrics.overshoot_deg <= 5.0 and trial.metrics.jitter_deg <= 2.0
            ]
            kd_pool = good_kd if good_kd else kd_trials
            best_kd = min(kd_pool, key=self._gear_kd_score).setting
        self._log(f"→ 选 Kd = {best_kd:.1f}")
        self._send(f"FOC,{axis},PD,{best_kd}")
        self._sleep(0.3)

        self._log("--- 阶段 3/4: Ki 扫描 ---")
        ki_trials: list[TuneTrial[float]] = []
        for ki in [0.02, 0.05, 0.1, 0.15, 0.2, 0.3]:
            self._send(f"FOC,{axis},PI,{ki}")
            self._sleep(0.3)
            metrics = self.step_response_test(axis, target, pre_settle_s=2.5, duration_s=duration)
            if metrics is None:
                continue
            self._log(
                f"  Ki={ki:.2f}: 过冲={metrics.overshoot_deg:.1f}° "
                f"稳态误差={metrics.steady_state_error_deg:.1f}° 抖={metrics.jitter_deg:.2f}°"
            )
            ki_trials.append(TuneTrial(ki, metrics))
            if metrics.overshoot_deg > 15 or metrics.jitter_deg > 3.0:
                self._log(f"  ⚠️ Ki={ki} 失稳，停止")
                break

        best_ki = 0.0
        if ki_trials:
            good_ki = [
                trial
                for trial in ki_trials
                if trial.metrics.steady_state_error_deg <= 1.0
                and trial.metrics.overshoot_deg <= 8.0
            ]
            ki_pool = good_ki if good_ki else ki_trials
            best_ki = min(ki_pool, key=self._gear_ki_score).setting
        self._log(f"→ 选 Ki = {best_ki:.2f}")
        self._send(f"FOC,{axis},PI,{best_ki}")
        self._sleep(0.3)

        self._log("--- 阶段 4/4: 验证 ---")
        self._send(f"FOC,{axis},PA,{best_kp}")
        self._sleep(0.1)
        self._send(f"FOC,{axis},PD,{best_kd}")
        self._sleep(0.1)
        self._send(f"FOC,{axis},PI,{best_ki}")
        self._sleep(0.1)
        self._send(f"FOC,{axis},H")
        self._sleep(1.0)
        verification = self.step_response_test(
            axis, target, pre_settle_s=3.0, duration_s=duration
        )
        if verification is not None:
            self._log(
                f"  验证结果: 过冲={verification.overshoot_deg:.1f}° "
                f"稳态误差={verification.steady_state_error_deg:.1f}° "
                f"上升={verification.rise_time_s:.2f}s 抖={verification.jitter_deg:.2f}°"
            )
            if verification.steady_state_error_deg <= 1.0:
                self._log("✅ 稳态误差 ≤ 1°，达标！")
            else:
                self._log(
                    f"⚠️ 稳态误差 {verification.steady_state_error_deg:.1f}° > 1°，"
                    "建议手动微调 Ki"
                )
        else:
            self._log("  验证: 无数据")

        self._send(f"FOC,{axis},A,0")
        self._sleep(2.0)
        self._log(
            f"✅ 轴{label} 推荐：Kp={best_kp}  Kd={best_kd:.1f}  Ki={best_ki:.2f}"
        )
        return GearTuneResult(
            True,
            best_kp,
            best_kd,
            best_ki,
            tuple(kp_trials),
            tuple(kd_trials),
            tuple(ki_trials),
            verification,
        )

    @staticmethod
    def _foc_pa_score(trial: TuneTrial[int]) -> float:
        metrics = trial.metrics
        return (
            metrics.overshoot_deg * 2
            + metrics.rise_time_s * 3
            + metrics.steady_state_error_deg * 2
            + metrics.jitter_deg * 5
        )

    @staticmethod
    def _foc_vp_score(trial: TuneTrial[float]) -> float:
        metrics = trial.metrics
        return (
            metrics.overshoot_deg * 2
            + metrics.rise_time_s * 2
            + metrics.jitter_deg * 10
        )

    @staticmethod
    def _gear_kp_score(trial: TuneTrial[int]) -> float:
        metrics = trial.metrics
        return (
            metrics.steady_state_error_deg * 5
            + metrics.overshoot_deg * 3
            + metrics.rise_time_s * 2
            + metrics.jitter_deg * 2
        )

    @staticmethod
    def _gear_kd_score(trial: TuneTrial[float]) -> float:
        metrics = trial.metrics
        return (
            metrics.overshoot_deg * 4
            + metrics.jitter_deg * 5
            + metrics.steady_state_error_deg * 3
        )

    @staticmethod
    def _gear_ki_score(trial: TuneTrial[float]) -> float:
        metrics = trial.metrics
        return (
            metrics.steady_state_error_deg * 10
            + metrics.overshoot_deg * 3
            + metrics.jitter_deg * 3
        )
