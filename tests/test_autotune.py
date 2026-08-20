import math
import unittest

from motor_control.autotune import (
    AutotuneRunner,
    FocTuneResult,
    GearTuneResult,
    StepResponseMetrics,
    TuneTrial,
    compute_step_response_metrics,
)


class StepResponseMetricTests(unittest.TestCase):
    def test_response_metrics_preserve_original_formulas(self):
        trace = [
            (9.9, 60.0, 99.0),  # Before the test start: ignored.
            (10.0, 59.0, 99.0),  # Reported target differs by >= 0.5: ignored.
            (10.0, 60.0, 0.0),
            (11.0, 60.0, 30.0),
            (12.0, 60.0, 54.0),
            (13.0, 60.0, 66.0),
            (14.0, 60.0, 61.0),
            (15.0, 60.0, 59.0),
        ]

        metrics = compute_step_response_metrics(trace, 60.0, 10.0, 5.0)

        self.assertIsInstance(metrics, StepResponseMetrics)
        self.assertAlmostEqual(metrics.overshoot_deg, 6.0)
        self.assertAlmostEqual(metrics.steady_state_error_deg, 2.0)
        self.assertAlmostEqual(metrics.rise_time_s, 2.0)
        self.assertAlmostEqual(metrics.jitter_deg, math.sqrt(26.0 / 3.0))

    def test_fewer_than_five_matching_samples_returns_none(self):
        trace = [(100.0 + index, 60.0, float(index)) for index in range(4)]
        self.assertIsNone(compute_step_response_metrics(trace, 60.0, 100.0, 5.0))

    def test_crossing_at_zero_keeps_legacy_99_second_fallback(self):
        trace = [(100.0 + index, 60.0, 60.0) for index in range(6)]
        metrics = compute_step_response_metrics(trace, 60.0, 100.0, 5.0)
        self.assertEqual(metrics.rise_time_s, 99.0)


class FakeTuneRig:
    def __init__(self, curves, duration):
        self.curves = curves
        self.duration = duration
        self.commands = []
        self.delays = []
        self.logs = []
        self.last_setting = None
        self.now = 100.0

    def send(self, command):
        self.commands.append(command)
        fields = command.split(",")
        if len(fields) == 4 and fields[0] == "FOC" and fields[2] in ("PA", "VP", "PD", "PI"):
            self.last_setting = (fields[2], float(fields[3]))
        return f"OK,{fields[1]}"

    def trace(self, _axis):
        currents = self.curves[self.last_setting]
        return [
            (self.now + index, 60.0, current)
            for index, current in enumerate(currents)
        ]

    def sleep(self, delay):
        # Intentionally no real sleep; retain calls so timing semantics can be asserted.
        self.delays.append(delay)

    def clock(self):
        return self.now

    def runner(self):
        return AutotuneRunner(
            send=self.send,
            trace_provider=self.trace,
            log=self.logs.append,
            sleep=self.sleep,
            clock=self.clock,
            axis_label=lambda axis: ("L", "R")[axis],
        )


class FocAutotuneTests(unittest.TestCase):
    def test_foc_score_weights_are_unchanged(self):
        trial = TuneTrial(1.0, StepResponseMetrics(2.0, 3.0, 4.0, 5.0))
        self.assertEqual(AutotuneRunner._foc_pa_score(trial), 47.0)
        self.assertEqual(AutotuneRunner._foc_vp_score(trial), 62.0)

    def test_foc_scan_selects_best_pa_and_vp_and_preserves_stop_rules(self):
        poor = [0.0, 20.0, 40.0, 50.0, 55.0, 55.0]
        perfect = [0.0, 55.0, 60.0, 60.0, 60.0, 60.0]
        excessive_overshoot = [0.0, 55.0, 100.0, 60.0, 60.0, 60.0]
        excessive_jitter = [0.0, 55.0, 60.0, 50.0, 70.0, 50.0]
        curves = {
            ("PA", 5.0): poor,
            ("PA", 10.0): perfect,
            ("PA", 15.0): poor,
            ("PA", 20.0): poor,
            ("PA", 25.0): excessive_overshoot,
            ("VP", 0.10): poor,
            ("VP", 0.15): perfect,
            ("VP", 0.20): poor,
            ("VP", 0.30): excessive_jitter,
        }
        rig = FakeTuneRig(curves, duration=5.0)

        result = rig.runner().run_foc(0)

        self.assertIsInstance(result, FocTuneResult)
        self.assertTrue(result.completed)
        self.assertEqual((result.best_pa, result.best_vp), (10, 0.15))
        self.assertEqual([trial.setting for trial in result.pa_trials], [5, 10, 15, 20, 25])
        self.assertEqual([trial.setting for trial in result.vp_trials], [0.10, 0.15, 0.20, 0.30])

        pa_commands = [command for command in rig.commands if command.startswith("FOC,0,PA,")]
        vp_commands = [command for command in rig.commands if command.startswith("FOC,0,VP,")]
        self.assertEqual(
            pa_commands,
            [
                "FOC,0,PA,5",
                "FOC,0,PA,10",
                "FOC,0,PA,15",
                "FOC,0,PA,20",
                "FOC,0,PA,25",
                "FOC,0,PA,10",
                "FOC,0,PA,10",
            ],
        )
        self.assertEqual(
            vp_commands,
            [
                "FOC,0,VP,0.15",
                "FOC,0,VP,0.1",
                "FOC,0,VP,0.15",
                "FOC,0,VP,0.2",
                "FOC,0,VP,0.3",
                "FOC,0,VP,0.15",
            ],
        )
        expected_delays = (
            [5.5, 0.5]
            + [0.3, 2.5, 5.0] * 5
            + [0.3]
            + [0.3, 2.5, 5.0] * 4
            + [0.2, 0.2]
        )
        self.assertEqual(rig.delays, expected_delays)
        self.assertEqual(rig.logs[0], "🤖 轴L 自动调参开始")
        self.assertEqual(rig.logs[-1], "✅ 轴L 推荐：PA=10  VP=0.15")

    def test_foc_no_response_data_returns_typed_failure(self):
        short_trace = [0.0, 20.0, 40.0, 50.0]
        curves = {("PA", float(pa)): short_trace for pa in [5, 10, 15, 20, 25, 30]}
        rig = FakeTuneRig(curves, duration=5.0)

        result = rig.runner().run_foc(0)

        self.assertFalse(result.completed)
        self.assertIsNone(result.best_pa)
        self.assertEqual(result.failure_reason, "no PA response data")
        self.assertEqual(rig.logs[-1], "❌ 无数据")

    def test_foc_aborts_on_unconfirmed_command(self):
        commands = []

        def reject(command):
            commands.append(command)
            return "ERR:hardware estop active"

        runner = AutotuneRunner(
            send=reject,
            trace_provider=lambda _axis: (),
            log=lambda _message: None,
            sleep=lambda _delay: None,
        )

        result = runner.run_foc(0)

        self.assertFalse(result.completed)
        self.assertIn("not confirmed", result.failure_reason)
        self.assertEqual(commands, ["FOC,0,EN,1", "FOC,0,EN,0"])

    def test_foc_failure_cleanup_disables_before_clearing_target(self):
        commands = []

        def respond(command):
            commands.append(command)
            if command == "FOC,0,EN,1":
                return "ERR:tuning rejected"
            return "OK,0"

        runner = AutotuneRunner(
            send=respond,
            trace_provider=lambda _axis: (),
            log=lambda _message: None,
            sleep=lambda _delay: None,
        )

        result = runner.run_foc(0)

        self.assertFalse(result.completed)
        self.assertEqual(
            commands,
            ["FOC,0,EN,1", "FOC,0,EN,0", "FOC,0,A,0"],
        )

    def test_foc_cancellation_prevents_first_command(self):
        commands = []
        runner = AutotuneRunner(
            send=lambda command: commands.append(command),
            trace_provider=lambda _axis: (),
            log=lambda _message: None,
            sleep=lambda _delay: None,
            cancelled=lambda: True,
        )

        result = runner.run_foc(0)

        self.assertFalse(result.completed)
        self.assertEqual(commands, [])
        self.assertIn("cancelled", result.failure_reason)


class GearAutotuneTests(unittest.TestCase):
    def test_gear_score_weights_are_unchanged(self):
        trial = TuneTrial(1.0, StepResponseMetrics(2.0, 3.0, 4.0, 5.0))
        self.assertEqual(AutotuneRunner._gear_kp_score(trial), 39.0)
        self.assertEqual(AutotuneRunner._gear_kd_score(trial), 42.0)
        self.assertEqual(AutotuneRunner._gear_ki_score(trial), 51.0)

    def test_gear_scan_selects_best_kp_kd_ki_and_verifies(self):
        poor_kp = [0.0, 20.0, 40.0, 50.0, 54.0, 55.0, 55.0, 55.0, 55.0]
        poor_kd = [0.0, 50.0, 58.0, 58.0, 58.0, 58.0, 58.0, 58.0, 58.0]
        poor_ki = [0.0, 55.0, 59.0, 59.0, 59.0, 59.0, 59.0, 59.0, 59.0]
        perfect = [0.0, 55.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0]
        curves = {
            **{("PA", float(value)): poor_kp for value in [5, 10, 20, 25, 30]},
            ("PA", 15.0): perfect,
            **{("PD", value): poor_kd for value in [0.3, 0.5, 1.0, 1.5, 2.0]},
            ("PD", 0.8): perfect,
            **{("PI", value): poor_ki for value in [0.0, 0.02, 0.05, 0.15, 0.2, 0.3]},
            ("PI", 0.1): perfect,
        }
        rig = FakeTuneRig(curves, duration=8.0)

        result = rig.runner().run_gear(1)

        self.assertIsInstance(result, GearTuneResult)
        self.assertTrue(result.completed)
        self.assertEqual((result.best_kp, result.best_kd, result.best_ki), (15, 0.8, 0.1))
        self.assertEqual([trial.setting for trial in result.kp_trials], [5, 10, 15, 20, 25, 30])
        self.assertEqual([trial.setting for trial in result.kd_trials], [0.3, 0.5, 0.8, 1.0, 1.5, 2.0])
        self.assertEqual([trial.setting for trial in result.ki_trials], [0.02, 0.05, 0.1, 0.15, 0.2, 0.3])
        self.assertIsNotNone(result.verification)
        self.assertEqual(result.verification.steady_state_error_deg, 0.0)

        pa_commands = [command for command in rig.commands if command.startswith("FOC,1,PA,")]
        pd_commands = [command for command in rig.commands if command.startswith("FOC,1,PD,")]
        pi_commands = [command for command in rig.commands if command.startswith("FOC,1,PI,")]
        self.assertEqual(
            pa_commands,
            ["FOC,1,PA,5", "FOC,1,PA,5", "FOC,1,PA,10", "FOC,1,PA,15",
             "FOC,1,PA,20", "FOC,1,PA,25", "FOC,1,PA,30", "FOC,1,PA,15",
             "FOC,1,PA,15"],
        )
        self.assertEqual(
            pd_commands,
            ["FOC,1,PD,0.5", "FOC,1,PD,0.3", "FOC,1,PD,0.5", "FOC,1,PD,0.8",
             "FOC,1,PD,1.0", "FOC,1,PD,1.5", "FOC,1,PD,2.0", "FOC,1,PD,0.8",
             "FOC,1,PD,0.8"],
        )
        self.assertEqual(
            pi_commands,
            ["FOC,1,PI,0", "FOC,1,PI,0.02", "FOC,1,PI,0.05", "FOC,1,PI,0.1",
             "FOC,1,PI,0.15", "FOC,1,PI,0.2", "FOC,1,PI,0.3", "FOC,1,PI,0.1",
             "FOC,1,PI,0.1"],
        )
        expected_delays = (
            [0.1, 0.1, 0.1, 0.1, 0.5, 0.3]
            + [0.3, 2.0, 8.0] * 6
            + [0.3]
            + [0.3, 2.0, 8.0] * 6
            + [0.3]
            + [0.3, 2.5, 8.0] * 6
            + [0.3]
            + [0.1, 0.1, 0.1, 1.0, 3.0, 8.0, 2.0]
        )
        self.assertEqual(rig.delays, expected_delays)
        self.assertEqual(rig.logs[0], "🤖 轴R GEAR 自动调参开始 (目标 ±60°)")
        self.assertIn("✅ 稳态误差 ≤ 1°，达标！", rig.logs)
        self.assertEqual(rig.logs[-1], "✅ 轴R 推荐：Kp=15  Kd=0.8  Ki=0.10")


if __name__ == "__main__":
    unittest.main()
