"""Pulse-rate safety guard: pure-function coverage for the speed hardening.

情景对应现场问题（2026-08-20）：
- 轴L（ppr=200, gr=21.5 减速箱）误用直线模式：最慢速度档 0.3 mm/s 即折合
  1290 pps，必须被护栏抓住；
- 轴R（ppr=200, gr=1 丝杆直连）常用 1/3 mm/s 档不得误报；
- 请求速度超过固件延时下限（0.05 ms + 200 us 补偿 = 4000 pps）时被静默钳位，
  speed_clamps_delay 必须与 speed_to_delay_ms 的钳位行为一致。
"""

import unittest

from motor_control.axis_math import (
    MIN_DELAY_MS,
    PULSE_RATE_WARN_PPS,
    delay_ms_to_pulse_rate,
    pulses_per_unit,
    speed_clamps_delay,
    speed_to_delay_ms,
)


class DelayPulseRateTests(unittest.TestCase):
    def test_basic_conversion(self):
        self.assertAlmostEqual(delay_ms_to_pulse_rate(1.0), 1000.0)
        self.assertAlmostEqual(delay_ms_to_pulse_rate(0.05), 20000.0)

    def test_rejects_non_positive(self):
        for bad in (0, -1, "x", None):
            with self.assertRaises(ValueError):
                delay_ms_to_pulse_rate(bad)


class SpeedClampTests(unittest.TestCase):
    def test_boundary_at_4000_pps(self):
        # 1e6 / (50 + 200) = 4000 pps：恰好达阈值不算钳位，超过即钳位
        self.assertFalse(speed_clamps_delay(4000.0, 1.0))
        self.assertTrue(speed_clamps_delay(4001.0, 1.0))

    def test_matches_speed_to_delay_clamp(self):
        for pps in (100.0, 600.0, 1000.0, 3999.0, 4000.0, 4001.0,
                    12900.0, 20000.0):
            with self.subTest(pps=pps):
                clamped = speed_clamps_delay(pps, 1.0)
                delay = speed_to_delay_ms(pps, 1.0)
                if clamped:
                    self.assertAlmostEqual(delay, MIN_DELAY_MS)
                else:
                    self.assertGreaterEqual(delay, MIN_DELAY_MS)


class FieldScenarioTests(unittest.TestCase):
    def test_geared_axis_linear_slowest_preset_is_caught(self):
        ppu = pulses_per_unit("linear", 200.0, 21.5, 1.0)
        self.assertAlmostEqual(ppu, 4300.0)
        delay = speed_to_delay_ms(0.3, ppu)
        self.assertGreater(
            delay_ms_to_pulse_rate(delay), PULSE_RATE_WARN_PPS)

    def test_lead_screw_common_presets_stay_below_warn(self):
        ppu = pulses_per_unit("linear", 200.0, 1.0, 1.0)
        for speed in (1.0, 3.0):
            with self.subTest(speed=speed):
                delay = speed_to_delay_ms(speed, ppu)
                self.assertLessEqual(
                    delay_ms_to_pulse_rate(delay), PULSE_RATE_WARN_PPS)

    def test_geared_axis_rotary_default_is_safe(self):
        ppu = pulses_per_unit("rotary", 200.0, 21.5)
        delay = speed_to_delay_ms(3.0, ppu)
        self.assertLessEqual(
            delay_ms_to_pulse_rate(delay), PULSE_RATE_WARN_PPS)

    def test_geared_axis_linear_fast_preset_reports_clamp(self):
        # 3 mm/s × 4300 ppu/mm = 12900 pps，远超 4000 pps 固件上限 → 钳位
        ppu = pulses_per_unit("linear", 200.0, 21.5, 1.0)
        self.assertTrue(speed_clamps_delay(3.0, ppu))


if __name__ == "__main__":
    unittest.main()
