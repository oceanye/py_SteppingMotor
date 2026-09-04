import math
import unittest

from motor_control.axis_math import (
    GEAR_RATIO_MAX,
    GEAR_RATIO_MIN,
    LEAD_MM_MAX,
    LEAD_MM_MIN,
    MODE_LINEAR,
    MODE_ROTARY,
    PULSE_PER_REV_MAX,
    PULSE_PER_REV_MIN,
    coerce_finite_in_range,
    convert_axis_value,
    pulses_per_unit,
    speed_to_delay_ms,
    steps_to_units,
    units_to_steps,
)
from motor_control.axis_model import AxisProfile, AxisRuntime
from motor_control.topology import (
    NUM_STEPPER_AXES,
    clamp_step_delay_us,
    get_stepper_axis_topology,
    pico_axis_to_global,
    step_delay_ms_to_pulse_rate,
    step_speed_clamps_delay,
    step_speed_to_delay_ms,
    stepper_axis_topology,
)


class StepperTopologyTests(unittest.TestCase):
    def test_local_boundary_axes_match_esp32_pin_map(self):
        self.assertEqual(
            stepper_axis_topology(0),
            {
                "axis": 0,
                "label": "左侧直",  # 2026-08-24 起本地轴按功能命名
                "controller": "esp32",
                "node": None,
                "local_axis": 0,
                "pins": {"pulse": 5, "direction": 6},
            },
        )
        axis_5 = get_stepper_axis_topology(5)
        self.assertEqual(axis_5.label, "右开合")
        self.assertEqual((axis_5.pulse_pin, axis_5.direction_pin), (38, 39))

    def test_remote_boundary_axes_match_pico_mapping(self):
        axis_6 = get_stepper_axis_topology(6)
        self.assertEqual(
            (axis_6.controller, axis_6.node, axis_6.local_axis, axis_6.label),
            ("pico", 1, 0, "P1-1"),
        )
        axis_29 = get_stepper_axis_topology(29)
        self.assertEqual(
            (axis_29.controller, axis_29.node, axis_29.local_axis, axis_29.label),
            ("pico", 6, 3, "P6-4"),
        )
        self.assertIsNone(axis_29.as_legacy_dict()["pins"])
        self.assertEqual(pico_axis_to_global(1, 0), 6)
        self.assertEqual(pico_axis_to_global(6, 3), 29)

    def test_axis_30_and_other_invalid_axes_are_rejected(self):
        self.assertEqual(NUM_STEPPER_AXES, 30)
        for axis in (-1, 30):
            with self.subTest(axis=axis):
                with self.assertRaises(ValueError):
                    get_stepper_axis_topology(axis)
        with self.assertRaises(TypeError):
            get_stepper_axis_topology(0.0)

    def test_remote_axes_enforce_rp2040_minimum_delay(self):
        self.assertEqual(clamp_step_delay_us(5, 1), 1)
        self.assertEqual(clamp_step_delay_us(6, 1), 100)
        self.assertEqual(clamp_step_delay_us(29, 250), 250)

    def test_controller_specific_speed_timing(self):
        self.assertAlmostEqual(step_speed_to_delay_ms(0, 1000, 1), 0.8)
        self.assertAlmostEqual(step_delay_ms_to_pulse_rate(0, 0.8), 1000.0)
        self.assertAlmostEqual(step_speed_to_delay_ms(6, 1000, 1), 1.0)
        self.assertAlmostEqual(step_delay_ms_to_pulse_rate(6, 1.0), 1000.0)
        self.assertFalse(step_speed_clamps_delay(0, 4000, 1))
        self.assertTrue(step_speed_clamps_delay(0, 4001, 1))
        self.assertFalse(step_speed_clamps_delay(6, 10000, 1))
        self.assertTrue(step_speed_clamps_delay(6, 10001, 1))


class AxisMathTests(unittest.TestCase):
    def test_default_linear_and_rotary_pulses_per_unit(self):
        self.assertEqual(pulses_per_unit(MODE_LINEAR), 200.0)
        self.assertAlmostEqual(pulses_per_unit(MODE_ROTARY), 200.0 / 360.0)

    def test_gear_and_lead_are_applied_with_existing_formulas(self):
        self.assertEqual(pulses_per_unit(MODE_LINEAR, 400, 10, 5), 800.0)
        self.assertAlmostEqual(
            pulses_per_unit(MODE_ROTARY, 400, 10, 5), 4000.0 / 360.0
        )

    def test_mode_conversion_preserves_equivalent_pulses(self):
        linear_ppu = pulses_per_unit(MODE_LINEAR)
        rotary_ppu = pulses_per_unit(MODE_ROTARY)
        rotary_degrees = convert_axis_value(10.0, linear_ppu, rotary_ppu)
        self.assertAlmostEqual(rotary_degrees, 3600.0)
        self.assertAlmostEqual(rotary_degrees * rotary_ppu, 10.0 * linear_ppu)
        self.assertAlmostEqual(
            convert_axis_value(rotary_degrees, rotary_ppu, linear_ppu), 10.0
        )
        self.assertIsNone(convert_axis_value(None, linear_ppu, rotary_ppu))

    def test_step_unit_round_trip(self):
        for ppu in (200.0, 200.0 / 360.0, 1234.5):
            for steps in (-2000, -1, 0, 1, 98765):
                with self.subTest(ppu=ppu, steps=steps):
                    self.assertAlmostEqual(
                        steps_to_units(steps, ppu) * ppu,
                        float(steps),
                    )
        self.assertEqual(units_to_steps(1.234, 200), 247)

    def test_speed_to_delay_preserves_slow_fast_and_clamped_paths(self):
        self.assertAlmostEqual(speed_to_delay_ms(0.3, 200), 1000 / 60)
        self.assertAlmostEqual(speed_to_delay_ms(3.0, 200), 1.4666666666666666)
        self.assertEqual(speed_to_delay_ms(1000, 200), 0.05)

    def test_parameter_ranges_are_inclusive(self):
        for minimum, maximum, label in (
            (PULSE_PER_REV_MIN, PULSE_PER_REV_MAX, "pulse_per_rev"),
            (GEAR_RATIO_MIN, GEAR_RATIO_MAX, "gear_ratio"),
            (LEAD_MM_MIN, LEAD_MM_MAX, "lead_mm"),
        ):
            with self.subTest(label=label):
                self.assertEqual(
                    coerce_finite_in_range(minimum, minimum, maximum, label), minimum
                )
                self.assertEqual(
                    coerce_finite_in_range(maximum, minimum, maximum, label), maximum
                )

    def test_nan_infinity_and_out_of_range_parameters_are_rejected(self):
        invalid_values = (math.nan, math.inf, -math.inf, 0.999, 10_001, 10**10_000)
        for value in invalid_values:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    coerce_finite_in_range(value, 1.0, 10_000.0, "pulse_per_rev")
        for value in (0, -1, math.nan, math.inf):
            with self.subTest(speed=value):
                with self.assertRaises(ValueError):
                    speed_to_delay_ms(value, 200)
        for value in (0, -1, math.nan, math.inf):
            with self.subTest(ppu=value):
                with self.assertRaises(ValueError):
                    speed_to_delay_ms(3, value)


class AxisModelTests(unittest.TestCase):
    def test_profile_rejects_invalid_mode_and_physical_parameters(self):
        invalid_profiles = (
            {"mode": "LINEAR"},
            {"pulse_per_rev": math.nan},
            {"pulse_per_rev": 10_001},
            {"gear_ratio": math.inf},
            {"gear_ratio": 0.0009},
            {"lead_mm": 0},
            {"lead_mm": 101},
        )
        for kwargs in invalid_profiles:
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    AxisProfile(**kwargs)

    def test_runtime_keeps_canonical_steps_when_mode_changes(self):
        linear = AxisProfile(mode=MODE_LINEAR)
        rotary = linear.with_mode(MODE_ROTARY)
        runtime = AxisRuntime.from_legacy_units(
            linear,
            position=12.5,
            minimum=-1.0,
            maximum=20.0,
            trusted=True,
        )
        before = (runtime.position_steps, runtime.min_steps, runtime.max_steps)
        self.assertEqual(before, (2500.0, -200.0, 4000.0))
        self.assertAlmostEqual(runtime.position_in(rotary), 4500.0)
        self.assertEqual((runtime.position_steps, runtime.min_steps, runtime.max_steps), before)
        self.assertAlmostEqual(runtime.position_in(linear), 12.5)

    def test_legacy_calibration_round_trip_preserves_values(self):
        profile = AxisProfile(
            mode=MODE_ROTARY, pulse_per_rev=400, gear_ratio=3, lead_mm=2
        )
        runtime = AxisRuntime.from_legacy_units(
            profile,
            position=12.34,
            minimum=-90.0,
            maximum=180.0,
            trusted=True,
        )
        restored = runtime.as_legacy_units(profile)
        self.assertAlmostEqual(restored["position"], 12.34)
        self.assertAlmostEqual(restored["min"], -90.0)
        self.assertAlmostEqual(restored["max"], 180.0)
        self.assertIs(restored["trusted"], True)

    def test_runtime_limit_checks_use_steps(self):
        profile = AxisProfile()
        runtime = AxisRuntime.from_legacy_units(
            profile, position=5, minimum=0, maximum=10, trusted=True
        )
        self.assertTrue(runtime.target_is_within_limits(runtime.target_steps(profile, 5)))
        self.assertFalse(runtime.target_is_within_limits(runtime.target_steps(profile, 5.01)))
        self.assertTrue(
            runtime.target_is_within_limits(
                runtime.target_steps(profile, 5.01), tolerance_steps=2
            )
        )


if __name__ == "__main__":
    unittest.main()
