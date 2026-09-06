import unittest

from motor_control.axis_math import MODE_ROTARY
from motor_control.axis_model import AxisProfile, AxisRuntime
from motor_control.coordinated_control import (
    AxisMotionTelemetry,
    BindingSet,
    BindingValidationError,
    LOGICAL_ROLE_ORDER,
    LogicalRole,
    POSITION_SOURCE,
    binding_compatibility_issues,
    build_coordinated_snapshot,
    parse_binding_document,
    projected_step_position,
)
from motor_control.topology import NUM_STEPPER_AXES


def _axis_state():
    return {
        "profiles": [AxisProfile() for _ in range(NUM_STEPPER_AXES)],
        "runtimes": [AxisRuntime() for _ in range(NUM_STEPPER_AXES)],
        "axis_param_valid": [True] * NUM_STEPPER_AXES,
        "running": [False] * NUM_STEPPER_AXES,
        "stepper_in_progress": [False] * NUM_STEPPER_AXES,
        "move_dispatching": [False] * NUM_STEPPER_AXES,
        "move_reservations": [None] * NUM_STEPPER_AXES,
        "web_step_pending": [None] * NUM_STEPPER_AXES,
        "pending_steps": [None] * NUM_STEPPER_AXES,
        "telemetry": [AxisMotionTelemetry() for _ in range(NUM_STEPPER_AXES)],
    }


class BindingModelTests(unittest.TestCase):
    def test_role_order_and_fail_safe_default_are_stable(self):
        bindings = BindingSet.empty()

        self.assertEqual(
            [role.value for role in LOGICAL_ROLE_ORDER],
            ["Mup1", "Mr1", "Mup2", "Mr2"],
        )
        self.assertEqual(
            bindings.as_axis_mapping(),
            {"Mup1": None, "Mr1": None, "Mup2": None, "Mr2": None},
        )
        self.assertFalse(bindings.is_complete)

    def test_suggested_mapping_is_not_the_default(self):
        self.assertEqual(
            BindingSet.suggested().as_axis_mapping(),
            {"Mup1": 0, "Mr1": 2, "Mup2": 1, "Mr2": 3},
        )
        self.assertNotEqual(BindingSet.suggested(), BindingSet.empty())

    def test_document_round_trip_preserves_revision_and_null(self):
        bindings = BindingSet.from_axis_mapping(
            {"Mup1": 0, "Mr1": 2, "Mup2": None, "Mr2": 3}, revision=8
        )

        self.assertEqual(parse_binding_document(bindings.as_document()), bindings)

    def test_role_set_must_be_exact(self):
        with self.assertRaisesRegex(BindingValidationError, "角色集合"):
            BindingSet.from_axis_mapping(
                {"Mup1": 0, "Mr1": 2, "Mup2": 1, "unexpected": 3}
            )

    def test_axis_type_range_and_uniqueness_are_strict(self):
        base = {"Mup1": 0, "Mr1": 2, "Mup2": 1, "Mr2": 3}
        for invalid in (True, 1.0, "1", -1, NUM_STEPPER_AXES):
            candidate = dict(base, Mup1=invalid)
            with self.subTest(axis=invalid), self.assertRaises(
                BindingValidationError
            ):
                BindingSet.from_axis_mapping(candidate)
        with self.assertRaisesRegex(BindingValidationError, "不能绑定多个"):
            BindingSet.from_axis_mapping(dict(base, Mr2=0))

    def test_direct_binding_set_construction_rejects_wrong_assignment_type(self):
        with self.assertRaisesRegex(BindingValidationError, "步进轴绑定或 null"):
            BindingSet(assignments=(0, None, None, None))

    def test_parser_rejects_unknown_schema_kind_and_fields(self):
        valid = BindingSet.suggested().as_document()
        for mutate in (
            lambda value: value.update(schema_version=2),
            lambda value: value["bindings"]["Mup1"].update(kind="gear"),
            lambda value: value["bindings"]["Mup1"].update(extra=True),
            lambda value: value.update(extra=True),
        ):
            candidate = {
                "schema_version": valid["schema_version"],
                "revision": valid["revision"],
                "bindings": {
                    role: None if item is None else dict(item)
                    for role, item in valid["bindings"].items()
                },
            }
            mutate(candidate)
            with self.assertRaises(BindingValidationError):
                parse_binding_document(candidate)

    def test_mode_and_parameter_compatibility_are_reported_per_role(self):
        bindings = BindingSet.suggested()
        profiles = [AxisProfile() for _ in range(NUM_STEPPER_AXES)]
        profiles[2] = AxisProfile(mode=MODE_ROTARY)
        profiles[3] = AxisProfile(mode=MODE_ROTARY)
        valid = [True] * NUM_STEPPER_AXES

        self.assertEqual(binding_compatibility_issues(bindings, profiles, valid), ())
        profiles[2] = AxisProfile()
        valid[1] = False
        issues = binding_compatibility_issues(bindings, profiles, valid)
        self.assertEqual(
            {(issue.role, issue.code) for issue in issues},
            {
                (LogicalRole.MR1, "mode_mismatch"),
                (LogicalRole.MUP2, "axis_params_invalid"),
            },
        )


class MotionProjectionTests(unittest.TestCase):
    def test_projection_uses_real_progress_without_mutating_committed_value(self):
        telemetry = AxisMotionTelemetry.starting(3, 200, now=10.0)
        telemetry = telemetry.with_progress(50, 200, now=10.25)

        committed = 1_000.0
        self.assertEqual(projected_step_position(committed, -200, telemetry), 950)
        self.assertEqual(committed, 1_000.0)

    def test_projection_is_absent_until_progress_frame_arrives(self):
        telemetry = AxisMotionTelemetry.starting(0, 200, now=10.0)
        self.assertIsNone(projected_step_position(1_000, 200, telemetry))

    def test_mismatched_or_impossible_progress_is_ignored(self):
        telemetry = AxisMotionTelemetry.starting(0, 200, now=10.0)
        self.assertEqual(telemetry.with_progress(10, 100), telemetry)
        self.assertEqual(telemetry.with_progress(201, 200), telemetry)

    def test_terminal_result_clears_transient_progress(self):
        telemetry = AxisMotionTelemetry.starting(0, 200, now=10.0).with_progress(
            50, 200, now=10.25
        )

        terminal = telemetry.terminal("DONE", 200, 200, now=10.5)

        self.assertEqual(terminal.state, "IDLE")
        self.assertEqual(terminal.last_result, "DONE")
        self.assertIsNone(terminal.executed_steps)
        self.assertIsNone(terminal.requested_steps)

    def test_snapshot_reports_units_truth_and_automation_blockers(self):
        state = _axis_state()
        state["profiles"][2] = AxisProfile(mode=MODE_ROTARY)
        state["profiles"][3] = AxisProfile(mode=MODE_ROTARY)
        state["runtimes"][0] = AxisRuntime(
            position_steps=1_000, min_steps=0, max_steps=2_000, position_trusted=True
        )
        state["pending_steps"][0] = 200
        state["stepper_in_progress"][0] = True
        state["telemetry"][0] = AxisMotionTelemetry.starting(
            0, 200, now=10.0
        ).with_progress(50, 200, now=10.25)

        control, motors = build_coordinated_snapshot(
            bindings=BindingSet.suggested().with_revision(4),
            connected=True,
            pico_node_health={},
            now=10.5,
            **state,
        )

        mup1 = motors[0]
        self.assertTrue(control["configuration_valid"])
        self.assertFalse(control["automation_ready"])
        self.assertIn("pico_node_health_incomplete", control["blockers"])
        self.assertEqual(mup1["state"], "MOVING")
        self.assertEqual(mup1["unit"], "mm")
        self.assertEqual(mup1["committed_position"], 5.0)
        self.assertEqual(mup1["projected_position"], 5.25)
        self.assertEqual(mup1["target_position"], 6.0)
        self.assertEqual(mup1["progress_percent"], 25.0)
        self.assertEqual(mup1["position_source"], POSITION_SOURCE)
        self.assertFalse(mup1["measured"])
        self.assertEqual(motors[1]["unit"], "°")

    def test_unbound_and_remote_unknown_states_are_explicit(self):
        state = _axis_state()
        empty_control, empty_motors = build_coordinated_snapshot(
            bindings=BindingSet.empty(), connected=True, **state
        )
        self.assertFalse(empty_control["configuration_valid"])
        self.assertTrue(all(item["state"] == "UNBOUND" for item in empty_motors))

        remote = BindingSet.from_axis_mapping(
            {"Mup1": 6, "Mr1": None, "Mup2": None, "Mr2": None}
        )
        _control, motors = build_coordinated_snapshot(
            bindings=remote, connected=True, pico_node_health={1: "unknown"}, **state
        )
        self.assertEqual(motors[0]["state"], "NODE_UNKNOWN")


if __name__ == "__main__":
    unittest.main()
