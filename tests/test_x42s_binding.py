"""绑定 schema v2 / X42S 执行器模型测试（feature/x42s-uart-actuator-binding）.

对应《REQUIREMENTS_2026-09-28_X42S_UART_ACTUATOR_BINDING》§17.1：
- v2 文档（backend=x42s_uart，kind=x42s + driver_id）严格解析与 round trip；
- driver_id 校验（0=广播拒绑、>255 拒、非整数拒）与角色/ID 双唯一；
- v1→v2 迁移仅在显式确认时生成建议映射；
- v1 文档与 stepper 路径不回归；x42s 绑定不借用本地轴校验；
- actuator_model 遥测新鲜度/offline/地址冲突投影。
"""
import json
import unittest

from motor_control import (
    BACKEND_STEPDIR,
    BACKEND_X42S_UART,
    BINDING_SCHEMA_VERSION,
    ActuatorBinding,
    AxisProfile,
    BindingSet,
    BindingValidationError,
    LogicalRole,
    SUGGESTED_X42S_ID_BY_ROLE,
    binding_compatibility_issues,
    build_coordinated_snapshot,
    parse_binding_document,
    propose_x42s_migration,
)
from motor_control.actuator_model import (
    KNOWN_X42S_FAULTS,
    TELEMETRY_FRESH_MAX_AGE_MS,
    TELEMETRY_STALE_AGE_MS,
    X42sActuatorState,
    x42s_state_issues,
)


class X42sBindingSchemaTests(unittest.TestCase):
    def test_suggested_x42s_round_trip(self):
        bindings = BindingSet.suggested_x42s()
        self.assertEqual(bindings.backend, BACKEND_X42S_UART)
        self.assertTrue(bindings.is_complete)
        document = bindings.as_document()
        self.assertEqual(document["schema_version"], BINDING_SCHEMA_VERSION)
        self.assertEqual(document["backend"], "x42s_uart")
        # gear_ratio（2026-10-10 实机 1:21.5）随绑定持久化；缺省即默认值。
        self.assertEqual(bindings.for_role(LogicalRole.MR1).gear_ratio, 21.5)
        self.assertEqual(
            document["bindings"]["Mr1"],
            {"kind": "x42s", "driver_id": 2, "gear_ratio": 21.5},
        )
        self.assertEqual(
            json.loads(json.dumps(document)), document  # JSON 可序列化
        )
        self.assertEqual(parse_binding_document(document), bindings)

    def test_partial_x42s_set_round_trip(self):
        values = {"Mup1": 1, "Mr1": None, "Mup2": None, "Mr2": 255}
        bindings = BindingSet.from_driver_id_mapping(values, revision=7)
        self.assertFalse(bindings.is_complete)
        parsed = parse_binding_document(bindings.as_document())
        self.assertEqual(parsed, bindings)
        self.assertEqual(parsed.revision, 7)

    def test_gear_ratio_default_and_round_trip(self):
        # 2026-10-10 实机确认 1:21.5 减速：缺省=DEFAULT，显式值往返不丢。
        self.assertEqual(
            ActuatorBinding(kind="x42s", driver_id=2).gear_ratio, 21.5
        )
        bindings = BindingSet.from_driver_id_mapping(
            {"Mup1": 1, "Mr1": 2, "Mup2": 3, "Mr2": 4},
            gear_ratios={"Mr1": 10.0},
        )
        self.assertEqual(bindings.for_role(LogicalRole.MR1).gear_ratio, 10.0)
        self.assertEqual(bindings.for_role(LogicalRole.MUP1).gear_ratio, 21.5)
        self.assertEqual(
            parse_binding_document(bindings.as_document()), bindings
        )

    def test_gear_ratio_legacy_document_defaults(self):
        # 旧 v2 文档无 gear_ratio 字段：解析后取默认 21.5（字段级向后兼容）。
        document = {
            "schema_version": 2,
            "revision": 3,
            "backend": "x42s_uart",
            "bindings": {
                "Mup1": {"kind": "x42s", "driver_id": 1},
                "Mr1": {"kind": "x42s", "driver_id": 2},
                "Mup2": {"kind": "x42s", "driver_id": 3},
                "Mr2": {"kind": "x42s", "driver_id": 4},
            },
        }
        parsed = parse_binding_document(document)
        for role in (LogicalRole.MUP1, LogicalRole.MR1,
                     LogicalRole.MUP2, LogicalRole.MR2):
            self.assertEqual(parsed.for_role(role).gear_ratio, 21.5)

    def test_gear_ratio_rejects_invalid_values(self):
        for bad in (0, -1, 0.0, 1001.0, float("inf"), float("nan"), True, "21.5"):
            with self.assertRaises(BindingValidationError) as ctx:
                ActuatorBinding(kind="x42s", driver_id=2, gear_ratio=bad)
            self.assertEqual(ctx.exception.code, "invalid_gear_ratio")
        # stepper 绑定不得携带减速比（字段语义按后端严格区分）。
        with self.assertRaises(BindingValidationError) as ctx:
            ActuatorBinding(axis=0, gear_ratio=21.5)
        self.assertEqual(ctx.exception.code, "binding_fields")

    def test_driver_id_boundaries(self):
        self.assertEqual(ActuatorBinding(kind="x42s", driver_id=1).driver_id, 1)
        self.assertEqual(ActuatorBinding(kind="x42s", driver_id=255).driver_id, 255)
        for bad in (0, 256, -1, True, 1.5, "1", None):
            with self.assertRaises(BindingValidationError) as ctx:
                ActuatorBinding(kind="x42s", driver_id=bad)
            self.assertEqual(ctx.exception.code, "invalid_driver_id")

    def test_kind_fields_are_exclusive(self):
        with self.assertRaises(BindingValidationError) as ctx:
            ActuatorBinding(kind="x42s", driver_id=2, axis=1)
        self.assertEqual(ctx.exception.code, "binding_fields")
        with self.assertRaises(BindingValidationError) as ctx:
            ActuatorBinding(axis=1, driver_id=2)  # stepper 不得携带 driver_id
        self.assertEqual(ctx.exception.code, "binding_fields")
        with self.assertRaises(BindingValidationError):
            ActuatorBinding(kind="servo")

    def test_duplicate_driver_id_rejected(self):
        with self.assertRaises(BindingValidationError) as ctx:
            BindingSet.from_driver_id_mapping(
                {"Mup1": 2, "Mr1": 2, "Mup2": 3, "Mr2": 4}
            )
        self.assertEqual(ctx.exception.code, "duplicate_driver_id")

    def test_backend_kind_mismatch_rejected(self):
        mixed = (ActuatorBinding(axis=0), None, None, None)
        with self.assertRaises(BindingValidationError) as ctx:
            BindingSet(mixed, backend=BACKEND_X42S_UART)
        self.assertEqual(ctx.exception.code, "backend_kind_mismatch")
        with self.assertRaises(BindingValidationError) as ctx:
            BindingSet(
                (ActuatorBinding(kind="x42s", driver_id=1), None, None, None),
                backend=BACKEND_STEPDIR,
            )
        self.assertEqual(ctx.exception.code, "backend_kind_mismatch")

    def test_unknown_backend_rejected(self):
        with self.assertRaises(BindingValidationError) as ctx:
            BindingSet(backend="canopen")
        self.assertEqual(ctx.exception.code, "unsupported_backend")

    def test_role_and_id_queries(self):
        bindings = BindingSet.suggested_x42s()
        self.assertEqual(bindings.bound_driver_ids, (1, 2, 3, 4))
        self.assertEqual(bindings.role_for_driver_id(3), LogicalRole.MUP2)
        self.assertIsNone(bindings.role_for_driver_id(9))
        with self.assertRaises(BindingValidationError):
            bindings.role_for_driver_id(0)
        self.assertEqual(bindings.as_axis_mapping(), {
            "Mup1": None, "Mr1": None, "Mup2": None, "Mr2": None,
        })
        self.assertEqual(
            bindings.as_driver_id_mapping(), {"Mup1": 1, "Mr1": 2, "Mup2": 3, "Mr2": 4}
        )


class X42sDocumentValidationTests(unittest.TestCase):
    def base_document(self):
        return {
            "schema_version": 2,
            "revision": 3,
            "backend": "x42s_uart",
            "bindings": {
                "Mup1": {"kind": "x42s", "driver_id": 1},
                "Mr1": {"kind": "x42s", "driver_id": 2},
                "Mup2": {"kind": "x42s", "driver_id": 3},
                "Mr2": {"kind": "x42s", "driver_id": 4},
            },
        }

    def assert_rejected(self, document, code):
        with self.assertRaises(BindingValidationError) as ctx:
            parse_binding_document(document)
        self.assertEqual(ctx.exception.code, code)

    def test_valid_document_parses(self):
        bindings = parse_binding_document(self.base_document())
        self.assertEqual(bindings, BindingSet.suggested_x42s().with_revision(3))

    def test_v2_missing_backend_key_rejected(self):
        document = self.base_document()
        del document["backend"]
        self.assert_rejected(document, "document_fields")

    def test_unknown_backend_value_rejected(self):
        document = self.base_document()
        document["backend"] = "rs485"
        self.assert_rejected(document, "unsupported_backend")

    def test_unknown_schema_version_rejected(self):
        document = self.base_document()
        document["schema_version"] = 3
        self.assert_rejected(document, "schema_version")

    def test_stepper_binding_in_x42s_document_rejected(self):
        document = self.base_document()
        document["bindings"]["Mr1"] = {"kind": "stepper", "axis": 2}
        self.assert_rejected(document, "unsupported_kind")

    def test_axis_field_in_x42s_binding_rejected(self):
        document = self.base_document()
        document["bindings"]["Mr1"] = {
            "kind": "x42s", "driver_id": 2, "axis": 2,
        }
        self.assert_rejected(document, "binding_fields")

    def test_driver_id_zero_in_document_rejected(self):
        document = self.base_document()
        document["bindings"]["Mup1"]["driver_id"] = 0
        self.assert_rejected(document, "invalid_driver_id")

    def test_driver_id_over_max_in_document_rejected(self):
        document = self.base_document()
        document["bindings"]["Mup1"]["driver_id"] = 256
        self.assert_rejected(document, "invalid_driver_id")

    def test_duplicate_ids_in_document_rejected(self):
        document = self.base_document()
        document["bindings"]["Mr1"]["driver_id"] = 1
        self.assert_rejected(document, "duplicate_driver_id")

    def test_missing_role_rejected(self):
        document = self.base_document()
        del document["bindings"]["Mup2"]
        self.assert_rejected(document, "role_set")

    def test_v2_stepdir_document_parses(self):
        document = self.base_document()
        document["backend"] = "stepdir"
        document["bindings"] = {
            "Mup1": {"kind": "stepper", "axis": 0},
            "Mr1": {"kind": "stepper", "axis": 2},
            "Mup2": {"kind": "stepper", "axis": 1},
            "Mr2": {"kind": "stepper", "axis": 3},
        }
        bindings = parse_binding_document(document)
        self.assertEqual(bindings.backend, BACKEND_STEPDIR)
        self.assertEqual(bindings, BindingSet.suggested().with_revision(3))


class V1CompatAndMigrationTests(unittest.TestCase):
    def test_v1_document_still_parses(self):
        document = {
            "schema_version": 1,
            "revision": 5,
            "bindings": {
                "Mup1": {"kind": "stepper", "axis": 0},
                "Mr1": {"kind": "stepper", "axis": 2},
                "Mup2": {"kind": "stepper", "axis": 1},
                "Mr2": None,
            },
        }
        bindings = parse_binding_document(document)
        self.assertEqual(bindings.backend, BACKEND_STEPDIR)
        self.assertFalse(bindings.is_complete)
        self.assertEqual(bindings.for_role("Mr2"), None)
        self.assertEqual(bindings.for_role("Mup1").axis, 0)

    def test_v1_write_shape_unchanged(self):
        # 既有 .stepper 绑定文件格式必须字节级稳定：stepdir 写出仍为 v1。
        bindings = BindingSet.suggested().with_revision(11)
        document = bindings.as_document()
        self.assertEqual(set(document), {"schema_version", "revision", "bindings"})
        self.assertEqual(document["schema_version"], 1)
        self.assertEqual(
            document["bindings"]["Mr1"], {"kind": "stepper", "axis": 2}
        )
        self.assertEqual(parse_binding_document(document), bindings)

    def test_migration_requires_explicit_confirmation(self):
        with self.assertRaises(BindingValidationError) as ctx:
            propose_x42s_migration(BindingSet.suggested(), confirmed=False)
        self.assertEqual(ctx.exception.code, "migration_unconfirmed")

    def test_migration_requires_complete_v1(self):
        incomplete = BindingSet.from_axis_mapping(
            {"Mup1": 0, "Mr1": 2, "Mup2": 1, "Mr2": None}, revision=2
        )
        with self.assertRaises(BindingValidationError) as ctx:
            propose_x42s_migration(incomplete, confirmed=True)
        self.assertEqual(ctx.exception.code, "migration_incomplete")
        with self.assertRaises(BindingValidationError):
            propose_x42s_migration(BindingSet.suggested_x42s(), confirmed=True)

    def test_migration_uses_suggested_role_mapping(self):
        bindings = BindingSet.suggested().with_revision(9)
        migrated = propose_x42s_migration(bindings, confirmed=True)
        self.assertEqual(migrated.backend, BACKEND_X42S_UART)
        self.assertEqual(migrated.revision, 9)
        self.assertEqual(migrated, BindingSet.suggested_x42s().with_revision(9))
        self.assertEqual(
            {role.value: migrated.for_role(role).driver_id
             for role in SUGGESTED_X42S_ID_BY_ROLE},
            {"Mup1": 1, "Mr1": 2, "Mup2": 3, "Mr2": 4},
        )


class CompatibilityAndSnapshotTests(unittest.TestCase):
    def test_x42s_bindings_skip_local_axis_checks(self):
        # x42s 绑定无本地轴 profile：即使 profiles 全是错误模式也不得报
        # mode_mismatch（其可用性由 X42S 后端遥测提供）。
        profiles = [AxisProfile() for _ in range(6)]  # 默认 linear
        valid = [True] * 6
        issues = binding_compatibility_issues(
            BindingSet.suggested_x42s(), profiles, valid
        )
        self.assertEqual(issues, ())

    def test_stepper_mode_mismatch_still_reported(self):
        # 旧 stepper 路径回归：Mr 角色绑到直线模式轴仍必须报 mode_mismatch。
        profiles = [AxisProfile() for _ in range(6)]
        issues = binding_compatibility_issues(
            BindingSet.suggested(), profiles, [True] * 6
        )
        codes = [issue.code for issue in issues]
        self.assertIn("mode_mismatch", codes)

    def test_snapshot_with_x42s_backend_is_honest(self):
        # 本轮无 X42S 运行时：快照必须如实投影 X42S_BACKEND_PENDING，
        # 不借用步进轴数据、不伪造位置。
        control, motors = build_coordinated_snapshot(
            bindings=BindingSet.suggested_x42s(),
            profiles=[object()] * 6,
            runtimes=[object()] * 6,
            axis_param_valid=[True] * 6,
            running=[False] * 6,
            stepper_in_progress=[False] * 6,
            move_dispatching=[False] * 6,
            move_reservations=[None] * 6,
            web_step_pending=[None] * 6,
            pending_steps=[None] * 6,
            telemetry=[object()] * 6,
            connected=False,
        )
        self.assertEqual(control["backend"], "x42s_uart")
        self.assertEqual(len(motors), 4)
        for motor in motors:
            self.assertTrue(motor["bound"])
            self.assertEqual(motor["state"], "X42S_BACKEND_PENDING")
            self.assertIn("X42S ID", motor["physical_label"])
            self.assertIsNone(motor["position"])
            self.assertFalse(motor["position_trusted"])
            self.assertTrue(motor["stale"])
            self.assertNotEqual(motor["position_source"], "host_pulse_accounting")

    def test_snapshot_with_stepper_backend_unchanged(self):
        control, motors = build_coordinated_snapshot(
            bindings=BindingSet.empty(),
            profiles=[object()] * 6,
            runtimes=[object()] * 6,
            axis_param_valid=[True] * 6,
            running=[False] * 6,
            stepper_in_progress=[False] * 6,
            move_dispatching=[False] * 6,
            move_reservations=[None] * 6,
            web_step_pending=[None] * 6,
            pending_steps=[None] * 6,
            telemetry=[object()] * 6,
            connected=False,
        )
        self.assertEqual(control["backend"], "stepdir")
        self.assertTrue(all(motor["state"] == "UNBOUND" for motor in motors))


class X42sActuatorStateTests(unittest.TestCase):
    def test_offline_state_carries_no_positions(self):
        state = X42sActuatorState.offline(LogicalRole.MUP1, 1)
        self.assertFalse(state.online)
        self.assertIsNone(state.measured_position)
        self.assertFalse(state.telemetry_trusted_at(now=100.0))
        snapshot = state.as_snapshot_dict(now=100.0)
        self.assertEqual(snapshot["label"], "X42S ID1")
        self.assertTrue(snapshot["stale"])
        self.assertFalse(snapshot["telemetry_trusted"])

    def test_fresh_telemetry_is_trusted_then_goes_stale(self):
        state = X42sActuatorState.offline(LogicalRole.MR2, 4).with_telemetry(
            now=10.0,
            enabled=True,
            command_position=1.5,
            measured_position=1.4,
            position_error=-0.1,
            reached=False,
        )
        self.assertTrue(state.online)
        self.assertTrue(state.telemetry_trusted_at(now=10.0 + TELEMETRY_FRESH_MAX_AGE_MS / 1000))
        self.assertFalse(
            state.telemetry_trusted_at(now=10.0 + TELEMETRY_FRESH_MAX_AGE_MS / 1000 + 0.001)
        )
        snapshot = state.as_snapshot_dict(now=10.0 + TELEMETRY_STALE_AGE_MS / 1000 + 0.001)
        self.assertTrue(snapshot["stale"])

    def test_none_positions_survive_round_trip(self):
        # 未知字段必须保持 None，显示层不得把未知画成 0。
        state = X42sActuatorState.offline(LogicalRole.MUP2, 3).with_telemetry(
            now=0.0, measured_position=None, command_position=None
        )
        snapshot = state.as_snapshot_dict(now=0.5)
        self.assertIsNone(snapshot["measured_position"])
        self.assertIsNone(snapshot["command_position"])

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):
            X42sActuatorState(role=LogicalRole.MUP1, driver_id=0)
        with self.assertRaises(ValueError):
            X42sActuatorState(role=LogicalRole.MUP1, driver_id=256)
        with self.assertRaises(ValueError):
            X42sActuatorState(role="Mup1", driver_id=1)  # 必须是 LogicalRole
        with self.assertRaises(ValueError):
            X42sActuatorState(role=LogicalRole.MUP1, driver_id=1, fault="exploded")

    def test_state_issues_report_offline_and_duplicate_id(self):
        good = X42sActuatorState.offline(LogicalRole.MUP1, 1).with_telemetry(now=0.0)
        offline = X42sActuatorState.offline(LogicalRole.MR1, 2)
        conflict = X42sActuatorState.offline(LogicalRole.MUP2, 1)
        issues = x42s_state_issues([good, offline, conflict])
        self.assertTrue(any("offline" in line for line in issues))
        self.assertTrue(any("地址冲突" in line for line in issues))
        self.assertEqual(x42s_state_issues([good]), ())
        self.assertTrue(set(KNOWN_X42S_FAULTS))


if __name__ == "__main__":
    unittest.main()
