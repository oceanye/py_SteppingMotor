"""2026-09-28 电机绑定页：逐电机调试与方向标定（真实方向 vs 驱动方向）。

- 点动按"轴坐标 +/-"映射 DIR（与步态执行器同一定义），只动该角色
  绑定的轴；未绑定/串口未连接拦截；
- 记录符号：真实=约定正方向 ⇒ +1，否则 −1；变更立即保存并使步态标定
  失效，Mr 符号变更令该侧零位签名失配（执行前必须重新记零）；
- 符号未变不动参数（不打翻已确认的标定）；步态执行期间禁止改符号。
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # 裸名互导
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

from test_desktop_axis_integration import _headless_app
from motor_control import BindingSet
from motor_control.desktop_app import DIR_INWARD, DIR_OUTWARD
from motor_control.gait_executor import GaitExecutorError


class _SignVar:
    def __init__(self):
        self.value = None

    def set(self, value):
        self.value = value


def calibrated_app():
    """绑定建议映射 + 可用 state_store 的最小应用（复用联动测试的夹具）。"""
    from test_gait_linkage import mechanism

    app = mechanism()
    saved = []
    app.state_store = SimpleNamespace(
        save_gait_params=lambda doc: saved.append(doc))
    app.saved_docs = saved
    app.gait_sign_vars = {}
    return app


class RoleDebugMoveTests(unittest.TestCase):
    def test_axis_positive_maps_to_direction_per_axis_convention(self):
        # 轴坐标 "+" ⇒ DIR：旋转轴(左侧转/右侧转) OUTWARD；直线轴
        # (左侧直/右侧直) INWARD（2026-09-07 用户约定：向上为正）。
        app = calibrated_app()
        calls = []
        app._quick_move = lambda axis, amount, direction: calls.append(
            (axis, amount, direction))
        app._role_debug_move("Mr1", True)
        app._role_debug_move("Mr1", False)
        app._role_debug_move("Mr2", True)
        app._role_debug_move("Mup1", True)
        app._role_debug_move("Mup2", False)
        self.assertEqual(calls, [
            (2, 10.0, DIR_OUTWARD), (2, 10.0, DIR_INWARD),
            (3, 10.0, DIR_OUTWARD),
            (0, 1.0, DIR_INWARD), (1, 1.0, DIR_OUTWARD),
        ])

    def test_unbound_role_is_rejected_with_reason(self):
        app = calibrated_app()
        app.control_bindings = BindingSet.empty()
        quick = []
        app._quick_move = lambda *args: quick.append(args)
        app._role_debug_move("Mr1", True)
        self.assertFalse(quick)
        self.assertTrue(any("未绑定" in line for line in app._logs))

    def test_serial_disconnected_blocks_jog(self):
        app = calibrated_app()
        quick = []
        app._quick_move = lambda *args: quick.append(args)
        app._is_serial_connected = lambda: False
        app._role_debug_move("Mup1", True)
        self.assertFalse(quick)
        self.assertTrue(any("串口" in line for line in app._logs))


class RecordDirectionSignTests(unittest.TestCase):
    def test_flip_persists_sign_invalidates_calibration_and_zero(self):
        app = calibrated_app()
        var = _SignVar()
        app.gait_sign_vars = {"mr2_sign": var}
        app._gait_record_direction_sign("Mr2", False)   # 实际俯视顺时针 ⇒ −1
        self.assertEqual(app.gait_params.mr2_sign, -1)
        self.assertFalse(app.gait_params.calibration_confirmed)
        self.assertEqual(app.saved_docs[-1]["mr2_sign"], -1)
        self.assertEqual(var.value, "-1")
        # 零位签名随符号失配：执行前必须重新记零（begin_run 门槛同源）
        self.assertNotEqual(app.gait_params.mr2_zero_signature,
                            app._gait_zero_signature("Mr2", app.gait_params))
        self.assertTrue(any("记零" in line for line in app._logs))

    def test_lift_sign_flip_has_no_zero_invalidation(self):
        app = calibrated_app()
        app._gait_record_direction_sign("Mup2", False)  # 实际下降 ⇒ −1
        self.assertEqual(app.gait_params.mup2_lift_sign, -1)
        self.assertFalse(any("记零" in line for line in app._logs))

    def test_unchanged_sign_keeps_confirmed_params(self):
        app = calibrated_app()
        before = app.gait_params
        app._gait_record_direction_sign("Mr1", True)    # 实际俯视逆时针 = 默认 +1
        self.assertIs(app.gait_params, before)
        self.assertTrue(app.gait_params.calibration_confirmed)
        self.assertEqual(app.saved_docs, [])

    def test_gait_ownership_blocks_recording(self):
        app = calibrated_app()
        app._gait_owned = {3: object()}
        with patch("motor_control.desktop_app.messagebox.showerror") as err:
            app._gait_record_direction_sign("Mr2", False)
        err.assert_called_once()
        self.assertEqual(app.gait_params.mr2_sign, 1)
        self.assertEqual(app.saved_docs, [])

    def test_unknown_role_raises(self):
        app = calibrated_app()
        with self.assertRaises(GaitExecutorError):
            app._gait_record_direction_sign("Mr3", True)


class DirectionSignTextTests(unittest.TestCase):
    def test_sign_text_spells_out_real_meaning_of_axis_positive(self):
        from motor_control.ui.coordinated_tab import _direction_sign_text
        self.assertIn("俯视逆时针", _direction_sign_text("Mr1", 1))
        self.assertIn("俯视顺时针", _direction_sign_text("Mr1", -1))
        self.assertIn("抬起", _direction_sign_text("Mup1", 1))
        self.assertIn("下降", _direction_sign_text("Mup2", -1))


class _FakeWidget:
    def __init__(self):
        self.configured = {}

    def configure(self, **kwargs):
        self.configured.update(kwargs)


class RefreshDirectionCalibrationTests(unittest.TestCase):
    def test_jog_enabled_only_when_bound_and_idle(self):
        from motor_control.gait_planner import GaitParams
        from motor_control.ui.coordinated_tab import (
            _refresh_direction_calibration)
        snapshots = {
            "Mr1": {"binding_valid": True, "state": "IDLE"},
            "Mr2": {"binding_valid": True, "state": "MOVING"},
            "Mup1": {"binding_valid": False, "state": "IDLE"},
        }
        direction = {
            role: {name: _FakeWidget() for name in
                   ("sign", "jog_plus", "jog_minus")}
            for role in snapshots
        }
        app = SimpleNamespace(
            gait_params=replace(GaitParams(), mr1_sign=-1),
            coordinated_widgets={"direction": direction})
        _refresh_direction_calibration(app, snapshots)
        # Mr1（绑定+空闲）：点动可用；符号 −1 按实际含义显示
        self.assertEqual(direction["Mr1"]["jog_plus"].configured["state"],
                         "normal")
        self.assertIn("俯视顺时针",
                      direction["Mr1"]["sign"].configured["text"])
        # Mr2 运动中 / Mup1 未绑定有效：点动禁用；记录按钮不在本函数职责内
        self.assertEqual(direction["Mr2"]["jog_minus"].configured["state"],
                         "disabled")
        self.assertEqual(direction["Mup1"]["jog_plus"].configured["state"],
                         "disabled")


try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = hasattr(tk, "Tk")
except ImportError:
    HAS_TK = False


@unittest.skipUnless(HAS_TK, "real Tk unavailable")
class DirectionCalibrationLayoutTests(unittest.TestCase):
    def test_buttons_wire_roles_and_polarity(self):
        # 四行按钮各携带 (角色, 轴坐标极性 / 实际方向极性)，接错即测出。
        from motor_control import AxisProfile
        from motor_control.gait_planner import GaitParams
        from motor_control.ui.coordinated_tab import build_coordinated_tab
        jogs, records = [], []
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        app = SimpleNamespace(
            root=root,
            axis_profiles=[AxisProfile() for _ in range(6)],
            control_bindings=BindingSet.suggested(),
            gait_params=GaitParams(),
            _on_binding_editor_change=lambda *a: None,
            _jump_to_binding_axis=lambda *a: None,
            _fill_suggested_bindings=lambda *a: None,
            _save_binding_editor=lambda *a: None,
            _clear_binding_editor=lambda *a: None,
            _role_debug_move=lambda role, positive: jogs.append(
                (role, positive)),
            _gait_record_direction_sign=lambda role, positive: records.append(
                (role, positive)),
        )
        page = ttk.Frame(root)
        page.pack(fill="both", expand=True)
        build_coordinated_tab(app, page)
        direction = app.coordinated_widgets["direction"]
        for role in ("Mup1", "Mr1", "Mup2", "Mr2"):
            direction[role]["jog_plus"].invoke()
            direction[role]["jog_minus"].invoke()
            direction[role]["record_positive"].invoke()
            direction[role]["record_negative"].invoke()
        self.assertEqual(jogs, [(r, p) for r in ("Mup1", "Mr1", "Mup2", "Mr2")
                                for p in (True, False)])
        self.assertEqual(records, [(r, p) for r in
                                   ("Mup1", "Mr1", "Mup2", "Mr2")
                                   for p in (True, False)])


if __name__ == "__main__":
    unittest.main()
