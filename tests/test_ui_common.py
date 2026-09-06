"""ui.common 数值输入工具：全角规范化与半成品校验（纯函数，无 Tk）。"""

import sys
import types
import unittest


try:
    import tkinter  # noqa: F401
except ImportError:
    tkinter_module = types.ModuleType("tkinter")
    tkinter_module.__path__ = []

    class TclError(Exception):
        pass

    tkinter_module.TclError = TclError
    ttk_module = types.ModuleType("tkinter.ttk")
    messagebox_module = types.ModuleType("tkinter.messagebox")
    for name in ("showerror", "showwarning", "showinfo", "askyesno", "askokcancel"):
        setattr(messagebox_module, name, lambda *_args, **_kwargs: None)
    tkinter_module.ttk = ttk_module
    tkinter_module.messagebox = messagebox_module
    sys.modules["tkinter"] = tkinter_module
    sys.modules["tkinter.ttk"] = ttk_module
    sys.modules["tkinter.messagebox"] = messagebox_module

from motor_control.ui.common import (
    is_complete_number,
    is_partial_number,
    normalize_number_text,
)


class NormalizeTests(unittest.TestCase):
    def test_fullwidth_digits_to_ascii(self):
        self.assertEqual(normalize_number_text("２１．５"), "21.5")

    def test_ideographic_and_fullwidth_period(self):
        for ch in "．。":
            with self.subTest(ch=hex(ord(ch))):
                self.assertEqual(normalize_number_text(f"21{ch}5"), "21.5")

    def test_fullwidth_comma_and_middle_dot_map_to_dot(self):
        self.assertEqual(normalize_number_text("3，5"), "3.5")
        self.assertEqual(normalize_number_text("3·5"), "3.5")

    def test_fullwidth_minus(self):
        self.assertEqual(normalize_number_text("－1.5"), "-1.5")

    def test_fullwidth_space_removed(self):
        self.assertEqual(normalize_number_text("　21　"), "21")

    def test_ascii_passthrough(self):
        self.assertEqual(normalize_number_text("21.5"), "21.5")
        self.assertEqual(normalize_number_text(""), "")

    def test_letters_untouched(self):
        # 字母不在规范化职责内（由按键过滤拒收），不应被改写
        self.assertEqual(normalize_number_text("a１"), "a1")


class PartialNumberTests(unittest.TestCase):
    def test_typing_intermediates_allowed(self):
        for text in ("", "-", ".", "-.", "21", "21.", "21.5", "-0.5"):
            with self.subTest(text=text):
                self.assertTrue(is_partial_number(text))

    def test_invalid_shapes_rejected(self):
        for text in ("--", "2.1.3", "1e5", "a", "1a", "２１", "2 1", "+1", "1,5"):
            with self.subTest(text=text):
                self.assertFalse(is_partial_number(text))

    def test_minus_only_at_start(self):
        self.assertTrue(is_partial_number("-1"))
        self.assertFalse(is_partial_number("1-"))


class CompleteNumberTests(unittest.TestCase):
    def test_completes(self):
        for text in ("0", "21", "21.5", "-0.5", ".5"):
            with self.subTest(text=text):
                self.assertTrue(is_complete_number(text))

    def test_incompletes(self):
        # "21." 不在此列：float("21.") == 21.0，变量可读，视为完整
        for text in ("", "-", ".", "-.", "abc", "1.2.3"):
            with self.subTest(text=text):
                self.assertFalse(is_complete_number(text))


if __name__ == "__main__":
    unittest.main()
