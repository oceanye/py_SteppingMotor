"""Compatibility launcher for the modular desktop motor controller.

Field scripts and handoff documents continue to use ``python pc_gui.py``.
The implementation lives under :mod:`motor_control`; this file intentionally
stays small and re-exports the historically useful names.
"""

from __future__ import annotations

import ctypes
import ctypes.wintypes
import os
import sys

# ── 单实例保护（必须先于任何重量级 import，抢锁越早窗口越小）──
# 2026-09-22 / 09-23 两次现场事故：双击 pc_gui.py 时 Windows 把启动
# 注册了两次执行（间隔约 0.1s）。第二个实例与第一个抢 Tk/串口/配置
# 资源，卡在界面半构建状态——没有事件循环，点不动也关不掉，成为
# "孤儿窗"。启动最早期抢命名互斥：已有实例在跑就提示并立即退出。
# argtypes 必须显式声明成宽字符：不声明时 ctypes 把 str 按 ANSI 字节
# 传给 W 函数，互斥名实为错位乱码，两个进程读到的名字可能不同而使
# 锁静默失效（弹窗中文同样会乱码）。
# STEPPING_GUI_ALLOW_MULTI=1 仅限自动化测试/诊断对照时跳过保护。
if sys.platform == "win32" and not os.environ.get("STEPPING_GUI_ALLOW_MULTI"):
    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _kernel32.CreateMutexW.argtypes = (
        ctypes.c_void_p, ctypes.wintypes.BOOL, ctypes.c_wchar_p)
    _kernel32.CreateMutexW.restype = ctypes.c_void_p
    _mutex = _kernel32.CreateMutexW(None, False, "Local\\SteppingMotorGUI")
    if _mutex and ctypes.get_last_error() == 183:   # ERROR_ALREADY_EXISTS
        _user32 = ctypes.WinDLL("user32", use_last_error=True)
        _user32.MessageBoxW.argtypes = (
            ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_wchar_p, ctypes.c_uint)
        _user32.MessageBoxW(
            None, "步进电机控制 GUI 已在运行（仅允许单实例）。",
            "SteppingMotor GUI", 0x40)
        raise SystemExit(0)
    # 互斥句柄故意不关闭：进程退出时由 Windows 自动释放。

from pathlib import Path  # noqa: E402

from motor_control.bootstrap import configure_standard_streams


_PROJECT_PATH = Path(__file__).resolve().parent
configure_standard_streams(_PROJECT_PATH)

from motor_control import (  # noqa: E402 - bootstrap must run first under pythonw
    AXIS_LABEL,
    DEFAULT_GEAR_RATIO,
    DEFAULT_LEAD_MM,
    DEFAULT_PULSE_PER_REV,
    DELAY_OVERHEAD_US,
    LOCAL_AXIS_LABELS,
    MODE_LINEAR,
    MODE_ROTARY,
    NUM_LOCAL_STEPPER_AXES,
    NUM_PICO_NODES,
    NUM_STEPPER_AXES,
    PICO_AXES_PER_NODE,
    SPEED_DEFAULT,
    STEPPER_PINS,
    stepper_axis_topology,
)
from motor_control.desktop_app import (  # noqa: E402
    CONTINUOUS_BURST_MM,
    DELAY_DEFAULT_MS,
    DIR_INVERT,
    DIR_INWARD,
    DIR_OUTWARD,
    FOC_POLL_INTERVAL_S,
    FOC_STATE_NAMES,
    GEAR_STATE_NAMES,
    MODE_FOC,
    MODE_GEAR,
    NUM_AXES,
    NUM_GEAR_AXES,
    NUM_MOTOR_AXES,
    PULSES_PER_MM,
    SPEED_PRESETS,
    StepperGUI,
    _empty_axis_dict,
    _speed_to_delay_ms,
)


# Preserve the file/path constants exposed by the historical single-file GUI.
# They stay strings because some field-side helper scripts import them directly.
PROJECT_DIR = str(_PROJECT_PATH)
CALIB_FILE = str(_PROJECT_PATH / ".stepper_calib.json")
FOC_TUNE_FILE = str(_PROJECT_PATH / ".foc_tune.json")
GEAR_TUNE_FILE = str(_PROJECT_PATH / ".gear_tune.json")
LOG_DIR = str(_PROJECT_PATH / "logs")


def main() -> None:
    import tkinter as tk

    from web_control import WebControlServer

    root = tk.Tk()
    StepperGUI(root, lambda controller: WebControlServer(controller))
    root.mainloop()


if __name__ == "__main__":
    main()
