"""Desktop recovery dialog and atomic ledger commit; no motion commands."""
import time
from tkinter import messagebox

from .coordinated_control import BACKEND_X42S_UART
from .gait_executor import GaitExecutorError
from .gait_recovery import RECOVERY_TOLERANCE, recovery_differences, recovery_reference


def verify_pre_run_restore(app):
    title = "核验恢复站位（不驱动电机）"
    snap = getattr(app, "_gait_recovery_snapshot", None)
    if snap is None:
        messagebox.showinfo(title, "没有执行前快照。")
        return

    def verify():
        app._gait_check_start_edit()
        if app.control_bindings.backend != BACKEND_X42S_UART or not app._is_serial_connected():
            raise ValueError("请连接 X42S 后端后再核验。")
        if (snap is not getattr(app, "_gait_recovery_snapshot", None)
                or snap.get("reference") != recovery_reference(app)):
            raise ValueError("连接、绑定或基准已变化，旧快照失效；请确认实际摆放并重建双侧基准。")
        return recovery_differences(
            app.control_bindings, app._x42s_states, snap["positions"], now=time.monotonic())

    try:
        with app.state_lock:
            differences = verify()
        remaining = {role: delta for role, delta in differences.items()
                     if abs(delta) > RECOVERY_TOLERANCE}
        if remaining:
            detail = "\n".join(
                f"{role}：目标 {snap['positions'][role]:.2f}，尚差 {delta:+.2f}"
                + (" mm" if role.startswith("Mup") else "°")
                for role, delta in remaining.items())
            messagebox.showwarning(title,
                "实测尚未回到执行前位置，账本未恢复，也未发送运动命令。\n\n"
                + detail + "\n\n自动逆向回位尚无经过高杆避让验证的路径；"
                "请先停机，确认支撑和腿/横梁避让，再人工安全回位后重新核验。"
                "也可按实际摆放重建双侧基准，不需要驱动器 ZERO。")
            return
        if not messagebox.askyesno(title,
                "四轴实测均已回到开始前位置（容差：升降 0.05 mm、旋转 0.05°）。\n"
                "请人工确认脚落点、支撑、横梁朝向与开始前一致。\n"
                "确认后只恢复站位账本，不发送 MOVE/ZERO，也不重新标定。"):
            return
        with app.state_lock:
            differences = verify()  # The dialog may have been open for minutes.
            if any(abs(delta) > RECOVERY_TOLERANCE for delta in differences.values()):
                raise ValueError("确认期间轴位发生变化，账本未恢复，请重新核验。")
            app._gait_supports = tuple(snap["supports"])
            app._gait_beta_deg = float(snap["beta"])
            app._gait_needs_recovery = False
            app._gait_recovery_snapshot = None
            app._gait_run = None
            app._gait_auto_run = False
            app._gait_last_report = None
        app.log("✓ 四轴实测到位且人工确认摆放，已恢复执行前站位账本（未驱动电机）")
        app._refresh_gait_ui()
    except (ValueError, GaitExecutorError) as exc:
        app.log(f"⛔ 恢复核验未通过：{exc}")
        messagebox.showwarning(title, str(exc))
