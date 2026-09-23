"""GUI 冒烟测试：构建完整界面并走四种换位方式的预览/播放路径。

用法：
    python scripts/gui_smoke.py          # 无窗口（隐藏），跑完即退
    python scripts/gui_smoke.py --show   # 显示窗口，便于人工查看布局

不连串口、不动电机，只读本地配置文件。窗口默认隐藏；退出时显式
quit+destroy 并吞掉销毁期 TclError——2026-09-22 曾因收尾不干净在桌面
留下一个无事件循环、点不动的孤儿窗口，此后冒烟一律走本脚本。
"""
import os
import sys
import math
import tkinter as tk
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_control.desktop_app import StepperGUI            # noqa: E402
from motor_control.state_store import StateStore
from motor_control.ui import reset_preview_view, stop_preview_animation  # noqa: E402
from motor_control.ui.gait_tab import _gait_mode_selected   # noqa: E402

# 四种换位方式 → (期望摆动侧, 期望弧度, 期望落点)。初始站位 (A,B)。
MODES = (
    ("L+", "left", 60.0, "C"),
    ("L-", "left", -60.0, "邻座(0,-1)"),
    ("R+", "right", 60.0, "邻座(0,-1)"),
    ("R-", "right", -60.0, "C"),
)
START_BUTTONS = ("start_left_cw", "start_left_ccw",
                 "start_right_cw", "start_right_ccw")


def run_smoke(app) -> None:
    widgets = app.gait_widgets
    missing = [key for key in START_BUTTONS + ("play_btn",) if key not in widgets]
    assert not missing, f"缺少控件: {missing}"

    # 执行区 5 个按钮（4 方向 + advance）grid 格子两两不重叠：
    # 2026-09-22 曾因 advance 与右顺/右逆同行同格，右侧两按钮被遮住。
    occupied = {}
    for key in START_BUTTONS + ("advance",):
        info = widgets[key].grid_info()
        for col in range(int(info["column"]),
                         int(info["column"]) + int(info["columnspan"])):
            cell = (info["in"], int(info["row"]), col)
            assert cell not in occupied, \
                f"grid 格子重叠: {key} 与 {occupied[cell]} 同占 {cell}"
            occupied[cell] = key

    for mode, side, arc, target in MODES:
        app.gait_mode_var.set(mode)
        _gait_mode_selected(app, mode)
        app._gait_play_preview_clicked()
        try:
            report = app._gait_last_report
            assert report.side == side, (mode, report.side)
            assert report.route[0] != report.route[1] and report.route[1] == target, \
                (mode, report.route)
            assert abs(report.samples[-1].phi_deg - arc) < 1e-6, (mode, arc)
            # Uncalibrated defaults may correctly FAIL leg clearance. Layout /
            # animation success is not evidence of physical collision safety.
            assert math.isfinite(report.min_margin_mm), (mode, report.min_margin_mm)
            assert report.feasible == (report.min_margin_mm > 0)
            playing = app.gait_widgets["preview_anim"].get("playing")
            assert playing, (mode, "动画未启动")
            print(f"{mode}: side={report.side:<5} {report.route[0]}->{report.route[1]}"
                  f" phi_end={report.samples[-1].phi_deg:+.0f}"
                  f" margin={report.min_margin_mm:.1f}mm playing={playing}")
        finally:
            stop_preview_animation(app)

    # 2026-09-23 预览画布视图交互：滚轮采用"左键选中画面"模式——未选中
    # 时滚轮只滚页面不缩放；选中（蓝框）后缩放并阻断页面滚动；移出画面
    # 自动取消选中。中键平移、双击复位照旧，动画播放中触发也不得崩溃。
    canvas = widgets["preview_canvas"]
    view = widgets["preview_view"]
    canvas.event_generate("<MouseWheel>", delta=120)
    assert view["zoom"] == 1.0, view          # 未选中：不缩放
    canvas.event_generate("<Button-1>")       # 左键点一下选中画面
    canvas.event_generate("<MouseWheel>", delta=120)
    canvas.event_generate("<MouseWheel>", delta=120)
    assert view["zoom"] > 1.0, view
    pan_before = (view["pan_x"], view["pan_y"])
    canvas.event_generate("<Button-2>", x=60, y=60)
    canvas.event_generate("<B2-Motion>", x=90, y=75)
    assert view["pan_x"] - pan_before[0] == 30.0, view
    assert view["pan_y"] - pan_before[1] == 15.0, view
    zoom_before_leave = view["zoom"]
    canvas.event_generate("<Leave>")          # 移出画面：取消选中
    canvas.event_generate("<MouseWheel>", delta=120)
    assert view["zoom"] == zoom_before_leave, view
    # event_generate 不能注入 Double 修饰事件；绑定存在性 + 复位函数效果
    assert canvas.bind("<Double-Button-1>")
    reset_preview_view(app)
    assert view["zoom"] == 1.0 and view["pan_x"] == 0.0, view

    # 2026-09-23 模拟动作三态：按钮在播放中=暂停（画面停在当前帧，
    # 滚轮缩放也不闪回静态预览），暂停中=继续（不从头重播）；
    # 轨迹变化/新报告才彻底停止并恢复"▶ 模拟动作"。
    app._gait_play_preview_clicked()
    anim = widgets["preview_anim"]
    assert anim["playing"], anim
    assert widgets["play_btn"].cget("text") == "⏸ 暂停模拟"
    app._gait_play_preview_clicked()          # 播放中 → 暂停
    assert anim["paused"] and not anim["playing"], anim
    assert anim["frame"] > 0, anim
    assert widgets["play_btn"].cget("text") == "▶ 继续模拟"
    paused_frame = anim["frame"]
    canvas.event_generate("<MouseWheel>", delta=120)   # 暂停帧上缩放
    assert anim["paused"] and anim["frame"] == paused_frame, anim
    app._gait_play_preview_clicked()          # 暂停 → 继续
    assert anim["playing"] and not anim["paused"], anim
    assert anim["frame"] >= paused_frame, anim
    assert widgets["play_btn"].cget("text") == "⏸ 暂停模拟"
    app._gait_play_preview_clicked()          # 再暂停
    assert anim["paused"], anim
    stop_preview_animation(app)               # 彻底停止（新报告前调用）
    assert not anim["playing"] and not anim["paused"], anim
    assert widgets["play_btn"].cget("text") == "▶ 模拟动作"


def main() -> int:
    root = tk.Tk()
    if "--show" not in sys.argv:
        root.withdraw()
    app = None
    try:
        with patch("motor_control.desktop_app.messagebox"), \
                patch.object(StateStore, "_save_object"):
            app = StepperGUI(root)
            run_smoke(app)
            root.update_idletasks()
        print("SMOKE OK")
        return 0
    finally:
        # 收尾必须无条件执行：先停动画回调，再 quit+destroy；销毁期
        # Tk 解释器可能半坏（嵌套 destroy 曾抛 TclError），全部吞掉，
        # 保证解释器正常退出、不残留无事件循环的孤儿窗口。
        if app is not None:
            try:
                stop_preview_animation(app)
            except Exception:
                pass
        for closer in (root.quit, root.destroy):
            try:
                closer()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
