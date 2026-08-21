"""所有 Tab 构建器共用的间距常量与数值输入工具。

排版统一原则：四个构建模块（main_window / stepper_tab / motor_tabs /
track_tab）一律从这里导入 ``PAD``，需要调整全局间距时只改本文件。

数值输入统一原则：所有 ttk.Spinbox 数值框都挂 ``attach_numeric_input``。
病根一（2026-08-21 实验证实）：中文输入法打出的全角字符（０-９ ． 。，－）
能进入输入框文本，但 DoubleVar 随即进入不可读状态（get() 抛
TclError），表现为"小数怎么都输不进去"。
病根二（2026-08-21 按键探针 logs/key_probe.log 证实）：NumLock 开着时
中文输入法转发的小键盘'.'事件"字符字段'.'+键名'Delete'+键码 46"——
放行给输入框默认处理会被按 Delete 编辑键丢掉，表现为"按第一次没反应，
按第二次才进"（第二次输入法改发 char='。' 的字符上屏事件，键码被改写
成 12290=全角句号码点，而非物理键码 110）。因此合法数字字符必须一律
亲手插入，不能信任默认处理。
"""

import re
import time
from pathlib import Path

import tkinter as tk

# 统一控件间距（原为 padx=10, pady=5，压缩后一屏可放下全部内容）
PAD = dict(padx=6, pady=2)

# 全角 → 半角映射：数字、小数点三种全角形态、全角减号；全角空格删除
_FULLWIDTH_MAP = {ord(c): r for c, r in zip("０１２３４５６７８９", "0123456789")}
_FULLWIDTH_MAP.update({
    ord("．"): ".",
    ord("。"): ".",
    ord("，"): ".",
    ord("·"): ".",
    ord("－"): "-",
    ord("　"): None,
})

# 允许出现在数值输入框里的"半成品"文本：空串、""-"、"."、"21."、"-.5" 等。
# 必须 re.ASCII：默认 \d 会匹配全角数字（２１），让它们溜进 DoubleVar。
_PARTIAL_NUMBER_RE = re.compile(r"-?\d*\.?\d*", re.ASCII)

# Windows 虚拟键码兜底（2026-08-21 真机复测：全角转换修好后句点仍插不进，
# 说明输入法上屏的句点在真实按键事件里 event.char 为空——类绑定拿不到
# 字符，什么都不插。此时物理键码仍可靠）：
#   主键盘句号 VK_OEM_PERIOD=190，小键盘小数点 VK_DECIMAL=110，
#   主键盘减号 VK_OEM_MINUS=189。
_VK_TO_CHAR = {190: ".", 110: ".", 189: "-"}

# keysym 兜底（优先于键码：跨键盘布局仍语义稳定）
_KEYSYM_TO_CHAR = {"period": ".", "KP_Decimal": ".", "decimal": ".",
                   "minus": "-", "KP_Subtract": "-"}

# 按键探针：异常按键（空字符兜底、全角转换、拒收）留档到 logs/key_probe.log，
# 现场再出现"输不进"时凭此文件定位；正常按键不记录。
_PROBE_LOG = Path("logs") / "key_probe.log"


def _probe(event, note):
    try:
        _PROBE_LOG.parent.mkdir(exist_ok=True)
        with _PROBE_LOG.open("a", encoding="utf-8") as fh:
            fh.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {note} "
                     f"char={event.char!r} keysym={event.keysym!r} "
                     f"keycode={getattr(event, 'keycode', None)}\n")
    except OSError:
        pass


def normalize_number_text(text):
    """把全角数字/小数点/减号转成半角，其余原样返回。"""
    return text.translate(_FULLWIDTH_MAP)


def is_partial_number(text):
    """文本是否仍是合法数字的"半成品"（输入过程中允许不完整）。"""
    return _PARTIAL_NUMBER_RE.fullmatch(text) is not None


def is_complete_number(text):
    """文本是否是最终可解析的完整数字。"""
    if not is_partial_number(text):
        return False
    try:
        float(text)
        return True
    except ValueError:
        return False


def attach_numeric_input(widget, var=None):
    """让数值输入框支持中文输入法小数输入并拒收非法字符。

    - <Key>（实例绑定先于类绑定执行）：合法数字字符（含全角转半角、
      键名/键码兜底）一律由本函数亲手插入并拦截原始按键——输入框默认
      处理靠键名分发，中文输入法转发的"字符'.'+键名'Delete'"类怪事件
      会被它当编辑键丢掉；无法构成数字的字符直接吞掉；
    - validate="key"：兜底拦截粘贴等绕过按键路径的非法文本；
    - <KeyRelease>：文本是完整数字时快照为"最后有效值"；
    - <FocusOut>：留下 "-"、"21." 之类半成品时恢复为最后有效值
      （半成品会把 DoubleVar 毒化成不可读，所以不能从 var 取）。
    """
    widget.configure(validate="key", validatecommand=(
        widget.register(_validate_proposed), "%P"))
    widget._num_last_good = _read_var(var)
    widget.bind("<Key>", lambda event: _filter_key(widget, event), add="+")
    widget.bind("<KeyRelease>", lambda event: _snapshot(widget), add="+")
    widget.bind("<FocusOut>", lambda event: _restore_on_blur(widget, var), add="+")


def _read_var(var):
    if var is None:
        return None
    try:
        return f"{var.get():g}"
    except (tk.TclError, ValueError, TypeError):
        return None


def _snapshot(widget):
    try:
        text = widget.get()
    except tk.TclError:
        return
    if is_complete_number(text):
        widget._num_last_good = text


def _validate_proposed(proposed):
    # 严格按原文判断（不先 normalize）：全角粘贴文本整体拒收，
    # 按键路径的全角转换由 _filter_key 负责。
    return is_partial_number(proposed)


def _filter_key(widget, event):
    ch = event.char or ""
    if not ch:
        vk = getattr(event, "keycode", 0) or 0
        ch = _KEYSYM_TO_CHAR.get(getattr(event, "keysym", "") or "") \
            or _VK_TO_CHAR.get(vk)
        if not ch:
            return None  # 方向键/功能键等，交给类绑定
        _probe(event, f"fallback insert {ch!r}")
    if ord(ch) < 32:
        return None  # 退格/回车等控制键，交给类绑定
    norm = normalize_number_text(ch)
    if not norm:
        _probe(event, "drop unmappable char")
        return "break"  # 全角空格等：直接丢弃
    if not is_partial_number(_proposed_after(widget, norm)):
        _probe(event, f"reject {ch!r}")
        return "break"  # 字母、第二个小数点等：吞掉
    if norm != ch:
        _probe(event, f"normalize {ch!r} -> {norm!r}")
    # 合法数字字符一律亲手插入并拦截原始按键。不能只对全角/兜底路径手工
    # 插入：真机实测（2026-08-21 日志）中文输入法转发的小键盘'.'事件
    # 字符字段是'.'、键名却挂着'Delete'——放行给类绑定会被当作编辑键，
    # 插入被丢掉，表现即"第一次按没反应，第二次才进"。
    try:
        if widget.selection_present():
            widget.delete("sel.first", "sel.last")
        widget.insert("insert", norm)
    except tk.TclError:
        return None
    return "break"


def _proposed_after(widget, ch):
    text = widget.get()
    try:
        if widget.selection_present():
            start, end = widget.index("sel.first"), widget.index("sel.last")
        else:
            start = end = widget.index("insert")
    except tk.TclError:
        start = end = len(text)
    return text[:start] + ch + text[end:]


def _restore_on_blur(widget, var):
    try:
        if is_complete_number(normalize_number_text(widget.get()).strip()):
            return
        fallback = (getattr(widget, "_num_last_good", None)
                    or _read_var(var) or "0")
        widget.delete(0, "end")
        widget.insert(0, fallback)
        _snapshot(widget)
    except tk.TclError:
        pass  # 窗口销毁等瞬间控件已失效
