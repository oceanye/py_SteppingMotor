"""A bounded Tk viewport that preserves access to wide/tall forms."""
import tkinter as tk
from tkinter import ttk


class ScrollableFrame(ttk.Frame):
    def __init__(self, parent, *, width=340, height=320):
        super().__init__(parent)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(self, width=width, height=height, highlightthickness=0)
        bg = ttk.Style().lookup("TFrame", "background")
        if bg:
            self.canvas.configure(background=bg)
        self.content = ttk.Frame(self.canvas)
        self._window = self.canvas.create_window((0, 0), window=self.content, anchor="nw")
        self._vertical = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self._horizontal = ttk.Scrollbar(self, orient="horizontal", command=self.canvas.xview)
        self.canvas.configure(yscrollcommand=self._vertical.set,
                              xscrollcommand=self._horizontal.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self._vertical.grid(row=0, column=1, sticky="ns")
        self._horizontal.grid(row=1, column=0, sticky="ew")
        self._horizontal.grid_remove()
        self.canvas.bind("<Configure>", self._resize)
        self.content.bind("<Configure>", self._resize)

    def _resize(self, _event=None):
        available = self.canvas.winfo_width()
        required = self.content.winfo_reqwidth()
        # Never squeeze a form smaller than its widgets. A horizontal scrollbar
        # remains available at high DPI or when the user narrows a pane.
        self.canvas.itemconfigure(self._window, width=max(available, required))
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        if required > available:
            self._horizontal.grid()
        else:
            self._horizontal.grid_remove()
            self.canvas.xview_moveto(0)

    def bind_navigation(self):
        """Call after adding children; wheel events stay local to this viewport."""
        def bind_tree(widget):
            widget.bind("<MouseWheel>", self._wheel, add="+")
            widget.bind("<Button-4>", self._wheel, add="+")
            widget.bind("<Button-5>", self._wheel, add="+")
            widget.bind("<Button-1>", self._mark_pointer_focus, add="+")
            widget.bind("<FocusIn>", self._reveal_focus, add="+")
            for child in widget.winfo_children():
                bind_tree(child)
        self.canvas.bind("<MouseWheel>", self._wheel, add="+")
        bind_tree(self.content)

    def _wheel(self, event):
        delta = getattr(event, "delta", 0)
        direction = -1 if delta > 0 or getattr(event, "num", None) == 4 else 1
        if self.canvas.yview() != (0.0, 1.0):
            self.canvas.yview_scroll(direction * max(1, abs(delta) // 120), "units")
        return "break"  # Do not accidentally change a direction combobox.

    def _mark_pointer_focus(self, event):
        # 鼠标按下时 widget 级绑定先于 ttk class 绑定设置焦点执行，
        # 记下 (控件, 时刻)；随后 0.5s 内该子树获得的 FocusIn 都是点击
        # 聚焦，不参与键盘导航的滚动露出。
        self._pointer_click = (event.widget, getattr(event, "time", 0))

    def _reveal_focus(self, event):
        # 键盘 Tab 导航时把聚焦控件滚进视口。鼠标点击同样触发 FocusIn，
        # 但 2026-09-23 现场表现为"每次点按钮右侧页面跳回顶部"（窗口
        # 刚从后台激活、坐标未同步时滚动目标被算成 0）——点击聚焦直接
        # 跳过，并且任何滚动前先同步几何。
        click = getattr(self, "_pointer_click", None)
        if click is not None and abs(
                getattr(event, "time", 0) - click[1]) < 500:
            node = event.widget
            while node is not None:
                if node is click[0]:
                    return
                node = node.master
        widget = event.widget
        self.update_idletasks()
        bounds = self.canvas.bbox("all")
        if bounds is None:
            return
        for coord, extent, scroll, total in (
            (widget.winfo_rooty() - self.canvas.winfo_rooty(), widget.winfo_height(),
             "y", bounds[3]),
            (widget.winfo_rootx() - self.canvas.winfo_rootx(), widget.winfo_width(),
             "x", bounds[2]),
        ):
            visible = self.canvas.winfo_height() if scroll == "y" else self.canvas.winfo_width()
            offset = self.canvas.canvasy(0) if scroll == "y" else self.canvas.canvasx(0)
            target = offset + min(0, coord) + max(0, coord + extent - visible)
            if target != offset and total > 0:
                getattr(self.canvas, f"{scroll}view_moveto")(max(0, target) / total)
