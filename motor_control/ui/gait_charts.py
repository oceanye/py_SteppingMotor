"""Shared horizontal twin/curve layout; owns no simulation or motor state."""
import tkinter as tk
from tkinter import ttk


def build_twin_charts(parent, *, height=400):
    # Small requested width keeps the surrounding scroll form usable on small
    # screens. Both panes stretch on wide windows; the sash is user adjustable.
    panes = tk.PanedWindow(parent, orient="horizontal", width=300, height=height,
                           sashwidth=8, sashrelief="raised", borderwidth=0,
                           opaqueresize=True)
    canvases = []
    for title in ("平面移动 · 5×5", "爪端–高杆距离 (mm)"):
        frame = ttk.LabelFrame(panes, text=title)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        canvas = tk.Canvas(frame, width=120, height=height-28, background="white",
                           highlightthickness=1, highlightbackground="#cbd5e1")
        canvas.grid(row=0, column=0, sticky="nsew", padx=3, pady=3)
        panes.add(frame, minsize=120, width=150, stretch="always")
        canvases.append(canvas)
    return {"chart_panes": panes, "canvas": canvases[0], "curve_canvas": canvases[1]}
