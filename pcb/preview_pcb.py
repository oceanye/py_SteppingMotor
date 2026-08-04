#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""preview_pcb.py — 解析 stepper_carrier.kicad_pcb 并渲染成 SVG 供人工检查。"""
import re, sys

HERE = __import__("pathlib").Path(__file__).resolve().parent
SRC = HERE / "stepper_carrier.kicad_pcb"
OUT = HERE / "preview.svg"
BOARD_W, BOARD_H = 180, 210

text = open(SRC, encoding="utf-8").read()

# --- 极小 S 表达式解析 ---
tokens = re.findall(r'\(|\)|[^\s()]+', text)
pos = 0
def parse():
    global pos
    if tokens[pos] == '(':
        pos += 1
        lst = []
        while tokens[pos] != ')':
            lst.append(parse())
        pos += 1
        return lst
    else:
        t = tokens[pos]; pos += 1
        return t
root = parse()
assert pos == len(tokens), "trailing tokens"
assert root[0] == "kicad_pcb"

def walk(node):
    yield node
    for ch in node[1:]:
        if isinstance(ch, list):
            yield from walk(ch)

def num(s):
    return float(s)

# 网络颜色
NET_COLOR = {"+24V": "#e65100", "+12V": "#1565c0", "+3V3": "#2e7d32", "GND": "#888888"}
SIG_COLORS = ["#6a1b9a", "#00838f", "#795548", "#c62828", "#ad1457", "#5d4037",
              "#00695c", "#4527a0", "#9e9d24", "#37474f", "#7b1fa2", "#0277bd"]
netnames = {}
sig_idx = {}

svg = []
svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-5 -8 {BOARD_W + 12} {BOARD_H + 16}" '
           f'style="background:#1b1b1b" width="1440">')
svg.append(f'<rect x="0" y="0" width="{BOARD_W}" height="{BOARD_H}" fill="none" stroke="#fff" stroke-width="0.3"/>')

for node in walk(root):
    tag = node[0]
    if tag == "net" and isinstance(node[1], str) and len(node) == 3:
        try: netnames[int(node[1])] = node[2].strip('"')
        except ValueError: pass

def color_for(netid):
    name = netnames.get(netid, "")
    if name in NET_COLOR: return NET_COLOR[name]
    if name not in sig_idx:
        sig_idx[name] = len(sig_idx) % len(SIG_COLORS)
    return SIG_COLORS[sig_idx[name]]

pads, traces, vias, texts, rects, holes = [], [], [], [], [], []
fp_at = (0, 0); fp_rot = 0
for node in walk(root):
    tag = node[0]
    if tag == "footprint":
        for ch in node[1:]:
            if isinstance(ch, list) and ch[0] == "at":
                fp_at = (num(ch[1]), num(ch[2]))
                fp_rot = num(ch[3]) if len(ch) > 3 else 0
    elif tag == "pad":
        at = next(c for c in node if isinstance(c, list) and c[0] == "at")
        size = next(c for c in node if isinstance(c, list) and c[0] == "size")
        drill = next((c for c in node if isinstance(c, list) and c[0] == "drill"), None)
        net = next((c for c in node if isinstance(c, list) and c[0] == "net"), None)
        x, y = num(at[1]), num(at[2])
        pads.append((fp_at[0] + x, fp_at[1] + y, num(size[1]),
                     num(drill[1]) if drill else 0,
                     color_for(int(net[1])) if net else "#555",
                     node[2] == "np_thru_hole"))
    elif tag == "segment":
        st = next(c for c in node if isinstance(c, list) and c[0] == "start")
        en = next(c for c in node if isinstance(c, list) and c[0] == "end")
        w = next(c for c in node if isinstance(c, list) and c[0] == "width")
        lay = next(c for c in node if isinstance(c, list) and c[0] == "layer")
        net = next(c for c in node if isinstance(c, list) and c[0] == "net")
        traces.append((num(st[1]), num(st[2]), num(en[1]), num(en[2]),
                       num(w[1]), color_for(int(net[1])), lay[1].strip('"'),
                       netnames.get(int(net[1]), "?")))
    elif tag == "via":
        at = next(c for c in node if isinstance(c, list) and c[0] == "at")
        net = next(c for c in node if isinstance(c, list) and c[0] == "net")
        vias.append((num(at[1]), num(at[2]), color_for(int(net[1]))))
    elif tag == "gr_text":
        at = next(c for c in node if isinstance(c, list) and c[0] == "at")
        texts.append((num(at[1]), num(at[2]), node[1].strip('"')))
    elif tag == "gr_rect":
        st = next(c for c in node if isinstance(c, list) and c[0] == "start")
        en = next(c for c in node if isinstance(c, list) and c[0] == "end")
        lay = next(c for c in node if isinstance(c, list) and c[0] == "layer")
        rects.append((num(st[1]), num(st[2]), num(en[1]), num(en[2]), lay[1].strip('"')))

for (x1, y1, x2, y2, layer) in rects:
    dash = ' stroke-dasharray="1,1"' if layer == "Edge.Cuts" else ''
    svg.append(f'<rect x="{min(x1,x2)}" y="{min(y1,y2)}" width="{abs(x2-x1)}" height="{abs(y2-y1)}"'
               f' fill="none" stroke="#ffff0088" stroke-width="0.2"{dash}/>')

# 先 B.Cu（虚线）后 F.Cu（实线）
for (x1, y1, x2, y2, w, col, layer, netname) in sorted(traces, key=lambda t: t[6] != "B.Cu"):
    dash = ' stroke-dasharray="1.2,0.8"' if layer == "B.Cu" else ''
    op = "0.75" if layer == "B.Cu" else "1"
    svg.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{col}"'
               f' stroke-width="{w}" stroke-linecap="round" opacity="{op}"{dash}>'
               f'<title>{netname} ({layer})</title></line>')

for (x, y, dia, drill, col, np_) in pads:
    svg.append(f'<circle cx="{x}" cy="{y}" r="{dia/2}" fill="{col}" stroke="#fff" stroke-width="0.15"/>')
    if drill:
        svg.append(f'<circle cx="{x}" cy="{y}" r="{drill/2}" fill="#1b1b1b"/>')
for (x, y, col) in vias:
    svg.append(f'<circle cx="{x}" cy="{y}" r="0.6" fill="{col}" stroke="#fff" stroke-width="0.1"/>')
    svg.append(f'<circle cx="{x}" cy="{y}" r="0.3" fill="#1b1b1b"/>')
for (x, y, s) in texts:
    svg.append(f'<text x="{x}" y="{y}" font-size="1.6" fill="#eee" text-anchor="middle">{s}</text>')

svg.append('</svg>')
open(OUT, "w", encoding="utf-8").write("\n".join(svg))
print(f"OK: {OUT}  pads={len(pads)} traces={len(traces)} vias={len(vias)} nets={len(netnames)}")
