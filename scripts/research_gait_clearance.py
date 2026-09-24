'''Offline research only: normalized leg/high-rod geometry, SVG plots and CSV.

No serial, GUI, firmware commands, configuration saves or production patches.
Uses only Python's standard library and the production pure geometry model.
Run: .venv/Scripts/python.exe -B scripts/research_gait_clearance.py
'''
from __future__ import annotations

import bisect
import csv
import hashlib
import html
import json
import math
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motor_control.gait_avoidance import HIGH_EXPONENT, avoidance_path, leg_clearance
from motor_control.gait_planner import GaitParams, effective_geometry, gait_pads

OUT = ROOT / 'docs' / 'assets' / 'gait_avoidance_2026-09-24'
D = math.sqrt(3)
RAD = math.pi / 180
BLUE, RED, GREEN = '#2563eb', '#dc2626', '#059669'


class Model:
    def __init__(self):
        self.params = GaitParams(trajectory_mode='two_mode_v1')
        self.geometry = effective_geometry(self.params)
        self.pads = list(gait_pads(self.params, 'B').values())
        radius = self.geometry.arm_length_mm
        self.rods = sorted({(round(x/radius, 12), round(y/radius, 12))
                            for pad in self.pads for x, y in pad.high_nodes(radius)})
        self.cache = {}

    @staticmethod
    def center(mode, x):
        phi = x if mode == 'LOW' else -x
        a = (180-phi)*RAD
        return D*math.cos(a), D*math.sin(a)

    def prepared(self, mode, x):
        key = mode, round(x, 10)
        if key not in self.cache:
            cx, cy = self.center(mode, x)
            self.cache[key] = sorted((math.hypot(rx-cx, ry-cy),
                                      math.atan2(ry-cy, rx-cx)) for rx, ry in self.rods)
        return self.cache[key]

    @staticmethod
    def distance(prepared, spin):
        # Closest of three equal radial segments = closest angular direction.
        # Sorted radial distance gives an exact exclusion bound, not a crop.
        psi = (30+spin)*RAD
        best2 = math.inf
        for radius, bearing in prepared:
            if radius >= 1 and (radius-1)**2 >= best2:
                break
            angle = (bearing-psi+math.pi/3) % (2*math.pi/3)-math.pi/3
            projection = radius*math.cos(angle)
            t = max(0., min(1., projection))
            best2 = min(best2, max(0., radius*radius+t*t-2*t*projection))
        return math.sqrt(best2)

    def evaluate(self, mode, x, spin):
        return self.distance(self.prepared(mode, x), spin)

    def closest_pair(self, mode, x, spin):
        cx, cy = self.center(mode, x)
        pairs = []
        for j in range(3):
            a = (30+spin+120*j)*RAD
            ux, uy = math.cos(a), math.sin(a)
            for k, (rx, ry) in enumerate(self.rods):
                t = max(0., min(1., (rx-cx)*ux+(ry-cy)*uy))
                px, py = cx+t*ux, cy+t*uy
                pairs.append((math.hypot(px-rx, py-ry), j, k, px, py, rx, ry))
        return min(pairs)


def knots(mode, exponent=18., center=30.):
    if mode == 'LOW':
        return [(0., 0.), (60., 120.)]
    c = center/60
    offset = math.log(c/(1-c))
    result = []
    for i in range(61):
        if i in (0, 60):
            spin = i*2.
        else:
            u = i/60
            z = exponent*(math.log(u/(1-u))-offset)
            spin = 120/(1+math.exp(-z))
        result.append((float(i), spin))
    return result


def spin_at(path, x):
    i = min(len(path)-2, max(0, bisect.bisect_right(path, (x, math.inf))-1))
    a, b = path[i:i+2]
    return a[1]+(b[1]-a[1])*(x-a[0])/(b[0]-a[0])


def evaluate_path(model, mode, path, step=.1):
    count = round(60/step)
    # Include every execution knot so each certificate interval is linear.
    xs = sorted({60*i/count for i in range(count+1)} | {x for x, _ in path})
    rows = [(x, spin_at(path, x), 0.) for x in xs]
    rows = [(x, y, model.evaluate(mode, x, y)) for x, y, _ in rows]
    bound = min(min(a[2], b[2])-(D*(b[0]-a[0])+abs(b[1]-a[1]))*RAD/2
                for a, b in zip(rows, rows[1:]))
    return rows, bound


def path_metrics(mode, path):
    sign = 1 if mode == 'LOW' else -1
    qs = [y+sign*x for x, y in path]
    slopes, times = [], []
    for a, b in zip(path, path[1:]):
        dp, ds = b[0]-a[0], b[1]-a[1]+sign*(b[0]-a[0])
        slopes.append(abs((b[1]-a[1])/dp))
        times.append(max(.05, 1.875*dp/6, 1.875*abs(ds)/18))
    return dict(joint_min=min(qs), joint_max=max(qs),
                max_spin_orbit_ratio=max(slopes), nominal_seconds=sum(times))


def extrema(rows, bound):
    def pack(row):
        return dict(orbit_abs_deg=row[0], world_spin_delta_deg=row[1], centerline_gap_over_R=row[2])
    lo, hi = min(r[2] for r in rows), max(r[2] for r in rows)
    # Record separate mirrored extrema and plateaus instead of one arbitrary tie.
    groups = []
    for kind, target in [('minimum', lo), ('maximum', hi)]:
        selected = [r for r in rows if abs(r[2]-target) <= 1e-8]
        spans = []
        for row in selected:
            if not spans or row[0]-spans[-1][-1][0] > .021:
                spans.append([row])
            else:
                spans[-1].append(row)
        groups.append([dict(start=pack(s[0]), end=pack(s[-1])) for s in spans])
    return dict(sampled_min=lo, sampled_max=hi, continuous_lower_bound=bound,
                minimum_locations=groups[0], maximum_locations=groups[1])


def validate(model):
    rng = random.Random(20260924)
    g = model.geometry
    thickness = g.arm_radius_mm+g.node_radius_mm+g.safety_margin_mm
    error = 0.
    for _ in range(240):
        mode = rng.choice(['LOW', 'HIGH'])
        x, spin = rng.uniform(0, 60), rng.uniform(-180, 240)
        cx, cy = model.center(mode, x)
        reference = (leg_clearance((cx*g.arm_length_mm, cy*g.arm_length_mm),
                                   30+spin, g, model.pads)[0]+thickness)/g.arm_length_mm
        value = model.evaluate(mode, x, spin)
        error = max(error, abs(reference-value))
        assert abs(reference-value) < 1e-8
        assert abs(value-model.evaluate(mode, x, spin+120)) < 1e-8
    for mode, arc in [('LOW', 60), ('HIGH', -60)]:
        current = avoidance_path(180, arc)
        for i in range(601):
            x = i/10
            assert abs(spin_at(knots(mode, HIGH_EXPONENT), x)-current.angles(x/60)[0]) < 1e-9
        path = knots(mode, HIGH_EXPONENT)
        coarse, bound = evaluate_path(model, mode, path, .1)
        fine, _ = evaluate_path(model, mode, path, .02)
        assert bound <= min(r[2] for r in fine)+1e-10
    mirror_pads = list(gait_pads(model.params, 'A').values())
    for mode in ['LOW', 'HIGH']:
        for i in range(61):
            x = float(i)
            spin = spin_at(knots(mode, HIGH_EXPONENT), x)
            cx, cy = model.center(mode, x)
            reference = (leg_clearance(((-D-cx)*g.arm_length_mm, cy*g.arm_length_mm),
                                       30-spin, g, mirror_pads)[0]+thickness)/g.arm_length_mm
            assert abs(reference-model.evaluate(mode, x, spin)) < 1e-8
    return dict(seed=20260924, reference_cases=240, max_error=error,
                production_paths_match=True, periodicity_checked=True, dense_bound_checked=True,
                right_action_mirror_cases=122)


def scan_parameters(model):
    coarse = []
    for n in range(10, 31):
        for ci in range(113, 128):  # center = 28.25 .. 31.75 degrees
            c = ci/4
            rows, bound = evaluate_path(model, 'HIGH', knots('HIGH', n, c), .1)
            coarse.append((min(r[2] for r in rows), n, c, bound))
    seeds = sorted(coarse, reverse=True)[:6]
    refined = {}
    for _, n, c, _ in seeds:
        for ni in range(round(n*10)-5, round(n*10)+6):
            for ci in range(round(c*20)-5, round(c*20)+6):
                key = round(ni/10, 8), round(ci/20, 8)
                if key in refined:
                    continue
                rows, bound = evaluate_path(model, 'HIGH', knots('HIGH', *key), .05)
                refined[key] = min(r[2] for r in rows)
    finalists = []
    for key, score in sorted(refined.items(), key=lambda item: item[1], reverse=True)[:8]:
        rows, bound = evaluate_path(model, 'HIGH', knots('HIGH', *key), .01)
        finalists.append((min(r[2] for r in rows), key[0], key[1], bound))
    winner = max(finalists)
    return coarse, refined, finalists, winner


def write_csv(name, header, rows):
    with (OUT/name).open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


class SVG:
    def __init__(self, width, height, title):
        self.items = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                      f'<title>{html.escape(title)}</title>', '<rect width="100%" height="100%" fill="white"/>',
                      '<g font-family="Segoe UI,Microsoft YaHei,Arial,sans-serif" fill="#0f172a">']
        self.text(24, 28, title, 18)

    def text(self, x, y, text, size=12, color='#0f172a', anchor='start'):
        self.items.append(f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" fill="{color}" text-anchor="{anchor}">{html.escape(str(text))}</text>')

    def line(self, x1, y1, x2, y2, color, width=1, dash=''):
        self.items.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>')

    def rect(self, x, y, w, h, color):
        self.items.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="{color}"/>')

    def circle(self, x, y, radius, color):
        self.items.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{color}"/>')

    def polyline(self, points, color, width=2):
        coords = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
        self.items.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"/>')

    def save(self, name):
        (OUT/name).write_text('\n'.join(self.items+['</g></svg>']), encoding='utf-8')


def axes(svg, box, xrange, yrange, title, xlabel, ylabel):
    left, top, width, height = box
    def xy(x, y):
        return left+(x-xrange[0])/(xrange[1]-xrange[0])*width, top+height-(y-yrange[0])/(yrange[1]-yrange[0])*height
    svg.text(left, top-20, title, 15)
    for i in range(7):
        v = xrange[0]+(xrange[1]-xrange[0])*i/6
        x, y = xy(v, yrange[0])
        svg.line(x, top, x, top+height, '#e2e8f0')
        svg.text(x, y+18, f'{v:g}', 11, anchor='middle')
    for i in range(5):
        v = yrange[0]+(yrange[1]-yrange[0])*i/4
        x, y = xy(xrange[0], v)
        svg.line(left, y, left+width, y, '#e2e8f0')
        svg.text(x-8, y+4, f'{v:.3g}', 11, anchor='end')
    svg.line(left, top+height, left+width, top+height, '#334155')
    svg.line(left, top, left, top+height, '#334155')
    svg.text(left+width/2, top+height+42, xlabel, 12, anchor='middle')
    svg.text(left, top-4, ylabel, 11, '#475569')
    return xy


def color_gap(gap):
    # Common absolute scale in both modes, no undocumented safety threshold.
    t = max(0., min(1., gap/.9))
    lo, hi = (252, 232, 230), (5, 105, 135)
    return '#'+''.join(f'{round(a+(b-a)*t):02x}' for a, b in zip(lo, hi))


def draw_curves(traces, candidate):
    svg = SVG(1100, 495, '01 | Two distinct angle laws / 两模态角度关系')
    for panel, title, limits in [(0, 'World spin delta', (-5, 125)),
                                  (1, 'Swing / support joint delta', (-65, 185))]:
        xy = axes(svg, (65+panel*550, 105, 440, 290), (0, 60), limits,
                  title, 'Orbit progress |phi| [deg]', 'Angle [deg]')
        for mode, color in [('LOW', BLUE), ('HIGH', RED)]:
            rows = traces[mode]
            sign = 1 if mode == 'LOW' else -1
            values = [(x, y if panel == 0 else y+sign*x) for x, y, _ in rows[::5]]
            svg.polyline([xy(x, y) for x, y in values], color, 2.4)
            if panel:
                svg.polyline([xy(0, 0), xy(60, sign*60)], color, 1)
        if not panel:
            svg.polyline([xy(x, y) for x, y, _ in candidate[::5]], GREEN, 1.5)
    svg.text(65, 66, 'LOW baseline', 13, BLUE)
    svg.text(240, 66, 'HIGH n=18 baseline', 13, RED)
    svg.text(470, 66, 'HIGH offline candidate (not deployed)', 13, GREEN)
    svg.text(24, 475, 'Both panels use LEFT steps: LOW phi=+x, HIGH phi=-x; world spin is CCW positive. Thin lines: support joint.', 12)
    svg.save('01_angle_curves.svg')


def draw_clearance(traces, candidate, envelopes):
    svg = SVG(1100, 510, '02 | Whole-leg / all-high-rod bottleneck distance / 全腿最小间距')
    for panel, mode, color in [(0, 'LOW', BLUE), (1, 'HIGH', RED)]:
        xy = axes(svg, (65+panel*550, 105, 440, 295), (0, 60), (0, .9),
                  mode, 'Orbit progress |phi| [deg]', 'Centerline distance / R')
        svg.polyline([xy(x, gap) for x, spin, gap in traces[mode][::4]], color, 2.5)
        for x, spin, gap in envelopes[mode][::3]:
            svg.circle(*xy(x, gap), 1.5, '#94a3b8')
        if mode == 'HIGH':
            svg.polyline([xy(x, gap) for x, spin, gap in candidate[::4]], GREEN, 1.8)
        lo = min(traces[mode], key=lambda r: r[2])
        svg.circle(*xy(lo[0], lo[2]), 4, '#111827')
        svg.text(80+panel*550, 137, f'Baseline min ~= {lo[2]:.6f} R', 12, color)
    svg.text(65, 65, 'Colored: current executable paths. Green: research candidate. Gray dots: pointwise phase-grid maxima, NOT a valid path.', 12)
    svg.text(24, 476, 'Net gap / R = plotted value - (leg radius + rod radius + safety allowance) / R.', 12)
    svg.text(24, 496, 'No site calibration supplied. These curves are geometric centerline distances, not hardware clearance approval.', 12, '#b45309')
    svg.save('02_clearance_curves.svg')


def draw_cspace(model, traces, candidate):
    svg = SVG(1100, 525, '03 | Angle configuration space / 公转-自转角度空间')
    table, envelopes = [], {}
    for panel, mode in enumerate(['LOW', 'HIGH']):
        box = (65+panel*550, 105, 440, 300)
        xy = axes(svg, box, (0, 60), (0, 120), mode,
                  'Orbit progress |phi| [deg]', 'World spin delta [deg]')
        for xi in range(60):
            for yi in range(120):
                gap = model.evaluate(mode, xi+.5, yi+.5)
                left, bottom = xy(xi, yi)
                svg.rect(left, bottom-2.5, 440/60+.05, 2.55, color_gap(gap))
        env = []
        for xi in range(121):
            x = xi/2
            values = [(x, yi/2, model.evaluate(mode, x, yi/2)) for yi in range(241)]
            table.extend((mode, *r) for r in values)
            env.append(max(values, key=lambda r: r[2]))
        envelopes[mode] = env
        svg.polyline([xy(x, y) for x, y, gap in traces[mode][::6]], '#ffffff', 4)
        svg.polyline([xy(x, y) for x, y, gap in traces[mode][::6]], '#111827', 1.7)
        if mode == 'HIGH':
            svg.polyline([xy(x, y) for x, y, gap in candidate[::6]], '#22c55e', 2)
    svg.text(65, 66, 'Same color scale in both modes: pink = smaller centerline gap; dark teal = larger gap. Black/white: current path.', 12)
    for i in range(180):
        svg.rect(330+i*2, 465, 2, 12, color_gap(.9*i/179))
    svg.text(318, 476, '0', 11, anchor='end')
    svg.text(700, 476, '0.9 R', 11)
    svg.text(24, 512, 'Delta-psi endpoints 0 and 120 are geometrically periodic, but different unwrapped motor positions. No free phase jumps.', 12)
    svg.save('03_configuration_space.svg')
    write_csv('configuration_grid.csv', ['mode', 'orbit_abs_deg', 'world_spin_delta_deg', 'centerline_gap_over_R'], table)
    write_csv('pointwise_phase_grid_envelope.csv', ['mode', 'orbit_abs_deg', 'world_spin_delta_deg', 'centerline_gap_over_R'],
              ((mode, *r) for mode, rows in envelopes.items() for r in rows))
    return envelopes


def draw_poses(model, traces):
    svg = SVG(1100, 875, '04 | Pose sketches at tight and wide positions / 极值姿态示意')
    for mi, mode in enumerate(['LOW', 'HIGH']):
        for kind, row, offset in [('minimum', min(traces[mode], key=lambda r: r[2]), 0),
                                  ('maximum (start)', traces[mode][0], 1)]:
            bx, by = 30+offset*550, 65+mi*400
            x, spin, gap = row
            cx, cy = model.center(mode, x)
            scale = 74
            def project(px, py):
                return bx+270+(px+.7)*scale, by+180-py*scale
            svg.text(bx+12, by+18, f'{mode}: {kind}, |phi|={x:.2f}, spin={spin:.2f}', 14)
            # Display exact normalized rod axes; dot sizes are symbols only.
            for rx, ry in model.rods:
                px, py = project(rx, ry)
                if bx+20 < px < bx+515 and by+40 < py < by+325:
                    svg.circle(px, py, 3.5, RED)
            center = project(cx, cy)
            pivot = project(0, 0)
            svg.line(*pivot, *center, '#94a3b8', 2)
            svg.circle(*pivot, 5, '#111827')
            for j in range(3):
                angle = (30+spin+120*j)*RAD
                tip = project(cx+math.cos(angle), cy+math.sin(angle))
                svg.line(*center, *tip, BLUE, 4)
                svg.circle(*tip, 3, BLUE)
            svg.circle(*center, 5, BLUE)
            # Mark the actual closest point pair (full segment, not tip-only).
            pairs = []
            for j in range(3):
                angle = (30+spin+120*j)*RAD
                ux, uy = math.cos(angle), math.sin(angle)
                for rx, ry in model.rods:
                    t = max(0, min(1, (rx-cx)*ux+(ry-cy)*uy))
                    point = cx+t*ux, cy+t*uy
                    pairs.append((math.dist(point, (rx, ry)), point, (rx, ry)))
            _, p, rod = min(pairs)
            svg.line(*project(*p), *project(*rod), '#f59e0b', 2.5, '4 3')
            svg.text(bx+12, by+350, f'Nearest whole-leg / high-rod axis gap = {gap:.6f} R', 13)
    svg.text(24, 858, 'Blue: three radial legs. Red: high-rod axes. Orange dashed: closest pair. Symbol sizes are NOT measured radii.', 12)
    svg.save('04_extremum_poses.svg')


def draw_scan(coarse, winner):
    svg = SVG(940, 475, '05 | Bounded HIGH parameter-family search / 有限参数族扫描')
    xy = axes(svg, (80, 105, 780, 270), (10, 30), (0, .16),
              'Center fixed at 30 deg, 60 executable edges', 'Exponent n', 'Bottleneck centerline gap / R')
    rows = sorted((n, score, bound) for score, n, c, bound in coarse if c == 30)
    svg.polyline([xy(n, score) for n, score, bound in rows], BLUE, 2.5)
    svg.polyline([xy(n, bound) for n, score, bound in rows], '#94a3b8', 2)
    svg.circle(*xy(winner[1], winner[0]), 4, GREEN)
    svg.text(80, 63, 'Blue: sampled minimum (0.1 deg grid). Gray: conservative bound on that grid.', 12)
    svg.text(80, 440, f'Green: refined candidate n={winner[1]:g}, center={winner[2]:g} deg; local bounded search, NOT global optimum.', 12)
    svg.save('05_parameter_scan.svg')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = Model()
    checks = validate(model)
    print('Reference cross-check passed:', checks, flush=True)
    traces, summaries = {}, {}
    for mode in ['LOW', 'HIGH']:
        path = knots(mode, HIGH_EXPONENT)
        rows, bound = evaluate_path(model, mode, path, .01)
        traces[mode] = rows
        summaries[mode] = {**extrema(rows, bound), **path_metrics(mode, path)}
        write_csv(f'{mode.lower()}_trace.csv',
                  ['orbit_abs_deg', 'orbit_signed_deg', 'world_spin_delta_deg', 'world_phase_deg',
                   'swing_joint_delta_deg', 'support_joint_delta_deg', 'center_x_over_R', 'center_y_over_R', 'centerline_gap_over_R'],
                  ((x, x if mode == 'LOW' else -x, y, 30+y, y+(x if mode == 'LOW' else -x),
                    x if mode == 'LOW' else -x, *model.center(mode, x), gap) for x, y, gap in rows))
    print('Current-path extrema evaluated.', flush=True)
    # Right actions are exact ideal-lattice mirrors, not separate site calibration.
    write_csv('four_actions_trace.csv',
              ['action', 'mode', 'orbit_abs_deg', 'orbit_signed_deg', 'world_spin_delta_deg',
               'swing_joint_delta_deg', 'support_joint_delta_deg', 'center_x_over_R', 'center_y_over_R', 'centerline_gap_over_R'],
              ((action, mode, x, direction*x, mirror*y, mirror*y+direction*x, direction*x,
                model.center(mode, x)[0] if mirror == 1 else -D-model.center(mode, x)[0],
                model.center(mode, x)[1], gap)
               for action, mode, direction, mirror in [('L+', 'LOW', 1, 1), ('R-', 'LOW', -1, -1),
                                                       ('L-', 'HIGH', -1, 1), ('R+', 'HIGH', 1, -1)]
               for x, y, gap in traces[mode]))
    coarse, refined, finalists, winner = scan_parameters(model)
    write_csv('high_parameter_scan_coarse.csv', ['sampled_min_over_R', 'exponent', 'center_deg', 'lower_bound_over_R'], coarse)
    write_csv('high_parameter_scan_refined.csv', ['exponent', 'center_deg', 'sampled_min_over_R'],
              ((*key, value) for key, value in sorted(refined.items())))
    candidate_path = knots('HIGH', winner[1], winner[2])
    candidate, bound = evaluate_path(model, 'HIGH', candidate_path, .01)
    write_csv('high_candidate_trace.csv', ['orbit_abs_deg', 'world_spin_delta_deg', 'centerline_gap_over_R'], candidate)
    critical = []
    for label, mode, rows in [('LOW_current', 'LOW', traces['LOW']),
                              ('HIGH_current', 'HIGH', traces['HIGH']),
                              ('HIGH_candidate', 'HIGH', candidate)]:
        lo, hi = min(r[2] for r in rows), max(r[2] for r in rows)
        for kind, target in [('minimum', lo), ('maximum', hi)]:
            for x, y, gap in rows:
                if abs(gap-target) <= 1e-8:
                    pair = model.closest_pair(mode, x, y)
                    assert abs(pair[0]-gap) < 1e-8
                    critical.append((label, kind, x, y, *pair))
    write_csv('critical_pose_pairs.csv',
              ['path', 'extremum', 'orbit_abs_deg', 'world_spin_delta_deg', 'centerline_gap_over_R',
               'leg_index_zero_based', 'rod_index_zero_based', 'leg_point_x_over_R', 'leg_point_y_over_R',
               'rod_x_over_R', 'rod_y_over_R'], critical)
    write_csv('execution_knots.csv', ['mode', 'orbit_abs_deg', 'world_spin_delta_deg'],
              ((mode, x, y) for mode, path in [('LOW_current', knots('LOW')), ('HIGH_current', knots('HIGH', HIGH_EXPONENT)),
                                             ('HIGH_candidate', candidate_path)] for x, y in path))
    print('Bounded search winner:', winner, flush=True)
    envelopes = draw_cspace(model, traces, candidate)
    upper_bounds = {}
    for mode, rows in envelopes.items():
        limiting = min(rows, key=lambda r: r[2])
        # Grid maximum is NOT a continuous-phase upper bound by itself.
        # Gap is 1-Lipschitz in phase (radians, R=1). Nearest phase grid
        # point is at most 0.25deg away. Any traversing path crosses this x.
        upper_bounds[mode] = dict(orbit_abs_deg=limiting[0],
                                 phase_grid_max=limiting[2],
                                 phase_discretization_allowance=.25*RAD,
                                 relaxed_path_upper_bound=limiting[2]+.25*RAD+1e-10,
                                 note='Loose geometric upper bound only; pointwise winners may be disconnected')
    draw_curves(traces, candidate)
    draw_clearance(traces, candidate, envelopes)
    draw_poses(model, traces)
    draw_scan(coarse, winner)
    convergence = []
    for mode, path in [('LOW', knots('LOW')), ('HIGH', knots('HIGH', HIGH_EXPONENT)), ('HIGH_candidate', candidate_path)]:
        for step in (.1, .02, .01, .005):
            rows, lower = evaluate_path(model, 'LOW' if mode == 'LOW' else 'HIGH', path, step)
            convergence.append((mode, step, min(r[2] for r in rows), lower))
    write_csv('sampling_convergence.csv', ['path', 'grid_step_deg', 'sampled_min_over_R', 'continuous_lower_bound_over_R'], convergence)
    try:
        commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = '57dc02bd1a0c40d41c7845e8940a3d7d9f5b4de3 (archive baseline; git unavailable)'
    output = dict(baseline_commit=commit, current_exponent=HIGH_EXPONENT,
                  units='All distances normalized by R; no site calibration; geometry only',
                  rods_xy_over_R=model.rods, d_over_R=D, initial_world_phase_deg=30,
                  initial_center_xy_over_R=[-D, 0], pivot_xy_over_R=[0, 0],
                  convention='LEFT LOW phi=+x; LEFT HIGH phi=-x; world spin CCW positive',
                  scalar_gap='min distance over all three full radial legs and all high-rod axes',
                  net_gap_formula='R*g - r_leg - r_rod - delta - validated_error_allowance',
                  illustrative_dimensions_not_site_calibration={'R_mm': 40, 'r_leg_mm': 4, 'r_rod_mm': 5, 'delta_mm': 2},
                  angle_space_grid={'orbit_step_deg': .5, 'phase_step_deg': .5, 'phase_range_deg': [0, 120]},
                  relaxed_path_upper_bounds=upper_bounds,
                  current_midpoint_gap_over_R={mode: model.evaluate(mode, 30, 60) for mode in ['LOW', 'HIGH']},
                  checks=checks, baselines=summaries,
                  high_candidate={**extrema(candidate, bound), **path_metrics('HIGH', candidate_path),
                                  'exponent': winner[1], 'center_deg': winner[2]},
                  search={'family': '120*sigmoid(n*(logit(u)-logit(center/60))), 60 linear edges',
                          'coarse_exponents': '10..30 step 1', 'coarse_centers_deg': '28.25..31.75 step 0.25',
                          'refine': 'top six seeds: exponent +/-0.5 step0.1; center +/-0.25 step0.05',
                          'objective': 'maximize sampled whole-path minimum, not single-pose maximum',
                          'certification': 'top eight refined candidates re-evaluated at 0.01deg, separate continuous lower bound',
                          'coarse_count': len(coarse), 'refined_count': len(refined), 'finalists': finalists,
                          'guarantee': 'bounded family search only; no global optimality or hardware feasibility guarantee'},
                  sources_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in [Path(__file__), ROOT/'motor_control/gait_avoidance.py', ROOT/'motor_control/gait_planner.py']})
    (OUT/'model_and_results.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8')
    # Check every SVG is well formed and every CSV numeric cell is finite.
    import xml.etree.ElementTree as ET
    for p in OUT.glob('*.svg'):
        ET.parse(p)
    for p in OUT.glob('*.csv'):
        with p.open(encoding='utf-8-sig', newline='') as stream:
            reader = csv.reader(stream)
            header = next(reader)
            for row in reader:
                assert len(row) == len(header), p
                for cell in row:
                    try:
                        value = float(cell)
                    except ValueError:
                        continue
                    assert math.isfinite(value), (p, cell)
    print(json.dumps({'baselines': summaries, 'candidate': output['high_candidate'], 'output': str(OUT)}, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
