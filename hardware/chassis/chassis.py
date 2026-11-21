#!/usr/bin/env python3
"""
Parametric two-deck acrylic chassis for the ESP32 4WD robot.

    Maker-ESP32 v1.7x board  +  4x JGA25-371 motors in L-brackets  +  65 mm wheels

One description of every plate feeds all outputs, so the laser files, the
paper template and the 3D model can never disagree:

    out/bottom_plate.dxf / .svg     laser-cut files (layer CUT = red, ENGRAVE = blue)
    out/top_plate.dxf / .svg
    out/fit_test_coupon.dxf / .svg  small test piece - cut this first
    out/print_template_1to1.pdf     print at 100 % to check holes on paper
    out/chassis_assembly.step       full 3D assembly (open in any CAD tool)
    out/chassis_assembly.glb        same, for viewer.html (interactive 3D in a browser)

The script also builds simplified 3D models of the motors, brackets, wheels,
battery, standoffs and board and FAILS if any two parts collide or a
clearance rule is broken. Change a parameter, re-run, and it re-checks.

Usage:  pip install -r requirements.txt && python chassis.py

Coordinates: origin at the centre of the bottom plate's underside,
+X = right, +Y = forward, +Z = up. All dimensions in mm.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

# =============================================================================
# Parameters
# =============================================================================
ACRYLIC_T = 3.0           # sheet thickness (3 mm cast acrylic recommended)
PLATE_W = 160.0           # plate width (X); bracket faces sit flush with the sides
PLATE_L = 200.0           # plate length (Y)
CORNER_R = 8.0
WHEELBASE = 130.0         # front-to-rear axle distance
STANDOFF_H = 35.0         # M3 hex standoffs between the decks
BOARD_STANDOFF_H = 10.0   # nylon M3 standoffs under the Maker-ESP32
M3_CLEAR = 3.3            # M3 clearance hole (3.2-3.4 typical; see fit coupon)

# JGA25-371 25 mm L-bracket (TechMakers drawing; verify with the fit coupon)
BRK_FACE_W = 26.5         # bracket width (along Y)
BRK_FACE_H = 34.0         # face height incl. flange thickness
BRK_T = 3.3               # bracket sheet-metal thickness
BRK_AXIS_FROM_FLANGE = 20.75  # motor axis to flange outer surface
BRK_FLANGE_DEPTH = 33.0   # flange length from face outer surface (along X)
BRK_HOLE_ROWS = (10.85, 24.85)  # flange hole rows, measured inward from face outer surface
BRK_HOLE_SPACING = 16.0   # flange hole columns (along Y)
BRK_SLOT_EXTRA = 3.0      # slots elongated by this much along X for tolerance

# JGA25-371 1:9.6 gearmotor (listing images)
GEARBOX_D, GEARBOX_L = 25.0, 17.5
MOTOR_D, MOTOR_L = 24.4, 31.0
ENCODER_L = 5.0           # encoder disc incl. PCB
CONNECTOR = (8.0, 14.0, 7.0)  # 6-pin header box: along X, along Y, height above can
PLUG_H = 10.0             # mating plug + wire bend above the header (needs the wire slot)
MOTOR_END_GAP_MIN = 12.0  # free space between opposing motor ends (power tabs, wires)
WHEEL_D, WHEEL_W = 65.0, 26.0
WHEEL_GAP = 5.0           # bracket face outer surface -> wheel inner face (screw heads + coupling)

# Maker-ESP32 v1.7x (from the board's official STEP model)
BOARD_W, BOARD_L, BOARD_T = 56.0, 80.0, 1.6   # board is mounted long-side along Y
BOARD_HOLES = [(4, 4), (4, 52), (76, 4), (76, 52)]  # (along length, along width) from corner
BOARD_HOLE_D = M3_CLEAR   # board holes are 4.65; M3 nylon screws are plenty
BOARD_CENTER_Y = -50.0    # board sits at the rear: USB / DC jack end faces backwards
BOARD_COMPONENT_H = 14.0  # tallest parts (DC jack, headers) for clearance checks

# Battery bay: sized for a 3S 2200 mAh LiPo (~105 x 34 x 25 mm); straps take up to ~50 mm wide packs
BATTERY = (34.0, 105.0, 26.0)   # X, Y, Z
STRAP_W = 20.0            # hook-and-loop strap width
STRAP_SLOT = (3.5, STRAP_W + 2.0)
STRAP_X = 27.0            # slot centre distance from centre line
STRAP_Y = (-30.0, 30.0)

GROUND_CLEARANCE_MIN = 10.0

# =============================================================================
# 2D feature model (single source for DXF, SVG, PDF and 3D)
# =============================================================================
CUT, ENGRAVE = "CUT", "ENGRAVE"


@dataclass
class Circle:
    x: float
    y: float
    d: float
    layer: str = CUT


@dataclass
class Slot:  # stadium between two centres
    x: float
    y: float
    length: float  # centre-to-centre distance
    width: float
    angle: float = 0.0  # degrees, 0 = along X
    layer: str = CUT


@dataclass
class RRect:
    x: float
    y: float
    w: float
    h: float
    r: float
    layer: str = CUT


@dataclass
class Polyline:
    points: list
    closed: bool = False
    layer: str = ENGRAVE


@dataclass
class Plate:
    name: str
    w: float
    h: float
    r: float
    features: list = field(default_factory=list)

    def add(self, *f):
        self.features.extend(f)


def motor_positions():
    """(side, y) for each motor: side = -1 left, +1 right."""
    return [(s, y) for y in (WHEELBASE / 2, -WHEELBASE / 2) for s in (-1, 1)]


def motor_inner_end_x():
    """|x| of the encoder end of each motor."""
    return PLATE_W / 2 - BRK_T - GEARBOX_L - MOTOR_L - ENCODER_L


def standoff_positions():
    xs = PLATE_W / 2 - 8.0
    ys = PLATE_L / 2 - 8.0
    return [(sx * xs, sy * ys) for sx in (-1, 1) for sy in (-1, 1)] + [(-xs, 0.0), (xs, 0.0)]


def board_hole_positions():
    out = []
    for along, across in BOARD_HOLES:
        y = BOARD_CENTER_Y - BOARD_L / 2 + along
        x = -BOARD_W / 2 + across
        out.append((x, y))
    return out


def zip_tie_pair(x, y, angle=0.0):
    """Two small slots for a 3-4 mm cable tie, centred on (x, y)."""
    a = math.radians(angle)
    dx, dy = -math.sin(a) * 4.0, math.cos(a) * 4.0
    return [Slot(x + dx, y + dy, 1.5, 3.0, angle), Slot(x - dx, y - dy, 1.5, 3.0, angle)]


def front_arrow(x0, y0):
    """Engraved arrow pointing forward, centred on (x0, y0), 20 mm tall."""
    pts = [(-8, 0), (0, 10), (8, 0), (3, 0), (3, -10), (-3, -10), (-3, 0)]
    return Polyline([(x0 + x, y0 + y) for x, y in pts], True)


def make_bottom_plate() -> Plate:
    p = Plate("bottom_plate", PLATE_W, PLATE_L, CORNER_R)
    # Motor brackets (flange on the underside, face flush with the plate edge)
    for side, y in motor_positions():
        for row in BRK_HOLE_ROWS:
            x = side * (PLATE_W / 2 - row)
            for dy in (-BRK_HOLE_SPACING / 2, BRK_HOLE_SPACING / 2):
                p.add(Slot(x, y + dy, BRK_SLOT_EXTRA, M3_CLEAR, 0.0))
    # Wire pass-throughs above the encoder ends of each motor pair: wide
    # enough for both encoder plugs (pointing up) to pass through.
    conn_outer = motor_inner_end_x() + CONNECTOR[0] + 0.5
    slot_w = 2 * (conn_outer + 3.0)
    for y in (WHEELBASE / 2, -WHEELBASE / 2):
        p.add(RRect(0.0, y, slot_w, CONNECTOR[1] + 6.0, 4.0))
        p.add(*zip_tie_pair(0.0, y + (CONNECTOR[1] + 6.0) / 2 + 6.0, 90.0))
    # Battery straps
    for y in STRAP_Y:
        for sx in (-1, 1):
            p.add(RRect(sx * STRAP_X, y, STRAP_SLOT[0], STRAP_SLOT[1], STRAP_SLOT[0] / 2 - 0.01))
    # Deck standoffs
    for x, y in standoff_positions():
        p.add(Circle(x, y, M3_CLEAR))
    # Accessory holes front & rear (bumper, line sensor, caster...)
    for y in (PLATE_L / 2 - 8.0, -PLATE_L / 2 + 8.0):
        for x in (-40.0, -20.0, 0.0, 20.0, 40.0):
            p.add(Circle(x, y, M3_CLEAR))
    # Visible beside the battery, clear of every cut-out (checked in run_checks)
    p.add(front_arrow(-50.0, 25.0))
    return p


def make_top_plate() -> Plate:
    p = Plate("top_plate", PLATE_W, PLATE_L, CORNER_R)
    for x, y in standoff_positions():
        p.add(Circle(x, y, M3_CLEAR))
    for x, y in board_hole_positions():
        p.add(Circle(x, y, BOARD_HOLE_D))
    # Cable slots either side of the board (motor ports / encoder headers)
    for sx in (-1, 1):
        p.add(RRect(sx * (BOARD_W / 2 + 12.0), BOARD_CENTER_Y, 10.0, 56.0, 4.0))
        p.add(*zip_tie_pair(sx * (BOARD_W / 2 + 24.0), BOARD_CENTER_Y + 18.0, 90.0))
        p.add(*zip_tie_pair(sx * (BOARD_W / 2 + 24.0), BOARD_CENTER_Y - 18.0, 90.0))
    # Accessory grid: M3 holes on 10 mm pitch in the free front area
    y_start = BOARD_CENTER_Y + BOARD_L / 2 + 30.0
    keep_out = [(x, y) for x, y in standoff_positions()]
    y = y_start
    while y <= PLATE_L / 2 - 10.0 + 1e-6:
        for i in range(-6, 7):
            x = i * 10.0
            if abs(x) > PLATE_W / 2 - 10.0:
                continue
            if any(math.hypot(x - sx, y - sy) < 8.0 for sx, sy in keep_out):
                continue
            p.add(Circle(x, y, M3_CLEAR))
        y += 10.0
    p.add(front_arrow(0.0, BOARD_CENTER_Y + BOARD_L / 2 + 14.0))  # just in front of the board
    return p


def make_coupon() -> Plate:
    """Cut this first (costs pennies): checks bracket hole pattern and M3 hole fit."""
    p = Plate("fit_test_coupon", 46.0, 56.0, 3.0)
    # One bracket pattern; bracket face edge = coupon edge at x = +23
    edge = 23.0
    for row in BRK_HOLE_ROWS:
        for dy in (-BRK_HOLE_SPACING / 2, BRK_HOLE_SPACING / 2):
            p.add(Slot(edge - row, 8.0 + dy, BRK_SLOT_EXTRA, M3_CLEAR, 0.0))
    # Hole-size ladder along the bottom: 3.0 / 3.2 / 3.4 / 3.6
    for i, d in enumerate((3.0, 3.2, 3.4, 3.6)):
        p.add(Circle(-15.0 + i * 10.0, -20.0, d))
    # 40 mm reference line to check scale with calipers
    p.add(Polyline([(-20.0, -10.0), (20.0, -10.0)], False))
    p.add(Polyline([(-20.0, -12.0), (-20.0, -8.0)], False), Polyline([(20.0, -12.0), (20.0, -8.0)], False))
    return p


# =============================================================================
# 2D geometry helpers
# =============================================================================
def slot_geometry(s: Slot):
    a = math.radians(s.angle)
    ux, uy = math.cos(a), math.sin(a)
    h = s.length / 2
    c1 = (s.x - ux * h, s.y - uy * h)
    c2 = (s.x + ux * h, s.y + uy * h)
    return c1, c2, s.width / 2, a


def rrect_corners(x, y, w, h, r):
    r = min(r, w / 2 - 1e-6, h / 2 - 1e-6)
    return [(x + w / 2 - r, y + h / 2 - r, 0), (x - w / 2 + r, y + h / 2 - r, 90),
            (x - w / 2 + r, y - h / 2 + r, 180), (x + w / 2 - r, y - h / 2 + r, 270)], r


# =============================================================================
# DXF
# =============================================================================
def write_dxf(plate: Plate, path: Path):
    import ezdxf

    doc = ezdxf.new("R2010", setup=False)
    doc.units = ezdxf.units.MM
    doc.header["$INSUNITS"] = 4
    doc.layers.add(CUT, color=1)      # red
    doc.layers.add(ENGRAVE, color=5)  # blue
    msp = doc.modelspace()

    def rrect(x, y, w, h, r, layer):
        corners, r = rrect_corners(x, y, w, h, r)
        for i, (cx, cy, start) in enumerate(corners):
            msp.add_arc((cx, cy), r, start, start + 90, dxfattribs={"layer": layer})
            nx, ny, nstart = corners[(i + 1) % 4]
            a_end = math.radians(start + 90)
            b_start = math.radians(nstart)
            msp.add_line((cx + r * math.cos(a_end), cy + r * math.sin(a_end)),
                         (nx + r * math.cos(b_start), ny + r * math.sin(b_start)), dxfattribs={"layer": layer})

    rrect(0, 0, plate.w, plate.h, plate.r, CUT)
    for f in plate.features:
        if isinstance(f, Circle):
            msp.add_circle((f.x, f.y), f.d / 2, dxfattribs={"layer": f.layer})
        elif isinstance(f, RRect):
            rrect(f.x, f.y, f.w, f.h, f.r, f.layer)
        elif isinstance(f, Slot):
            c1, c2, r, a = slot_geometry(f)
            deg = math.degrees(a)
            msp.add_arc(c2, r, deg - 90, deg + 90, dxfattribs={"layer": f.layer})
            msp.add_arc(c1, r, deg + 90, deg + 270, dxfattribs={"layer": f.layer})
            nx, ny = -math.sin(a) * r, math.cos(a) * r
            msp.add_line((c1[0] + nx, c1[1] + ny), (c2[0] + nx, c2[1] + ny), dxfattribs={"layer": f.layer})
            msp.add_line((c1[0] - nx, c1[1] - ny), (c2[0] - nx, c2[1] - ny), dxfattribs={"layer": f.layer})
        elif isinstance(f, Polyline):
            msp.add_lwpolyline(f.points, close=f.closed, dxfattribs={"layer": f.layer})
    doc.saveas(path)


# =============================================================================
# SVG (mm units, hairline strokes: red = cut, blue = engrave)
# =============================================================================
def svg_paths(plate: Plate):
    """Returns list of (layer, path_d) in plate coordinates with Y up."""
    out = []

    def rrect_d(x, y, w, h, r):
        corners, r = rrect_corners(x, y, w, h, r)
        d = ""
        for i, (cx, cy, start) in enumerate(corners):
            a0, a1 = math.radians(start), math.radians(start + 90)
            p0 = (cx + r * math.cos(a0), cy + r * math.sin(a0))
            p1 = (cx + r * math.cos(a1), cy + r * math.sin(a1))
            d += (f"M{p0[0]:.3f},{p0[1]:.3f}" if i == 0 else f"L{p0[0]:.3f},{p0[1]:.3f}")
            d += f"A{r:.3f},{r:.3f} 0 0 1 {p1[0]:.3f},{p1[1]:.3f}"
        return d + "Z"

    out.append((CUT, rrect_d(0, 0, plate.w, plate.h, plate.r)))
    for f in plate.features:
        if isinstance(f, Circle):
            r = f.d / 2
            out.append((f.layer, f"M{f.x + r:.3f},{f.y:.3f}A{r:.3f},{r:.3f} 0 1 1 {f.x - r:.3f},{f.y:.3f}"
                                 f"A{r:.3f},{r:.3f} 0 1 1 {f.x + r:.3f},{f.y:.3f}Z"))
        elif isinstance(f, RRect):
            out.append((f.layer, rrect_d(f.x, f.y, f.w, f.h, f.r)))
        elif isinstance(f, Slot):
            c1, c2, r, a = slot_geometry(f)
            nx, ny = -math.sin(a) * r, math.cos(a) * r
            out.append((f.layer, f"M{c1[0] + nx:.3f},{c1[1] + ny:.3f}L{c2[0] + nx:.3f},{c2[1] + ny:.3f}"
                                 f"A{r:.3f},{r:.3f} 0 0 0 {c2[0] - nx:.3f},{c2[1] - ny:.3f}"
                                 f"L{c1[0] - nx:.3f},{c1[1] - ny:.3f}A{r:.3f},{r:.3f} 0 0 0 {c1[0] + nx:.3f},{c1[1] + ny:.3f}Z"))
        elif isinstance(f, Polyline):
            d = "M" + "L".join(f"{x:.3f},{y:.3f}" for x, y in f.points) + ("Z" if f.closed else "")
            out.append((f.layer, d))
    return out


def write_svg(plate: Plate, path: Path):
    margin = 2.0
    w, h = plate.w + 2 * margin, plate.h + 2 * margin
    colours = {CUT: "#ff0000", ENGRAVE: "#0000ff"}
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.3f}mm" height="{h:.3f}mm" viewBox="{-w / 2:.3f} {-h / 2:.3f} {w:.3f} {h:.3f}">',
        f"<!-- {plate.name}: {plate.w:g} x {plate.h:g} mm, {ACRYLIC_T:g} mm acrylic. red = cut, blue = engrave/score -->",
        '<g transform="scale(1,-1)" fill="none" stroke-width="0.01">',
    ]
    for layer, d in svg_paths(plate):
        lines.append(f'<path stroke="{colours[layer]}" d="{d}"/>')
    lines += ["</g>", "</svg>"]
    path.write_text("\n".join(lines) + "\n")


# =============================================================================
# 1:1 PDF print template (Letter and A4 both fit)
# =============================================================================
def write_pdf(plates, path: Path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib.path import Path as MPath
    from matplotlib.patches import PathPatch

    page_w, page_h = 210.0, 279.4  # fits both A4 (210x297) and Letter (215.9x279.4)
    inch = 25.4


    with PdfPages(path) as pdf:
        for plate in plates:
            fig = plt.figure(figsize=(page_w / inch, page_h / inch))
            ax = fig.add_axes([0, 0, 1, 1])
            ax.set_xlim(-page_w / 2, page_w / 2)
            ax.set_ylim(-page_h / 2, page_h / 2)
            ax.set_aspect("equal")
            ax.axis("off")
            for layer, pts_list in tessellate(plate):
                for pts in pts_list:
                    xs, ys = zip(*pts)
                    ax.plot(xs, ys, color="red" if layer == CUT else "blue", linewidth=0.4)
            # Scale check bar: 100 mm
            y0 = -page_h / 2 + 12
            ax.plot([-50, 50], [y0, y0], color="black", linewidth=0.8)
            for x in (-50, 50):
                ax.plot([x, x], [y0 - 2, y0 + 2], color="black", linewidth=0.8)
            ax.text(0, y0 + 3, "this bar must measure exactly 100 mm", ha="center", fontsize=7)
            ax.text(0, page_h / 2 - 8, f"{plate.name} - 1:1 template - print at 100 % / 'actual size', no 'fit to page'",
                    ha="center", fontsize=8)
            pdf.savefig(fig)
            plt.close(fig)


def tessellate(plate: Plate, seg=48):
    """Polylines for each feature (for PDF and the preview)."""
    def arc(cx, cy, r, a0, a1, n):
        return [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]

    def rrect_pts(x, y, w, h, r):
        corners, r = rrect_corners(x, y, w, h, r)
        pts = []
        for cx, cy, start in corners:
            pts += arc(cx, cy, r, math.radians(start), math.radians(start + 90), 8)
        return pts + [pts[0]]

    out = {CUT: [], ENGRAVE: []}
    out[CUT].append(rrect_pts(0, 0, plate.w, plate.h, plate.r))
    for f in plate.features:
        if isinstance(f, Circle):
            out[f.layer].append(arc(f.x, f.y, f.d / 2, 0, 2 * math.pi, seg))
        elif isinstance(f, RRect):
            out[f.layer].append(rrect_pts(f.x, f.y, f.w, f.h, f.r))
        elif isinstance(f, Slot):
            c1, c2, r, a = slot_geometry(f)
            pts = arc(c2[0], c2[1], r, a - math.pi / 2, a + math.pi / 2, 12) + arc(c1[0], c1[1], r, a + math.pi / 2, a + 3 * math.pi / 2, 12)
            out[f.layer].append(pts + [pts[0]])
        elif isinstance(f, Polyline):
            pts = list(f.points) + ([f.points[0]] if f.closed else [])
            out[f.layer].append(pts)
    return list(out.items())


# =============================================================================
# 3D model + checks
# =============================================================================
def plate_solid(plate: Plate, z0: float):
    import cadquery as cq

    sk = cq.Sketch().rect(plate.w, plate.h).vertices().fillet(plate.r)
    for f in plate.features:
        if f.layer != CUT:
            continue
        if isinstance(f, Circle):
            sk = sk.push([(f.x, f.y)]).circle(f.d / 2, mode="s").reset()
        elif isinstance(f, RRect):
            sk = sk.push([(f.x, f.y)]).rect(f.w, f.h, mode="s").reset()  # corner radius irrelevant for checks
        elif isinstance(f, Slot):
            sk = sk.push([(f.x, f.y)]).slot(f.length, f.width, angle=f.angle, mode="s").reset()
    return cq.Workplane("XY").workplane(offset=z0).placeSketch(sk).extrude(ACRYLIC_T)


def build_assembly(bottom: Plate, top: Plate):
    import cadquery as cq

    parts = {}  # name -> (workplane, colour)
    axis_z = -BRK_AXIS_FROM_FLANGE
    half_w = PLATE_W / 2

    parts["bottom_plate"] = (plate_solid(bottom, 0.0), (0.75, 0.88, 1.0, 0.55))
    top_z = ACRYLIC_T + STANDOFF_H
    parts["top_plate"] = (plate_solid(top, top_z), (0.75, 0.88, 1.0, 0.55))

    def cyl_x(d, x0, x1, y, z):
        return cq.Workplane("YZ").workplane(offset=min(x0, x1)).center(y, z).circle(d / 2).extrude(abs(x1 - x0))

    for i, (side, y) in enumerate(motor_positions()):
        tag = f"{'L' if side < 0 else 'R'}{'F' if y > 0 else 'B'}"
        s = side
        face_out = s * half_w
        face_in = s * (half_w - BRK_T)
        # bracket: flange under the plate + vertical face
        flange = cq.Workplane("XY").box(BRK_FLANGE_DEPTH, BRK_FACE_W, BRK_T).translate(
            (s * (half_w - BRK_FLANGE_DEPTH / 2), y, -BRK_T / 2))
        face = cq.Workplane("XY").box(BRK_T, BRK_FACE_W, BRK_FACE_H - BRK_T).translate(
            (s * (half_w - BRK_T / 2), y, -BRK_T - (BRK_FACE_H - BRK_T) / 2))
        face = face.cut(cyl_x(8.0, face_out - s * 5, face_out + s * 5, y, axis_z))
        parts[f"bracket_{tag}"] = (flange.union(face), (0.15, 0.15, 0.15, 1.0))
        # flange screws: nut + screw tail under the flange (M3 x 10 from above)
        hw = None
        for row in BRK_HOLE_ROWS:
            for dy in (-BRK_HOLE_SPACING / 2, BRK_HOLE_SPACING / 2):
                x = s * (half_w - row)
                nut = cq.Workplane("XY").center(x, y + dy).polygon(6, 6.35).extrude(-2.4).translate((0, 0, -BRK_T))
                tail = cq.Workplane("XY").center(x, y + dy).circle(1.5).extrude(-(10.0 - ACRYLIC_T - BRK_T)).translate((0, 0, -BRK_T))
                piece = nut.union(tail)
                hw = piece if hw is None else hw.union(piece)
        parts[f"bracket_screws_{tag}"] = (hw, (0.6, 0.6, 0.6, 1.0))
        # motor (inward from the face)
        x0 = face_in
        x1 = x0 - s * GEARBOX_L
        x2 = x1 - s * MOTOR_L
        x3 = x2 - s * ENCODER_L
        motor = cyl_x(GEARBOX_D, x0, x1, y, axis_z).union(cyl_x(MOTOR_D, x1, x2, y, axis_z)).union(cyl_x(MOTOR_D, x2, x3, y, axis_z))
        # 6-pin header on the encoder PCB pointing up, plus the mating plug and
        # wire bend above it (rotate the motor in its bracket so it points up).
        conn_h = CONNECTOR[2] + PLUG_H
        conn_z0 = axis_z + MOTOR_D / 2 - 4.0
        conn = cq.Workplane("XY").box(CONNECTOR[0], CONNECTOR[1], conn_h).translate(
            (x3 + s * (CONNECTOR[0] / 2 + 0.5), y, conn_z0 + conn_h / 2))
        parts[f"motor_{tag}"] = (motor.union(conn), (0.8, 0.8, 0.82, 1.0))
        # wheel
        w0 = face_out + s * WHEEL_GAP
        w1 = w0 + s * WHEEL_W
        parts[f"wheel_{tag}"] = (cyl_x(WHEEL_D, w0, w1, y, axis_z), (0.2, 0.2, 0.22, 1.0))

    # standoffs between decks
    for i, (x, y) in enumerate(standoff_positions()):
        so = cq.Workplane("XY").center(x, y).polygon(6, 6.35).extrude(STANDOFF_H).translate((0, 0, ACRYLIC_T))
        parts[f"standoff_{i}"] = (so, (0.85, 0.7, 0.3, 1.0))

    # battery on the bottom deck
    bx, by, bz = BATTERY
    parts["battery"] = (cq.Workplane("XY").box(bx, by, bz).translate((0, 0, ACRYLIC_T + bz / 2)), (0.2, 0.35, 0.8, 1.0))

    # board on nylon standoffs on the top deck
    board_z = top_z + ACRYLIC_T + BOARD_STANDOFF_H
    board = cq.Workplane("XY").box(BOARD_W, BOARD_L, BOARD_T).translate((0, BOARD_CENTER_Y, board_z + BOARD_T / 2))
    comps = cq.Workplane("XY").box(BOARD_W - 4, BOARD_L - 4, BOARD_COMPONENT_H).translate(
        (0, BOARD_CENTER_Y, board_z + BOARD_T + BOARD_COMPONENT_H / 2))
    parts["maker_esp32"] = (board, (0.1, 0.45, 0.3, 1.0))
    parts["board_components"] = (comps, (0.25, 0.25, 0.3, 0.6))
    for i, (x, y) in enumerate(board_hole_positions()):
        so = cq.Workplane("XY").center(x, y).polygon(6, 5.5).extrude(BOARD_STANDOFF_H).translate((0, 0, top_z + ACRYLIC_T))
        parts[f"board_standoff_{i}"] = (so, (0.95, 0.95, 0.9, 1.0))
    return parts


def run_checks(parts, bottom: Plate, top: Plate):
    problems = []
    names = list(parts)
    solids = {n: parts[n][0].val() for n in names}
    # 1. no two parts may overlap
    for i, a in enumerate(names):
        bba = solids[a].BoundingBox()
        for b in names[i + 1:]:
            bbb = solids[b].BoundingBox()
            if (bba.xmax < bbb.xmin or bbb.xmax < bba.xmin or bba.ymax < bbb.ymin or bbb.ymax < bba.ymin
                    or bba.zmax < bbb.zmin or bbb.zmax < bba.zmin):
                continue
            vol = solids[a].intersect(solids[b]).Volume()
            if vol > 0.01:
                problems.append(f"COLLISION {a} <-> {b}: {vol:.1f} mm^3")
    # 2. ground clearance (everything except wheels)
    ground = -BRK_AXIS_FROM_FLANGE - WHEEL_D / 2
    lowest = min((solids[n].BoundingBox().zmin, n) for n in names if not n.startswith("wheel"))
    clearance = lowest[0] - ground
    if clearance < GROUND_CLEARANCE_MIN:
        problems.append(f"ground clearance {clearance:.1f} mm at {lowest[1]} (< {GROUND_CLEARANCE_MIN})")
    # 3. every feature inside its plate with >= 2 mm of material to the edge
    for plate in (bottom, top):
        for layer, polys in tessellate(plate):
            if layer != CUT:
                continue
            for poly in polys[1:]:
                for x, y in poly:
                    if abs(x) > plate.w / 2 - 2.0 or abs(y) > plate.h / 2 - 2.0:
                        problems.append(f"{plate.name}: feature at ({x:.1f},{y:.1f}) too close to the edge")
                        break
    # 4b. engravings must not run over cut-outs (they'd be cut away)
    for plate in (bottom, top, make_coupon()):
        cut_boxes = []
        for layer, polys in tessellate(plate):
            if layer == CUT:
                for poly in polys[1:]:
                    xs, ys = zip(*poly)
                    cut_boxes.append((min(xs) - 1, min(ys) - 1, max(xs) + 1, max(ys) + 1))
        for f in plate.features:
            if isinstance(f, Polyline) and f.layer == ENGRAVE:
                pts = list(f.points) + ([f.points[0]] if f.closed else [])
                samples = [(a[0] + (b[0] - a[0]) * t / 10, a[1] + (b[1] - a[1]) * t / 10)
                           for a, b in zip(pts, pts[1:]) for t in range(11)]
                if any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in samples for x0, y0, x1, y1 in cut_boxes):
                    problems.append(f"{plate.name}: engraving near ({pts[0][0]:.0f},{pts[0][1]:.0f}) overlaps a cut-out")
    # 4. minimum web between holes (acrylic cracks below ~2 mm)
    for plate in (bottom, top):
        circles = [f for f in plate.features if isinstance(f, Circle) and f.layer == CUT]
        for i, a in enumerate(circles):
            for b in circles[i + 1:]:
                web = math.hypot(a.x - b.x, a.y - b.y) - (a.d + b.d) / 2
                if web < 2.0:
                    problems.append(f"{plate.name}: holes at ({a.x},{a.y}) and ({b.x},{b.y}) only {web:.1f} mm apart")
    # 5. room behind the motors for their power tabs and wires
    end_gap = 2 * motor_inner_end_x()
    if end_gap < MOTOR_END_GAP_MIN:
        problems.append(f"only {end_gap:.1f} mm between opposing motor ends (< {MOTOR_END_GAP_MIN})")
    # 6. battery fits between strap slots and under the top deck
    if BATTERY[0] / 2 > STRAP_X - STRAP_SLOT[0] / 2 - 0.5:
        problems.append("battery wider than the strap slots allow")
    return problems, clearance


def write_step(parts, path: Path):
    import cadquery as cq

    assy = cq.Assembly(name="esp32_robot_chassis")
    for name, (wp, rgba) in parts.items():
        assy.add(wp, name=name, color=cq.Color(*rgba))
    assy.export(str(path))


def write_glb(parts, path: Path):
    """glTF binary for the web viewer (viewer.html) and renders."""
    import cadquery as cq

    assy = cq.Assembly(name="esp32_robot_chassis")
    for name, (wp, rgba) in parts.items():
        assy.add(wp, name=name, color=cq.Color(*rgba))
    assy.export(str(path), tolerance=0.05, angularTolerance=0.2)


def main():
    out = Path(__file__).resolve().parent / "out"
    out.mkdir(exist_ok=True)
    bottom, top, coupon = make_bottom_plate(), make_top_plate(), make_coupon()

    parts = build_assembly(bottom, top)
    problems, clearance = run_checks(parts, bottom, top)
    track = PLATE_W + 2 * (WHEEL_GAP + WHEEL_W / 2)
    print(f"plates {PLATE_W:g} x {PLATE_L:g} mm, wheelbase {WHEELBASE:g} mm, track (wheel centres) {track:g} mm")
    print(f"ground clearance {clearance:.1f} mm, deck gap {STANDOFF_H:g} mm, "
          f"overall {track + WHEEL_W:g} W x {max(PLATE_L, WHEELBASE + WHEEL_D):g} L mm")
    if problems:
        print("\nDESIGN CHECK FAILED:")
        for p in problems:
            print("  -", p)
        sys.exit(1)
    print("design checks: no collisions, clearances OK")

    for plate in (bottom, top, coupon):
        write_dxf(plate, out / f"{plate.name}.dxf")
        write_svg(plate, out / f"{plate.name}.svg")
    write_pdf([bottom, top, coupon], out / "print_template_1to1.pdf")
    write_step(parts, out / "chassis_assembly.step")
    write_glb(parts, out / "chassis_assembly.glb")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
