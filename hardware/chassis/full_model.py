#!/usr/bin/env python3
"""
Detailed, all-solid 3D model of the complete robot.

chassis.py keeps deliberately simple shapes (boxes and cylinders) for its
collision and physics checks. This script builds the same robot - every
position and dimension comes from chassis.py, so the two can't drift apart -
with realistic parts:

  * both acrylic plates with all their holes
  * JGA25 L-brackets (D-shaped face, motor and flange holes)
  * JGA25-371 motors: gearbox, D-shaft, can, encoder PCB, 6-pin header + plug
    and the six encoder/motor wires running up through the wire slots
  * 65 mm wheels: treaded tyre, spoked rim, brass hex coupling
  * brass M3 x 40 deck standoffs, nylon board standoffs, all screws and nuts
  * the 3S LiPo with its straps and lead up to the DC jack
  * the Maker-ESP32 itself, from NULLLAB's official STEP model (every component)

Outputs (out/):
  robot_full.step   solid bodies, one named + coloured part each: opens in any
                    CAD program (SolidWorks, Fusion 360, Onshape, FreeCAD, Shapr3D...)
  robot_full.stl    single triangle mesh for 3D viewers, slicers and simple apps
  robot_full.glb    for viewer.html

Usage:
  python full_model.py [path/to/maker-esp32.step]
The board model is the path given, or else downloaded (once, checksum-verified)
from a pinned revision of github.com/nulllaborg/maker-esp32.

The script also checks the real board model against the plates, straps and
battery lead, and refuses to write files if anything collides.
"""
from __future__ import annotations

import hashlib
import math
import sys
import urllib.request
from pathlib import Path

import cadquery as cq

import chassis as C

HERE = Path(__file__).resolve().parent
# Pinned to the exact revision this model was verified with; the checksum
# guards against the vendor silently changing the file.
BOARD_STEP_COMMIT = "ac8312a9f0112df1a102d355afaab9a18095f6ea"
BOARD_STEP_URL = f"https://raw.githubusercontent.com/nulllaborg/maker-esp32/{BOARD_STEP_COMMIT}/maker-esp32.step"
BOARD_STEP_SHA256 = "4aa2532d7529cae74760495fbb065082ebda870340544e933ddb7413fcd84e04"

# Colours (r, g, b, a)
ACRYLIC = (0.80, 0.90, 1.00, 0.55)
BLACK = (0.10, 0.10, 0.11, 1.0)
STEEL = (0.62, 0.64, 0.67, 1.0)
DARK_STEEL = (0.25, 0.26, 0.28, 1.0)
BRASS = (0.80, 0.65, 0.30, 1.0)
NYLON = (0.93, 0.92, 0.88, 1.0)
GEARBOX = (0.78, 0.79, 0.80, 1.0)
CAN = (0.70, 0.72, 0.75, 1.0)
PCB_RED = (0.75, 0.12, 0.12, 1.0)
HEADER_WHITE = (0.95, 0.95, 0.93, 1.0)
TYRE = (0.13, 0.13, 0.14, 1.0)
RIM_BLUE = (0.10, 0.45, 0.85, 1.0)
LIPO = (0.12, 0.22, 0.55, 1.0)
XT60 = (0.95, 0.75, 0.10, 1.0)
WIRE = {"red": (0.85, 0.1, 0.1, 1.0), "black": (0.08, 0.08, 0.08, 1.0), "white": (0.95, 0.95, 0.95, 1.0),
        "blue": (0.1, 0.3, 0.9, 1.0), "yellow": (0.95, 0.85, 0.1, 1.0), "green": (0.1, 0.65, 0.2, 1.0)}

M3_HEAD_D, M3_HEAD_H = 5.5, 2.1
NUT_AF, NUT_H = 5.5, 2.4
HEX = lambda af: af / math.cos(math.radians(30))  # across-flats -> polygon diameter


# ---------------------------------------------------------------------------
# primitives
# ---------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)).translate(
        ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl_x(d, x0, x1, y, z):
    return cq.Workplane("YZ").workplane(offset=min(x0, x1)).center(y, z).circle(d / 2).extrude(abs(x1 - x0))


def cyl_z(d, x, y, z0, z1):
    return cq.Workplane("XY").workplane(offset=min(z0, z1)).center(x, y).circle(d / 2).extrude(abs(z1 - z0))


def hex_z(af, x, y, z0, z1, hole=0.0):
    w = cq.Workplane("XY").workplane(offset=min(z0, z1)).center(x, y).polygon(6, HEX(af)).extrude(abs(z1 - z0))
    return w.cut(cyl_z(hole, x, y, z0 - 1, z1 + 1)) if hole else w


def hex_x(af, x0, x1, y, z, hole=0.0):
    w = cq.Workplane("YZ").workplane(offset=min(x0, x1)).center(y, z).polygon(6, HEX(af)).extrude(abs(x1 - x0))
    return w.cut(cyl_x(hole, x0 - 1, x1 + 1, y, z)) if hole else w


def pan_screw_z(x, y, z_surface, up, length):
    """M3 pan-head screw whose head sits on z_surface; up=True: head above."""
    s = 1 if up else -1
    head = cyl_z(M3_HEAD_D, x, y, z_surface, z_surface + s * M3_HEAD_H)
    shank = cyl_z(3.0, x, y, z_surface, z_surface - s * length)
    return head.union(shank)


def wire_path(points, d=1.2):
    """Wire as straight cylinder segments through the given 3D points."""
    out = None
    for (x0, y0, z0), (x1, y1, z1) in zip(points, points[1:]):
        v = cq.Vector(x1 - x0, y1 - y0, z1 - z0)
        seg = cq.Workplane(cq.Plane(origin=(x0, y0, z0), xDir=_perp(v), normal=v)).circle(d / 2).extrude(v.Length)
        joint = cq.Workplane("XY").sphere(d / 2).translate((x1, y1, z1))
        piece = seg.union(joint)
        out = piece if out is None else out.union(piece)
    return out


def _boxes_overlap(a, b):
    return not (a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin
                or a.zmax < b.zmin or b.zmax < a.zmin)


def _perp(v):
    a = cq.Vector(1, 0, 0) if abs(v.normalized().x) < 0.9 else cq.Vector(0, 1, 0)
    return v.cross(a).normalized()


# ---------------------------------------------------------------------------
# parts
# ---------------------------------------------------------------------------
AXIS_Z = -C.BRK_AXIS_FROM_FLANGE
HALF = C.PLATE_W / 2
FACE_SCREW_SPACING = 17.0  # bracket face holes (TechMakers drawing)


def bracket(s, y):
    T, W = C.BRK_T, C.BRK_FACE_W
    r = W / 2
    flange = box(s * (HALF - C.BRK_FLANGE_DEPTH), s * HALF, y - r, y + r, -T, 0)
    # D-shaped face: straight down to the axis, then a half-circle around it
    face = box(s * (HALF - T), s * HALF, y - r, y + r, AXIS_Z, -T).union(cyl_x(W, s * (HALF - T), s * HALF, y, AXIS_Z))
    b = flange.union(face)
    b = b.cut(cyl_x(8.0, s * (HALF - T - 1), s * (HALF + 1), y, AXIS_Z))
    for dy in (-FACE_SCREW_SPACING / 2, FACE_SCREW_SPACING / 2):
        b = b.cut(cyl_x(3.2, s * (HALF - T - 1), s * (HALF + 1), y + dy, AXIS_Z))
    for row in C.BRK_HOLE_ROWS:
        for dy in (-C.BRK_HOLE_SPACING / 2, C.BRK_HOLE_SPACING / 2):
            b = b.cut(cyl_z(3.3, s * (HALF - row), y + dy, -T - 1, 1))
    b = b.cut(cyl_z(9.8, s * (HALF - 17.8), y, -T - 1, 1))
    return b


def bracket_hardware(s, y):
    """Flange screws (heads on the plate) + nylock nuts, and the 2 motor screws."""
    T = C.BRK_T
    screws = nuts = None
    for row in C.BRK_HOLE_ROWS:
        for dy in (-C.BRK_HOLE_SPACING / 2, C.BRK_HOLE_SPACING / 2):
            x = s * (HALF - row)
            sc = pan_screw_z(x, y + dy, C.ACRYLIC_T, True, 10.0)
            nt = hex_z(NUT_AF, x, y + dy, -T - NUT_H, -T, hole=3.0)
            screws = sc if screws is None else screws.union(sc)
            nuts = nt if nuts is None else nuts.union(nt)
    for dy in (-FACE_SCREW_SPACING / 2, FACE_SCREW_SPACING / 2):
        head = cyl_x(M3_HEAD_D, s * HALF, s * (HALF + M3_HEAD_H), y + dy, AXIS_Z)
        shank = cyl_x(3.0, s * HALF, s * (HALF - T - 4.0), y + dy, AXIS_Z)
        screws = screws.union(head).union(shank)
    return screws, nuts


def motor(s, y):
    """Returns dict of sub-parts with colours."""
    x0 = s * (HALF - C.BRK_T)                 # gearbox front face (against bracket)
    x1 = x0 - s * C.GEARBOX_L
    x2 = x1 - s * C.MOTOR_L
    gear = cyl_x(C.GEARBOX_D, x0, x1, y, AXIS_Z)
    for dy in (-FACE_SCREW_SPACING / 2, FACE_SCREW_SPACING / 2):
        gear = gear.cut(cyl_x(2.5, x0, x0 - s * 5, y + dy, AXIS_Z))
    boss = cyl_x(7.0, x0, x0 + s * 2.5, y, AXIS_Z)
    gear = gear.union(boss)
    shaft = cyl_x(4.0, x0, x0 + s * 12.0, y, AXIS_Z)
    shaft = shaft.cut(box(x0 + s * 3.0, x0 + s * 13.0, y - 3, y + 3, AXIS_Z + 1.5, AXIS_Z + 3))  # D-flat
    can = cyl_x(C.MOTOR_D, x1, x2, y, AXIS_Z)
    can = can.cut(cyl_x(C.MOTOR_D + 2, x1 - s * 2, x1 - s * 2.6, y, AXIS_Z)).union(
        cyl_x(C.MOTOR_D - 1.0, x1 - s * 2, x1 - s * 2.6, y, AXIS_Z))  # seam groove
    magnet = cyl_x(10.0, x2, x2 - s * 2.5, y, AXIS_Z)
    pcb = cyl_x(C.MOTOR_D, x2 - s * 3.4, x2 - s * C.ENCODER_L, y, AXIS_Z)
    # 6-pin header on the encoder PCB pointing up, plug on top of it
    hx0 = x2 - s * C.ENCODER_L
    hx1 = hx0 + s * C.CONNECTOR[0]
    header = box(hx0, hx1, y - C.CONNECTOR[1] / 2, y + C.CONNECTOR[1] / 2,
                 AXIS_Z + C.MOTOR_D / 2 - 4.0, AXIS_Z + C.MOTOR_D / 2 + 3.0)
    plug = box(hx0 + s * 0.8, hx1 - s * 0.8, y - C.CONNECTOR[1] / 2 + 0.8, y + C.CONNECTOR[1] / 2 - 0.8,
               AXIS_Z + C.MOTOR_D / 2 + 3.0, AXIS_Z + C.MOTOR_D / 2 + 8.0)
    # Six wires straight up through the wire slot, then a gentle bend inward
    wires = {}
    xw = (hx0 + hx1) / 2
    z_plug_top = AXIS_Z + C.MOTOR_D / 2 + 8.0
    z_above = C.ACRYLIC_T + 9.0
    toward_board = -1 if y > 0 else 1
    for i, colour in enumerate(("red", "white", "black", "blue", "yellow", "green")):
        yw = y - 5.0 + i * 2.0
        pts = [(xw, yw, z_plug_top), (xw, yw, z_above), (xw * 0.85, yw + toward_board * 8.0, z_above + 4.0)]
        wires[colour] = wire_path(pts)
    return {"gearbox": (gear, GEARBOX), "shaft": (shaft, STEEL), "can": (can, CAN), "magnet": (magnet, BLACK),
            "encoder_pcb": (pcb, PCB_RED), "header": (header, HEADER_WHITE), "plug": (plug, HEADER_WHITE),
            **{f"wire_{k}": (v, WIRE[k]) for k, v in wires.items()}}


def wheel(s, y):
    face_out = s * HALF
    w0 = face_out + s * C.WHEEL_GAP
    w1 = w0 + s * C.WHEEL_W
    rim_d, tyre_d = 50.0, C.WHEEL_D
    tyre = cyl_x(tyre_d, w0, w1, y, AXIS_Z).cut(cyl_x(rim_d, w0 - s, w1 + s, y, AXIS_Z))
    for k in range(4):  # circumferential tread grooves
        xm = w0 + s * (4.0 + k * 6.0)
        groove = cyl_x(tyre_d + 2, xm - s * 0.9, xm + s * 0.9, y, AXIS_Z).cut(cyl_x(tyre_d - 3.0, xm - 2, xm + 2, y, AXIS_Z))
        tyre = tyre.cut(groove)
    try:
        tyre = tyre.faces(f"{'>' if s > 0 else '<'}X or {'<' if s > 0 else '>'}X").edges().fillet(1.2)
    except Exception:
        pass  # cosmetic only
    # rim: outer ring, web with 5 lightening holes, hub
    ring = cyl_x(rim_d, w0 + s * 1, w1 - s * 1, y, AXIS_Z).cut(cyl_x(rim_d - 5, w0, w1, y, AXIS_Z))
    web_x0, web_x1 = w0 + s * 1, w0 + s * 5
    web = cyl_x(rim_d - 4, web_x0, web_x1, y, AXIS_Z)
    for k in range(5):
        a = 2 * math.pi * k / 5
        web = web.cut(cyl_x(11.0, web_x0 - s, web_x1 + s, y + 14.5 * math.cos(a), AXIS_Z + 14.5 * math.sin(a)))
    hub = cyl_x(14.0, w0, w0 + s * 18.0, y, AXIS_Z)
    coupling = hex_x(7.0, face_out + s * 1.5, face_out + s * 19.0, y, AXIS_Z, hole=4.0)
    rim = ring.union(web).union(hub).cut(hex_x(7.0, face_out, face_out + s * 20.0, y, AXIS_Z))
    return {"tyre": (tyre, TYRE), "rim": (rim, RIM_BLUE), "coupling": (coupling, BRASS)}


def deck_standoffs():
    parts = []
    z0, z1 = C.ACRYLIC_T, C.ACRYLIC_T + C.STANDOFF_H
    for i, (x, y) in enumerate(C.standoff_positions()):
        so = hex_z(5.5, x, y, z0, z1, hole=2.5)
        top = pan_screw_z(x, y, z1 + C.ACRYLIC_T, True, 6.0)
        bot = pan_screw_z(x, y, 0.0, False, 6.0)
        parts.append((f"standoff_{i}", so, BRASS))
        parts.append((f"standoff_screws_{i}", top.union(bot), DARK_STEEL))
    return parts


def board_hardware(board_z):
    parts = []
    for i, (x, y) in enumerate(C.board_hole_positions()):
        so = hex_z(5.5, x, y, C.ACRYLIC_T, board_z, hole=2.5)
        top = pan_screw_z(x, y, board_z + C.BOARD_T, True, 5.0)
        bot = pan_screw_z(x, y, 0.0, False, 5.0)
        parts.append((f"board_standoff_{i}", so, NYLON))
        parts.append((f"board_screws_{i}", top.union(bot), NYLON))
    return parts


def battery_and_straps(board_z, dc_jack):
    """Battery under the deck with two strap loops and its lead up to the DC jack."""
    bx, by = C.battery_footprint()
    bz = C.BATTERY[2]
    z_top, z_bot = -0.2, -0.2 - bz
    batt = box(-bx / 2, bx / 2, -by / 2, by / 2, z_bot, z_top)
    try:
        batt = batt.edges("|X").fillet(2.5)
    except Exception:
        pass
    parts = [("battery", batt, LIPO)]
    # strap loops: run over the plate, down through the slots, under the battery
    yo = by / 2 + 3.0 + C.STRAP_SLOT_W / 2
    for i, k in enumerate((-1, 1)):
        xs = k * C.STRAP_OFFSET
        outer = box(xs - C.STRAP_W / 2, xs + C.STRAP_W / 2, -yo - 0.75, yo + 0.75, z_bot - 1.5, C.ACRYLIC_T + 1.5)
        inner = box(xs - C.STRAP_W / 2 - 1, xs + C.STRAP_W / 2 + 1, -yo + 0.75, yo - 0.75, z_bot, C.ACRYLIC_T)
        parts.append((f"strap_{i}", outer.cut(inner), BLACK))
    # lead: from the battery end up through the power slot to a barrel plug in the DC jack
    px, py, _, _ = C.power_slot()
    jx, jy, jz = dc_jack
    plug_x0 = jx - 14.0
    barrel = cyl_x(9.0, plug_x0, jx + 2.0, jy, jz)
    parts.append(("dc_plug", barrel, BLACK))
    xt60 = box(-bx / 2 - 16.0, -bx / 2 - 1.0, -4.0, 4.0, z_top - 12.0, z_top - 4.0)
    parts.append(("xt60", xt60, XT60))
    for colour, dy in (("red", -1.0), ("black", 1.0)):
        pts = [(-bx / 2 - 16.0, dy, z_top - 8.0), (px - 2.0, dy, z_top - 8.0), (px - 2.0, dy, jz),
               (plug_x0, jy + dy, jz)]
        parts.append((f"battery_lead_{colour}", wire_path(pts, d=2.0), WIRE[colour]))
    return parts


# ---------------------------------------------------------------------------
def load_board_by_colour(path: Path):
    """Reads the board STEP with its colours (via OCAF) and returns
    [(cq.Shape compound, (r, g, b, a)), ...], one entry per colour.

    (cq.Assembly.importStep can't be used: the vendor file reuses component
    names, which a CadQuery assembly rejects.)"""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_ColorGen, XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("board"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.Transfer(doc):
        sys.exit(f"could not read {path}")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colour_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    labels = TDF_LabelSequence()
    shape_tool.GetFreeShapes(labels)
    whole = shape_tool.GetShape_s(labels.Value(1)) if labels.Length() == 1 else None
    if whole is None:
        from OCP.TopoDS import TopoDS_Compound
        from OCP.BRep import BRep_Builder
        whole, builder = TopoDS_Compound(), BRep_Builder()
        builder.MakeCompound(whole)
        for i in range(1, labels.Length() + 1):
            builder.Add(whole, shape_tool.GetShape_s(labels.Value(i)))

    groups = {}
    exp = TopExp_Explorer(whole, TopAbs_SOLID)
    while exp.More():
        solid = exp.Current()
        c = Quantity_Color()
        found = (colour_tool.GetColor(solid, XCAFDoc_ColorSurf, c) or colour_tool.GetColor(solid, XCAFDoc_ColorGen, c))
        if not found:  # colour may be stored on the solid's faces
            from OCP.TopAbs import TopAbs_FACE
            fe = TopExp_Explorer(solid, TopAbs_FACE)
            while fe.More() and not found:
                found = colour_tool.GetColor(fe.Current(), XCAFDoc_ColorSurf, c)
                fe.Next()
        if found:
            r, g, b = c.Values(Quantity_TOC_sRGB)
            key = (round(r, 2), round(g, 2), round(b, 2), 1.0)
        else:
            key = (0.3, 0.3, 0.32, 1.0)
        groups.setdefault(key, []).append(cq.Shape.cast(solid))
        exp.Next()
    return [(cq.Compound.makeCompound(solids), rgba) for rgba, solids in groups.items()]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def board_step_path(arg: str | None) -> Path:
    if arg:
        path = Path(arg)
        if not path.exists():
            sys.exit(f"board model not found: {path}")
        if _sha256(path) != BOARD_STEP_SHA256:
            print(f"warning: {path} differs from the verified board model revision {BOARD_STEP_COMMIT[:7]}")
        return path
    dest = HERE / "out" / "_maker-esp32.step"
    if not dest.exists() or _sha256(dest) != BOARD_STEP_SHA256:
        print(f"downloading the official board model ({BOARD_STEP_COMMIT[:7]}) from GitHub ...")
        tmp = dest.with_suffix(".part")
        urllib.request.urlretrieve(BOARD_STEP_URL, tmp)
        if _sha256(tmp) != BOARD_STEP_SHA256:
            tmp.unlink()
            sys.exit("downloaded board model failed its checksum - refusing to use it")
        tmp.replace(dest)
    return dest


def main():
    out = HERE / "out"
    out.mkdir(exist_ok=True)
    board_step = board_step_path(sys.argv[1] if len(sys.argv) > 1 else None)

    bottom, top = C.make_bottom_plate(), C.make_top_plate()
    top_z = C.ACRYLIC_T + C.STANDOFF_H
    board_z = C.ACRYLIC_T + C.BOARD_STANDOFF_H

    parts = [("bottom_plate", C.plate_solid(bottom, 0.0), ACRYLIC), ("top_plate", C.plate_solid(top, top_z), ACRYLIC)]
    for s, y in C.motor_positions():
        tag = f"{'left' if s < 0 else 'right'}_{'front' if y > 0 else 'rear'}"
        parts.append((f"bracket_{tag}", bracket(s, y), BLACK))
        screws, nuts = bracket_hardware(s, y)
        parts.append((f"bracket_screws_{tag}", screws, DARK_STEEL))
        parts.append((f"bracket_nuts_{tag}", nuts, STEEL))
        for name, (wp, col) in motor(s, y).items():
            parts.append((f"motor_{tag}_{name}", wp, col))
        for name, (wp, col) in wheel(s, y).items():
            parts.append((f"wheel_{tag}_{name}", wp, col))
    parts += deck_standoffs()
    parts += board_hardware(board_z)

    # The real board, placed so its holes land on the plate's board holes.
    board_loc = cq.Location(cq.Vector(-C.BOARD_L / 2, -C.BOARD_W / 2, board_z))
    board_groups = [(sh.moved(board_loc), rgba) for sh, rgba in load_board_by_colour(board_step)]
    board_shape = cq.Compound.makeCompound([sh for sh, _ in board_groups])
    print(f"board model: {len(board_shape.Solids())} solids in {len(board_groups)} colours")
    # DC jack = the tall part overhanging the board's -X end (see README)
    jack_solids = [s for s in board_shape.Solids()
                   if s.BoundingBox().xmin < -C.BOARD_L / 2 - 0.5 and s.BoundingBox().zlen > 8]
    if len(jack_solids) != 1:
        sys.exit(f"could not identify the DC jack in the board model ({len(jack_solids)} candidates)")
    jb = jack_solids[0].BoundingBox()
    dc_jack = (jb.xmin, (jb.ymin + jb.ymax) / 2, board_z + 5.5)
    if C.DC_JACK_SIDE != -1:
        sys.exit("full_model.py places the DC jack on the left (DC_JACK_SIDE = -1)")
    parts += battery_and_straps(board_z, dc_jack)

    # --- check the REAL board against everything around it -----------------
    problems = []
    board_bb = board_shape.BoundingBox()
    for name, wp, _ in parts:
        if name.startswith(("board_standoff", "board_screws", "dc_plug")):
            continue  # designed to touch the board
        sh = wp.val()
        if not _boxes_overlap(sh.BoundingBox(), board_bb):  # cheap reject
            continue
        vol = board_shape.intersect(sh).Volume()
        if vol > 0.05:
            problems.append(f"real board model collides with {name}: {vol:.2f} mm^3")
    if problems:
        print("FULL MODEL CHECK FAILED:")
        for p in problems:
            print("  -", p)
        sys.exit(1)
    print(f"real board model: clear of plates, straps, battery and wiring "
          f"(top of board at {board_shape.BoundingBox().zmax:.1f} mm, top deck at {top_z:.1f} mm)")

    # --- export ---------------------------------------------------------------
    assy = cq.Assembly(name="esp32_4wd_robot")
    for name, wp, rgba in parts:
        assy.add(wp, name=name, color=cq.Color(*rgba))
    for i, (sh, rgba) in enumerate(board_groups):
        assy.add(sh, name=f"maker_esp32_{i:02d}", color=cq.Color(*rgba))
    assy.export(str(out / "robot_full.step"))
    assy.export(str(out / "robot_full.glb"), tolerance=0.05, angularTolerance=0.25)
    mesh = cq.Compound.makeCompound([wp.val() for _, wp, _ in parts] + [board_shape])
    cq.exporters.export(mesh, str(out / "robot_full.stl"), tolerance=0.05, angularTolerance=0.25)
    for f in ("robot_full.step", "robot_full.stl", "robot_full.glb"):
        print(f"wrote out/{f} ({(out / f).stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
