#!/usr/bin/env python3
"""
1:1 size-check poster for a 24-inch plotter: out/size_check_24x36.pdf

One 24 x 36 in (ARCH D) sheet with
  * both plates and the fit coupon at exactly 1:1, every hole and slot drawn
    (red = cut, blue = engrave), plus grey dashed outlines of the real parts
    that sit on them - lay the hardware on the paper to check the fit
  * scale bars in X and in Y (plotters can scale one axis differently) and a
    1-inch square
  * renders of the complete robot (images/render_*.png, from viewer.html)

Usage:  python size_check_poster.py
Print at 100 % / "actual size", no "fit to page".
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.image as mpimg  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

import chassis as C  # noqa: E402

HERE = Path(__file__).resolve().parent
INCH = 25.4
PAGE_W, PAGE_H = 24 * INCH, 36 * INCH  # 609.6 x 914.4 mm
GREY = "#8a8f96"


def draw_plate(ax, plate, cx, cy):
    for layer, polys in C.tessellate(plate):
        for pts in polys:
            xs, ys = zip(*[(cx + x, cy + y) for x, y in pts])
            ax.plot(xs, ys, color="red" if layer == C.CUT else "blue", linewidth=0.5)


def outline(ax, cx, cy, w, h, label=None, style="--", fs=6):
    ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h, fill=False, linestyle=style, linewidth=0.6, edgecolor=GREY))
    if label:
        ax.text(cx, cy, label, ha="center", va="center", fontsize=fs, color=GREY)


def bottom_overlays(ax, cx, cy):
    # Maker-ESP32 (DC jack on the left)
    outline(ax, cx, cy, C.BOARD_L, C.BOARD_W, "Maker-ESP32\n80 x 56\n(DC jack on this side <)")
    # bracket flanges (under the plate) at each motor
    for s, y in C.motor_positions():
        fx = s * (C.PLATE_W / 2 - C.BRK_FLANGE_DEPTH / 2)
        outline(ax, cx + fx, cy + y, C.BRK_FLANGE_DEPTH, C.BRK_FACE_W, "bracket\nflange", fs=5)
    # battery (under the plate)
    bx, by = C.battery_footprint()
    outline(ax, cx, cy - 0.0, bx, by, None, style=":")
    ax.text(cx, cy - by / 2 - 3, "battery outline (under)", ha="center", va="top", fontsize=5, color=GREY)
    if C.SENSORS:
        ina_x = sum(x for x, _ in C.INA_HOLES) / 2
        outline(ax, cx + ina_x, cy + C.INA_HOLES[0][1], 30, 24, "INA226\ncarrier", fs=5)
        for sx in (-1, 1):
            outline(ax, cx + sx * C.CLIFF_X, cy + C.CLIFF_Y, 25, 10.7, "cliff (under)", fs=4.5)


def top_overlays(ax, cx, cy):
    if C.SENSORS:
        outline(ax, cx, cy, C.LD06_BASE, C.LD06_BASE)
        ax.text(cx, cy - C.LD06_BASE / 2 + 2, "LD06 lidar base", ha="center", va="bottom", fontsize=5, color=GREY)
        outline(ax, cx + C.IMU_XY[0], cy + C.IMU_XY[1], 30, 30, "IMU carrier", fs=5)


def scale_bar(ax, x0, y0, length, vertical=False):
    if vertical:
        ax.plot([x0, x0], [y0, y0 + length], color="black", linewidth=0.8)
        for y in (y0, y0 + length):
            ax.plot([x0 - 3, x0 + 3], [y, y], color="black", linewidth=0.8)
        for k in range(0, int(length) + 1, 10):
            ax.plot([x0, x0 + (2.5 if k % 50 else 4)], [y0 + k, y0 + k], color="black", linewidth=0.4)
        ax.text(x0 - 5, y0 + length / 2, f"{length:.0f} mm (Y)", rotation=90, ha="right", va="center", fontsize=7)
    else:
        ax.plot([x0, x0 + length], [y0, y0], color="black", linewidth=0.8)
        for x in (x0, x0 + length):
            ax.plot([x, x], [y0 - 3, y0 + 3], color="black", linewidth=0.8)
        for k in range(0, int(length) + 1, 10):
            ax.plot([x0 + k, x0 + k], [y0, y0 + (2.5 if k % 50 else 4)], color="black", linewidth=0.4)
        ax.text(x0 + length / 2, y0 - 5, f"{length:.0f} mm (X)", ha="center", va="top", fontsize=7)


def image(ax, path, x0, y_top, width):
    img = mpimg.imread(path)
    h = width * img.shape[0] / img.shape[1]
    ax.imshow(img, extent=(x0, x0 + width, y_top - h, y_top), interpolation="lanczos")
    ax.add_patch(Rectangle((x0, y_top - h), width, h, fill=False, linewidth=0.4, edgecolor=GREY))
    return h


def main():
    out = HERE / "out" / "size_check_24x36.pdf"
    bottom, top, coupon = C.make_bottom_plate(), C.make_top_plate(), C.make_coupon()
    fig = plt.figure(figsize=(PAGE_W / INCH, PAGE_H / INCH))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, PAGE_W)
    ax.set_ylim(0, PAGE_H)
    ax.set_aspect("equal")
    ax.axis("off")

    # --- title ------------------------------------------------------------------
    ax.text(20, PAGE_H - 18, "ESP32 4WD robot - 1:1 size check", fontsize=26, weight="bold", va="top")
    ax.text(20, PAGE_H - 34, "Print at 100 % / \"actual size\" - never \"fit to page\".  "
            "Red = cut, blue = engrave, grey dashed = outlines of the real parts (not cut).",
            fontsize=10, va="top")

    # --- 1:1 parts ------------------------------------------------------------------
    y_mid = PAGE_H - 160
    bx, tx = 30 + C.PLATE_W / 2, 30 + C.PLATE_W + 30 + C.PLATE_W / 2
    draw_plate(ax, bottom, bx, y_mid)
    bottom_overlays(ax, bx, y_mid)
    draw_plate(ax, top, tx, y_mid)
    top_overlays(ax, tx, y_mid)
    for cx, label in ((bx, "BOTTOM PLATE (lower deck)"), (tx, "TOP PLATE (upper deck)")):
        ax.text(cx, y_mid + C.PLATE_L / 2 + 6, f"{label}  -  {C.PLATE_W:g} x {C.PLATE_L:g} mm  -  front is up",
                ha="center", va="bottom", fontsize=8, weight="bold")
    kx = tx + C.PLATE_W / 2 + 30 + coupon.w / 2
    ky = y_mid + C.PLATE_L / 2 - coupon.h / 2
    draw_plate(ax, coupon, kx, ky)
    ax.text(kx, ky + coupon.h / 2 + 6, "FIT COUPON", ha="center", va="bottom", fontsize=8, weight="bold")
    ax.text(kx, ky - coupon.h / 2 - 4, "bracket slots (right edge = plate edge)\n"
            "hole ladder 3.0 / 3.2 / 3.4 / 3.6\n40 mm engraved line",
            ha="center", va="top", fontsize=6)

    # --- scale checks --------------------------------------------------------------------
    sx0 = kx - coupon.w / 2
    scale_bar(ax, sx0, y_mid - 30, 150)
    scale_bar(ax, PAGE_W - 25, y_mid - C.PLATE_L / 2, 150, vertical=True)
    ax.add_patch(Rectangle((sx0, y_mid - 85), INCH, INCH, fill=False, linewidth=0.8, edgecolor="black"))
    ax.text(sx0 + INCH + 4, y_mid - 85 + INCH / 2, "1 inch square", va="center", fontsize=7)
    ax.text(sx0, y_mid - 100, "Measure both bars with a ruler:\nexactly 150 mm, or the print is scaled.",
            va="top", fontsize=7)

    # --- renders ---------------------------------------------------------------------------
    y_r = y_mid - C.PLATE_L / 2 - 30
    ax.text(20, y_r, "The complete robot", fontsize=16, weight="bold", va="bottom")
    w2 = (PAGE_W - 40 - 10) / 2
    h = image(ax, HERE / "images" / "render_iso.png", 20, y_r - 4, w2)
    image(ax, HERE / "images" / "render_inside.png", 30 + w2, y_r - 4, w2)
    ax.text(20 + w2 / 2, y_r - 4 - h - 2, "Assembled, with sensors", ha="center", va="top", fontsize=8)
    ax.text(30 + 1.5 * w2, y_r - 4 - h - 2, "Top deck removed: controller between the motors",
            ha="center", va="top", fontsize=8)
    y_r2 = y_r - 4 - h - 14
    w3 = (PAGE_W - 40 - 20) / 3
    h3 = 0
    for k, (name, cap) in enumerate((("render_side", "Side"), ("render_front", "Front"),
                                     ("render_under", "Underside: battery, cliff sensors, front ToF"))):
        h3 = image(ax, HERE / "images" / f"{name}.png", 20 + k * (w3 + 10), y_r2, w3)
        ax.text(20 + k * (w3 + 10) + w3 / 2, y_r2 - h3 - 2, cap, ha="center", va="top", fontsize=8)

    # --- notes -----------------------------------------------------------------------------------
    y_n = y_r2 - h3 - 16
    track = C.track_width()
    left = [
        "CHECK ON THE PAPER",
        "1.  Both scale bars measure exactly 150 mm; the square is exactly 1 inch.",
        "2.  Bracket: lay a bracket flange on a corner outline - bend on the plate",
        "     edge; all four of its holes inside the red slots.",
        "3.  Maker-ESP32: its four corner holes on the four circles at the centre.",
        "4.  LD06 lidar: base on its outline, holes on the two top-plate holes.",
        "5.  Cliff sensors (GY-530): both holes inside the slots near the front corners.",
        "6.  Battery: fits inside the dotted outline (it straps on underneath).",
        "",
        "Then cut the coupon, then the plates (cut_sheet_both_plates.svg).",
    ]
    right = [
        "KEY DIMENSIONS",
        f"Plates  {C.PLATE_W:g} x {C.PLATE_L:g} x {C.ACRYLIC_T:g} mm acrylic, decks {C.STANDOFF_H:g} mm apart",
        f"Wheelbase {C.WHEELBASE:g} mm, track {track:g} mm, overall {track + C.WHEEL_W:g} mm wide",
        f"Bracket flange {C.BRK_FLANGE_DEPTH:g} x {C.BRK_FACE_W:g} mm, holes on a "
        f"{C.BRK_HOLE_SQUARE:g} mm square",
        f"Bracket slots: M3 {C.M3_CLEAR:g} mm, +/-{C.BRK_SLOT_EXTRA / 2:g} mm along the motor axis",
        f"Wheels {C.WHEEL_D:g} x {C.WHEEL_W:g} mm, {C.WHEEL_GAP:g} mm from the bracket face",
        f"LD06 holes M2.5 ({C.LD06_HOLE_D:g} mm) on a 28.2 mm square diagonal",
        f"Cliff sensor holes {C.CLIFF_HOLE_SPACING:g} mm apart, slotted +/-{C.CLIFF_SLOT_EXTRA / 2:g} mm",
        "Source: hardware/chassis/chassis.py (matches the SolidCraft model esp32_bot.sc)",
    ]
    for k, line in enumerate(left):
        ax.text(20, y_n - k * 7, line, fontsize=9 if k else 11, weight="bold" if k == 0 else "normal", va="top",
                family="DejaVu Sans")
    for k, line in enumerate(right):
        ax.text(PAGE_W / 2 + 10, y_n - k * 7, line, fontsize=9 if k else 11, weight="bold" if k == 0 else "normal",
                va="top", family="DejaVu Sans")

    with PdfPages(out, metadata={"CreationDate": None, "ModDate": None}) as pdf:
        pdf.savefig(fig)
    plt.close(fig)
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB), page {PAGE_W:.1f} x {PAGE_H:.1f} mm")


if __name__ == "__main__":
    main()
