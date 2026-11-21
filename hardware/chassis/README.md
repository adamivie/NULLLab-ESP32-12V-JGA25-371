# Acrylic chassis

A two-deck, laser-cut acrylic chassis:

- **Lower deck:** the four JGA25-371 motors hang under it in their L-brackets, and the battery sits on top of it.
- **Upper deck:** the Maker-ESP32 sits at the rear, with a grid of M3 accessory holes at the front.

![assembly](images/assembly.png)

| | |
|---|---|
| Plates | 2 × 160 × 200 mm, 3 mm acrylic |
| Footprint | 222 mm wide (over tyres) × 200 mm long |
| Wheelbase / track | 130 mm / 196 mm (wheel centres) |
| Ground clearance | 19 mm (bracket bottoms); 53 mm under the lower deck |
| Deck spacing | 35 mm (fits a 3S 2200 mAh LiPo up to ~28 mm tall) |

![lower deck](images/lower_deck.png)

## Files

| File | Use |
|---|---|
| `out/bottom_plate.dxf` / `.svg` | Laser cutting, lower deck |
| `out/top_plate.dxf` / `.svg` | Laser cutting, upper deck |
| `out/fit_test_coupon.dxf` / `.svg` | **Cut this first.** A small test piece, see below |
| `out/print_template_1to1.pdf` | Print at 100 % to check fit on paper before ordering |
| `out/chassis_assembly.step` | Full 3D assembly for any CAD program |
| `viewer.html` | Interactive 3D view (`python3 -m http.server` in this folder, then open `/viewer.html`) |
| `chassis.py` | Generator for all of the above |

Layers and colours: **CUT = red**, **ENGRAVE = blue** (a forward-pointing
arrow on each plate). Units are millimetres. SVG strokes are 0.01 mm
hairlines, which most laser services expect. Check your service's rules.

![plates](images/plates.png)

## Before ordering: verify the fit (5 minutes)

The bracket hole pattern comes from a published drawing of a JGA25 bracket,
not from your actual bracket, so check it:

1. Print `out/print_template_1to1.pdf` at **100 % / "Actual size"**. Measure
   the 100 mm bar at the bottom. If it isn't exactly 100 mm, your printer
   scaled the page.
2. Lay a bracket flange on one of the four slot groups on the bottom-plate
   page. The bracket's bend (outer face) goes on the plate edge. All four
   flange holes should sit inside the slots.
3. Lay the Maker-ESP32 on the top-plate page. Its four corner holes should
   line up with the four circles near the rear.

If you can, cut `fit_test_coupon` first (it costs pennies):

- **Slots:** check them against a real bracket. The coupon's right edge represents the plate edge.
- **Hole ladder:** pick the hole size (3.0 / 3.2 / 3.4 / 3.6 mm) where an M3 screw slides in freely.
- **40 mm engraved line:** measure it with calipers to confirm the cutter's scale.

If anything is off, edit the parameters at the top of `chassis.py`
(`BRK_HOLE_ROWS`, `BRK_HOLE_SPACING`, `M3_CLEAR`, ...) and regenerate (see
below).

## Ordering

- Material: **3 mm cast acrylic**. It cuts more cleanly and cracks less around
  screws than extruded acrylic. Clear or any colour.
- Any laser service that accepts DXF or SVG works, as does a makerspace laser.
  Upload the three files and tell them red = cut and blue = engrave (or score).

## Parts list

| Qty | Part | Notes |
|---|---|---|
| 1 | Bottom plate, top plate, fit coupon | 3 mm acrylic |
| 4 | JGA25 L-brackets + motor screws | from the motor kit |
| 16 | M3 × 10 screw + **M3 nylock nut** | brackets to the bottom plate |
| 6 | M3 × 35 hex standoff, female–female | between the decks |
| 12 | M3 × 6 screw | standoff ends |
| 4 | M3 × 10 **nylon** standoff + 8 × M3 × 6 nylon screws | board on the top plate (nylon can't short anything) |
| 2 | 20 mm hook-and-loop strap, ~250 mm | battery |
| ~10 | 3 mm cable ties | wire management |
| 1 | 3S LiPo, ~2200 mAh, ≤ 110 × 50 × 28 mm | or a 12 V pack of similar size |
| 1 | XT60 (or your battery's plug) to 5.5 × 2.1 mm DC barrel lead | power to the board's DC jack |
| 1 | **LiPo voltage alarm (buzzer)** on the balance lead | the board can't measure battery voltage, and an over-discharged LiPo is ruined |

Acrylic cracks if over-tightened. Snug the screws, then stop. Nylon washers
under screw heads help.

## Assembly

1. **Peel** the protective film. Note the engraved arrows: they point **forward**.
2. **Motors into brackets.** Rotate each motor so its **encoder connector (6-pin
   plug) points up**, towards where the plate will be. The wire slots above
   the motors are sized for that orientation.
3. **Brackets under the bottom plate.** The flange goes under the plate,
   pointing inwards, and the bracket face lines up with the plate's side edge.
   Fit an M3 × 10 screw from the top and a nylock nut underneath. Before tightening,
   slide the bracket outward in its slots until the face is flush with the edge.
4. **Wheels:** fit the couplings and wheels onto the shafts.
5. **Wires:** bring each motor's cable up through the slot above its pair of
   motors, then tie it down at the cable-tie holes.
6. **Standoffs:** screw the six M3 × 35 standoffs onto the bottom plate.
7. **Battery:** thread the two straps down through one pair of strap slots and
   up through the other, and strap the battery in the middle.
8. **Board:** fit the nylon standoffs on the top plate and mount the
   Maker-ESP32 with its **USB / DC-jack end toward the rear** (away from the
   arrow).
9. **Top plate:** lower it onto the standoffs, feeding the wires up through
   the two long slots beside the board. Motor wires go to the board's motor
   ports; encoder wires go to the pins in [docs/WIRING.md](../../docs/WIRING.md).
   Screw the top plate down.
10. Continue with bring-up in the [main README](../../README.md#3-bring-up-and-calibration-do-this-once).

## Changing the design

```sh
cd hardware/chassis
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
python chassis.py
```

All dimensions are parameters at the top of `chassis.py`: plate size,
wheelbase, deck spacing, acrylic thickness, battery size, board position,
hole sizes, bracket pattern.

Every run rebuilds the 3D assembly and **refuses to write files if the
design is broken**:

- any two parts collide (motors, encoder plugs, bracket nuts, wheels, battery,
  standoffs, board)
- ground clearance is too low
- a hole is within 2 mm of an edge or of another hole
- an engraving runs over a cut-out
- there's no room behind the motors for their wires
