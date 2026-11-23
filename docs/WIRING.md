# Wiring

Board: **NULLLAB Maker-ESP32 v1.7x** ([pinout and schematic](https://github.com/nulllaborg/maker-esp32)).
Motors: **4 × JGA25-371, 12 V, ~500 RPM, Hall encoder**.

## 1. Each motor's 6-wire cable

| Wire   | Function            | Goes to |
|--------|---------------------|---------|
| Red    | Motor +             | Motor port Mx (2-pin PH2.0) |
| White  | Motor −             | Motor port Mx (2-pin PH2.0) |
| Blue   | Encoder + (3.3–5 V) | **3.3 V** pin (see step 3) |
| Black  | Encoder − (GND)     | GND pin next to it |
| Yellow | Encoder A           | GPIO in the table below |
| Green  | Encoder B           | GPIO in the table below |

The board's motor ports only carry the two motor wires. Split the cable:
red and white go to a 2-pin PH2.0 plug for the motor port; blue, black,
yellow and green go to female Dupont jumpers for the headers.

Motor polarity doesn't matter: the bring-up `dir` wizard detects it and
sets `invertMotor` in software.

## 2. Pin assignment

| Port | Wheel | Motor (fixed by board) | Yellow (A) | Green (B) |
|------|-------|------------------------|------------|-----------|
| M1   | rear-left   | GPIO 27 / 13 | **GPIO 19** | **GPIO 23** |
| M2   | front-left  | GPIO 4 / 2   | **GPIO 5**  | **GPIO 18** |
| M3   | rear-right  | GPIO 17 / 12 | **GPIO 33** | **GPIO 32** |
| M4   | front-right | GPIO 15 / 14 | **GPIO 26** | **GPIO 25** |

Layout, top view:

```
      FRONT
   M2      M4
   M1      M3
      REAR
```

Where these pins are:

- **GPIO 5, 18, 19, 23** are the `S` pins of the four 3-pin "SPI" headers
  (labelled IO5 / IO18 / IO19 / IO23 on the board).
- **GPIO 25, 26, 32, 33** are the `S` pins of the four 3-pin servo headers.
- **Never connect anything to the 12/14/15/17 pin of the 4-pin "ADC" header
  rows**: those are the M3/M4 motor-driver signals.

Why these pins: the motor ports use 8 GPIOs. The encoders take the SPI and
servo header signals, which keeps every encoder on a 3-pin header with its own
`G`/`V`. GPIO 34/35 read the PM02 battery monitor (section 5); 36/39 and
I2C (21/22) stay free for more sensors such as an IMU. The servo headers are now taken, and
34–39 are input-only, so add a PCA9685 board on I2C if you want servos later.

GPIO 5 is a boot strapping pin, but it only selects SDIO timing, so an
encoder holding it low at power-up does not stop the board booting.

## 3. Encoder power: check the voltage first

ESP32 inputs tolerate **3.3 V only**. The encoder's outputs swing up to
whatever voltage powers it, so **the encoders must be powered from 3.3 V**.

The board's schematic shows 3.3 V on the `V` pins of the ADC and SPI
headers, but NULLLAB's pinout picture labels the ADC rows "5V". So before
connecting any encoder:

1. Power the board (USB is enough).
2. With a multimeter, measure between `G` and `V` on one ADC row and on
   one SPI header.
3. Use only `V` pins that read **≈3.3 V**. If the ADC rows read 5 V, take
   the encoders' blue wires from the SPI headers' `V` pins, or from the `3.3V`
   pin of the power header row, instead.

Each encoder draws only a few mA, so sharing 3.3 V pins is fine.

## 4. Power

- Motors run from the **DC jack, 6–16 V** (5.5 × 2.1 mm, centre positive).
  These are 12 V motors, so a **3S LiPo (11.1–12.6 V)** or a 12 V pack gives
  full speed. A 2S pack (7.4 V) works at roughly 60 % speed.
- Turn the board's power switch **on**. USB alone powers the ESP32 but not
  the motors.
- Put the **Motor/IO DIP switch** on the **Motor** side. Otherwise ports M3
  and M4 are disconnected.
- An inline 5 A fuse on the battery lead is cheap insurance.

## 5. Battery monitor: Holybro PM02 (V3)

The PM02 sits in the battery lead and reports pack voltage and current as
two analog signals. The robot uses them to warn you, cap speed on a low
battery, and stop before the LiPo is run flat.

**Power path:** battery → PM02 XT60 **in** → PM02 XT60 **out** → board DC
jack (an XT60 to 5.5 × 2.1 mm lead). Keep the inline fuse on the battery side.

**Signal cable** (6-pin JST-GH; pin 1 is the red wire):

| PM02 pin | Signal | Connect to |
|----------|--------|------------|
| 1 (red)  | VCC 5.2 V out | **nothing**: insulate it |
| 2        | VCC 5.2 V out | **nothing**: insulate it |
| 3        | CURRENT (0–3.3 V) | **GPIO 35** |
| 4        | VOLTAGE (0–3.3 V) | **GPIO 34** |
| 5        | GND | a board `G` pin |
| 6        | GND | (spare) |

- GPIO 34 and 35 are the analog pins of two of the 4-pin "ADC" header rows.
  Use only their analog pin and `G`, and as always nothing on the
  12/14/15/17 pin of those rows.
- **Do not connect the PM02's 5.2 V** to the board. The board makes its own
  supply from the DC jack, and two supplies fighting can damage both.
- Both signals are at most 3.3 V by design, so they connect directly.

**Check it** with the bring-up firmware: `batt` shows live readings. Measure
the battery with a multimeter and run `batt cal <volts>` to fine-tune the
voltage scale. The pin should read about 1/18 of the pack voltage
(≈ 0.66 V at 12 V); if it reads 0, swap pins 3 and 4 or check GND.

**What the robot does** (thresholds in `robot_config.h`, set for a 3S LiPo):

| Level | Per cell, under load | Effect |
|---|---|---|
| OK | above 3.50 V | normal |
| LOW | below 3.50 V for 5 s | amber banner, speed capped at 50 %, amber LEDs when disarmed |
| CRITICAL | below 3.30 V for 3 s | ramps to a stop, refuses to arm, red pulsing LEDs |

The voltage is filtered and must stay low for several seconds, so a hard
launch (which briefly pulls the voltage down) does not trip it. LOW and
CRITICAL stay set until the robot is switched off: a flat pack reads healthy
again after resting, and driving on would damage it.

**Current** is approximate. The PM02 is scaled for 120 A, so the robot's few
amps are only tens of millivolts at the pin. Use it as a guide; the
protection relies on voltage only. If you use a different pack, set
`kBatteryCells` to its cell count (or 0 to detect it at power-up), and adjust
`kCellLowV` / `kCellCriticalV` for non-LiPo chemistries.

## 6. Before the first power-up

- [ ] No wire on the 12 / 14 / 15 / 17 pins of the ADC rows
- [ ] Encoder `V` pins measured at ≈3.3 V
- [ ] Yellow/green wires on the GPIOs in the table above
- [ ] Each encoder's black wire is on a GND pin
- [ ] DIP switch on the Motor side
- [ ] PM02: pins 1/2 (5.2 V) insulated, VOLTAGE → GPIO 34, CURRENT → GPIO 35, GND → `G`
- [ ] Robot propped up so the wheels can spin freely for the first tests
