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

| Port | Motor (fixed by board) | Yellow (A) | Green (B) |
|------|------------------------|------------|-----------|
| M1   | GPIO 27 / 13           | **GPIO 34** | **GPIO 18** |
| M2   | GPIO 4 / 2             | **GPIO 35** | **GPIO 19** |
| M3   | GPIO 17 / 12           | **GPIO 36** | **GPIO 23** |
| M4   | GPIO 15 / 14           | **GPIO 39** | **GPIO 5**  |

Where these pins are:

- **GPIO 34, 35, 36, 39** are on the four 4-pin "ADC" header rows next to the
  M1/M2 terminals. Each row has `G`, `V`, an analog pin (34/35/36/39) and a
  digital pin (12/14/15/17). **Use only the G, V and 34–39 pins.**
  **Never connect anything to the 12/14/15/17 pin of those rows**: those
  are the M3/M4 motor-driver signals.
- **GPIO 5, 18, 19, 23** are the `S` pins of the four 3-pin "SPI" headers
  (labelled IO5 / IO18 / IO19 / IO23 on the board).

Why these pins: the motor ports use 8 GPIOs and the servo headers 4 more.
These 8 are what's left over. That still leaves the servo headers (25, 26,
32, 33) and I2C (21/22) free for a gripper, a display or an IMU later.

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

## 5. Before the first power-up

- [ ] No wire on the 12 / 14 / 15 / 17 pins of the ADC rows
- [ ] Encoder `V` pins measured at ≈3.3 V
- [ ] Each encoder's black wire is on a GND pin
- [ ] DIP switch on the Motor side
- [ ] Robot propped up so the wheels can spin freely for the first tests
