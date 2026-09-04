# Trolley-X - As-Built Wiring

Source of truth for the physical prototype's electrical wiring, reconstructed
from the build log ([../docs/build-log/](../docs/build-log/)) and bench
verification. Where a value was measured it says so; open items are called out
in **Known issues / open items** at the end.

## 1. Component inventory (electrical)

| Role | Part | Notes |
| --- | --- | --- |
| Low-level controller | Arduino Uno R3 + Prototype Shield v5 | Screw-terminal shield so wires don't vibrate loose. A 2nd Uno R3 is spare. |
| Motor drivers | 2x L298N dual H-bridge | "Top" board = LEFT pair, "Bottom" board = RIGHT pair. |
| Motors | 4x DC gear motor w/ quadrature encoder | **6 V rated**, 65 mm wheels. Front + rear per side. |
| Onboard compute | Raspberry Pi 5 (16 GB), Ubuntu 24.04 + ROS 2 Jazzy | Hosts + powers the Arduino over USB. |
| LiDAR | Slamtec RPLIDAR A1 | `/scan` via sllidar_ros2. |
| UWB | REYAX RYUW122_Lite x3 | 2 anchors on cart + 1 hand-carried tag. UART, self-contained ToF. |
| IMU | MPU accelerometer | To be integrated (Arduino or Pi - TBD). |
| Battery | ~11.1 V 3S pack | Reads ~11.6 V charged. Main WAGO hub distributes it. |
| Buck | LM2596 | 11.1 V -> **5.1 V** for the Pi. Rated 3 A max (~2.4 A observed load). |
| Removed | HW-851 USB board, relay bank | HW-851 redundant (Pi USB powers the Arduino); relays deferred. |

## 2. Power distribution (three lanes off the WAGO hub)

```text
Battery (~11.1 V)
   |
   +-- Lane 1 (MUSCLE): WAGO hub --> L298N "12V" input on BOTH boards --> motors
   |
   +-- Lane 2 (BRAIN):  WAGO hub --> LM2596 (=> 5.1 V) --> Raspberry Pi 5 (5V/GND)
   |
   +-- Lane 3 (SENSORS): 5.1 V lane --> RPLIDAR A1 (5 V).  (UWB is 3.3 V, off the Pi header - see section 6, NOT this lane.)
```

Rules learned the hard way:

- Motors take **raw ~11.1 V** at the L298N input. The L298N's own ~2 V drop
  (BJT VCEsat) means ~9.1 V actually reaches the motors.
- The RPLIDAR A1 is a **5 V** part off the LM2596 5.1 V lane - never the 11.1 V
  bus (instant death).
- The UWB (RYUW122_Lite) is a **3.3 V** part (2.4-3.6 V; 5 V destroys it). It is
  powered from the Pi's 3.3 V header (pin 1) and shares the Pi ground. Its UART
  is 3.3 V logic and wires straight to the Pi UART - no level shifter (section 6).
- The Pi 5 caps total USB power to **0.6 A** unless it sees an official 5 A
  supply. Powering the LiDAR from Pi USB requires either that supply or an
  external 5 V feed. Prefer feeding the LiDAR from the LM2596 lane directly.

## 3. Common ground (do not skip)

All grounds must be tied or the logic signals float and motors twitch randomly:

- L298N GND (middle pin of the 3-pin power port, the brown/-ve wire) -> negative WAGO block. Leave as-is.
- Arduino GND header -> a free slot on the **same** negative WAGO block.
- Encoder Black (GND) -> Arduino GND header.

Note on the L298N 3-pin power port: pin order is **+Vs, GND, +5V-out**. The
brown negative wire correctly sits in the **middle (GND)** pin. Moving it to the
5 V pin forces 11 V backwards through the regulator and fries the board.

## 4. Motor drive wiring (Arduino Uno R3)

Each motor keeps its own L298N channel (do NOT parallel two motors onto one
channel - stall current exceeds the 2 A/channel limit). The two channels on a
board are driven **identically** by Y-cabling the logic lines, so a side moves
as a synchronized pair.

Motor power: each motor's **Red / White** leads -> that motor's L298N OUT block.

Logic (single Uno controlling both L298Ns):

| Signal | Arduino pin | L298N wiring |
| --- | ---: | --- |
| EN_LEFT (PWM speed) | **9** | Top board ENA + ENB, Y-cabled (inner prong only) |
| IN_LEFT_FWD | **8** | Top board IN1 + IN3, Y-cabled |
| IN_LEFT_REV | **7** | Top board IN2 + IN4, Y-cabled |
| EN_RIGHT (PWM speed) | **10** | Bottom board ENA + ENB, Y-cabled (inner prong only) |
| IN_RIGHT_FWD | **6** | Bottom board IN1 + IN3, Y-cabled |
| IN_RIGHT_REV | **5** | Bottom board IN2 + IN4, Y-cabled |

Jumper caps:

- The small **5 V logic-enable caps** behind each L298N power block stay **ON**
  (powers the driver's logic from the 11 V input).
- The **ENA / ENB caps** come **OFF** (removed). The single control wire plugs
  onto the **inner** Enable prong only; leave the outer 5 V prong bare.

If a wheel spins the wrong way, don't change code - swap that motor's Red/Black
at its L298N OUT terminal.

## 5. Encoder wiring (front two motors only)

Rear encoders are left disconnected to save interrupt pins / processing. Only
the two front encoders feed odometry.

Encoder wire colours (per motor datasheet):

| Wire | Meaning |
| --- | --- |
| Red | Motor power + (to L298N OUT) |
| White | Motor power - (to L298N OUT) |
| Blue | Encoder Vcc 3.3-5 V |
| Black | Encoder GND |
| Yellow | Encoder phase A (11 pulses/rev at the motor shaft) |
| Green | Encoder phase B |

Connections to the Arduino Uno (needs the 2 hardware-interrupt pins for phase A):

| Encoder | Yellow (phase A) | Green (phase B) | Blue (5V) | Black (GND) |
| --- | ---: | ---: | --- | --- |
| Front-left | **Pin 2** (INT0) | **Pin 4** | 5V header | GND header |
| Front-right | **Pin 3** (INT1) | **Pin 11** | 5V header | GND header |

## 6. UWB serial + reset (Pi GPIO header)

Two RYUW122_Lite anchors ride on the front pillars (RHS and LHS); each has its
own Pi UART. The person carries a third module as the tag (not on the Pi).

The RYUW122_Lite is a **3.3 V** module: supply 2.4-3.6 V (3.3 V typical), 3.3 V
UART logic. It wires straight to the Pi's 3.3 V GPIO UART with no level shifter.
Do NOT power it from 5 V - 5 V exceeds its 3.6 V maximum and destroys it.

Module 6-pin header (datasheet): 1 VDD, 2 NRST, 3 RXD, 4 TXD, 5 PA7 (mode flag,
high=normal / low=sleep), 6 GND.

### RHS anchor -> UART0 (`/dev/ttyAMA0`)

| UWB pin | Signal | Pi physical pin | Pi function |
| --- | --- | ---: | --- |
| 1 VDD | 3.3 V power | **1** | 3V3 |
| 6 GND | Ground | **14** | GND |
| 4 TXD | UART out | **10** | GPIO15 / RXD0 |
| 3 RXD | UART in | **8** | GPIO14 / TXD0 |
| 2 NRST | Active-low reset | **11** | GPIO17 |
| 5 PA7 | Mode flag (out) | - | leave unconnected |

### LHS anchor -> UART2 (`/dev/ttyAMA2`)

| UWB pin | Signal | Pi physical pin | Pi function |
| --- | --- | ---: | --- |
| 1 VDD | 3.3 V power | **17** | 3V3 |
| 6 GND | Ground | **25** | GND |
| 4 TXD | UART out | **29** | GPIO5 / RXD2 |
| 3 RXD | UART in | **7** | GPIO4 / TXD2 |
| 2 NRST | Active-low reset | **13** | GPIO27 |
| 5 PA7 | Mode flag (out) | - | leave unconnected |

- TX and RX cross over on both: module TXD -> Pi RXD, module RXD -> Pi TXD.
- Both are Pi hardware UARTs, not USB, so they never show under
  `/dev/serial/by-id/` (only the Arduino and the LiDAR's CP2102 bridge do), and
  Ubuntu does not make the `/dev/serial0` alias - use the `ttyAMA*` names.

### Boot config (`/boot/firmware/config.txt`)

UART0 is the default header UART; UART2 (GPIO4/5) needs its Pi 5 overlay:

```
enable_uart=1
dtoverlay=uart2-pi5
```

On the Pi 5 the Bluetooth is on a separate UART, so `disable-bt` is NOT needed to
free the header UART. Do not use `uart3-pi5` (GPIO8/9) - those pins clash with
SPI0, which is enabled here. After boot, `sudo dmesg | grep ttyAMA` lists
`ttyAMA0` (GPIO14/15) and `ttyAMA2` (GPIO4/5); `ttyAMA10` is the SoC debug UART,
unrelated.

### Reset service

NRST has no reliable internal pull-up: after boot each module stays silent until
NRST gets a clean low->high edge, then the line must be held high. The `uwb-reset`
service drives BOTH NRST pins (GPIO17 = RHS, GPIO27 = LHS) low->high at boot and
holds them high - see [../scripts/uwb-reset/](../scripts/uwb-reset/).

Verified on hardware (Pi 5, Ubuntu 24.04): with the service running, both modules
answer `AT` with `+OK` on `ttyAMA0` and `ttyAMA2`, no manual pulse. Note the
RYUW122 is command-driven - it does not stream unsolicited, so a passive `cat` on
the port shows nothing even when the module is healthy; always test with `AT`.

## 7. Measured drivetrain constants

| Constant | Value | Notes |
| --- | --- | --- |
| Wheel diameter | **65 mm** (radius 0.0325 m) | Measured. |
| Ticks per rev (front-left) | **332** | Measured (11 PPR x gearing x edges). |
| Ticks per rev (front-right) | **325** | Measured; slight L/R mismatch is real. |
| Encoder base PPR | 11 | Before gear reduction. |
| Wheel base (track width) | **TODO - measure** | Needed for odometry/turn rate. |
| PWM hard cap | **160 / 255** | Caps ~9.1 V rail to ~6 V for the 6 V motors. |

These feed the ROS odometry params in
`ros2_ws/src/trolley_x_bringup/config/prototype.yaml`.

## 8. Sign convention observed

Front-left: forward -> ticks count **positive**, reverse -> negative. Front-right
is **inverted** (forward -> negative). Handle in firmware/odometry (negate the
right encoder). Cart also veers slightly right at equal PWM - correct with a
per-side Kp trim, not by hand-matching wires.

## 9. Known issues / open items

- **Wheel base not yet measured** - required before odometry is trustworthy.
- **UWB anchors verified (bring-up).** Both RHS and LHS RYUW122_Lite anchors are
  wired, powered at 3.3 V off the Pi header, on `ttyAMA0` / `ttyAMA2`, and reset
  at boot by the `uwb-reset` service; both answer `AT` -> `+OK`. Remaining UWB
  work is tag config + the ranging/follow node, not the wiring.
- **IMU (MPU) integration** undecided: Arduino I2C vs Pi I2C.
- **6 V motors on a ~9 V rail** are only safe while the PWM cap holds - any
  code path that writes `analogWrite(pin, 255)` will cook them. Keep MAX_PWM=160.
- Encoder splices were hand-soldered and described as "weak" - re-sheath /
  reinforce before sustained driving.
- Battery chemistry label (LiFePO4 vs LiPo) inconsistent across BOM and build
  log - confirm the actual pack.
