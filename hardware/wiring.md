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
   +-- Lane 3 (SENSORS): 5.1 V lane --> RPLIDAR A1 + UWB modules (5 V, NOT 11 V, NOT Pi 3.3 V)
```

Rules learned the hard way:

- Motors take **raw ~11.1 V** at the L298N input. The L298N's own ~2 V drop
  (BJT VCEsat) means ~9.1 V actually reaches the motors.
- LiDAR and UWB are **5 V** parts. They must come off the LM2596 5.1 V lane -
  never the 11.1 V bus (instant death) and never the Pi's 3.3 V rail (rail sag
  breaks the UWB UART).
- The Pi 5 caps total USB power to **0.6 A** unless it sees an official 5 A
  supply. Powering LiDAR/UWB from Pi USB requires either that supply or an
  external 5 V feed. Prefer feeding sensors from the LM2596 lane directly.

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

## 6. Measured drivetrain constants

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

## 7. Sign convention observed

Front-left: forward -> ticks count **positive**, reverse -> negative. Front-right
is **inverted** (forward -> negative). Handle in firmware/odometry (negate the
right encoder). Cart also veers slightly right at equal PWM - correct with a
per-side Kp trim, not by hand-matching wires.

## 8. Known issues / open items

- **Wheel base not yet measured** - required before odometry is trustworthy.
- **UWB power feed** not yet wired - needs a 5 V tap off the LM2596 lane (or a
  small dedicated buck) sized for anchors + tag; confirm against the LM2596's
  remaining 3 A headroom after the Pi.
- **IMU (MPU) integration** undecided: Arduino I2C vs Pi I2C.
- **6 V motors on a ~9 V rail** are only safe while the PWM cap holds - any
  code path that writes `analogWrite(pin, 255)` will cook them. Keep MAX_PWM=160.
- Encoder splices were hand-soldered and described as "weak" - re-sheath /
  reinforce before sustained driving.
- Battery chemistry label (LiFePO4 vs LiPo) inconsistent across BOM and build
  log - confirm the actual pack.
