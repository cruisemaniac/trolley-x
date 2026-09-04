# Trolley-X - As-Built Wiring

This file records the as-built electrical wiring of the prototype. It is the
authority for all wiring questions. The content comes from the build log
([../docs/build-log/](../docs/build-log/)) and from bench measurements. A
measured value is marked as measured. Section 9 lists the open items.

## 1. Component inventory (electrical)

| Role | Part | Notes |
| --- | --- | --- |
| Low-level controller | Arduino Uno R3 + Prototype Shield v5 | Screw-terminal shield. It holds the wires so they do not vibrate loose. A second Uno R3 is a spare. |
| Motor drivers | 2x L298N dual H-bridge | "Top" board = LEFT pair, "Bottom" board = RIGHT pair. |
| Motors | 4x DC gear motor w/ quadrature encoder | **6 V rated**, 65 mm wheels. Front and rear per side. |
| Onboard compute | Raspberry Pi 5 (16 GB), Ubuntu 24.04 + ROS 2 Jazzy | Hosts and powers the Arduino over USB. |
| LiDAR | Slamtec RPLIDAR A1 | `/scan` via sllidar_ros2. |
| UWB | REYAX RYUW122_Lite x3 | 2 anchors on the cart. 1 tag carried by the person. UART interface. Self-contained ToF. |
| IMU | MPU accelerometer | Not yet integrated. Host is Arduino or Pi (TBD). |
| Battery | ~11.1 V 3S pack | Reads ~11.6 V when charged. The main WAGO hub distributes it. |
| Buck | LM2596 | Steps 11.1 V down to **5.1 V** for the Pi. Rated 3 A maximum. Observed load ~2.4 A. |
| Removed | HW-851 USB board, relay bank | The HW-851 is redundant because Pi USB powers the Arduino. The relays are deferred. |

## 2. Power distribution (three lanes off the WAGO hub)

```text
Battery (~11.1 V)
   |
   +-- Lane 1 (MOTORS):  WAGO hub --> L298N "12V" input on BOTH boards --> motors
   |
   +-- Lane 2 (COMPUTE): WAGO hub --> LM2596 (=> 5.1 V) --> Raspberry Pi 5 (5V/GND)
   |
   +-- Lane 3 (SENSORS): 5.1 V lane --> RPLIDAR A1 (5 V).  (UWB is 3.3 V, off the Pi header - see section 6, NOT this lane.)
```

Power rules:

- The motors receive the raw ~11.1 V at the L298N input. The L298N drops about
  2 V (BJT VCEsat). The motors therefore receive about 9.1 V.
- The RPLIDAR A1 is a **5 V** part. Power it from the LM2596 5.1 V lane. Do not
  connect it to the 11.1 V bus. The 11.1 V bus destroys it.
- The UWB (RYUW122_Lite) is a **3.3 V** part (range 2.4-3.6 V). 5 V destroys it.
  Power it from the Pi 3.3 V header (pin 1). Tie its ground to the Pi ground. Its
  UART is 3.3 V logic. It connects to the Pi UART with no level shifter (section 6).
- The Pi 5 limits total USB power to **0.6 A** without an official 5 A supply. To
  power the LiDAR from Pi USB, use that supply or an external 5 V feed. Prefer to
  power the LiDAR from the LM2596 lane.

## 3. Common ground (do not skip)

Tie all grounds together. Without a common ground, the logic signals float and
the motors move without command.

- Connect the L298N GND (the middle pin of the 3-pin power port, the brown
  negative wire) to the negative WAGO block. Leave this as it is.
- Connect the Arduino GND header to a free slot on the **same** negative WAGO block.
- Connect the encoder Black wire (GND) to the Arduino GND header.

Note on the L298N 3-pin power port. The pin order is **+Vs, GND, +5V-out**. Put
the brown negative wire in the **middle (GND)** pin. Do not move it to the 5 V
pin. That forces 11 V back through the regulator and destroys the board.

## 4. Motor drive wiring (Arduino Uno R3)

Each motor uses its own L298N channel. Do NOT connect two motors to one channel.
The stall current is more than the 2 A channel limit. Y-cable the logic lines so
both channels on a board receive the same signal. Each side then moves as a pair.

Motor power: connect each motor's **Red / White** leads to that motor's L298N OUT block.

Logic (one Uno controls both L298Ns):

| Signal | Arduino pin | L298N wiring |
| --- | ---: | --- |
| EN_LEFT (PWM speed) | **9** | Top board ENA + ENB, Y-cabled (inner prong only) |
| IN_LEFT_FWD | **8** | Top board IN1 + IN3, Y-cabled |
| IN_LEFT_REV | **7** | Top board IN2 + IN4, Y-cabled |
| EN_RIGHT (PWM speed) | **10** | Bottom board ENA + ENB, Y-cabled (inner prong only) |
| IN_RIGHT_FWD | **6** | Bottom board IN1 + IN3, Y-cabled |
| IN_RIGHT_REV | **5** | Bottom board IN2 + IN4, Y-cabled |

Jumper caps:

- Keep the small **5 V logic-enable caps** behind each L298N power block **ON**.
  They power the driver logic from the 11 V input.
- Remove the **ENA / ENB caps**. Plug the single control wire onto the **inner**
  Enable prong only. Leave the outer 5 V prong bare.

If a wheel turns the wrong way, do not change the code. Swap that motor's Red and
Black wires at its L298N OUT terminal.

## 5. Encoder wiring (front two motors only)

The rear encoders are not connected. This saves interrupt pins and processing.
Only the two front encoders feed odometry.

Encoder wire colours (per motor datasheet):

| Wire | Meaning |
| --- | --- |
| Red | Motor power + (to L298N OUT) |
| White | Motor power - (to L298N OUT) |
| Blue | Encoder Vcc 3.3-5 V |
| Black | Encoder GND |
| Yellow | Encoder phase A (11 pulses/rev at the motor shaft) |
| Green | Encoder phase B |

Connections to the Arduino Uno. Phase A needs the two hardware-interrupt pins.

| Encoder | Yellow (phase A) | Green (phase B) | Blue (5V) | Black (GND) |
| --- | ---: | ---: | --- | --- |
| Front-left | **Pin 2** (INT0) | **Pin 4** | 5V header | GND header |
| Front-right | **Pin 3** (INT1) | **Pin 11** | 5V header | GND header |

## 6. UWB serial + reset (Pi GPIO header)

Two RYUW122_Lite anchors are mounted on the front pillars (RHS and LHS). Each
anchor has its own Pi UART. The person carries a third module as the tag. The tag
is not on the Pi.

The RYUW122_Lite is a **3.3 V** module: supply 2.4-3.6 V (3.3 V typical), 3.3 V
UART logic. It connects to the Pi 3.3 V GPIO UART with no level shifter. Do NOT
power it from 5 V. 5 V is more than its 3.6 V maximum and destroys it.

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

- Cross TX and RX on both anchors: module TXD to Pi RXD, module RXD to Pi TXD.
- Both anchors use Pi hardware UARTs, not USB. They do not appear under
  `/dev/serial/by-id/`. Only the Arduino and the LiDAR CP2102 bridge appear there.
  Ubuntu does not create the `/dev/serial0` alias. Use the `ttyAMA*` names.

### Boot config (`/boot/firmware/config.txt`)

UART0 is the default header UART. UART2 (GPIO4/5) needs its Pi 5 overlay:

```
enable_uart=1
dtoverlay=uart2-pi5
```

On the Pi 5, the Bluetooth uses a separate UART. `disable-bt` is therefore NOT
needed to free the header UART. Do not use `uart3-pi5` (GPIO8/9). Those pins
conflict with SPI0, which is enabled here. After boot, `sudo dmesg | grep ttyAMA`
lists `ttyAMA0` (GPIO14/15) and `ttyAMA2` (GPIO4/5). `ttyAMA10` is the SoC debug
UART and is not related.

### Reset service

NRST has no reliable internal pull-up. After boot, each module stays silent until
NRST gets a clean low-to-high edge. The line must then stay high. The `uwb-reset`
service drives both NRST pins (GPIO17 = RHS, GPIO27 = LHS) low-to-high at boot. It
then holds them high. See [../scripts/uwb-reset/](../scripts/uwb-reset/).

Verified on hardware (Pi 5, Ubuntu 24.04). With the service running, both modules
answer `AT` with `+OK` on `ttyAMA0` and `ttyAMA2`. No manual pulse is needed. The
RYUW122 is command-driven. It does not send data on its own. A passive `cat` on
the port shows nothing, even when the module is healthy. Always test with an `AT`
command.

## 7. Measured drivetrain constants

| Constant | Value | Notes |
| --- | --- | --- |
| Wheel diameter | **65 mm** (radius 0.0325 m) | Measured. |
| Ticks per rev (front-left) | **332** | Measured (11 PPR x gearing x edges). |
| Ticks per rev (front-right) | **325** | Measured. The small L/R mismatch is real. |
| Encoder base PPR | 11 | Before gear reduction. |
| Wheel base (track width) | **TODO - measure** | Needed for odometry/turn rate. |
| PWM hard cap | **160 / 255** | Caps the ~9.1 V rail to ~6 V for the 6 V motors. |

These values feed the ROS odometry parameters in
`ros2_ws/src/trolley_x_bringup/config/prototype.yaml`.

## 8. Sign convention observed

Front-left encoder: forward counts **positive**, reverse counts negative.
Front-right encoder is **inverted**: forward counts negative. Handle this in the
firmware or odometry. Negate the right encoder count. The cart also turns slightly
right at equal PWM. Correct this with a per-side Kp trim. Do not try to match the
wires by hand.

## 9. Known issues / open items

- **Wheel base not yet measured.** Measure it before you trust the odometry.
- **UWB anchors verified (bring-up).** Both RHS and LHS RYUW122_Lite anchors are
  wired. They are powered at 3.3 V off the Pi header. They run on `ttyAMA0` and
  `ttyAMA2`. The `uwb-reset` service resets them at boot. Both answer `AT` with
  `+OK`. The remaining UWB work is the tag config and the ranging/follow node,
  not the wiring.
- **IMU (MPU) integration not decided.** The host is Arduino I2C or Pi I2C.
- **6 V motors on a ~9 V rail.** They are safe only while the PWM cap holds. Any
  code that writes `analogWrite(pin, 255)` destroys them. Keep MAX_PWM=160.
- The encoder splices are hand-soldered and weak. Re-sheath and reinforce them
  before sustained driving.
- The battery chemistry label (LiFePO4 or LiPo) is not consistent across the BOM
  and the build log. Confirm the actual pack.
