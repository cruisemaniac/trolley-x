# Architecture

## Mission

Trolley-X follows an operator at ~1.0 m while enforcing collision-free behaviour,
and supports quantitative comparison against the validated CW2 simulation.

## Runtime (as-built)

- Onboard compute: Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy (host `trolley-x`, user `navigator`).
- Low-level controller: Arduino Uno R3 -> 2x L298N + 2 front encoders + MPU6050 IMU (9600 baud).
- Drivetrain: 4x 6 V gear motors, skid-steer, forward PID capped at 160/255.
- Range: Slamtec RPLIDAR A1 (`sllidar_ros2`), mounted flipped (see safety notes).
- Positioning: REYAX RYUW122 UWB - 2 cart anchors + 1 operator tag. The driver
  reads both anchors; the ESP32 tag firmware and the follow controller are next.
- Wireless teleop: M5Stack Cardputer -> UDP:5005.
- Power: the Pi 5 needs a dedicated clean 5 V/5 A feed, separate from the motor
  rail. Sharing the LM2596 lane browns out the Pi under motor load (confirmed).

See [../hardware/wiring.md](../hardware/wiring.md) for the electrical detail.

## Packages

`trolley_core` (arduino_base, cardputer_teleop, safety_braking, uwb_ranging) +
vendored `sllidar_ros2`. See [../ros2_ws/src/README.md](../ros2_ws/src/README.md).

## Control flow - current vs target

```text
CURRENT (safety in the loop, sectorized):
  Cardputer --UDP:5005--> cardputer_teleop --/motion_request--> safety_braking --/cmd_vel--> arduino_base --serial--> Arduino --> L298N --> motors
  RPLIDAR --/scan--> safety_braking  (4 sectors, directional; clear/warn/slow/stop + fail-safe)
  UWB anchors --serial--> uwb_ranging --/uwb/*--> uwb_localizer(EKF,+/odom) --/follow/target--> follow_controller --/motion_request--> safety_braking
  Arduino --CSV telemetry--> arduino_base  (parsed into /odom + /imu/data)

TARGET (built):
  UWB + /odom --> uwb_localizer(EKF) --> follow_controller --/motion_request--> safety_braking --> arduino_base
  Remaining: field-tune the EKF + evaluation trials.
```

## Safety (sectorized, directional)

`safety_braking` sits between motion sources and `arduino_base`. Teleop (and later
the follower) publish `/motion_request`; safety fuses that with `/scan` and
republishes the limited `/cmd_vel`, and only that reaches the Arduino.

The scan is split into four sectors in the cart frame - FRONT, REAR, LEFT, RIGHT.
Linear motion is gated by the sector it drives into: forward by the front sector,
reverse by the rear, rotation always allowed. So a wall ahead blocks forward
motion but the cart can still back away. STOP is enforced now; SLOW becomes
physical once the firmware accepts a speed value.

The bearing of each obstacle comes from its beam angle. The RPLIDAR is mounted
flipped, so `invert_scan` + `bearing_offset_deg=180` map the beams to the cart
frame (front/back and left/right both correct).

## Development order

1. DONE: safety intercepts commands (real stop).
2. DONE: gyro-aided `/odom` + `/imu/data` from telemetry (65 mm wheels, TPR 332/325).
3. DONE: bringup launch (lidar + safety + base + teleop).
4. DONE: sectorized directional safety with bearing.
5. DONE: UWB driver + ESP32 tag + `follow_controller` + `uwb_localizer` EKF
   (fused `/follow/target`, Kalman follow).
6. Field-tune the EKF, then integrated follow + safety trials; evaluation +
   sim-to-real comparison.

## Non-goals

Vision item recognition; checkout/retail logic; external human-participant workflow.
