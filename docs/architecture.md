# Architecture

## Mission

Trolley-X follows an operator at ~1.0 m while enforcing collision-free behaviour,
and supports quantitative comparison against the validated CW2 simulation.

## Runtime (as-built)

- Onboard compute: Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy (host `trolley-x`, user `navigator`).
- Low-level controller: Arduino Uno R3 -> 2x L298N + 2 front encoders + MPU6050 IMU (9600 baud).
- Drivetrain: 4x 6 V gear motors, skid-steer, forward PID capped at 160/255.
- Range: Slamtec RPLIDAR A1 (`sllidar_ros2`).
- Positioning (planned): REYAX RYUW122_Lite UWB (2 anchors + 1 tag).
- Wireless teleop: M5Stack Cardputer -> UDP:5005.

See [../hardware/wiring.md](../hardware/wiring.md) for the electrical detail.

## Packages

`trolley_core` (arduino_base, cardputer_teleop, safety_braking) + vendored
`sllidar_ros2`. See [../ros2_ws/src/README.md](../ros2_ws/src/README.md).

## Control flow - current vs target

```text
CURRENT (safety in the loop):
  Cardputer --UDP:5005--> cardputer_teleop --/motion_request--> safety_braking --/cmd_vel--> arduino_base --serial--> Arduino --> L298N --> motors
  RPLIDAR --/scan--> safety_braking  (clear/warn/slow/stop + fail-safe)
  Arduino --CSV telemetry--> arduino_base  (logged, not yet turned into /odom)

TARGET:
  motion sources --/motion_request--> safety_braking (limits) --/cmd_vel--> arduino_base --> Arduino
  Arduino telemetry --> /odom ; UWB --> follow controller --> /motion_request
```

## Safety (closed the gap)

`safety_braking` now sits between motion sources and `arduino_base`: teleop (and
later the follower) publish `/motion_request`, safety fuses that with `/scan` and
republishes the limited `/cmd_vel`, and only that reaches the Arduino. STOP is
enforced now; SLOW becomes physical once the firmware accepts a speed value.

## Development order

1. DONE: safety intercepts commands (real stop).
2. Parse Arduino CSV telemetry into `/odom` (65 mm wheels, TPR 332/325).
3. Bringup launch (lidar + safety + base together).
4. UWB read + Kalman follow (on RYUW122 arrival).
5. Integrated follow + safety trials; evaluation + sim-to-real comparison.

## Non-goals

Vision item recognition; checkout/retail logic; external human-participant workflow.
