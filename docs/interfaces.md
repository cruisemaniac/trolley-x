# Interfaces (as implemented)

These contracts reflect the **actual code** in `firmware/` and
`ros2_ws/src/trolley_core/`. Where they fall short of the target architecture,
it is noted.

## ROS topics

| Purpose | Topic | Type | Producer -> Consumer |
| --- | --- | --- | --- |
| Drive command | `/cmd_vel` | `geometry_msgs/Twist` | `cardputer_teleop` -> `arduino_base_controller` |
| Laser scan | `/scan` | `sensor_msgs/LaserScan` | `sllidar_ros2` -> `safety_braking` |

> Gap: `safety_braking` currently only **logs** zone violations. It does not yet
> publish a stop or sit between `/cmd_vel` and the Arduino, so there is no
> automatic braking. See architecture.md "Safety gap".

## Wireless teleop (Cardputer -> Pi)

- `cardputer_teleop` binds **UDP 0.0.0.0:5005** and reads single chars
  `W/A/S/D` (X = stop implied by no packet), publishing a fixed-magnitude Twist
  (`linear.x = +/-0.5`, `angular.z = +/-1.0`).

## Arduino serial contract (as built)

- Link: `/dev/ttyACM0` (fallback `/dev/ttyUSB0`), **9600 baud**.
- Pi -> Arduino: a **single character** per command - `W` fwd, `S` reverse,
  `A` spin-left, `D` spin-right, `X` stop. `arduino_base_controller` derives it
  from the sign of `/cmd_vel` (magnitude is ignored - drive is bang-bang).
- Arduino -> Pi (unified firmware, 20 Hz): **CSV telemetry**
  `left_odom_m,right_odom_m,accel_x,gyro_z`. The ROS side currently logs this
  raw; it is not yet parsed into `/odom`.

## Frames (target, not yet published)

`map` -> `odom` -> `base_link` -> `laser`; `uwb_tag`, `uwb_anchor_<n>` later.

## Known interface gaps

- No `/odom` publisher yet (telemetry is logged, not parsed).
- `/cmd_vel` -> char mapping is mutually exclusive (an `elif` chain), so the cart
  cannot drive and turn simultaneously, and speed magnitude is dropped.
- UWB range topic + follow controller not implemented yet.
