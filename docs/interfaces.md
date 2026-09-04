# Interfaces (as implemented)

Contracts reflect the actual code in `firmware/` and
`ros2_ws/src/trolley_core/`.

## ROS topics

| Purpose | Topic | Type | Producer -> Consumer |
| --- | --- | --- | --- |
| Raw motion request | `/motion_request` | `geometry_msgs/Twist` | `cardputer_teleop` (and later the follower) -> `safety_braking` |
| Limited drive command | `/cmd_vel` | `geometry_msgs/Twist` | `safety_braking` -> `arduino_base_controller` |
| Laser scan | `/scan` | `sensor_msgs/LaserScan` | `sllidar_ros2` -> `safety_braking` |
| Safety state | `/safety/state` | `std_msgs/String` | `safety_braking` -> telemetry / evaluation |

Safety now sits in the command path: every motion source publishes
`/motion_request`, and only `safety_braking` publishes `/cmd_vel`.

## Wireless teleop (Cardputer -> Pi)

`cardputer_teleop` binds UDP 0.0.0.0:5005, reads `W/A/S/D` (anything else = stop),
and publishes a fixed-magnitude Twist to `/motion_request`
(`linear.x = +/-0.5`, `angular.z = +/-1.0`; all parameterized).

## Safety supervisor (`safety_braking`)

Fuses `/scan` + `/motion_request` and republishes `/cmd_vel` at 20 Hz:

- CLEAR (d > 2.0 m): pass the request through.
- WARN (<= 2.0 m): pass through, announce state.
- SLOW (<= 1.0 m): clamp `|linear.x|` to `slow_max_linear_mps` (0.15); turning allowed.
- STOP (<= 0.5 m): `linear.x = 0`; rotation still allowed to reorient.
- Fail-safe: scan stale -> hold STOP; request stale -> publish zero.

A cart-footprint box (`cart_x/y_min/max`) filters the LiDAR's self-hits. Zone
distances are parameters and must be validated physically.

> Firmware dependency: the Arduino is bang-bang today (any `linear.x > 0` ->
> full DRIVE_SPEED), so the SLOW clamp has no physical effect until the firmware
> accepts a speed value. STOP is fully enforced now (`linear.x = 0` -> `X`).

## Arduino serial contract (as built)

- `/dev/ttyACM0` (fallback `/dev/ttyUSB0`), 9600 baud.
- Pi -> Arduino: single char `W/S/A/D/X`, derived from the sign of `/cmd_vel`.
- Arduino -> Pi (unified firmware, 20 Hz): CSV `left_m,right_m,accel_x,gyro_z`
  (logged today; not yet parsed into `/odom`).

## Known interface gaps

- No `/odom` publisher yet (telemetry is logged, not parsed).
- SLOW zone needs variable-speed firmware to take physical effect.
- UWB range topic + follow controller not implemented yet.
