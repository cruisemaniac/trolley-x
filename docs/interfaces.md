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
| UWB range (right anchor) | `/uwb/right` | `sensor_msgs/Range` | `uwb_ranging` -> follow controller / eval |
| UWB range (left anchor) | `/uwb/left` | `sensor_msgs/Range` | `uwb_ranging` -> follow controller / eval |
| IMU | `/imu/data` | `sensor_msgs/Imu` | `arduino_base_controller` -> localization / eval |
| Odometry | `/odom` | `nav_msgs/Odometry` | `arduino_base_controller` -> localization / eval |
| Fused tag (base_link) | `/follow/target` | `geometry_msgs/PointStamped` | `uwb_localizer` -> `follow_controller` |
| Fused tag (odom) | `/uwb/tag_odom` | `geometry_msgs/PointStamped` | `uwb_localizer` -> RViz / eval |

Safety sits in the command path: every motion source publishes `/motion_request`,
and only `safety_braking` publishes `/cmd_vel`.

## Wireless teleop (Cardputer -> Pi)

`cardputer_teleop` binds UDP 0.0.0.0:5005, reads `W/A/S/D` (anything else = stop),
and publishes a fixed-magnitude Twist to `/motion_request`
(`linear.x = +/-0.5`, `angular.z = +/-1.0`; all parameterized).

## Safety supervisor (`safety_braking`)

Fuses `/scan` + `/motion_request` and republishes `/cmd_vel` at 20 Hz. The scan is
split into four sectors in the cart frame - FRONT, REAR, LEFT, RIGHT - and the
nearest obstacle in each is tracked.

Zones (by sector distance):

- CLEAR (d > 2.0 m): pass the request through.
- WARN (<= 2.0 m): pass through, announce state.
- SLOW (<= 1.0 m): clamp `|linear.x|` to `slow_max_linear_mps` (0.15).
- STOP (<= 0.5 m): block that direction.

Directional gating: forward (`linear.x > 0`) is limited by the FRONT sector,
reverse (`linear.x < 0`) by the REAR sector, rotation (`angular.z`) is always
allowed. A wall ahead blocks forward but the cart can still back away. (Rotation
is not yet gated by the side sectors.)

Bearing: each beam angle gives the obstacle bearing (0 = ahead, +90 = left,
-90 = right, 180 = behind). The RPLIDAR is mounted flipped, so `invert_scan=True`
+ `bearing_offset_deg=180` map the beams to the cart frame. `front_arc_deg`
(default 90) sets the width of the front and rear guards.

`/safety/state` publishes at 20 Hz as `<STATE> F<front> R<right> B<rear> L<left>`
in metres (e.g. `WARN F1.80 R3.10 B4.00 L0.90`), or `STOP scan_stale` on a stale
scan.

Fail-safe: scan stale -> hold STOP in every sector; request stale -> publish zero.
A cart-footprint box (`cart_x/y_min/max`, in the raw lidar frame) filters the
LiDAR's self-hits; it must be re-validated for the flipped mount.

> Firmware dependency: the Arduino is bang-bang today, so the SLOW clamp has no
> physical effect until the firmware accepts a speed value. STOP is fully enforced
> now (`linear.x = 0` -> `X`).

## UWB ranging (`uwb_ranging`)

Two RYUW122 modules run as ANCHORs on the cart, one per front pillar; the operator
carries one RYUW122 as the TAG. The anchor starts each range and reports the
distance, so the Pi drives both anchors and reads the two distances to the tag.

- Ports: right `/dev/ttyAMA0`, left `/dev/ttyAMA2`, 115200 baud.
- Network: all three share `AT+NETWORKID` (default `TROLLEYX`) and, if used, one
  `AT+CPIN`. Anchors `ANCHOR_R` / `ANCHOR_L`; tag `TAG00001`.
- One worker thread ranges right, then left, so the two anchors never transmit at
  once (time-multiplex).
- Exchange: send `AT+ANCHOR_SEND=<tag>,<len>,<data>`; read the distance from
  `+ANCHOR_RCV=<tag>,<len>,<data>,<DISTANCE cm>,<RSSI>`.
- Publishes `sensor_msgs/Range` on `/uwb/right` (frame `uwb_right`) and `/uwb/left`
  (frame `uwb_left`).
- `configure_on_start` (default True) writes `AT+MODE=1`, network id and address to
  each anchor. RYUW122 saves to flash, so set False after the first good run.

Operator tag (set once, e.g. from the ESP32): `AT+MODE=0`,
`AT+NETWORKID=TROLLEYX`, `AT+ADDRESS=TAG00001` (and the same `AT+CPIN` if used).

## Arduino serial contract (as built)

- `/dev/arduino` (udev symlink; was `/dev/ttyACM0`), 9600 baud.
- Pi -> Arduino: `<dir> <pwm>` per line from `/cmd_vel` - `W`/`S` forward/reverse,
  `A`/`D` left/right spin, `X` stop. PWM is scaled from the velocity magnitude and
  capped at 160. (Firmware is bang-bang today and ignores the PWM value.)
- Arduino -> Pi (unified firmware, 20 Hz): CSV `left_m,right_m,accel_x,gyro_z`,
  parsed into `/odom` + `/imu/data` (see Odometry & IMU below).

## Odometry & IMU (`arduino_base_controller`)

The base controller also parses the telemetry and publishes:

- `/imu/data` (`sensor_msgs/Imu`): `angular_velocity.z` = gyro yaw rate (rad/s),
  `linear_acceleration.x` = forward accel (m/s^2); orientation not provided.
- `/odom` (`nav_msgs/Odometry`) + the `odom -> base_link` TF.

Odometry is gyro-aided: distance from the wheel encoders, heading integrated from
the gyro (not the wheel difference), so it needs no track width and tolerates
skid-steer slip. The gyro bias is averaged from a stationary startup window
(`gyro_bias_samples`, ~3 s) - keep the cart still until "gyro bias calibrated".
Heading still drifts slowly; the localisation EKF (UWB + odom + IMU) corrects it.
Set `publish_odom_tf=False` once an EKF owns `odom -> base_link`.

## Follow: localizer + controller

`uwb_localizer` is an EKF. It fuses the two anchor ranges with `/odom` into a
smooth tag position - state `[px, py, vx, vy]` in the odom frame, constant-
velocity operator model, one update per (time-multiplexed) range, an innovation
gate for outliers. It publishes the tag in base_link on `/follow/target` and in
odom on `/uwb/tag_odom`. Set the anchor geometry - `anchor_x` (forward offset)
and `anchor_baseline` (left-right separation); a wider baseline sharpens bearing.

`follow_controller` turns a tag position + a stand-off into `/motion_request`:

- `use_target=False` (node default): raw differential ranging on `/uwb/left`,
  `/uwb/right` - steer on `d_right - d_left`, speed on the mean range.
- `use_target=True` (set by `follow.launch`): consume `/follow/target` and steer
  on the true bearing + range from the EKF - smoother, and rides through UWB
  dropouts. This is the "Kalman follow".

Both turn toward the tag, hold forward until roughly aligned, keep the stand-off,
and stop on stale input. Motion still passes through `safety_braking`.

## Known interface gaps

- SLOW zone needs variable-speed firmware to take physical effect.
- Anchor geometry (`anchor_x`, `anchor_baseline`) must be measured for the EKF;
  a wider baseline improves bearing accuracy (small baseline = weak bearing).
- Cart-footprint box needs re-validation for the flipped lidar mount.
