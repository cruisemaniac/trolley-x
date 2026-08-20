# ROS packages

Actual packages live here now:

- `trolley_core` - Arduino base controller, Cardputer teleop, LiDAR safety.
- `sllidar_ros2` - vendored RPLIDAR A1 driver.

Future work may split `trolley_core` into follower / safety / bringup packages,
but only when that split earns its keep. Keep new code inside `trolley_core`
until then.
