# ROS 2 Workspace (Jazzy)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Do not commit `build/`, `install/`, or `log/` (gitignored).

## Packages (src/)

- **`trolley_core`** - the project's nodes (ament_python):
  - `arduino_base` - `/cmd_vel` (Twist) -> single-char serial to the Arduino @9600; logs telemetry.
  - `cardputer_teleop` - UDP:5005 from the Cardputer -> `/cmd_vel`.
  - `safety_braking` - `/scan` three-zone monitor (0.5 stop / 1.0 slow / 2.0 warn). **Advisory only today.**
- **`sllidar_ros2`** - vendored Slamtec RPLIDAR driver (upstream:
  github.com/Slamtec/sllidar_ros2). Launch: `sllidar_a1_launch.py`.

Run examples:

```bash
ros2 run trolley_core arduino_base
ros2 run trolley_core cardputer_teleop
ros2 run trolley_core safety_braking
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

## Next integration steps

1. Make `safety_braking` intercept `/cmd_vel` (subscribe raw motion, publish
   limited `/cmd_vel`) so stops actually stop the cart.
2. Parse the Arduino CSV telemetry into `/odom` (constants: 65 mm wheels,
   TPR 332/325 - already in the firmware).
3. Add a `trolley_x_bringup`-style launch file to start lidar + safety + base together.
4. Add the UWB range + Kalman follow node when the RYUW122 modules arrive.
