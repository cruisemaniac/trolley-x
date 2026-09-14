# Pi runbook - follow fix + data collection

This runbook covers the differential-drive fix and the data capture. Run every
step from your own terminal on the Pi (host `trolley-x`, user `navigator`).

## 0. Get the new code onto the Pi

On the Pi:
```
cd ~/trolley_ws/src/trolley-x        # or wherever the repo lives on the Pi
git pull                              # after you push trolley-x-clean from the Mac
```
(If you do not pull over git, copy the changed files across with scp/rsync.)

Changed files:
- `firmware/Trolley-X-unified-Spinal-setup/Trolley-X-unified-Spinal-setup.ino`
- `ros2_ws/src/trolley_core/trolley_core/arduino_base_controller.py`
- `ros2_ws/src/trolley_core/trolley_core/follow_controller.py`
- `ros2_ws/src/trolley_core/trolley_core/uwb_ranging.py`
- `ros2_ws/src/trolley_core/trolley_core/uwb_localizer.py`
- `ros2_ws/src/trolley_core/trolley_core/follow_logger.py`      (new)
- `ros2_ws/src/trolley_core/trolley_core/static_capture.py`     (new)
- `ros2_ws/src/trolley_core/setup.py`
- `ros2_ws/src/trolley_core/launch/follow.launch.py`

## 1. Measure and set the geometry (important)

The differential drive needs the wheel track. The EKF needs the anchor geometry.
Measure them, then edit the defaults in `follow.launch.py`:
- `wheel_track_m` - distance between the left and right wheel contact points (m).
- `anchor_x` - forward offset of both UWB anchors from base_link (m).
- `anchor_baseline` - left-right separation of the two anchors (m).

## 2. Flash the Arduino

```
cd ~/trolley_ws/src/trolley-x
./scripts/flash-arduino.sh          # or use the Arduino IDE / arduino-cli
```
The new firmware keeps W/A/S/D/X and adds `M <left> <right>` (signed per-wheel PWM)
plus a 600 ms watchdog. Test it once by hand on the bench:
```
# stop the ROS stack first so it does not fight you
picocom -b 9600 /dev/arduino       # or: screen /dev/arduino 9600
# type these (each line + Enter):
M 120 120      # both wheels forward -> drives straight
M 120 -120     # spin in place
M 120 60       # gentle arc to the right
X              # stop
```
If a wheel turns the wrong way, swap that side's IN_x_FWD / IN_x_REV wires (or
the encoder B pin only affects odometry sign, not drive).

## 3. Build the workspace

```
cd ~/trolley_ws
colcon build --packages-select trolley_core --symlink-install
source install/setup.bash
```

## 4. Bench test the drive path (wheels up, cart on a stand)

```
ros2 launch trolley_core follow.launch.py lidar:=false log:=false
```
In a second terminal, watch the command the base sends:
```
ros2 topic echo /cmd_vel
```
Move the tag around by hand. You should see `linear.x` AND `angular.z` change
together (an arc), not one at a time. Keep the cart still first until the log says
"gyro bias calibrated".

## 5. Follow test (on the floor, clear space)

```
ros2 launch trolley_core follow.launch.py
```
Expected behaviour now:
- The cart arcs toward you (drive + turn together), not lurch-spin-lurch.
- It holds ~1.0 m. It slows near obstacles (SLOW now bites, because PWM is honoured).
- Watch `/follow/target` and `/safety/state` in RViz or `ros2 topic echo`.

If it still hunts left/right, lower `k_bearing` (e.g. 0.8) or raise
`bearing_deadband_rad` (e.g. 0.12). If it is sluggish to turn, raise `min_turn_pwm`
on the base controller.

## 6. Collect data

The logger runs with the follow stack (`log:=true`, the default) and writes
`~/trolley_logs/follow_<timestamp>.csv`. One follow run gives the RQ1 + RQ2 + RQ3
streams (see `docs/evaluation-plan.md`).

Static accuracy stations (for the distance-dependent tables). With the stack
running, stand the tag at a known distance and run:
```
ros2 run trolley_core static_capture --ros-args -p truth_m:=1.0 -p seconds:=5.0
ros2 run trolley_core static_capture --ros-args -p truth_m:=2.0 -p seconds:=5.0
ros2 run trolley_core static_capture --ros-args -p truth_m:=3.0 -p seconds:=5.0
# ... repeat at 0.5 m steps
```
Each run appends one row to `~/trolley_logs/accuracy_stations.csv`.

Compare Kalman follow vs raw differential (RQ1 "alternatives"):
```
# Kalman (default):
ros2 launch trolley_core follow.launch.py
# Raw differential (one run):
ros2 launch trolley_core follow.launch.py   # then, in another terminal:
ros2 param set /follow_controller use_target false
```

## 7. Pull the logs back

Copy `~/trolley_logs/*.csv` from the Pi to the Mac (scp/rsync) into
`msc-dissertation/trolley-x-clean/evaluation/`. Tell me when they are there and I
will plot them and build the report tables.

## Notes / safety
- Keep the E-stop in reach for every floor test.
- Recharge the power bank before an accuracy session (Judhi: avoid voltage drift).
- The `M` watchdog stops the motors if ROS stops publishing, so a crash = a stop.
