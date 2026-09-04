# Open Questions

## Resolved during consolidation
- IMU location: **Arduino** (MPU6050 over I2C, in the unified firmware).
- Right-encoder sign: handled in firmware ISR (inverted vs left).
- Serial protocol: `<dir> <pwm>` @ 9600 + CSV telemetry (Pi sends PWM; firmware
  is still bang-bang).
- Safety architecture: motion sources -> `/motion_request` -> safety -> `/cmd_vel`.
- Safety is sectorized and directional (front/rear/left/right); bringup launch done.
- LiDAR mount: the RPLIDAR is mounted flipped; safety uses `invert_scan` +
  `bearing_offset_deg=180` to map beams to the cart frame.

## Priority / safety
- **Power (blocking for mobile):** the Pi 5 browns out on the shared LM2596 lane
  under motor load - confirmed via `vcgencmd get_throttled`; the lidar motor
  starved and could not start a scan. Give the Pi a dedicated 5 V/5 A feed, put the
  motors on their own rail, and the lidar on clean 5 V. Re-budget before adding the
  UWB modules.
- SLOW zone needs variable-speed firmware to physically slow (bang-bang today);
  STOP is fully enforced. Fold into the firmware handoff for Shalaby.
- **Cap turn PWM at 160** for the 6 V motors (turns/teleop currently hit 200-255).
- **Measure the wheel track width** - needed before `/odom` is trustworthy.
- **Re-validate the cart-footprint box** for the flipped lidar mount.

## ROS / software
- Parse Arduino CSV telemetry into a real `/odom` publisher + TF.
- DONE: bringup launch (lidar + safety + base + teleop).
- DONE: `uwb_ranging` reads both anchors and publishes `/uwb/left`, `/uwb/right`.
- NEXT: ESP32 tag firmware (RYUW122 tag config + power) and the Kalman follow
  controller (`/uwb/*` -> `/motion_request`).
- `/cmd_vel` -> serial now sends `<dir> <pwm>`; still mutually exclusive (no
  simultaneous drive + turn).

## Project
- Repo license (Apache-2.0 common for ROS) and public-repo timing.
- Team contribution workflow (branches/PRs).
- Confirm module Gen-AI policy with the supervisor; declare where required.

## Evaluation
- Trials per metric; thresholds for latency, jitter, stop distance, reliability.
- Exact CW2 dataset for the sim-to-real comparison.
