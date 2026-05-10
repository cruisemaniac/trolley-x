# Open Questions

## Resolved during consolidation
- IMU location: **Arduino** (MPU6050 over I2C, in the unified firmware).
- Right-encoder sign: handled in firmware ISR (inverted vs left).
- Serial protocol: single-char W/A/S/D/X @ 9600 + CSV telemetry (not V/ENC).

## Priority / safety
- **Close the safety gap:** `safety_braking` only logs - make it intercept
  `/cmd_vel` so the cart actually stops. Blocks enabling follow mode.
- **Cap turn PWM at 160** for the 6 V motors (turns/teleop currently hit 200-255).
- **Measure the wheel track width** - needed before `/odom` is trustworthy.
- **UWB 5 V feed** off the LM2596 lane (confirm 3 A headroom after the Pi).

## ROS / software
- Parse Arduino CSV telemetry into a real `/odom` publisher + TF.
- Add a bringup launch (lidar + safety + base).
- `/cmd_vel` -> char mapping is bang-bang and mutually exclusive; decide whether
  to keep or move to proportional/simultaneous drive+turn.
- UWB range read + Kalman follow node (on RYUW122 arrival); package placement.

## Project
- Repo license (Apache-2.0 common for ROS) and public-repo timing.
- Team contribution workflow (branches/PRs).
- Confirm module Gen-AI policy with the supervisor; declare where required.

## Evaluation
- Trials per metric; thresholds for latency, jitter, stop distance, reliability.
- Exact CW2 dataset for the sim-to-real comparison.
