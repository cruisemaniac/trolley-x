# ADR 0001 - UWB follow, L298N/4-motor drivetrain, ROS 2 Jazzy monorepo

Status: accepted (2026-08). Supersedes the initial repo scaffold and the early
build plan.

## Context
The early build plan and BOM specified webcam vision-follow on a SmartDrive
MDDS30 + 2-motor drivetrain. During fabrication the drivetrain became a 4-motor
skid-steer on 2x L298N driven by an Arduino Uno R3, and the team committed to UWB
following (REYAX RYUW122_Lite) rather than vision.

## Decision
- Follow method: **UWB only**. Vision-follow is documented as superseded history.
- Drivetrain: **4x 6 V motors, skid-steer, 2x L298N, Arduino Uno R3**; PWM capped 160/255.
- One colcon monorepo (`ros2_ws/`), packages split sensing/estimation/safety/control/bringup.
- Bring-up uses a simple serial bridge (V/ENC), not `ros2_control`, until it earns its place.

## Consequences
- Odometry params come from measured constants (65 mm wheels, ~325-332 TPR), not proposal figures.
- The proposal BOM stays the academic/cost source; wiring.md is the build source.
- Vision code and the MDDS30 interface are out of scope for the prototype.
