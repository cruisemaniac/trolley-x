# ADR 0001 - UWB follow, L298N/4-motor drivetrain, ROS 2 Jazzy monorepo

Status: accepted (2026-08). This ADR supersedes the initial repo scaffold and the
early build plan.

## Context
The early build plan and BOM specified webcam vision-follow. They specified a
SmartDrive MDDS30 and a 2-motor drivetrain. During fabrication, the drivetrain
became a 4-motor skid-steer. It uses 2x L298N drivers and an Arduino Uno R3. The
team also chose UWB follow (REYAX RYUW122_Lite) instead of vision.

## Decision
- Follow method: **UWB only**. Vision-follow is superseded. It stays in the docs as history.
- Drivetrain: **4x 6 V motors, skid-steer, 2x L298N, Arduino Uno R3**. Cap the PWM at 160/255.
- One colcon monorepo (`ros2_ws/`). Split the packages into sensing, estimation, safety, control, and bringup.
- Bring-up uses a simple serial bridge (V/ENC), not `ros2_control`. Add `ros2_control` only when the project needs it.

## Consequences
- The odometry parameters come from measured constants (65 mm wheels, ~325-332 TPR). They do not come from the proposal figures.
- The proposal BOM stays the academic and cost source. wiring.md is the build source.
- The vision code and the MDDS30 interface are out of scope for the prototype.
