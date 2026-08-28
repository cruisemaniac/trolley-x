# Trolley-X

Autonomous, retrofittable **follow-me cart** for multi-payload transport. This
repository is the physical-prototype codebase for the **PDE4445 Robotics
Dissertation** (Middlesex University Dubai). It is the sim-to-real development
of the validated CW2 ROS 2 simulation - the new contribution is the working,
load-bearing hardware plus fresh evaluation, **not** a re-run of the sim.

> Team: Ashwin Murali Thanalapati (M01037932) - Mohammed Shalaby (M01035318) -
> Vignesh Lakshmanasamy (M01026685)

## Follow method: UWB (canonical)

Operator following is **UWB-based** (REYAX RYUW122_Lite modules): two anchors on
the cart triangulate a hand-carried tag. Vision-follow (720p webcam) from the
early build plan is **superseded** and out of scope - see
[docs/proposal/build-plan.md](docs/proposal/build-plan.md) for that history.

Also in scope: three-zone LiDAR safety (warn / slow / stop) on an RPLIDAR A1,
onboard ROS 2 Jazzy on a Raspberry Pi 5, and Arduino low-level motor control.

Out of scope: vision item recognition, self-checkout, external participant studies.

## The robot, as actually built

The physical drivetrain diverges from every early document (proposal, BOM, and
the first repo scaffold all describe a SmartDrive MDDS30 + 2 motors). The real
cart is:

- **Skid-steer, 4 encoder gear motors** (65 mm wheels, ~325-332 ticks/rev measured).
- **2x L298N** dual H-bridge drivers (one per side), **not** an MDDS30.
- **2x Arduino Uno R3** (one on a Prototype Shield v5 with screw terminals) for
  low-level control. Motor logic runs on the Uno; the second Uno is spare/expansion.
- **~11.1 V 3S pack** (BOM says "LiFePO4 12 V"; the build chat calls it LiPo -
  reconcile the chemistry label), distributed via WAGO lever hubs.
- **LM2596** buck kept to feed the Pi at 5.1 V. HW-851 USB board and the relay
  bank were removed.
- Motors are **6 V rated** -> firmware hard-caps PWM at 160/255 (~6 V average).

Full pin-level detail is in **[hardware/wiring.md](hardware/wiring.md)**.

## Repository layout

```text
trolley-x/
  docs/          Architecture, interfaces, decisions, the proposal + assessment
                 briefs, the reference library, and the build log.
  hardware/      As-built wiring, BOM reconciliation, power, chassis, sensors.
  firmware/      Arduino motor-controller firmware (V/ENC serial protocol).
  ros2_ws/       ROS 2 Jazzy colcon workspace. Packages live under src/.
  evaluation/    Test plans, metrics, scripts, captured data.
  scripts/       Small maintenance scripts only.
```

## Where the code lives (consolidated)

Laptop + rover code is now merged into this tree:

| What | Where |
| --- | --- |
| Arduino sketches (5, incl. unified production firmware) | `firmware/` |
| ROS nodes (arduino_base, cardputer_teleop, safety_braking) | `ros2_ws/src/trolley_core/` |
| RPLIDAR A1 driver (git submodule) | `ros2_ws/src/sllidar_ros2/` |

## Cloning

`sllidar_ros2` is a git submodule, so pull it in on clone:

```bash
git clone --recurse-submodules <repo-url>
# or, if already cloned:
git submodule update --init --recursive
```

## Build and test (on the ROS 2 Jazzy target)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch trolley_x_bringup prototype.launch.py
```

## Status

Hardware fabricated and driving (open-loop + forward PID verified). Laptop/rover
code consolidated here. **Top priority:** close the safety gap (`safety_braking`
is advisory-only today) and cap turn PWM at 160 for the 6 V motors. UWB modules
ordered. See [docs/architecture.md](docs/architecture.md) and
[docs/open-questions.md](docs/open-questions.md).
