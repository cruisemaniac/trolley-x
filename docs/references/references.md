# Reference library

Papers the dissertation intends to cite, grouped by the two technical pillars.
PDFs sit alongside this file. Use this as the seed for the report's *Related Work*.

## UWB following & state estimation (Kalman)

- **Xu, Shmaliy, Bi, Chen, Zhuang (2023).** "Extended Kalman/UFIR Filters for
  UWB-Based Indoor Robot Localization Under Time-Varying Colored Measurement
  Noise." *IEEE Internet of Things Journal* 10(17):15632.
  `Extended_Kalman_UFIR_Filters_for_UWBBased_Indoor_Robot_Localization...pdf`
  - EKF vs UFIR under colored noise; directly relevant to the follow filter choice.
- **He, Tang, Li, Zhang (2026).** "Enhanced Indoor Mobile Robot Localization via
  Lie-Group IMU-UWB Fusion and Dual-Stage Kalman Filtering." *Sensors* 2026.
  `sensors2602686v3.pdf`
  - Tight IMU+UWB fusion (ROS wheeled platform). Maps to our UWB + MPU plan.
- **Puriyanto, Fathurrahman, Rahani, Solikhah, Musa (2025).** "Implementation of
  Fuzzy Kalman Filter in Indoor Localization Using Ultra-Wideband Sensor."
  *Int. J. Robotics and Control Systems* 5(6):3047. `202880462PB.pdf`
  - Practical UWB + (fuzzy) Kalman implementation; a concrete build reference.

## LiDAR safety & obstacle avoidance

- **Leong, Ahmad (2024).** "LiDAR-Based Obstacle Avoidance With Autonomous
  Vehicles: A Comprehensive Review." *IEEE Access*.
  `LiDARBased_Obstacle_Avoidance_With_Autonomous_Vehicles_A_Comprehensive_Review.pdf`
  - Survey; grounds the three-zone safety approach in the literature.
- **Huang, Zeng, Chi, Sreenath, Liu, Su (2025).** "Dynamic Collision Avoidance
  Using Velocity Obstacle-Based Control Barrier Functions." *IEEE Trans. Control
  Systems Technology* 33(5):1601.
  `Dynamic_Collision_Avoidance_Using_Velocity_ObstacleBased_Control_Barrier_Functions.pdf`
  - Formal CBF/VO method; future-work angle beyond simple zone stopping.
- **Sousa, Silva, Schettino, Santos, Zachi, Gouvea, Pinto (2025).** "Obstacle
  Avoidance Technique for Mobile Robots at Autonomous Human-Robot Collaborative
  Warehouse Environments." *Sensors* 25(7):2387. `sensors2502387.pdf`
  - ROS/Gazebo fuzzy+CNN avoidance in a warehouse HRC setting; close to our domain.

> TODO: export a BibTeX file (`references.bib`) from these once the report's
> citation manager is chosen.
