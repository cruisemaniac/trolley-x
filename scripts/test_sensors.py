#!/usr/bin/env python3
"""All-sensor health check (ROS).

Subscribes to every sensor topic for a fixed window, then prints a PASS/FAIL
table with the rate and a sample value for each. Also reports per-anchor UWB
range mean/std, so it doubles as the left-vs-right anchor comparison.

Run the sensors first (no motion), then this script:
    ros2 launch trolley_core sensors.launch.py
    python3 scripts/test_sensors.py            # after: source install/setup.bash

For UWB ranges to appear, the operator tag must be powered and in view.
"""

import argparse
import math
import statistics as st
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped


class SensorCheck(Node):
    def __init__(self, secs):
        super().__init__('sensor_check')
        self.secs = secs
        self.t0 = time.time()
        self.scan_n = 0
        self.scan_valid = 0
        self.left, self.right = [], []
        self.imu_n = 0
        self.last_imu = None
        self.odom_n = 0
        self.tgt_n = 0
        self.last_tgt = None

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Range, '/uwb/left', lambda m: self.left.append(m.range), 20)
        self.create_subscription(Range, '/uwb/right', lambda m: self.right.append(m.range), 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 20)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(PointStamped, '/follow/target', self.on_tgt, 10)
        self.create_timer(0.2, self.tick)
        print(f"Listening {secs:.0f} s ...")

    def on_scan(self, m):
        self.scan_n += 1
        self.scan_valid = sum(1 for r in m.ranges if m.range_min < r < m.range_max)

    def on_imu(self, m):
        self.imu_n += 1
        self.last_imu = (m.linear_acceleration.x, m.angular_velocity.z)

    def on_odom(self, m):
        self.odom_n += 1

    def on_tgt(self, m):
        self.tgt_n += 1
        self.last_tgt = (math.hypot(m.point.x, m.point.y),
                         math.degrees(math.atan2(m.point.y, m.point.x)))

    def tick(self):
        if time.time() - self.t0 < self.secs:
            return
        self.report()
        raise SystemExit

    def _line(self, name, n, extra, ok):
        rate = n / self.secs
        tag = "PASS" if ok else "FAIL"
        print(f"  [{tag}] {name:14} {rate:5.1f} Hz   {extra}")

    def report(self):
        print("\n=== sensor health ===")
        self._line("LiDAR /scan", self.scan_n,
                   f"{self.scan_valid} valid pts", self.scan_n / self.secs > 3)
        for name, ds in (("UWB left", self.left), ("UWB right", self.right)):
            if ds:
                extra = f"mean {st.fmean(ds):.2f} m  std {st.pstdev(ds):.3f} m  n={len(ds)}"
            else:
                extra = "no ranges (tag off / out of view?)"
            self._line(name, len(ds), extra, len(ds) / self.secs > 2)
        imu_extra = (f"accel_x {self.last_imu[0]:+.2f}  gyro_z {self.last_imu[1]:+.3f}"
                     if self.last_imu else "no data")
        self._line("IMU /imu/data", self.imu_n, imu_extra, self.imu_n / self.secs > 10)
        self._line("Odom /odom", self.odom_n, "", self.odom_n / self.secs > 10)
        if self.tgt_n:
            r, b = self.last_tgt
            self._line("EKF target", self.tgt_n, f"range {r:.2f} m  bearing {b:+.0f} deg", True)
        else:
            print("  [ -- ] EKF target    (localizer idle / no UWB fix)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=8.0)
    args = ap.parse_args()
    rclpy.init()
    node = SensorCheck(args.secs)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
