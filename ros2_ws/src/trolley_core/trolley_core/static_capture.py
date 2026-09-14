"""Static ground-truth capture for accuracy stations.

Stand the operator (tag) at a KNOWN distance from the cart, run this once, and it
collects a few seconds of samples and appends one summary row to a CSV. Repeat at
several distances to build the distance-dependent accuracy tables Judhi asked for:

  - UWB range accuracy: measured vs truth, mean error and std, per anchor + fused
  - UWB triangulation accuracy: fused range/position error vs distance
  - LiDAR accuracy: front-min readout vs truth, error and variance
  - RSSI vs distance: mean RSSI per anchor at each station

Usage (on the Pi, with the follow stack running):
  ros2 run trolley_core static_capture --ros-args -p truth_m:=2.0 -p seconds:=5.0

Output row -> <out_dir>/accuracy_stations.csv (one row per run).
"""

import os
import csv
import math
import statistics as st

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


ROW = [
    'truth_m', 'n_samples',
    'left_mean', 'left_err', 'left_std',
    'right_mean', 'right_err', 'right_std',
    'uwb_mean', 'uwb_err', 'uwb_std',
    'fused_range_mean', 'fused_range_err', 'fused_range_std',
    'fused_lat_std_m',
    'rssi_left_mean', 'rssi_right_mean',
    'lidar_mean', 'lidar_err', 'lidar_std',
]


class StaticCapture(Node):
    def __init__(self):
        super().__init__('static_capture')
        self.declare_parameter('truth_m', 1.0)
        self.declare_parameter('seconds', 5.0)
        self.declare_parameter('out_dir', os.path.expanduser('~/trolley_logs'))
        self.declare_parameter('front_arc_deg', 30.0)      # narrow: aim at the operator
        self.declare_parameter('bearing_offset_deg', 180.0)
        self.declare_parameter('invert_scan', True)

        g = lambda n: self.get_parameter(n).value
        self.truth = float(g('truth_m'))
        self.seconds = float(g('seconds'))
        self.out_dir = os.path.expanduser(str(g('out_dir')))
        os.makedirs(self.out_dir, exist_ok=True)
        self.half_front = math.radians(float(g('front_arc_deg')) / 2.0)
        self.bearing_offset = math.radians(float(g('bearing_offset_deg')))
        self.invert = bool(g('invert_scan'))

        self.left, self.right = [], []
        self.rssi_l, self.rssi_r = [], []
        self.fused_range, self.fused_lat = [], []
        self.lidar = []

        self.create_subscription(Range, '/uwb/left', lambda m: self.left.append(m.range), 30)
        self.create_subscription(Range, '/uwb/right', lambda m: self.right.append(m.range), 30)
        self.create_subscription(Float32, '/uwb/left/rssi', lambda m: self.rssi_l.append(m.data), 30)
        self.create_subscription(Float32, '/uwb/right/rssi', lambda m: self.rssi_r.append(m.data), 30)
        self.create_subscription(PointStamped, '/follow/target', self.on_target, 20)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.start = self.get_clock().now()
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f"Capturing {self.seconds:.0f} s at truth {self.truth:.2f} m. Hold still...")

    def on_target(self, m):
        self.fused_range.append(math.hypot(m.point.x, m.point.y))
        self.fused_lat.append(m.point.y)   # lateral error proxy at a head-on station

    def on_scan(self, msg):
        best = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            raw = -angle if self.invert else angle
            bearing = math.atan2(math.sin(raw + self.bearing_offset),
                                 math.cos(raw + self.bearing_offset))
            if abs(bearing) <= self.half_front and r < best:
                best = r
        if best != float('inf'):
            self.lidar.append(best)

    @staticmethod
    def _stats(samples, truth=None):
        if not samples:
            return ('', '', '')
        mean = st.fmean(samples)
        sd = st.pstdev(samples) if len(samples) > 1 else 0.0
        err = (mean - truth) if truth is not None else ''
        r = lambda x: round(x, 4) if isinstance(x, float) else x
        return (r(mean), r(err) if err != '' else '', r(sd))

    def tick(self):
        if (self.get_clock().now() - self.start).nanoseconds / 1e9 < self.seconds:
            return
        uwb_all = self.left + self.right
        lm, le, ls = self._stats(self.left, self.truth)
        rm, re, rs = self._stats(self.right, self.truth)
        um, ue, us = self._stats(uwb_all, self.truth)
        fm, fe, fs = self._stats(self.fused_range, self.truth)
        _, _, flat = self._stats(self.fused_lat)
        rl = round(st.fmean(self.rssi_l), 2) if self.rssi_l else ''
        rr = round(st.fmean(self.rssi_r), 2) if self.rssi_r else ''
        dm, de, ds = self._stats(self.lidar, self.truth)
        n = max(len(self.left), len(self.right), len(self.fused_range), len(self.lidar))

        row = [self.truth, n, lm, le, ls, rm, re, rs, um, ue, us,
               fm, fe, fs, flat, rl, rr, dm, de, ds]

        path = os.path.join(self.out_dir, 'accuracy_stations.csv')
        new = not os.path.exists(path)
        with open(path, 'a', newline='') as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(ROW)
            w.writerow(row)

        self.get_logger().info(
            f"truth {self.truth:.2f} m | UWB mean {um} (err {ue}, std {us}) | "
            f"fused {fm} (err {fe}) | lidar {dm} (err {de}) | RSSI L{rl}/R{rr} -> {path}")
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = StaticCapture()
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
