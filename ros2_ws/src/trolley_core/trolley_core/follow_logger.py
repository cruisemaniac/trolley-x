"""Follow-run data logger -> CSV.

Records one row at a fixed rate with the latest value of every follow-relevant
stream, so a single run yields the raw data for most report metrics:

  - UWB raw ranges (left/right) and RSSI       -> path-loss, RSSI-vs-error, noise
  - EKF fused target (range + bearing)         -> smoothing (raw vs filtered)
  - distance-hold error vs the stand-off       -> following accuracy
  - requested vs safety-limited command        -> latency, hunting, SLOW effect
  - EKF innovation + cumulative reject count    -> filter consistency / robustness
  - LiDAR front-min distance + safety state    -> safety behaviour during the run
  - odometry pose                               -> trajectory / heading drift

Output: <out_dir>/follow_<timestamp>.csv  (out_dir default ~/trolley_logs).
"""

import os
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float32, String


COLUMNS = [
    't_s', 'uwb_left_m', 'uwb_right_m', 'rssi_left', 'rssi_right',
    'target_x', 'target_y', 'target_range_m', 'target_bearing_deg',
    'tag_odom_x', 'tag_odom_y', 'odom_x', 'odom_y', 'odom_theta_deg',
    'req_lin', 'req_ang', 'cmd_lin', 'cmd_ang', 'safety_state',
    'lidar_front_min_m', 'innovation_m', 'reject_count', 'dist_hold_err_m',
]


class FollowLogger(Node):
    def __init__(self):
        super().__init__('follow_logger')

        self.declare_parameter('out_dir', os.path.expanduser('~/trolley_logs'))
        self.declare_parameter('standoff_m', 1.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('front_arc_deg', 90.0)
        self.declare_parameter('bearing_offset_deg', 180.0)  # match the safety node
        self.declare_parameter('invert_scan', True)

        g = lambda n: self.get_parameter(n).value
        out_dir = os.path.expanduser(str(g('out_dir')))
        os.makedirs(out_dir, exist_ok=True)
        self.standoff = float(g('standoff_m'))
        self.half_front = math.radians(float(g('front_arc_deg')) / 2.0)
        self.bearing_offset = math.radians(float(g('bearing_offset_deg')))
        self.invert = bool(g('invert_scan'))

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(out_dir, f'follow_{stamp}.csv')
        self.fh = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.fh)
        self.writer.writerow(COLUMNS)

        # latest values
        self.v = {c: '' for c in COLUMNS}
        self.t0 = self.get_clock().now()

        self.create_subscription(Range, '/uwb/left', self.on_left, 20)
        self.create_subscription(Range, '/uwb/right', self.on_right, 20)
        self.create_subscription(Float32, '/uwb/left/rssi', self.on_rssi_l, 20)
        self.create_subscription(Float32, '/uwb/right/rssi', self.on_rssi_r, 20)
        self.create_subscription(PointStamped, '/follow/target', self.on_target, 10)
        self.create_subscription(PointStamped, '/uwb/tag_odom', self.on_tag_odom, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Twist, '/motion_request', self.on_req, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(String, '/safety/state', self.on_safety, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 5)
        self.create_subscription(Float32, '/uwb/innovation', self.on_innov, 20)
        self.create_subscription(Float32, '/uwb/reject_count', self.on_reject, 10)

        self.create_timer(1.0 / float(g('rate_hz')), self.on_timer)
        self.get_logger().info(f"Logging to {self.path}")

    def on_left(self, m): self.v['uwb_left_m'] = round(m.range, 4)
    def on_right(self, m): self.v['uwb_right_m'] = round(m.range, 4)
    def on_rssi_l(self, m): self.v['rssi_left'] = m.data
    def on_rssi_r(self, m): self.v['rssi_right'] = m.data

    def on_target(self, m):
        x, y = m.point.x, m.point.y
        self.v['target_x'] = round(x, 4)
        self.v['target_y'] = round(y, 4)
        rng = math.hypot(x, y)
        self.v['target_range_m'] = round(rng, 4)
        self.v['target_bearing_deg'] = round(math.degrees(math.atan2(y, x)), 2)
        self.v['dist_hold_err_m'] = round(rng - self.standoff, 4)

    def on_tag_odom(self, m):
        self.v['tag_odom_x'] = round(m.point.x, 4)
        self.v['tag_odom_y'] = round(m.point.y, 4)

    def on_odom(self, m):
        self.v['odom_x'] = round(m.pose.pose.position.x, 4)
        self.v['odom_y'] = round(m.pose.pose.position.y, 4)
        q = m.pose.pose.orientation
        th = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.v['odom_theta_deg'] = round(math.degrees(th), 2)

    def on_req(self, m):
        self.v['req_lin'] = round(m.linear.x, 4)
        self.v['req_ang'] = round(m.angular.z, 4)

    def on_cmd(self, m):
        self.v['cmd_lin'] = round(m.linear.x, 4)
        self.v['cmd_ang'] = round(m.angular.z, 4)

    def on_safety(self, m):
        self.v['safety_state'] = m.data.split()[0] if m.data else ''

    def on_innov(self, m): self.v['innovation_m'] = round(m.data, 4)
    def on_reject(self, m): self.v['reject_count'] = m.data

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
        self.v['lidar_front_min_m'] = round(best, 4) if best != float('inf') else ''

    def on_timer(self):
        self.v['t_s'] = round((self.get_clock().now() - self.t0).nanoseconds / 1e9, 3)
        self.writer.writerow([self.v[c] for c in COLUMNS])
        self.fh.flush()

    def destroy_node(self):
        try:
            self.fh.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Saved {node.path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
