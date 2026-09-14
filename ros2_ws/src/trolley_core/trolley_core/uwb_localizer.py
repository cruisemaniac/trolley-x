"""UWB + odometry EKF localizer for the operator tag.

The two cart anchors range the operator's tag. This node fuses the two range
streams with the cart's odometry into a smooth estimate of the tag position, in
the fixed odom frame, using a constant-velocity model for the (walking)
operator. It then publishes the tag position in the base_link frame for the
follow controller, which turns it into range + bearing.

State (odom frame): [px, py, vx, vy]  (tag position + velocity).

v2 changes (calmer filter - Judhi: v1 was jerky/stuttering):
  - process_accel_std default lowered 1.5 -> 0.6 (less aggressive prediction).
  - operator velocity is clamped to a walking-speed ceiling.
  - innovation and a cumulative gate-reject count are published for evaluation
    (filter consistency / ride-through robustness, per the EKF references).

Publishes:
  /follow/target     geometry_msgs/PointStamped (base_link)  fused tag, for follow
  /uwb/tag_odom      geometry_msgs/PointStamped (odom)        for RViz / evaluation
  /uwb/innovation    std_msgs/Float32  signed range innovation of accepted updates
  /uwb/reject_count  std_msgs/Float32  cumulative count of gate-rejected ranges
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


class UwbLocalizer(Node):
    def __init__(self):
        super().__init__('uwb_localizer')

        self.declare_parameter('anchor_x', 0.35)
        self.declare_parameter('anchor_baseline', 0.30)
        self.declare_parameter('left_topic', '/uwb/left')
        self.declare_parameter('right_topic', '/uwb/right')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('target_topic', '/follow/target')
        self.declare_parameter('odom_target_topic', '/uwb/tag_odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('range_noise_std', 0.15)
        self.declare_parameter('process_accel_std', 0.6)     # was 1.5; calmer
        self.declare_parameter('max_operator_speed', 2.0)    # m/s clamp on the tag velocity
        self.declare_parameter('min_valid_m', 0.15)
        self.declare_parameter('max_valid_m', 30.0)
        self.declare_parameter('innovation_gate_m', 1.0)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('publish_debug', True)

        g = lambda n: self.get_parameter(n).value
        ax = float(g('anchor_x'))
        half = float(g('anchor_baseline')) / 2.0
        self.mounts = {'left': (ax, +half), 'right': (ax, -half)}
        self.base_frame = g('base_frame')
        self.odom_frame = g('odom_frame')
        self.R = float(g('range_noise_std')) ** 2
        self.q = float(g('process_accel_std')) ** 2
        self.vmax = float(g('max_operator_speed'))
        self.min_valid = float(g('min_valid_m'))
        self.max_valid = float(g('max_valid_m'))
        self.gate = float(g('innovation_gate_m'))
        self.debug = bool(g('publish_debug'))

        self.x = None
        self.P = None
        self.t = None
        self.cart = None
        self.reject_count = 0.0

        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 20)
        self.create_subscription(Range, g('left_topic'),
                                 lambda m: self.on_range('left', m), 20)
        self.create_subscription(Range, g('right_topic'),
                                 lambda m: self.on_range('right', m), 20)
        self.tgt_pub = self.create_publisher(PointStamped, g('target_topic'), 10)
        self.odom_tgt_pub = self.create_publisher(PointStamped, g('odom_target_topic'), 10)
        self.innov_pub = self.create_publisher(Float32, '/uwb/innovation', 10)
        self.reject_pub = self.create_publisher(Float32, '/uwb/reject_count', 10)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self.on_timer)
        self.get_logger().info("UWB localizer (EKF v2, calmer) active.")

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.cart = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)

    def _anchor_odom(self, name):
        xc, yc, th = self.cart
        mx, my = self.mounts[name]
        return (xc + mx * math.cos(th) - my * math.sin(th),
                yc + mx * math.sin(th) + my * math.cos(th))

    def _clamp_velocity(self):
        vx, vy = self.x[2], self.x[3]
        sp = math.hypot(vx, vy)
        if sp > self.vmax and sp > 1e-6:
            k = self.vmax / sp
            self.x[2] *= k
            self.x[3] *= k

    def _predict(self, now):
        if self.x is None or self.t is None:
            self.t = now
            return
        dt = (now - self.t).nanoseconds / 1e9
        if dt <= 0.0:
            return
        dt = min(dt, 1.0)
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        self.x = F @ self.x
        self._clamp_velocity()
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = self.q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                               [0, dt4 / 4, 0, dt3 / 2],
                               [dt3 / 2, 0, dt2, 0],
                               [0, dt3 / 2, 0, dt2]], float)
        self.P = F @ self.P @ F.T + Q
        self.t = now

    def on_range(self, name, msg):
        if self.cart is None:
            return
        r = float(msg.range)
        if not (self.min_valid < r < self.max_valid):
            return
        now = self.get_clock().now()
        ax, ay = self._anchor_odom(name)
        if self.x is None:
            xc, yc, th = self.cart
            self.x = np.array([ax + r * math.cos(th), ay + r * math.sin(th), 0.0, 0.0])
            self.P = np.diag([r * r, r * r, 1.0, 1.0]).astype(float)
            self.t = now
            return
        self._predict(now)
        dx, dy = self.x[0] - ax, self.x[1] - ay
        h = math.hypot(dx, dy)
        if h < 1e-3:
            return
        y = r - h                                   # innovation
        if self.gate > 0.0 and abs(y) > self.gate:
            self.reject_count += 1.0
            if self.debug:
                self.reject_pub.publish(Float32(data=self.reject_count))
            return
        H = np.array([[dx / h, dy / h, 0.0, 0.0]], float)
        S = (H @ self.P @ H.T)[0, 0] + self.R
        K = (self.P @ H.T) / S
        self.x = self.x + K[:, 0] * y
        self.P = (np.eye(4) - K @ H) @ self.P
        self._clamp_velocity()
        if self.debug:
            self.innov_pub.publish(Float32(data=float(y)))

    def on_timer(self):
        if self.x is None or self.cart is None:
            return
        now = self.get_clock().now()
        self._predict(now)
        px, py = float(self.x[0]), float(self.x[1])

        po = PointStamped()
        po.header.stamp = now.to_msg()
        po.header.frame_id = self.odom_frame
        po.point.x, po.point.y = px, py
        self.odom_tgt_pub.publish(po)

        xc, yc, th = self.cart
        c, s = math.cos(th), math.sin(th)
        pb = PointStamped()
        pb.header.stamp = now.to_msg()
        pb.header.frame_id = self.base_frame
        pb.point.x = (px - xc) * c + (py - yc) * s
        pb.point.y = -(px - xc) * s + (py - yc) * c
        self.tgt_pub.publish(pb)


def main(args=None):
    rclpy.init(args=args)
    node = UwbLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
