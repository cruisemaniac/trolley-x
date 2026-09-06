"""UWB follow controller."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, PointStamped


class FollowController(Node):
    def __init__(self):
        super().__init__('follow_controller')

        self.declare_parameter('use_target', False)         # True = EKF fused target
        self.declare_parameter('left_topic', '/uwb/left')
        self.declare_parameter('right_topic', '/uwb/right')
        self.declare_parameter('target_topic', '/follow/target')
        self.declare_parameter('output_topic', '/motion_request')
        self.declare_parameter('standoff_m', 1.0)
        self.declare_parameter('deadband_m', 0.15)
        self.declare_parameter('k_linear', 0.6)             # m/s per m of range error
        self.declare_parameter('k_angular', 1.5)            # raw mode: rad/s per m of L-R diff
        self.declare_parameter('k_bearing', 1.2)            # target mode: rad/s per rad of bearing
        self.declare_parameter('max_linear_mps', 0.4)
        self.declare_parameter('max_angular_rps', 1.0)
        self.declare_parameter('turn_hold_m', 0.5)          # raw mode: L-R diff that holds forward
        self.declare_parameter('turn_hold_rad', 0.6)        # target mode: bearing that holds forward
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('min_valid_m', 0.15)
        self.declare_parameter('max_valid_m', 30.0)
        self.declare_parameter('input_timeout_s', 0.6)
        self.declare_parameter('ema_alpha', 0.4)            # raw-range smoothing; 1.0 = off
        self.declare_parameter('publish_rate_hz', 15.0)

        g = lambda n: self.get_parameter(n).value
        self.use_target = bool(g('use_target'))
        self.standoff = float(g('standoff_m'))
        self.deadband = float(g('deadband_m'))
        self.k_lin = float(g('k_linear'))
        self.k_ang = float(g('k_angular'))
        self.k_bearing = float(g('k_bearing'))
        self.max_lin = float(g('max_linear_mps'))
        self.max_ang = float(g('max_angular_rps'))
        self.turn_hold = float(g('turn_hold_m'))
        self.turn_hold_rad = float(g('turn_hold_rad'))
        self.allow_reverse = bool(g('allow_reverse'))
        self.min_valid = float(g('min_valid_m'))
        self.max_valid = float(g('max_valid_m'))
        self.timeout = float(g('input_timeout_s'))
        self.alpha = float(g('ema_alpha'))

        self.dl = self.dr = None
        self.tl = self.tr = None
        self.target = None
        self.tt = None
        self.state = 'INIT'

        self.create_subscription(Range, g('left_topic'), self.on_left, 10)
        self.create_subscription(Range, g('right_topic'), self.on_right, 10)
        self.create_subscription(PointStamped, g('target_topic'), self.on_target, 10)
        self.pub = self.create_publisher(Twist, g('output_topic'), 10)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self.on_timer)

        mode = 'EKF target' if self.use_target else 'raw ranges'
        self.get_logger().info(
            f"Follow controller active ({mode}). standoff {self.standoff} m -> {g('output_topic')}")

    def _accept(self, r):
        return self.min_valid < r < self.max_valid

    def _ema(self, prev, new):
        return new if prev is None else self.alpha * new + (1.0 - self.alpha) * prev

    def on_left(self, msg):
        if self._accept(msg.range):
            self.dl = self._ema(self.dl, float(msg.range))
            self.tl = self.get_clock().now()

    def on_right(self, msg):
        if self._accept(msg.range):
            self.dr = self._ema(self.dr, float(msg.range))
            self.tr = self.get_clock().now()

    def on_target(self, msg):
        self.target = (float(msg.point.x), float(msg.point.y))
        self.tt = self.get_clock().now()

    def _fresh(self, stamp):
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 <= self.timeout

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def on_timer(self):
        cmd = Twist()

        if self.use_target:
            if not (self._fresh(self.tt) and self.target is not None):
                self._log('LOST'); self.pub.publish(cmd); return
            bx, by = self.target
            rng = math.hypot(bx, by)
            bearing = math.atan2(by, bx)                 # true bearing, rad
            range_err = rng - self.standoff
            ang = self._clamp(self.k_bearing * bearing, -self.max_ang, self.max_ang)
            align = (max(0.0, 1.0 - abs(bearing) / self.turn_hold_rad)
                     if self.turn_hold_rad > 0 else 1.0)
            diag = f"range {rng:.2f} m, bearing {math.degrees(bearing):+.0f} deg"
        else:
            if not (self._fresh(self.tl) and self._fresh(self.tr)) or self.dl is None or self.dr is None:
                self._log('LOST'); self.pub.publish(cmd); return
            diff = self.dr - self.dl                     # >0 : tag to the left
            mean = 0.5 * (self.dl + self.dr)
            range_err = mean - self.standoff
            ang = self._clamp(self.k_ang * diff, -self.max_ang, self.max_ang)
            align = (max(0.0, 1.0 - abs(diff) / self.turn_hold)
                     if self.turn_hold > 0 else 1.0)
            diag = f"range {mean:.2f} m, L-R {diff:+.2f} m"

        if range_err > self.deadband:
            lin = self._clamp(self.k_lin * range_err, 0.0, self.max_lin) * align
            state = 'FOLLOW' if align > 0.5 else 'ALIGN'
        elif self.allow_reverse and range_err < -self.deadband:
            lin = self._clamp(self.k_lin * range_err, -self.max_lin, 0.0)
            state = 'BACKOFF'
        else:
            lin = 0.0
            state = 'HOLD'

        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.pub.publish(cmd)
        self._log(state, diag)

    def _log(self, state, diag=None):
        if state != self.state:
            if diag:
                self.get_logger().info(f"{state}: {diag}")
            else:
                self.get_logger().warn(f"{state}: no fix -> stop")
            self.state = state


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
