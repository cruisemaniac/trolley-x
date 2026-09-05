"""UWB follow controller."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class FollowController(Node):
    def __init__(self):
        super().__init__('follow_controller')

        self.declare_parameter('left_topic', '/uwb/left')
        self.declare_parameter('right_topic', '/uwb/right')
        self.declare_parameter('output_topic', '/motion_request')
        self.declare_parameter('standoff_m', 1.0)       # follow distance
        self.declare_parameter('deadband_m', 0.15)      # no-move band around standoff
        self.declare_parameter('k_linear', 0.6)         # m/s per m of range error
        self.declare_parameter('k_angular', 1.5)        # rad/s per m of L-R difference
        self.declare_parameter('max_linear_mps', 0.4)
        self.declare_parameter('max_angular_rps', 1.0)
        self.declare_parameter('turn_hold_m', 0.5)      # L-R diff that fully holds forward
        self.declare_parameter('allow_reverse', False)  # back off if closer than standoff
        self.declare_parameter('min_valid_m', 0.15)
        self.declare_parameter('max_valid_m', 30.0)
        self.declare_parameter('range_timeout_s', 0.6)
        self.declare_parameter('ema_alpha', 0.4)        # 1.0 = no smoothing
        self.declare_parameter('publish_rate_hz', 15.0)

        g = lambda n: self.get_parameter(n).value
        self.standoff = float(g('standoff_m'))
        self.deadband = float(g('deadband_m'))
        self.k_lin = float(g('k_linear'))
        self.k_ang = float(g('k_angular'))
        self.max_lin = float(g('max_linear_mps'))
        self.max_ang = float(g('max_angular_rps'))
        self.turn_hold = float(g('turn_hold_m'))
        self.allow_reverse = bool(g('allow_reverse'))
        self.min_valid = float(g('min_valid_m'))
        self.max_valid = float(g('max_valid_m'))
        self.range_timeout = float(g('range_timeout_s'))
        self.alpha = float(g('ema_alpha'))

        self.dl = None
        self.dr = None
        self.tl = None
        self.tr = None
        self.state = 'INIT'

        self.create_subscription(Range, g('left_topic'), self.on_left, 10)
        self.create_subscription(Range, g('right_topic'), self.on_right, 10)
        self.pub = self.create_publisher(Twist, g('output_topic'), 10)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self.on_timer)

        self.get_logger().info(
            f"Follow controller active. standoff {self.standoff} m -> {g('output_topic')}")

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

    def _fresh(self, stamp):
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 <= self.range_timeout

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def on_timer(self):
        cmd = Twist()
        if not (self._fresh(self.tl) and self._fresh(self.tr)) or self.dl is None or self.dr is None:
            self._log_state('LOST')
            self.pub.publish(cmd)   # zero -> stop
            return

        diff = self.dr - self.dl            # >0 : tag to the left
        mean = 0.5 * (self.dl + self.dr)
        range_err = mean - self.standoff

        ang = self._clamp(self.k_ang * diff, -self.max_ang, self.max_ang)
        align = max(0.0, 1.0 - abs(diff) / self.turn_hold) if self.turn_hold > 0 else 1.0

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
        self._log_state(state, mean, diff)

    def _log_state(self, state, mean=None, diff=None):
        if state != self.state:
            if mean is not None:
                self.get_logger().info(f"{state}: range {mean:.2f} m, L-R {diff:+.2f} m")
            else:
                self.get_logger().warn(f"{state}: no UWB fix -> stop")
            self.state = state


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
