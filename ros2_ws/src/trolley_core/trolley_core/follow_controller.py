"""UWB follow controller (v2 - blended arc control).

Drives the cart to follow the operator's tag at a set stand-off, publishing
/motion_request (which passes through safety_braking).

WHAT CHANGED FROM v1
  The old controller drove OR turned (it drove straight only when nearly aligned,
  and the base then ignored the turn). With the new differential base, this node
  now commands linear AND angular TOGETHER, so the cart arcs toward the operator.
  It only falls back to a pure spin when the bearing is very large. A bearing
  deadband and angular slew-rate limit remove the left-right hunting.

Two input modes:
  use_target = False: raw differential ranging. Read /uwb/left and /uwb/right,
    steer on d_right-d_left, speed on the mean range.
  use_target = True (follow.launch): consume the EKF fused tag on /follow/target
    (base_link) for a TRUE bearing and range - smoother, rides through dropouts.

Either way: arc toward the tag, keep the stand-off, and stop (publish zero) if
the input goes stale.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, PointStamped


class FollowController(Node):
    def __init__(self):
        super().__init__('follow_controller')

        self.declare_parameter('use_target', False)
        self.declare_parameter('left_topic', '/uwb/left')
        self.declare_parameter('right_topic', '/uwb/right')
        self.declare_parameter('target_topic', '/follow/target')
        self.declare_parameter('output_topic', '/motion_request')
        self.declare_parameter('standoff_m', 0.45)      # follow gap to the front anchors (m)
        self.declare_parameter('deadband_m', 0.08)      # hold band; follow when gap > standoff+deadband
        self.declare_parameter('range_offset_m', 0.30)  # EKF range is from base_link; subtract anchor_x for the gap
        self.declare_parameter('k_linear', 0.6)             # m/s per m of range error
        self.declare_parameter('k_angular', 1.2)            # raw mode: rad/s per m of L-R diff
        self.declare_parameter('k_bearing', 1.0)            # target mode: rad/s per rad of bearing
        self.declare_parameter('max_linear_mps', 0.4)
        self.declare_parameter('max_angular_rps', 1.0)
        self.declare_parameter('turn_first_rad', 1.2)       # far off to the side => turn first, only creep forward
        self.declare_parameter('creep_frac', 0.5)           # forward fraction while turning hard (keeps an arc)
        self.declare_parameter('bearing_deadband_rad', 0.09)  # ignore bearing under ~5 deg (kills hunting)
        self.declare_parameter('raw_diff_deadband_m', 0.10)   # raw mode: ignore small L-R diff
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('min_valid_m', 0.15)
        self.declare_parameter('max_valid_m', 30.0)
        self.declare_parameter('input_timeout_s', 0.6)
        self.declare_parameter('ema_alpha', 0.4)            # raw-range smoothing; 1.0 = off
        self.declare_parameter('slew_lin_mps2', 0.8)        # max change in linear per second
        self.declare_parameter('slew_ang_rps2', 3.0)        # max change in angular per second
        self.declare_parameter('publish_rate_hz', 15.0)

        g = lambda n: self.get_parameter(n).value
        self.use_target = bool(g('use_target'))
        self.standoff = float(g('standoff_m'))
        self.deadband = float(g('deadband_m'))
        self.range_off = float(g('range_offset_m'))
        self.k_lin = float(g('k_linear'))
        self.k_ang = float(g('k_angular'))
        self.k_bearing = float(g('k_bearing'))
        self.max_lin = float(g('max_linear_mps'))
        self.max_ang = float(g('max_angular_rps'))
        self.turn_first = float(g('turn_first_rad'))
        self.creep = float(g('creep_frac'))
        self.bear_db = float(g('bearing_deadband_rad'))
        self.raw_db = float(g('raw_diff_deadband_m'))
        self.allow_reverse = bool(g('allow_reverse'))
        self.min_valid = float(g('min_valid_m'))
        self.max_valid = float(g('max_valid_m'))
        self.timeout = float(g('input_timeout_s'))
        self.alpha = float(g('ema_alpha'))
        self.rate = float(g('publish_rate_hz'))
        self.dt = 1.0 / self.rate
        self.slew_lin = float(g('slew_lin_mps2')) * self.dt
        self.slew_ang = float(g('slew_ang_rps2')) * self.dt

        self.dl = self.dr = None
        self.tl = self.tr = None
        self.target = None
        self.tt = None
        self.state = 'INIT'
        self.prev_lin = 0.0
        self.prev_ang = 0.0

        self.create_subscription(Range, g('left_topic'), self.on_left, 10)
        self.create_subscription(Range, g('right_topic'), self.on_right, 10)
        self.create_subscription(PointStamped, g('target_topic'), self.on_target, 10)
        self.pub = self.create_publisher(Twist, g('output_topic'), 10)
        self.create_timer(self.dt, self.on_timer)

        mode = 'EKF target' if self.use_target else 'raw ranges'
        self.get_logger().info(
            f"Follow controller v2 active ({mode}). standoff {self.standoff} m -> {g('output_topic')}")

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

    def _slew(self, prev, target, step):
        if target > prev + step:
            return prev + step
        if target < prev - step:
            return prev - step
        return target

    def _publish(self, lin, ang, state, diag):
        lin = self._slew(self.prev_lin, lin, self.slew_lin)
        ang = self._slew(self.prev_ang, ang, self.slew_ang)
        self.prev_lin, self.prev_ang = lin, ang
        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.pub.publish(cmd)
        self._log(state, diag)

    def on_timer(self):
        if self.use_target:
            if not (self._fresh(self.tt) and self.target is not None):
                self._publish(0.0, 0.0, 'LOST', None); return
            bx, by = self.target
            bearing = math.atan2(by, bx)                 # bearing from the EKF
            rng = math.hypot(bx, by)                      # EKF range from base_link
            frange = max(0.0, rng - self.range_off)       # gap to the front anchors
            heading = 0.0 if abs(bearing) < self.bear_db else bearing
            ang_cmd = self._clamp(self.k_bearing * heading, -self.max_ang, self.max_ang)
            off = abs(bearing)
            diag = f"gap {frange:.2f} m, bearing {math.degrees(bearing):+.0f} deg"
        else:
            if not (self._fresh(self.tl) and self._fresh(self.tr)) or self.dl is None or self.dr is None:
                self._publish(0.0, 0.0, 'LOST', None); return
            diff = self.dr - self.dl                     # >0 : tag to the left
            frange = 0.5 * (self.dl + self.dr)
            steer = 0.0 if abs(diff) < self.raw_db else diff
            ang_cmd = self._clamp(self.k_ang * steer, -self.max_ang, self.max_ang)
            off = min(math.pi / 2.0, abs(diff))
            diag = f"gap {frange:.2f} m, L-R {diff:+.2f} m"

        range_err = frange - self.standoff
        if range_err > self.deadband:
            # the operator has moved away -> follow: drive forward AND steer (arc)
            lin = self._clamp(self.k_lin * range_err, 0.0, self.max_lin)
            if off > self.turn_first:          # far off to the side: turn first, only creep forward
                lin *= self.creep
            ang = ang_cmd
            state = 'FOLLOW'
        else:
            # within the hold band / too close -> full stop, never spin in place
            lin = 0.0
            ang = 0.0
            state = 'HOLD'
        self._publish(lin, ang, state, diag)

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
