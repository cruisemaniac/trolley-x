"""LiDAR safety supervisor for motion requests."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SafetyBrakingNode(Node):
    def __init__(self):
        super().__init__('safety_braking_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('motion_request_topic', '/motion_request')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('safety_state_topic', '/safety/state')
        self.declare_parameter('stop_distance_m', 0.5)
        self.declare_parameter('slow_distance_m', 1.0)
        self.declare_parameter('warn_distance_m', 2.0)
        self.declare_parameter('slow_max_linear_mps', 0.15)
        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('range_max_m', 12.0)
        self.declare_parameter('cart_x_min', -0.10)
        self.declare_parameter('cart_x_max', 0.35)
        self.declare_parameter('cart_y_min', -0.20)
        self.declare_parameter('cart_y_max', 0.20)
        self.declare_parameter('scan_timeout_s', 0.5)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        g = lambda n: self.get_parameter(n).value
        self.stop_d = g('stop_distance_m')
        self.slow_d = g('slow_distance_m')
        self.warn_d = g('warn_distance_m')
        self.slow_max = g('slow_max_linear_mps')
        self.range_min = g('range_min_m')
        self.range_max = g('range_max_m')
        self.box = (g('cart_x_min'), g('cart_x_max'), g('cart_y_min'), g('cart_y_max'))
        self.scan_timeout = g('scan_timeout_s')
        self.cmd_timeout = g('command_timeout_s')

        self.min_dist = float('inf')
        self.last_scan_time = None
        self.latest_request = None
        self.last_request_time = None
        self.state = 'INIT'

        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, 10)
        self.create_subscription(Twist, g('motion_request_topic'), self.on_request, 10)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.state_pub = self.create_publisher(String, g('safety_state_topic'), 10)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self.on_timer)

        self.get_logger().info(
            f"Safety supervisor active. zones stop<={self.stop_d} "
            f"slow<={self.slow_d} warn<={self.warn_d} m")

    def on_scan(self, msg):
        x_min, x_max, y_min, y_max = self.box
        best = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (self.range_min < r < self.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            if (x_min < x < x_max) and (y_min < y < y_max):
                continue  # ignore the cart's own footprint
            if r < best:
                best = r
        self.min_dist = best
        self.last_scan_time = self.get_clock().now()

    def on_request(self, msg):
        self.latest_request = msg
        self.last_request_time = self.get_clock().now()

    def _stale(self, stamp, timeout):
        if stamp is None:
            return True
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 > timeout

    def _zone(self, d):
        if d <= self.stop_d:
            return 'STOP'
        if d <= self.slow_d:
            return 'SLOW'
        if d <= self.warn_d:
            return 'WARN'
        return 'CLEAR'

    def on_timer(self):
        scan_stale = self._stale(self.last_scan_time, self.scan_timeout)
        d = self.min_dist
        state = 'STOP' if scan_stale else self._zone(d)

        cmd = Twist()
        if self.latest_request is not None and not self._stale(self.last_request_time, self.cmd_timeout):
            cmd.linear.x = float(self.latest_request.linear.x)
            cmd.angular.z = float(self.latest_request.angular.z)

        if state == 'STOP':
            cmd.linear.x = 0.0  # forward/back blocked; rotation still allowed
        elif state == 'SLOW':
            cmd.linear.x = max(-self.slow_max, min(self.slow_max, cmd.linear.x))

        self.cmd_pub.publish(cmd)

        msg = String()
        msg.data = state if not math.isfinite(d) else f'{state} {d:.2f}'
        self.state_pub.publish(msg)

        if state != self.state:
            note = 'scan timeout -> holding STOP' if scan_stale else f'obstacle at {d:.2f} m'
            if state == 'STOP':
                self.get_logger().error(f'STOP zone: {note}')
            elif state == 'SLOW':
                self.get_logger().warn(f'SLOW zone: {note}')
            elif state == 'WARN':
                self.get_logger().info(f'WARN zone: {note}')
            else:
                self.get_logger().info('CLEAR')
            self.state = state


def main(args=None):
    rclpy.init(args=args)
    node = SafetyBrakingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # stop on the way out
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
