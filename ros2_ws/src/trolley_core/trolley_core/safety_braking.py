"""LiDAR safety supervisor for motion requests."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from trolley_core.diagnostics import Diagnostics, OK, WARN, ERROR, CRITICAL, error_level


class SafetyBrakingNode(Node):
    def __init__(self):
        super().__init__('safety_braking_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('motion_request_topic', '/motion_request')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('safety_state_topic', '/safety/state')
        self.declare_parameter('stop_distance_m', 0.30)
        self.declare_parameter('slow_distance_m', 0.45)
        self.declare_parameter('warn_distance_m', 0.70)
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
        # Sectoring
        self.declare_parameter('front_arc_deg', 90.0)     # width of FRONT and REAR arcs
        self.declare_parameter('bearing_offset_deg', 0.0)  # rotate: lidar 0 -> cart forward
        self.declare_parameter('invert_scan', False)       # negate beam angle for a flipped/reversed mount

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
        self.half_front = math.radians(float(g('front_arc_deg')) / 2.0)
        self.bearing_offset = math.radians(float(g('bearing_offset_deg')))
        self.invert_scan = bool(g('invert_scan'))

        self.sect = {'FRONT': float('inf'), 'REAR': float('inf'),
                     'LEFT': float('inf'), 'RIGHT': float('inf')}
        self.min_dist = float('inf')
        self.min_bearing = 0.0
        self.last_scan_time = None
        self.latest_request = None
        self.last_request_time = None
        self.state = 'INIT'

        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, 10)
        self.create_subscription(Twist, g('motion_request_topic'), self.on_request, 10)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.state_pub = self.create_publisher(String, g('safety_state_topic'), 10)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self.on_timer)

        self.diag = Diagnostics(self)
        self.diag.add('lidar scan', self._diag_scan)
        self.diag.add('obstacle zone', self._diag_zone)
        self.diag.add('motion request', self._diag_request)

        self.get_logger().info(
            f"Sectorized safety active. zones stop<={self.stop_d} "
            f"slow<={self.slow_d} warn<={self.warn_d} m; "
            f"front/rear arc {g('front_arc_deg')} deg")

    @staticmethod
    def _wrap(a):
        # normalize to (-pi, pi]
        return math.atan2(math.sin(a), math.cos(a))

    def _sector(self, bearing):
        if abs(bearing) <= self.half_front:
            return 'FRONT'
        if abs(bearing) >= math.pi - self.half_front:
            return 'REAR'
        return 'LEFT' if bearing > 0.0 else 'RIGHT'

    def _zone(self, d):
        if d <= self.stop_d:
            return 'STOP'
        if d <= self.slow_d:
            return 'SLOW'
        if d <= self.warn_d:
            return 'WARN'
        return 'CLEAR'

    def on_scan(self, msg):
        x_min, x_max, y_min, y_max = self.box
        front = rear = left = right = float('inf')
        best = float('inf')
        best_bearing = 0.0
        for i, r in enumerate(msg.ranges):
            if not (self.range_min < r < self.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            if (x_min < x < x_max) and (y_min < y < y_max):
                continue  # ignore the cart's own footprint
            raw = -angle if self.invert_scan else angle
            bearing = self._wrap(raw + self.bearing_offset)
            sect = self._sector(bearing)
            if sect == 'FRONT' and r < front:
                front = r
            elif sect == 'REAR' and r < rear:
                rear = r
            elif sect == 'LEFT' and r < left:
                left = r
            elif sect == 'RIGHT' and r < right:
                right = r
            if r < best:
                best = r
                best_bearing = bearing
        self.sect = {'FRONT': front, 'REAR': rear, 'LEFT': left, 'RIGHT': right}
        self.min_dist = best
        self.min_bearing = best_bearing
        self.last_scan_time = self.get_clock().now()

    def on_request(self, msg):
        self.latest_request = msg
        self.last_request_time = self.get_clock().now()

    def _stale(self, stamp, timeout):
        if stamp is None:
            return True
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 > timeout

    # ---- diagnostics ----
    def _age(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _diag_scan(self):
        age = self._age(self.last_scan_time)
        if age is None:
            return ERROR, 'no scan received; holding STOP', error_level(CRITICAL)
        values = {'scan_age_s': f'{age:.2f}', 'timeout_s': self.scan_timeout}
        if age > self.scan_timeout:
            return ERROR, f'scan stale for {age:.1f} s; holding STOP', error_level(CRITICAL, values)
        return OK, 'scan fresh', values

    def _diag_zone(self):
        values = {'state': self.state,
                  'front_m': f"{self.sect['FRONT']:.2f}", 'rear_m': f"{self.sect['REAR']:.2f}",
                  'left_m': f"{self.sect['LEFT']:.2f}", 'right_m': f"{self.sect['RIGHT']:.2f}"}
        if self._stale(self.last_scan_time, self.scan_timeout):
            return WARN, 'unknown: no fresh scan', values
        if self.state == 'STOP':
            # The safety system doing its job, not a fault: WARN, not ERROR.
            return WARN, f'STOP: obstacle at {self.min_dist:.2f} m', values
        return OK, self.state, values

    def _diag_request(self):
        age = self._age(self.last_request_time)
        if age is None:
            return WARN, 'no motion request received', {}
        values = {'request_age_s': f'{age:.2f}', 'timeout_s': self.cmd_timeout}
        if age > self.cmd_timeout:
            return WARN, f'no motion request for {age:.1f} s', values
        return OK, 'receiving motion requests', values

    def _limit(self, linear, sector_dist):
        # apply the zone for one direction; returns limited linear.x
        z = self._zone(sector_dist)
        if z == 'STOP':
            return 0.0
        if z == 'SLOW':
            return max(-self.slow_max, min(self.slow_max, linear))
        return linear

    def on_timer(self):
        scan_stale = self._stale(self.last_scan_time, self.scan_timeout)
        if scan_stale:
            front = rear = left = right = 0.0  # force STOP everywhere
        else:
            front, rear = self.sect['FRONT'], self.sect['REAR']
            left, right = self.sect['LEFT'], self.sect['RIGHT']

        cmd = Twist()
        if self.latest_request is not None and not self._stale(self.last_request_time, self.cmd_timeout):
            cmd.linear.x = float(self.latest_request.linear.x)
            cmd.angular.z = float(self.latest_request.angular.z)

        # gate linear motion by the sector it drives into; rotation stays free
        if cmd.linear.x > 0.0:
            cmd.linear.x = self._limit(cmd.linear.x, front)
        elif cmd.linear.x < 0.0:
            cmd.linear.x = self._limit(cmd.linear.x, rear)

        # full stop (including rotation) when an obstacle is right in front
        if front <= self.stop_d:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        overall = min(front, rear, left, right)
        state = 'STOP' if scan_stale else self._zone(overall)

        s = String()
        if scan_stale:
            s.data = 'STOP scan_stale'
        else:
            s.data = (f'{state} F{front:.2f} R{right:.2f} B{rear:.2f} L{left:.2f}')
        self.state_pub.publish(s)

        if state != self.state:
            if scan_stale:
                self.get_logger().error('STOP: scan stale -> holding STOP')
            else:
                note = f'nearest {overall:.2f} m @ {math.degrees(self.min_bearing):+.0f} deg'
                if state == 'STOP':
                    self.get_logger().error(f'STOP: {note}')
                elif state == 'SLOW':
                    self.get_logger().warn(f'SLOW: {note}')
                elif state == 'WARN':
                    self.get_logger().info(f'WARN: {note}')
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
