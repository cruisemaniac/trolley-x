"""Arduino serial bridge with odometry and IMU output."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import threading


def _cov(entries):
    c = [0.0] * 36
    for i, v in entries.items():
        c[i] = v
    return c


class ArduinoBaseController(Node):
    def __init__(self):
        super().__init__('arduino_base_controller')

        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('max_linear_mps', 0.5)
        self.declare_parameter('max_angular_rps', 1.5)
        self.declare_parameter('max_pwm', 160)
        self.declare_parameter('min_move_pwm', 60)   # overcome the gearboxes
        self.declare_parameter('min_turn_pwm', 120)  # skid-steer needs torque
        # Odometry / IMU
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_odom_tf', True)   # off if an EKF owns odom->base_link
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'base_link')
        self.declare_parameter('gyro_bias_samples', 60)   # ~3 s at 20 Hz, stay still
        self.declare_parameter('max_wheel_jump_m', 1.0)   # reject Arduino resets

        g = lambda n: self.get_parameter(n).value
        self.serial_port = g('serial_port')
        self.baud_rate = int(g('baud_rate'))
        self.max_lin = float(g('max_linear_mps'))
        self.max_ang = float(g('max_angular_rps'))
        self.max_pwm = int(g('max_pwm'))
        self.min_move = int(g('min_move_pwm'))
        self.min_turn = int(g('min_turn_pwm'))
        self.publish_odom = bool(g('publish_odom'))
        self.publish_tf = bool(g('publish_odom_tf'))
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.imu_frame = g('imu_frame')
        self.bias_n = int(g('gyro_bias_samples'))
        self.max_jump = float(g('max_wheel_jump_m'))

        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")

        self.subscription = self.create_subscription(
            Twist, g('cmd_vel_topic'), self.cmd_vel_callback, 10)
        self.imu_pub = self.create_publisher(Imu, g('imu_topic'), 10)
        self.odom_pub = self.create_publisher(Odometry, g('odom_topic'), 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # odom state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = None
        self.last_right = None
        self.last_time = None
        # gyro bias calibration
        self.gyro_bias = 0.0
        self.bias_sum = 0.0
        self.bias_count = 0
        self.calibrated = False

        self.read_thread = threading.Thread(target=self.read_telemetry, daemon=True)
        self.read_thread.start()

    # ---- drive ----
    def _scale(self, value, max_value, floor_pwm):
        frac = min(1.0, abs(value) / max_value) if max_value > 0 else 0.0
        pwm = int(round(frac * self.max_pwm))
        return max(floor_pwm, min(self.max_pwm, pwm))

    def cmd_vel_callback(self, msg):
        if msg.linear.x > 0.0:
            command = f"W {self._scale(msg.linear.x, self.max_lin, self.min_move)}"
        elif msg.linear.x < 0.0:
            command = f"S {self._scale(msg.linear.x, self.max_lin, self.min_move)}"
        elif msg.angular.z > 0.0:
            command = f"A {self._scale(msg.angular.z, self.max_ang, self.min_turn)}"
        elif msg.angular.z < 0.0:
            command = f"D {self._scale(msg.angular.z, self.max_ang, self.min_turn)}"
        else:
            command = "X"

        if hasattr(self, 'arduino') and self.arduino.is_open:
            self.arduino.write((command + "\n").encode('utf-8'))

    # ---- sense ----
    def read_telemetry(self):
        while rclpy.ok():
            if not (hasattr(self, 'arduino') and self.arduino.is_open):
                continue
            try:
                line = self.arduino.readline().decode('utf-8').strip()
            except Exception:
                continue
            if not line:
                continue
            parts = line.split(',')
            if len(parts) != 4:
                self.get_logger().debug(f"telemetry (unparsed): [{line}]")
                continue
            try:
                left, right, accel_x, gyro_z = (float(p) for p in parts)
            except ValueError:
                continue
            now = self.get_clock().now()
            self._publish_imu(now, accel_x, gyro_z)
            if self.publish_odom:
                self._update_odom(now, left, right, gyro_z)

    def _publish_imu(self, now, accel_x, gyro_z):
        m = Imu()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.imu_frame
        m.orientation_covariance[0] = -1.0            # no orientation estimate
        m.angular_velocity.z = float(gyro_z)          # rad/s, raw
        m.linear_acceleration.x = float(accel_x)      # m/s^2
        m.angular_velocity_covariance = _cov({0: 1e6, 4: 1e6, 8: 0.02})
        m.linear_acceleration_covariance = _cov({0: 0.05, 4: 1e6, 8: 1e6})
        self.imu_pub.publish(m)

    def _update_odom(self, now, left, right, gyro_z):
        # gyro bias: average a stationary window at startup
        if not self.calibrated:
            self.bias_sum += gyro_z
            self.bias_count += 1
            if self.bias_count >= self.bias_n:
                self.gyro_bias = self.bias_sum / self.bias_count
                self.calibrated = True
                self.get_logger().info(
                    f"gyro bias calibrated: {self.gyro_bias:+.4f} rad/s")
            self.last_left, self.last_right, self.last_time = left, right, now
            return

        if self.last_time is None:
            self.last_left, self.last_right, self.last_time = left, right, now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:                     # stale / first / gap
            self.last_left, self.last_right, self.last_time = left, right, now
            return

        dl = left - self.last_left
        dr = right - self.last_right
        if abs(dl) > self.max_jump or abs(dr) > self.max_jump:   # Arduino reset
            self.last_left, self.last_right, self.last_time = left, right, now
            return

        ds = 0.5 * (dl + dr)
        w = gyro_z - self.gyro_bias
        dtheta = w * dt
        theta_mid = self.theta + 0.5 * dtheta
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        self.theta = math.atan2(math.sin(self.theta + dtheta),
                                math.cos(self.theta + dtheta))
        v = ds / dt

        self.last_left, self.last_right, self.last_time = left, right, now
        self._publish_odom(now, v, w)

    def _publish_odom(self, now, v, w):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.pose.covariance = _cov({0: 0.02, 7: 0.02, 14: 1e6, 21: 1e6, 28: 1e6, 35: 0.05})
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        od.twist.covariance = _cov({0: 0.02, 7: 1e6, 14: 1e6, 21: 1e6, 28: 1e6, 35: 0.05})
        self.odom_pub.publish(od)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'arduino') and node.arduino.is_open:
            node.arduino.write("X\n".encode('utf-8'))
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
