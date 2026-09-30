"""Arduino base controller + odometry/IMU bridge.

Two jobs:
  1. Drive: subscribe to the safety-limited /cmd_vel and send a per-wheel command
     so the cart can drive AND turn at the same time (arcs), not one or the other.
       (v, omega) -> left/right wheel speed -> signed PWM -> "M <left> <right>"
     A pure spin (v=0) and a straight drive (omega=0) are just special cases.
     "X" is sent when the command is zero.
  2. Sense: parse the Arduino's 20 Hz CSV telemetry
       left_m, right_m, accel_x, gyro_z
     and publish /imu/data (sensor_msgs/Imu) and /odom (nav_msgs/Odometry) with
     the odom->base_link TF.

Odometry is gyro-aided: distance from the wheel encoders, heading from the
integrated gyro (not the wheel difference), so it does NOT need the wheel track
width and is robust to skid-steer slip. The gyro bias is estimated from a short
stationary window at startup - keep the cart still until "gyro bias calibrated".

NOTE: wheel_track_m IS used for the drive mapping (v, omega -> wheel speeds).
Measure the left-right wheel contact separation and set it.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import threading
import time

from trolley_core.diagnostics import Diagnostics, OK, WARN, ERROR


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
        self.declare_parameter('max_pwm', 120)   # BTS7960: cap ~6 V on the 6 V motors
        self.declare_parameter('min_move_pwm', 60)    # overcome the gearboxes (drive)
        self.declare_parameter('min_turn_pwm', 120)   # skid-steer breakaway torque (turn)
        self.declare_parameter('wheel_track_m', 0.30) # MEASURE THIS: left-right wheel separation
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
        self.track = float(g('wheel_track_m'))
        self.publish_odom = bool(g('publish_odom'))
        self.publish_tf = bool(g('publish_odom_tf'))
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.imu_frame = g('imu_frame')
        self.bias_n = int(g('gyro_bias_samples'))
        self.max_jump = float(g('max_wheel_jump_m'))

        # Wheel-speed normaliser: the fastest a wheel is ever asked to go equals
        # a full-speed straight plus a full-speed turn. Map that to max_pwm.
        self.wheel_max_mps = self.max_lin + self.max_ang * self.track / 2.0
        if self.wheel_max_mps <= 0.0:
            self.wheel_max_mps = 0.5

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

        # telemetry bookkeeping for diagnostics (written by the read thread)
        self._telemetry_count = 0
        self._telemetry_last = None
        self._telemetry_unparsed = 0
        self._diag_last_count = 0
        self.diag = Diagnostics(self)
        self.diag.add('serial link', self._diag_serial)
        self.diag.add('gyro calibration', self._diag_gyro)

        self.read_thread = threading.Thread(target=self.read_telemetry, daemon=True)
        self.read_thread.start()
        self.get_logger().info(
            f"Base controller (differential). track {self.track} m, "
            f"wheel_max {self.wheel_max_mps:.2f} m/s -> pwm {self.max_pwm}")

    # ---- diagnostics ----
    def _diag_serial(self):
        rate = self._telemetry_count - self._diag_last_count  # lines in the last second
        self._diag_last_count = self._telemetry_count
        values = {'port': self.serial_port, 'telemetry_hz': rate, 'unparsed_lines': self._telemetry_unparsed}
        if not (hasattr(self, 'arduino') and self.arduino.is_open):
            return ERROR, f'serial port {self.serial_port} not open', values
        if self._telemetry_last is None:
            return WARN, 'no telemetry received yet', values
        age = time.monotonic() - self._telemetry_last
        values['telemetry_age_s'] = f'{age:.2f}'
        if age > 0.5:
            return ERROR, f'no telemetry from the Arduino for {age:.1f} s', values
        return OK, 'telemetry flowing', values

    def _diag_gyro(self):
        values = {'gyro_bias': f'{self.gyro_bias:.4f}'}
        if not self.calibrated:
            return WARN, 'calibrating gyro bias; keep the cart still', values
        return OK, 'calibrated', values

    # ---- drive ----
    def cmd_vel_callback(self, msg):
        v = max(-self.max_lin, min(self.max_lin, float(msg.linear.x)))
        w = max(-self.max_ang, min(self.max_ang, float(msg.angular.z)))

        # Differential kinematics: turn -> wheel-speed difference.
        vl = v - w * self.track / 2.0
        vr = v + w * self.track / 2.0

        # While driving, a wheel may NOT reverse: a tight turn becomes a pivot/arc,
        # never an in-place counter-rotating spin. Pure spin is only for v ~ 0.
        spinning = abs(v) < 0.02 and abs(w) > 1e-3
        if v > 0.02:
            vl = max(0.0, vl); vr = max(0.0, vr)
        elif v < -0.02:
            vl = min(0.0, vl); vr = min(0.0, vr)

        # To PWM, preserving the left/right ratio (the arc shape). One shared
        # movement floor lifts the FASTER wheel to break away; the slower wheel
        # keeps its proportion instead of being floored up into a spin.
        lpwm = abs(vl) / self.wheel_max_mps * self.max_pwm
        rpwm = abs(vr) / self.wheel_max_mps * self.max_pwm
        peak = max(lpwm, rpwm)
        floor = self.min_turn if spinning else self.min_move
        if peak < 1.0:
            left = right = 0
        else:
            if peak < floor:
                k = floor / peak
                lpwm *= k; rpwm *= k
            left = int(round(min(self.max_pwm, lpwm)))
            right = int(round(min(self.max_pwm, rpwm)))
            if vl < 0: left = -left
            if vr < 0: right = -right

        command = "X" if (left == 0 and right == 0) else f"M {left} {right}"

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
                self._telemetry_unparsed += 1
                self.get_logger().debug(f"telemetry (unparsed): [{line}]")
                continue
            try:
                left, right, accel_x, gyro_z = (float(p) for p in parts)
            except ValueError:
                continue
            self._telemetry_count += 1
            self._telemetry_last = time.monotonic()
            now = self.get_clock().now()
            self._publish_imu(now, accel_x, gyro_z)
            if self.publish_odom:
                self._update_odom(now, left, right, gyro_z)

    def _publish_imu(self, now, accel_x, gyro_z):
        m = Imu()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.imu_frame
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.z = float(gyro_z)
        m.linear_acceleration.x = float(accel_x)
        m.angular_velocity_covariance = _cov({0: 1e6, 4: 1e6, 8: 0.02})
        m.linear_acceleration_covariance = _cov({0: 0.05, 4: 1e6, 8: 1e6})
        self.imu_pub.publish(m)

    def _update_odom(self, now, left, right, gyro_z):
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
        if dt <= 0.0 or dt > 0.5:
            self.last_left, self.last_right, self.last_time = left, right, now
            return

        dl = left - self.last_left
        dr = right - self.last_right
        if abs(dl) > self.max_jump or abs(dr) > self.max_jump:
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
