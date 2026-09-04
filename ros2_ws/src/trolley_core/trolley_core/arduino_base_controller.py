"""Arduino serial bridge with odometry and IMU output."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading


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

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.max_lin = float(self.get_parameter('max_linear_mps').value)
        self.max_ang = float(self.get_parameter('max_angular_rps').value)
        self.max_pwm = int(self.get_parameter('max_pwm').value)
        self.min_move = int(self.get_parameter('min_move_pwm').value)
        self.min_turn = int(self.get_parameter('min_turn_pwm').value)

        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")

        self.subscription = self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self.cmd_vel_callback, 10)

        self.read_thread = threading.Thread(target=self.read_telemetry, daemon=True)
        self.read_thread.start()

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

    def read_telemetry(self):
        while rclpy.ok():
            if hasattr(self, 'arduino') and self.arduino.is_open:
                try:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().debug(f"telemetry: [{line}]")
                except Exception:
                    pass


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
