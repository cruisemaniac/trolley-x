"""Cardputer UDP teleoperation node."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket


class CardputerTeleop(Node):
    def __init__(self):
        super().__init__('cardputer_teleop')

        self.declare_parameter('output_topic', '/motion_request')
        self.declare_parameter('udp_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)

        out_topic = self.get_parameter('output_topic').value
        self.lin = float(self.get_parameter('linear_speed').value)
        self.ang = float(self.get_parameter('angular_speed').value)

        self.publisher_ = self.create_publisher(Twist, out_topic, 10)

        self.udp_ip = self.get_parameter('udp_ip').value
        self.udp_port = int(self.get_parameter('udp_port').value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            f"Cardputer teleop on UDP {self.udp_port} -> {out_topic}")

    def timer_callback(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            cmd = data.decode('utf-8').strip()
            msg = Twist()
            if cmd == 'W':
                msg.linear.x = self.lin
            elif cmd == 'S':
                msg.linear.x = -self.lin
            elif cmd == 'A':
                msg.angular.z = self.ang
            elif cmd == 'D':
                msg.angular.z = -self.ang
            # anything else (incl. 'X') publishes a zero Twist -> stop
            self.publisher_.publish(msg)
        except BlockingIOError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CardputerTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
