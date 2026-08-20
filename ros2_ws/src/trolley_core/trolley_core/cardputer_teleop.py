import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class CardputerTeleop(Node):
    def __init__(self):
        super().__init__('cardputer_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.udp_ip = "0.0.0.0"
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False) 
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"Cardputer Teleop Node Active on UDP port {self.udp_port}")

    def timer_callback(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            cmd = data.decode('utf-8').strip()
            msg = Twist()

            if cmd == 'W': msg.linear.x = 0.5
            elif cmd == 'S': msg.linear.x = -0.5
            elif cmd == 'A': msg.angular.z = 1.0
            elif cmd == 'D': msg.angular.z = -1.0
                
            self.publisher_.publish(msg)
        except BlockingIOError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CardputerTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
