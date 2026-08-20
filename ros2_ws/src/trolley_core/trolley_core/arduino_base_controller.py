import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading

class ArduinoBaseController(Node):
    def __init__(self):
        super().__init__('arduino_base_controller')
        self.serial_port = '/dev/ttyACM0' # Change to /dev/ttyUSB0 if it fails
        self.baud_rate = 9600
        
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.read_thread = threading.Thread(target=self.read_telemetry)
        self.read_thread.daemon = True
        self.read_thread.start()

    def cmd_vel_callback(self, msg):
        command = 'X'
        if msg.linear.x > 0: command = 'W'
        elif msg.linear.x < 0: command = 'S'
        elif msg.angular.z > 0: command = 'A'
        elif msg.angular.z < 0: command = 'D'

        if hasattr(self, 'arduino') and self.arduino.is_open:
            self.arduino.write(command.encode('utf-8'))

    def read_telemetry(self):
        while rclpy.ok():
            if hasattr(self, 'arduino') and self.arduino.is_open:
                try:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line: self.get_logger().info(f"Hardware Telemetry: [{line}]")
                except Exception:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBaseController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if hasattr(node, 'arduino') and node.arduino.is_open:
            node.arduino.write('X'.encode('utf-8')) 
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
