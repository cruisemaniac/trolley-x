import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SafetyBrakingNode(Node):
    def __init__(self):
        super().__init__('safety_braking_node')
        # Subscribe to the RPLiDAR topic
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("3-Zone Safety System Active. Scanning environment...")

    def scan_callback(self, msg):
        valid_ranges = []

        for i, r in enumerate(msg.ranges):
            # Filter out hardware noise and infinity readings
            if 0.05 < r < 12.0:
                # 1. Get the angle of the laser pulse in radians
                angle_rad = msg.angle_min + (i * msg.angle_increment)

                # 2. Convert Polar (radius, angle) to Cartesian (X, Y)
                x = r * math.cos(angle_rad)
                y = r * math.sin(angle_rad)

                # 3. THE CART BOUNDING BOX (In Meters)
                # X is Forward/Backward, Y is Left/Right from the LiDAR's center
                cart_x_min = -0.10  # e.g., Pillars are 10cm behind the sensor
                cart_x_max =  0.35  # e.g., Pillars are 35cm in front of the sensor
                cart_y_min = -0.20  # e.g., Pillars are 20cm to the right
                cart_y_max =  0.20  # e.g., Pillars are 20cm to the left

                # If the laser hits anything inside this physical rectangle, ignore it
                if (cart_x_min < x < cart_x_max) and (cart_y_min < y < cart_y_max):
                    continue

                valid_ranges.append(r)

        if not valid_ranges:
            return

        # Find the closest object outside the cart's physical footprint
        min_dist = min(valid_ranges)

        # Evaluate the 3 Static Zones for collision avoidance[cite: 8]
        if min_dist <= 0.5:
            self.get_logger().error(f"STOP ZONE [0.5m]: Obstacle at {min_dist:.2f}m! Brakes Applied.")
        elif min_dist <= 1.0:
            self.get_logger().warn(f"SLOW ZONE [1.0m]: Obstacle at {min_dist:.2f}m. Reducing velocity.")
        elif min_dist <= 2.0:
            self.get_logger().info(f"WARN ZONE [2.0m]: Obstacle at {min_dist:.2f}m. Monitoring.")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyBrakingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
