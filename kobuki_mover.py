#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class KobukiMover(Node):
    def __init__(self):
        super().__init__('kobuki_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription_people = self.create_subscription(
            String,  # Simulated people detection message
            '/people_positions',
            self.people_callback,
            10)

        self.twist = Twist()
        self.front_clear = True
        self.people_in_front = False

    def lidar_callback(self, msg):
        """ Process LiDAR data to check for obstacles in the front sector. """
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        front_min_distance = 0.5  # Stop if obstacle <0.5m in front
        front_sector = [i for i in range(len(ranges)) if abs(math.degrees(angle_min + i * angle_increment)) < 30]

        self.front_clear = all(ranges[i] > front_min_distance for i in front_sector)

    def people_callback(self, msg):
        """ Process people detection and update movement logic. """
        self.get_logger().info(f"People positions: {msg.data}")
        if "front" in msg.data:  # Simplified people detection condition
            self.people_in_front = True
        else:
            self.people_in_front = False

    def control_loop(self):
        """ Decide Kobuki motion based on LiDAR and people detection data. """
        if not self.front_clear or self.people_in_front:
            self.get_logger().info("Obstacle or person detected: Stopping")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5  # Rotate to avoid obstacle
        else:
            self.get_logger().info("Path clear: Moving forward")
            self.twist.linear.x = 0.2  # Move forward
            self.twist.angular.z = 0.0

        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = KobukiMover()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
