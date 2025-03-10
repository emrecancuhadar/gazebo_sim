#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomFilter(Node):
    def __init__(self):
        super().__init__('odometry_filter')
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive/odometry',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(
            Odometry,
            '/filtered_odom',
            10)

    def odom_callback(self, msg):
        self.get_logger().info(f"Publishing odometry from child_frame_id: {msg.child_frame_id}")
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = OdomFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
