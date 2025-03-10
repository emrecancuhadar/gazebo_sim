#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Path publisher
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "diff_drive/odom"  # Ensure consistent frame

        # Timer to update the path
        self.timer = self.create_timer(0.1, self.update_path)

        self.last_x = None
        self.last_y = None
        self.distance_threshold = 0.05  # Minimum movement to record a new point

    def update_path(self):
        try:
            # Get the transform from odom -> base_link (robot frame)
            transform = self.tf_buffer.lookup_transform("diff_drive/odom", "diff_drive/chassis", rclpy.time.Time())

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "diff_drive/odom"

            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = transform.transform.rotation

            # Avoid adding points if movement is too small
            if self.last_x is not None and self.last_y is not None:
                distance = ((pose.pose.position.x - self.last_x) ** 2 + (pose.pose.position.y - self.last_y) ** 2) ** 0.5
                if distance < self.distance_threshold:
                    return  # Ignore very small movements

            # Update last known position
            self.last_x = pose.pose.position.x
            self.last_y = pose.pose.position.y

            # Append pose and publish
            self.path_msg.poses.append(pose)
            self.path_msg.header.stamp = pose.header.stamp
            self.path_publisher.publish(self.path_msg)

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF lookup failed. Waiting for valid transform...")

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
