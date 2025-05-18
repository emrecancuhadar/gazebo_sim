#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time

class MultiPathPublisher(Node):
    def __init__(self):
        super().__init__('multi_path_publisher')

        # TF buffer & listener
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Which robots to track?
        self.robot_list = self.declare_parameter(
            'robot_list',
            ['diff_drive', 'diff_drive2']
        ).value

        # Publishers, Path msgs & last positions
        self.path_pubs         = {}    # <<-- renamed from publishers
        self.paths             = {}
        self.last_pos          = {}
        self.distance_threshold = self.declare_parameter(
            'distance_threshold', 0.05
        ).value

        for ns in self.robot_list:
            topic = f'/{ns}_path'
            pub = self.create_publisher(Path, topic, 10)

            path = Path()
            path.header.frame_id = f'{ns}/odom'

            self.path_pubs[ns] = pub      # <<-- use path_pubs
            self.paths[ns]     = path
            self.last_pos[ns]  = (None, None)

            self.get_logger().info(f'Publishing path for "{ns}" on "{topic}"')

        # Timer to update both paths
        self.create_timer(0.1, self.update_paths)

    def update_paths(self):
        now_msg = self.get_clock().now().to_msg()

        for ns in self.robot_list:
            odom_frame = f'{ns}/odom'
            base_frame = f'{ns}/chassis'

            try:
                trans = self.tf_buffer.lookup_transform(
                    odom_frame, base_frame, Time()
                )

                pose = PoseStamped()
                pose.header.stamp    = now_msg
                pose.header.frame_id = odom_frame
                pose.pose.position.x = trans.transform.translation.x
                pose.pose.position.y = trans.transform.translation.y
                pose.pose.position.z = 0.0
                pose.pose.orientation = trans.transform.rotation

                # only record if moved enough
                last_x, last_y = self.last_pos[ns]
                if last_x is not None:
                    dx = pose.pose.position.x - last_x
                    dy = pose.pose.position.y - last_y
                    if (dx*dx + dy*dy) < self.distance_threshold**2:
                        continue

                # append & publish
                self.last_pos[ns] = (pose.pose.position.x, pose.pose.position.y)
                path = self.paths[ns]
                path.poses.append(pose)
                path.header.stamp = now_msg
                self.path_pubs[ns].publish(path)   # <<-- use path_pubs

            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().debug(
                    f'Waiting for transform {odom_frame}→{base_frame}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = MultiPathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
