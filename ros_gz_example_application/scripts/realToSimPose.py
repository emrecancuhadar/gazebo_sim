#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose

class RealToSimPose(Node):
    def __init__(self):
        super().__init__('real_to_sim_pose')

        # 1) Gazebo set_pose service client
        self.srv_name = '/world/demo/set_pose'
        self.cli = self.create_client(SetEntityPose, self.srv_name)
        self.get_logger().info(f'Waiting for service {self.srv_name}…')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available, shutting down.')
            rclpy.shutdown()
            return

        # 2) Which real robots to listen to?
        self.robot_list = self.declare_parameter(
            'robot_list',
            ['robot1', 'robot2']
        ).value

        # 3) Subscribe to each namespace's simlocalization topic
        for ns in self.robot_list:
            topic = f'{ns}/simlocalization'
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, ns=ns: self.odom_callback(msg, ns),
                10
            )
            self.get_logger().info(f'Subscribed to {topic}')

    def odom_callback(self, msg: Odometry, ns: str):
        # Extract pose
        sim_pose = Pose()
        sim_pose.position = msg.pose.pose.position
        sim_pose.orientation = msg.pose.pose.orientation

        # Build and send the service request
        req = SetEntityPose.Request()
        # Model names in your SDF: "diff_drive" and "diff_drive2"
        req.entity = Entity(name=ns, type=Entity.MODEL)
        req.pose = sim_pose
        self.cli.call_async(req)

        self.get_logger().info(
            f'[{ns}] Bridged real → sim: '
            f'x={sim_pose.position.x:.2f}, '
            f'y={sim_pose.position.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RealToSimPose()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
