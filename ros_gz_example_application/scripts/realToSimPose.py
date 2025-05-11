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

        # 1) Service client for Gazebo‐Sim set_pose
        self.srv_name = '/world/demo/set_pose'
        self.cli = self.create_client(SetEntityPose, self.srv_name)
        self.get_logger().info(f'Waiting for service {self.srv_name}…')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available, shutting down.')
            rclpy.shutdown()
            return

        # 2) Subscribe to your real‐robot odom
        self.create_subscription(
            Odometry,
            'robot1/sim_localization',
            self.odom_callback,
            10)

    def odom_callback(self, msg: Odometry):
        # Extract pose from Odometry
        sim_pose = Pose()
        sim_pose.position = msg.pose.pose.position
        sim_pose.orientation = msg.pose.pose.orientation

        # Build service request
        req = SetEntityPose.Request()
        req.entity = Entity(name='diff_drive', type=Entity.MODEL)
        req.pose = sim_pose

        # Call the service (fire‐and‐forget)
        self.cli.call_async(req)
        self.get_logger().info(
            f'Bridged real → sim: x={sim_pose.position.x:.2f}, '
            f'y={sim_pose.position.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = RealToSimPose()
    if rclpy.ok():
        rclpy.spin(node)
    # Clean shutdown
    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
