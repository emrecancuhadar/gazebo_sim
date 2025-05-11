#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose

class PoseClient(Node):
    def __init__(self):
        super().__init__('set_entity_pose_client')
        self.cli = self.create_client(SetEntityPose, '/world/demo/set_pose')
        self.get_logger().info('Waiting for service…')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available, shutting down.')
            rclpy.shutdown()
            return

        req = SetEntityPose.Request()
        req.entity = Entity(name='diff_drive', type=Entity.MODEL)
        req.pose = Pose()
        req.pose.position.x = 0.41
        req.pose.position.y = 0.0
        req.pose.position.z = 0.01
        req.pose.orientation.w = 1.0

        self.cli.call_async(req).add_done_callback(self._on_response)

    def _on_response(self, future):
        resp = future.result()
        if resp.success:
            self.get_logger().info('✅ Pose set successfully')
        else:
            self.get_logger().error('❌ Failed to set pose')
        self.destroy_node()
        rclpy.shutdown()            # ← shutdown exactly once

def main(args=None):
    rclpy.init(args=args)
    node = PoseClient()
    rclpy.spin(node)               # will return when shutdown() is called
    # NO second rclpy.shutdown() here

if __name__ == '__main__':
    main()
