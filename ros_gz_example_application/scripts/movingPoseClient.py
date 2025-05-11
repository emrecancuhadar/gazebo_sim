#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose

class MovingPoseClient(Node):
    def __init__(self):
        super().__init__('moving_pose_client')

        self.cli = self.create_client(SetEntityPose, '/world/demo/set_pose')
        self.get_logger().info('Waiting for service /world/demo/set_pose…')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available, shutting down.')
            rclpy.shutdown()
            return

        self.steps = 1000
        self.delta = 10.0 / (self.steps - 1)
        self.current = 0

        self.timer = self.create_timer(0.1, self._on_timer)

    def _on_timer(self):
        if self.current >= self.steps:
            self.get_logger().info('✅ Completed sweep, shutting down.')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()               # ← shutdown exactly once here
            return

        x = self.current * self.delta
        req = SetEntityPose.Request()
        req.entity = Entity(name='diff_drive', type=Entity.MODEL)
        req.pose = Pose()
        req.pose.position.x = x
        req.pose.position.y = 0.0
        req.pose.position.z = 0.01
        req.pose.orientation.w = 1.0

        self.get_logger().info(f'→ Calling set_pose: x = {x:.3f}')
        self.cli.call_async(req).add_done_callback(self._on_response)
        self.current += 1

    def _on_response(self, future):
        resp = future.result()
        if resp.success:
            self.get_logger().info('   ↳ OK')
        else:
            self.get_logger().error('   ↳ ✗ failed')

def main(args=None):
    rclpy.init(args=args)
    node = MovingPoseClient()
    rclpy.spin(node)                   # will return once shutdown() is called
    # No second shutdown() here!

if __name__ == '__main__':
    main()
