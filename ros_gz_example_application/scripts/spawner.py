#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from ros_gz_interfaces.msg import EntityFactory, Entity
from geometry_msgs.msg import Pose

class Spawner(Node):
    def __init__(self):
        super().__init__('box_spawner')

        # Create service clients
        self.spawn_cli  = self.create_client(SpawnEntity, '/world/demo/create')
        self.delete_cli = self.create_client(DeleteEntity, '/world/demo/remove')

        self.get_logger().info('Waiting for /world/demo/create and /world/demo/remove services...')
        if not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('SpawnEntity service not available, exiting')
            rclpy.shutdown()
            return
        if not self.delete_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('DeleteEntity service not available, exiting')
            rclpy.shutdown()
            return

        # Spawn request
        self.entity_name = 'test_box'
        spawn_req = SpawnEntity.Request()
        spawn_req.entity_factory = EntityFactory(
            name=self.entity_name,
            allow_renaming=False,
            sdf_filename='/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_description/models/forest_1_ball/forest_1_ball_state_0/model.sdf',
            pose=Pose(),
            relative_to='world'
        )

        # Call spawn service
        self.get_logger().info(f'Spawning box "{self.entity_name}"...')
        self.spawn_cli.call_async(spawn_req).add_done_callback(self.on_spawn_response)

    def on_spawn_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception on spawn: {e}')
            rclpy.shutdown()
            return

        if resp.success:
            self.get_logger().info(f'✅ Spawn succeeded: {self.entity_name}')
            # schedule deletion
            self.delete_timer = self.create_timer(2.0, self.delete_box)
        else:
            self.get_logger().error(f'❌ Spawn failed: {self.entity_name}')
            rclpy.shutdown()

    def delete_box(self):
        # one-shot
        self.destroy_timer(self.delete_timer)
        self.get_logger().info(f'Removing box "{self.entity_name}" after 2 seconds...')

        # DeleteEntity expects an Entity message
        delete_req = DeleteEntity.Request()
        delete_req.entity = Entity(
            id=0,
            name=self.entity_name,
            type=Entity.MODEL
        )

        self.delete_cli.call_async(delete_req).add_done_callback(self.on_delete_response)

    def on_delete_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception on delete: {e}')
            rclpy.shutdown()
            return

        if resp.success:
            self.get_logger().info(f'✅ Delete succeeded: {self.entity_name}')
        else:
            self.get_logger().error(f'❌ Delete failed: {self.entity_name}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Spawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
