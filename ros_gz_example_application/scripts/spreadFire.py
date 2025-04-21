#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose

class SpreadFire(Node):
    def __init__(self):
        super().__init__('spread_fire')

    # persistent clients for the Gazebo‑Ignition bridge
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.get_logger().info('Waiting for /spawn_entity & /delete_entity…')
        self.spawn_cli.wait_for_service()
        self.delete_cli.wait_for_service()
        self.get_logger().info('…services are available.')

        # counters & state
        self.spawn_count = 0
        self.delete_count = 0
        self.next_spawn = True
        self.model_base = 'test_box'
        self.start_time = self.get_clock().now()

        # timers
        self.create_timer(1.0, self.cycle)         # every 1 s spawn/delete
        self.create_timer(5.0, self.print_stats)  # every 5 s log stats

    def cycle(self):
        if self.next_spawn:
            self.spawn_model()
        else:
            self.delete_model()
        self.next_spawn = not self.next_spawn

    def spawn_model(self):
        name = f'{self.model_base}_{self.spawn_count}'
        req = SpawnEntity.Request()
        req.name = name

        # minimal 1×1×1 box SDF
        req.xml = f"""<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <pose>0 0 0.5 0 0 0</pose>
      <collision name='col'><geometry><box><size>1 1 1</size></box></geometry></collision>
      <visual name='vis'><geometry><box><size>1 1 1</size></box></geometry></visual>
    </link>
  </model>
</sdf>"""

        # drop in at z=0.5
        p = Pose()
        p.position.z = 0.5
        req.initial_pose = p

        fut = self.spawn_cli.call_async(req)
        fut.add_done_callback(lambda f: self._on_spawn(f, name))

    def _on_spawn(self, future, name):
        try:
            _ = future.result()
            self.spawn_count += 1
            self.get_logger().info(f'[SPAWNED] {name}')
        except Exception as e:
            self.get_logger().error(f'[SPAWN ✗] {name}: {e}')

    def delete_model(self):
        if self.spawn_count == 0:
            return
        name = f'{self.model_base}_{self.spawn_count-1}'
        req = DeleteEntity.Request()
        req.name = name

        fut = self.delete_cli.call_async(req)
        fut.add_done_callback(lambda f: self._on_delete(f, name))

    def _on_delete(self, future, name):
        try:
            _ = future.result()
            self.delete_count += 1
            self.get_logger().info(f'[DELETED] {name}')
        except Exception as e:
            self.get_logger().error(f'[DELETE ✗] {name}: {e}')

    def print_stats(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.get_logger().info(
            f'Elapsed: {elapsed:.1f}s | Spawns: {self.spawn_count} | Deletes: {self.delete_count}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SpreadFire()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()