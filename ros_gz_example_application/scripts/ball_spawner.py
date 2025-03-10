#!/usr/bin/env python3

import rclpy
import sys
import subprocess
import time
import random
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray

BALL_MODEL_PATH = os.path.join(
    get_package_share_directory('ros_gz_example_description'),
    'models', 'ball', 'model.sdf'
)

class BallSpawner(Node):
    def __init__(self, num_balls, ball_positions):
        super().__init__('ball_spawner')

        # Create ROS 2 Publisher for ball positions
        self.ball_position_pub = self.create_publisher(PoseArray, "/ball_positions", 10)

        # Spawn balls and store their positions
        self.spawned_ball_positions = self.spawn_balls(num_balls, ball_positions)

        # Create a timer to continuously publish ball positions
        self.create_timer(1.0, self.publish_ball_positions)

    def spawn_balls(self, num_balls, ball_positions):
        """ Spawns a given number of balls at specific or random positions using `spawn_model.launch.py` """
        spawned_positions = []

        for i in range(num_balls):
            # Use given positions if provided, else generate random ones
            if i < len(ball_positions):
                x, y = ball_positions[i]  # Take only (x, y)
                z = 0.3  # Default z-value
            else:
                x = random.uniform(-5.0, 5.0)
                y = random.uniform(-5.0, 5.0)
                z = 0.3  # Fixed height for spawning

            ball_name = f"ball_{i}"

            self.get_logger().info(f"Spawning ball {i+1}/{num_balls}: {ball_name} at ({x}, {y}, {z})")

            spawn_command = [
                "ros2", "run", "ros_gz_sim", "create",
                "-demo", "default",                      # world name (use -world if required)
                "-file", BALL_MODEL_PATH,                # path to the model file
                "-x", str(x),                           # X position (meters)
                "-y", str(y),                           # Y position (meters)
                "-z", str(z),                           # Z position (meters)
                "-name", ball_name                      # entity name
            ]


            try:
                subprocess.run(spawn_command, check=True)
                self.get_logger().info(f"✅ Successfully spawned {ball_name}")

                # Store the ball's position
                spawned_positions.append((x, y, z))

                # Small delay to prevent spawning issues
                time.sleep(1.0)

            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"❌ Failed to spawn ball: {str(e)}")

        return spawned_positions

    def publish_ball_positions(self):
        """ Continuously publishes the positions of spawned balls """
        pose_array_msg = PoseArray()
        for pos in self.spawned_ball_positions:
            pose_msg = Pose()
            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = pos
            pose_array_msg.poses.append(pose_msg)

        self.ball_position_pub.publish(pose_array_msg)
        self.get_logger().info("📢 Published ball positions")


def main(args=None):
    rclpy.init(args=args)

    # Read arguments from the command line
    if len(sys.argv) < 2:
        print("Usage: ros2 run ros_gz_example_application ball_spawner.py <num_balls> [x1 y1 x2 y2 ...]")
        return

    num_balls = int(sys.argv[1])
    ball_positions = []

    if len(sys.argv) > 2:
        try:
            # Read positions from command-line arguments
            values = list(map(float, sys.argv[2:]))
            if len(values) % 2 != 0:
                print("⚠️ Error: Ball positions must be provided in sets of (x, y)")
                return
            ball_positions = [(values[i], values[i+1]) for i in range(0, len(values), 2)]
        except ValueError:
            print("⚠️ Error: Invalid number format for positions")
            return

    node = BallSpawner(num_balls, ball_positions)
    rclpy.spin(node)  # Keep running until manually stopped
    rclpy.shutdown()


if __name__ == '__main__':
    main()
