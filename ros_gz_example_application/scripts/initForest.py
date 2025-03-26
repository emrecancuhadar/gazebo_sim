#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import json
import time
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

class InitForest(Node):
    def __init__(self):
        super().__init__('init_forest')

        # Grid + plane info (20×20 m with 2×2 m cells)
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
        self.offset = -10.0

        # Path to greenIntensity.py (adjust if needed)
        self.intensity_script = "/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_application/scripts/greenIntensity.py"

        # Get the share directory for the description package (where the models are installed)
        share_dir = get_package_share_directory('ros_gz_example_description')
        # Build paths to the healthy (state 0) forest model SDF files:
        self.sdf_forest_1 = os.path.join(share_dir, "models", "forest_1_ball/forest_1_ball_state_0", "model.sdf")
        self.sdf_forest_2 = os.path.join(share_dir, "models", "forest_2_ball/forest_2_ball_state_0", "model.sdf")
        self.sdf_forest_3 = os.path.join(share_dir, "models", "forest_3_ball/forest_3_ball_state_0", "model.sdf")
        self.sdf_forest_4 = os.path.join(share_dir, "models", "forest_4_ball/forest_4_ball_state_0", "model.sdf")
        self.sdf_forest_5 = os.path.join(share_dir, "models", "forest_5_ball/forest_5_ball_state_0", "model.sdf")

        # Dictionary to hold all forest cell info.
        self.forest_info = {}

        # Publisher to broadcast forest information as a JSON array.
        self.forest_info_pub = self.create_publisher(String, 'forest_info', 10)

        self.get_logger().info("Starting forest initialization node...")

        # Call greenIntensity.py to get the 10×10 intensity values.
        intensities = self.call_green_intensity_script()
        if intensities is None:
            self.get_logger().error("No intensities received; aborting.")
            return

        # Spawn models and populate forest_info.
        for i in range(self.grid_rows):
            for j in range(self.grid_cols):
                intensity = intensities[i][j]
                num_balls = self.get_num_balls(intensity)
                if num_balls == 0:
                    continue

                if num_balls == 1:
                    sdf_path = self.sdf_forest_1
                elif num_balls == 2:
                    sdf_path = self.sdf_forest_2
                elif num_balls == 3:
                    sdf_path = self.sdf_forest_3
                elif num_balls == 4:
                    sdf_path = self.sdf_forest_4
                else:
                    sdf_path = self.sdf_forest_5

                self.spawn_one_model(i, j, intensity, num_balls, sdf_path)

        # Publish forest info only once, after initialization.
        self.publish_forest_info()

        self.get_logger().info("Forest initialization complete.")
        self.get_logger().info(f"Spawned forest info: {self.forest_info}")

    def call_green_intensity_script(self):
        try:
            result = subprocess.run(
                ["python3", self.intensity_script],
                capture_output=True, text=True, check=True, timeout=30
            )
            self.get_logger().info(f"greenIntensity.py output: {result.stdout}")
            intensities = json.loads(result.stdout)
            return intensities
        except Exception as e:
            self.get_logger().error(f"Error running {self.intensity_script}: {str(e)}")
            return None

    def get_num_balls(self, intensity):
        if intensity > 1.7:
            return 5
        elif intensity > 1.4:
            return 4
        elif intensity > 1.1:
            return 3
        elif intensity > 0.8:
            return 2
        elif intensity > 0.5:
            return 1
        else:
            return 0

    def spawn_one_model(self, row, col, intensity, num_balls, sdf_path):
        cell_origin_x = self.offset + col * self.cell_size
        cell_origin_y = self.offset + (self.grid_rows - 1 - row) * self.cell_size
        x, y, z = cell_origin_x, cell_origin_y, 0.01
        model_name = f"forest_cell{row}_{col}_{int(time.time()*1000)}"
        self.get_logger().info(
            f"Spawning {model_name} from {os.path.basename(sdf_path)} at cell ({row},{col}), intensity {intensity:.2f}"
        )

        spawn_command = [
            "ros2", "run", "ros_gz_sim", "create",
            "-demo", "default",
            "-file", sdf_path,
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
            "-name", model_name
        ]

        try:
            subprocess.run(spawn_command, check=True)
            self.get_logger().info(f"Spawned {model_name}")
            cell_info = {
                "row": row,
                "col": col,
                "model_name": model_name,
                "num_balls": num_balls,
                "state": 0
            }
            self.forest_info[(row, col)] = cell_info
            time.sleep(0.5)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to spawn {model_name}: {str(e)}")

    def publish_forest_info(self):
        info_array = list(self.forest_info.values())
        msg = String()
        msg.data = json.dumps(info_array)
        self.forest_info_pub.publish(msg)
        self.get_logger().info(f"Published forest info: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = InitForest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
