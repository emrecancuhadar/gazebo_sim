#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import subprocess
import time
import os
from ament_index_python.packages import get_package_share_directory

class HandleFire(Node):
    def __init__(self):
        super().__init__('handle_fire')

        # Grid + plane info (same as initForest)
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
        self.offset = -10.0

        # Get share directory (for constructing SDF paths)
        self.share_dir = get_package_share_directory('ros_gz_example_description')

        # Local copy of forest info (key: (row,col) -> cell info)
        self.forest_info = {}

        # Subscribe to forest_info topic published by initForest
        self.forest_info_sub = self.create_subscription(String, 'forest_info', self.forest_info_callback, 10)

        # Publisher to update forest info (if needed)
        self.forest_info_pub = self.create_publisher(String, 'forest_info', 10)

        # For demonstration, update a specific cell (e.g. (0, 0)) periodically.
        # You can change target_cell or extend to multiple cells.
        self.target_cell = (3, 3)
        self.create_timer(5.0, self.update_target_cell)

    def forest_info_callback(self, msg):
        try:
            info_array = json.loads(msg.data)
            # Update local dictionary (keyed by (row, col))
            for cell in info_array:
                key = (cell["row"], cell["col"])
                self.forest_info[key] = cell
            self.get_logger().info(f"Updated local forest info with {len(self.forest_info)} cells")
        except Exception as e:
            self.get_logger().error(f"Error parsing forest_info: {e}")

    def update_target_cell(self):
        key = self.target_cell
        if key not in self.forest_info:
            self.get_logger().info(f"Target cell {key} not in forest info yet.")
            return

        cell_info = self.forest_info[key]
        current_state = cell_info.get("state", 0)
        num_balls = cell_info.get("num_balls", 0)
        if num_balls == 0:
            self.get_logger().info(f"Target cell {key} has no forest model.")
            return

        max_state = num_balls * 7 - 1  # For example, 5 balls => max state 34
        if current_state >= max_state:
            self.get_logger().info(f"Cell {key} is fully burnt (state {current_state}).")
            return

        new_state = current_state + 1
        self.get_logger().info(f"Updating cell {key}: state {current_state} -> {new_state}")

        # Delete the current model
        model_name = cell_info.get("model_name")
        if not model_name:
            self.get_logger().error("No model name found in cell info!")
            return
        
        delete_command = [
            "ros2", "run", "ros_gz_sim", "remove",
            "--ros-args", "-p", f"entity_name:={model_name}"
        ]


        try:
            subprocess.run(delete_command, check=True)
            self.get_logger().info(f"Deleted model {model_name}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to delete model {model_name}: {e}")
            return

        # Construct new SDF path
        # Assumes SDF path structure: models/forest_<num_balls>_ball/forest_<num_balls>_ball_state_<new_state>/model.sdf
        new_sdf = os.path.join(
            self.share_dir,
            "models",
            f"forest_{num_balls}_ball",
            f"forest_{num_balls}_ball_state_{new_state}",
            "model.sdf"
        )
        # Compute spawn location (same as original cell)
        cell_origin_x = self.offset + key[1] * self.cell_size
        cell_origin_y = self.offset + (self.grid_rows - 1 - key[0]) * self.cell_size
        x, y, z = cell_origin_x, cell_origin_y, 0.01

        new_model_name = f"forest_cell{key[0]}_{key[1]}_{int(time.time()*1000)}"
        spawn_command = [
            "ros2", "run", "ros_gz_sim", "create",
            "-demo", "default",
            "-file", new_sdf,
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
            "-name", new_model_name
        ]
        try:
            subprocess.run(spawn_command, check=True)
            self.get_logger().info(f"Spawned updated model {new_model_name} with state {new_state}")
            # Update cell info locally
            cell_info["model_name"] = new_model_name
            cell_info["state"] = new_state
            self.forest_info[key] = cell_info
            self.publish_forest_info()
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to spawn updated model: {e}")

    def publish_forest_info(self):
        """Publish the entire forest_info dictionary as a JSON array."""
        info_array = list(self.forest_info.values())
        msg = String()
        msg.data = json.dumps(info_array)
        self.forest_info_pub.publish(msg)
        self.get_logger().info("Published updated forest info.")

def main(args=None):
    rclpy.init(args=args)
    node = HandleFire()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
