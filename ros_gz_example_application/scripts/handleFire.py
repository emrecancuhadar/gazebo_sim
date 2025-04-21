#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, subprocess, time, os
from ament_index_python.packages import get_package_share_directory

class HandleFire(Node):
    def __init__(self):
        super().__init__('handle_fire')

        # Grid configuration (10x10; must match InitForest)
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
        # Center the grid so that cell centers are centered at (0,0)
        self.offset_x = -((self.grid_cols - 1) * self.cell_size) / 2.0
        self.offset_y = -((self.grid_rows - 1) * self.cell_size) / 2.0

        self.share_dir = get_package_share_directory('ros_gz_example_description')
        # Local forest info dictionary. Keys: (row, col) with values:
        # { "row", "col", "model_name", "max_fire", "state", "cstate", "spawned" }
        self.forest_info = {}

        # Counters for service calls
        self.spawn_count = 0
        self.delete_count = 0
        # Start time for timer
        self.start_time = time.time()

        # Subscribers and publisher
        self.forest_info_sub = self.create_subscription(String, 'forest_info', self.forest_info_callback, 10)
        self.create_subscription(String, 'ignite_cell', self.ignite_callback, 10)
        self.forest_info_pub = self.create_publisher(String, 'forest_info', 10)

        # Wind configuration for spreading fire
        self.wind_dx = 1
        self.wind_dy = -1

        self.increment = 20  # cstate increment every update (every 2 seconds)
        # Timer: update burning cells every 2 seconds.
        self.create_timer(2.0, self.update_burning_cells)
        # Timer: print elapsed time and counters every 5 seconds.
        self.create_timer(5.0, self.print_counters)

    def print_counters(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Elapsed time: {elapsed:.1f} sec | Spawns: {self.spawn_count} | Deletes: {self.delete_count}")

    def forest_info_callback(self, msg):
        try:
            cells = json.loads(msg.data)
            for cell in cells:
                key = (cell["row"], cell["col"])
                # Preserve higher cstate if already burning
                if key in self.forest_info and self.forest_info[key].get("cstate", 0) > cell.get("cstate", 0):
                    continue
                self.forest_info[key] = cell
            self.get_logger().info(f"HandleFire: Updated forest info with {len(self.forest_info)} cells")
        except Exception as e:
            self.get_logger().error(f"HandleFire: Error parsing forest_info: {e}")

    def ignite_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            key = (cmd["row"], cmd["col"])
            self.get_logger().info(f"HandleFire: Received ignite command for cell {key}")
            # If the cell is missing or healthy (cstate == 0), spawn a new cluster.
            if key not in self.forest_info or self.forest_info[key].get("cstate", 0) == 0:
                self.spawn_cluster(key, initial_state=1)
            else:
                self.get_logger().info(f"HandleFire: Cell {key} is already ignited.")
        except Exception as e:
            self.get_logger().error(f"HandleFire: Error in ignite_callback: {e}")

    def update_burning_cells(self):
        # Iterate over a snapshot of keys to avoid runtime errors
        for key in list(self.forest_info.keys()):
            cell = self.forest_info.get(key)
            if not cell:
                continue
            cstate = cell.get("cstate", 0)
            # Update only burning cells (cstate > 0 and < 1000)
            if cstate > 0 and cstate < 1000:
                new_cstate = min(cstate + self.increment, 1000)
                cell["cstate"] = new_cstate
                self.get_logger().info(f"HandleFire: Cell {key} cstate updated to {new_cstate}")
                self.forest_info[key] = cell
                # Force visual state to 5 when cstate >= 801 if not already.
                if new_cstate >= 801 and cell.get("state", 0) < 5:
                    self.get_logger().info(f"HandleFire: Forcing cell {key} state to 5 (cstate={new_cstate})")
                    self.update_cell(key, force_state=5)
                # Spread fire if cstate >= 800
                if new_cstate >= 800:
                    self.spread_fire(key)
        self.publish_forest_info()

    def update_cell(self, key, force_state=None):
        if key not in self.forest_info:
            self.get_logger().info(f"HandleFire: Cell {key} not found.")
            return
        cell = self.forest_info[key]
        current_state = cell.get("state", 0)
        max_fire = cell.get("max_fire", 1)  # max_fire indicates the forest type.
        # Use forced state if provided and greater than current.
        new_state = force_state if (force_state is not None and force_state > current_state) else (current_state + 1)

        # For burning cells (states 1 to 5) assume maximum continuous state is 5*200.
        max_cstate = 5 * 200
        if cell.get("cstate", 0) >= max_cstate:
            self.get_logger().info(f"HandleFire: Cell {key} is fully burnt (cstate={cell.get('cstate')}).")
            return

        self.get_logger().info(f"HandleFire: Updating cell {key}: state {current_state} -> {new_state}")
        # Delete the current model.
        model_name = cell.get("model_name")
        if model_name:
            delete_cmd = [
                "ros2", "run", "ros_gz_sim", "remove",
                "--ros-args", "-p", f"entity_name:={model_name}"
            ]
            try:
                subprocess.run(delete_cmd, check=True)
                self.delete_count += 1
                self.get_logger().info(f"HandleFire: Deleted model {model_name}")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"HandleFire: Failed to delete model {model_name}: {e}")
                return

        new_sdf = os.path.join(
            self.share_dir,
            "models",
            f"forest_{max_fire}_ball",
            f"forest_{max_fire}_ball_state_{new_state}",
            "model.sdf"
        )
        x = self.offset_x + key[1] * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
        z = 0.01
        new_model_name = f"forest_cell{key[0]}_{key[1]}_{int(time.time()*1000)}"
        spawn_cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-demo", "default",
            "-file", new_sdf,
            "-x", str(x), "-y", str(y), "-z", str(z),
            "-name", new_model_name
        ]
        try:
            subprocess.run(spawn_cmd, check=True)
            self.spawn_count += 1
            self.get_logger().info(f"HandleFire: Spawned updated model {new_model_name} with state {new_state}")
            cell["model_name"] = new_model_name
            cell["state"] = new_state
            self.forest_info[key] = cell
            self.publish_forest_info()
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"HandleFire: Failed to spawn updated model: {e}")

    def spread_fire(self, source_key):
        row, col = source_key
        target_row = row + self.wind_dy
        target_col = col + self.wind_dx
        if 0 <= target_row < self.grid_rows and 0 <= target_col < self.grid_cols:
            target_key = (target_row, target_col)
            target_cell = self.forest_info.get(target_key)
            if target_cell is None or target_cell.get("cstate", 0) == 0:
                self.get_logger().info(f"HandleFire: Spreading fire from {source_key} to {target_key}")
                if target_cell and target_cell.get("spawned", False) and target_cell.get("model_name"):
                    self.delete_model(target_cell["model_name"])
                    self.delete_count += 1
                    target_cell["spawned"] = False
                    target_cell["model_name"] = None
                    self.forest_info[target_key] = target_cell
                self.spawn_cluster(target_key, initial_state=1)
        else:
            self.get_logger().info(f"HandleFire: Target cell ({target_row},{target_col}) out of bounds for {source_key}")

    def spawn_cluster(self, center_key, initial_state=1):
        c_row, c_col = center_key
        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                cell_key = (c_row + di, c_col + dj)
                # Bounds check
                if 0 <= cell_key[0] < self.grid_rows and 0 <= cell_key[1] < self.grid_cols:
                    # If it’s the exact center cell (the newly ignited one), spawn with initial_state.
                    if cell_key == center_key:
                        self.spawn_cell(cell_key, initial_state)
                    else:
                        # For surrounding cells, spawn only if cstate=0 or not spawned at all
                        info = self.forest_info.get(cell_key, {})
                        if info.get("cstate", 0) == 0:
                            self.spawn_cell(cell_key, initial_state=0)
                        # else do nothing, so as not to overwrite an already-burning cell


    def spawn_cell(self, key, initial_state=0):
        row, col = key
        cell = self.forest_info.get(key, {"row": row, "col": col, "max_fire": 1})
        max_fire = cell.get("max_fire", 1)
        state = initial_state
        cstate = 0 if state == 0 else (state - 1)*200 + 1
        sdf_path = os.path.join(
            self.share_dir,
            "models",
            f"forest_{max_fire}_ball",
            f"forest_{max_fire}_ball_state_{state}",
            "model.sdf"
        )
        x = self.offset_x + col * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - row) * self.cell_size
        z = 0.01
        model_name = f"forest_cell{row}_{col}_{int(time.time()*1000)}"
        spawn_cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-demo", "default",
            "-file", sdf_path,
            "-x", str(x), "-y", str(y), "-z", str(z),
            "-name", model_name
        ]
        try:
            subprocess.run(spawn_cmd, check=True)
            self.spawn_count += 1
            self.get_logger().info(f"HandleFire: Spawned cell {model_name} at {key} with state {state}")
            cell["state"] = state
            cell["cstate"] = cstate
            cell["model_name"] = model_name
            cell["spawned"] = True
            self.forest_info[key] = cell
            time.sleep(0.3)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"HandleFire: Failed to spawn cell {model_name} at {key}: {e}")

    def delete_model(self, model_name):
        cmd = [
            "ros2", "run", "ros_gz_sim", "remove",
            "--ros-args", "-p", f"entity_name:={model_name}"
        ]
        try:
            subprocess.run(cmd, check=True)
            self.get_logger().info(f"HandleFire: Deleted model {model_name}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"HandleFire: Failed to delete model {model_name}: {e}")

    def publish_forest_info(self):
        msg = String()
        msg.data = json.dumps(list(self.forest_info.values()))
        self.forest_info_pub.publish(msg)
        self.get_logger().info("HandleFire: Published updated forest info.")

def main(args=None):
    rclpy.init(args=args)
    node = HandleFire()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
