#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, subprocess, time, os
from subprocess import CalledProcessError, TimeoutExpired
from ament_index_python.packages import get_package_share_directory

class HandleFire(Node):
    def __init__(self):
        super().__init__('handle_fire')

        # Grid setup
        self.grid_rows = 10; self.grid_cols = 10; self.cell_size = 2.0
        self.offset_x = -((self.grid_cols - 1) * self.cell_size) / 2.0
        self.offset_y = -((self.grid_rows - 1) * self.cell_size) / 2.0

        self.share_dir = get_package_share_directory('ros_gz_example_description')
        self.forest_info = {}

        # Counters
        self.spawn_count = 0
        self.delete_count = 0
        self.start_time = time.time()

        # Pub/Sub
        self.create_subscription(String, 'forest_info', self.forest_info_callback, 10)
        self.create_subscription(String, 'ignite_cell',  self.ignite_callback,     10)
        self.forest_info_pub = self.create_publisher(String, 'forest_info', 10)

        # Fire‐spread parameters
        self.wind_dx = 1; self.wind_dy = 0
        self.increment = 20

        # Timers
        self.create_timer(2.0, self.update_burning_cells)
        self.create_timer(5.0, self.print_counters)

    def run_cmd_with_retry(self, cmd, timeout=4.0, retries=3):
        """Run a subprocess command, retrying up to `retries` times on failure."""
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                result = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=True,
                    timeout=timeout,
                    text=True
                )
                return result
            except TimeoutExpired as e:
                last_exc = e
                self.get_logger().warn(f"[retry {attempt}/{retries}] timeout: {e}")
            except CalledProcessError as e:
                last_exc = e
                self.get_logger().warn(f"[retry {attempt}/{retries}] error: {e.stderr.strip()}")
            if attempt < retries:
                time.sleep(0.5)
        # all attempts failed
        raise last_exc

    def print_counters(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Elapsed {elapsed:.1f}s | Spawns: {self.spawn_count} | Deletes: {self.delete_count}")

    def forest_info_callback(self, msg):
        try:
            cells = json.loads(msg.data)
            for cell in cells:
                key = (cell["row"], cell["col"])
                if key in self.forest_info and self.forest_info[key].get("cstate", 0) > cell.get("cstate", 0):
                    continue
                self.forest_info[key] = cell
            self.get_logger().info(f"Forest info updated ({len(self.forest_info)} cells)")
        except Exception as e:
            self.get_logger().error(f"Error parsing forest_info: {e}")

    def ignite_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            key = (cmd["row"], cmd["col"])
            if self.forest_info.get(key, {}).get("cstate", 0) == 0:
                self.get_logger().info(f"Igniting cell {key}")
                self.spawn_cluster(key, initial_state=1)
            else:
                self.get_logger().info(f"Cell {key} already burning")
        except Exception as e:
            self.get_logger().error(f"Error in ignite_callback: {e}")

    def update_burning_cells(self):
        for key in list(self.forest_info):
            cell = self.forest_info[key]
            c = cell.get("cstate", 0)
            if 0 < c < 1000:
                new_c = min(c + self.increment, 1000)
                cell["cstate"] = new_c
                self.get_logger().info(f"Cell {key} cstate -> {new_c}")

                if new_c >= 801 and cell.get("state", 0) < 5:
                    self.update_cell(key, force_state=5)
                if new_c >= 800:
                    self.spread_fire(key)

        self.publish_forest_info()

    def update_cell(self, key, force_state=None):
        cell = self.forest_info[key]
        cur = cell.get("state", 0)
        new_state = force_state if (force_state and force_state > cur) else (cur + 1)
        if cell["cstate"] >= 5 * 200:
            self.get_logger().info(f"Cell {key} fully burnt")
            return

        # DELETE old model
        old_name = cell.get("model_name")
        if old_name:
            cmd = [
                "ros2", "run", "ros_gz_sim", "remove",
                "--ros-args", "-p", f"entity_name:={old_name}"
            ]
            try:
                self.run_cmd_with_retry(cmd)
                self.delete_count += 1
                self.get_logger().info(f"Deleted model {old_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to delete {old_name} after retries: {e}")
                return

        # SPAWN new model
        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{cell.get('max_fire',1)}_ball",
            f"forest_{cell.get('max_fire',1)}_ball_state_{new_state}",
            "model.sdf"
        )
        x = self.offset_x + key[1] * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
        z = 0.01
        new_name = f"forest_cell{key[0]}_{key[1]}_{int(time.time()*1000)}"

        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-demo", "default", "-file", sdf,
            "-x", str(x), "-y", str(y), "-z", str(z),
            "-name", new_name
        ]
        try:
            self.run_cmd_with_retry(cmd)
            self.spawn_count += 1
            cell.update(model_name=new_name, state=new_state)
            self.forest_info[key] = cell
            self.get_logger().info(f"Spawned {new_name} with state {new_state}")
            self.publish_forest_info()
        except Exception as e:
            self.get_logger().error(f"Failed to spawn {new_name} after retries: {e}")

    def spread_fire(self, source_key):
        r, c = source_key
        tgt = (r + self.wind_dy, c + self.wind_dx)
        if 0 <= tgt[0] < self.grid_rows and 0 <= tgt[1] < self.grid_cols:
            info = self.forest_info.get(tgt, {})
            if info.get("cstate", 0) == 0:
                self.get_logger().info(f"Spreading fire from {source_key} to {tgt}")
                old = info.get("model_name")
                if info.get("spawned") and old:
                    self.delete_model(old)
                    self.delete_count += 1
                    info.update(spawned=False, model_name=None)
                    self.forest_info[tgt] = info
                self.spawn_cluster(tgt, initial_state=1)

    def spawn_cluster(self, center_key, initial_state=0):
        cr, cc = center_key
        for di in (-1,0,1):
            for dj in (-1,0,1):
                key = (cr+di, cc+dj)
                if 0 <= key[0] < self.grid_rows and 0 <= key[1] < self.grid_cols:
                    info = self.forest_info.get(key, {"row":key[0],"col":key[1],"max_fire":1})
                    if key == center_key or info.get("cstate",0) == 0:
                        self.spawn_cell(key, initial_state if key==center_key else 0)

    def spawn_cell(self, key, state=0):
        info = self.forest_info.get(key, {"row":key[0],"col":key[1],"max_fire":1})
        maxf = info["max_fire"]
        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{maxf}_ball",
            f"forest_{maxf}_ball_state_{state}",
            "model.sdf"
        )
        x = self.offset_x + key[1] * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
        z = 0.01
        name = f"forest_cell{key[0]}_{key[1]}_{int(time.time()*1000)}"

        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-demo", "default", "-file", sdf,
            "-x", str(x), "-y", str(y), "-z", str(z),
            "-name", name
        ]
        try:
            self.run_cmd_with_retry(cmd)
            self.spawn_count += 1
            info.update(
                state=state,
                cstate=(state-1)*200+1 if state>0 else 0,
                model_name=name,
                spawned=True
            )
            self.forest_info[key] = info
            self.get_logger().info(f"Spawned cell {name} at {key} state {state}")
            time.sleep(0.3)
        except Exception as e:
            self.get_logger().error(f"Spawn cell {name} failed after retries: {e}")

    def delete_model(self, model_name):
        cmd = [
            "ros2", "run", "ros_gz_sim", "remove",
            "--ros-args", "-p", f"entity_name:={model_name}"
        ]
        try:
            self.run_cmd_with_retry(cmd)
            self.get_logger().info(f"Deleted model {model_name}")
        except Exception as e:
            self.get_logger().error(f"Delete model {model_name} failed after retries: {e}")

    def publish_forest_info(self):
        msg = String()
        msg.data = json.dumps(list(self.forest_info.values()))
        self.forest_info_pub.publish(msg)
        self.get_logger().info("Published updated forest_info")

def main(args=None):
    rclpy.init(args=args)
    node = HandleFire()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
