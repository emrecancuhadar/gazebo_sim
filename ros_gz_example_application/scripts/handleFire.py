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
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
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
        self.forest_info_pub = self.create_publisher(String, 'forest_info', 10)

        # Fire‐spread parameters
        self.wind_dx = 1
        self.wind_dy = 1   # adjust to your convention (+1 downwards, -1 upwards, etc.)
        self.state_thresholds = [200, 400, 600, 800, 1000]
        self.state_increments = [5, 8, 12, 16, 18]

        # Batching container
        self._pending_spreads = set()

        # Timers
        self.create_timer(2.0, self.update_burning_cells)
        self.create_timer(5.0, self.print_counters)

    def run_cmd_with_retry(self, cmd, timeout=4.0, retries=5):
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                return subprocess.run(
                    cmd, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, check=True,
                    timeout=timeout, text=True
                )
            except (TimeoutExpired, CalledProcessError) as e:
                last_exc = e
                self.get_logger().warn(f"[retry {attempt}/{retries}] {e}")
                time.sleep(0.5)
        raise last_exc

    def print_counters(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Elapsed {elapsed:.1f}s | Spawns: {self.spawn_count} | Deletes: {self.delete_count}")

    def forest_info_callback(self, msg):
        try:
            cells = json.loads(msg.data)
            for cell in cells:
                key = (cell["row"], cell["col"])
                if key in self.forest_info and self.forest_info[key].get("cstate",0) > cell.get("cstate",0):
                    continue
                self.forest_info[key] = cell
            self.get_logger().info(f"Forest info updated ({len(self.forest_info)} cells)")
        except Exception as e:
            self.get_logger().error(f"Error parsing forest_info: {e}")

    def update_burning_cells(self):
        # 1) collect which cells need to spread
        self._pending_spreads.clear()
        for key, cell in list(self.forest_info.items()):
            c = cell.get("cstate", 0)
            if 0 < c < 1000:
                # increment burn
                for idx, th in enumerate(self.state_thresholds):
                    if c < th:
                        inc = self.state_increments[idx]
                        break
                new_c = min(c + inc, 1000)
                cell["cstate"] = new_c
                self.get_logger().info(f"Cell {key} cstate -> {new_c}")

                # update visual to fully burning if needed
                if new_c >= 801 and cell.get("state",0) < 5:
                    self.update_cell(key, force_state=5)

                # schedule a spread if it’s hot
                if new_c >= 800:
                    self._pending_spreads.add(key)

        # 2) do each spread exactly once
        for src in self._pending_spreads:
            self.spread_fire(src)

        # 3) publish
        self.publish_forest_info()

    def update_cell(self, key, force_state=None):
        cell = self.forest_info[key]
        cur = cell.get("state", 0)
        new_state = force_state if (force_state and force_state > cur) else (cur + 1)

        # fully burnt?
        if cell["cstate"] >= 5 * 200:
            self.get_logger().info(f"Cell {key} fully burnt")
            return

        # delete old model
        old = cell.get("model_name")
        if old:
            cmd = ["ros2","run","ros_gz_sim","remove","--ros-args","-p",f"entity_name:={old}"]
            try:
                self.run_cmd_with_retry(cmd)
                self.delete_count += 1
                self.get_logger().info(f"Deleted {old}")
            except Exception as e:
                self.get_logger().error(f"Delete failed: {e}")
                return

        # spawn new visual
        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{cell.get('max_fire',1)}_ball",
            f"forest_{cell.get('max_fire',1)}_ball_state_{new_state}",
            "model.sdf"
        )
        x = self.offset_x + key[1] * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
        name = f"forest_cell{key[0]}_{key[1]}_{int(time.time()*1000)}"
        cmd = ["ros2","run","ros_gz_sim","create","-demo","default",
               "-file",sdf,"-x",str(x),"-y",str(y),"-z","0.01","-name",name]
        try:
            self.run_cmd_with_retry(cmd)
            self.spawn_count += 1
            cell.update(model_name=name, state=new_state)
            self.forest_info[key] = cell
            self.get_logger().info(f"Spawned {name} state {new_state}")
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")

    def spread_fire(self, source_key):
        r, c = source_key
        tgt = (r + self.wind_dy, c + self.wind_dx)

        # bounds check
        if not (0 <= tgt[0] < self.grid_rows and 0 <= tgt[1] < self.grid_cols):
            return

        info = self.forest_info.get(tgt)
        # only if still cold
        if not info or info.get("cstate",0) != 0:
            return

        self.get_logger().info(f"Spreading fire from {source_key} to {tgt}")

        # delete its healthy model
        old = info.get("model_name")
        if old:
            self.delete_model(old)
            self.delete_count += 1
            info.update(spawned=False, model_name=None)
            self.forest_info[tgt] = info

        # now ignite that cell *and* spawn its healthy cluster
        self.spawn_cluster(tgt, initial_state=1)

    def spawn_cluster(self, center_key, initial_state=0):
        cr, cc = center_key
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                key = (cr + di, cc + dj)
                if not (0 <= key[0] < self.grid_rows and 0 <= key[1] < self.grid_cols):
                    continue
                info = self.forest_info[key]
                if key == center_key:
                    # always ignite the new cell
                    self.spawn_cell(key, state=initial_state)
                else:
                    # only spawn a healthy neighbour if it hasn't been spawned yet
                    if info.get("spawned", False) is False:
                        self.spawn_cell(key, state=0)

    def spawn_cell(self, key, state=0):
        info = self.forest_info.get(key, {"row":key[0],"col":key[1],"max_fire":1})

        # 🎯 delete _any_ old model before we spawn a fresh one
        old = info.get("model_name")
        if old:
            try:
                self.delete_model(old)
                self.delete_count += 1
                self.get_logger().info(f"Deleted old model {old} before spawning at {key}")
            except Exception as e:
                self.get_logger().error(f"Failed to delete {old}: {e}")
            info["model_name"] = None

        maxf = info["max_fire"]
        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{maxf}_ball",
            f"forest_{maxf}_ball_state_{state}",
            "model.sdf"
        )
        x = self.offset_x + key[1] * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
        name = f"forest_cell{key[0]}_{key[1]}_{int(time.time()*1000)}"
        cmd = ["ros2","run","ros_gz_sim","create","-demo","default",
               "-file",sdf,"-x",str(x),"-y",str(y),"-z","0.01","-name",name]
        try:
            self.run_cmd_with_retry(cmd)
            self.spawn_count += 1
            # set cstate for the newly burning cell
            cstate = (state-1)*200 + 1 if state>0 else 0
            info.update(state=state, cstate=cstate, model_name=name, spawned=True)
            self.forest_info[key] = info
            self.get_logger().info(f"Spawned cell {name} at {key} state {state}")
            time.sleep(0.3)
        except Exception as e:
            self.get_logger().error(f"Spawn cell failed: {e}")

    def delete_model(self, model_name):
        cmd = ["ros2","run","ros_gz_sim","remove",
               "--ros-args","-p",f"entity_name:={model_name}"]
        try:
            self.run_cmd_with_retry(cmd)
            self.get_logger().info(f"Deleted {model_name}")
        except Exception as e:
            self.get_logger().error(f"Delete failed: {e}")

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
