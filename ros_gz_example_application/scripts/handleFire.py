#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
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

        self.fire_count_pub  = self.create_publisher(Float32MultiArray, '/grid/fire_count', 10)
        self.fuel_load_pub   = self.create_publisher(Float32MultiArray, '/grid/fuel_load', 10)
        self.vegetation_pub  = self.create_publisher(String,            '/grid/vegetation', 10)


        self.share_dir = get_package_share_directory('ros_gz_example_description')
        self.forest_info = {}  # will hold per-cell dicts, including 'max_fire', 'cstate', 'state', and now 'detected_balls'

        # Counters
        self.spawn_count = 0
        self.delete_count = 0
        self.start_time = time.time()

        # Subscribers & Publishers
        #  - forest_info should eventually include a 'detected_balls' field for each cell
        self.create_subscription(String, 'forest_info', self.forest_info_callback, 10)
        self.forest_info_pub = self.create_publisher(String, 'forest_info', 10)

        # Wind (for simple, single-direction spread)
        self.wind_dx = 0
        self.wind_dy = 1

        # We’ll batch spreads every cycle and then apply
        self._pending_spreads = set()

        # Timers
        self.create_timer(2.0, self.update_burning_cells)
        self.create_timer(5.0, self.print_counters)
        self.spawn_wind_model(self.wind_dx, self.wind_dy)

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

    def spawn_wind_model(self, dx, dy):
        if(dx == 0 and dy == 0):
            return
            
        dy = -dy
        sdf=os.path.join(self.share_dir, 'models','wind',f'{dx}_{dy}','model.sdf')
        x = 14.5
        y = 0
        name="wind_direction"
        cmd=["ros2","run","ros_gz_sim","create","-demo","default","-file",sdf,
             "-x",str(x),"-y",str(y),"-z","0.01","-name",name]
        try:
            self.run_cmd_with_retry(cmd)
        except Exception as e:
            self.get_logger.error("Error while spawning the wind direction model.")

    def print_counters(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Elapsed {elapsed:.1f}s | Spawns: {self.spawn_count} | Deletes: {self.delete_count}")

    def forest_info_callback(self, msg):
        try:
            cells = json.loads(msg.data)
            for cell in cells:
                key = (cell["row"], cell["col"])
                # only accept newer cstate
                if key in self.forest_info and self.forest_info[key].get("cstate",0) > cell.get("cstate",0):
                    continue
                # you can include 'detected_balls' in each cell dict to drive initial cstate
                self.forest_info[key] = cell
            self.get_logger().info(f"Forest info updated ({len(self.forest_info)} cells)")
        except Exception as e:
            self.get_logger().error(f"Error parsing forest_info: {e}")

    def update_burning_cells(self):
        self._pending_spreads.clear()

        for key, cell in list(self.forest_info.items()):
            c = cell.get("cstate", 0)
            m = cell.get("max_fire", 1)
            c_max = m * 200
            # only progress burning cells up to 3x max
            if c > 0 and c < c_max * 3:
                interval = c_max / 5.0

                # new increment values: 3,6,9,12,15
                if c < c_max:
                    idx = int((c - 1) // interval)
                    inc = (idx + 1) * 3
                else:
                    inc = 15

                new_c = min(c + inc, int(c_max * 3))
                cell["cstate"] = new_c
                self.get_logger().info(f"Cell {key} cstate -> {new_c}")

                # determine visual state 1-5
                new_state = min(int((new_c - 1) // interval) + 1, 5)
                if new_state > cell.get("state", 0):
                    self.update_cell(key, force_state=new_state)

                if new_c >= c_max:
                    self._pending_spreads.add(key)

        # perform spreads
        for src in self._pending_spreads:
            self.spread_fire(src)

        self.publish_forest_info()
        self.publish_grid_data()

    def update_cell(self, key, force_state=None):
        cell = self.forest_info[key]
        cur = cell.get("state", 0)
        new_state = force_state if force_state and force_state > cur else (cur + 1)

        # if fully burnt (cstate >= max_fire*200*3), we leave it at state 5
        if cell["cstate"] >= cell["max_fire"] * 200 * 3:
            self.get_logger().info(f"Cell {key} fully burnt, holding at state 5")
            return

        # remove old model
        old = cell.get("model_name")
        if old:
            cmd = ["ros2", "run", "ros_gz_sim", "remove", "--ros-args", "-p", f"entity_name:={old}"]
            try:
                self.run_cmd_with_retry(cmd)
                self.delete_count += 1
                self.get_logger().info(f"Deleted {old}")
            except Exception as e:
                self.get_logger().error(f"Delete failed: {e}")
                return

        # spawn new visual for the new_state
        m = cell["max_fire"]
        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{m}_ball",
            f"forest_{m}_ball_state_{new_state}",
            "model.sdf"
        )
        x = self.offset_x + key[1] * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
        name = f"forest_cell{key[0]}_{key[1]}_{int(time.time()*1000)}"
        cmd = [
            "ros2", "run", "ros_gz_sim", "create", "-demo", "default",
            "-file", sdf, "-x", str(x), "-y", str(y), "-z", "0.01", "-name", name
        ]
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

        if not (0 <= tgt[0] < self.grid_rows and 0 <= tgt[1] < self.grid_cols):
            return

        info = self.forest_info.get(tgt)
        if not info or info.get("cstate", 0) != 0:
            return

        self.get_logger().info(f"Spreading fire from {source_key} to {tgt}")

        # delete healthy model
        old = info.get("model_name")
        if old:
            self.delete_model(old)
            self.delete_count += 1
            info.update(spawned=False, model_name=None)
            self.forest_info[tgt] = info

        # ignite that cell and its neighbours
        self.spawn_cluster(tgt)

    def spawn_cluster(self, center_key):
        cr, cc = center_key
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                key = (cr + di, cc + dj)
                if not (0 <= key[0] < self.grid_rows and 0 <= key[1] < self.grid_cols):
                    continue
                if key == center_key:
                    # ignite center
                    self.spawn_cell(key, ignite=True)
                else:
                    # healthy neighbour
                    info = self.forest_info[key]
                    if not info.get("spawned", False):
                        self.spawn_cell(key, ignite=False)

    def spawn_cell(self, key, ignite=False):
        # fetch or default
        info = self.forest_info.get(key, {"row":key[0],"col":key[1],"max_fire":1})
        old = info.get("model_name")
        if old:
            try:
                self.delete_model(old)
                self.delete_count += 1
            except Exception:
                pass

        m = info["max_fire"]
        state = 1 if ignite else 0

        # if ignition, compute initial cstate from detected_balls
        if ignite:
            c_max = m * 200
            interval = c_max / 5.0
            balls = info.get("detected_balls", 1)
            balls = max(1, min(int(balls), 5))
            cstate = int(interval * (balls - 1) + 1)
        else:
            cstate = 0

        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{m}_ball",
            f"forest_{m}_ball_state_{state}",
            "model.sdf"
        )
        x = self.offset_x + key[1] * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
        name = f"forest_cell{key[0]}_{key[1]}_{int(time.time()*1000)}"
        cmd = [
            "ros2", "run", "ros_gz_sim", "create", "-demo", "default",
            "-file", sdf, "-x", str(x), "-y", str(y), "-z", "0.01", "-name", name
        ]
        try:
            self.run_cmd_with_retry(cmd)
            self.spawn_count += 1
            info.update(model_name=name, state=state, cstate=cstate, spawned=True)
            self.forest_info[key] = info
            self.get_logger().info(f"Spawned cell {name} at {key} state {state} cstate {cstate}")
            time.sleep(0.3)
        except Exception as e:
            self.get_logger().error(f"Spawn cell failed: {e}")

    def delete_model(self, model_name):
        cmd = [
            "ros2","run","ros_gz_sim","remove",
            "--ros-args","-p",f"entity_name:={model_name}"
        ]
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

    def publish_grid_data(self):
        # walk row-major over the 10×10 grid
        fire_count = []
        fuel_load  = []
        vegetation = []

        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                key  = (r, c)
                cell = self.forest_info.get(key, {})
                # 1. burning or not
                fire_count.append(float(cell.get('state', 0)))
                # 2. fuel load (reuse max_fire as placeholder)
                fuel_load.append(float(cell.get('max_fire', 1)))
                # 3. vegetation label
                vegetation.append(cell.get('vegetation', 'sparse'))

        # publish fire_count
        fc_msg = Float32MultiArray()
        fc_msg.data = fire_count
        self.fire_count_pub.publish(fc_msg)

        # publish fuel_load
        fl_msg = Float32MultiArray()
        fl_msg.data = fuel_load
        self.fuel_load_pub.publish(fl_msg)

        # publish vegetation JSON
        veg_msg = String()
        veg_msg.data = json.dumps(vegetation)
        self.vegetation_pub.publish(veg_msg)

        self.get_logger().info("Published /grid/fire_count, /grid/fuel_load, /grid/vegetation")


def main(args=None):
    rclpy.init(args=args)
    node = HandleFire()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
