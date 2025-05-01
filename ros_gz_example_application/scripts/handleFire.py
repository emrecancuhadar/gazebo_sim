#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import json, subprocess, time, os
from subprocess import CalledProcessError, TimeoutExpired
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry

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

        self.robot_cell = None
        self.paused_cells = {}  # key → pause_end_time (epoch seconds)

        self._last_pose = None  # will hold (x, y, row, col)

        # Counters
        self.spawn_count = 0
        self.delete_count = 0
        self.start_time = time.time()
        

        # Initial forest_info subscription: keep only first snapshot
        self.initial_info_received = False
        self.forest_info_sub = self.create_subscription(
            String, 'forest_info', self.forest_info_callback, 10
        )
        # Publisher for downstream _processed_ forest info
        self.forest_info_pub = self.create_publisher(String, '/processed_forest_info', 10)

        # Odometry subscriber
        self.create_subscription(Odometry, '/diff_drive/odometry', self.odometry_callback, 10)


        # Wind (for simple, single-direction spread)
        self.wind_dx = 1
        self.wind_dy = 0

        # We’ll batch spreads every cycle and then apply
        self._pending_spreads = set()

        # Timers
        self.create_timer(2.0, self.update_burning_cells)
        self.create_timer(5.0, self.print_counters)
        self.create_timer(2.0, self.log_robot_position)
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
            # only grab the VERY FIRST non-empty forest_info
            if self.initial_info_received:
                return

            cells = json.loads(msg.data)
            if len(cells) == 0:
                # we got an empty one (probably our own!), so wait for the real publisher
                self.get_logger().info("forest_info_callback: empty, waiting for real data…")
                return

            # now we have data!
            for cell in cells:
                key = (cell['row'], cell['col'])
                self.forest_info[key] = cell
            self.initial_info_received = True

            # tear down this sub so we don't hear our own republish
            self.destroy_subscription(self.forest_info_sub)
            self.get_logger().info(
                f"Initial forest_info loaded ({len(self.forest_info)} cells); unsubscribed."
            )       
        except Exception as e:
            self.get_logger().error(f"Error parsing forest_info: {e}")


    def odometry_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # map to nearest column
        idx_col = int(round((x - self.offset_x) / self.cell_size))
        # map to nearest "y‐index" then invert to row
        idx_y   = int(round((y - self.offset_y) / self.cell_size))
        row     = self.grid_rows - 1 - idx_y
        col     = idx_col

        # clamp to [0..9]
        row = max(0, min(self.grid_rows - 1, row))
        col = max(0, min(self.grid_cols - 1, col))

        self._last_pose = (x, y, row, col)
        if not (0 <= row < self.grid_rows and 0 <= col < self.grid_cols):
            return

        key = (row, col)
        cell = self.forest_info.get(key)
        if cell and cell.get("cstate", 0) > 0 and cell.get("state", 0) < 6:
            # compute world‐coords of cell center
            cx = self.offset_x + key[1] * self.cell_size
            cy = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
            dist = ((x - cx)**2 + (y - cy)**2)**0.5
            if dist <= 0.05 and key not in self.paused_cells:
                # record the moment we arrived
                self.paused_cells[key] = time.time()
                self.get_logger().info(f"Paused cell {key} at t={self.paused_cells[key]:.1f}")

    def log_robot_position(self):
        # called by timer every 2s
        # called by timer every 2s to report the last-seen odom
        if self._last_pose is None:
            return
        x, y, row, col = self._last_pose
        self.get_logger().info(f"Robot at x={x:.2f}, y={y:.2f} → cell ({row}, {col})")

    def update_burning_cells(self):
        self._pending_spreads.clear()
        now = time.time()

        for key, cell in list(self.forest_info.items()):
            # 1) never touch an already‐extinguished cell
            if cell.get("state", 0) == 6:
                continue

            # 2) handle robot‐paused cells
            if key in self.paused_cells:
                start = self.paused_cells[key]
                if now < start + 30.0:
                    # still within 30s window: skip this cell entirely
                    continue
                # 30s elapsed → extinguish permanently
                del self.paused_cells[key]
                cell["cstate"] = -1
                self.get_logger().info(f"30 s done for {key}; forcing state 6")
                self.update_cell(key, force_state=6)
                # *immediately* push out the new zeroed grid so clients can pick up
                self.publish_forest_info()
                self.publish_grid_data()
                continue


            # regular burning progression (unchanged)
            c = cell.get("cstate", 0)
            m = cell.get("max_fire", 1)
            c_max = m * 200
            if c > 0 and c < c_max * 3:
                interval = c_max / 5.0

                if c < c_max:
                    idx = int((c - 1) // interval)
                    inc = (idx + 1) * 3
                else:
                    inc = 15

                new_c = min(c + inc, int(c_max * 3))
                cell["cstate"] = new_c
                self.get_logger().info(f"Cell {key} cstate -> {new_c}")

                new_state = min(int((new_c - 1) // interval) + 1, 5)
                if new_state > cell.get("state", 0):
                    self.update_cell(key, force_state=new_state)

                if new_c >= c_max:
                    self._pending_spreads.add(key)

        # fire spread (unchanged)
        for src in self._pending_spreads:
            self.spread_fire(src)

        self.publish_forest_info()
        self.publish_grid_data()

    def update_cell(self, key, force_state=None):
        cell = self.forest_info[key]
        cur = cell.get("state", 0)
        new_state = force_state if force_state and force_state > cur else (cur + 1)

        # if fully burnt and state 5, just hold
        if cell["cstate"] >= cell["max_fire"] * 200 * 3 and new_state <= 5:
            self.get_logger().info(f"Cell {key} fully burnt, holding at state 5")
            return

        # remove old model
        old = cell.get("model_name")
        if old:
            try:
                self.run_cmd_with_retry([
                    "ros2", "run", "ros_gz_sim", "remove",
                    "--ros-args", "-p", f"entity_name:={old}"
                ])
                self.delete_count += 1
                self.get_logger().info(f"Deleted {old}")
            except Exception as e:
                self.get_logger().error(f"Delete failed: {e}")
                return

        # spawn new visual for the new_state (1–6)
        m = cell["max_fire"]
        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{m}_ball",
            f"forest_{m}_ball_state_{new_state}",
            "model.sdf"
        )
        x = self.offset_x + key[1] * self.cell_size
        y = self.offset_y + (self.grid_rows - 1 - key[0]) * self.cell_size
        name = f"forest_cell{key[0]}_{key[1]}"
        try:
            self.run_cmd_with_retry([
                "ros2", "run", "ros_gz_sim", "create", "-demo", "default",
                "-file", sdf, "-x", str(x), "-y", str(y), "-z", "0.01", "-name", name
            ])
            self.spawn_count += 1
            cell.update(model_name=name, state=new_state)
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
        name = f"forest_cell{key[0]}_{key[1]}"
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
        self.get_logger().info("Published updated processed_forest_info")

    def publish_grid_data(self):
        fire_count = []
        fuel_load  = []
        vegetation = []

        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                key  = (r, c)
                cell = self.forest_info.get(key, {})

                # if state==6, report 0; else report its state
                st = cell.get('state', 0)
                fire_count.append(0.0 if st == 6 else float(st))

                fuel_load.append(float(cell.get('max_fire', 1)))
                vegetation.append(cell.get('label', 'sparse'))

        fc_msg = Float32MultiArray(data=fire_count)
        fl_msg = Float32MultiArray(data=fuel_load)
        veg_msg = String(data=json.dumps(vegetation))

        self.fire_count_pub.publish(fc_msg)
        self.fuel_load_pub.publish(fl_msg)
        self.vegetation_pub.publish(veg_msg)
        self.get_logger().info("Published /grid/fire_count, /grid/fuel_load, /grid/vegetation")


def main(args=None):
    rclpy.init(args=args)
    node = HandleFire()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
