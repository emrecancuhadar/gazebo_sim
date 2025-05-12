#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from ros_gz_interfaces.msg import EntityFactory, Entity
from ament_index_python.packages import get_package_share_directory

import json, time, os
from collections import deque

class HandleFire(Node):
    def __init__(self):
        super().__init__('handle_fire')

        # ─── Grid setup ─────────────────────────────────────────────────────
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
        self.offset_x = -((self.grid_cols - 1) * self.cell_size) / 2.0
        self.offset_y = -((self.grid_rows - 1) * self.cell_size) / 2.0

        # ─── Publishers & Subscribers ──────────────────────────────────────
        self.fire_count_pub  = self.create_publisher(Float32MultiArray, '/grid/fire_count', 10)
        self.fuel_load_pub   = self.create_publisher(Float32MultiArray, '/grid/fuel_load', 10)
        self.vegetation_pub  = self.create_publisher(String,            '/grid/vegetation', 10)
        self.forest_info_pub = self.create_publisher(String,            '/processed_forest_info', 10)

        self.initial_info_received = False
        self.forest_info_sub = self.create_subscription(
            String, 'forest_info', self.forest_info_callback, 10
        )
        self.create_subscription(Odometry, '/diff_drive/odometry', self.odometry_callback, 10)

        # ─── Internal State ─────────────────────────────────────────────────
        self.share_dir    = get_package_share_directory('ros_gz_example_description')
        self.forest_info  = {}
        self.paused_cells = {}
        self._last_pose   = None

        # ─── Counters & Queues ──────────────────────────────────────────────
        self.spawn_count  = 0
        self.delete_count = 0
        self.start_time   = time.time()
        self.spawn_queue  = deque()
        self.delete_queue = deque()
        self._max_retries = 3

        # ─── Wind Model Direction ──────────────────────────────────────────
        self.wind_dx = 1
        self.wind_dy = 1

        # ─── Service Clients ───────────────────────────────────────────────
        self.spawn_cli  = self.create_client(SpawnEntity, '/world/demo/create')
        self.delete_cli = self.create_client(DeleteEntity, '/world/demo/remove')
        if not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('SpawnEntity service not available, exiting')
            rclpy.shutdown(); return
        if not self.delete_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('DeleteEntity service not available, exiting')
            rclpy.shutdown(); return

        # ─── Timers ─────────────────────────────────────────────────────────
        self.create_timer(2.0, self.update_burning_cells)
        self.create_timer(5.0, self.print_counters)
        self.create_timer(2.0, self.log_robot_position)
        # throttle: one request every 0.1s
        self.create_timer(0.1, self._process_queues)

        # ─── Spawn wind arrow ───────────────────────────────────────────────
        self.spawn_wind_model(self.wind_dx, self.wind_dy)


    # ─── Queue Processor (deletes first!) ───────────────────────────────
    def _process_queues(self):
        if self.delete_queue:
            name, attempt = self.delete_queue.popleft()
            self._call_delete(name, attempt)
        elif self.spawn_queue:
            name, sdf, x, y, attempt = self.spawn_queue.popleft()
            self._call_spawn(name, sdf, x, y, attempt)


    # ─── Spawn Logic ─────────────────────────────────────────────────────
    def _call_spawn(self, name, sdf, x, y, attempt):
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory(
            name=name,
            allow_renaming=False,
            sdf_filename=sdf,
            pose=Pose(),
            relative_to='world'
        )
        req.entity_factory.pose.position.x = x
        req.entity_factory.pose.position.y = y
        req.entity_factory.pose.position.z = 0.01

        fut = self.spawn_cli.call_async(req)
        fut.add_done_callback(
            lambda f, nm=name, s=sdf, xx=x, yy=y, att=attempt:
                self._on_spawn_response(nm, f, s, xx, yy, att)
        )

    def _on_spawn_response(self, name, future, sdf, x, y, attempt):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception spawning {name}: {e}')
            if attempt < self._max_retries:
                self.spawn_queue.append((name, sdf, x, y, attempt+1))
            return

        if resp.success:
            self.spawn_count += 1
            self.get_logger().info(f'✅ Spawn succeeded: {name}')
        else:
            self.get_logger().error(f'❌ Spawn failed: {name}')
            if attempt < self._max_retries:
                self.spawn_queue.append((name, sdf, x, y, attempt+1))


    # ─── Delete Logic ────────────────────────────────────────────────────
    def _call_delete(self, name, attempt):
        req = DeleteEntity.Request()
        req.entity = Entity(id=0, name=name, type=Entity.MODEL)

        fut = self.delete_cli.call_async(req)
        fut.add_done_callback(
            lambda f, nm=name, att=attempt:
                self._on_delete_response(nm, f, att)
        )

    def _on_delete_response(self, name, future, attempt):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception deleting {name}: {e}')
            if attempt < self._max_retries:
                self.delete_queue.append((name, attempt+1))
            return

        if resp.success:
            self.delete_count += 1
            self.get_logger().info(f'✅ Delete succeeded: {name}')
        else:
            self.get_logger().error(f'❌ Delete failed: {name}')
            if attempt < self._max_retries:
                self.delete_queue.append((name, attempt+1))


    # ─── Wind Model Spawn ────────────────────────────────────────────────
    def spawn_wind_model(self, dx, dy):
        if dx == 0 and dy == 0:
            return
        dy = -dy
        sdf = os.path.join(self.share_dir, 'models', 'wind', f'{dx}_{dy}', 'model.sdf')
        name, x, y = "wind_direction", 14.5, 0.0
        self.spawn_queue.append((name, sdf, x, y, 1))


    # ─── Receive initial forest_info ────────────────────────────────────
    def forest_info_callback(self, msg: String):
        if self.initial_info_received:
            return

        cells = json.loads(msg.data)
        if not cells:
            self.get_logger().info("forest_info_callback: empty, waiting…")
            return

        for cell in cells:
            key = (cell['row'], cell['col'])
            self.forest_info[key] = cell

        self.initial_info_received = True
        self.destroy_subscription(self.forest_info_sub)
        self.get_logger().info(f"Loaded {len(cells)} initial cells.")
        self.get_logger().info(
            f"Got forest_info for {(cell['row'],cell['col'])}: max_fire={cell.get('max_fire')} "
            f"detected_balls={cell.get('detected_balls')}"
        )


        # (Optional) enqueue a state‐0 spawn for every cell here
        # to visualize the full grid at startup.


    # ─── Odometry → cell mapping ─────────────────────────────────────────
    def odometry_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        col = int(round((x - self.offset_x) / self.cell_size))
        row = self.grid_rows - 1 - int(round((y - self.offset_y) / self.cell_size))
        row = max(0, min(self.grid_rows-1, row))
        col = max(0, min(self.grid_cols-1, col))

        self._last_pose = (x, y, row, col)
        cell = self.forest_info.get((row,col))
        if cell and       cell.get("cstate",0) > 0 \
            and  cell.get("state",  0) < 6:
            cx = self.offset_x + col*self.cell_size
            cy = self.offset_y + (self.grid_rows-1-row)*self.cell_size
            if ((x-cx)**2 + (y-cy)**2)**0.5 <= 0.05 and (row,col) not in self.paused_cells:
                self.paused_cells[(row,col)] = time.time()
                self.get_logger().info(f"Paused cell {(row,col)} at t={self.paused_cells[(row,col)]:.1f}")


    # ─── Timed Utilities ──────────────────────────────────────────────────
    def print_counters(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"Elapsed {elapsed:.1f}s | Spawns: {self.spawn_count} | Deletes: {self.delete_count}")

    def log_robot_position(self):
        if not self._last_pose:
            return
        x, y, r, c = self._last_pose
        self.get_logger().info(f"Robot at x={x:.2f}, y={y:.2f} → cell ({r},{c})")


    # ─── Fire progression ────────────────────────────────────────────────
    def update_burning_cells(self):
        now = time.time()
        pending_spreads = set()

        for key, cell in list(self.forest_info.items()):
            state  = cell.get("state",0)
            cstate = cell.get("cstate",0)
            m      = cell.get("max_fire",1)
            c_max  = m * 200

            # already extinguished?
            if state == 6:
                continue

            # paused by robot?
            if key in self.paused_cells:
                start = self.paused_cells[key]
                # still within 30s: skip burning/progression
                if now < start + 30.0:
                    continue
                # 30s elapsed → extinguish permanently
                del self.paused_cells[key]
                cell["cstate"] = -1
                self.get_logger().info(f"30 s done for {key}; forcing state 6")
                self.update_cell(key, force_state=6)
                # immediately push out the zeroed grid so clients see it
                self.publish_forest_info()
                self.publish_grid_data()
                continue

            # fully burnt?
            if cstate >= c_max*3 and state != 7:
                self.get_logger().info(f"Cell {key} burnt → state 7")
                self.update_cell(key, force_state=7)
                continue

            # regular progression
            if 0 < cstate < c_max*3:
                interval = c_max / 5.0

                if cstate < c_max:
                    # state 1–4: increase by 3×(current bucket index+1)
                    idx = int((cstate - 1) // interval)
                    inc = (idx + 1) * 3
                else:
                    # state 5: always +15
                    inc = 15

                new_c = min(cstate + inc, int(c_max * 3))
                cell["cstate"] = new_c
                self.get_logger().info(f"Cell {key} cstate → {new_c}")

                # update visual state if we crossed into a new bucket
                new_state = min(int((new_c - 1) // interval) + 1, 5)
                if new_state > state:
                    self.update_cell(key, force_state=new_state)

                # when we hit or exceed c_max, schedule a spread
                if new_c >= c_max:
                    pending_spreads.add(key)

        for src in pending_spreads:
            self.spread_fire(src)

        self.publish_forest_info()
        self.publish_grid_data()


    # ─── Replace one cell’s model ────────────────────────────────────────
    def update_cell(self, key, force_state=None):
        cell = self.forest_info[key]
        cur  = cell.get("state",0)
        new_state = force_state if force_state and force_state>cur else (cur+1)
        m = cell.get("max_fire",1)

        if cell.get("cstate",0) >= m*200*3 and new_state <= 5:
            return

        # queue delete of old
        old = cell.get("model_name")
        if old:
            self.delete_queue.append((old,1))

        # queue spawn of new
        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{m}_ball",
            f"forest_{m}_ball_state_{new_state}",
            "model.sdf"
        )
        x = self.offset_x + key[1]*self.cell_size
        y = self.offset_y + (self.grid_rows-1-key[0])*self.cell_size
        name = f"forest_cell{key[0]}_{key[1]}"
        self.spawn_queue.append((name,sdf,x,y,1))

        cell.update(model_name=name, state=new_state)


    # ─── Fire spread ─────────────────────────────────────────────────────
    def spread_fire(self, source_key):
        r, c = source_key
        tgt = (r + self.wind_dy, c + self.wind_dx)
        if not (0 <= tgt[0] < self.grid_rows and 0 <= tgt[1] < self.grid_cols):
            return

        info = self.forest_info[tgt]
        if info.get("cstate",0) != 0:
            return

        self.get_logger().info(f"Spreading from {source_key} to {tgt}")
        old = info.get("model_name")
        if old:
            self.delete_queue.append((old,1))
            info.update(spawned=False, model_name=None)

        self.spawn_cluster(tgt)


    # ─── Ignite cluster ──────────────────────────────────────────────────
    def spawn_cluster(self, center_key):
        r0, c0 = center_key
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                key = (r0 + dr, c0 + dc)
                # only within grid
                if not (0 <= key[0] < self.grid_rows and 0 <= key[1] < self.grid_cols):
                    continue

                if key == center_key:
                    # always ignite the center of the new fire
                    self.spawn_cell(key, ignite=True)
                else:
                    # only create the healthy-neighbour visuals if they haven't been spawned yet
                    info = self.forest_info.get(key)
                    if info and not info.get("spawned", False):
                        self.spawn_cell(key, ignite=False)


    # ─── Spawn a single cell ─────────────────────────────────────────────
    def spawn_cell(self, key, ignite=False):
        info = self.forest_info.get(key, {"row":key[0],"col":key[1],"max_fire":1})
        old  = info.get("model_name")
        if old:
            self.delete_queue.append((old,1))

        m = info["max_fire"]
        state = 1 if ignite else 0
        cstate = 0
        if ignite:
            c_max = m * 200
            interval = c_max / 5.0
            balls = max(1, min(int(info.get("detected_balls",1)), 5))
            cstate = int(interval * (balls-1) + 1)

        sdf = os.path.join(
            self.share_dir,
            "models", f"forest_{m}_ball",
            f"forest_{m}_ball_state_{state}",
            "model.sdf"
        )
        x = self.offset_x + key[1]*self.cell_size
        y = self.offset_y + (self.grid_rows-1-key[0])*self.cell_size
        name = f"forest_cell{key[0]}_{key[1]}"
        self.spawn_queue.append((name,sdf,x,y,1))

        info.update(model_name=name, state=state, cstate=cstate, spawned=True)
        self.forest_info[key] = info
        self.get_logger().info(f"Ignited {key}: state {state}, cstate {cstate}")


    # ─── Publish forest_info & grid data ─────────────────────────────────
    def publish_forest_info(self):
        self.forest_info_pub.publish(String(data=json.dumps(list(self.forest_info.values()))))

    def publish_grid_data(self):
        fc, fl, veg = [], [], []
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                cell = self.forest_info.get((r,c), {})
                st = cell.get("state",0)
                fc.append(0.0 if st in (6,7) else float(st))
                fl.append(float(cell.get("max_fire",1)))
                veg.append(cell.get("label","sparse"))
        self.fire_count_pub.publish(Float32MultiArray(data=fc))
        self.fuel_load_pub.publish(Float32MultiArray(data=fl))
        self.vegetation_pub.publish(String(data=json.dumps(veg)))


def main(args=None):
    rclpy.init(args=args)
    node = HandleFire()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
