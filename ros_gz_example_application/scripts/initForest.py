#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, subprocess, time, os
from ament_index_python.packages import get_package_share_directory

class InitForest(Node):
    def __init__(self):
        super().__init__('init_forest')

        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
        # Center the 10×10 grid at (0,0)
        self.offset_x = -((self.grid_cols - 1) * self.cell_size) / 2.0
        self.offset_y = -((self.grid_rows - 1) * self.cell_size) / 2.0

        self.intensity_script = "/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_application/scripts/greenIntensity.py"
        self.share_dir = get_package_share_directory('ros_gz_example_description')

        # Dictionary: key=(row,col) -> { row, col, max_fire, state, cstate, model_name, spawned }
        self.forest_info = {}
        self.forest_info_pub = self.create_publisher(String, 'forest_info', 10)

        self.get_logger().info("InitForest: Starting forest initialization...")

        # 1) Get intensities from greenIntensity.py
        intensities = self.call_green_intensity_script()
        if intensities is None:
            self.get_logger().error("InitForest: No intensities, aborting.")
            return

        # 2) For each cell, determine the forest type (max_fire) from intensity.
        #    We'll store max_fire in forest_info, but not necessarily spawn anything yet.
        for i in range(self.grid_rows):
            for j in range(self.grid_cols):
                val = intensities[i][j]
                if val > 1.7:
                    mfire = 5
                elif val > 1.4:
                    mfire = 4
                elif val > 1.1:
                    mfire = 3
                elif val > 0.8:
                    mfire = 2
                else:
                    mfire = 1
                self.forest_info[(i, j)] = {
                    "row": i,
                    "col": j,
                    "max_fire": mfire,
                    "state": 0,         # 0=healthy, 1–5=burning
                    "cstate": 0,        # continuous state
                    "model_name": None, # not spawned yet
                    "spawned": False    # whether we've actually spawned a visual
                }

        # 3) Hard-coded pseudo ball detector: only cell (5,5)=3, others=0
        pseudo_detector = [[0]*self.grid_cols for _ in range(self.grid_rows)]
        pseudo_detector[9][0] = 3

        # 4) For each cell with a nonzero ball value, spawn a 3×3 cluster
        #    The center is burning, the others are healthy.
        for i in range(self.grid_rows):
            for j in range(self.grid_cols):
                if pseudo_detector[i][j] > 0:
                    # Center cell is (i,j), so spawn a cluster around it
                    for di in [-1,0,1]:
                        for dj in [-1,0,1]:
                            rr = i + di
                            cc = j + dj
                            if 0 <= rr < self.grid_rows and 0 <= cc < self.grid_cols:
                                if rr == i and cc == j:
                                    # burning center
                                    ball_val = pseudo_detector[i][j] # e.g. 3
                                    cstate = (ball_val - 1)*200 + 1   # e.g. 401
                                    self.spawn_cell(rr, cc, state=ball_val, cstate=cstate)
                                else:
                                    # healthy
                                    self.spawn_cell(rr, cc, state=0, cstate=0)

        self.publish_forest_info()
        self.get_logger().info("InitForest: Initialization complete.")

    def call_green_intensity_script(self):
        try:
            result = subprocess.run(
                ["python3", self.intensity_script],
                capture_output=True, text=True, check=True, timeout=30
            )
            intensities = json.loads(result.stdout)
            self.get_logger().info(f"InitForest: greenIntensity output:\n{result.stdout}")
            return intensities
        except Exception as e:
            self.get_logger().error(f"InitForest: Error running greenIntensity: {e}")
            return None

    def spawn_cell(self, row, col, state, cstate):
        # Only spawn if not yet spawned
        info = self.forest_info.get((row,col), None)
        if info is None:
            self.get_logger().warn(f"InitForest: Missing forest_info for cell ({row},{col})")
            return
        if info["spawned"]:
            # Already spawned?
            return

        max_fire = info["max_fire"]
        # SDF path: models/forest_<max_fire>_ball/forest_<max_fire>_ball_state_<state>/model.sdf
        sdf_path = os.path.join(
            self.share_dir,
            "models",
            f"forest_{max_fire}_ball",
            f"forest_{max_fire}_ball_state_{state}",
            "model.sdf"
        )
        x = self.offset_x + col*self.cell_size
        y = self.offset_y + (self.grid_rows-1 - row)*self.cell_size
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
            self.get_logger().info(f"InitForest: Spawned {model_name} = type={max_fire}, state={state}")
            info["state"] = state
            info["cstate"] = cstate
            info["model_name"] = model_name
            info["spawned"] = True
            self.forest_info[(row,col)] = info
            time.sleep(0.3)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"InitForest: spawn_cell error for {model_name}: {e}")

    def publish_forest_info(self):
        arr = list(self.forest_info.values())
        msg = String()
        msg.data = json.dumps(arr)
        self.forest_info_pub.publish(msg)
        self.get_logger().info("InitForest: Published forest info.")


def main(args=None):
    rclpy.init(args=args)
    node = InitForest()
    # Spin briefly so it can publish forest_info once
    rclpy.spin_once(node, timeout_sec=2)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
