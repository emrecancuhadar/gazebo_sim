#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time, os, subprocess
from subprocess import CalledProcessError, TimeoutExpired
from ament_index_python.packages import get_package_share_directory

from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose

class InitForest(Node):
    def __init__(self):
        super().__init__('init_forest')
        # Grid params
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
        self.offset_x = -((self.grid_cols - 1)*self.cell_size)/2.0
        self.offset_y = -((self.grid_rows - 1)*self.cell_size)/2.0

        # Script paths
        self.intensity_script = "/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_application/scripts/greenIntensity.py"
        self.season_script    = "/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_application/scripts/fourseasons.py"
        self.share_dir        = get_package_share_directory('ros_gz_example_description')

        # Data paths
        self.summer_red = '/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_description/models/my_ground_plane/materials/textures/summer_red.tiff'
        self.summer_nir = '/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_description/models/my_ground_plane/materials/textures/summer_nir.tiff'
        self.winter_red = '/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_description/models/my_ground_plane/materials/textures/winter_red.tiff'
        self.winter_nir = '/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_description/models/my_ground_plane/materials/textures/winter_nir.tiff'
        self.winter_img = '/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_description/models/my_ground_plane/materials/textures/14ocak2024normalview.jpg'

        # Publisher
        self.forest_info = {}
        self.pub = self.create_publisher(String, 'forest_info', 10)
        self.get_logger().info("InitForest: Starting forest initialization...")

        # Service client for spawn
        self.spawn_cli = self.create_client(SpawnEntity, '/world/demo/create')
        if not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("InitForest: spawn service not available")
            return

        # 1) Run greenIntensity
        intens = self.call_green_intensity_script()
        # 2) Run fourseasons
        labels = self.call_season_script()
        if intens is None or labels is None:
            self.get_logger().error("InitForest: failed to compute grids")
            return

        # 3) Initialize forest_info
        label_map = {1:'Bare Soil',2:'Deciduous',3:'Coniferous',4:'Sparse Veg'}
        for i in range(self.grid_rows):
            for j in range(self.grid_cols):
                v = intens[i][j]
                if   v>1.7: m=5
                elif v>1.4: m=4
                elif v>1.1: m=3
                elif v>0.8: m=2
                else:      m=1
                code = int(labels[i][j])
                self.forest_info[(i,j)] = {
                    'row':i,'col':j,
                    'max_fire':m,'state':0,'cstate':0,
                    'model_name':None,'spawned':False,
                    'label':label_map.get(code,'Unclassified')
                }

        # 4) DATA‐DRIVEN SPAWNS
        centers = [
            (8, 8, 2),
            (5, 1, 2),
            (0, 0, 2),
            (2, 2, 2),
            (1, 3, 2)
        ]
        spawn_map = {}
        n_states = 5
        for i, j, fv in centers:
            cell   = self.forest_info[(i, j)]
            m      = cell['max_fire']
            c_max  = m * 200
            interval = c_max / n_states
            cs     = int(interval * (fv - 1) + 1)
            spawn_map[(i, j)] = (fv, cs)
        for i, j, _ in centers:
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    ni, nj = i + di, j + dj
                    if (ni, nj) == (i, j): continue
                    if not (0 <= ni < self.grid_rows and 0 <= nj < self.grid_cols): continue
                    nbr = self.forest_info[(ni, nj)]
                    if nbr['cstate'] == 0:
                        spawn_map.setdefault((ni, nj), (0, 0))

        # spawn each cell via service
        for (row, col), (st, cs) in spawn_map.items():
            self.spawn_cell(row, col, st, cs)

        # 5) Publish once
        self.publish_forest_info()
        self.get_logger().info("InitForest: Initialization complete.")

    def call_green_intensity_script(self):
        try:
            res = subprocess.run(
                ["python3", self.intensity_script, self.winter_img],
                capture_output=True, text=True, check=True, timeout=30
            )
            arr = json.loads(res.stdout)
            self.get_logger().info(f"InitForest: greenIntensity output:\n{res.stdout}")
            return arr
        except Exception as e:
            self.get_logger().error(f"InitForest: Error running greenIntensity: {e}")
            return None

    def call_season_script(self):
        try:
            res = subprocess.run(
                ["python3", self.season_script,
                 self.summer_red, self.summer_nir,
                 self.winter_red, self.winter_nir],
                capture_output=True, text=True, check=True, timeout=60
            )
            arr = json.loads(res.stdout)
            self.get_logger().info(f"InitForest: fourseasons output:\n{res.stdout}")
            return arr
        except Exception as e:
            self.get_logger().error(f"InitForest: Error running fourseasons: {e}")
            return None

    def spawn_cell(self, row, col, state, cstate):
        info = self.forest_info[(row, col)]
        if info['spawned']:
            return

        mf = info['max_fire']
        sdf = os.path.join(
            self.share_dir,
            'models', f'forest_{mf}_ball',
            f'forest_{mf}_ball_state_{state}',
            'model.sdf'
        )
        x = self.offset_x + col*self.cell_size
        y = self.offset_y + (self.grid_rows-1-row)*self.cell_size
        name = f"forest_cell{row}_{col}"

        # build and send service request instead of subprocess
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

        self.spawn_cli.call_async(req)

        info.update(state=state, cstate=cstate, model_name=name, spawned=True)
        self.get_logger().info(f"Spawned cell {name} at {(row, col)} state {state}")
        time.sleep(0.3)

    def publish_forest_info(self):
        arr = list(self.forest_info.values())
        msg = String()
        msg.data = json.dumps(arr)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InitForest()
    # spin briefly so that async calls go out
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
