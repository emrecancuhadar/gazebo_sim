#!/usr/bin/env python3
import time
import os
import json
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory

class InitForestOnce(Node):
    """
    A one-shot forest initializer:
    - Subscribes once to '/grid/detected_balls'
    - Constructs centers from that data
    - Runs greenIntensity & fourseasons scripts
    - Spawns cells around each center and neighbors
    - Publishes forest_info and exits
    """
    def __init__(self):
        super().__init__('init_forest_once')

        # Grid parameters
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
        self.offset_x = -((self.grid_cols - 1) * self.cell_size) / 2.0
        self.offset_y = -((self.grid_rows - 1) * self.cell_size) / 2.0

        # Script & data paths
        pkg_share = get_package_share_directory('ros_gz_example_description')
        self.intensity_script = "/home/cagan-ozsir/two_wheel_ws/src/gazebo_sim/ros_gz_example_application/scripts/greenIntensity.py"
        self.season_script    = "/home/cagan-ozsir/two_wheel_ws/src/gazebo_sim/ros_gz_example_application/scripts/fourseasons.py"
        self.winter_img       = os.path.join(pkg_share, 'models', 'my_ground_plane', 'materials', 'textures', '14ocak2024normalview.jpg')
        self.summer_red       = os.path.join(pkg_share, 'models', 'my_ground_plane', 'materials', 'textures', 'summer_red.tiff')
        self.summer_nir       = os.path.join(pkg_share, 'models', 'my_ground_plane', 'materials', 'textures', 'summer_nir.tiff')
        self.winter_red       = os.path.join(pkg_share, 'models', 'my_ground_plane', 'materials', 'textures', 'winter_red.tiff')
        self.winter_nir       = os.path.join(pkg_share, 'models', 'my_ground_plane', 'materials', 'textures', 'winter_nir.tiff')

        # Placeholder for detected balls
        self.detected_balls = None  # list of (row, col, value)

        # Subscribe once to detected balls grid
        self.create_subscription(
            Float32MultiArray,
            '/grid/detected_balls',
            self._balls_callback,
            QoSProfile(depth=1)
        )
        self.get_logger().info('Waiting for /grid/detected_balls message...')

        # Spin until data arrives or timeout
        timeout = time.time() + 15.0
        while rclpy.ok() and self.detected_balls is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.detected_balls is None:
            self.get_logger().error('No ball data received; aborting.')
            return

        # Run preprocessing scripts
        intens = self._call_script(["python3", self.intensity_script, self.winter_img], 'greenIntensity')
        season = self._call_script([
            "python3", self.season_script,
            self.summer_red, self.summer_nir,
            self.winter_red, self.winter_nir
        ], 'fourseasons')
        if intens is None or season is None:
            self.get_logger().error('Preprocessing failed; aborting.')
            return

        # Build forest info
        self._build_forest_info(intens, season)

        # Determine centers from detected balls
        centers = self.detected_balls
        self.get_logger().info(f'Using centers: {centers}')

        # Prepare spawn client
        self.spawn_cli = self.create_client(SpawnEntity, '/world/demo/create')
        if not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Spawn service unavailable; aborting.')
            return

        # Spawn cells (centers + neighbors)
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

        # Spawn and publish
        for (row, col), (st, cs) in spawn_map.items():
            self._spawn_cell(row, col, st, cs)
        self._publish_forest_info()
        self.get_logger().info('Initialization complete.')

    def _balls_callback(self, msg: Float32MultiArray):
        lst = []
        for idx, w in enumerate(msg.data):
            # half-up rounding:
            rounded = int(w + 0.5)       # 0.4→0, 0.5→1, 0.6→1, 1.4→1, 1.5→2, etc.
            # clamp anything below 1 to zero
            count = rounded if rounded >= 1 else 0

            # skip zero entries
            if count == 0:
                continue

            row = idx // self.grid_cols
            col = idx % self.grid_cols
            lst.append((row, col, count))

        self.detected_balls = lst


    def _call_script(self, cmd, name):
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, check=True, timeout=60)
            self.get_logger().info(f'{name} output: {res.stdout.strip()}')
            return json.loads(res.stdout)
        except Exception as e:
            self.get_logger().error(f'{name} failed: {e}')
            return None

    def _build_forest_info(self, intens, season):
        labels = season['classification']
        dem    = season['elevation']
        slp    = season['slope']
        asp    = season['aspect']
        label_map = {1:'Bare Soil',2:'Deciduous',3:'Coniferous',4:'Sparse Veg'}
        self.forest_info = {}
        for i in range(self.grid_rows):
            for j in range(self.grid_cols):
                v = intens[i][j]
                m = 5 if v>1.7 else 4 if v>1.4 else 3 if v>1.1 else 2 if v>0.8 else 1
                code = int(labels[i][j])
                self.forest_info[(i,j)] = {
                    'row':i, 'col':j, 'max_fire':m,
                    'state':0, 'cstate':0, 'model_name':None,
                    'spawned':False,
                    'label':label_map.get(code,'Unclassified'),
                    'elevation':float(dem[i][j]),
                    'slope':float(slp[i][j]),
                    'aspect':float(asp[i][j])
                }

    def _spawn_cell(self, row, col, state, cstate):
        info = self.forest_info[(row, col)]
        if info['spawned']:
            return
        mf = info['max_fire']
        sdf = os.path.join(
            get_package_share_directory('ros_gz_example_description'),
            'models', f'forest_{mf}_ball',
            f'forest_{mf}_ball_state_{state}',
            'model.sdf'
        )
        x = self.offset_x + col*self.cell_size
        y = self.offset_y + (self.grid_rows-1-row)*self.cell_size
        name = f"forest_cell{row}_{col}"
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory(
            name=name, allow_renaming=False,
            sdf_filename=sdf, pose=Pose(), relative_to='world'
        )
        req.entity_factory.pose.position.x = x
        req.entity_factory.pose.position.y = y
        req.entity_factory.pose.position.z = 0.01
        self.spawn_cli.call_async(req)
        info.update(state=state, cstate=cstate, model_name=name, spawned=True)
        self.get_logger().info(f'Spawned {name} at {(row,col)}')
        time.sleep(0.2)

    def _publish_forest_info(self):
        arr = list(self.forest_info.values())
        msg = String()
        msg.data = json.dumps(arr)
        pub = self.create_publisher(String, 'forest_info', 10)
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InitForestOnce()
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
