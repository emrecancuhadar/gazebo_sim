#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time, os, subprocess
from subprocess import CalledProcessError, TimeoutExpired
from ament_index_python.packages import get_package_share_directory

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
        # ----------------------------------
        # Define your “center” fires
        centers = [
            #(row, col, fire_strength),
            (3, 3, 1),  
            (9, 9, 1)
        ]

        # Build a map from (row,col) -> (state, cstate)
        spawn_map = {}
        # Phase 1: schedule the centers
        for i, j, fv in centers:
            cs = (fv - 1) * 200 + 1
            spawn_map[(i, j)] = (fv, cs)

        # Phase 2: schedule *only* healthy neighbours
        for i, j, _ in centers:
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    ni, nj = i + di, j + dj
                    if (ni, nj) == (i, j):
                        continue
                    if not (0 <= ni < self.grid_rows and 0 <= nj < self.grid_cols):
                        continue

                    nbr = self.forest_info[(ni, nj)]
                    # only schedule if truly healthy
                    if nbr['cstate'] == 0:
                        # state=0, cstate=0 for a “cold” neighbour
                        spawn_map.setdefault((ni, nj), (0, 0))

        # Now do *one* pass over spawn_map and actually spawn each cell
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
        info=self.forest_info[(row,col)]
        if info['spawned']: return
        mf=info['max_fire']
        sdf=os.path.join(self.share_dir,'models',f'forest_{mf}_ball',f'forest_{mf}_ball_state_{state}','model.sdf')
        x=self.offset_x+col*self.cell_size
        y=self.offset_y+(self.grid_rows-1-row)*self.cell_size
        name=f"forest_cell{row}_{col}_{int(time.time()*1000)}"
        cmd=["ros2","run","ros_gz_sim","create","-demo","default","-file",sdf,
             "-x",str(x),"-y",str(y),"-z","0.01","-name",name]
        try:
            self.run_cmd_with_retry(cmd)
            info.update(state=state,cstate=cstate,model_name=name,spawned=True)
            self.forest_info[(row,col)]=info
            self.get_logger().info(f"Spawned cell {name} at {(row,col)} state {state}")
            time.sleep(0.3)
        except Exception as e:
            self.get_logger().error(f"Spawn cell {name} failed: {e}")

    def run_cmd_with_retry(self, cmd, timeout=4.0, retries=3):
        last=None
        for i in range(1,retries+1):
            try:
                return subprocess.run(cmd,stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE,check=True,
                                      text=True,timeout=timeout)
            except (TimeoutExpired,CalledProcessError) as e:
                last=e;self.get_logger().warn(f"[retry {i}/{retries}] {e}");time.sleep(0.5)
        raise last

    def publish_forest_info(self):
        arr=list(self.forest_info.values())
        msg=String();msg.data=json.dumps(arr)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    n=InitForest();rclpy.spin_once(n,timeout_sec=2);rclpy.shutdown()

if __name__=='__main__':
    main()