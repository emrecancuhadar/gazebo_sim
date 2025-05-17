#!/usr/bin/env python3
import rclpy
import math
import time
import random
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
import json

from fire_weather_index import (
    calculate_ffmc,
    calculate_dmc,
    calculate_dc,
    calculate_isi,
    calculate_bui,
    calculate_fwi,
)

# --- Fire Cell Goal Client Node ---
class FireCellGoalClient(Node):
    def __init__(self):
        super().__init__('fire_cell_goal_client')
        
        # Subscribe to grid data for fire counts
        self.fire_count_sub = self.create_subscription(
            Float32MultiArray, "/grid/fire_count", self.fire_count_callback, 10
        )
        # Subscribe to grid data for fuel load (w)
        self.fuel_load_sub = self.create_subscription(
            Float32MultiArray, "/grid/fuel_load", self.fuel_load_callback, 10
        )
        # Subscribe to vegetation data (each cell is a string: sparse, bare, conifer, or deciduous)
        self.vegetation_sub = self.create_subscription(
            String, "/grid/vegetation", self.vegetation_callback, 10
        )
        # Subscribe to elevation, slope, and aspect (each as Float32MultiArray)
        self.elevation_sub = self.create_subscription(
            Float32MultiArray, "/grid/elevation", self.elevation_callback, 10
        )
        self.slope_sub = self.create_subscription(
            Float32MultiArray, "/grid/slope", self.slope_callback, 10
        )
        self.aspect_sub = self.create_subscription(
            Float32MultiArray, "/grid/aspect", self.aspect_callback, 10
        )
        # Subscribe to odometry to get current robot pose
        self.odom_sub = self.create_subscription(
            Odometry, "/diff_drive/odometry", self.odom_callback, 10)

        # — Instead of an action client, create a simple publisher —
        self.goal_pub = self.create_publisher(Float32MultiArray, "/fire_cell_goal", 10)

        self.create_subscription(String, '/fire_cell_done',
                                 self.done_callback, 10)

        

        self.blacklist = set()

        
        # State
        self.current_pose      = [0.0, 0.0]
        self.grid_data         = None
        self.fuel_load_data    = None
        self.vegetation_data   = None
        self.elevation_data    = None
        self.slope_data        = None
        self.aspect_data       = None
        self.last_update_time  = 0.0
        self.update_interval   = 1.0
        self.goal_active       = False
        self.current_goal      = [0.0, 0.0]
        self.prev_best_center  = None 

        # NEW: only dispatch a goal when extinguish finished
        self.ready_for_next = True
        self.just_extinguished = None

        # Parameters for grid dimensions and platform size (real-world)
        self.grid_rows               = self.declare_parameter("grid_rows", 10).value
        self.grid_cols               = self.declare_parameter("grid_cols", 10).value
        self.platform_width          = self.declare_parameter("platform_width", 20).value
        self.platform_height         = self.declare_parameter("platform_height", 20).value
        
        # Extra cell parameters (randomized for each cell // max and min values manually set):
        #  - wind_speed in m/s (0.0 to 50.0) according to Brauford scale.
        self.relative_humidity_min   = self.declare_parameter("relative_humidity_min", 0.0).value
        self.relative_humidity_max   = self.declare_parameter("relative_humidity_max", 100.0).value
        self.wind_speed_min          = self.declare_parameter("wind_speed_min", 0.0).value   # in m/s
        self.wind_speed_max          = self.declare_parameter("wind_speed_max", 50.0).value
        self.H_min                   = self.declare_parameter("H_min", 16000.0).value # kJ/kg
        self.H_max                   = self.declare_parameter("H_max", 20000.0).value

        # Global parameters for weighted algorithm (Manually Set):
        self.precipitation           = self.declare_parameter("precipitation", 1200.0).value
        self.precipitation_threshold = self.declare_parameter("precipitation_threshold", 1000.0).value
        self.temperature_celsius     = self.declare_parameter("temperature", 25.0).value
        self.temperature_threshold   = self.declare_parameter("temperature_threshold", 30.0).value
        self.wind_direction_param    = self.declare_parameter("wind_direction", "SE").value

        # Global vegetation multipliers based on your image processor classes.
        self.vegetation_factors = {
            'sparse': 1.0,
            'bare': 0.3,
            'conifer': 1.4,
            'deciduous': 0.8
        }

        # Weights for scoring factors (declared once here)
        self.weight_distance       = self.declare_parameter("weight_distance",     0.4).value
        self.weight_flammability   = self.declare_parameter("weight_flammability", 0.15).value
        self.weight_fire_intensity = self.declare_parameter("weight_fire_intensity", 0.25).value
        self.weight_wind_fuel      = self.declare_parameter("weight_wind_fuel",      0.1).value
        self.weight_wind_speed     = self.declare_parameter("weight_wind_speed",     0.1).value
        self.weight_vpd            = self.declare_parameter("weight_vpd",            0.15).value
        self.weight_elevation      = self.declare_parameter("weight_elevation",      0.1).value
        self.weight_slope          = self.declare_parameter("weight_slope",          0.2).value
        self.weight_aspect         = self.declare_parameter("weight_aspect",         0.15).value
        self.weight_fwi            = self.declare_parameter("weight_fwi",            0.2).value

        # === OPTIMIZATION: make the “previous” FWI params configurable ===
        self.prev_ffmc_default = self.declare_parameter("prev_ffmc", 85.0).value   # used to seed FFMC
        self.prev_dmc_default  = self.declare_parameter("prev_dmc", 6.0).value     # seed for DMC
        self.prev_dc_default   = self.declare_parameter("prev_dc", 15.0).value    # seed for DC

        # === OPTIMIZATION: Pre-compute grid cell centers once ===
        cell_w = self.platform_width / self.grid_cols
        cell_h = self.platform_height / self.grid_rows
        x_centers = (np.arange(self.grid_cols) + 0.5) * cell_w
        y_centers = (np.arange(self.grid_rows) + 0.5) * cell_h
        xg, yg = np.meshgrid(x_centers, y_centers)
        self.x_grid = xg - self.platform_width/2.0
        self.y_grid = self.platform_height/2.0 - yg

        # === OPTIMIZATION: Initialize cell-unique H array and global wind/RH ===
        self.H_arr          = np.random.uniform(self.H_min, self.H_max, size=(self.grid_rows, self.grid_cols))
        self.wind_speed_val = random.uniform(self.wind_speed_min, self.wind_speed_max)
        self.rh_val         = random.uniform(self.relative_humidity_min, self.relative_humidity_max)

        # === RE-ADDED LOGGING: initial cell parameters ===
        self.log_initial_cell_parameters()

        self.get_logger().info("Fire Cell Goal Client initialized.")

    def odom_callback(self, odom_msg):
        # Update current robot pose.
        self.current_pose[0] = odom_msg.pose.pose.position.x
        self.current_pose[1] = odom_msg.pose.pose.position.y

    def fire_count_callback(self, msg: Float32MultiArray):
        self.grid_data = msg.data
        arr = np.array(self.grid_data).reshape(self.grid_rows, self.grid_cols)

        # 1) If our current_goal cell has gone to zero, we need to replan:
        if self.current_goal is not None:
            # find its (r,c) in the grid
            idx = np.argwhere(
                (self.x_grid == self.current_goal[0]) &
                (self.y_grid == self.current_goal[1])
            )
            if idx.size:
                r, c = idx[0]
                if arr[r, c] == 0.0:
                    self.get_logger().info(
                        f"Current target cell {(r,c)} just burnt out → re‐planning."
                    )
                    # blacklist it so we never pick it again
                    self.blacklist.add((r, c))
                    # pretend we just finished extinguishing so we unlock planning
                    self.goal_active     = False
                    self.ready_for_next  = True
                    self.current_goal    = None

        # 2) Your existing “just_extinguished” logic (robot‐callback un‐blacklisting)
        if self.just_extinguished is not None:
            r, c = self.just_extinguished
            if arr[r, c] == 0.0:
                self.get_logger().info(f"Grid cell {(r,c)} cleared — un‐blacklisting")
                self.blacklist.discard((r, c))
                self.just_extinguished = None
                self.ready_for_next    = True

        # now try to pick a brand-new or “closer” goal
        self.try_process_grid()


    def fuel_load_callback(self, msg: Float32MultiArray):
        # Store fuel load (w) data (flat list in row-major order)
        self.fuel_load_data = msg.data
        self.try_process_grid()

    def vegetation_callback(self, msg: String):
        # Store vegetation data (flat list of strings in row-major order)
        # msg.data is a JSON-encoded flat list of 100 strings
        try:
            veg_list = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse vegetation JSON: {e}")
            return
        if len(veg_list) != self.grid_rows * self.grid_cols:
            self.get_logger().warn(f"Expected {self.grid_rows*self.grid_cols} veg labels, got {len(veg_list)}")
        self.vegetation_data = veg_list
        self.try_process_grid()

    def elevation_callback(self, msg: Float32MultiArray):
        # Store elevation data (flat list in row-major order)
        self.elevation_data = msg.data
        self.try_process_grid()

    def slope_callback(self, msg: Float32MultiArray):
        # Store slope data (flat list in row-major order)
        self.slope_data = msg.data
        self.try_process_grid()

    def aspect_callback(self, msg: Float32MultiArray):
        # Store aspect data (flat list in row-major order)
        self.aspect_data = msg.data
        self.try_process_grid()

    def try_process_grid(self):
        # Process grid if all required data streams are available.
        if (self.grid_data is not None and 
            self.fuel_load_data is not None and 
            self.vegetation_data is not None and
            self.elevation_data is not None and
            self.slope_data is not None and
            self.aspect_data is not None):
            current_time = time.time()
            if current_time - self.last_update_time >= self.update_interval:
                self.last_update_time = current_time
                self.process_grid_and_send_goal()
                
    def get_wind_vector(self, wind_direction):
        dirs = {
            'N':  (0, -1), 'NE': (math.sqrt(2)/2, -math.sqrt(2)/2),
            'E':  (1, 0),  'SE': (math.sqrt(2)/2, math.sqrt(2)/2),
            'S':  (0, 1),  'SW': (-math.sqrt(2)/2, math.sqrt(2)/2),
            'W':  (-1, 0), 'NW': (-math.sqrt(2)/2, -math.sqrt(2)/2)
        }
        return dirs.get(wind_direction.upper(), (0, -1))

    def calculate_vpd(self, temp_c, rh_pct):
        svp = 0.61078 * math.exp((17.2694 * temp_c) / (temp_c + 237.3))
        ea  = svp * (1 - rh_pct / 100.0)
        return svp - ea


    def log_initial_cell_parameters(self):
        try:
            with open("initial_cell_parameters.log", "w") as f:
                for r in range(self.grid_rows):
                    for c in range(self.grid_cols):
                        f.write(f"Cell({r},{c}): RH={self.rh_val:.2f}%, "
                                f"WS={self.wind_speed_val:.2f}m/s, "
                                f"H={self.H_arr[r,c]:.2f}\n")
            self.get_logger().info("Initial cell parameters logged.")
        except Exception as e:
            self.get_logger().error(f"Failed to log initial params: {e}")

    def log_scoring_parameters(self, candidate_cells, rainfall_factor, temperature_factor):
        """Log all variables affecting score to a file for review."""
        try:
            cell = candidate_cells[0]  # Assuming only one candidate cell for logging
            with open("scoring_parameters.log", "a") as f:
                f.write("Scoring Parameters for Candidate Cells:\n")
                f.write(f"Cell at {cell['center']}:\n")
                f.write(f"  Fire Intensity: {cell['fire_intensity']:.2f}\n")
                f.write(f"  Heat Yield (H): {cell['H']:.2f} kJ/kg\n")
                f.write(f"  Fuel Load (w): {cell['w']:.2f} kg/m²\n")
                f.write(f"  fire_count (r): {cell['r']}\n")
                f.write(f"  Distance: {cell['distance']:.2f} m\n")
                f.write(f"  Relative Humidity: {cell['relative_humidity']:.2f}%\n")
                f.write(f"  Vegetation: {cell['vegetation']}\n")
                f.write(f"  Veg Factor: {self.vegetation_factors.get(cell['vegetation'], 1.0):.2f}\n")
                f.write(f"  Flammability: {cell['flammability']:.2f}\n")
                f.write(f"  Wind Speed: {cell['wind_speed']:.2f} m/s\n")
                f.write(f"  Wind Fuel Sum: {cell.get('wind_fuel_sum', 0):.2f} kg/m²\n")
                f.write(f"  VPD: {cell['vpd']:.3f} kPa\n")
                f.write(f"  Elevation Risk: {cell['elevation_risk']:.2f}\n")
                f.write(f"  Slope Risk: {cell['slope_risk']:.2f}\n")
                f.write(f"  Aspect Risk: {cell['aspect_risk']:.2f}\n")
                f.write(f"  FWI: {cell['fwi']:.2f}\n")
                f.write(f"  Score: {cell['score']:.2f}\n\n")
                f.write(f"Global Rainfall Factor: {rainfall_factor}\n")
                f.write(f"Global Temperature Factor: {temperature_factor}\n\n")
            self.get_logger().info("Scoring parameters logged to 'scoring_parameters.log'.")
        except Exception as e:
            self.get_logger().error(f"Failed to log scoring parameters: {e}")

    def process_grid_and_send_goal(self):
        if not any([self.grid_data, self.fuel_load_data, self.vegetation_data,
                    self.elevation_data, self.slope_data, self.aspect_data]):
            return

        # Convert flat lists to 2D NumPy arrays.
        fire_counts        = np.array(self.grid_data).reshape((self.grid_rows, self.grid_cols))
        fuel_load          = np.array(self.fuel_load_data).reshape((self.grid_rows, self.grid_cols))
        vegetation_array   = np.array(self.vegetation_data).reshape((self.grid_rows, self.grid_cols))
        elev               = np.array(self.elevation_data).reshape((self.grid_rows, self.grid_cols))
        slope              = np.array(self.slope_data).reshape((self.grid_rows, self.grid_cols))
        aspect             = np.array(self.aspect_data).reshape((self.grid_rows, self.grid_cols))

        mask = fire_counts > 0
        for (br, bc) in self.blacklist:
            mask[br, bc] = False
        if not np.any(mask):
            self.get_logger().info("No active fire cells.")
            return

        # Compute distances from current robot pose (vectorized).
        robot_x, robot_y = self.current_pose
        distances = np.hypot(self.x_grid - robot_x, self.y_grid - robot_y)
        fire_intensity = self.H_arr * fuel_load * fire_counts

        # --- Wind-fuel sum per candidate ---
        wind_fuel_sum = np.zeros_like(fuel_load)
        if self.wind_speed_val >= 17.2:
            wx, wy = self.get_wind_vector(self.wind_direction_param)
            for r, c in zip(*np.where(mask)):
                dx = self.x_grid - self.x_grid[r, c]
                dy = self.y_grid - self.y_grid[r, c]
                norms = np.hypot(dx, dy) + 1e-6
                dots = (dx/norms)*wx + (dy/norms)*wy
                wind_fuel_sum[r, c] = float(np.sum(fuel_load[(dots > 0.7071) & (norms > 1e-6)]))

        # --- Compute FWI once ---
        month = time.localtime().tm_mon
        ffmc = calculate_ffmc(self.prev_ffmc_default, self.temperature_celsius, self.rh_val, self.wind_speed_val, self.precipitation)
        dmc  = calculate_dmc(self.prev_dmc_default, self.temperature_celsius, self.rh_val, self.precipitation, month)
        dc   = calculate_dc(self.prev_dc_default, self.temperature_celsius, self.precipitation, month)
        isi  = calculate_isi(ffmc, self.wind_speed_val)
        bui  = calculate_bui(dmc, dc)
        fwi  = calculate_fwi(isi, bui)
        self.get_logger().info(f"FWI={fwi:.2f} (FFMC={ffmc:.2f}, DMC={dmc:.2f}, DC={dc:.2f}, ISI={isi:.2f}, BUI={bui:.2f})")

       # --- Normalization constants ---
        eps                = 1e-6
        max_distance       = distances[mask].max()
        max_fire_intensity = fire_intensity[mask].max()
        max_wind_fuel_sum  = wind_fuel_sum[mask].max() if np.any(wind_fuel_sum[mask]) else 1.0
        max_w              = fuel_load.max()
        vpd_val            = self.calculate_vpd(self.temperature_celsius, self.rh_val)

        # --- Normalized arrays ---
        normalized_distance       = distances / (max_distance + eps)
        normalized_fire_intensity = fire_intensity / (max_fire_intensity + eps)
        normalized_w             = fuel_load / (max_w + eps)
        normalized_wind_fuel_sum = wind_fuel_sum / (max_wind_fuel_sum + eps)
        normalized_wind_speed     = self.wind_speed_val / (self.wind_speed_max + eps)
        normalized_vpd            = vpd_val / (vpd_val + eps)

        # --- Vegetation factors & flammability ---
        veg_factors  = np.vectorize(lambda v: self.vegetation_factors.get(v, 1.0))(vegetation_array)
        flammability = np.minimum(
            1.0,
            veg_factors * (0.4 * normalized_vpd + 0.4 * normalized_w + 0.2 * normalized_wind_speed)
        )

        # --- Topographic risk factors ---
        min_e, max_e         = elev.min(), elev.max()
        norm_elev_risk       = 1.0 - (elev - min_e) / (max_e - min_e + eps)
        norm_slope_risk      = np.minimum(slope / 90.0, 1.0)
        aspect_rad           = np.deg2rad(aspect - 180.0)
        norm_aspect_risk     = (1.0 + np.cos(aspect_rad)) / 2.0


       # --- Weights and dynamic wind-speed weight ---
        weights = {
            'distance':       self.weight_distance,
            'flammability':   self.weight_flammability,
            'fire_intensity': self.weight_fire_intensity,
            'wind_fuel':      self.weight_wind_fuel,
            'wind_speed':     self.weight_wind_speed,
            'vpd':            self.weight_vpd,
            'elevation':      self.weight_elevation,
            'slope':          self.weight_slope,
            'aspect':         self.weight_aspect
        }

        global_wind_speed = self.wind_speed_val
        if global_wind_speed < 10.8:
            global_ws_weight = weights['wind_speed']
        elif global_wind_speed <= 17.1:
            global_ws_weight = 0.13
        elif global_wind_speed <= 24.4:
            global_ws_weight = 0.17
        elif global_wind_speed <= 32.6:
            global_ws_weight = 0.21
        else:
            global_ws_weight = 0.25

        # --- Global modifiers ---
        rainfall_factor    = 0.95 if self.precipitation > self.precipitation_threshold else 1.0
        temperature_factor = 1.3  if self.temperature_celsius >= self.temperature_threshold else 1.0
        final_ws_weight    = global_ws_weight

        # --- Score computation ---
        base = (
            weights['distance']      * (1 - normalized_distance) +
            weights['flammability']  * flammability                +
            final_ws_weight          * normalized_wind_speed       +
            weights['vpd']           * normalized_vpd              +
            weights['fire_intensity']* normalized_fire_intensity   +
            weights['wind_fuel']     * normalized_wind_fuel_sum    +
            weights['elevation']     * norm_elev_risk              +
            weights['slope']         * norm_slope_risk             +
            weights['aspect']        * norm_aspect_risk
        )
        score = base * (rainfall_factor * temperature_factor) + self.weight_fwi * (fwi / 100.0)

        # --- Identify best cell ---
        masked_scores = np.where(mask, score, -np.inf)
        best_idx      = int(np.argmax(masked_scores))
        br, bc        = np.unravel_index(best_idx, score.shape)
        new_goal      = (self.x_grid[br, bc], self.y_grid[br, bc])

        # --- Build best_cell dict for detailed logging ---
        best_cell = {
            'center':            new_goal,
            'score':             float(score[br, bc]),
            'fire_intensity':    float(fire_intensity[br, bc]),
            'H':                 float(self.H_arr[br, bc]),
            'w':                 float(fuel_load[br, bc]),
            'r':                 int(fire_counts[br, bc]),
            'distance':          float(distances[br, bc]),
            'wind_speed':        float(self.wind_speed_val),
            'wind_fuel_sum':     float(wind_fuel_sum[br, bc]),
            'flammability':      float(flammability[br, bc]),
            'vpd':               float(vpd_val),
            'relative_humidity': float(self.rh_val),
            'vegetation':        vegetation_array[br, bc],
            'veg_factor':        float(veg_factors[br, bc]),
            'elevation_risk':    float(norm_elev_risk[br, bc]),
            'slope_risk':        float(norm_slope_risk[br, bc]),
            'aspect_risk':       float(norm_aspect_risk[br, bc]),
            'fwi':               float(fwi),
        }
        
        if new_goal != self.prev_best_center:
            self.log_scoring_parameters([best_cell], rainfall_factor, temperature_factor)
            self.prev_best_center = new_goal

        # === DETAILED LOGGING LIKE REQUESTED ===
        self.get_logger().info(
            f"Candidate cell at {new_goal} with final score {best_cell['score']:.2f}:\n"
            f"  fire_intensity {best_cell['fire_intensity']:.2f} kJ/m², "
            f"H {best_cell['H']:.2f} kJ/kg, "
            f"w {best_cell['w']:.2f} kg/m², "
            f"r {best_cell['r']}, "
            f"distance {best_cell['distance']:.2f} m, "
            f"wind_speed {best_cell['wind_speed']:.2f} m/s (global weight {final_ws_weight}), "
            f"wind_fuel_sum {best_cell['wind_fuel_sum']:.2f} kg/m², "
            f"flammability {best_cell['flammability']:.2f}, "
            f"VPD {best_cell['vpd']:.3f} kPa, "
            f"elevation_risk {best_cell['elevation_risk']:.2f}, "
            f"slope_risk {best_cell['slope_risk']:.2f}, "
            f"aspect_risk {best_cell['aspect_risk']:.2f}, "
            f"fwi {best_cell['fwi']:.2f}, "
            f"temperature {self.temperature_celsius:.2f}°C, "
            f"rainfall_factor {rainfall_factor} (precip {self.precipitation:.2f}), "
            f"temperature_factor {temperature_factor}"
        )


        # If a goal is already active, compare distances to decide whether to update the goal.
        if self.goal_active:
            current_goal_distance = math.hypot((self.current_goal[0] - robot_x), 
                                              (self.current_goal[1] - robot_y))
            new_goal_distance = math.hypot((new_goal[0] - robot_x),
                                          (new_goal[1] - robot_y))
            if new_goal_distance < current_goal_distance * 0.8:
                self.get_logger().info(f"New closer cell detected at {new_goal}. Redirecting goal.")
                self.current_goal = new_goal
                self.send_goal(*new_goal)
            else:
                self.get_logger().info("Current goal remains the best candidate.")
        else:
            self.current_goal = new_goal
            self.send_goal(*new_goal)

    def send_goal(self, x, y):
        msg = Float32MultiArray()
        msg.data = [float(x), float(y)]
        self.goal_active     = True
        self.ready_for_next = False  
        self.get_logger().info(f"Sending goal to cell center at x={x:.2f}, y={y:.2f}")
        self.goal_pub.publish(msg)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            self.goal_active = False
            return
        self.get_logger().info("Goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.goal_reached:
            self.get_logger().info("Goal reached successfully.")
        else:
            self.get_logger().info("Failed to reach goal.")
        self.goal_active = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: Current position ({feedback.current_x}, {feedback.current_y}), Distance: {feedback.distance}"
        )

    def done_callback(self, msg):
        x, y = map(float, msg.data.split(','))
        idx = np.argwhere((self.x_grid==x) & (self.y_grid==y))
        if idx.size:
            r,c = idx[0]
            self.blacklist.add((int(r), int(c)))
            self.just_extinguished = (r, c)

        self.ready_for_next = False
        self.goal_active = False
        self.get_logger().info(f"Extinguish done: ({x:.2f},{y:.2f}); waiting for grid zero")


def main(args=None):
    rclpy.init(args=args)
    node = FireCellGoalClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()