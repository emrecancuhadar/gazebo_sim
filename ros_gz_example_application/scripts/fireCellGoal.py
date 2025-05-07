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

# --- FWI Calculation Functions ---
def calculate_ffmc(prev_ffmc, temperature, relative_humidity, wind_speed, rain):
    mo = 147.2 * (101 - prev_ffmc) / (59.5 + prev_ffmc)
    if rain > 0.5:
        rf = rain - 0.5
        if mo > 150:
            mo += 42.5 * rf * math.exp(-100 / (251 - mo)) * (1 - math.exp(-6.93 / rf)) + 0.0015 * (mo - 150)**2 * math.sqrt(rf)
        else:
            mo += 42.5 * rf * math.exp(-100 / (251 - mo)) * (1 - math.exp(-6.93 / rf))
        mo = min(mo, 250)
    ed = 0.942 * relative_humidity**0.679 + 11 * math.exp((relative_humidity - 100) / 10) + \
         0.18 * (21.1 - temperature) * (1 - math.exp(-0.115 * relative_humidity))
    if mo > ed:
        ko = 0.424 * (1 - (relative_humidity / 100)**1.7) + \
             0.0694 * math.sqrt(wind_speed) * (1 - (relative_humidity / 100)**8)
        kd = ko * 0.581 * math.exp(0.0365 * temperature)
        mo = ed + (mo - ed) * 10**(-kd)
    else:
        ew = 0.618 * relative_humidity**0.753 + 10 * math.exp((relative_humidity - 100) / 10) + \
             0.18 * (21.1 - temperature) * (1 - math.exp(-0.115 * relative_humidity))
        if mo < ew:
            ko = 0.424 * (1 - ((100 - relative_humidity) / 100)**1.7) + \
                 0.0694 * math.sqrt(wind_speed) * (1 - ((100 - relative_humidity) / 100)**8)
            kw = ko * 0.581 * math.exp(0.0365 * temperature)
            mo = ew - (ew - mo) * 10**(-kw)
    ffmc = (59.5 * (250 - mo)) / (147.2 + mo)
    return max(0, min(ffmc, 101))



def calculate_dmc(prev_dmc, temperature, relative_humidity, rain, month):
    if rain > 1.5:
        re = 0.92 * rain - 1.27
        mo = 20 + math.exp(5.6348 - prev_dmc / 43.43)
        if prev_dmc <= 33:
            b = 100 / (0.5 + 0.3 * prev_dmc)
        elif prev_dmc <= 65:
            b = 14 - 1.3 * math.log(prev_dmc)
        else:
            b = 6.2 * math.log(prev_dmc) - 17.2
        mr = mo + (1000 * re) / (48.77 + b * re)
        dmc = 244.72 - 43.43 * math.log(mr - 20)
        dmc = max(dmc, 0)
    else:
        dmc = prev_dmc
    le_table = [6.5, 7.5, 9.0, 12.8, 13.9, 13.9, 12.4, 10.9, 9.4, 8.0, 7.0, 6.0]
    le = le_table[month - 1]
    if temperature < -1.1:
        temperature = -1.1
    k = 1.894 * (temperature + 1.1) * (100 - relative_humidity) * le * 1e-6
    dmc += 100 * k
    return dmc

def calculate_dc(prev_dc, temperature, rain, month):
    if rain > 2.8:
        rd = 0.83 * rain - 1.27
        qo = 800 * math.exp(-prev_dc / 400)
        qr = qo + 3.937 * rd
        dc = 400 * math.log(800 / qr)
        dc = max(dc, 0)
    else:
        dc = prev_dc
    fl_table = [-1.6, -1.6, -1.6, 0.9, 3.8, 5.8, 6.4, 5.0, 2.4, 0.4, -1.6, -1.6]
    fl = fl_table[month - 1]
    if temperature < -2.8:
        temperature = -2.8
    v = 0.36 * (temperature + 2.8) + fl
    v = max(v, 0)
    dc += 0.5 * v
    return dc

def calculate_isi(ffmc, wind_speed):
    mo = 147.2 * (101 - ffmc) / (59.5 + ffmc)
    fF = 91.9 * math.exp(-0.1386 * mo) * (1 + (mo**5.31) / (4.93e7))
    fW = math.exp(0.05039 * wind_speed)
    return 0.208 * fW * fF

def calculate_bui(dmc, dc):
    if dmc <= 0.4 * dc:
        bui = (0.8 * dmc * dc) / (dmc + 0.4 * dc)
    else:
        bui = dmc - (1 - (0.8 * dc) / (dmc + 0.4 * dc)) * (0.92 + (0.0114 * dmc)**1.7)
    return max(bui, 0)

def calculate_fwi(isi, bui):
    if bui <= 80:
        fD = 0.626 * bui**0.809 + 2
    else:
        fD = 1000 / (25 + 108.64 * math.exp(-0.023 * bui))
    b = 0.1 * isi * fD
    if b <= 1:
        return b
    return math.exp(2.72 * (math.log(b) * 0.434) ** 0.647)

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
        # Subscribe to odometry to get current robot pose
        self.odom_sub = self.create_subscription(
            Odometry, "/diff_drive/odometry", self.odom_callback, 10)

        # — Instead of an action client, create a simple publisher —
        self.goal_pub = self.create_publisher(Float32MultiArray, "/fire_cell_goal", 10)

        self.create_subscription(String, '/fire_cell_done',
                                 self.done_callback, 10)

        

        self.blacklist = set()

        
        # State
        self.current_pose     = [0.0, 0.0]
        self.grid_data        = None
        self.fuel_load_data   = None
        self.vegetation_data  = None
        self.last_update_time = 0.0
        self.update_interval  = 1.0
        self.goal_active      = False
        self.current_goal     = [0.0, 0.0]
        self.prev_best_center = None

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
        self.wind_direction_param    = self.declare_parameter("wind_direction", "E").value
        self.wind_fuel_weight = 0.1  # Weight for the additional wind fuel effect:

        # Global vegetation multipliers based on your image processor classes.
        self.vegetation_factors = {
            'sparse': 1.0,
            'bare': 0.3,
            'conifer': 1.4,
            'deciduous': 0.8
        }

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

    def try_process_grid(self):
        # Process grid if fire count, fuel load, and vegetation data are available.
        if (self.grid_data is not None and 
            self.fuel_load_data is not None and 
            self.vegetation_data is not None):
            current_time = time.time()
            if current_time - self.last_update_time >= self.update_interval:
                self.last_update_time = current_time
                self.process_grid_and_send_goal()
                
    def get_wind_vector(self, wind_direction):
        """Return a unit vector (dx, dy) corresponding to the wind direction.
           Assumes wind_direction indicates the direction the wind is blowing TO.
           Coordinate system: x increases to the right, y increases downward.
        """
        dirs = {
            'N':  (0, -1), 'NE': (math.sqrt(2)/2, -math.sqrt(2)/2),
            'E':  (1, 0),  'SE': (math.sqrt(2)/2, math.sqrt(2)/2),
            'S':  (0, 1),  'SW': (-math.sqrt(2)/2, math.sqrt(2)/2),
            'W':  (-1, 0), 'NW': (-math.sqrt(2)/2, -math.sqrt(2)/2)
        }
        return dirs.get(wind_direction.upper(), (0, -1))

    def calculate_vpd(self, temperature_celsius, relative_humidity_percent):
            """
            Compute Vapor Pressure Deficit (VPD) in kPa.
            """
            svp = 0.61078 * math.exp((17.2694 * temperature_celsius) / (temperature_celsius + 237.3))  # saturation vapor pressure
            ea = svp * (1 - relative_humidity_percent / 100.0)  # actual vapor pressure
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
                f.write(f"  Score: {cell['score']:.2f}\n")
                f.write("\n")
                f.write(f"Global Rainfall Factor: {rainfall_factor}\n")
                f.write(f"Global Temperature Factor: {temperature_factor}\n")
            self.get_logger().info("Scoring parameters logged to 'scoring_parameters.log'.")
        except Exception as e:
            self.get_logger().error(f"Failed to log scoring parameters: {e}")

    def process_grid_and_send_goal(self):
        if not any([self.grid_data, self.fuel_load_data, self.vegetation_data]):
            return

        # Convert flat lists to 2D NumPy arrays.
        fire_counts = np.array(self.grid_data).reshape((self.grid_rows, self.grid_cols))
        fuel_load = np.array(self.fuel_load_data).reshape((self.grid_rows, self.grid_cols))
        vegetation_array = np.array(self.vegetation_data).reshape((self.grid_rows, self.grid_cols))

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
        ffmc = calculate_ffmc(85.0, self.temperature_celsius, self.rh_val, self.wind_speed_val, self.precipitation)
        dmc  = calculate_dmc(6.0, self.temperature_celsius, self.rh_val, self.precipitation, month)
        dc   = calculate_dc(15.0, self.temperature_celsius, self.precipitation, month)
        isi  = calculate_isi(ffmc, self.wind_speed_val)
        bui  = calculate_bui(dmc, dc)
        fwi  = calculate_fwi(isi, bui)
        self.get_logger().info(f"FWI={fwi:.2f} (FFMC={ffmc:.2f}, DMC={dmc:.2f}, DC={dc:.2f}, ISI={isi:.2f}, BUI={bui:.2f})")

        # --- Normalization constants ---
        eps   = 1e-6
        max_distance  = distances[mask].max()
        max_fire_intensity  = fire_intensity[mask].max()
        max_wind_fuel_sum = wind_fuel_sum[mask].max() if np.any(wind_fuel_sum[mask]) else 1.0
        max_w = fuel_load.max()
        vpd_val= self.calculate_vpd(self.temperature_celsius, self.rh_val)

        # --- Normalized arrays ---
        normalized_distance   = distances / (max_distance + eps)
        normalized_fire_intensity   = fire_intensity / (max_fire_intensity + eps)
        normalized_w  = fuel_load / (max_w + eps)
        normalized_wind_fuel_sum  = wind_fuel_sum / (max_wind_fuel_sum + eps)
        normalized_wind_speed  = self.wind_speed_val / (self.wind_speed_max + eps)
        normalized_vpd = vpd_val / (vpd_val + eps)

        # --- Vegetation factors & flammability ---
        veg_factors  = np.vectorize(lambda v: self.vegetation_factors.get(v, 1.0))(vegetation_array)
        flammability = np.minimum(
            1.0,
            veg_factors * (0.4 * normalized_vpd + 0.4 * normalized_w + 0.2 * normalized_wind_speed)
        )

        # --- Weights and dynamic wind-speed weight ---
        weights = {
            'distance':       0.4,
            'flammability':   0.15,
            'fire_intensity': 0.25,
            'wind_fuel':      self.wind_fuel_weight,
            'vpd':            0.15,
            'wind_speed':     0.1
        }

        global_wind_speed = self.wind_speed_val
        if global_wind_speed < 10.8: global_ws_weight = weights['wind_speed']
        elif global_wind_speed <= 17.1: global_ws_weight = 0.13
        elif global_wind_speed <= 24.4: global_ws_weight = 0.17
        elif global_wind_speed <= 32.6: global_ws_weight = 0.21
        else: global_ws_weight = 0.25

        # --- Global modifiers ---
        rainfall_factor    = 0.95 if self.precipitation > self.precipitation_threshold else 1.0
        temperature_factor = 1.3  if self.temperature_celsius >= self.temperature_threshold else 1.0
        fwi_weight         = 0.2
        final_ws_weight    = global_ws_weight

        # --- Score computation ---
        base = (
            weights['distance'] * (1 - normalized_distance) +
            weights['flammability'] * flammability +
            final_ws_weight * normalized_wind_speed +
            weights['vpd'] * normalized_vpd +
            weights['fire_intensity'] * normalized_fire_intensity +
            weights['wind_fuel'] * normalized_wind_fuel_sum
        )
        score = base * (rainfall_factor * temperature_factor) + fwi_weight * (fwi / 100.0)

        # --- Identify best cell ---
        masked_scores = np.where(mask, score, -np.inf)
        best_idx      = int(np.argmax(masked_scores))
        br, bc        = np.unravel_index(best_idx, score.shape)
        new_goal      = (self.x_grid[br, bc], self.y_grid[br, bc])

                # --- Build best_cell dict for detailed logging ---
        best_cell = {
            'center':          (new_goal),
            'score':           float(score[br, bc]),
            'fire_intensity':  float(fire_intensity[br, bc]),
            'H':               float(self.H_arr[br, bc]),
            'w':               float(fuel_load[br, bc]),
            'r':               int(fire_counts[br, bc]),
            'distance':        float(distances[br, bc]),
            'wind_speed':      float(self.wind_speed_val),
            'wind_fuel_sum':   float(wind_fuel_sum[br, bc]),
            'flammability':    float(flammability[br, bc]),
            'vpd':             float(vpd_val),
            'relative_humidity': float(self.rh_val),
            'vegetation': (vegetation_array[br, bc]),
            'veg_factor': (vegetation_array[br, bc])
        }
        
        if new_goal != self.prev_best_center:
            self.log_scoring_parameters([best_cell], rainfall_factor, temperature_factor)
            self.prev_best_center = new_goal

        # === DETAILED LOGGING LIKE REQUESTED ===
        self.get_logger().info(
            f"Candidate cell at {new_goal} with final score {best_cell['score']:.2f}:\n"
            f"fire_intensity {best_cell['fire_intensity']:.2f} kW/m,\n "
            f"low heat of combustion (H) {best_cell['H']:.2f} kJ/kg,\n "
            f"fuel_load (w) {best_cell['w']:.2f} kg/m²,\n "
            f"fire_count (r) {best_cell['r']} m/s,\n "
            f"distance {best_cell['distance']:.2f} m,\n "
            f"wind_speed {best_cell['wind_speed']:.2f} m/s (global weight {final_ws_weight}),\n "
            f"wind_fuel_sum {best_cell['wind_fuel_sum']:.2f} kg/m²,\n "
            f"Flammability {best_cell['flammability']:.2f},\n "
            f"VPD {best_cell['vpd']:.3f} kPa,\n "
            f"relative_humidity {best_cell['relative_humidity']:.2f}%,\n "
            f"temperature {self.temperature_celsius:.2f}°C,\n "
            f"rainfall_factor {rainfall_factor} with rainfall {self.precipitation:.2f} mm,\n "
            f"temperature_factor {temperature_factor} with temperature {self.temperature_celsius:.2f}°C"
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