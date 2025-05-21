#!/usr/bin/env python3
import rclpy
import math
import time
import random
import json
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry


# --- IMPORT FWI FUNCTIONS FROM NEW MODULE ===
from fire_weather_index import (
    calculate_ffmc,
    calculate_dmc,
    calculate_dc,
    calculate_isi,
    calculate_bui,
    calculate_fwi,
)
# --- Import Rothermel model functions ---
from rothermelmodel import (
    packing_ratio,
    optimum_packing_ratio,
    optimum_reaction_velocity,
    moisture_damping_coefficient,
    mineral_damping_coefficient,
    reaction_velocity,
    reaction_intensity,
    propagating_flux_ratio,
    wind_factor,
    slope_factor_from_angle,
    live_moisture_of_extinction,
    bulk_density,
    effective_heating_number,
    heat_of_preignition,
    rate_of_spread
)

class FireCellGoalClient(Node):
    def __init__(self):
        super().__init__('fire_cell_goal_client')

        # ─── Robot list & per-robot state ─────────────────────────────────
        self.robot_list = self.declare_parameter(
            'robot_list',
            ['diff_drive','diff_drive2','diff_drive3']
        ).value

        self.poses      = {ns: (0.0, 0.0) for ns in self.robot_list}
        self.ready      = {ns: True        for ns in self.robot_list}
        self.blacklists = {ns: set()       for ns in self.robot_list}

        # per-robot publishers & subscriptions
        self.goal_pubs = {}
        for ns in self.robot_list:
            # odom
            self.create_subscription(
                Odometry,
                f'/{ns}/odometry',
                lambda msg, ns=ns: self.odom_callback(msg, ns),
                10
            )
            # done-extinguish
            self.create_subscription(
                String,
                f'/{ns}/fire_cell_done',
                lambda msg, ns=ns: self.done_callback(msg, ns),
                10
            )
            # goal pub
            self.goal_pubs[ns] = self.create_publisher(
                Float32MultiArray,
                f'/{ns}/fire_cell_goal',
                10
            )

        # ─── Global grid data subscriptions ──────────────────────────────
        self.fire_count_sub = self.create_subscription(
            Float32MultiArray, "/grid/fire_count", self.fire_count_callback, 10
        )
        self.fuel_load_sub  = self.create_subscription(
            Float32MultiArray, "/grid/fuel_load",  self.fuel_load_callback, 10
        )
        self.vegetation_sub = self.create_subscription(
            String, "/grid/vegetation", self.vegetation_callback, 10
        )
        self.elevation_sub  = self.create_subscription(
            Float32MultiArray, "/grid/elevation", self.elevation_callback, 10
        )
        self.slope_sub      = self.create_subscription(
            Float32MultiArray, "/grid/slope",     self.slope_callback, 10
        )
        self.aspect_sub     = self.create_subscription(
            Float32MultiArray, "/grid/aspect",    self.aspect_callback, 10
        )

        # ─── State for grid & timing ──────────────────────────────────────
        self.grid_data       = None
        self.fuel_load_data  = None
        self.vegetation_data = None
        self.elevation_data  = None
        self.slope_data      = None
        self.aspect_data     = None

        self.last_update_time = 0.0
        self.update_interval  = 1.0
        self.prev_best_center = {ns: None for ns in self.robot_list}

        # ─── Parameters for grid dims & cell centers ────────────────────
        self.grid_rows      = self.declare_parameter("grid_rows", 10).value
        self.grid_cols      = self.declare_parameter("grid_cols", 10).value
        self.platform_width = self.declare_parameter("platform_width", 20.0).value
        self.platform_height= self.declare_parameter("platform_height",20.0).value

        cell_w = self.platform_width  / self.grid_cols
        cell_h = self.platform_height / self.grid_rows
        xs = (np.arange(self.grid_cols) + 0.5) * cell_w
        ys = (np.arange(self.grid_rows) + 0.5) * cell_h
        xg, yg = np.meshgrid(xs, ys)
        self.x_grid = xg - self.platform_width/2.0
        self.y_grid = self.platform_height/2.0 - yg
        
        # Extra cell parameters (randomized for each cell // max and min values manually set):
        #  - wind_speed in ft/min (0.0 to 9842.52) according to Brauford scale.
        self.relative_humidity_min   = self.declare_parameter("relative_humidity_min", 0.0).value
        self.relative_humidity_max   = self.declare_parameter("relative_humidity_max", 100.0).value
        self.wind_speed_min          = self.declare_parameter("wind_speed_min", 0.0).value   # in ft/min
        self.wind_speed_max          = self.declare_parameter("wind_speed_max", 9842.52).value
        self.H_min                   = self.declare_parameter("H_min", 6878.76).value # Btu/lb
        self.H_max                   = self.declare_parameter("H_max", 8598.45).value

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
        self.weight_flammability   = self.declare_parameter("weight_flammability", 0.18).value
        self.weight_fire_intensity = self.declare_parameter("weight_fire_intensity", 0.23).value
        self.weight_wind_fuel      = self.declare_parameter("weight_wind_fuel",      0.1).value
        self.weight_wind_speed     = self.declare_parameter("weight_wind_speed",     0.1).value
        self.weight_vpd            = self.declare_parameter("weight_vpd",            0.1).value
        self.weight_elevation      = self.declare_parameter("weight_elevation",      0.03).value
        self.weight_aspect         = self.declare_parameter("weight_aspect",         0.06).value
        self.weight_fwi            = self.declare_parameter("weight_fwi",            0.15).value

        # Rothermel input parameters (tunable per cell)
        self.sigma            = self.declare_parameter("sigma", 2000.0).value       # ft^2/ft^3
        self.rho_p            = self.declare_parameter("rho_p", 32.0).value         # lb/ft^3
        self.delta            = self.declare_parameter("delta", 0.5).value          # ft
        # dead‐fuel moisture & extinction
        self.M_dead           = self.declare_parameter("moisture_dead",     0.07).value
        self.Mx_dead          = self.declare_parameter("moisture_ext_dead", 0.20).value
        # live‐fuel moisture & extinction
        self.M_live           = self.declare_parameter("moisture_live",     1.50).value
        self.Mx_live          = self.declare_parameter("moisture_ext_live", 2.00).value
        self.S_t              = self.declare_parameter("S_t", 0.0555).value         # lb minerals / lb wood     
        s_e_value             = (1.01 * self.S_t) - 0.05  
        self.S_e              = self.declare_parameter("S_e", s_e_value).value           # lb minerals – lb silica / lb wood

        # === OPTIMIZATION: make the “previous” FWI params configurable ===
        self.prev_ffmc_default = self.declare_parameter("prev_ffmc", 85.0).value   # used to seed FFMC
        self.prev_dmc_default  = self.declare_parameter("prev_dmc", 6.0).value     # seed for DMC
        self.prev_dc_default   = self.declare_parameter("prev_dc", 15.0).value    # seed for DC

        # ─── Randomized H & global wind/humidity ─────────────────────────
        self.H_arr          = np.random.uniform(self.H_min, self.H_max,
                                                size=(self.grid_rows, self.grid_cols))
        self.wind_speed_val = random.uniform(self.wind_speed_min, self.wind_speed_max)
        self.rh_val         = random.uniform(self.relative_humidity_min,
                                             self.relative_humidity_max)

        self.log_initial_cell_parameters()
        self.get_logger().info("Fire Cell Goal Client initialized for robots: " +
                               ", ".join(self.robot_list))

    # ─── Odometry callback ─────────────────────────────────────────────
    def odom_callback(self, msg, ns):
        self.poses[ns] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    # ─── Grid callbacks ───────────────────────────────────────────────
    def fire_count_callback(self, msg):
        self.grid_data = msg.data
        self.maybe_dispatch()

    def fuel_load_callback(self, msg):
        self.fuel_load_data = msg.data
        self.maybe_dispatch()

    def vegetation_callback(self, msg):
        try:
            self.vegetation_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"veg JSON parse failed: {e}")
            return
        self.maybe_dispatch()

    def elevation_callback(self, msg):
        self.elevation_data = msg.data
        self.maybe_dispatch()

    def slope_callback(self, msg):
        self.slope_data = msg.data
        self.maybe_dispatch()

    def aspect_callback(self, msg):
        self.aspect_data = msg.data
        self.maybe_dispatch()

    # ─── Throttle & dispatch per-robot ─────────────────────────────────
    def maybe_dispatch(self):
        if None in (
            self.grid_data, self.fuel_load_data, self.vegetation_data,
            self.elevation_data, self.slope_data, self.aspect_data
        ):
            return

        now = time.time()
        if now - self.last_update_time < self.update_interval:
            return
        self.last_update_time = now

        for ns in self.robot_list:
            if self.ready[ns]:
                self.process_grid_and_send_goal(ns)

    # ─── Compute & send one goal for robot `ns` ────────────────────────
    def process_grid_and_send_goal(self, ns):
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

        #––– Core Rothermel terms per cell (heterogeneous fuel) –––#
        # 1) compute updated Mx_live via Albini
        self.Mx_live = live_moisture_of_extinction(
            w_dead = fuel_load,
            sigma_dead = self.sigma,
            w_live = fuel_load,       # replace with live-specific load if available
            sigma_live = self.sigma,  # replace with live-specific SAV if available
            Mf_dead = self.M_dead,
            Mx_dead = self.Mx_dead
        )

        # 2) dead‐fuel reaction‐intensity
        β_dead   = packing_ratio(fuel_load, self.rho_p, self.delta)
        βop_dead = optimum_packing_ratio(self.sigma)
        Γp_dead  = optimum_reaction_velocity(self.sigma, β_dead, βop_dead)
        ηM_dead  = moisture_damping_coefficient(self.M_dead, self.Mx_dead)
        ηs_dead  = mineral_damping_coefficient(self.S_e)
        Γ_dead   = reaction_velocity(Γp_dead, ηM_dead, ηs_dead)
        IR_dead  = reaction_intensity(Γ_dead, fuel_load, self.H_arr)

        # 3) live‐fuel reaction‐intensity
        β_live   = β_dead       # or packing_ratio(fuel_load_live, …)
        βop_live = βop_dead     # or optimum_packing_ratio(σ_live)
        Γp_live  = optimum_reaction_velocity(self.sigma, β_live, βop_live)
        ηM_live  = moisture_damping_coefficient(self.M_live, self.Mx_live)
        ηs_live  = ηs_dead      # or separate mineral_damping_coefficient(self.S_e_live)
        Γ_live   = reaction_velocity(Γp_live, ηM_live, ηs_live)
        IR_live  = reaction_intensity(Γ_live, fuel_load, self.H_arr)

        # 4) total heat‐source term
        I_R       = IR_dead + IR_live

        # 5) rest of spread algorithm unchanged
        xi        = propagating_flux_ratio(self.sigma, β_dead)
        phi_w     = wind_factor(self.sigma, β_dead, βop_dead, self.wind_speed_val)
        phi_s     = slope_factor_from_angle(β_dead, slope)
        rho_b     = bulk_density(fuel_load, self.delta)
        eps       = effective_heating_number(self.sigma)
        Q_ig      = heat_of_preignition(self.M_dead)
        R_arr     = rate_of_spread(I_R, xi, rho_b, eps, Q_ig, phi_w, phi_s)



        mask = fire_counts > 0
        for (r,c) in self.blacklists[ns]:
            mask[r,c] = False
        if not mask.any():
            self.get_logger().info(f"[{ns}] No active fire cells.")
            return

        rx, ry    = self.poses[ns]
        distances = np.hypot(self.x_grid - rx, self.y_grid - ry)
        fire_intensity = (self.H_arr * 2.326) * fuel_load * R_arr * fire_counts

        # wind-fuel sum (same as before)…
        wind_fuel_sum = np.zeros_like(fuel_load)
        if self.wind_speed_val >= 3385.82:
            wx, wy = self.get_wind_vector(self.wind_direction_param)
            for r, c in zip(*np.where(mask)):
                dx = self.x_grid - self.x_grid[r,c]
                dy = self.y_grid - self.y_grid[r,c]
                norms = np.hypot(dx, dy) + 1e-6
                dots  = (dx/norms)*wx + (dy/norms)*wy
                wind_fuel_sum[r,c] = float(np.sum(fuel_load[(dots>0.7071)&(norms>1e-6)]))

        # --- Compute FWI once ---
        wind_ft_per_min = self.wind_speed_val
        wind_kmh = wind_ft_per_min * 0.018288
        month = time.localtime().tm_mon
        ffmc = calculate_ffmc(self.prev_ffmc_default, self.temperature_celsius, self.rh_val, wind_kmh, self.precipitation)
        dmc  = calculate_dmc(self.prev_dmc_default, self.temperature_celsius, self.rh_val, self.precipitation, month)
        dc   = calculate_dc(self.prev_dc_default, self.temperature_celsius, self.precipitation, month)
        isi  = calculate_isi(ffmc, wind_kmh)
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
        normalized_w              = fuel_load / (max_w + eps)
        normalized_wind_fuel_sum  = wind_fuel_sum / (max_wind_fuel_sum + eps)
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
        slope_factor         = phi_s
        aspect_rad           = np.deg2rad(aspect - 180.0)
        norm_aspect_risk     = (1.0 + np.cos(aspect_rad)) / 2.0
        
        final_ws_weight = self.weight_wind_speed
        if self.wind_speed_val < 2125.98:     final_ws_weight = self.weight_wind_speed
        elif self.wind_speed_val <= 3366.14:  final_ws_weight = 0.13
        elif self.wind_speed_val <= 4803.15:  final_ws_weight = 0.17
        elif self.wind_speed_val <= 6417.32:  final_ws_weight = 0.21
        else:                                 final_ws_weight = 0.25

        # --- Global modifiers ---
        rainfall_factor    = 0.95 if self.precipitation > self.precipitation_threshold else 1.0
        temperature_factor = 1.3  if self.temperature_celsius >= self.temperature_threshold else 1.0

        # --- Score computation ---
        base = (
            self.weight_distance         * (1 - normalized_distance) +
            self.weight_flammability     * flammability                +
            final_ws_weight              * normalized_wind_speed       +
            self.weight_vpd              * normalized_vpd              +
            self.weight_fire_intensity   * normalized_fire_intensity   +
            self.weight_wind_fuel        * normalized_wind_fuel_sum    +
            self.weight_elevation        * norm_elev_risk              +
            self.weight_aspect           * norm_aspect_risk
        )
        score = base * (rainfall_factor * temperature_factor) + self.weight_fwi * (fwi / 100.0)

        # pick best
        sc = np.where(mask, score, -np.inf)
        best = int(np.argmax(sc))
        br, bc = np.unravel_index(best, sc.shape)
        new_goal = float(self.x_grid[br,bc]), float(self.y_grid[br,bc])

        best_cell = {
            'center':             new_goal,
            'score':              float(score[br, bc]),
            'fire_intensity':     float(fire_intensity[br, bc]),
            'H':                  float(self.H_arr[br, bc]),
            'w':                  float(fuel_load[br, bc]),
            'r':                  float(fire_counts[br, bc]),
            'distance':           float(distances[br, bc]),
            'wind_speed':         float(self.wind_speed_val),
            'wind_fuel_sum':      float(wind_fuel_sum[br, bc]),
            'flammability':       float(flammability[br, bc]),
            'vpd':                float(vpd_val),
            'relative_humidity':  float(self.rh_val),
            'vegetation':         vegetation_array[br, bc],
            'veg_factor':         float(veg_factors[br, bc]),
            'elevation_risk':     float(norm_elev_risk[br, bc]),
            'slope_factor':       float(slope_factor[br, bc]),
            'aspect_risk':        float(norm_aspect_risk[br, bc]),
            'fwi':                float(fwi),
        }

        if new_goal != self.prev_best_center[ns]:
            self.log_scoring_parameters(ns, [best_cell], rainfall_factor, temperature_factor)
            self.prev_best_center[ns] = new_goal


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
            f"slope_factor {best_cell['slope_factor']:.2f}, "
            f"aspect_risk {best_cell['aspect_risk']:.2f}, "
            f"fwi {best_cell['fwi']:.2f}, "
            f"temperature {self.temperature_celsius:.2f}°C, "
            f"rainfall_factor {rainfall_factor} (precip {self.precipitation:.2f}), "
            f"temperature_factor {temperature_factor}"
        )
        # publish & mark busy
        msg = Float32MultiArray(data=new_goal)
        self.goal_pubs[ns].publish(msg)
        self.ready[ns] = False

    # ─── Done (extinguish) callback ───────────────────────────────────
    def done_callback(self, msg, ns):
        try:
            x_str, y_str = msg.data.split(',')
            x, y = float(x_str), float(y_str)
        except:
            return

        # find nearest cell
        d2 = (self.x_grid - x)**2 + (self.y_grid - y)**2
        idx = int(np.argmin(d2))
        r, c = np.unravel_index(idx, d2.shape)

        self.blacklists[ns].add((r,c))
        self.get_logger().info(f"[{ns}] Extinguish done → blacklisted {(r,c)}")
        self.ready[ns] = True

    # ─── Helpers ─────────────────────────────────────────────────────
    def get_wind_vector(self, wdir):
        dirs = {
            'N':  (0, -1), 'NE': (math.sqrt(2)/2, -math.sqrt(2)/2),
            'E':  (1, 0),   'SE': (math.sqrt(2)/2, math.sqrt(2)/2),
            'S':  (0, 1),   'SW': (-math.sqrt(2)/2, math.sqrt(2)/2),
            'W':  (-1, 0),  'NW': (-math.sqrt(2)/2, -math.sqrt(2)/2)
        }
        return dirs.get(wdir.upper(), (0, -1))

    def calculate_vpd(self, t_c, rh):
        svp = 0.61078 * math.exp((17.2694 * t_c)/(t_c+237.3))
        ea  = svp * (1 - rh/100.0)
        return svp - ea

    def log_initial_cell_parameters(self):
        try:
            with open("initial_cell_parameters.log", "w") as f:
                for r in range(self.grid_rows):
                    for c in range(self.grid_cols):
                        f.write(f"Cell({r},{c}): RH={self.rh_val:.2f}%, "
                                f"WS={self.wind_speed_val:.2f}m/s, "
                                f"H={self.H_arr[r,c]:.2f}btu/lb\n")
            self.get_logger().info("Initial cell parameters logged.")
        except Exception as e:
            self.get_logger().error(f"Failed to log initial params: {e}")

    def log_scoring_parameters(self, ns, candidate_cells, rainfall_factor, temperature_factor):
        """Log all variables affecting score to a file for review."""
        filename = f"{ns}_scoring_parameters.log"
        try:
            cell = candidate_cells[0]  # Assuming only one candidate cell for logging
            with open(filename, "a") as f:
                f.write(f"[{ns}] Scoring Parameters for Candidate Cell at {cell['center']}:\n")
                f.write(f"  Fire Intensity: {cell['fire_intensity']:.2f}\n")
                f.write(f"  Heat Yield (H): {cell['H']:.2f} kJ/kg\n")
                f.write(f"  Fuel Load (w): {cell['w']:.2f} kg/m²\n")
                f.write(f"  fire_count (r): {cell['r']:.2f}\n")
                f.write(f"  Distance: {cell['distance']:.2f} m\n")
                f.write(f"  Relative Humidity: {cell['relative_humidity']:.2f}%\n")
                f.write(f"  Vegetation: {cell['vegetation']}\n")
                f.write(f"  Veg Factor: {self.vegetation_factors.get(cell['vegetation'], 1.0):.2f}\n")
                f.write(f"  Flammability: {cell['flammability']:.2f}\n")
                f.write(f"  Wind Speed: {cell['wind_speed']:.2f} m/s\n")
                f.write(f"  Wind Fuel Sum: {cell.get('wind_fuel_sum', 0):.2f} kg/m²\n")
                f.write(f"  VPD: {cell['vpd']:.3f} kPa\n")
                f.write(f"  Elevation Risk: {cell['elevation_risk']:.2f}\n")
                f.write(f"  Slope Factor: {cell['slope_factor']:.2f}\n")
                f.write(f"  Aspect Risk: {cell['aspect_risk']:.2f}\n")
                f.write(f"  FWI: {cell['fwi']:.2f}\n")
                f.write(f"  Score: {cell['score']:.2f}\n\n")
                f.write(f"Global Rainfall Factor: {rainfall_factor}\n")
                f.write(f"Global Temperature Factor: {temperature_factor}\n\n")
            self.get_logger().info(f"[{ns}] Scoring parameters logged to '{filename}'.")
        except Exception as e:
            self.get_logger().error(f"[{ns}] Failed to log scoring parameters: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FireCellGoalClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()