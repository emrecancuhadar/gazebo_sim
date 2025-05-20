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

from fire_weather_index import (
    calculate_ffmc,
    calculate_dmc,
    calculate_dc,
    calculate_isi,
    calculate_bui,
    calculate_fwi,
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

        # ─── Fire-weather and scoring parameters ─────────────────────────
        # (all your existing parameters: humidity, wind, H_min/H_max, precipitation, temperature, vegetation_factors, weights…)
        self.relative_humidity_min   = self.declare_parameter("relative_humidity_min", 0.0).value
        self.relative_humidity_max   = self.declare_parameter("relative_humidity_max", 100.0).value
        self.wind_speed_min          = self.declare_parameter("wind_speed_min", 0.0).value
        self.wind_speed_max          = self.declare_parameter("wind_speed_max", 50.0).value
        self.H_min                   = self.declare_parameter("H_min", 16000.0).value
        self.H_max                   = self.declare_parameter("H_max", 20000.0).value

        self.precipitation           = self.declare_parameter("precipitation", 1200.0).value
        self.precipitation_threshold = self.declare_parameter("precipitation_threshold",1000.0).value
        self.temperature_celsius     = self.declare_parameter("temperature", 25.0).value
        self.temperature_threshold   = self.declare_parameter("temperature_threshold",30.0).value
        self.wind_direction_param    = self.declare_parameter("wind_direction","SE").value

        self.vegetation_factors = {
            'sparse':   1.0,
            'bare':     0.3,
            'conifer':  1.4,
            'deciduous':0.8
        }

        self.weight_distance       = self.declare_parameter("weight_distance",     0.4).value
        self.weight_flammability   = self.declare_parameter("weight_flammability", 0.15).value
        self.weight_fire_intensity = self.declare_parameter("weight_fire_intensity",0.25).value
        self.weight_wind_fuel      = self.declare_parameter("weight_wind_fuel",     0.1).value
        self.weight_wind_speed     = self.declare_parameter("weight_wind_speed",    0.1).value
        self.weight_vpd            = self.declare_parameter("weight_vpd",           0.15).value
        self.weight_elevation      = self.declare_parameter("weight_elevation",   0.1).value
        self.weight_slope          = self.declare_parameter("weight_slope",       0.2).value
        self.weight_aspect         = self.declare_parameter("weight_aspect",      0.15).value
        self.weight_fwi            = self.declare_parameter("weight_fwi",          0.2).value

        self.prev_ffmc_default = self.declare_parameter("prev_ffmc",85.0).value
        self.prev_dmc_default  = self.declare_parameter("prev_dmc", 6.0).value
        self.prev_dc_default   = self.declare_parameter("prev_dc",15.0).value

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
        # reshape
        fc   = np.array(self.grid_data).reshape(self.grid_rows, self.grid_cols)
        w    = np.array(self.fuel_load_data).reshape(self.grid_rows, self.grid_cols)
        veg  = np.array(self.vegetation_data).reshape(self.grid_rows, self.grid_cols)
        elev = np.array(self.elevation_data).reshape(self.grid_rows, self.grid_cols)
        slo  = np.array(self.slope_data).reshape(self.grid_rows, self.grid_cols)
        asp  = np.array(self.aspect_data).reshape(self.grid_rows, self.grid_cols)

        mask = fc > 0
        for (r,c) in self.blacklists[ns]:
            mask[r,c] = False
        if not mask.any():
            self.get_logger().info(f"[{ns}] No active fire cells.")
            return

        rx, ry    = self.poses[ns]
        distances = np.hypot(self.x_grid - rx, self.y_grid - ry)
        fire_intensity = self.H_arr * w * fc

        # wind-fuel sum (same as before)…
        wind_fuel_sum = np.zeros_like(w)
        if self.wind_speed_val >= 17.2:
            wx, wy = self.get_wind_vector(self.wind_direction_param)
            for r, c in zip(*np.where(mask)):
                dx = self.x_grid - self.x_grid[r,c]
                dy = self.y_grid - self.y_grid[r,c]
                norms = np.hypot(dx, dy) + 1e-6
                dots  = (dx/norms)*wx + (dy/norms)*wy
                wind_fuel_sum[r,c] = float(np.sum(w[(dots>0.7071)&(norms>1e-6)]))

        # FWI
        month = time.localtime().tm_mon
        ffmc = calculate_ffmc(self.prev_ffmc_default, self.temperature_celsius,
                              self.rh_val, self.wind_speed_val, self.precipitation)
        dmc  = calculate_dmc(self.prev_dmc_default, self.temperature_celsius,
                              self.rh_val, self.precipitation, month)
        dc   = calculate_dc(self.prev_dc_default, self.temperature_celsius,
                              self.precipitation, month)
        isi  = calculate_isi(ffmc, self.wind_speed_val)
        bui  = calculate_bui(dmc, dc)
        fwi  = calculate_fwi(isi, bui)

        # normalize & score
        eps = 1e-6
        max_dist = distances[mask].max()
        max_fi   = fire_intensity[mask].max()
        max_wf   = wind_fuel_sum[mask].max() if mask.any() else 1.0
        max_w    = w.max()
        vpd_val  = self.calculate_vpd(self.temperature_celsius, self.rh_val)

        nd = distances / (max_dist+eps)
        nfi = fire_intensity / (max_fi+eps)
        nw = w / (max_w+eps)
        nwf = wind_fuel_sum / (max_wf+eps)
        nws = self.wind_speed_val / (self.wind_speed_max+eps)
        nvpd= vpd_val / (vpd_val+eps)

        vf = np.vectorize(lambda v: self.vegetation_factors.get(v,1.0))(veg)
        flam = np.minimum(1.0, vf*(0.4*nvpd + 0.4*nw + 0.2*nws))

        me, xe = elev.min(), elev.max()
        ner = 1.0 - (elev-me)/(xe-me+eps)
        nsr = np.minimum(slo/90.0,1.0)
        asp_rad = np.deg2rad(asp-180.0)
        nar = (1.0 + np.cos(asp_rad))/2.0

        # dynamic wind weight (as before)…
        wws = self.weight_wind_speed
        if self.wind_speed_val < 10.8:    wws = self.weight_wind_speed
        elif self.wind_speed_val <=17.1:  wws = 0.13
        elif self.wind_speed_val <=24.4:  wws = 0.17
        elif self.wind_speed_val <=32.6:  wws = 0.21
        else:                             wws = 0.25

        rf = 0.95 if self.precipitation>self.precipitation_threshold else 1.0
        tf = 1.3  if self.temperature_celsius>=self.temperature_threshold else 1.0

        base = (
            self.weight_distance       * (1-nd) +
            self.weight_flammability   * flam  +
            wws                        * nws   +
            self.weight_vpd            * nvpd  +
            self.weight_fire_intensity * nfi   +
            self.weight_wind_fuel      * nwf   +
            self.weight_elevation      * ner   +
            self.weight_slope          * nsr   +
            self.weight_aspect         * nar
        )
        score = base*(rf*tf) + self.weight_fwi*(fwi/100.0)

        # pick best
        sc = np.where(mask, score, -np.inf)
        best = int(np.argmax(sc))
        br, bc = np.unravel_index(best, sc.shape)
        xg, yg = float(self.x_grid[br,bc]), float(self.y_grid[br,bc])

        self.get_logger().info(f"[{ns}] Goal → cell {(br,bc)} @ ({xg:.2f},{yg:.2f}), score={score[br,bc]:.2f}")

        # publish & mark busy
        msg = Float32MultiArray(data=[xg, yg])
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
                                f"H={self.H_arr[r,c]:.2f}\n")
            self.get_logger().info("Initial cell parameters logged.")
        except Exception as e:
            self.get_logger().error(f"Failed to log initial params: {e}")

    def log_scoring_parameters(self, candidate_cells, rainfall_factor, temperature_factor):
        # unchanged…
        pass

def main(args=None):
    rclpy.init(args=args)
    node = FireCellGoalClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
