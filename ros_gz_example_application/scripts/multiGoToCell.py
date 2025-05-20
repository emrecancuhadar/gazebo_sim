#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class MultiGoToCell(Node):
    def __init__(self):
        super().__init__('multi_go_to_cell')

        # ─── grid→world params (must match HandleFire!) ──────────────────
        self.grid_rows = 10
        self.grid_cols = 10
        self.cell_size = 2.0
        self.offset_x = -((self.grid_cols - 1) * self.cell_size) / 2.0
        self.offset_y = -((self.grid_rows - 1) * self.cell_size) / 2.0

        # ─── which robots, which target cells ────────────────────────────
        self.robots = ['diff_drive','diff_drive2','diff_drive3']
        # assign each robot a (row, col)
        self.goals_grid = {
            'diff_drive':  (2, 2),
            'diff_drive2': (2, 7),
            'diff_drive3': (7, 7),
        }

        # compute world-frame goals
        self.goals = {}
        for ns, (r, c) in self.goals_grid.items():
            x = self.offset_x + c * self.cell_size
            y = self.offset_y + (self.grid_rows-1 - r) * self.cell_size
            self.goals[ns] = (x, y)
            self.get_logger().info(f'{ns} goal → cell ({r},{c}) at world ({x:.2f},{y:.2f})')

        # ─── state per robot ──────────────────────────────────────────────
        self.pose       = {ns: (0.0,0.0,0.0) for ns in self.robots}  # x,y,yaw
        self.aligned    = {ns: False for ns in self.robots}
        self.reached    = {ns: False for ns in self.robots}

        # control gains
        self.max_lin = 0.5
        self.max_ang = 0.3
        self.lin_gain = 0.6
        self.ang_gain = 0.8
        self.align_tol   = 0.05
        self.unalign_tol = 0.15
        self.dist_tol    = 0.05

        # ─── pubs & subs ─────────────────────────────────────────────────
        self.cmd_pubs = {}
        for ns in self.robots:
            # publisher for cmd_vel
            pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
            self.cmd_pubs[ns] = pub

            # subscription to odometry
            self.create_subscription(
                Odometry,
                f'/{ns}/odometry',
                lambda msg, ns=ns: self.odom_cb(msg, ns),
                10
            )
            self.get_logger().info(f'Subscribed to /{ns}/odometry')

        # ─── drive loop @ 10Hz ───────────────────────────────────────────
        self.create_timer(0.1, self.drive_robots)

    def odom_cb(self, msg: Odometry, ns: str):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _, _, yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        self.pose[ns] = (x, y, yaw)

    def drive_robots(self):
        all_done = True

        for ns in self.robots:
            if self.reached[ns]:
                continue

            all_done = False
            x, y, yaw = self.pose[ns]
            xg, yg    = self.goals[ns]

            dx = xg - x
            dy = yg - y
            dist = math.hypot(dx, dy)
            target_theta = math.atan2(dy, dx)
            err = math.atan2(
                math.sin(target_theta - yaw),
                math.cos(target_theta - yaw)
            )

            twist = Twist()

            # check arrival
            if dist < self.dist_tol:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
                self.reached[ns] = True
                self.get_logger().info(f'{ns} reached goal ({xg:.2f},{yg:.2f})')
            else:
                # rotate first
                if not self.aligned[ns]:
                    if abs(err) < self.align_tol:
                        self.aligned[ns] = True
                    else:
                        twist.angular.z = max(-self.max_ang,
                                              min(self.max_ang,
                                                  self.ang_gain * err))
                else:
                    # correct if we drift
                    if abs(err) > self.unalign_tol:
                        self.aligned[ns] = False
                    else:
                        twist.linear.x = max(0.0,
                                             min(self.max_lin,
                                                 self.lin_gain * dist))

            self.cmd_pubs[ns].publish(twist)

        if all_done:
            self.get_logger().info('All robots have reached their goals. Stopping node.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MultiGoToCell()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
