#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import time

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # --- Publishers / Subscribers ---
        self.vel_pub  = self.create_publisher(Twist, "/diff_drive/cmd_vel", 10)
        self.done_pub = self.create_publisher(String, "/fire_cell_done", 10)

        self.create_subscription(Odometry,             "/diff_drive/odometry", self.odom_callback, 10)
        self.create_subscription(Float32MultiArray,    "/fire_cell_goal",     self.goal_callback, 10)

        # --- Robot State ---
        self.robot_position    = (0.0, 0.0)
        self.robot_orientation = 0.0
        self.current_goal      = None
        self.goal_active       = False
        self.aligned           = False   # ← alignment state

        # --- Speed & Control Params ---
        self.max_linear_speed  = 0.1    # m/s
        self.max_angular_speed = 0.1    # rad/s
        self.lin_gain          = 0.6
        self.ang_gain          = 0.5

        # Hysteresis thresholds (rad)
        self.align_tol   = 0.05   # when |err| < 0.05, we declare “aligned”
        self.unalign_tol = 0.15   # when |err| > 0.15, we go back to “aligning”

        # --- Timer to run control loop ---
        self.create_timer(0.1, self.navigate)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_position = (x, y)

        _, _, yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.robot_orientation = yaw

    def goal_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.current_goal = (msg.data[0], msg.data[1])
            self.goal_active  = True
            self.aligned      = False
            self.get_logger().info(
                f"New goal: x={self.current_goal[0]:.2f}, y={self.current_goal[1]:.2f}"
            )

    def navigate(self):
        if not self.goal_active or self.current_goal is None:
            return

        x_goal, y_goal = self.current_goal
        x, y           = self.robot_position
        dx, dy         = x_goal - x, y_goal - y
        dist           = math.hypot(dx, dy)
        target_theta   = math.atan2(dy, dx)
        err = math.atan2(
            math.sin(target_theta - self.robot_orientation),
            math.cos(target_theta - self.robot_orientation)
        )

        # --- Check goal reached ---
        if dist < 0.05:
            # 1) Stop
            self.stop_robot()
            self.get_logger().info(f"Reached ({x_goal:.2f}, {y_goal:.2f}) → waiting 30 s before reporting done")

            # 2) Wait 30 s
            time.sleep(30)

            # 3) Publish done notice
            done = String()
            done.data = f"{x_goal:.2f},{y_goal:.2f}"
            self.done_pub.publish(done)
            self.get_logger().info("Done published, ready for next goal")

            # 4) Clear active goal
            self.goal_active = False
            return

        # --- Normal control (rotate or drive) ---
        twist = Twist()
        if not self.aligned:
            if abs(err) < self.align_tol:
                self.aligned = True
            else:
                ang = self.ang_gain * err
                ang = max(-self.max_angular_speed, min(self.max_angular_speed, ang))
                twist.angular.z = ang
        else:
            if abs(err) > self.unalign_tol:
                self.aligned = False
            else:
                lin = self.lin_gain * dist
                lin = min(self.max_linear_speed, lin)
                twist.linear.x = lin

        self.vel_pub.publish(twist)

    def stop_robot(self):
        self.vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
