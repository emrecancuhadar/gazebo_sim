#!/usr/bin/env python3

import rclpy
import time
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion  # ✅ Convert quaternion to euler angles

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # ✅ Publish velocity commands
        self.vel_pub = self.create_publisher(Twist, "/diff_drive/cmd_vel", 10)

        # 🎯 Subscribe to ball positions
        self.ball_sub = self.create_subscription(PoseArray, "/ball_positions", self.ball_callback, 10)

        # 📡 Subscribe to odometry data for position updates
        self.odom_sub = self.create_subscription(Odometry, "/diff_drive/odometry", self.odom_callback, 10)

        # 🚀 Initialize variables
        self.robot_position = (0.0, 0.0)  # Start position
        self.robot_orientation = 0.0  # ✅ Store robot yaw angle (heading)
        self.ball_positions = set()  # ✅ Track only unique balls
        self.visited_targets = set()  # ✅ Keep track of visited targets
        self.current_target = None
        self.last_log_time = time.time()  # ⏳ Timestamp for rate limiting

        # Low-pass filter variables to smooth angular velocity
        self.previous_angular_speed = 0.0
        self.angular_smoothing_factor = 0.3  # Adjust to reduce quick oscillations

        # Timer for processing updates every 0.1 seconds
        self.timer = self.create_timer(0.1, self.navigate)

    def odom_callback(self, msg):
        """ Update the robot's position and orientation from odometry data """
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # ✅ Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.robot_orientation = yaw  # ✅ Store robot heading

    def ball_callback(self, msg):
        """ Callback to receive ball positions, ensuring duplicates are not added """
        for pose in msg.poses:
            ball = (pose.position.x, pose.position.y)

            # ✅ Add only if the ball hasn't been visited
            if ball not in self.visited_targets:
                self.ball_positions.add(ball)

        if self.ball_positions and time.time() - self.last_log_time >= 2.0:
            self.get_logger().info(f"🎯 Tracking {len(self.ball_positions)} balls")
            self.last_log_time = time.time()

    def find_nearest_ball(self):
        """ Find the closest ball to the robot """
        if not self.ball_positions:
            return None

        distances = [math.dist(self.robot_position, ball) for ball in self.ball_positions]
        min_index = np.argmin(distances)
        return list(self.ball_positions)[min_index]

    def navigate(self):
        """ Navigate towards the closest ball """
        if not self.ball_positions:
            if time.time() - self.last_log_time >= 2.0:
                self.get_logger().info("🚫 No balls left. Stopping robot.")
                self.stop_robot()
                self.last_log_time = time.time()
            return

        # ✅ Find a new target if none exists
        if self.current_target is None:
            self.current_target = self.find_nearest_ball()

        # ✅ Check if we've reached the current target
        if self.current_target:
            x_goal, y_goal = self.current_target
            x_robot, y_robot = self.robot_position

            distance = math.hypot(x_goal - x_robot, y_goal - y_robot)

            if distance < 0.15:  # **Reaching threshold**
                self.get_logger().info(f"✅ Reached target ball at {self.current_target}. Waiting 2s before next move.")

                self.stop_robot()
                time.sleep(2)  # **Wait for 2 seconds**

                # **Mark as visited & remove from list**
                self.visited_targets.add(self.current_target)
                self.ball_positions.discard(self.current_target)  
                self.current_target = None  # **Reset target to select a new one**
                
                return  # **Exit to allow re-evaluation in next cycle**

        # ✅ Move towards the target
        self.move_towards_target(self.current_target)


    def move_towards_target(self, target):
        """ Moves the robot towards a target ball """
        x_goal, y_goal = target
        x_robot, y_robot = self.robot_position

        # Compute direction
        dx, dy = x_goal - x_robot, y_goal - y_robot
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)

        # ✅ Compute angle error (difference between current heading and goal)
        angle_error = angle_to_target - self.robot_orientation

        # ✅ Normalize angle error to [-π, π] to prevent excessive spinning
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # 🚀 Adjust speed balance between rotation and forward movement
        max_angular_speed = 0.5  # **Lowered from 1.0 to 0.5**
        min_angular_speed = 0.05  # **Ensures small corrections**
        
        # ✅ **New: Reduce Angular Dominance**
        angular_gain = 0.8  # **Reduced to prevent over-rotation**
        raw_angular_speed = max(-max_angular_speed, min(max_angular_speed, angular_gain * angle_error))

        # ✅ **New: Ensure Forward Movement**
        if abs(angle_error) < 0.4:  # **If close to correct direction, move forward**
            linear_speed = min(0.4, distance)  # **Move at 0.4 m/s max**
        else:
            linear_speed = 0.0  # **Stop moving forward if not well-aligned**

        # ✅ Apply smoothing to prevent oscillations
        self.previous_angular_speed = (
            0.6 * self.previous_angular_speed + 0.4 * raw_angular_speed
        )

        # 🚀 Set velocities
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = self.previous_angular_speed

        self.vel_pub.publish(vel_msg)




    def stop_robot(self):
        """ Stops the robot """
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
