#!/usr/bin/env python3
import rclpy, math
from collections import deque
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # Publishers / Subscribers
        self.vel_pub  = self.create_publisher(Twist, "/diff_drive/cmd_vel", 10)
        self.done_pub = self.create_publisher(String, "/fire_cell_done", 10)
        self.create_subscription(Odometry, "/diff_drive/odometry",  self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, "/fire_cell_goal", self.goal_callback, 10)

        # Robot state
        self.robot_position    = (0.0, 0.0)
        self.robot_orientation = 0.0

        # Goal queue and flags
        self.goal_queue         = deque()
        self.current_goal       = None
        self.goal_active        = False
        self.waiting_extinguish = False
        self.aligned            = False

        # Control parameters
        self.max_linear_speed  = 0.5
        self.max_angular_speed = 0.1
        self.lin_gain          = 0.6
        self.ang_gain          = 0.5
        self.align_tol         = 0.05
        self.unalign_tol       = 0.15

        # Timer for navigate loop
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
        """Receive a new goal.  Preempt the active one if we’re driving."""
        if len(msg.data) < 2:
            return
        new_goal = (msg.data[0], msg.data[1])
        self.get_logger().info(f"Received goal: {new_goal}")

        if self.waiting_extinguish:
            # during extinguish, queue up
            self.goal_queue.append(new_goal)
            self.get_logger().info("→ Currently extinguishing; queued for later.")
        elif self.goal_active:
            # preempt current drive goal
            self.current_goal = new_goal
            self.aligned = False
            self.get_logger().info(f"→ Preempting and switching to new goal {new_goal}")
        else:
            # idle: just queue & start
            self.goal_queue.append(new_goal)
            self.next_goal()

    def next_goal(self):
        """Pop a goal off the queue if we’re not extinguishing or already driving."""
        if not self.waiting_extinguish and not self.goal_active and self.goal_queue:
            self.current_goal = self.goal_queue.popleft()
            self.goal_active  = True
            self.aligned      = False
            x, y = self.current_goal
            self.get_logger().info(f"New active goal: ({x:.2f}, {y:.2f})")

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

        # reached?
        if dist < 0.05:
            self.stop_robot()
            self.get_logger().info(
                f"Reached ({x_goal:.2f}, {y_goal:.2f}) → extinguishing 30 s"
            )
            self.goal_active        = False
            self.waiting_extinguish = True
            # start extinguish timer
            self.ext_timer = self.create_timer(30.0, self.extinguish_done)
            return

        # otherwise: either rotate to align, or drive straight
        twist = Twist()
        if not self.aligned:
            if abs(err) < self.align_tol:
                self.aligned = True
            else:
                ang = self.ang_gain * err
                twist.angular.z = max(-self.max_angular_speed,
                                      min(self.max_angular_speed, ang))
        else:
            if abs(err) > self.unalign_tol:
                self.aligned = False
            else:
                lin = self.lin_gain * dist
                twist.linear.x = min(self.max_linear_speed, lin)

        self.vel_pub.publish(twist)

    def extinguish_done(self):
        # extinguish finishes: cancel timer, publish done, then start next
        self.ext_timer.cancel()
        self.waiting_extinguish = False

        x, y = self.current_goal
        done = String()
        done.data = f"{x:.2f},{y:.2f}"
        self.done_pub.publish(done)
        self.get_logger().info(f"Extinguish done: ({x:.2f}, {y:.2f}); ready for next")

        # kick off the next queued goal, if any
        self.next_goal()

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
