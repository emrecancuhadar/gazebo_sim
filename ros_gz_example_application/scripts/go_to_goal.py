#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
from tf_transformations import euler_from_quaternion

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.ns = self.declare_parameter('robot_ns', 'diff_drive').value

        # publishers & subscribers
        self.vel_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.done_pub = self.create_publisher(String, f'/{self.ns}/fire_cell_done', 10)
        self.create_subscription(Odometry, f'/{self.ns}/odometry', self.odom_cb, 10)
        self.create_subscription(Float32MultiArray, f'/{self.ns}/fire_cell_goal', self.goal_cb, 10)

        # track peers for repulsion
        self.robot_list = self.declare_parameter(
            'robot_list',
            ['diff_drive','diff_drive2','diff_drive3']
        ).value
        self.peer_positions = {
            peer: None for peer in self.robot_list if peer != self.ns
        }
        for peer in self.peer_positions:
            self.create_subscription(
                Odometry,
                f'/{peer}/odometry',
                lambda msg, peer=peer: self.peer_odom_cb(msg, peer),
                10
            )

        # state
        self.robot_x = self.robot_y = self.robot_yaw = 0.0
        self.goal       = None
        self.goal_active = False

        # gains & thresholds  
        self.k_att      = 1.0    # attractive gain
        self.k_repulse  = 1.5    # repulsive gain
        self.safe_radius = 0.2   # m, start repelling inside this
        self.max_lin    = 0.5
        self.max_ang    = 1.0
        self.dist_tol   = 0.05
        self.yaw_tol    = 0.05

        self.create_timer(0.1, self.navigate)
        self.get_logger().info(f"[{self.ns}] go_to_goal (potential field) ready")

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _,_,self.robot_yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

    def peer_odom_cb(self, msg, peer):
        self.peer_positions[peer] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def goal_cb(self, msg):
        if len(msg.data) < 2:
            return
        self.goal = (msg.data[0], msg.data[1])
        self.goal_active = True
        self.get_logger().info(f"[{self.ns}] New goal → {self.goal}")

    def navigate(self):
        if not self.goal_active or self.goal is None:
            return

        gx, gy = self.goal
        dx, dy = gx - self.robot_x, gy - self.robot_y
        rho   = math.hypot(dx, dy)

        # attractive force (world frame)
        if rho > self.dist_tol:
            f_ax = self.k_att * dx
            f_ay = self.k_att * dy
        else:
            # reached goal
            self.stop()
            done = String(); done.data = f"{gx:.2f},{gy:.2f}"
            self.done_pub.publish(done)
            self.get_logger().info(f"[{self.ns}] Goal reached")
            self.goal_active = False
            return

        # repulsive force from each peer
        f_rx, f_ry = 0.0, 0.0
        for pos in self.peer_positions.values():
            if pos is None:
                continue
            px, py = pos
            dxp = self.robot_x - px
            dyp = self.robot_y - py
            dist_p = math.hypot(dxp, dyp)
            if dist_p < 1e-6:
                continue
            if dist_p < self.safe_radius:
                # simple 1/r^2 style repulsion
                mag = self.k_repulse * (1.0/dist_p - 1.0/self.safe_radius) / (dist_p**2)
                f_rx += mag * (dxp/dist_p)
                f_ry += mag * (dyp/dist_p)

        # total force in world frame
        fx = f_ax + f_rx
        fy = f_ay + f_ry

        # convert to desired heading & speed
        desired_yaw = math.atan2(fy, fx)
        yaw_err     = math.atan2(
            math.sin(desired_yaw - self.robot_yaw),
            math.cos(desired_yaw - self.robot_yaw)
        )
        # magnitude of force vector as forward speed
        f_mag = math.hypot(fx, fy)

        twist = Twist()
        # if heading error large, rotate first
        if abs(yaw_err) > self.yaw_tol:
            twist.angular.z = max(-self.max_ang, min(self.max_ang, yaw_err))
        else:
            twist.linear.x = max(-self.max_lin, min(self.max_lin, self.k_att * f_mag))

        self.vel_pub.publish(twist)

    def stop(self):
        self.vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
