#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import numpy as np
from scipy.spatial.transform import Rotation

class LandingController(Node):
    def __init__(self):
        super().__init__('landing_controller')
        self.get_logger().info('Landing Controller Node Started')

        # Parameters
        self.declare_parameter('kp_xy', 0.5)  # Proportional gain for x, y
        self.declare_parameter('kp_z', 0.3)   # Proportional gain for z
        self.declare_parameter('kp_yaw', 0.5) # Proportional gain for yaw
        self.declare_parameter('max_speed_xy', 0.3)  # Max speed (m/s) for x, y
        self.declare_parameter('max_speed_z', 0.3)   # Max speed (m/s) for z
        self.declare_parameter('max_yaw_rate', 0.5)  # Max yaw rate (rad/s)
        self.declare_parameter('landing_height', 0.1) # Height to stop descending
        self.declare_parameter('alignment_threshold', 0.05) # Position error threshold (m)

        self.kp_xy = self.get_parameter('kp_xy').value
        self.kp_z = self.get_parameter('kp_z').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.max_speed_xy = self.get_parameter('max_speed_xy').value
        self.max_speed_z = self.get_parameter('max_speed_z').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.landing_height = self.get_parameter('landing_height').value
        self.alignment_threshold = self.get_parameter('alignment_threshold').value

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_pose',
            self.pose_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/uav/control/cmd_vel',
            10
        )

        # State
        self.landing = False

        # Error offset
        self.target_yaw = -1.57079633
        self.y_offset = 0.1

    def pose_callback(self, msg: PoseStamped):
        # Extract position and orientation
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        r = Rotation.from_quat(q)
        euler = r.as_euler('xyz')  # Roll, pitch, yaw
        yaw = euler[2]

        # Control logic
        cmd_vel = Twist()

        # Position error
        error_x = px  # Marker right (+x) -> move right
        error_y = py - self.y_offset  # Marker down (+y) -> move down
        error_z = pz - self.landing_height
        error_yaw = yaw - self.target_yaw  # Align yaw to marker

        # Proportional control
        cmd_vel.linear.x = -self.kp_xy * error_y
        cmd_vel.linear.y = -self.kp_xy * error_x
        cmd_vel.linear.z = -self.kp_z * error_z
        cmd_vel.angular.z = -self.kp_yaw * error_yaw

        # Limit speeds
        cmd_vel.linear.x = np.clip(cmd_vel.linear.x, -self.max_speed_xy, self.max_speed_xy)
        cmd_vel.linear.y = np.clip(cmd_vel.linear.y, -self.max_speed_xy, self.max_speed_xy)
        cmd_vel.linear.z = np.clip(cmd_vel.linear.z, -self.max_speed_z, self.max_speed_z)
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        # Check if aligned for landing
        if (abs(error_x) < self.alignment_threshold and
            abs(error_y) < self.alignment_threshold and
            pz < self.landing_height + 0.05):
            self.landing = True
            cmd_vel.linear.z = -0.1  # Slow descent
            if pz < 0.05:  # Close to platform
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                cmd_vel.linear.z = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().info("Landing complete!")

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = LandingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()