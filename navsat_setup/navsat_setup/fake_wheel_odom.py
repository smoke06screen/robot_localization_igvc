#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class ZeroOdom(Node):
    def __init__(self):
        super().__init__("zero_odometry")

        self.odom_pub = self.create_publisher(Odometry, "/wheel/odometry", 10)

        # Publish at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_zero_odom)

        self.get_logger().info("Zero Odometry node started.")

    def publish_zero_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Zero pose
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # Zero orientation (quaternion identity)
        msg.pose.pose.orientation = Quaternion(
            x = 0.0,
            y = 0.0,
            z = 0.0,
            w = 1.0
        )

        # Zero twist
        msg.twist.twist.linear.x  = 0.0
        msg.twist.twist.angular.z = 0.0

        # Small covariances (robot_localization requirement)
        msg.pose.covariance[0]  = 1e-3
        msg.pose.covariance[7]  = 1e-3
        msg.pose.covariance[35] = 1e-3

        msg.twist.covariance[0]  = 1e-3
        msg.twist.covariance[35] = 1e-3

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZeroOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

