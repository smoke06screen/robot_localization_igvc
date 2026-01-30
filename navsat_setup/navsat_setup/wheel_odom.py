#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import math


class DiffDriveRPMOdom(Node):
    def __init__(self):
        super().__init__("diff_rpm_odometry")

        # ==== Robot Parameters ====
        self.wheel_radius = 0.185   # meters
        self.wheel_track  = 0.78    # meters (distance between wheels)

        # ==== Serial Config ====
        self.port = "/dev/ttyACM0"
        self.baud = 115200
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)

        # ==== State Variables ====
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry, "/wheel/odometry", 10
        )

        # 100 Hz timer
        self.create_timer(1.0 / 100.0, self.update)

        self.get_logger().info("RPM Odometry node started.")

    # --------------------------------------------------
    def rpm_to_speed(self, rpm):
        """RPM -> m/s"""
        return (rpm * 2.0 * math.pi / 60.0) * self.wheel_radius

    # --------------------------------------------------
    def update(self):
        line = self.ser.readline().decode(errors="ignore").strip()
        if not line or "," not in line:
            return

        try:
            # Expected format: Rrpm,_,Lrpm,_
            Rrpm, _, Lrpm, _ = map(float, line.split(","))
        except ValueError:
            self.get_logger().warn(f"Parse error: {line}")
            return

        # Wheel linear velocities
        v_l = self.rpm_to_speed(Lrpm)
        v_r = self.rpm_to_speed(Rrpm)

        # Differential drive kinematics
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.wheel_track

        # Time integration (ROS time)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # Pose integration
        if abs(w) < 1e-6:
            self.x += v * dt * math.cos(self.theta)
            self.y += v * dt * math.sin(self.theta)
        else:
            dtheta = w * dt
            R = v / w
            self.x += R * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += R * (math.cos(self.theta) - math.cos(self.theta + dtheta))
            self.theta += dtheta

        # Normalize yaw
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        # --------------------------------------------------
        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.theta / 2.0),
            w=math.cos(self.theta / 2.0),
        )

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # --------------------------------------------------
        # Covariances (NO ZEROS)
        for i in range(36):
            odom.pose.covariance[i] = 1e6
            odom.twist.covariance[i] = 1e6

        # Pose covariance
        odom.pose.covariance[0]  = 0.02   # x
        odom.pose.covariance[7]  = 0.02   # y
        odom.pose.covariance[35] = 0.2    # yaw

        # Twist covariance
        odom.twist.covariance[0]  = 0.05  # vx
        odom.twist.covariance[35] = 0.3   # wz

        self.odom_pub.publish(odom)


# ======================================================
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveRPMOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
