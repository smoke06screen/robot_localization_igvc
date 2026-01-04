import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import math
import time

class DiffDriveRPMOdom(Node):
    def __init__(self):
        super().__init__("diff_rpm_odometry")

        # ==== Robot Parameters ====
        self.wheel_radius = 0.185      # meters (example: 5cm wheel radius)
        self.wheel_track  = 0.78      # meters (distance between wheels)

        # ==== Serial Config ====
        self.port = "/dev/ttyACM0"
        self.baud = 115200
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)

        # ==== State Variables ====
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odometry", 10)

        # 100 Hz timer
        self.create_timer(1.0 / 100.0, self.update)

        self.get_logger().info("RPM Odometry node started.")

    def rpm_to_speed(self, rpm):
        # rpm -> rad/s -> m/s
        return (rpm * 2.0 * math.pi / 60.0) * self.wheel_radius

    def update(self):
        # Read line from serial port
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line or "," not in line:
            return
        
        try:
            # Parse four values: Rrpm, _, Lrpm, _
            Rrpm, _, Lrpm, _ = map(float, line.split(","))
        except ValueError:
            self.get_logger().warn(f"Parse error: {line}")
            return

        # Wheel linear speeds (m/s)
        v_l = self.rpm_to_speed(Lrpm)
        v_r = self.rpm_to_speed(Rrpm)

        # Differential drive kinematics
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.wheel_track

        # Time integration
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if abs(w) < 1e-6:
            # Straight line motion
            self.x += v * dt * math.cos(self.theta)
            self.y += v * dt * math.sin(self.theta)
        else:
            # Arc motion
            R = v / w
            dtheta = w * dt
            self.x += R * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += R * (math.cos(self.theta) - math.cos(self.theta + dtheta))
            self.theta += dtheta

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.theta / 2.0),
            w=math.cos(self.theta / 2.0)
        )

        # Twist (linear and angular velocities)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Covariance (minimal, tweak if needed)
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.1

        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.1

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveRPMOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
