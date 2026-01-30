#!/usr/bin/env python3

import math
import struct
import threading
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class WTIMUNode(Node):
    def __init__(self):
        super().__init__('wt_imu_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/imu')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'imu_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        frame_id = self.get_parameter('frame_id').value

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = frame_id

        # Covariances (important for EKF)
        self.imu_msg.orientation_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        self.imu_msg.angular_velocity_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        self.imu_msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        # ---------------- State ----------------
        self.accel = [0.0, 0.0, 0.0]   # m/s^2
        self.gyro  = [0.0, 0.0, 0.0]   # rad/s
        self.euler = [0.0, 0.0, 0.0]   # rad

        self.buffer = bytearray()
        self.lock = threading.Lock()
        self.running = True

        # ---------------- Serial ----------------
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"IMU connected on {port} @ {baud}")
        except Exception as e:
            self.get_logger().fatal(str(e))
            raise SystemExit

        # ---------------- Thread ----------------
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    # ============================================================
    def read_loop(self):
        while rclpy.ok() and self.running:
            data = self.ser.read(64)
            if data:
                self.buffer.extend(data)
                self.parse_buffer()

    # ============================================================
    def parse_buffer(self):
        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                self.buffer.pop(0)
                continue

            frame = self.buffer[:11]
            self.buffer = self.buffer[11:]

            if (sum(frame[:10]) & 0xFF) != frame[10]:
                continue

            frame_id = frame[1]
            payload = struct.unpack('<hhhh', frame[2:10])

            with self.lock:
                if frame_id == 0x51:  # Acceleration (g)
                    self.accel = [
                        payload[0] / 32768.0 * 16.0 * 9.80665,
                        payload[1] / 32768.0 * 16.0 * 9.80665,
                        payload[2] / 32768.0 * 16.0 * 9.80665,
                    ]

                elif frame_id == 0x52:  # Gyro (deg/s)
                    self.gyro = [
                        math.radians(payload[0] / 32768.0 * 2000.0),
                        math.radians(payload[1] / 32768.0 * 2000.0),
                        math.radians(payload[2] / 32768.0 * 2000.0),
                    ]

                elif frame_id == 0x53:  # Euler angles (deg)
                    self.euler = [
                        math.radians(payload[0] / 32768.0 * 180.0),
                        math.radians(payload[1] / 32768.0 * 180.0),
                        math.radians(payload[2] / 32768.0 * 180.0),
                    ]
                    self.publish_imu()

    # ============================================================
    def publish_imu(self):
        roll, pitch, yaw = self.euler
        qx, qy, qz, qw = self.euler_to_quat(roll, pitch, yaw)

        msg = self.imu_msg
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]

        msg.linear_acceleration.x = self.accel[0]
        msg.linear_acceleration.y = self.accel[1]
        msg.linear_acceleration.z = self.accel[2]

        self.pub.publish(msg)

    # ============================================================
    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    # ============================================================
    def destroy_node(self):
        self.running = False
        self.ser.close()
        super().destroy_node()


# ================================================================
def main():
    rclpy.init()
    node = WTIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

