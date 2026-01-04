#!/usr/bin/env python3
import time
import math
import serial
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Globals used by the parser / publisher
key = 0
buff = {}
angularVelocity = [0.0, 0.0, 0.0]   # rad/s (after scaling in parser)
acceleration = [0.0, 0.0, 0.0]      # m/s^2  (after scaling in parser)
magnetometer = [0, 0, 0]
angle_degree = [0.0, 0.0, 0.0]      # degrees (from parser)


def hex_to_short(raw_bytes):
    """
    raw_bytes: iterable of 8 integers (bytes)
    returns list of 4 signed shorts
    """
    # Ensure bytes object
    b = bytes(raw_bytes)
    try:
        # little-endian signed shorts (4 values)
        return list(struct.unpack('<hhhh', b))
    except struct.error:
        # In case of malformed input, return zeros
        return [0, 0, 0, 0]


def check_sum(list_data, check_data):
    return (sum(list_data) & 0xff) == check_data


def handle_serial_data(raw_byte):
    """
    Called per-byte from the serial read loop.
    Updates global acceleration, angularVelocity, angle_degree, magnetometer.
    Returns True when a complete orientation packet (0x53) was parsed and data is ready to publish.
    """
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity

    angle_flag = False
    buff[key] = raw_byte
    key += 1

    # Sync: first byte must be 0x55
    if buff.get(0, None) != 0x55:
        # reset and wait for next frame header
        key = 0
        buff = {}
        return False

    # wait until we have full packet (11 bytes)
    if key < 11:
        return False

    # We have 11 bytes -> parse
    data_buff = list(buff.values())
    pkt_type = data_buff[1]

    if pkt_type == 0x51:
        # Accel packet
        if check_sum(data_buff[0:10], data_buff[10]):
            # bytes 2..9 hold 4 shorts; first 3 are accel
            raw_shorts = hex_to_short(data_buff[2:10])
            # scaling: original code used /32768 * 16 * 9.8  -> result in m/s^2
            acceleration = [
                raw_shorts[0] / 32768.0 * 16.0 * 9.8,
                raw_shorts[1] / 32768.0 * 16.0 * 9.8,
                raw_shorts[2] / 32768.0 * 16.0 * 9.8,
            ]
        else:
            # checksum failed
            pass

    elif pkt_type == 0x52:
        # Gyro packet
        if check_sum(data_buff[0:10], data_buff[10]):
            raw_shorts = hex_to_short(data_buff[2:10])
            # scaling: /32768 * 2000 deg/s -> convert to rad/s
            angularVelocity = [
                raw_shorts[0] / 32768.0 * 2000.0 * math.pi / 180.0,
                raw_shorts[1] / 32768.0 * 2000.0 * math.pi / 180.0,
                raw_shorts[2] / 32768.0 * 2000.0 * math.pi / 180.0,
            ]
        else:
            pass

    elif pkt_type == 0x53:
        # Angle packet (degrees)
        if check_sum(data_buff[0:10], data_buff[10]):
            raw_shorts = hex_to_short(data_buff[2:10])
            angle_degree = [
                raw_shorts[0] / 32768.0 * 180.0,
                raw_shorts[1] / 32768.0 * 180.0,
                raw_shorts[2] / 32768.0 * 180.0,
            ]
            angle_flag = True
        else:
            pass

    elif pkt_type == 0x54:
        # Magnetometer packet
        if check_sum(data_buff[0:10], data_buff[10]):
            magnetometer = hex_to_short(data_buff[2:10])
        else:
            pass

    # reset buffer for next frame
    key = 0
    buff = {}
    return angle_flag


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (radians) to quaternion [x,y,z,w]
    """
    qx = np.sin(roll / 2.0) * np.cos(pitch / 2.0) * np.cos(yaw / 2.0) - np.cos(roll / 2.0) * np.sin(
        pitch / 2.0) * np.sin(yaw / 2.0)
    qy = np.cos(roll / 2.0) * np.sin(pitch / 2.0) * np.cos(yaw / 2.0) + np.sin(roll / 2.0) * np.cos(
        pitch / 2.0) * np.sin(yaw / 2.0)
    qz = np.cos(roll / 2.0) * np.cos(pitch / 2.0) * np.sin(yaw / 2.0) - np.sin(roll / 2.0) * np.sin(
        pitch / 2.0) * np.cos(yaw / 2.0)
    qw = np.cos(roll / 2.0) * np.cos(pitch / 2.0) * np.cos(yaw / 2.0) + np.sin(roll / 2.0) * np.sin(
        pitch / 2.0) * np.sin(yaw / 2.0)
    return [qx, qy, qz, qw]


class IMUDriverNode(Node):
    def __init__(self, port_name: str):
        super().__init__('imu_driver_node')

        # IMU message template
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        # IMPORTANT: covariance arrays must be length 9 and floats
        # Tune these values to your sensor spec. They MUST be > 0 (not all zeros).
        self.imu_msg.orientation_covariance = [
            0.002, 0.0, 0.0,
            0.0, 0.002, 0.0,
            0.0, 0.0, 0.002
        ]
        self.imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]
        self.imu_msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # Launch serial reader thread (daemon so it won't block shutdown)
        self.driver_thread = threading.Thread(target=self.driver_loop, args=(port_name,), daemon=True)
        self.driver_thread.start()

        self.get_logger().info(f"IMUDriverNode started, reading from {port_name}")

    def driver_loop(self, port_name):
        """
        Open serial port and feed bytes into handle_serial_data.
        When handle_serial_data returns True (orientation packet), call imu_data().
        """
        try:
            wt_imu = serial.Serial(port=port_name, baudrate=9600, timeout=0.5)
            if wt_imu.isOpen():
                self.get_logger().info("Serial port opened successfully")
            else:
                wt_imu.open()
                self.get_logger().info("Serial port opened successfully")
        except Exception as e:
            self.get_logger().error(f"Serial port opening failure: {e}")
            return

        try:
            while rclpy.ok():
                try:
                    buff_count = wt_imu.in_waiting
                except Exception as e:
                    self.get_logger().error(f"Serial error (in_waiting): {e}")
                    break

                if buff_count and buff_count > 0:
                    try:
                        buff_data = wt_imu.read(buff_count)
                    except Exception as e:
                        self.get_logger().error(f"Serial read error: {e}")
                        break

                    # iterate bytes and call parser
                    for i in range(len(buff_data)):
                        # each element is an int 0..255
                        tag = handle_serial_data(buff_data[i])
                        if tag:
                            # orientation packet processed -> publish
                            self.imu_data()
                else:
                    # avoid busy loop
                    time.sleep(0.001)
        finally:
            try:
                wt_imu.close()
            except Exception:
                pass
            self.get_logger().info("Serial driver loop exiting")

    def imu_data(self):
        """
        Compose and publish sensor_msgs/Imu using global values set by the parser.
        This assumes the parser already scaled the raw values to SI units:
          - acceleration: m/s^2
          - angularVelocity: rad/s
          - angle_degree: degrees
        """
        # Local copy to avoid race conditions
        ax, ay, az = acceleration[0], acceleration[1], acceleration[2]
        gx, gy, gz = angularVelocity[0], angularVelocity[1], angularVelocity[2]
        ad0, ad1, ad2 = angle_degree[0], angle_degree[1], angle_degree[2]

        # Defensive checks
        for v in (ax, ay, az, gx, gy, gz, ad0, ad1, ad2):
            if not np.isfinite(v):
                self.get_logger().warn("Dropping IMU publish: non-finite value detected")
                return

        # stamp
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()

        # fill fields
        self.imu_msg.linear_acceleration.x = float(ax)
        self.imu_msg.linear_acceleration.y = float(ay)
        self.imu_msg.linear_acceleration.z = float(az)

        self.imu_msg.angular_velocity.x = float(gx)
        self.imu_msg.angular_velocity.y = float(gy)
        self.imu_msg.angular_velocity.z = float(gz)

        # angles -> radians -> quaternion
        angle_radian = [math.radians(ad0), math.radians(ad1), math.radians(ad2)]
        qx, qy, qz, qw = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

        # normalize quaternion
        qnorm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if qnorm == 0.0 or not np.isfinite(qnorm):
            self.get_logger().warn("Invalid quaternion norm, skipping publish")
            return
        qx, qy, qz, qw = qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm

        self.imu_msg.orientation.x = float(qx)
        self.imu_msg.orientation.y = float(qy)
        self.imu_msg.orientation.z = float(qz)
        self.imu_msg.orientation.w = float(qw)

        # publish
        self.imu_pub.publish(self.imu_msg)


def main():
    rclpy.init()
    node = IMUDriverNode('/dev/imu')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info("Shutting down IMUDriverNode")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
