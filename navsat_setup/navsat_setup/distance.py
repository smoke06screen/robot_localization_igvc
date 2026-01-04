import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
import serial
import math
import time

class DistanceGPS(Node):
    def __init__(self):
        super().__init__("distance_gps")

        self.curr = []
        self.create_subscription(NavSatFix, "/gps/coords", self.gps_callback, 10)
        
    def gps_callback(self, msg):
        x = msg.latitude
        y = msg.longitude
        