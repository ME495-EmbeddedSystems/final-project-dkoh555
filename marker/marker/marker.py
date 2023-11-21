"""
Pick up marker.

Publishers:
  + abc (type) - description

Services:
  + abc (type) - description

Parameter
  + abc (type) - description
-
"""

# ROS libraries
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge

# Interfaces
from std_msgs.msg import String, Float32
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CompressedImage
from polyglotbot_interfaces.srv import GetCharacters

# Other libraries
import math
from enum import Enum, auto
from paddleocr import PaddleOCR
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    TBD = auto()  # To be determined


class Marker(Node):
    """USe computer vision to determine handwritten words on a whiteboard."""

    def __init__(self):
        super().__init__("marker")

        # define parameters
        self.dt = 1/10.0  # 10 Hz, 30 Hz max

        # Services

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        pass


def entry_point(args=None):
    rclpy.init(args=args)
    node = Marker()
    rclpy.spin(node)
    rclpy.shutdown()
