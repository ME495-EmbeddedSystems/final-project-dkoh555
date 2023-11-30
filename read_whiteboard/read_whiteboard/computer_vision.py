"""
Determine what robot should do based on computer vision.

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


class ComputerVision(Node):
    """USe computer vision to determine handwritten words on a whiteboard."""

    def __init__(self):
        super().__init__("computer_vision")

        # define parameters
        self.dt = 1 / 10.0  # 10 Hz, 30 Hz max

        self.frame = Image().data
        # self.temp_frame = np.asanyarray(cv2.imread("filename.png"))
        # self.get_logger().info(f"Got image: {self.temp_frame}")
        # print(self.temp_frame)

        self.bridge = CvBridge()
        self.ocr = PaddleOCR(use_angle_cls=True)
        self.model = YOLO("yolov8n.pt")  # pass any model type

        self.ave_num_people = 0.0
        self.num_people = np.zeros(20)

        # Publishers
        self.person_detect = self.create_publisher(
            Float32, "person_detect", QoSProfile(depth=10)
        )

        # Subscribers
        self.get_image = self.create_subscription(
            Image,
            "camera/color/image_raw",
            self.get_image_callback,
            QoSProfile(depth=10),
        )
        while self.count_publishers("camera/color/image_raw") < 1:
            self.get_logger().info(
                "waiting for camera/color/image_raw publisher", once=True
            )

        # Services
        self.get_characters = self.create_service(
            GetCharacters, "get_characters", self.get_characters_callback
        )

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # self.setup_camera()

    def timer_callback(self):
        """Control the camera and computer vision."""

        # print(self.frame)
        # self.get_logger().info(f"Got image: {self.frame}")
        temp_frame = self.frame

        # results = self.model.predict(source=np.asanyarray(self.frame), stream=True, classes=[0])
        results = self.model.predict(
            source=temp_frame, stream=True, classes=[0], verbose=False
        )
        for result in results:
            # self.get_logger().info(f"Results: {result.__len__()}")
            # average num people across 20 frames (2 seconds)
            self.num_people = np.delete(self.num_people, 0)
            self.num_people = np.append(self.num_people, result.__len__())
            self.average_num_people = np.average(self.num_people)

        self.person_detect.publish(Float32(data=self.average_num_people))

    def get_image_callback(self, msg):
        """Get an image from the camera."""
        self.frame = self.bridge.imgmsg_to_cv2(msg)
        # self.frame = np.asanyarray(msg.data)
        # self.get_logger().info(f"Got image: {self.frame}")

    def get_characters_callback(self, request, response):
        """Find characters in image and return string."""

        self.temp_frame = self.frame

        cv2.imwrite("node_pic.png", self.temp_frame)
        # cv2.imwrite("filename3.png", self.frame)

        # reader = easyocr.Reader(['es', 'en', 'de', 'fr'], gpu=False)  # this needs to run only once to load the model into memory
        # results = reader.readtext(self.temp_frame, detail=0)

        send_results = []
        result = self.ocr.ocr("node_pic.png", cls=True)
        if result[0] is None:
            self.get_logger().info("No text detected")
            send_results = []
        else:
            for idx in range(len(result)):
                res = result[idx]
                for line_num, line in enumerate(res):
                    self.get_logger().info(f"Text: {line}")
                    # send_results.append(line[1][0])
                    if line_num == 0:
                        send_results.append(line[1][0])
                    elif line_num == 1:
                        send_results.append(line[1][0])
                    else:
                        send_results[1] = send_results[1] + " " + line[1][0]

        # for result in results:
        #     self.get_logger().info(f"Text: {result}")

        # response.words = ["hello", "world"]
        response.words = send_results

        return response


def entry_point(args=None):
    rclpy.init(args=args)
    node = ComputerVision()
    rclpy.spin(node)
    rclpy.shutdown()
