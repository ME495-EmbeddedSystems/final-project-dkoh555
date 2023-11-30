"""
Combines individual packages with control from a single node.

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
from rclpy.task import Future
from tf2_ros import Buffer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup

# Interfaces
from std_msgs.msg import String, Float32
from std_srvs.srv import Empty
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CompressedImage
from polyglotbot_interfaces.msg import CharacterPath, AprilCoords
from polyglotbot_interfaces.srv import GetCharacters
from polyglotbot_interfaces.srv import TranslateString, StringToWaypoint, Write

# Other libraries
import math
from enum import Enum, auto
import numpy as np


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    WAITING = auto(),  # Waiting to receive go-ahead to translate
    PERSON = auto(),  # Detected person in frame
    SCANNING = auto(),  # Scans the latest frame for words
    TRANSLATING = auto(),  # Translates the scanned words
    CALIBRATE = auto(), # Move to calibrate pose
    HOMING = auto(), # Move to home pose
    DETECTING = auto(), # Detect the april tag
    PROCESSING = auto(),
    CREATE_WAYPOINTS = auto(),  # Create waypoints from translated words
    DRAWING = auto(),  # Drawing the waypoints
    COMPLETE = auto()  # When the Polyglotbot has completed translating


class Polyglotbot(Node):
    """Combines individual packages with control from a single node."""

    def __init__(self):
        # Initialize node
        super().__init__("polyglotbot")

        self.init_var()

        # This node will use Reentrant Callback Groups for nested services
        self.cbgroup = ReentrantCallbackGroup()

        # Subscribers
        self.detect_person = self.create_subscription(Float32, "person_detect", self.detect_person_callback, 10)
        self.get_apriltag = self.create_subscription(AprilCoords, 'april_tag_coords', self.apriltags_callback, 10)

        # SERVICES

        # Create service for user to call to trigger polyglotbot to run
        self.srv_start_translating = self.create_service(Empty, "start_translating", self.start_translating_callback, callback_group=self.cbgroup)

        # CLIENTS

        # Create client for getting characters from Realsense camera
        self.get_characters_client = self.create_client(GetCharacters, "get_characters", callback_group=self.cbgroup)
        while not self.get_characters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(" get_characters service not available, waiting again...")

        # Create client for setting the target language for the translator node
        self.cli_target_language = self.create_client(TranslateString, "target_language", callback_group=self.cbgroup)
        while not self.cli_target_language.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("target_language service not available, waiting again...")

        # Create client for sending source string for translating
        self.cli_translate_string = self.create_client(TranslateString, "input_msg", callback_group=self.cbgroup)
        while not self.cli_translate_string.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("input_msg service not available, waiting again...")

        self.waypoints_client = self.create_client(StringToWaypoint, "string2waypoint", callback_group=self.cbgroup)
        while not self.waypoints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("string2waypoint service not available, waiting again...")

        self.write_client = self.create_client(Write, "write", callback_group=self.cbgroup)
        while not self.write_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("write service not available, waiting again...")

        self.calibrate_client = self.create_client(Empty, "calibrate", callback_group=self.cbgroup)
        while not self.calibrate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("calibrate service not available, waiting again ...")
            
        self.homing_client = self.create_client(Empty, 'homing')

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

    def init_var(self):
        """Initialize all of the polyglotbot node's variables"""
        self.state = State.CALIBRATE
        self.dt = 1 / 100.0  # 100 Hz
        self.source_string = None
        self.target_language = None
        self.translated_string = None

        self.char_num = 0
        self.waypoints = []

        self.num_people = 0.0
        self.april_cords_received = False

    def timer_callback(self):
        """Control the Franka."""

        if self.state == State.CALIBRATE:
            self.get_logger().info("Calibrating")
            future_calibrate = self.calibrate_client.call_async(Empty.Request())
            future_calibrate.add_done_callback(self.future_calibrate_callback)
            
            self.state = State.PROCESSING

        elif self.state == State.HOMING:
            self.get_logger().info('Homing')
            future_homing = self.homing_client.call_async(Empty.Request())
            future_homing.add_done_callback(self.future_homing_callback)
            
            self.state = State.PROCESSING
            
        elif self.state == State.DETECTING:
            if self.april_cords_received:
                self.state = State.HOMING

        elif self.state == State.WAITING:
            # Wait for person to be detected
            self.get_logger().info("Waiting", once=True)
            if self.num_people > 0.3:
                self.state = State.PERSON

        elif self.state == State.PERSON:
            # Wait for person to leave
            self.get_logger().info("Person Detected", once=True)
            if self.num_people < 0.1:
                self.get_logger().info("Polyglotbot beginning translation...")
                self.state = State.SCANNING

        elif self.state == State.SCANNING:
            # Scan the frame for words
            self.get_logger().info("Scanning")
            future_get_characters = self.get_characters_client.call_async(GetCharacters.Request())
            future_get_characters.add_done_callback(self.future_get_characters_callback)

            self.state = State.PROCESSING

        elif self.state == State.TRANSLATING:
            # Translate the scanned words
            self.get_logger().info("Translating")
            req = TranslateString.Request()
            req.input = self.target_language
            future_target_language = self.cli_target_language.call_async(req)
            future_target_language.add_done_callback(self.future_target_language_callback)

            self.state = State.PROCESSING

        elif self.state == State.CREATE_WAYPOINTS:
            # Create waypoints from the translated words
            self.target_language = self.target_language.lower()
            req = StringToWaypoint.Request()
            req.language = self.target_language
            if self.char_num < len(self.translated_string):
                if self.translated_string[self.char_num] != ' ':
                    req.text = self.translated_string[self.char_num]
                    future_waypoints = self.waypoints_client.call_async(req)
                    future_waypoints.add_done_callback(self.future_waypoints_callback)
                    self.state = State.PROCESSING
                else:
                    self.char_num = self.char_num + 1
                    msg = CharacterPath(points=[])
                    self.waypoints.append(msg)
            else:
                self.state = State.DRAWING # Change this to DRAWING when functionality is added
                self.get_logger().info("Waypoints: %s" % self.waypoints)

        elif self.state == State.DRAWING:
            self.get_logger().info("Drawing")
            req = Write.Request(characters=self.waypoints)
            future_write = self.write_client.call_async(req)
            future_write.add_done_callback(self.future_write_callback)
            self.state = State.PROCESSING

        elif self.state is State.COMPLETE:
            # Reset the node
            self.source_string = None
            self.target_language = None
            self.translated_string = None
            self.char_num = 0
            self.waypoints = []
            self.state = State.WAITING
            
        elif self.state == State.PROCESSING:
            pass

    # Subscriber Callbacks
    # #############################################################################################################

    def apriltags_callback(self, msg):
        if not self.april_cords_received:
            self.april_cords_received = True
            
            self.get_logger().info(f'apriltags received')

    def detect_person_callback(self, msg):
        """Callback for when a person is detected in the frame."""
        self.num_people = msg.data

        if self.state == State.WAITING or self.state == State.PERSON:
            self.get_logger().info(f"Number of people detected: {self.num_people}")

    # Service Callbacks
    # #############################################################################################################

    def start_translating_callback(self, request, response):
        self.get_logger().info("Polyglotbot beginning translation...")
        self.state = State.SCANNING
        return response

    # Future Callbacks
    # #############################################################################################################

    def future_calibrate_callback(self, future_calibrate):
        self.get_logger().info(f'{future_calibrate.result()}')
        
        self.state = State.DETECTING
        
    def future_homing_callback(self, future_homing):
        self.get_logger().info(f'{future_homing.result()}')
        
        self.state = State.WAITING

    def future_get_characters_callback(self, future_get_characters):
        try:
            self.target_language = future_get_characters.result().words[0]
            self.source_string = future_get_characters.result().words[0]
            self.state = State.TRANSLATING
        except Exception as e:
            # Go back to the WAITING state if test fails
            self.get_logger().warn("Failed to identify a target_language and source_string")
            self.state = State.WAITING

    def future_target_language_callback(self, future_target_language):
        self.get_logger().info("%s" % future_target_language.result().output)
        if future_target_language.result().output == "INVALID LANGUAGE":
            self.state = State.COMPLETE
            return
        # Send the source string to the translator node
        req = TranslateString.Request()
        req.input = self.source_string
        future_translated_string = self.cli_translate_string.call_async(req)
        # Once done translating, switch to polyglotbot back to COMPLETE state
        future_translated_string.add_done_callback(self.future_translated_string_callback)

    def future_translated_string_callback(self, future_translated_string):
        self.translated_string = future_translated_string.result().output
        self.get_logger().info("Translated string: %s" % self.translated_string)
        self.state = State.CREATE_WAYPOINTS

    def future_waypoints_callback(self, future_waypoints):
        self.get_logger().info(f"char_num: {self.char_num}")
        # Change self.waypoints to CharacterPath msg type
        msg = CharacterPath(points=future_waypoints.result().waypoints)
        self.waypoints.append(msg)
        self.char_num = self.char_num + 1
        self.state = State.CREATE_WAYPOINTS

    def future_write_callback(self, future_write):
        self.state = State.COMPLETE


def entry_point(args=None):
    rclpy.init(args=args)
    node = Polyglotbot()
    rclpy.spin(node)
    rclpy.shutdown()
