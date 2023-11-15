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
from std_msgs.msg import String
from std_srvs.srv import Empty
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CompressedImage
from polyglotbot_interfaces.srv import GetCharacters
from polyglotbot_interfaces.srv import TranslateString

# Other libraries
import math
from enum import Enum, auto
import numpy as np


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    WAITING = auto()  # Waiting to receive go-ahead to translate
    SCANNING = auto()  # Scans the latest frame for words
    TRANSLATING = auto()  # Translates the scanned words
    PROCESSING = auto()
    COMPLETE = auto()  # When the Polyglotbot has completed translating


class Polyglotbot(Node):
    """Combines individual packages with control from a single node."""

    def __init__(self):
        # Initialize node
        super().__init__("polyglotbot")
        self.init_var()
        # This node will use Reentrant Callback Groups for nested services
        self.cbgroup = ReentrantCallbackGroup()

        # #
        # # SUBSCRIBERS
        # #
        # # Create subscriber for translated_msg messages
        # self.sub_translated_msg = self.create_subscription(
        #     String, "translated_msg", self.sub_translated_msg_callback, 10
        # )
        # self.sub_translated_msg  # Used to prevent warnings

        #
        # SERVICES
        #
        # Create service for user to call to trigger polyglotbot to run
        self.srv_start_translating = self.create_service(
            Empty,
            "start_translating",
            self.start_translating_callback,
            callback_group=self.cbgroup,
        )

        #
        # CLIENTS
        #
        # Create client for getting characters from Realsense camera
        self.cli_get_characters = self.create_client(
            GetCharacters, "get_characters", callback_group=self.cbgroup
        )
        while not self.cli_get_characters.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        # Create client for setting the target language for the translator node
        self.cli_target_language = self.create_client(
            TranslateString, "target_language", callback_group=self.cbgroup
        )
        while not self.cli_target_language.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        # Create client for sending source string for translating
        self.cli_translate_string = self.create_client(
            TranslateString, "input_msg", callback_group=self.cbgroup
        )
        while not self.cli_translate_string.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

    def init_var(self):
        """Initialize all of the polyglotbot node's variables"""
        self.state = State.WAITING
        self.dt = 1 / 100.0  # 100 Hz
        self.source_string = None
        self.target_language = None
        self.translated_string = None

    def timer_callback(self):
        """Control the Franka."""
        # If WAITING, then do nothing except wait for the start_translating service call
        if self.state is State.WAITING:
            pass

        # If SCANNING, when the new source_string and target_language is assigned begin translating
        if self.state is State.SCANNING:
            if self.source_string is not None and self.target_language is not None:
                self.state = State.TRANSLATING

        # If TRANSLATING, then send target_language and source_string to translator node
        if self.state is State.TRANSLATING:
            # self.get_logger().warn("HERE! TRANSLATING STATE")
            # Change the target language of the translator
            req = TranslateString.Request()
            req.input = self.target_language
            future_target_language = self.cli_target_language.call_async(req)
            # When target language is processed, send source string for translating in callback
            future_target_language.add_done_callback(
                self.future_target_language_callback
            )
            # Whilst waiting for both services to complete, polyglotbot is PROCESSING
            self.state = State.PROCESSING

        # Whilst PROCESSING, do nothing until both services are complete
        if self.state is State.PROCESSING:
            pass

        # If COMPLETE, then reset variables and go back to WAITING mode to redo process
        if self.state is State.COMPLETE:
            self.source_string = None
            self.target_language = None
            self.translated_string = None
            self.state = State.WAITING

    #
    # SERVICE CALLBACKS
    #
    async def start_translating_callback(self, request, response):
        self.get_logger().info("Polyglotbot beginning translation...")
        self.state = State.SCANNING
        req = GetCharacters.Request()
        result = await self.cli_get_characters.call_async(req)
        # Test to make sure that both a target_language and source_string are identified
        try:
            self.target_language = result.words[0]
            self.source_string = result.words[1]
        # Go back to the WAITING state if test fails
        except Exception as e:
            self.get_logger().warn("Failed to identify a target_language and source_string")
            self.state = State.WAITING
        return response

    #
    # FUTURE CALLBACKS
    #
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
        future_translated_string.add_done_callback(
            self.future_translated_string_callback
        )

    def future_translated_string_callback(self, future_translated_string):
        self.translated_string = future_translated_string.result().output
        self.get_logger().info("Translated string: %s" % self.translated_string)
        self.state = State.COMPLETE


def entry_point(args=None):
    rclpy.init(args=args)
    node = Polyglotbot()
    rclpy.spin(node)
    rclpy.shutdown()
