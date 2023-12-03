"""
Combines individual packages with control from a single node.

Subscribers:
    + person_detect (std_msgs/msg/Float32) - Average number of people detected
    in the frame over the last 2 seconds
    + april_tag_coords (polyglotbot_interfaces/msg/AprilCoords) - The
    coordinates of april tags
    + writer_state (std_msgs/msg/String) - State of the writer node

Clients:
    + get_characters (polyglotbot_interfaces/srv/GetCharacters) - Returns a
    string of characters from the image
    + target_language (polyglotbot_interfaces/srv/TranslateString) - Sets the
    target language for the translator node
    + input_msg (polyglotbot_interfaces/srv/TranslateString) - Sends the source
    string to the translator node
    + string2waypoint (polyglotbot_interfaces/srv/StringToWaypoint) - Creates
    waypoints from the translated words
    + write (polyglotbot_interfaces/srv/Write) - Writes the translated words
    + calibrate (std_srvs/srv/Empty) - Moves the robot to the calibration pose
    + homing (std_srvs/srv/Empty) - Moves the robot to the home pose
    + change_writer_state (std_srvs/srv/Empty) - Changes the state of the
    writer node

Services:
    + start_translating (std_srvs/srv/Empty) - Starts the translation process
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
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CompressedImage
from polyglotbot_interfaces.msg import CharacterPath, AprilCoords
from polyglotbot_interfaces.srv import (
    GetCharacters,
    TranslateString,
    StringToWaypoint,
    Write,
    SpeakText,
    GrabPen,
    RecordingString,
)

# Other libraries
import math
from enum import Enum, auto
import numpy as np


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    MARKER = auto(),  # Picks up the marker
    WAITING = auto(),  # Waiting to receive go-ahead to translate
    PERSON = auto(),  # Detected person in frame
    SCANNING = auto(),  # Scans the latest frame for words
    TRANSLATING = auto(),  # Translates the scanned words
    CALIBRATE = auto(),  # Move to calibrate pose
    HOMING = auto(),  # Move to home pose
    DETECTING = auto(),  # Detect the april tag
    PROCESSING = auto(),  # Waiting for a service to complete
    CREATE_WAYPOINTS = auto(),  # Create waypoints from translated words
    DRAWING = auto(),  # Drawing the waypoints
    SPEAKING = auto(),  # Speak the translated words
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
        while self.count_publishers("read_text/person_detect") < 1:
            self.get_logger().info("waiting for person_detect publisher", once=True)

        self.get_apriltag = self.create_subscription(AprilCoords, "april_tag_coords", self.apriltags_callback, 10)
        while self.count_publishers("tag_detect/april_tag_coords") < 1:
            self.get_logger().info("waiting for april_tag_coords publisher", once=True)

        self.get_writer_state = self.create_subscription(String, "writer_state", self.writer_state_callback, 10)
        while self.count_publishers("writer_state") < 1:
            self.get_logger().info("waiting for writer_state publisher", once=True)

        # Services
        self.srv_start_translating = self.create_service(Empty, "start_translating", self.start_translating_callback, callback_group=self.cbgroup)

        self.srv_receive_recording = self.create_service(RecordingString, "receive_recording", self.receive_recording_callback, callback_group=self.cbgroup)

        # Clients
        self.get_characters_client = self.create_client(GetCharacters, "get_characters", callback_group=self.cbgroup)
        while not self.get_characters_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(" get_characters service not available, waiting again...")

        self.cli_target_language = self.create_client(TranslateString, "target_language", callback_group=self.cbgroup)
        while not self.cli_target_language.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("target_language service not available, waiting again...")

        self.cli_translate_string = self.create_client(TranslateString, "input_msg", callback_group=self.cbgroup)
        while not self.cli_translate_string.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("input_msg service not available, waiting again...")

        self.speak_client = self.create_client(SpeakText, "speak", callback_group=self.cbgroup)
        while not self.speak_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("speak service not available, waiting again ...")

        self.waypoints_client = self.create_client(StringToWaypoint, "string2waypoint", callback_group=self.cbgroup)
        while not self.waypoints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("string2waypoint service not available, waiting again...")

        self.write_client = self.create_client(Write, "write", callback_group=self.cbgroup)
        while not self.write_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("write service not available, waiting again...")

        self.grab_pen_client = self.create_client(GrabPen, "grab_pen", callback_group=self.cbgroup)
        while not self.grab_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("grab_pen service not available, waiting again ...")

        self.calibrate_client = self.create_client(Empty, "calibrate", callback_group=self.cbgroup)
        while not self.calibrate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("calibrate service not available, waiting again ...")

        self.homing_client = self.create_client(Empty, "homing")
        while not self.homing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("homing service not available, waiting again ...")

        self.write_state_client = self.create_client(Empty, "change_writer_state")
        while not self.write_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("change_writer_state service not available, waiting again ...")

        # Timer
        self.tmr = self.create_timer(self.dt, self.timer_callback)

    def init_var(self):
        """Initialize all of the polyglotbot node's variables"""
        self.state = State.MARKER
        self.dt = 1 / 100.0  # 100 Hz
        self.source_string = None
        self.target_language = None
        self.translated_string = None

        self.char_num = 0
        self.waypoints = []

        self.num_people = 0.0
        self.april_cords_received = False
        self.writer_state = ""
        self.m: Marker = None

    def add_person_detection(self, num_person):
        self.m = Marker()
        self.m.header.frame_id = "panda_link0"
        self.m.header.stamp = self.get_clock().now().to_msg()
        self.m.id = 15
        self.m.type = Marker.TEXT_VIEW_FACING
        self.m.action = Marker.ADD
        self.m.scale.x = 1.0
        self.m.scale.y = 1.0
        self.m.scale.z = 1.0
        self.m.text = f"Number of person: {num_person}"
        self.m.lifetime.sec = 1
        self.m.pose.position.x = 0.0
        self.m.pose.position.y = 0.4
        self.m.pose.position.z = 0.7
        self.m.pose.orientation.x = 0.0
        self.m.pose.orientation.y = 0.0
        self.m.pose.orientation.z = 0.7071068
        self.m.pose.orientation.w = 0.7071068
        self.m.color.r = 1.0
        self.m.color.g = 1.0
        self.m.color.b = 1.0
        self.m.color.a = 0.8
        self.markers.publish(self.m)

    def remove_person_detection(self):
        if self.m:
            self.m.action = Marker.DELETE
            self.markers.publish(self.m)

    def timer_callback(self):
        """Control the Franka."""
        self.get_logger().debug(f"State: {self.state}")

        if self.state == State.CALIBRATE:
            self.get_logger().info("Calibrating")
            future_calibrate = self.calibrate_client.call_async(Empty.Request())
            future_calibrate.add_done_callback(self.future_calibrate_callback)

            self.state = State.PROCESSING

        elif self.state == State.MARKER:
            self.get_logger().info("Grabbing Marker")
            marker_point = Point()
            marker_point.x = 0.452
            marker_point.y = 0.0
            marker_point.z = 0.175
            my_request = GrabPen.Request()
            my_request.position = marker_point
            future_grab_pen = self.grab_pen_client.call_async(my_request)
            future_grab_pen.add_done_callback(self.future_grab_pen_callback)

            self.state = State.PROCESSING

        elif self.state == State.HOMING:
            self.get_logger().info("Homing")
            future_homing = self.homing_client.call_async(Empty.Request())
            future_homing.add_done_callback(self.future_homing_callback)

            self.state = State.PROCESSING

        elif self.state == State.DETECTING:
            if self.april_cords_received and self.writer_state == "State.DONE":
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
            future_get_characters = self.get_characters_client.call_async(
                GetCharacters.Request()
            )
            future_get_characters.add_done_callback(self.future_get_characters_callback)

            self.state = State.PROCESSING

        elif self.state == State.TRANSLATING:
            # Translate the scanned words
            self.get_logger().info("Translating")
            req = TranslateString.Request()
            req.input = self.target_language
            future_target_language = self.cli_target_language.call_async(req)
            future_target_language.add_done_callback(
                self.future_target_language_callback
            )

            self.state = State.PROCESSING

        elif self.state == State.SPEAKING:
            req = SpeakText.Request(text=self.translated_string)
            future_speak = self.speak_client.call_async(request=req)
            future_speak.add_done_callback(self.future_speak_callback)

            self.state = State.PROCESSING

        elif self.state == State.CREATE_WAYPOINTS:
            # Create waypoints from the translated words
            self.target_language = self.target_language.lower()
            req = StringToWaypoint.Request()
            req.language = self.target_language
            if self.char_num < len(self.translated_string):
                if self.translated_string[self.char_num] != " ":
                    req.text = self.translated_string[self.char_num]
                    future_waypoints = self.waypoints_client.call_async(req)
                    future_waypoints.add_done_callback(self.future_waypoints_callback)
                    self.state = State.PROCESSING
                else:
                    self.char_num = self.char_num + 1
                    msg = CharacterPath(points=[])
                    self.waypoints.append(msg)
            else:
                self.state = (
                    State.DRAWING
                )  # Change this to DRAWING when functionality is added
                self.get_logger().info("Waypoints: %s" % self.waypoints)

        elif self.state == State.DRAWING:
            self.get_logger().info("Drawing")
            req = Write.Request(characters=self.waypoints)
            future_write = self.write_client.call_async(req)
            future_write.add_done_callback(self.future_write_callback)
            self.state = State.PROCESSING

        elif self.state == State.COMPLETE:
            # Reset the node
            self.source_string = None
            self.target_language = None
            self.translated_string = None
            self.char_num = 0
            self.waypoints = []
            self.state = State.WAITING

        elif self.state == State.PROCESSING:
            if self.writer_state == "State.DONEWRITING":
                self.get_logger().info("DRAWING DONE")
                future_done_writing = self.write_state_client.call_async(Empty.Request())
                future_done_writing.add_done_callback(self.future_done_writing_callback)
                # self.state = State.COMPLETE

    # Subscriber Callbacks
    # #############################################################################################################

    def writer_state_callback(self, msg):
        """
        Callback for the state of the writer node.

        Args:
            msg (String): State of the writer node.
        """
        self.writer_state = msg.data

    def apriltags_callback(self, msg):
        """
        Callback for when apriltag has been detected.

        Args:
            msg (AprilCoords): The coordinates of apirl tags
        """
        if not self.april_cords_received:
            self.april_cords_received = True

            self.get_logger().info(f"apriltags received")

    def detect_person_callback(self, msg):
        """Callback for when a person is detected in the frame."""
        self.num_people = msg.data

        if self.state == State.WAITING or self.state == State.PERSON:
        # if self.state == State.PERSON:
            # self.remove_person_detection()
            # self.add_person_detection(self.num_people)
            self.get_logger().info(f"Number of people detected: {self.num_people}")

    # Service Callbacks
    # #############################################################################################################

    def start_translating_callback(self, request, response):
        self.get_logger().info("Polyglotbot beginning translation...")
        self.state = State.SCANNING
        return response
    
    def start_translating_callback(self, request, response):
        response.output = "Polyglotbot currently not receiving inputs"
        if self.state==State.WAITING:
            self.target_language = request.target_language
            self.source_string = request.input
            response.output= "Recording target language and message received"
            self.state = State.TRANSLATING
        return response

    # Future Callbacks
    # #############################################################################################################

    def future_calibrate_callback(self, future_calibrate):
        self.get_logger().info(f"{future_calibrate.result()}")

        self.state = State.DETECTING

    def future_homing_callback(self, future_homing):
        self.get_logger().info(f"{future_homing.result()}")

        self.state = State.WAITING

    def future_get_characters_callback(self, future_get_characters):
        try:
            self.target_language = future_get_characters.result().words[0]
            self.source_string = future_get_characters.result().words[1]
            self.state = State.TRANSLATING
        except Exception as e:
            # Go back to the WAITING state if test fails
            self.get_logger().warn(
                "Failed to identify a target_language and source_string"
            )
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
        future_translated_string.add_done_callback(
            self.future_translated_string_callback
        )

    def future_translated_string_callback(self, future_translated_string):
        self.translated_string = future_translated_string.result().output
        self.get_logger().info("Translated string: %s" % self.translated_string)
        self.state = State.SPEAKING
        # self.state = State.CREATE_WAYPOINTS

    def future_speak_callback(self, future_speak):
        result = future_speak.result()
        self.get_logger().info(f"Text spoken success: {result.success}")
        self.state = State.CREATE_WAYPOINTS

    def future_waypoints_callback(self, future_waypoints):
        self.get_logger().info(f"char_num: {self.char_num}")
        # Change self.waypoints to CharacterPath msg type
        msg = CharacterPath(points=future_waypoints.result().waypoints)
        self.waypoints.append(msg)
        self.char_num = self.char_num + 1
        self.state = State.CREATE_WAYPOINTS

    def future_write_callback(self, future_write):
        self.get_logger().info(f"{future_write.result().success}")
        # self.state = State.COMPLETE

    def future_done_writing_callback(self, future_done_writing):
        # self.get_logger().info(f"{future_done_writing.result()}")
        self.state = State.COMPLETE

    def future_grab_pen_callback(self, future_grab_pen):
        self.get_logger().info(f"{future_grab_pen.result()}")
        # self.state = State.WAITING


def entry_point(args=None):
    rclpy.init(args=args)
    node = Polyglotbot()
    rclpy.spin(node)
    rclpy.shutdown()
