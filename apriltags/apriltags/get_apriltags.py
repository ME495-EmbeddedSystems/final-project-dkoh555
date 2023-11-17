import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Quaternion, Pose, TransformStamped, Point
from shape_msgs.msg import SolidPrimitive
from .move_robot import MoveRobot
from enum import Enum, auto
import numpy as np
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from polyglotbot_interfaces.msg import AprilCoords
from polyglotbot_interfaces.srv import SaveApril


class State(Enum):
    WAITING = auto()
    LOOK_UP_TRANSFORM = auto()
    ADD_BOX = auto()


class GetAprilTags(Node):
    """Gets April Tag information"""

    def __init__(self):
        super().__init__("get_apriltags")

        self.state = State.LOOK_UP_TRANSFORM
        self.cb_group = ReentrantCallbackGroup()

        self.timer = self.create_timer(1.0 / 100.0, self.timer_callback)

        # Initialize transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Make static transform between camera_color_optical frame and panda_hand
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # self.broadcast_static_transform()
        # Camera in the frame of the hand
        hand_camera_tf = TransformStamped()
        hand_camera_tf.header.stamp = self.get_clock().now().to_msg()
        hand_camera_tf.header.frame_id = "panda_hand"
        hand_camera_tf.child_frame_id = "camera_link"

        hand_camera_tf.transform.translation.x = 50e-3
        hand_camera_tf.transform.translation.y = 15e-3
        hand_camera_tf.transform.translation.z = 65e-3

        hand_camera_tf.transform.rotation = Quaternion(
            x=0.7071068, y=0.0, z=0.7071068, w=0.0
        )

        self.tf_static_broadcaster.sendTransform(hand_camera_tf)

        # Call calibrate service immediately after launching node
        self.client_calibrate = self.create_client(
            Empty, "calibrate", callback_group=self.cb_group
        )

        self.client_calibrate.call_async(Empty.Request())

        while not self.client_calibrate.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Calibrate service not available, waiting again ...")

        # while not self.tf_buffer.can_transform(
        #     "tag36h11:1", "panda_link0", rclpy.time.Time()
        # ):
        #     self.get_logger().info("Could not get transform tag 1, waiting again ...")

        # while not self.tf_buffer.can_transform(
        #     "tag36h11:3", "panda_link0", rclpy.time.Time()
        # ):
        #     self.get_logger().info("Could not get transform tag 3, waiting again ...")

        # while not self.tf_buffer.can_transform(
        #     "tag36h11:4", "panda_link0", rclpy.time.Time()
        # ):
        #     self.get_logger().info("Could not get transform tag 4, waiting again ...")

        self.publish_april_coords = self.create_publisher(
            AprilCoords, "april_tag_coords", 10
        )

        self.top_left_position = []
        self.bottom_left_position = []
        self.bottom_right_position = []

    # Tag 3 is top left, tag 4 is bottom left, tag 1 is bottom right
    #########################################################################################################################
    def srv_save_april_callback(self, request, response):
        if self.p1 and self.p2 and self.p3:
            self.msg_pub = AprilCoords()
            self.msg_pub.p1 = self.p1
            self.msg_pub.p2 = self.p2
            self.msg_pub.p3 = self.p3

            response.success = True
        else:
            response.success = False

        return response

    def timer_callback(self):
        # Publish the x,y,z of each AprilTag
        if self.state == State.LOOK_UP_TRANSFORM:
            ### TOP LEFT
            try:
                t = self.tf_buffer.lookup_transform(
                    "tag36h11:3",
                    "panda_link0",
                    rclpy.time.Time(),
                )

                self.position_TL = t.transform.translation
                self.rotation_TL = t.transform.rotation

                self.T_0_TL = self.matrix_from_rot_and_trans(
                    self.rotation_TL, self.position_TL
                )
                self.top_left_position = self.T_0_TL[:3, 3]

                # self.get_logger().info(f"{self.T_0_TL}")

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_link0"} to {"tag36h11:3"}: {ex}',
                    once=True,
                )
                return

            ### BOTTOM LEFT
            try:
                t = self.tf_buffer.lookup_transform(
                    "tag36h11:4",
                    "panda_link0",
                    rclpy.time.Time(),
                )

                self.position_BL = t.transform.translation
                self.rotation_BL = t.transform.rotation

                self.T_0_BL = self.matrix_from_rot_and_trans(
                    self.rotation_BL, self.position_BL
                )
                self.bottom_left_position = self.T_0_BL[:3, 3]

                # self.get_logger().info(f"{self.T_0_BL}")

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_link0"} to {"tag36h11:4"}: {ex}',
                    once=True,
                )
                return

            ### BOTTOM RIGHT
            try:
                t = self.tf_buffer.lookup_transform(
                    "tag36h11:1",
                    "panda_link0",
                    rclpy.time.Time(),
                )

                self.position_BR = t.transform.translation
                self.rotation_BR = t.transform.rotation

                self.T_0_BR = self.matrix_from_rot_and_trans(
                    self.rotation_BR, self.position_BR
                )
                self.bottom_right_position = self.T_0_BR[:3, 3]

                # self.get_logger().info(f"{self.T_0_BR}")

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_link0"} to {"tag36h11:1"}: {ex}',
                    once=True,
                )
                return

            if (
                any(self.top_left_position)
                and any(self.bottom_left_position)
                and any(self.bottom_right_position)
            ):
                msg = AprilCoords()
                msg.p1 = self.p1
                msg.p2 = self.p2
                msg.p3 = self.p3

                self.publish_april_coords.publish(msg)

            # ### TOP LEFT
            # try:
            #     s1 = self.tf_buffer.lookup_transform(
            #         "tag36h11:3",
            #         "panda_hand",
            #         rclpy.time.Time(),
            #     )

            #     position_s1 = s1.transform.translation
            #     rotation_s1 = s1.transform.rotation

            #     t_0_h_s1 = self.matrix_from_rot_and_trans(
            #         self.rotation_hand, self.position_hand
            #     )
            #     t_h_tag_s1 = self.matrix_from_rot_and_trans(rotation_s1, position_s1)

            #     t_0_top_left = np.matmul(t_0_h_s1, t_h_tag_s1)
            #     self.top_left_position = t_0_top_left[:3, 3]

            #     # self.get_logger().info(f"{t_0_top_left}")
            #     # self.get_logger().info(f"{self.top_left_position}")

            # except TransformException as ex:
            #     self.get_logger().info(
            #         f'Could not transform {"panda_hand"} to {"tag36h11:3"}: {ex}',
            #         once=True,
            #     )
            #     return

            # ### BOTTOM LEFT
            # try:
            #     s2 = self.tf_buffer.lookup_transform(
            #         "tag36h11:4",
            #         "panda_hand",
            #         rclpy.time.Time(),
            #     )

            #     position_s2 = s2.transform.translation
            #     rotation_s2 = s2.transform.rotation

            #     t_0_h_s2 = self.matrix_from_rot_and_trans(
            #         self.rotation_hand, self.position_hand
            #     )
            #     t_h_tag_s2 = self.matrix_from_rot_and_trans(rotation_s2, position_s2)

            #     t_0_bottom_left = np.matmul(t_0_h_s2, t_h_tag_s2)
            #     self.bottom_left_position = t_0_bottom_left[:3, 3]

            #     # self.get_logger().info(f"{t_0_bottom_left}")
            #     # self.get_logger().info(f"{self.bottom_left_position}")

            # except TransformException as ex:
            #     self.get_logger().info(
            #         f'Could not transform {"panda_hand"} to {"tag36h11:4"}: {ex}',
            #         once=True,
            #     )
            #     return

            # ### BOTTOM RIGHT
            # try:
            #     s3 = self.tf_buffer.lookup_transform(
            #         "tag36h11:1",
            #         "panda_hand",
            #         rclpy.time.Time(),
            #     )

            #     position_s3 = s3.transform.translation
            #     rotation_s3 = s3.transform.rotation

            #     t_0_h_s3 = self.matrix_from_rot_and_trans(
            #         self.rotation_hand, self.position_hand
            #     )
            #     t_h_tag_s3 = self.matrix_from_rot_and_trans(rotation_s3, position_s3)

            #     t_0_bottom_right = np.matmul(t_0_h_s3, t_h_tag_s3)
            #     self.bottom_right_position = t_0_bottom_right[:3, 3]

            #     # self.get_logger().info(f"{t_0_bottom_right}")
            #     # self.get_logger().info(f"{self.bottom_right_position}")

            # except TransformException as ex:
            #     self.get_logger().info(
            #         f'Could not transform {"panda_hand"} to {"tag36h11:1"}: {ex}',
            #         once=True,
            #     )
            #     return

            # if (
            #     any(self.top_left_position)
            #     and any(self.bottom_left_position)
            #     and any(self.bottom_right_position)
            # ):
            #     msg = AprilCoords()
            #     msg.p1 = Point(
            #         x=self.top_left_position[0],
            #         y=self.top_left_position[1],
            #         z=self.top_left_position[2],
            #     )
            #     msg.p2 = Point(
            #         x=self.bottom_left_position[0],
            #         y=self.bottom_left_position[1],
            #         z=self.bottom_left_position[2],
            #     )
            #     msg.p3 = Point(
            #         x=self.bottom_right_position[0],
            #         y=self.bottom_right_position[1],
            #         z=self.bottom_right_position[2],
            #     )

            #     self.publish_april_coords.publish(msg)

    #########################################################################################################################

    def matrix_from_rot_and_trans(self, rotation: Quaternion, translation: Vector3):
        """
        Construct the transformation matrix from rotation and translation.

        Args:
        ----
            rotation (Quaternion): Quaternion object representing the rotation
            translation (Point): Point object representing the

        Returns:
        -------
            numpy.ndarray: The resulting homogenous transformation matrix.

        """
        # Extract the values from Q and T
        q0 = rotation.x
        q1 = rotation.y
        q2 = rotation.z
        q3 = rotation.w

        px = translation.x
        py = translation.y
        pz = translation.z

        # First row of the rotation matrix
        r00 = 2.0 * (q0 * q0 + q1 * q1) - 1.0
        r01 = 2.0 * (q1 * q2 - q0 * q3)
        r02 = 2.0 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2.0 * (q1 * q2 + q0 * q3)
        r11 = 2.0 * (q0 * q0 + q2 * q2) - 1.0
        r12 = 2.0 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2.0 * (q1 * q3 - q0 * q2)
        r21 = 2.0 * (q2 * q3 + q0 * q1)
        r22 = 2.0 * (q0 * q0 + q3 * q3) - 1.0

        return np.array(
            [
                [r00, r01, r02, px],
                [r10, r11, r12, py],
                [r20, r21, r22, pz],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )


def get_apriltags_entry(args=None):
    rclpy.init(args=args)
    node = GetAprilTags()
    rclpy.spin(node)
    rclpy.shutdown()
