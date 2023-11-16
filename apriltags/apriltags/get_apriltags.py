import rclpy
from rclpy.node import Node

# from tf2_ros.transform_listener import TransformListener
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

        # hand_camera_tf.transform.translation.x = 50e-3
        # hand_camera_tf.transform.translation.y = 15e-3
        # hand_camera_tf.transform.translation.z = 65e-3

        hand_camera_tf.transform.translation.x = 50e-3
        hand_camera_tf.transform.translation.y = 15e-3
        hand_camera_tf.transform.translation.z = 65e-3

        
        hand_camera_tf.transform.rotation = Quaternion(
            x=0.7071068, y=0.0, z=0.7071068, w=0.0
        )

        self.tf_static_broadcaster.sendTransform(hand_camera_tf)

        # Call calibrate service immediately after lauching node
        self.client_calibrate = self.create_client(
            Empty, "calibrate", callback_group=self.cb_group
        )

        if not self.client_calibrate.wait_for_service(timeout_sec=6.0):
            raise RuntimeError("Service 'calibrate' not available")
        
        self.client_calibrate.call_async(Empty.Request())

        self.publish_april_coords = self.create_publisher(AprilCoords, 'april_tag_coords', 10)

        self.top_left_position = []
        self.bottom_left_position = []
        self.bottom_right_position = []

    # 3 is top left, 4 is bottom left, 1 is bottom right
    #########################################################################################################################
    def timer_callback(self):

        # Publish the x,y,z of each AprilTag
        if self.state == State.LOOK_UP_TRANSFORM:

            ### PANDA HAND
            try:
                t = self.tf_buffer.lookup_transform(
                    "panda_hand",
                    "panda_link0",
                    rclpy.time.Time(),
                )

                self.position_1 = t.transform.translation
                self.rotation_1 = t.transform.rotation

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_link0"} to {"panda_hand"}: {ex}',
                    once=True,
                )
                return
            
            ### TOP LEFT
            try:
                s = self.tf_buffer.lookup_transform(
                    "tag36h11:3",
                    "panda_hand",
                    rclpy.time.Time(),
                )

                self.position_2 = s.transform.translation
                self.rotation_2 = s.transform.rotation

                t_0_h = self.matrix_from_rot_and_trans(self.rotation_1, self.position_1)
                t_h_tag = self.matrix_from_rot_and_trans(self.rotation_2, self.position_2)

                t_0_top_left = np.matmul(t_0_h, t_h_tag)
                self.top_left_position = t_0_top_left[:3, 3]

                # self.get_logger().info(f"{t_0_top_left}")
                # self.get_logger().info(f"{self.top_left_position}")

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_hand"} to {"tag36h11:3"}: {ex}',
                    once=True,
                )
                return
            
            ### BOTTOM LEFT
            try:
                s = self.tf_buffer.lookup_transform(
                    "tag36h11:4",
                    "panda_hand",
                    rclpy.time.Time(),
                )

                self.position_2 = s.transform.translation
                self.rotation_2 = s.transform.rotation

                t_0_h = self.matrix_from_rot_and_trans(self.rotation_1, self.position_1)
                t_h_tag = self.matrix_from_rot_and_trans(self.rotation_2, self.position_2)

                t_0_bottom_left = np.matmul(t_0_h, t_h_tag)
                self.bottom_left_position = t_0_bottom_left[:3, 3]

                self.get_logger().info(f"{t_0_bottom_left}")
                self.get_logger().info(f"{self.bottom_left_position}")

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_hand"} to {"tag36h11:4"}: {ex}',
                    once=True,
                )
                return
            
            ### BOTTOM RIGHT
            try:
                s = self.tf_buffer.lookup_transform(
                    "tag36h11:1",
                    "panda_hand",
                    rclpy.time.Time(),
                )

                self.position_2 = s.transform.translation
                self.rotation_2 = s.transform.rotation

                t_0_h = self.matrix_from_rot_and_trans(self.rotation_1, self.position_1)
                t_h_tag = self.matrix_from_rot_and_trans(self.rotation_2, self.position_2)

                t_0_bottom_right = np.matmul(t_0_h, t_h_tag)
                self.bottom_right_position = t_0_top_left[:3, 3]

                # self.get_logger().info(f"{t_0_bottom_right}")
                # self.get_logger().info(f"{self.bottom_right_position}")

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"panda_hand"} to {"tag36h11:1"}: {ex}',
                    once=True,
                )
                return
            
            






            if any(self.top_left_position) and any(self.bottom_left_position) and any(self.bottom_right_position):
                msg = AprilCoords()
                msg.p1 = Point(x = self.top_left_position[0],y = self.top_left_position[1],z = self.top_left_position[2])
                msg.p2 = Point(x = self.bottom_left_position[0],y = self.bottom_left_position[1],z = self.bottom_left_position[2])
                msg.p3 = Point(x = self.bottom_right_position[0],y = self.bottom_right_position[1],z = self.bottom_right_position[2])

                self.publish_april_coords.publish(msg)



            

            


#########################################################################################################################

# def broadcast_static_transform(self):

#     # Camera in the frame of the hand
#     hand_camera_tf = TransformStamped()
#     hand_camera_tf.header.stamp = self.get_clock().now().to_msg()
#     hand_camera_tf.header.frame_id = "panda_hand"
#     hand_camera_tf.child_frame_id = "camera_color_optical_frame"

#     hand_camera_tf.transform.translation.x = 0.50
#     hand_camera_tf.transform.translation.y = 0.0
#     hand_camera_tf.transform.translation.z = 0.65

#     hand_camera_tf.transform.rotation.x = Quaternion(x=0.0, y=0.0, z=0.7071068, w=0.7071068)

#     self.tf_static_broadcaster.sendTransform(hand_camera_tf)

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
            [[r00, r01, r02, px], [r10, r11, r12, py], [r20, r21, r22, pz], [0.0, 0.0, 0.0, 1.0]]
        )


def get_apriltags_entry(args=None):
    rclpy.init(args=args)
    node = GetAprilTags()
    rclpy.spin(node)
    rclpy.shutdown()
