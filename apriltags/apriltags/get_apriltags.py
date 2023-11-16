import numpy as np
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


def matrix_from_rot_and_trans(rotation: Quaternion, translation: Point):
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
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    return np.array(
        [[r00, r01, r02, px], [r10, r11, r12, py], [r20, r21, r22, pz], [0, 0, 0, 1]]
    )


class State(Enum):
    WAITING = auto()
    LOOK_UP_TRANSFORM = auto()
    ADD_BOX = auto()


class GetAprilTags(Node):
    """Gets April Tag information"""

    def __init__(self):
        super().__init__("get_apriltags")

        self.state = State.LOOK_UP_TRANSFORM

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

        # hand_camera_tf.transform.rotation = Quaternion(
        #     x=0.8320387, y=-0.0052696, z=0.5546925, w=-0.0000832
        # )
        hand_camera_tf.transform.rotation = Quaternion(
            x=0.7071068, y=0.0, z=0.7071068, w=0.0
        )

        self.tf_static_broadcaster.sendTransform(hand_camera_tf)

    # 3 is top left, 4 is bottom left, 1 is bottom right
    #########################################################################################################################
    def timer_callback(self):
        if self.state == State.LOOK_UP_TRANSFORM:
            try:
                t = self.tf_buffer.lookup_transform(
                    "tag36h11:3",
                    "camera_color_optical_frame",  # /tf publishes camera_color_optical_frame, not camera link. But camera_link is root. Unsure which to use.
                    rclpy.time.Time(),
                )

                position_3 = (
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                )
                orientation_3 = Quaternion(
                    x=t.transform.rotation.x,
                    y=t.transform.rotation.y,
                    z=t.transform.rotation.z,
                    w=t.transform.rotation.w,
                )

                # posi_info = f"Position: {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z}"

                self.get_logger().info("\n" + f"{position_3[2]}" + "\n")

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {"tag36h11:3"} to {"camera_color_optical_frame"}: {ex}',
                    once=True,
                )
                return


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


def get_apriltags_entry(args=None):
    rclpy.init(args=args)
    node = GetAprilTags()
    rclpy.spin(node)
    rclpy.shutdown()
