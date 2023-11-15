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

        hand_camera_tf.transform.translation.x = 50e-3
        hand_camera_tf.transform.translation.y = 15e-3
        hand_camera_tf.transform.translation.z = 65e-3

        hand_camera_tf.transform.rotation = Quaternion(
            x=0.8320387, y=-0.0052696, z=0.5546925, w=-0.0000832
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
