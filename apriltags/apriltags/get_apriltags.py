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
        
        self.state = State.WAITING

        self.timer = self.create_timer(1.0/100.0, self.timer_callback)
        
        # Initialize transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.broadcast_static_transform()
        
        # Initialize robot information
        self.robot = MoveRobot(self, "panda_manipulator", True, "panda_hand_tcp")
        self.orientation: Quaternion = None

        self.whiteboard_distance = None

        self.top_left = None
        self.bottom_left = None
        self.top_right = None

#########################################################################################################################
    def timer_callback(self):

        if self.state == State.LOOK_UP_TRANSFORM:

            ########## TOP LEFT
            try:
                t = self.tf_buffer.lookup_transform(
                    "tag36h11:1",
                    "camera_color_optical_frame", # /tf publishes camera_color_optical_frame, not camera link. But camera_link is root. Unsure which to use.
                    rclpy.time.Time())
                self.orientation = Quaternion(x=t.transform.rotation.x, y=t.transform.rotation.y, z=t.transform.rotation.z, w=t.transform.rotation.w)
                self.state = State.ADD_BOX

            except TransformException as ex:
                self.get_logger().info(
                            f'Could not transform {"tag36h11:1"} to {"camera_color_optical_frame"}: {ex}', once=True)
                return
                    
            pose_info = f"Position: {t.transform.translation.x, t.transform.translation.y, 0.1*(t.transform.translation.z)}\n"\
                        f"Orientation: {t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w}"

            # self.get_logger().info("\n" + pose_info + "\n")

            self.top_left = -0.1 * t.transform.translation.z

            ########## BOTTOM LEFT
            try:
                t = self.tf_buffer.lookup_transform(
                    "tag36h11:3",
                    "camera_color_optical_frame", # /tf publishes camera_color_optical_frame, not camera link. But camera_link is root. Unsure which to use.
                    rclpy.time.Time())
                self.orientation = Quaternion(x=t.transform.rotation.x, y=t.transform.rotation.y, z=t.transform.rotation.z, w=t.transform.rotation.w)
                self.state = State.ADD_BOX

            except TransformException as ex:
                self.get_logger().info(
                            f'Could not transform {"tag36h11:3"} to {"camera_color_optical_frame"}: {ex}', once=True)
                return
                    
            pose_info = f"Position: {t.transform.translation.x, t.transform.translation.y, 0.1*(t.transform.translation.z)}\n"\
                        f"Orientation: {t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w}"

            # self.get_logger().info("\n" + pose_info + "\n")

            self.bottom_left = -0.1 * t.transform.translation.z

            ########## TOP RIGHT
            try:
                t = self.tf_buffer.lookup_transform(
                    "tag36h11:2",
                    "camera_color_optical_frame", # /tf publishes camera_color_optical_frame, not camera link. But camera_link is root. Unsure which to use.
                    rclpy.time.Time())
                self.orientation = Quaternion(x=t.transform.rotation.x, y=t.transform.rotation.y, z=t.transform.rotation.z, w=t.transform.rotation.w)
                self.state = State.ADD_BOX

            except TransformException as ex:
                self.get_logger().info(
                            f'Could not transform {"tag36h11:2"} to {"camera_color_optical_frame"}: {ex}', once=True)
                return
                    
            pose_info = f"Position: {t.transform.translation.x, t.transform.translation.y, 0.1*(t.transform.translation.z)}\n"\
                        f"Orientation: {t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w}"

            # self.get_logger().info("\n" + pose_info + "\n")

            self.top_right = -0.1 * t.transform.translation.z

        elif self.state == State.ADD_BOX:
            if self.top_left and self.bottom_left and self.top_right:
                name = "whiteboard"
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = (self.top_left + self.bottom_left + self.top_right)/3
                pose.position.z = 0.5
                pose.orientation = self.orientation
                size = [0.5, 1.0, 0.05]
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                self.robot.add_box(name=name, pose=pose, shape=shape)

            self.state = State.LOOK_UP_TRANSFORM
#########################################################################################################################


    def broadcast_static_transform(self):
        # Treating panda_link0 as the world/base_link frame
        panda_camera_tf = TransformStamped()
        panda_camera_tf.header.stamp = self.get_clock().now().to_msg()
        panda_camera_tf.header.frame_id = "panda_link0"
        panda_camera_tf.child_frame_id = "camera_color_optical_frame"

        panda_camera_tf.transform.translation.x = 0.0
        panda_camera_tf.transform.translation.y = 0.0
        panda_camera_tf.transform.translation.z = 0.0

        panda_camera_tf.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.tf_static_broadcaster.sendTransform(panda_camera_tf)
        self.state = State.LOOK_UP_TRANSFORM

        
def get_apriltags_entry(args=None):
    rclpy.init(args=args)
    node = GetAprilTags()
    rclpy.spin(node)
    rclpy.shutdown()
