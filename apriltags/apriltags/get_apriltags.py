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

        self.timer = self.create_timer(1.0/100.0, self.timer_callback)
        
        # Initialize transform information
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)



#########################################################################################################################
    def timer_callback(self):

        if self.state == State.LOOK_UP_TRANSFORM:

            try:
                t = self.tf_buffer.lookup_transform(
                    "tag36h11:3",
                    "camera_color_optical_frame", # /tf publishes camera_color_optical_frame, not camera link. But camera_link is root. Unsure which to use.
                    rclpy.time.Time())
                


                pose_info = f"Position: {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z}"



                self.get_logger().info("\n" + pose_info + "\n")


            except TransformException as ex:
                self.get_logger().info(
                            f'Could not transform {"tag36h11:3"} to {"camera_color_optical_frame"}: {ex}', once=True)
                return
                    
            
            



#########################################################################################################################


        
def get_apriltags_entry(args=None):
    rclpy.init(args=args)
    node = GetAprilTags()
    rclpy.spin(node)
    rclpy.shutdown()
