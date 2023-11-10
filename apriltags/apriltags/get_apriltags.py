import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped


class GetAprilTags(Node):
    """Gets April Tag information"""
    def __init__(self):
        super().__init__("get_apriltags")

        self.timer = self.create_timer(1.0/10.0, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)






    def timer_callback(self):

        try:
            t = self.tf_buffer.lookup_transform(
                "tag36h11:9",
                "camera_link",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                        f'Could not transform {"tag36h11:9"} to {"camera_link"}: {ex}') # /tf publishes camera_color_optical_frame, not camera link. But camera_link is root. Unsure which to use.
            return
        
        self.get_logger().info(f"{t.transform.translation.z}")

        # camera_apriltag1 = TransformStamped()

        


        



def get_apriltags_entry(args=None):
    rclpy.init(args=args)
    node = GetAprilTags()
    rclpy.spin(node)
    rclpy.shutdown()