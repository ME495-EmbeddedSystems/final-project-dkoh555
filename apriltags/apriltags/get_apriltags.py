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

        # Needs to be able to get data from multiple AprilTags, not just this one
        try:
            t = self.tf_buffer.lookup_transform(
                "tag36h11:9",
                "camera_link", # /tf publishes camera_color_optical_frame, not camera link. But camera_link is root. Unsure which to use.
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                        f'Could not transform {"tag36h11:9"} to {"camera_link"}: {ex}')
            return
        
        pose_info = f"Position: {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z}\n"\
                    f"Orientation: {t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w}"

        self.get_logger().info("\n" + pose_info + "\n")

        
def get_apriltags_entry(args=None):
    rclpy.init(args=args)
    node = GetAprilTags()
    rclpy.spin(node)
    rclpy.shutdown()
