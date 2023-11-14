from random import random

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from polyglotbot_interfaces.srv import Path
from geometry_msgs.msg import Point


class VecParser(Node):
    def __init__(self):
        super().__init__("vec_parser")

        self.client_points = self.create_client(Path, "load_path")

        if not self.client_points.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Service 'load_path' not available")

        self.__send_points()

    def __send_points(self):
        points = []
        x = 0.8
        y = 0.0
        z = 0.05

        for i in range(10):
            x_pos = x
            y_pos = y + random() * 0.4 - 0.2
            z_pos = z + random() * 0.5

            points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        points.append(Point(x=0.3, y=0.0, z=0.5))

        future: Future = self.client_points.call_async(Path.Request(points=points))
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info(f'{future.result()}')


def main(args=None):
    rclpy.init(args=args)
    node_parser = VecParser()

    node_parser.destroy_node()
    rclpy.shutdown()
