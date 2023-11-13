from random import random
import numpy as np

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

        self.H = [
            [0.60453125, 0.0],
            [0.578125, 0.0],
            [0.578125, 0.3525],
            [0.1240625, 0.3525],
            [0.1240625, 0.0],
            [0.09765625, 0.0],
            [0.09765625, 0.7109375],
            [0.1240625, 0.7109375],
            [0.1240625, 0.37890625],
            [0.578125, 0.37890625],
            [0.578125, 0.7109375],
            [0.60453125, 0.7109375],
            [0.60453125, 0.0],
        ]

        self.__send_points()

    def __send_points(self):
        points = []
        # x = 0.893
        # y = 0.0
        # z = 0.2

        offset = 0.11

        # theta = np.linspace(0, 2 * np.pi, 100)
        # x_pos = x - 0.1
        # y_pos = y + 0.05 * np.cos(theta[0])
        # z_pos = z + 0.05 * np.sin(theta[0])

        # # points.append(Point(x=y, y=-0.6, z=z))
        # points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        # for i in range(100):
        #     x_pos = x
        #     y_pos = y + 0.05 * np.cos(theta[i])
        #     z_pos = z + 0.05 * np.sin(theta[i])

        #     points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        # x_pos = x - 0.1
        # y_pos = y + 0.05 * np.cos(theta[-1])
        # z_pos = z + 0.05 * np.sin(theta[-1])

        # points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        # x_pos = x - 0.1
        # y_pos = self.H[0][0] / 6.0 - 0.05
        # z_pos = self.H[0][0] / 6.0 + 0.1
        x_pos = self.H[0][0] / 6.0 + 0.25
        y_pos = self.H[0][1] / 6.0 - 0.05
        z_pos = offset + 0.1

        points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        for i in range(len(self.H)):
            x_pos = self.H[i][0] / 6.0 + 0.25
            y_pos = self.H[i][1] / 6.0 - 0.05
            z_pos = offset

            points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        x_pos = self.H[-1][0] / 6.0 + 0.25
        y_pos = self.H[-1][1] / 6.0 - 0.05
        z_pos = offset + 0.1

        points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        # points.append(Point(x=y, y=-0.6, z=z))
        points.append(Point(x=0.3, y=0.0, z=0.5))

        self.get_logger().info(f"Sent points: {points}")

        # points.append(Point(x=0.306891, y=0.0, z=0.486882))
        # points.append(Point(x=0.2, y=0.4, z=0.3))

        future: Future = self.client_points.call_async(Path.Request(points=points))
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(f"{future.result()}")


def main(args=None):
    rclpy.init(args=args)
    node_parser = VecParser()

    node_parser.destroy_node()
    rclpy.shutdown()
