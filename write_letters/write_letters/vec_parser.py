"""
Parser node that parses the all waypoints of a set of letters to a path.

Services
--------
    write [Write]: Write the characters on the board.

Clients
-------
    load_path [Path]: Load the path on robot.
    
Subscriptions
-------------
    april_tag_coords [AprilCoords]: The coordinate of all april tags.

Returns
-------

"""
from random import random
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup

# messages
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point

# services
from polyglotbot_interfaces.srv import Path, Write
from polyglotbot_interfaces.msg import AprilCoords


class VecParser(Node):
    def __init__(self):
        super().__init__("vec_parser")
        self.cb_group = ReentrantCallbackGroup()
        self.april_1: Point = None
        self.april_2: Point = None
        self.april_3: Point = None

        self.client_points = self.create_client(
            Path, "load_path", callback_group=self.cb_group
        )

        self.declare_parameter(
            "offset_letter",
            0.03,
            ParameterDescriptor(description="The gap between the letters"),
        )
        self.declare_parameter(
            "offset_standoff",
            0.1,
            ParameterDescriptor(
                description="The distance between the pen and board when lifting the pen."
            ),
        )
        self.declare_parameter(
            "offset_penup",
            0.05,
            ParameterDescriptor(
                description="Distance between the pen and board when lifting the pen within a letter"
            ),
        )
        self.declare_parameter(
            "scaling",
            12.0,
            ParameterDescriptor(description="Scaling factor for the letter size"),
        )

        self.offset_letter = (
            self.get_parameter("offset_letter").get_parameter_value().double_value
        )
        self.offset_standoff = (
            self.get_parameter("offset_standoff").get_parameter_value().double_value
        )
        self.offset_penup = (
            self.get_parameter("offset_penup").get_parameter_value().double_value
        )
        self.scaling = self.get_parameter("scaling").get_parameter_value().double_value

        self.offset = 0.4
        self.offset_x = 0.1
        self.offset_z = 0.3
        # self.offset_letter = 0.03
        # self.offset_standoff = 0.1
        # self.offset_penup = 0.05
        self.gap_letter = 0.05
        # self.scaling = 12.0
        self.x_max = 0.7
        self.x_min = -0.4
        self.z_max = 0.8

        self.srv_write = self.create_service(
            Write,
            "write",
            callback=self.srv_write_callback,
            callback_group=self.cb_group,
        )

        if not self.client_points.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Service 'load_path' not available")

        while not self.client_points.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                "'load_path' service not available, waiting again ..."
            )

        # while self.count_publishers("april_tag_coords") < 1:
        #     self.get_logger().info("Not receiving tag coords waiting again ...")

        # self.__send_points()

    def sub_april_coords_callback(self, msg: AprilCoords):
        self.april_1 = msg.p1
        self.april_2 = msg.p2
        self.april_3 = msg.p3

        # self.get_logger().info(f"{self.april_1}, {self.april_2}, {self.april_3}")

    def srv_write_callback(self, request, response):
        """
        Callback function for the write service.

        Args:
        ----
            request (_type_): _description_
            response (_type_): _description_

        Returns:
        -------
            _type_: _description_
        """
        self.get_logger().info("Writing ...")

        if not self.april_1 or not self.april_2 or not self.april_3:
            response.result = "No april tag received yet"
            return response

        curr_x = -0.55
        curr_z = 0.45
        self.points = []

        is_pen_up = False

        for character in request.characters:
            # self.get_logger().info("START")

            max_x = 0.0
            max_y = 0.0
            has_stand_down = False

            p1 = np.array([self.april_1.x, self.april_1.y, self.april_1.z])
            p2 = np.array([self.april_2.x, self.april_2.y, self.april_2.z])
            p3 = np.array([self.april_3.x, self.april_3.y, self.april_3.z])

            v1 = p3 - p1
            v2 = p2 - p1

            cp = np.cross(v1, v2)
            a, b, c = cp

            d = np.dot(cp, p3)

            for point in character.points:
                # self.get_logger().info(f"{point}")

                px = point.x / self.scaling
                py = point.y / self.scaling

                max_x = max(max_x, px)
                max_y = max(max_y, py)

                x_pos = -(curr_x + px)
                # y_pos = -self.offset
                z_pos = py + curr_z

                y_val = (d - a * x_pos - c * z_pos) / b
                self.get_logger().info(f"Y offset: {y_val}")
                y_pos = y_val + 0.062

                if point.z == 1:
                    is_pen_up = True
                elif point.z == -1:
                    is_pen_up = False

                if not has_stand_down:
                    self.points.append(
                        Point(x=x_pos, y=y_pos + self.offset_standoff, z=z_pos)
                    )
                    has_stand_down = True

                if is_pen_up:
                    self.points.append(
                        Point(x=x_pos, y=y_pos + self.offset_penup, z=z_pos)
                    )
                else:
                    self.points.append(Point(x=x_pos, y=y_pos, z=z_pos))

            self.points.append(Point(x=x_pos, y=y_pos + self.offset_standoff, z=z_pos))
            curr_x += max_x + self.offset_letter

            if curr_x > 0.25:
                curr_z -= max_y + self.offset_letter
                curr_x = -0.55

                self.get_logger().info("Changing line ...")

            # self.get_logger().info("END")

        self.get_logger().info(f"max x: {curr_x}")
        self.points.append(Point(x=0.3, y=0.0, z=0.5))
        future = self.client_points.call_async(Path.Request(points=self.points))
        # self.get_logger().info(f"{self.points}")
        # rclpy.spin_until_future_complete(self, future)
        future.add_done_callback(self.path_future_callback)

        response.result = "Finished"
        # self.get_logger().info(f"{future.result()}")
        return response

    def path_future_callback(self, future_path: Future):
        self.get_logger().info(f"{future_path.result()}")


def main(args=None):
    rclpy.init(args=args)
    node_parser = VecParser()
    rclpy.spin(node_parser)

    node_parser.destroy_node()
    rclpy.shutdown()
