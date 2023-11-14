"""
Parser node that parses the all waypoints of a set of letters to a path.

Raises
------
    RuntimeError: _description_

Returns
-------
    _type_: _description_
"""
from random import random
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup

# messages
from geometry_msgs.msg import Point

# services
from polyglotbot_interfaces.srv import Path, Write


class VecParser(Node):
    def __init__(self):
        super().__init__("vec_parser")
        self.cb_group = ReentrantCallbackGroup()

        self.client_points = self.create_client(
            Path, "load_path", callback_group=self.cb_group
        )
        self.srv_write = self.create_service(
            Write,
            "write",
            callback=self.srv_write_callback,
            callback_group=self.cb_group,
        )

        if not self.client_points.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("Service 'load_path' not available")

        self.offset = 0.6
        self.offset_x = 0.1
        self.offset_z = 0.3
        self.offset_letter = 0.05
        self.offset_standup = 0.1
        self.gap_letter = 0.05
        self.scaling = 8.0

        ################# Points for Testing #################
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

        self.O = [
            [4.60109375, 0.3021875],
            [4.60109375, 0.21],
            [4.568125, 0.13890625],
            [4.53515625, 0.0678125],
            [4.4740625, 0.02890625],
            [4.413125, -0.00984375],
            [4.3334375, -0.00984375],
            [4.2134375, -0.00984375],
            [4.13921875, 0.075625],
            [4.065, 0.16109375],
            [4.065, 0.30609375],
            [4.065, 0.40875],
            [4.065, 0.50046875],
            [4.09796875, 0.57171875],
            [4.1309375, 0.643125],
            [4.1921875, 0.681875],
            [4.2534375, 0.72078125],
            [4.3325, 0.72078125],
            [4.4121875, 0.72078125],
            [4.4734375, 0.681875],
            [4.5346875, 0.643125],
            [4.5678125, 0.5715625],
            [4.60109375, 0.5],
            [4.60109375, 0.40875],
            [4.60109375, 0.3021875],
            [4.5746875, 0.4096875],
            [4.5746875, 0.53859375],
            [4.50921875, 0.61640625],
            [4.44390625, 0.694375],
            [4.3325, 0.694375],
            [4.22265625, 0.694375],
            [4.1571875, 0.616875],
            [4.091875, 0.53953125],
            [4.091875, 0.40625],
            [4.091875, 0.3021875],
            [4.091875, 0.21734375],
            [4.12140625, 0.15203125],
            [4.1509375, 0.086875],
            [4.205625, 0.05171875],
            [4.2603125, 0.0165625],
            [4.3334375, 0.0165625],
            [4.44484375, 0.0165625],
            [4.5096875, 0.09421875],
            [4.5746875, 0.171875],
            [4.5746875, 0.30609375],
            [4.5746875, 0.4096875],
        ]

        self.HELLO = [
            ##### H #####
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
            ##### E #####
            [1.48046875, 0.3525],
            [1.1240625, 0.3525],
            [1.1240625, 0.02640625],
            [1.5303125, 0.02640625],
            [1.5303125, 0.0],
            [1.09765625, 0.0],
            [1.09765625, 0.7109375],
            [1.5303125, 0.7109375],
            [1.5303125, 0.68453125],
            [1.1240625, 0.68453125],
            [1.1240625, 0.37890625],
            [1.48046875, 0.37890625],
            [1.48046875, 0.3525],
            ##### L #####
            [2.12453125, 0.02640625],
            [2.48921875, 0.02640625],
            [2.48921875, 0.0],
            [2.09765625, 0.0],
            [2.09765625, 0.7109375],
            [2.12453125, 0.7109375],
            [2.12453125, 0.02640625],
            ##### L #####
            [3.12453125, 0.02640625],
            [3.48921875, 0.02640625],
            [3.48921875, 0.0],
            [3.09765625, 0.0],
            [3.09765625, 0.7109375],
            [3.12453125, 0.7109375],
            [3.12453125, 0.02640625],
            ##### O #####
            [4.60109375, 0.3021875],
            [4.60109375, 0.21],
            [4.568125, 0.13890625],
            [4.53515625, 0.0678125],
            [4.4740625, 0.02890625],
            [4.413125, -0.00984375],
            [4.3334375, -0.00984375],
            [4.2134375, -0.00984375],
            [4.13921875, 0.075625],
            [4.065, 0.16109375],
            [4.065, 0.30609375],
            [4.065, 0.40875],
            [4.065, 0.50046875],
            [4.09796875, 0.57171875],
            [4.1309375, 0.643125],
            [4.1921875, 0.681875],
            [4.2534375, 0.72078125],
            [4.3325, 0.72078125],
            [4.4121875, 0.72078125],
            [4.4734375, 0.681875],
            [4.5346875, 0.643125],
            [4.5678125, 0.5715625],
            [4.60109375, 0.5],
            [4.60109375, 0.40875],
            [4.60109375, 0.3021875],
            [4.5746875, 0.4096875],
            [4.5746875, 0.53859375],
            [4.50921875, 0.61640625],
            [4.44390625, 0.694375],
            [4.3325, 0.694375],
            [4.22265625, 0.694375],
            [4.1571875, 0.616875],
            [4.091875, 0.53953125],
            [4.091875, 0.40625],
            [4.091875, 0.3021875],
            [4.091875, 0.21734375],
            [4.12140625, 0.15203125],
            [4.1509375, 0.086875],
            [4.205625, 0.05171875],
            [4.2603125, 0.0165625],
            [4.3334375, 0.0165625],
            [4.44484375, 0.0165625],
            [4.5096875, 0.09421875],
            [4.5746875, 0.171875],
            [4.5746875, 0.30609375],
            [4.5746875, 0.4096875],
        ]

        # self.__send_points()

    def __send_points(self):
        """Send points to draw on the board."""
        points = []

        # offset = 0.11
        # offset = 0.6
        # scaling = 8.0

        ###################### Stand-up Position ######################

        ###################################
        ############  ON TABLE  ###########
        # x_pos = self.H[0][0] / 6.0 + 0.25
        # y_pos = self.H[0][1] / 6.0 - 0.05
        # z_pos = offset + 0.1

        ###################################
        ############  ON BOARD  ###########
        x_pos = (self.HELLO[0][0] - 2.0) / self.scaling + 0.1
        y_pos = -self.offset + 0.1
        z_pos = self.HELLO[0][1] / self.scaling + 0.3

        points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        ###################### Letter Path ######################

        for i in range(len(self.HELLO)):
            ###################################
            ############  ON TABLE  ###########
            # x_pos = self.H[i][0] / 6.0 + 0.25
            # y_pos = self.H[i][1] / 6.0 - 0.05
            # z_pos = offset

            ###################################
            ############  ON BOARD  ###########
            x_pos = (self.HELLO[i][0] - 2.0) / self.scaling + 0.1
            y_pos = -self.offset
            z_pos = self.HELLO[i][1] / self.scaling + 0.3

            points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        ###################### Stand-up Position ######################

        ###################################
        ############  ON TABLE  ###########
        # x_pos = self.H[-1][0] / 6.0 + 0.25
        # y_pos = self.H[-1][1] / 6.0 - 0.05
        # z_pos = offset + 0.1

        ###################################
        ############  ON BOARD  ###########
        x_pos = (self.HELLO[-1][0] - 2.0) / self.scaling + 0.1
        y_pos = -self.offset + 0.1
        z_pos = self.HELLO[-1][1] / self.scaling + 0.3

        points.append(Point(x=x_pos, y=y_pos, z=z_pos))

        # points.append(Point(x=y, y=-0.6, z=z))
        points.append(Point(x=0.3, y=0.0, z=0.5))

        self.get_logger().info(f"Sent points: {points}")

        future: Future = self.client_points.call_async(Path.Request(points=points))
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(f"{future.result()}")

    def srv_write_callback(self, request, response):
        self.get_logger().info("Writing ...")

        curr_x = -0.4
        points = []

        for character in request.characters:
            # self.get_logger().info("START")

            max_x = 0.0
            has_stand_down = False

            for point in character.points:
                # self.get_logger().info(f"{point}")

                px = point.x / self.scaling
                py = point.y / self.scaling

                max_x = max(max_x, px)

                x_pos = -(curr_x + px + self.offset_x)
                y_pos = -self.offset
                z_pos = py + self.offset_z

                if not has_stand_down:
                    points.append(
                        Point(x=x_pos, y=y_pos + self.offset_standup, z=z_pos)
                    )
                    has_stand_down = True

                points.append(Point(x=x_pos, y=y_pos, z=z_pos))

            points.append(Point(x=x_pos, y=y_pos + self.offset_standup, z=z_pos))
            curr_x += max_x + self.offset_letter

            # self.get_logger().info("END")

        self.get_logger().info(f"curr x: {curr_x}")
        points.append(Point(x=0.3, y=0.0, z=0.5))
        future = self.client_points.call_async(Path.Request(points=points))
        rclpy.spin_until_future_complete(self, future)

        response.success = future.result().success
        self.get_logger().info(f"{future.result()}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node_parser = VecParser()
    rclpy.spin(node=node_parser)

    node_parser.destroy_node()
    rclpy.shutdown()
