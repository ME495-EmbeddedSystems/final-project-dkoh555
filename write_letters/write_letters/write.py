"""
Controls the motion of the franka robot to grab a paper and drop it.

Parameters
----------
    move_group_name: The name of the move group.
    ee_frame_id: The name of the end-effector frame.
    fake_mode: If using the fake robot or not.
    x: The x coordinate of the paper location.
    y: The y coordinate of the paper location.
    theta: The angle of rotation about axis in radians.
    rotation_axis: The axis rotating about.
Services
--------
    load_path: Load the path for the robot to follow.
    calibrate: Make the robot arm to go to the calibration pose
"""
import math
import waiting

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from motion_plan_pkg.move_robot import MoveRobot

from geometry_msgs.msg import Pose, Point
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point, Quaternion
from shape_msgs.msg import SolidPrimitive

from polyglotbot_interfaces.srv import Path
from std_srvs.srv import Empty

from enum import Enum, auto

from motion_plan_pkg.move_robot import State as MOVEROBOT_STATE


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    ADDBOX = (auto(),)  # Add a box to the environment
    REMOVEBOX = (auto(),)  # Remove a box from the environment
    MOVEARM = (auto(),)  # Move the arm to a new position
    GRASP = (auto(),)  # Move the gripper to a new configuration
    HOMING = (auto(),)
    DONE = auto()  # Do nothing


class Picker(Node):
    """Controlling the robot to move and pickup a paper and drop it."""

    def __init__(self):
        super().__init__("picker")

        self.declare_parameter(
            "move_group_name",
            "panda_manipulator",
            ParameterDescriptor(description="Name of the move group"),
        )
        self.declare_parameter(
            "ee_frame_id",
            "panda_hand_tcp",
            ParameterDescriptor(description="Name of the e-e frame"),
        )
        self.declare_parameter(
            "fake_mode",
            True,
            ParameterDescriptor(
                description="Axis that the end effector will rotate around"
            ),
        )

        self.move_group_name = (
            self.get_parameter("move_group_name").get_parameter_value().string_value
        )
        self.ee_frame_id = (
            self.get_parameter("ee_frame_id").get_parameter_value().string_value
        )

        self.fake_mode = (
            self.get_parameter("fake_mode").get_parameter_value().bool_value
        )

        self.robot = MoveRobot(
            self, self.move_group_name, self.fake_mode, self.ee_frame_id
        )

        self.posittion: Point = None
        self.orientation: Quaternion = None

        self.cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            1 / 100, self.timer_callback, callback_group=self.cb_group
        )

        self.srv_path = self.create_service(
            Path,
            "load_path",
            self.srv_path_callback,
            callback_group=self.cb_group,
        )
        self.srv_calibrate = self.create_service(
            Empty,
            "calibrate",
            callback=self.srv_calibrate_callback,
            callback_group=self.cb_group,
        )
        self.srv_homing = self.create_service(
            Empty,
            "homing",
            callback=self.srv_homing_callback,
            callback_group=self.cb_group,
        )
        self.srv_grab = self.create_service(
            Point,
            "grab_pen",
            callback=self.srv_grab_callback,
            callback_group=self.cb_group,
        )

        self.comm_count = 0

        self.state = State.ADDBOX
        self.grasp_called = False
        self.points: list[Point] = None
        self.quats: list[Quaternion] = None
        self.poses: list[Pose] = None
        self.calibrate = False
        self.do_homing = False

    def srv_grab_callback(self, request, response):
        pass

    def srv_homing_callback(self, request, response):
        self.poses = []

        self.poses.append(
            Pose(
                position=Point(x=0.3, y=0.0, z=0.5),
                orientation=self.robot.angle_axis_to_quaternion(
                    theta=math.pi, axis=[1.0, 0.0, 0.0]
                ),
            )
        )

        self.get_logger().info("homing ...")

        if self.state == State.DONE:
            self.comm_count = 0
            self.state = State.MOVEARM
            self.robot.state = MOVEROBOT_STATE.WAITING
            self.do_homing = True
        else:
            return response

        # waiting.wait(lambda: self.state == State.DONE, timeout_seconds=100.0)
        return response

    def srv_calibrate_callback(self, request, response):
        self.poses = []

        self.poses.append(
            Pose(
                position=Point(x=0.132874, y=0.326641, z=0.644133),
                orientation=Quaternion(x=0.617469, y=-0.57324, z=0.385767, w=0.375913),
            )
        )

        # self.get_logger().info(f"pose to go: {self.poses}")
        self.get_logger().info("calibrating ...")

        if self.state == State.DONE:
            self.calibrate = True
            self.comm_count = 0
            self.state = State.MOVEARM
            self.robot.state = MOVEROBOT_STATE.WAITING
        else:
            return response

        # waiting.wait(lambda: self.state == State.DONE, timeout_seconds=100.0)
        return response

    def srv_path_callback(self, request, response):
        """
        Stores the path from load_path service.

        Args:
            request (Path_Request): Request object from the load_path service.
            response (Path_Response): Response object of the load_path service.

        Returns:
            Path_Response: Response to the load_path service.
        """
        self.points = request.points
        self.quats = []
        self.poses = []

        for i in range(len(self.points)):
            # if i == 0:
            #     quat = self.robot.angle_axis_to_quaternion(math.pi, [1.0, 0.0, 0.0])
            if i < len(self.points) - 1:
                q = self.robot.angle_axis_to_quaternion(math.pi, [1.0, 0.0, 0.0])
                quat = self.robot.quaternion_mult(
                    q0=q,
                    q1=self.robot.angle_axis_to_quaternion(
                        math.pi / 2, [0.0, 0.0, 1.0]
                    ),
                )
            else:
                quat = self.robot.angle_axis_to_quaternion(math.pi, [1.0, 0.0, 0.0])

            self.quats.append(quat)

            pose = Pose()
            pose.position = self.points[i]
            pose.orientation = quat

            self.poses.append(pose)

        self.pos_list = self.points
        self.ori_list = self.quats

        if self.state == State.DONE:
            response.success = True
            self.comm_count = 0
            self.state = State.MOVEARM
            self.robot.state = MOVEROBOT_STATE.WAITING
        else:
            response.success = False

        return response

    def timer_callback(self):
        """Timer callback function of the picker node."""
        self.get_logger().info("Timer callback", once=True)

        # counter allows for only sending 1 goal position

        self.get_logger().info(f"State: {self.state}, Robot State: {self.robot.state}")

        if self.state == State.MOVEARM:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("In executable code")
                self.get_logger().info("Publishing command no.%s" % self.comm_count)
                # self.robot.find_and_execute(
                #     point=self.pos_list[self.comm_count],
                #     quat=self.ori_list[self.comm_count],
                # )
                # if self.calibrate:
                #     self.robot.find_and_execute(
                #         point=self.poses[0].position, quat=self.poses[0].orientation
                #     )
                # else:
                self.robot.find_and_execute_cartesian(self.poses)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                # self.get_logger().info("Done moving arm")
                # self.comm_count += 1
                # self.get_logger().info("comm_count:%s" % self.comm_count)
                # if self.comm_count == 1 and not self.fake_mode:
                #     self.get_logger().info("Executing close gripper", once=True)
                #     self.state = State.GRIPPER
                #     self.robot.grasp()
                # elif self.comm_count == (len(self.pos_list) - 1) and not self.fake_mode:
                #     self.get_logger().info("Executing open gripper", once=True)
                #     self.state = State.GRIPPER
                #     self.robot.grasp()
                # elif self.comm_count < len(self.pos_list):
                #     self.get_logger().info("Executing next command", once=True)
                #     self.state = State.MOVEARM
                #     self.robot.state = MOVEROBOT_STATE.WAITING
                #     self.get_logger().info(f"{self.robot.state}")
                # else:
                #     self.comm_count = 0
                #     self.robot.state = MOVEROBOT_STATE.WAITING
                #     self.state = State.DONE
                self.calibrate = False
                self.robot.state = MOVEROBOT_STATE.WAITING
                if self.do_homing:
                    self.state = State.HOMING
                else:
                    self.state = State.DONE

        elif self.state == State.GRASP:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("Executing gripper command", once=True)
                self.robot.grasp(0.05)

            if self.robot.state == MOVEROBOT_STATE.DONE:
                self.state = State.MOVEARM
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.get_logger().info(f"{self.robot.state}")

        elif self.state == State.HOMING:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.do_homing = False
                self.robot.homing()

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.state = State.DONE
                self.robot.state = MOVEROBOT_STATE.WAITING

        elif self.state == State.ADDBOX:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("add box", once=True)

                # Add a box to environment
                name = "box_0"
                pose = Pose()
                pose.position.x = 0.1
                pose.position.y = 0.0
                pose.position.z = -0.15
                size = [0.9, 0.6, 0.1]
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                self.robot.add_box(name=name, pose=pose, shape=shape)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.state = State.DONE

        elif self.state == State.REMOVEBOX:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("add box", once=True)
                name = "box_0"
                self.robot.remove_box(name=name)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.state = State.DONE

            elif self.state == State.GRASP:
                if self.robot.state == MOVEROBOT_STATE.WAITING:
                    if not self.grasp_called:
                        self.robot.grasp(0.08)
                        self.grasp_called = True
                elif self.robot.state == MOVEROBOT_STATE.DONE:
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.state = State.DONE
                    self.grasp_called = False


def main(args=None):
    """
    Start of the node.

    Args:
    ----
        args (list[str], optional): Arguments passed to the ros. Defaults to None.

    """
    rclpy.init(args=args)
    node_writer = Picker()
    rclpy.spin(node=node_writer)

    node_writer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
