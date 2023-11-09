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
    load_path: Load the path for the robot to follow
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from motion_plan_pkg.move_robot import MoveRobot

from geometry_msgs.msg import Pose, Point
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Point, Quaternion
from shape_msgs.msg import SolidPrimitive

from writer_interfaces.srv import Path

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
    GRIPPER = (auto(),)  # Move the gripper to a new configuration
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

        self.move_group_name = (
            self.get_parameter("move_group_name").get_parameter_value().string_value
        )

        self.ee_frame_id = (
            self.get_parameter("ee_frame_id").get_parameter_value().string_value
        )

        # Fake or Real Mode
        self.declare_parameter(
            "fake_mode",
            True,
            ParameterDescriptor(
                description="Axis that the end effector will rotate around"
            ),
        )
        self.fake_mode = (
            self.get_parameter("fake_mode").get_parameter_value().bool_value
        )

        # Position parameters
        self.declare_parameter(
            "x", 0.5, ParameterDescriptor(description="X coordinate goal")
        )
        self.declare_parameter(
            "y", 0.5, ParameterDescriptor(description="Y coordinate goal")
        )

        self.goal_x = self.get_parameter("x").get_parameter_value().double_value
        self.goal_y = self.get_parameter("y").get_parameter_value().double_value
        self.goal_z = 0.05

        # Orientation
        self.declare_parameter(
            "theta", math.pi / 2, ParameterDescriptor(description="Angle of rotation")
        )
        self.declare_parameter(
            "rotation_axis",
            [0.0, 1.0, 0.0],
            ParameterDescriptor(
                description="Axis that the end effector will rotate around"
            ),
        )
        self.theta = self.get_parameter("theta").get_parameter_value().double_value
        self.rotation_axis = (
            self.get_parameter("rotation_axis").get_parameter_value().double_array_value
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
        )

        self.comm_count = 0
        self.pos_list = [
            Point(x=self.goal_x, y=self.goal_y, z=self.goal_z),
            Point(x=self.goal_x, y=self.goal_y + 0.05, z=self.goal_z + 0.05),
            Point(x=0.3, y=0.0, z=0.5),
        ]
        self.ori_list = [
            self.robot.angle_axis_to_quaternion(self.theta, self.rotation_axis),
            self.robot.angle_axis_to_quaternion(self.theta, self.rotation_axis),
            self.robot.angle_axis_to_quaternion(math.pi, [1.0, 0.0, 0.0]),
        ]

        self.state = State.ADDBOX
        self.grasp_called = False
        self.points: list[Point] = None

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
        response.success = True
        return response

    def timer_callback(self):
        """Timer callback function of the picker node."""
        self.get_logger().info("Timer callback", once=True)

        # counter allows for only sending 1 goal position
        if self.state == State.MOVEARM:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("In executable code")
                self.get_logger().info("Publishing command no.%s" % self.comm_count)
                self.robot.find_and_execute(
                    point=self.pos_list[self.comm_count],
                    quat=self.ori_list[self.comm_count],
                )

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.get_logger().info("Done moving arm")
                self.comm_count += 1
                self.get_logger().info("comm_count:%s" % self.comm_count)
                if self.comm_count == 1 and not self.fake_mode:
                    self.get_logger().info("Executing close gripper", once=True)
                    self.state = State.GRIPPER
                    self.robot.grasp()
                elif self.comm_count == 2 and not self.fake_mode:
                    self.get_logger().info("Executing open gripper", once=True)
                    self.state = State.GRIPPER
                    self.robot.grasp()
                elif self.comm_count <= 2:
                    self.get_logger().info("Executing next command", once=True)
                    self.state = State.MOVEARM
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.get_logger().info(f"{self.robot.state}")
                else:
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.state = State.DONE

        elif self.state == State.GRIPPER:
            self.get_logger().info("Executing gripper command", once=True)
            if self.robot.state == MOVEROBOT_STATE.DONE:
                self.state = State.MOVEARM
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.get_logger().info(f"{self.robot.state}")

        elif self.state == State.ADDBOX:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("add box", once=True)

                # Add a box to environment
                name = "box_0"
                pose = Pose()
                pose.position.x = 0.1
                pose.position.y = 0.0
                pose.position.z = -0.15
                size = [1.0, 0.6, 0.1]
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                self.robot.add_box(name=name, pose=pose, shape=shape)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.state = State.MOVEARM

        elif self.state == State.REMOVEBOX:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("add box", once=True)
                name = "box_0"
                self.robot.remove_box(name=name)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.state = State.DONE

            elif self.state == State.GRIPPER:
                if self.robot.state == MOVEROBOT_STATE.WAITING:
                    if not self.grasp_called:
                        self.robot.grasp()
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
    node_picker = Picker()
    rclpy.spin(node=node_picker)

    node_picker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
