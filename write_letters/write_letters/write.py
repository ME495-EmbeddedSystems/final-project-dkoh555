"""
Controls the motion of the franka robot to grab a paper and drop it.

Parameters
----------
    move_group_name: The name of the move group.
    ee_frame_id: The name of the end-effector frame.
    fake_mode: If using the fake robot or not.

Services
--------
    load_path: Load the path for the robot to follow.
    calibrate: Make the robot arm to go to the calibration pose.
    homing: Make the robot and gripper to home pose.
    grab_pen: Let robot move to the desired pose and grab the pen there.
    change_writer_state: Change the state of the writer node to DONE.

Publishers
----------
    writer_state [String]: The state of the writer node.

Returns
-------
    None

"""
import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from motion_plan_pkg.move_robot import MoveRobot, State as MoveRobot_State

# Messages
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

# Services
from polyglotbot_interfaces.srv import Path, GrabPen
from std_srvs.srv import Empty, Trigger


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    ADDBOX_TABLE = (auto(),)  # Add a table to the environment
    ADDBOX_ROD = (auto(),)  # Add a rod to the evironment
    ADDBOX_BOARD = (auto(),)  # Add a board to the environment
    REMOVEBOX = (auto(),)  # Remove a box from the environment
    MOVEARM = (auto(),)  # Move the arm to a new position
    GRASP = (auto(),)  # Move the gripper to a new configuration
    HOMING = (auto(),)  # Move the gripper to home
    REACHING = (auto(),)  # Reaching the pen
    GRABING = (auto(),)  # Grabbing the pen from pen case
    DONEWRITING = (auto(),)  # Done writing
    PUTTINGBACK = (auto(),)  # Putting pen back to the pen case
    GOINGBACK = (auto(),)  # Going back to home after putting back
    DONE = auto()  # Do nothing


class Writer(Node):
    """
    Controlling the robot to move and write the letter requested and drop it.

    Args:
    ----
        Node (rclpy.node.Node): Super class of the writer node.

    Attributes
    ----------
        comm_count (int): Counter for how many commands sent to the robot.
        state (writer.State): The state of the writer Node.
        grasp_called (bool): Whether the grasp action has been called.
        points (list[Point]): The list of points to go to.
        quats (list[Quaternion]): The list of quaternions to go to.
        poses (list[Pose]): The list of points to go to.
        poses_reaching (list[Pose]): The list of poses to go during reaching.
        poses_grabing (list[Pose]): The list of poses to go during grabing.
        poses_putback (list[Pose]): The list of poses to go during the putting back.
        poses_goback (list[Pose]): The list of poses to go during the putting back.
        calibrate (bool): Whether do the calibration.
        do_homing (bool): Whether do the homing process.
        do_grabing (bool): Whether grab the pen.
        do_reaching (bool): Whether reach the pen.
        do_goback (bool): Whether go back after reaching.
        home_pose (Pose): The home pose.

    """

    def __init__(self):
        super().__init__("writer")

        # ----- Declare Parameters -----
        self.declare_parameter(
            "move_group_name",
            "panda_manipulator",
            ParameterDescriptor(description="Name of the move group"),
        )
        self.declare_parameter(
            "ee_frame_id",
            "panda_link8",
            ParameterDescriptor(description="Name of the e-e frame"),
        )
        self.declare_parameter(
            "fake_mode",
            True,
            ParameterDescriptor(
                description="Axis that the end effector will rotate around"
            ),
        )

        # ----- Get Parameters -----
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

        # ----- Services -----
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
            GrabPen,
            "grab_pen",
            callback=self.srv_grab_callback,
            callback_group=self.cb_group,
        )

        self.srv_state = self.create_service(
            Empty,
            "change_writer_state",
            callback=self.srv_state_callback,
            callback_group=self.cb_group,
        )

        self.srv_put_back = self.create_service(
            Trigger,
            "put_back",
            callback=self.srv_put_back_callback,
            callback_group=self.cb_group,
        )

        # ----- Publishers -----
        self.pub_state = self.create_publisher(String, "writer_state", 10)

        self.comm_count = 0

        self.state = State.ADDBOX_TABLE
        self.grasp_called = False
        self.points: list[Point] = None
        self.quats: list[Quaternion] = None
        self.poses: list[Pose] = None
        self.poses_reaching: list[Pose] = None
        self.poses_grabing: list[Pose] = None
        self.poses_putback: list[Pose] = None
        self.poses_goback: list[Pose] = None
        self.calibrate = False
        self.do_homing = False
        self.do_grabing = False
        self.do_reaching = False
        self.do_goback = False

        self.pose_home = Pose(
            position=Point(x=0.3, y=0.0, z=0.5),
            orientation=Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0),
        )

    # --------------- SERVICE CALLBACK ---------------
    def srv_put_back_callback(self, request, response):
        """
        Put the pen back to the pen case.

        Args:
        ----
            request (Trigger_Request): Resquest of the put_back service
            response (Trigger_Response): Response of the put_back service

        Returns
        -------
            Trigger_Response: Response of the put_back service.

        """
        position = Point(x=0.452, y=0.0, z=0.175)
        orientation = Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0)

        pos_standoff = Point(x=position.x - 0.09, y=position.y, z=position.z)

        pose_case = Pose()
        pose_case.position = position
        pose_case.orientation = orientation

        pose_standoff = Pose()
        pose_standoff.position = pos_standoff
        pose_standoff.orientation = orientation

        self.poses_putback = []
        self.poses_putback.append(pose_standoff)
        self.poses_putback.append(pose_case)

        self.poses_goback = []
        self.poses_goback.append(pose_standoff)
        self.poses_goback.append(self.pose_home)

        if self.state == State.DONE or self.state == State.DONEWRITING:
            self.robot.state = MoveRobot_State.WAITING
            self.state = State.PUTTINGBACK
            self.do_goback = True
            response.success = True
            response.message = "Puting pen back succeeded"
        else:
            response.success = False
            response.message = "Putting pen back failed"

        return response

    def srv_grab_callback(self, request, response):
        """
        Grab a pen from the pen case.

        Args:
        ----
            request (GrabPen_Request): Request of the grab_pen service.
            response (GrabPen_Response): Response of the grab_pen service.

        Returns
        -------
            GrabPen_Response: Response of the grab_pen service.

        """
        position: Point = request.position
        orientation = Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0)

        pos_standoff = Point(x=position.x - 0.09, y=position.y, z=position.z)

        pose_grab = Pose()
        pose_grab.position = position
        pose_grab.orientation = orientation

        pose_standoff = Pose()
        pose_standoff.position = pos_standoff
        pose_standoff.orientation = orientation

        self.poses_reaching = []
        self.poses_reaching.append(pose_standoff)
        self.poses_reaching.append(pose_grab)

        self.poses_grabing = []
        self.poses_grabing.append(pose_standoff)
        self.poses_grabing.append(self.pose_home)

        if self.state == State.DONE or self.state == State.DONEWRITING:
            self.robot.state = MoveRobot_State.WAITING
            self.state = State.HOMING
            self.do_reaching = True
            response.success = True
        else:
            response.success = False

        return response

    def srv_homing_callback(self, request, response):
        """
        Send the command to robot back to home pose.

        Args:
        ----
            request (Empty_Request): Request of the homing service.
            response (Empty_Response): Response of the homing service.

        Returns
        -------
            Empty_Response: Response of the hominmg service.

        """
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

        if self.state == State.DONE or self.state == State.DONEWRITING:
            self.comm_count = 0
            self.state = State.MOVEARM
            self.robot.state = MoveRobot_State.WAITING
            self.do_homing = False
        else:
            return response

        return response

    def srv_calibrate_callback(self, request, response):
        """
        Calibrate the camera to get position of apriltags.

        Args:
        ----
            request (Empty_Request): Request object for calibrate service.
            response (Empty_Response): Response object for the calibrate service.

        Returns
        -------
            Empty_Response: Response object for the calibrate service.

        """
        self.poses = []

        self.poses.append(
            Pose(
                position=Point(x=0.278, y=0.0071, z=0.536),
                orientation=Quaternion(x=0.975, y=-0.197, z=0.0764, w=0.06),
            )
        )

        self.poses.append(
            Pose(
                position=Point(x=0.2398, y=0.0302, z=0.582),
                orientation=Quaternion(x=0.891, y=-0.375, z=0.213, w=0.141),
            )
        )

        self.get_logger().info("calibrating ...")

        if self.state == State.DONE or self.state == State.DONEWRITING:
            self.calibrate = True
            self.comm_count = 0
            self.state = State.MOVEARM
            self.robot.state = MoveRobot_State.WAITING
        else:
            return response

        return response

    def srv_path_callback(self, request, response):
        """
        Store the path from load_path service.

        Args:
        ----
            request (Path_Request): Request object from the load_path service.
            response (Path_Response): Response object of the load_path service.

        Returns
        -------
            Path_Response: Response to the load_path service.

        """
        self.points = request.points
        self.quats = []
        self.poses = []

        for i in range(len(self.points)):
            if i < len(self.points) - 1:
                quat = Quaternion(x=-0.3826834, y=0.9238795, z=0.0, w=0.0)
            else:
                quat = Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0)

            self.quats.append(quat)

            pose = Pose()
            pose.position = self.points[i]
            pose.orientation = quat

            self.poses.append(pose)

        pose = Pose()
        pose.position = Point(x=0.32, y=-0.2, z=0.6)
        pose.orientation = Quaternion(x=0.9238795, y=-0.3826834, z=0.0, w=0.0)
        self.poses.insert(0, pose)

        pose.position = Point(x=0.32, y=-0.2, z=0.6)
        pose.orientation = Quaternion(x=-0.3826834, y=0.9238795, z=0.0, w=0.0)

        self.poses.insert(1, pose)

        self.pos_list = self.points
        self.ori_list = self.quats

        if self.state == State.DONE or self.state == State.DONEWRITING:
            response.success = True
            self.comm_count = 0
            self.state = State.MOVEARM
            self.robot.state = MoveRobot_State.WAITING
        else:
            response.success = False

        return response

    def srv_state_callback(self, request, response):
        """
        Change the state of the write node to be done.

        Args:
        ----
            request (Empty_Request): Request of the change_writer_state service.
            response (Empty_response): Response of the change_writer_state service.

        Returns
        -------
            Empty_Response: Response of the service callMain function of the parser node.

        """
        self.state = State.DONE

        response = Empty.Response()

        return response

    # ---------- TIMER CALLBACK ----------
    def timer_callback(self):
        """Timer callback function of the writer node."""
        self.get_logger().info("Timer callback", once=True)

        # counter allows for only sending 1 goal position

        self.get_logger().debug(f"State: {self.state}, Robot State: {self.robot.state}")
        self.pub_state.publish(String(data=f"{self.state}"))

        if self.state == State.MOVEARM:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("In executable code")
                self.get_logger().info("Publishing command no.%s" % self.comm_count)
                self.robot.find_and_execute_cartesian(self.poses)

            elif self.robot.state == MoveRobot_State.DONE:
                # self.calibrate = False
                # self.state = State.DONEWRITING
                self.robot.state = MoveRobot_State.WAITING
                if self.do_homing:
                    self.state = State.HOMING
                    self.do_homing = False
                else:
                    self.state = State.DONEWRITING

        elif self.state == State.REACHING:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("reaching the pen ...")
                self.robot.find_and_execute_cartesian(self.poses_reaching)

            elif self.robot.state == MoveRobot_State.DONE:
                self.do_grabing = True
                self.state = State.GRASP
                self.robot.state = MoveRobot_State.WAITING

        elif self.state == State.GRABING:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("grabing the pen ...")
                self.robot.find_and_execute_cartesian(self.poses_grabing)

            elif self.robot.state == MoveRobot_State.DONE:
                self.state = State.DONEWRITING
                self.robot.state = MoveRobot_State.WAITING

        elif self.state == State.PUTTINGBACK:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("putting back the pen ...")
                self.robot.find_and_execute_cartesian(self.poses_putback)

            elif self.robot.state == MoveRobot_State.DONE:
                self.state = State.HOMING
                self.robot.state = MoveRobot_State.WAITING

        elif self.state == State.GOINGBACK:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("putting back the pen ...")
                self.robot.find_and_execute_cartesian(self.poses_goback)

            elif self.robot.state == MoveRobot_State.DONE:
                self.state = State.DONEWRITING
                self.robot.state = MoveRobot_State.WAITING

        elif self.state == State.GRASP:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("Executing gripper command", once=True)
                self.robot.grasp(width=0.00, speed=0.05, force=50.0)

            if self.robot.state == MoveRobot_State.DONE:
                self.robot.state = MoveRobot_State.WAITING
                self.get_logger().info(f"{self.robot.state}")
                if self.do_grabing:
                    self.state = State.GRABING
                    self.do_grabing = False
                else:
                    self.state = State.MOVEARM

        elif self.state == State.HOMING:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("homing the gripper  ...")
                self.robot.homing()

            elif self.robot.state == MoveRobot_State.DONE:
                if self.do_reaching:
                    self.state = State.REACHING
                    self.do_reaching = False
                elif self.do_goback:
                    self.state = State.GOINGBACK
                    self.do_goback = False
                else:
                    self.state = State.DONE
                self.robot.state = MoveRobot_State.WAITING

        elif self.state == State.ADDBOX_TABLE:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("add table box", once=True)

                # Add a box to environment
                name = "table"
                pose = Pose()
                pose.position.x = 0.1
                pose.position.y = 0.0
                pose.position.z = -0.15
                size = [0.9, 0.6, 0.1]
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                self.robot.add_box(name=name, pose=pose, shape=shape)

            elif self.robot.state == MoveRobot_State.DONE:
                self.robot.state = MoveRobot_State.WAITING
                self.state = State.ADDBOX_BOARD

        elif self.state == State.ADDBOX_ROD:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("add rod box", once=True)

                name = "rod"
                pose = Pose()
                pose.position.x = 0.45
                pose.position.y = 0.225
                pose.position.z = 0.25
                size = [0.2, 0.15, 0.7]
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                self.robot.add_box(name=name, pose=pose, shape=shape)

            elif self.robot.state == MoveRobot_State.DONE:
                self.robot.state = MoveRobot_State.WAITING
                self.state = State.ADDBOX_BOARD

        elif self.state == State.ADDBOX_BOARD:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("add board box", once=True)

                name = "board"
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = -0.6
                pose.position.z = 0.2
                size = [2.0, 0.05, 2.0]
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                self.robot.add_box(name=name, pose=pose, shape=shape)

            elif self.robot.state == MoveRobot_State.DONE:
                self.robot.state = MoveRobot_State.WAITING
                self.state = State.DONE

        elif self.state == State.REMOVEBOX:
            if self.robot.state == MoveRobot_State.WAITING:
                self.get_logger().info("remove box", once=True)
                name = "box_0"
                self.robot.remove_box(name=name)

            elif self.robot.state == MoveRobot_State.DONE:
                self.robot.state = MoveRobot_State.WAITING
                self.state = State.DONE


def main(args=None):
    """
    Start of the node.

    Args:
    ----
        args (list[str], optional): Arguments passed to the ros. Defaults to None.

    """
    rclpy.init(args=args)
    node_writer = Writer()
    rclpy.spin(node=node_writer)

    node_writer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
