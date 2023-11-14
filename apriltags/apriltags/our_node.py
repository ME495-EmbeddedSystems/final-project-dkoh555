import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Header
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose  # TransformStamped, PoseStamped, Twist
# from moveit_msgs.srv import GetPositionIK_Response
from rcl_interfaces.msg import ParameterDescriptor
from .move_robot import MoveRobot  # This should be "from move_robot import MoveRobot"
from geometry_msgs.msg import Point, Quaternion  # TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup  # MutuallyExclusiveCallbackGroup,
from shape_msgs.msg import SolidPrimitive
from enum import Enum, auto
from .move_robot import State as MOVEROBOT_STATE


class State(Enum):
    """
    Enum representing the current state of the system.

    This enum is used to determine the main timer function's behavior on each iteration.

    Attributes
    ----------
        ADDBOX: Add a box to the environment
        REMOVEBOX: Remove a box from the environment
        MOVEARM: Move the arm to a new position
        GRIPPER: Move the gripper to a new configuration
        DONE: Do nothing

    """

    ADDBOX = auto(),
    REMOVEBOX = auto(),
    MOVEARM = auto(),
    GRIPPER = auto(),
    DONE = auto(),


class OurNode(Node):
    """
    Custom ROS2 Node for controlling a robotic arm and performing various tasks.

    This node initializes parameters and controls the robotic arm's movements,
    gripper actions, and the addition/removal of collision objects.

    Parameters
    ----------
        move_group_name (str): Name of the move group for the robot arm.
        ee_frame_id (str): Name of the end-effector frame.
        fake_mode (bool): Flag to enable or disable fake mode for testing.
        goal_x (float): X coordinate goal for arm movement.
        goal_y (float): Y coordinate goal for arm movement.
        goal_z (float): Z coordinate goal for arm movement.
        theta (float): Angle of rotation for the end-effector.
        rotation_axis (list of float): Axis around which the end-effector rotates.
        keep_pos (bool): Flag to keep the current position.
        keep_ori (bool): Flag to keep the current orientation.

    Attributes
    ----------
        move_group_name (str): Name of the move group.
        ee_frame_id (str): End-effector frame ID.
        fake_mode (bool): Flag indicating fake mode.
        goal_x (float): X coordinate for arm goal.
        goal_y (float): Y coordinate for arm goal.
        goal_z (float): Z coordinate for arm goal.
        theta (float): Angle of rotation for the end-effector.
        rotation_axis (list of float): Axis of rotation for the end-effector.
        robot (MoveRobot): Object for controlling the robot's movements and actions.
        position (Point): Current position.
        orientation (Quaternion): Current orientation.
        cb_group (ReentrantCallbackGroup): Callback group for timer.
        timer (Timer): Timer for the main control loop.
        comm_count (int): Counter for sequential position commands.
        pos_list (list of Point): List of goal positions for arm movement.
        ori_list (list of Quaternion): List of goal orientations for arm movement.
        state (State): Current state of the robot control.
        grasp_called (bool): Flag to track if the grasp action has been called.

    Methods
    -------
        timer_callback(): Callback function for the timer to control robot actions.

    """

    def __init__(self):
        super().__init__("our_node")

        self.declare_parameter(
            "move_group_name",
            "panda_manipulator",
            ParameterDescriptor(description="Name of the move group"),
        )
        self.declare_parameter(
            "ee_frame_id",
            "panda_hand_tcp",
            ParameterDescriptor(description="Name of the e-e frame")
        )

        self.move_group_name = (
            self.get_parameter("move_group_name").get_parameter_value().string_value
        )

        self.ee_frame_id = (
            self.get_parameter('ee_frame_id').get_parameter_value().string_value
        )

        self.ee_frame_id = "panda_hand_tcp"

        # Fake or Real Mode
        self.declare_parameter(
            "fake_mode",
            True,
            ParameterDescriptor(
                description="Axis that the end effector will rotate around"
            ),
        )
        self.fake_mode = self.get_parameter("fake_mode").get_parameter_value().bool_value

        # Position
        self.declare_parameter(
            "goal_x", 0.5, ParameterDescriptor(description="X coordinate goal")
        )
        self.declare_parameter(
            "goal_y", 0.5, ParameterDescriptor(description="Y coordinate goal")
        )
        self.declare_parameter(
            "goal_z", 0.5, ParameterDescriptor(description="Z coordinate goal")
        )
        self.goal_x = self.get_parameter("goal_x").get_parameter_value().double_value
        self.goal_y = self.get_parameter("goal_y").get_parameter_value().double_value
        self.goal_z = self.get_parameter("goal_z").get_parameter_value().double_value

        # Orientation
        self.declare_parameter(
            "theta", 3.141529, ParameterDescriptor(description="Angle of rotation")
        )
        self.declare_parameter(
            "rotation_axis",
            [1.0, 0.0, 0.0],
            ParameterDescriptor(description="Axis that the end effector will rotate around"),
        )
        self.theta = self.get_parameter("theta").get_parameter_value().double_value
        self.rotation_axis = (
            self.get_parameter("rotation_axis").get_parameter_value().double_array_value
        )

        self.declare_parameter('keep_pos', False, ParameterDescriptor(description='If \
                                                                      keepting the curremt pose'))
        self.declare_parameter('keep_ori', False, ParameterDescriptor(description='If keeping the \
                                                                      current orientation'))

        self.robot = MoveRobot(self, self.move_group_name, self.fake_mode, self.ee_frame_id)

        # self.keep_pos = self.get_parameter('keep_pos').get_parameter_value().bool_value
        # self.keep_ori = self.get_parameter('keep_ori').get_parameter_value().bool_value

        self.posittion: Point = None
        self.orientation: Quaternion = None

        self.cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(1/100, self.timer_callback, callback_group=self.cb_group)

        # For sequential Pos commands
        self.comm_count = 0
        self.pos_list = [Point(x=self.goal_x, y=self.goal_y, z=self.goal_z),
                         Point(x=0.2, y=0.4, z=0.2), Point(x=0.5, y=0.0, z=0.4)]
        self.ori_list = [self.robot.angle_axis_to_quaternion(self.theta, self.rotation_axis),
                         Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
                         Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)]

        self.state = State.ADDBOX
        self.grasp_called = False

    def timer_callback(self):
        """
        Give a function (callback) for the timer to control robot actions.

        This method handles different states and actions of the robot, including arm movements,
        gripper control, and collision object management.

        If the state is `MOVEARM`, it executes sequential arm movements specified by `pos_list`
        and `ori_list`. After each movement, it checks if the robot is in a fake mode and, if not,
        it triggers the gripper action by changing the state to `GRIPPER`.

        If the state is `GRIPPER`, it handles gripper actions. When the gripper action is complete,
        it changes the state back to `MOVEARM` to execute the next command.

        If the state is `ADDBOX`, it adds a box to the environment. Once the addition is complete,
        it changes the state to `MOVEARM`.

        If the state is `REMOVEBOX`, it removes a box from the environment. Once the removal is
        complete,it changes the state to `DONE`.

        If the state is `GRASP`, it executes the grasp action. Once the grasp action is complete,
        it changes the state to `DONE`.

        The `comm_count` variable keeps track of the sequential commands, and the `grasp_called`
        flag is used to track if the grasp action has been called.

        Returns
        -------
            None

        """
        self.get_logger().info("Timer callback", once=True)
        # counter allows for only sending 1 goal position
        if self.state == State.MOVEARM:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("In executable code")
                # Define goal point msg
                # point = Point(x=self.goal_x, y=self.goal_y, z=self.goal_z)
                # point.x = self.goal_x
                # point.y = self.goal_y
                # point.z = self.goal_z

                # find orientation in quaternion
                # quat = self.robot.angle_axis_to_quaternion(self.theta, self.rotation_axis)
                self.get_logger().info('Publishing command no.%s' % self.comm_count)
                self.robot.find_and_execute(point=self.pos_list[self.comm_count],
                                            quat=self.ori_list[self.comm_count])

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.get_logger().info("Done moving arm")
                self.comm_count += 1
                self.get_logger().info('comm_count:%s' % self.comm_count)
                if self.comm_count == 1 and not self.fake_mode:
                    self.get_logger().info("Executing close gripper", once=True)
                    self.state = State.GRIPPER
                    # self.comm_count += 1
                    self.robot.grasp()
                elif self.comm_count == 2 and not self.fake_mode:
                    self.get_logger().info("Executing open gripper", once=True)
                    self.state = State.GRIPPER
                    # self.comm_count += 1
                    self.robot.grasp()
                # if self.comm_count == 2:
                #     self.get_logger().info("Executing open gripper", once=True)
                #     pass
                elif self.comm_count <= 2:
                    self.get_logger().info("Executing next command", once=True)
                    self.state = State.MOVEARM
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.get_logger().info(f"{self.robot.state}")
                    # self.comm_count += 1
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
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = -0.2
                size = [2.5, 2.5, 0.2]
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                self.robot.add_box(name=name, pose=pose, shape=shape)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.state = State.MOVEARM

        elif self.state == State.REMOVEBOX:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("add box", once=True)

                # Remove a box to environment
                name = "box_0"
                self.robot.remove_box(name=name)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.state = State.DONE

            elif self.state == State.GRASP:
                if self.robot.state == MOVEROBOT_STATE.WAITING:
                    if not self.grasp_called:
                        self.robot.grasp()
                        self.grasp_called = True
                elif self.robot.state == MOVEROBOT_STATE.DONE:
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.state = State.DONE
                    self.grasp_called = False


def entry_point(args=None):
    """
    Entry point for the 'our_node' ROS2 node.

    Initializes ROS2, creates the 'our_node' node, enters the ROS2 spin loop,
    and shuts down ROS2 when the node execution is complete.
    """
    rclpy.init(args=args)
    node = OurNode()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()
