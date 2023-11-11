# Necessary Imports
import math
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage
from moveit_msgs.msg import (
    PositionIKRequest,
    MotionPlanRequest,
    WorkspaceParameters,
    RobotState,
    JointConstraint,
    OrientationConstraint,
    Constraints,
    CollisionObject
)
from franka_msgs.action import (
    Grasp,
    # Homing,
    # Move
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, ApplyPlanningScene
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Pose
from rcl_interfaces.srv import ListParameters, ListParameters_Response
from enum import Enum, auto


class State(Enum):
    """
    Enum representing the current state of the system.

    This enum is used to determine the main timer function's behavior on each iteration.

    Attributes
    ----------
        STARTUP: Startup state
        WAITING: Waiting for user input
        IK: Computing Inverse Kinematics
        FINDPATH: Finding a path
        EXECUTEPATH: Executing a path
        BOX: Adding or removing a box from the environment
        GETPLANNING: Getting the planning scene
        DONOTHING: Performing various tasks
        GRASP: Performing a grasping operation
        HOMING
        DONE: Task is complete

    """

    STARTUP = auto(),
    WAITING = auto(),
    IK = auto(),
    FINDPATH = auto(),
    EXECUTE = auto(),
    BOX = auto(),
    GETPLANNING = auto(),
    DOTHING = auto(),
    GRASP = auto(),
    HOMING = auto(),
    DONE = auto(),


class MoveRobot:
    """
    This class provides functionality for controlling a robot's motion using ROS2 and MoveIt.

    It initializes various ROS2 components, handles subscriptions, and provides methods to perform
    actions like grasping, finding paths, executing paths, adding or removing objects from the
    scene,
    and more.

    Attributes
    ----------
        node (Node): The ROS2 node used to communicate with the ROS2 ecosystem
        js_current (JointState): The current joint state of the robot
        future (Future): A future object for asynchronous operations
        param_result (ListParameters_Response): Result of parameter listing service
        tf_current (TFMessage): The current transformation (tf) of the robot
        tf_static (TFMessage): The static transformation (tf) of the robot
        cb_group (ReentrantCallbackGroup): A callbackgroup for concurrent callback functions
        frame_id (str): The frame ID for the robot's base
        ee_frame_id(str): The frame ID for the robot's end effector
        tolerance (float): Tolerance used in path planning
        tmr (Timer): A timer for scheduling callback functions
        tf_buffer (Buffer): A buffer for managing transformations
        tf_listener (TransformListener): A listener for transforming coordinate frames
        state (State): The current state of the robot controller
        keep_ori (bool): Whether to keep the current orientation in path planning
        plan_only(bool): Whether to plan only or plan and execute
        counter (int): A counter for internal use

    Methods
    -------
        USER METHODS:
        grasp(): Initiates a grasping action
        find_path(point=None, quat=None): Finds a motion path given a point and a quaternion
        read_path(): Reads the planned motion path
        execute_path(path): Executes a planned motion path
        find_and_execute(point=None, quat=None): Finds and executes a motion path
        add_box(name, pose, shape): Adds a box to the robot's planning scene
        remove_box(name): Removes a box from the robot's planning scene
        angle_axis_to_quaternion(theta, axis): converts angle-axis rotation to a quaternion
        CLASS METHODS
        joint_states_callback(msg: JointState): Callback function for joint state updates
        transform_callback(msg: TFMessage): Callback function for transformation updates
        tf_static_callback(msg: TFMessage): Callback function for static transform updates
        future_IK_callback(future_IK): Callback function for asynchronous move planning
        future_move_callback(future_move): Callback function for asynchronous move planning
        future_b_callback(future_b): Callback function for asynchronous get result of move
                                    planning
        future_execute_callback(future_execute): Callback function for asynchronous execution of
                                                planned paths
        future_c_callback(future_c): Callback function for asynchronous get result of execution
        future_scene_callback(future_scene): Callback function for
                                             asynchronous planning scene updates
        future_box_callback(future_box): Callback function for asynchronous box
                                            addition/removal updates
        future_grasp_callback(future_grasp): Callback function for asynchronous grasping operation
        updates. future_grasp_d_callback(future_e): Callback function for asynchronous
                                            grasping result
        updates get_root_frame_id(): Obtains the root frame ID of the robot's coordinate frames
        FormatPositionIKRequest(): Formats messages for compute IK requests

    """

    def __init__(self, node: Node, move_group_name, fake_mode, ee_frame_id):
        self.node = node
        self.js_current: JointState = None
        self.robot_state = RobotState()
        self.future: Future = None
        self.param_result: ListParameters_Response = None
        self.tf_current: TFMessage = None
        self.tf_static: TFMessage = None
        self.cb_group = ReentrantCallbackGroup()

        # User Inputs - edit later
        # self.group_name = "panda_manipulator"
        self.frame_id = "panda_link0"
        self.group_name = move_group_name
        # self.frame_id = "" #pandalink0
        self.ee_frame_id = "panda_hand_tcp"  # ee_frame_id # panda_hand_tcp
        self.tolerance = 0.001

        # QOS Profile
        qos_tf_static = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Subscriptions
        self.sub_joint_states = self.node.create_subscription(JointState, 'joint_states',
                                                              self.joint_states_callback, 10)
        while self.node.count_publishers('joint_states') < 1:
            self.node.get_logger().info("waiting for joint_states publisher", once=True)

        self.sub_transforms = self.node.create_subscription(TFMessage, "tf",
                                                            self.transform_callback, 10)
        while self.node.count_publishers('tf') < 1:
            self.node.get_logger().info("waiting for tf publisher", once=True)

        self.sub_tf_static = self.node.create_subscription(TFMessage, "tf_static",
                                                           self.tf_static_callback,
                                                           qos_profile=qos_tf_static)
        while self.node.count_publishers('tf_static') < 1:
            self.node.get_logger().info("waiting for tf_static publisher", once=True)

        # Clients
        self.compute_ik = self.node.create_client(GetPositionIK, "compute_ik",
                                                  callback_group=self.cb_group)
        self.node.get_logger().info("create compute ik client")
        while not self.compute_ik.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info("compute IK Service not available, waiting again...")

        self.client_mg_params = self.node.create_client(ListParameters,
                                                        'move_group/list_parameters')
        while not self.client_mg_params.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info("Service not available, waiting again...")

        self.move_action_client = ActionClient(self.node, MoveGroup,
                                               'move_action', callback_group=self.cb_group)
        while not self.move_action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().info("Move group server not available, \
                                        waiting again ...")

        self.execute_trajectory_client = ActionClient(self.node, ExecuteTrajectory,
                                                      'execute_trajectory',
                                                      callback_group=self.cb_group)
        while not self.execute_trajectory_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().info("Execute trajectory server not available, \
                                        waiting again ...")

        if not fake_mode:
            self.gripper_grasp_client = ActionClient(self.node, Grasp,
                                                     'panda_gripper/grasp',
                                                     callback_group=self.cb_group)
            while not self.gripper_grasp_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().info("Grasp server not available,\
                                            waiting again ...")

        # self.gripper_home_client = ActionClient(self.node, Homing, 'panda_gripper/homing', \
        # callback_group=self.cb_group)
        # while not self.gripper_grasp_client.wait_for_server(timeout_sec=2.0):
        #     self.node.get_logger().info("Homing server not available, waiting again ...")

        self.get_planning_scene = self.node.create_client(GetPlanningScene,
                                                          "get_planning_scene",
                                                          callback_group=self.cb_group)
        while not self.get_planning_scene.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('get planning scence service not available, \
                                        waiting again...')

        self.apply_planning_scene = self.node.create_client(ApplyPlanningScene,
                                                            "apply_planning_scene",
                                                            callback_group=self.cb_group)
        while not self.apply_planning_scene.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('apply planning scence service not available, \
                                        waiting again...')

        # Timer
        self.tmr = self.node.create_timer(1/100, self.timer_callback, callback_group=self.cb_group)

        # Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # self.get_root_frame_id()

        self.state = State.STARTUP
        self.keep_ori = False
        self.plan_only = True  # True for plan, False for plan and execute

        self.counter = 0

        self.node.get_logger().info("MoveRobot initialized")

    def timer_callback(self):
        """
        Start timer callback for controlling robot actions and state transitions.

        This callback function is executed periodically by a timer and controls
        the state of the robot. The possible states include STARTUP,
        IK(inverse kinematics computation), FINDPATH (motion planning),
        EXECUTE (trajectory execution), DOTHING(idle), GETPLANNING (planning scene retrieval),
        BOX (scene object addition/removal), and GRASP (grasping operation).

        The behavior of the robot is determined by the current state, and the
        corresponding actions are initiated asynchronously. Transition between states is
        based on the completion of these actions.

        The function logs relevant information for debugging and state tracking.

        Args
        ----
            None

        Returns
        -------
            None

        """
        self.node.get_logger().info("Tmr callback", once=True)
        if self.state == State.STARTUP:
            if self.js_current and self.tf_current and self.tf_static:
                self.get_root_frame_id()
                self.node.get_logger().info("Got root frame id", once=True)
                self.state = State.WAITING

        elif self.state == State.IK:
            self.node.get_logger().info("Call compute ik")
            future_IK = self.compute_ik.call_async(self.FormatPositionIKRequest())
            self.node.get_logger().info("during compute ik", once=True)
            future_IK.add_done_callback(self.future_IK_callback)

        elif self.state == State.FINDPATH:
            self.node.get_logger().info("sending goal ...")
            future_move = self.move_action_client.send_goal_async(self.goal_msg)
            future_move.add_done_callback(self.future_move_callback)
            self.state = State.DOTHING

        elif self.state == State.EXECUTE:
            self.node.get_logger().info("executing ...")
            future_execute = self.execute_trajectory_client.send_goal_async(self.path_msg)
            future_execute.add_done_callback(self.future_execute_callback)
            self.state = State.DOTHING

        elif self.state == State.DOTHING:
            # self.node.get_logger().info("doing nothing")
            pass

        elif self.state == State.GETPLANNING:
            future_scene = self.get_planning_scene.call_async(GetPlanningScene.Request())
            future_scene.add_done_callback(self.future_scene_callback)

        elif self.state == State.BOX:
            future_box = self.apply_planning_scene.call_async(ApplyPlanningScene.Request
                                                              (scene=self.planning_scene))
            future_box.add_done_callback(self.future_box_callback)

        elif self.state == State.GRASP:
            future_grasp = self.gripper_grasp_client.send_goal_async(self.grasp_msg)
            future_grasp.add_done_callback(self.future_grasp_callback)
            self.state = State.DOTHING

# ############# USER METHODS #########################
# These are methods the user can call to interact with moveit

    def grasp(self):
        """
        Initiate a grasping operation with the specified parameters.

        This method sets up a grasping goal and transitions the robot's state to GRASP.
        The parameters for the grasp operation, such as width, speed, and force, are pre-defined.

        Args
        ----
            None

        Returns
        -------
            None

        """
        self.node.get_logger().info("grasp called")
        self.grasp_msg = Grasp.Goal()
        self.grasp_msg.width = 0.08
        self.grasp_msg.speed = 0.05
        self.grasp_msg.force = 20.0

        self.state = State.GRASP

    def find_path(self, point=None, quat=None):
        """
        Initiate a path planning operation to reach a desired end-effector pose.

        This method takes in a position and an orientation (as a quaternion) and plans a path
        to reach the specified end-effector pose. If the position or orientation is not provided,
        it will use the current robot's state. The path planning operation is then triggered,
        and the state transitions to IK (Inverse Kinematics).

        Args:
            point (Optional[geometry_msgs.msg.Point]): The desired 3D position
            (x, y, z) of the end-effector.
            quat (Optional[geometry_msgs.msg.Quaternion]): The desired orientation as a
            quaternion (x, y, z, w).

        Returns
        -------
            None

        """
        self.counter = self.counter + 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

        self.node.get_logger().info("in find_path")

        # find current position/quat if not privided
        self.starting_state = RobotState()
        self.starting_state.joint_state = self.js_current

        self.keep_ori = False
        if not point or not quat:
            target = self.frame_id
            source = self.ee_frame_id
            temp_tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())

            if not point:
                temp = temp_tf.transform.translation
                point = Point()
                point.x = temp.x
                point.y = temp.y
                point.z = temp.z

            if not quat:
                quat = temp_tf.transform.rotation
                self.quat = quat
                self.keep_ori = True

        # define a Pose() from point and quat
        pose = Pose()
        pose.position = point
        pose.orientation = quat

        # call compute_angles
        self.desired_pose = pose

        self.state = State.IK

        self.counter = self.counter - 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

    def read_path(self):
        """
        Read and retrieves the planned path.

        This method is used to retrieve the planned path generated by a previous
        path planning operation. After calling this method, the state transitions back to WAITING.

        Returns
        -------
            trajectory_msgs.msg.JointTrajectory: The planned path to the desired end-effector pose.

        """
        self.node.get_logger().info("in read path", once=True)
        self.state = State.WAITING
        return self.path

    def execute_path(self, path):
        """
        Execute the planned path obtained from the `find_path` method.

        This method is used to execute a previously planned path. It sets the
        execution state and starts the execution of the specified trajectory.

        Args:
        ----
            path (trajectory_msgs.msg.JointTrajectory): The planned path to be executed.

        """
        self.node.get_logger().info("in execute path")
        self.counter = self.counter + 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

        self.path = path

        self.path_msg = ExecuteTrajectory.Goal()
        self.path_msg.trajectory = self.path

        self.state = State.EXECUTE

        self.counter = self.counter - 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

    def find_and_execute(self, point=None, quat=None):
        """
        Find and executes a path to a specified end-effector pose.

        If provided, the position and orientation are used to plan a path to the desired pose.
        The path is both planned and executed. If no position and orientation are provided,
        the method will use the current position and orientation.

        Args:
        ----
            point (geometry_msgs.msg.Point, optional): The desired position (x, y, z).
            quat (geometry_msgs.msg.Quaternion, optional): The desired orientation (x, y, z, w).

        """
        self.plan_only = False  # True for plan, False for plan and execute

        # Feed in the current position/orientation if no input
        self.find_path(point, quat)

    def add_box(self, name, pose, shape):
        """
        Add a box to the planning scene.

        This method adds a box to the planning scene with the specified name, pose, and shape.
        The box is defined by its name, a geometry_msgs/Pose representing its position
        and orientation, and the shape of the box.

        Args:
        ----
            name (str): The name of the box.
            pose (geometry_msgs.msg.Pose): The pose of the box in the scene.
            shape: The shape of the box.

        """
        self.node.get_logger().info("add_box")
        self.collision_object = CollisionObject()
        self.collision_object.header.stamp = self.node.get_clock().now().to_msg()
        self.collision_object.header.frame_id = "panda_link0"
        self.collision_object.id = name
        self.collision_object.pose = pose
        self.collision_object.primitives = [shape]

        self.collision_object.operation = CollisionObject.ADD

        self.state = State.GETPLANNING

    def remove_box(self, name):
        """
        Remove a box from the planning scene.

        This method removes a previously added box from the planning scene
        by setting the operation of the collision object to REMOVE.

        Args:
        ----
            name (str): The name of the box to be removed.

        """
        self.collision_object = CollisionObject()
        self.collision_object.header.stamp = self.node.get_clock().now().to_msg()
        self.collision_object.header.frame_id = "panda_link0"
        self.collision_object.id = name

        self.collision_object.operation = CollisionObject.REMOVE

        self.state = State.GETPLANNING

    def angle_axis_to_quaternion(self, theta, axis):
        # Code comes from Matt
        """
        Convert from angle-axis of rotation to a quaternion.

        Args
        ----
        theta:  rotation angle, in radians
        axis: the rotational axis. This will be normalized

        Returns
        -------
        A Quaternion corresponding to the rotation

        """
        magnitude = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
        normalized = [v/magnitude for v in axis]
        sinTheta2 = math.sin(theta/2.0)
        return Quaternion(x=normalized[0]*sinTheta2,
                          y=normalized[1]*sinTheta2,
                          z=normalized[2]*sinTheta2,
                          w=math.cos(theta/2.0))

# ######################################################

# ################# CLASS METHODS #######################
# These are for internal use by the class only

    def joint_states_callback(self, msg: JointState):
        """
        Call back function to receive and process joint states of the robot.

        Args:
        ----
            msg (sensor_msgs.msg.JointState): The JointState
            message containing joint state information.

        """
        self.js_current = msg

    def transform_callback(self, msg: TFMessage):
        """
        Call back function to receive and process transformations (TF) of the robot.

        Args:
        ----
            msg (geometry_msgs.msg.TFMessage): The TFMessage containing transformation information.

        """
        self.tf_current = msg

    def tf_static_callback(self, msg: TFMessage):
        """
        Call back function to receive and process static transformations (TF) of the robot (base).

        Args:
        ----
            msg (geometry_msgs.msg.TFMessage): The TFMessage
            containing static transformation information.

        """
        self.tf_static = msg

    def future_IK_callback(self, future_IK):
        """
        Call back for when the compute IK future is done.

        Processes the IK result and initiates motion planning.

        Args:
        ----
            future_IK (rclpy.client.Future): The future representing
            the result of the compute IK request.

        """
        self.node.get_logger().info("end compute ik")

        self.counter = self.counter + 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

        thetalist = future_IK.result().solution
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()

        # Robot space bounds
        ws_param = WorkspaceParameters()
        ws_param.header.stamp = self.node.get_clock().now().to_msg()
        ws_param.header.frame_id = self.frame_id

        ws_param.min_corner.x = -1.0
        ws_param.min_corner.y = -1.0
        ws_param.min_corner.z = -1.0

        ws_param.max_corner.x = 1.0
        ws_param.max_corner.y = 1.0
        ws_param.max_corner.z = 1.0

        request.workspace_parameters = ws_param

        # if theta_start:
        #     joint_state = JointState()
        #     joint_state.name = self.js_current.name.copy()
        #     joint_state.position = theta_start.copy()

        #     start_state.joint_state = joint_state
        # else:
        #     start_state.joint_state = self.js_current

        self.starting_state.joint_state = self.js_current

        request.start_state = self.starting_state

        joint_constrains = []

        for i in range(len(thetalist.joint_state.position)):
            constraint = JointConstraint()
            constraint.joint_name = thetalist.joint_state.name[i]
            constraint.position = thetalist.joint_state.position[i]
            constraint.tolerance_above = self.tolerance
            constraint.tolerance_below = self.tolerance
            constraint.weight = 1.0

            joint_constrains.append(constraint)

        goal_constraint = Constraints()
        goal_constraint.name = ""
        goal_constraint.joint_constraints = joint_constrains
        request.goal_constraints = [goal_constraint]

        path_constraint = Constraints()
        path_constraint.name = ""

        if self.keep_ori:
            ee_ori_cons = OrientationConstraint()
            ee_ori_cons.header.stamp = self.node.get_clock().now().to_msg()
            ee_ori_cons.header.frame_id = self.frame_id
            ee_ori_cons.orientation = self.quat
            ee_ori_cons.link_name = self.ee_frame_id
            ee_ori_cons.absolute_x_axis_tolerance = 0.5
            ee_ori_cons.absolute_y_axis_tolerance = 0.5
            ee_ori_cons.absolute_z_axis_tolerance = 0.5
            ee_ori_cons.weight = 1.0

            path_constraint.orientation_constraints = [ee_ori_cons]

        request.path_constraints = path_constraint

        request.pipeline_id = "move_group"
        request.group_name = self.group_name
        request.num_planning_attempts = 40
        request.allowed_planning_time = 20.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        plan_decision = True  # True for plan, False for execute
        goal_msg.request = request
        goal_msg.planning_options.plan_only = plan_decision

        self.goal_msg = goal_msg

        self.state = State.FINDPATH

        self.counter = self.counter - 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

    def future_move_callback(self, future_move):
        """
        Call back for when the move action future is done.

        Initiates the process of getting the result of motion planning.

        Args:
        ----
            future_move (rclpy.action.ClientGoalHandle): The future
            representing the move action request.

        """
        self.node.get_logger().info("end plan")

        self.counter = self.counter + 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

        a = future_move.result()
        future_b = a.get_result_async()
        future_b.add_done_callback(self.future_b_callback)

        future_move = None

        self.counter = self.counter - 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

    def future_b_callback(self, future_b):
        """
        Call back when get result future of motion planning is done.

        Retrieves the planned trajectory.

        Args:
        ----
            future_b (rclpy.action.ClientGoalHandle): The future representing
            the result of motion planning.

        """
        self.path = future_b.result().result.planned_trajectory
        self.node.get_logger().info(f"{self.path}", once=True)
        self.node.get_logger().info("path")

        future_b = None

        self.counter = self.counter + 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

        if self.plan_only:
            self.state = State.DONE
        else:
            self.execute_path(self.path)

        self.counter = self.counter - 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

    def future_execute_callback(self, future_execute):
        """
        Call back when execute action future is done. Initiate process of getting execution result.

        Args:
        ----
            future_execute (rclpy.action.ClientGoalHandle): The future representing
            the execute action request.

        """
        self.node.get_logger().info("end execute")

        self.counter = self.counter + 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

        d = future_execute.result()
        future_c = d.get_result_async()
        future_c.add_done_callback(self.future_c_callback)

        self.counter = self.counter - 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

        # self.plan_only = True # True for plan, False for plan and execute
        # self.state = State.DONE

    def future_c_callback(self, future_c):
        """
        Call back for when get result future of execution is done. Completes execution process.

        Args:
        ----
            future_c (rclpy.action.ClientGoalHandle): The future representing the
            result of execution.

        """
        self.node.get_logger().info("end result")

        self.counter = self.counter + 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

        self.plan_only = True  # True for plan, False for plan and execute
        self.state = State.DONE

        self.counter = self.counter - 1
        self.node.get_logger().info(f"self.counter: {self.counter}")

    def future_scene_callback(self, future_scene):
        """
        Call back for when the get planning scene future is done.

        Processes the planning scene information and adds collision objects.

        Args:
        ----
            future_scene (rclpy.client.Future): Future representing result of
            get planning scene request.

        """
        self.node.get_logger().info("end get planning scene", once=True)
        self.planning_scene = future_scene.result().scene
        self.planning_scene.world.collision_objects.append(self.collision_object)
        self.state = State.BOX

    def future_box_callback(self, future_box):
        """
        Call back for when the apply planning scene future is done.

        Completes the process of adding collision objects to the planning scene.

        Args:
        ----
            future_box (rclpy.client.Future): Future representing result of
            applying planning scene.

        """
        self.node.get_logger().info("end box", once=True)
        self.state = State.DONE

    def future_grasp_callback(self, future_grasp):
        """
        Call back when grasp action future is done, initiates process of getting grasping result.

        Args:
        ----
            future_grasp (rclpy.action.ClientGoalHandle): Future representing grasp action request.

        """
        self.node.get_logger().info('end grasp')
        d = future_grasp.result()
        future_e = d.get_result_async()
        future_e.add_done_callback(self.future_grasp_d_callback)

    def future_grasp_d_callback(self, future_e):
        """
        Call back for when the get result future of grasping is done. Complete grasp process.

        Args:
        ----
            future_e (rclpy.action.ClientGoalHandle): The future representing
            the result of grasping.

        """
        self.node.get_logger().info('end end grasp')
        self.state = State.DONE

    def get_root_frame_id(self):
        """
        Retrieve the root frame ID of the robot based on the TF tree.

        Raises
        ------
            RuntimeError: If the TF tree is not valid.

        """
        frames = {}

        all_transforms = self.tf_current.transforms + self.tf_static.transforms

        for tf in all_transforms:
            tf: TransformStamped

            parant = tf.header.frame_id
            child = tf.child_frame_id

            self.node.get_logger().info(f"tf: {parant} -> {child}")

            frames[child] = parant

            if parant not in frames.keys():
                frames[parant] = None

        for frame in frames:
            if not frames[frame]:
                self.frame_id = frame
                return

        raise RuntimeError("TF tree is not valid")

    def FormatPositionIKRequest(self):
        """
        Format the messages that get sent to the compute IK request.

        Requires:
            string: group_name
            --- from get_robot_state---
            RobotState message: moveit_msgs/RobotState: robot_state
            Constraints: constraints
            bool: is_diff
            ---------------------------
            bool: avoid_collisions
            string: ik_link_name (OPTIONAL)
            geometry_msgs/PoseStamped: pose_stamped
            string[]: ik_link_names (OPTIONAL)
            geometry_msgs/PoseStamped[]: pose_stamped_vector (OPTIONAL)
            builtin_interfaces/Duration: timeout
        """
        self.node.get_logger().info("in format", once=True)
        self.robot_state.joint_state = self.js_current
        # self.node.get_logger().info("Goal 2")
        request = GetPositionIK.Request()
        position = PositionIKRequest()
        position.group_name = self.group_name
        position.robot_state = self.robot_state
        # position.constraints = []
        position.avoid_collisions = True
        position.pose_stamped.header.stamp = (self.node.get_clock().now().to_msg())
        position.pose_stamped.header.frame_id = self.frame_id

        position.pose_stamped.pose = self.desired_pose
        position.timeout.sec = 5
        request.ik_request = position
        # self.node.get_logger().info("Goal 3")
        self.node.get_logger().info("out format", once=True)
        return request
