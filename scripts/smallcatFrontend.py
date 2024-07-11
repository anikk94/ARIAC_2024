#!/usr/bin/env python3
import rclpy
import rclpy.callback_groups
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import threading
import time

from ariac_msgs.msg import (
    CompetitionState,
    Order,
    AdvancedLogicalCameraImage,
    PartLot,
    PartPose,
    Parts,
    Part,
    KittingPart,
    VacuumGripperState,
)
from ariac_msgs.srv import (
    SubmitOrder,
    VacuumGripperControl,
    ChangeGripper

)
from std_srvs.srv import Trigger
from std_msgs.msg import (
    String,
    Header
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Quaternion
)

from moveit import MoveItPy
from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionFK
)
from moveit.core.kinematic_constraints import construct_joint_constraint

import PyKDL
# shared data


import sys
import signal

from copy import deepcopy


from typing import List, Tuple

import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# handle ^C


def signal_handler(sig, frame):
    print("you pressed Crtl+C")
    rclpy.shutdown()

class Error(Exception):
  def __init__(self, value: str):
      self.value = value

  def __str__(self):
      return repr(self.value)


class ARIACenv:
    # sensor data
    # raw camera data
    # kts1_camera = []
    # kts2_camera = []
    # kts1_camera = None
    # kts2_camera = None
    kts1_camera: AdvancedLogicalCameraImage
    kts2_camera: AdvancedLogicalCameraImage

    # left_bins_camera = []
    # right_bins_camera = []

    left_bins_camera: AdvancedLogicalCameraImage
    right_bins_camera: AdvancedLogicalCameraImage

    as_1_camera = []
    as_2_camera = []
    as_3_camera = []
    as_4_camera = []

    conveyor_camera = []

    floor_robot_camera = []
    ceiling_robot_camera = []

    # orders
    # functions?

    # part type/colour
    # part pose
    # fill these when needed out of information from raw camera data buckets
    bin_1 = []
    bin_2 = []
    bin_3 = []
    bin_4 = []
    bin_5 = []
    bin_6 = []
    bin_7 = []
    bin_8 = []

    # parts installed
    as_1 = []
    as_2 = []
    as_3 = []
    as_4 = []

    # kit tray stations/tool change stations
    kts1 = []
    kts2 = []

    # robots
    # agv location
    # agv tray held
    # agv status?
    agv_1 = {
        "location": 4.8,
        "tray": None,
    }
    agv_2 = {
        "location": 1.2,
        "tray": None,
    }
    agv_3 = {
        "location": -1.2,
        "tray": None,
    }
    agv_4 = {
        "location": -4.8,
        "tray": None,
    }
    agv = {
        1: {
            # add other position/location info here
            "location": 4.8,
            "tray": None,
        },
        2: {
            "location": 1.2,
            "tray": None,
        },
        3: {
            "location": -1.2,
            "tray": None,
        },
        4: {
            "location": -4.8,
            "tray": None,
        },
    }

    # parts held
    # quadrant information
    tray_1 = []
    tray_2 = []
    tray_3 = []
    tray_4 = []
    tray_5 = []
    tray_6 = []

    # location information
    # gripper information
    # health
    floor_robot = []
    celing_robot = []

    # parts seen on conveyor
    # part locations
    conveyor = []

    # warehouse?
    warehouse = []

    # ARIAC part dimensions
    # L x B(W) x H
    part_sizes = {
        Part.BATTERY: [0.13, 0.05, 0.04],
        # TODO
        Part.PUMP: [0.1, 0.1, 0.12],
        # TODO
        Part.SENSOR: [],
        # TODO
        Part.REGULATOR: [],
    }

    tray_size = {
        "L": 0.52,
        "W": 0.38,
        "H": 0.01,
    }

    # known_poses

    known_floor_robot_configurations = {

    }

    # known_floor_arm_configurations
    
    # known_ceiling_robot_configurations

    # known_ceiling_arm_configurations



    def __init__(self):
        pass

    def find_tray(self):
        pass

    def find_part(self, part: Part) -> PartPose:
        '''
        return PartPose message of the first part found in stored of camera messages
        TODO
        not the closest part, no particular bin/location, not even the part's world coordinate
        '''
        # search left bins
        for part_pose in self.left_bins_camera.part_poses:
            if part_pose.part.type == part.type and part_pose.part.color == part.color:
                # return part_pose
                # return self.multiply_pose(self.left_bins_camera.sensor_pose, part_pose.pose)
                part_pose.pose = self.multiply_pose(
                    self.left_bins_camera.sensor_pose, part_pose.pose)
                return part_pose

        # search right bins
        for part_pose in self.right_bins_camera.part_poses:
            if part_pose.part.type == part.type and part_pose.part.color == part.color:
                # return part_pose
                # return self.multiply_pose(self.right_bins_camera.sensor_pose, part_pose.pose)
                part_pose.pose = self.multiply_pose(
                    self.right_bins_camera.sensor_pose, part_pose.pose)
                return part_pose

        # search conveyor

        # search other AGVs

        # search assembly stations

        # not found until now?
        return False

    def multiply_pose(self, p1: Pose, p2: Pose) -> Pose:
        # quaternion multiplication is equivalent to channel rotation
        o1 = p1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(o1.x, o1.y, o1.z, o1.w),
            PyKDL.Vector(p1.position.x, p1.position.y, p1.position.z)
        )
        o2 = p2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(o2.x, o2.y, o2.z, o2.w),
            PyKDL.Vector(p2.position.x, p2.position.y, p2.position.z)
        )
        frame3 = frame1 * frame2

        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def rpy_from_quaternion(self, q: Quaternion) -> Tuple[float, float, float]:
        ''' 
        Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
        Args:
            q (Quaternion): quaternion to convert
        Returns:
            Tuple[float, float, float]: roll, pitch, yaw
        '''

        R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        return R.GetRPY()

    # def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    def quaternion_from_rpy(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        q_msg = Quaternion()
        q_msg.w = q[0]
        q_msg.x = q[1]
        q_msg.y = q[2]
        q_msg.z = q[3]

        return q_msg


# WorldData class ends here
ariac_env = ARIACenv()


class SmallCatCCS(Node):
    '''
    SmallCatCCS interfaces CCS to AM
    sub1 - competition state subscriber
    sub2 - order processing complete subscriber
    cli1 - competition start service client
    cli2 - competition end service client
    cb_group_1 - [ME] competition state
    '''

    def __init__(self):
        super().__init__("SmallCatCCS")

        self.competition_state = None

        self.order_processing_complete = False

        self.cb_group_1 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        # self.cb_group_2 = rclpy.callback_groups.ReentrantCallbackGroup()

        # TODO
        # defining callback group is needed for async service calls to not get stuck
        # both Reentrant and MutuallyExclusive work (why?)
        self.sub1 = self.create_subscription(
            CompetitionState,
            "/ariac/competition_state",
            self.sub1_cb,
            10,
            callback_group=self.cb_group_1,
        )

        self.sub2 = self.create_subscription(
            String,
            "/smallcat/CCS_OrderProcessor",
            self.sub2_cb,
            10,
            callback_group=self.cb_group_1,
        )

        self.pub1 = self.create_publisher(
            String,
            "/smallcat/CCS_OrderProcessor",
            10
        )

        self.cli1 = self.create_client(
            Trigger,
            "/ariac/start_competition"
        )

        self.cli2 = self.create_client(
            Trigger,
            "/ariac/end_competition"
        )

    # wait for competition state to be ready
    # competitor lifecycle manager
    def sub1_cb(self, msg: CompetitionState):
        '''
        competition state subscriber callback

        CompetitionState.IDLE                      = 0
        CompetitionState.READY                     = 1
        CompetitionState.STARTED                   = 2
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE  = 3
        CompetitionState.ENDED                     = 4
        '''
        if self.competition_state == msg.competition_state:
            return
        self.competition_state = msg.competition_state
        self.get_logger().info(f"competition state: {self.competition_state}")

        if self.competition_state == CompetitionState.IDLE:
            #     # just wait
            self.get_logger().info("waiting")

        elif self.competition_state == CompetitionState.READY:
            #     # call start competition service
            self.get_logger().info("starting competition")

        elif self.competition_state == CompetitionState.STARTED:
            #     # listen for orders and process orders
            self.get_logger().info("listening for orders")

        elif self.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
            #     # continue processing orders until all submitted
            self.get_logger().info("all orders announced")

        elif self.competition_state == CompetitionState.ENDED:
            #     # call end competition serivce
            self.get_logger().info("shutting down SmallCatCCS")

    def sub2_cb(self, msg: String):
        '''
        subscriber to order processor node
        '''
        if msg.data == "order_processing_complete":
            self.order_processing_complete = True

    # start competition
    def start_competition_service_call(self):
        '''
        call the start competition service
        '''
        # when this function is called, it means that the program is ready to start competition
        # check if service is available now with 1second timeout
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/ariac/start_competition service not available, waiting again...")
        # make a service request object
        req = Trigger.Request()
        # return future from async call
        future = self.cli1.call_async(req)
        # resolve future object
        while not future.done():
            # time.sleep(0.1)
            # self.get_logger().info(f"comp start: waiting for future raw>{future.done()}")
            continue
        # check call response
        if future.result().success:
            self.get_logger().info("Competition started")
        else:
            self.get_logger().error("Failed to start competition")

    # end competition
    def end_competition_service_call(self):
        '''
        call the end competition service
        '''
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/ariac/end_competition service not available, waiting again")
        req = Trigger.Request()
        future = self.cli2.call_async(req)
        while not future.done():
            # time.sleep(0.1)
            # self.get_logger().info("comp end: waiting for future")
            continue
        if future.result().success:
            self.get_logger().info("Competition ended")
        else:
            self.get_logger().error("Failed to end competition")

    def start_order_processing(self):
        '''
        publish message to order processor node
        '''
        msg = String()
        msg.data = "order_processing_start"
        self.pub1.publish(msg)

# SmallCatCCS class ends here


class OrderProcessor(Node):
    '''
    Order Processing Node
    reads global (singleton?) world data shared object (database?) - which holds sensor data and robot functions

    sub1 - orders subscriber
    sub2 - ccs order processor communication
    pub1 - ccs order processor communication
    cli1 - submit order service client
    cb_group_1 - [ME] orders
    '''

    def __init__(self):
        super().__init__("OrderProcessor")

        # order status
        self.PENDING = 0
        self.COMPLETE = 1
        self.LOADING = 2
        self.PAUSED = 3

        self.colors = {
            Part.RED: "red",
            Part.GREEN: "green",
            Part.BLUE: "blue",
            Part.ORANGE: "orange",
            Part.PURPLE: "purple",
        }

        self.types = {
            Part.BATTERY: "battery",
            Part.PUMP: "pump",
            Part.SENSOR: "sensor",
            Part.REGULATOR: "regulator",
        }

        self.orders_received = 0

        self.orders_completed = 0

        self.orderbook = []

        self.worker_thread_started = False

        self.cb_group_1 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.sub1 = self.create_subscription(
            Order,
            "/ariac/orders",
            self.sub1_cb,
            10,
            callback_group=self.cb_group_1,
        )

        self.sub2 = self.create_subscription(
            String,
            "/smallcat/CCS_OrderProcessor",
            self.sub2_cb,
            10,
            callback_group=self.cb_group_1,
        )

        self.pub1 = self.create_publisher(
            String,
            "/smallcat/CCS_OrderProcessor",
            10
        )

        self.cli1 = self.create_client(
            SubmitOrder,
            "/ariac/submit_order",
        )

     # listen for orders
    def sub1_cb(self, msg: Order):
        '''
        new_order = {
            "order_msg": ,
            "order_status": ,
        }
        '''
        new_order = {
            "order_msg": msg,
            "order_status": self.PENDING,
        }
        self.orderbook.append(new_order)
        self.get_logger(). info(
            f"Order received. ID: {msg.id}. orderbook size: {len(self.orderbook)}")
        self.orders_received += 1

    def sub2_cb(self, msg: String):
        if msg.data == "order_processing_start":
            # start order processing thread
            self.start_worker_thread()
            self.worker_thread_started = True

    def start_worker_thread(self):
        if not self.worker_thread_started:
            self.worker_thread = threading.Thread(target=self.process_orders)
            self.get_logger().info("starting worker thread")
            self.worker_thread.start()

    def process_orders(self):
        self.get_logger().info("process_orders()")
        # TODO
        # just sitting and waiting in a while loop?
        while self.orders_received == 0:
            continue

        while self.orders_completed < self.orders_received:  # or competition_state < 3:
            # flip through orders and do them
            for order in self.orderbook:
                # check if order is completed in order to skip it
                if order["order_status"] == self.COMPLETE:
                    continue

                # PAUSED ORDER

                # PENDING/unworked ORDER
                # work on order
                self.process_current_order(order)

                # TEMP TEST CODE
                # self.robot_test()

                # submit order
                self.submit_order(order['order_msg'].id)
                self.orders_completed += 1

            self.get_logger().info("Completed all orders on book - waiting for more")

        self.get_logger().info("All orders processed and submitted")

        # stop order processing thread

        # communicate with SmallCatCCS Node
        msg = String()
        msg.data = "order_processing_complete"
        self.pub1.publish(msg)

    def process_current_order(self, order):
        self.get_logger().info(f"working on order: {order['order_msg'].id}")
        time.sleep(2)

        o: Order = order["order_msg"]

        # if kitting or combined - start kitting
        if o.type == Order.KITTING or o.type == Order.COMBINED:
            # self.get_logger().info(f"Kitting order parts:")

            # PP tray
            # find tray
            # ariac_env.find_tray()

            # equip tray gripper
            # ariac_env.robots.floor_robot_equip_tray_gripper()

            # PP tray
            ariac_env.robots.floor_robot_kitting_PP_tray(o.kitting_task.tray_id, o.kitting_task.agv_number)

            for _part in o.kitting_task.parts:
                _part: KittingPart

                # find part, get pose
                part_validated = ariac_env.find_part(_part.part)

                # if a PartPose is returned
                if part_validated:

                    # ariac_env.robots.floor_robot_pick_part(part_validated, _part.part.type)
                    # ariac_env.robots.floor_robot_flip_held_part(_part.part.type)

                    ariac_env.robots.floor_robot_kitting_PP_part(part_validated, o.kitting_task.agv_number, _part.quadrant)

                    # TODO
                    # break
                else:
                    self.get_logger().error(f"part missing {_part.part.type, _part.part.color}")
                    break

            # PP tray
            # for part in order
                # PP part
                # check and fix faulty/flipped
                # check dropped part
                # check gripper fault
                # move on next part
            # lock and move AGV

        # if assembly or combined - start assembly
            # move AGV if needed
            # for part in order
            # PP&A part

    # call the submit order service

    def submit_order(self, order_id):
        req = SubmitOrder.Request()
        req.order_id = order_id
        future = self.cli1.call_async(req)
        while not future.done():
            continue
        if future.result().success:
            self.get_logger().info(f"submitted order: {order_id}")
        else:
            self.get_logger().error(f"failed to submit order {order_id}")

    def robot_test(self):
        # test 1
        pose = Pose()
        pose.position.x = -1.720000
        pose.position.y = 2.445000
        pose.position.z = 0.720000 + 1
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        ariac_env.robots.floor_robot_move_to_pose(pose)

        # test 2
        self.get_logger().warn("moving to test position start")
        ariac_env.robots.floor_robot_move_test_position()
        self.get_logger().warn("moving to test position complete")

# OrderProcessor class ends here


class Sensors(Node):
    '''
    sensor node
    sub1 - left bin camera
    sub2 - right bin camera
    sub3 - kts1 camera
    sub4 - kts2 camera
    cb_group_1 - [ME] sensor data
    '''

    def __init__(self):
        super().__init__("Sensors")

        self.cb_group_1 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.sub1 = self.create_subscription(
            AdvancedLogicalCameraImage,
            "/ariac/sensors/left_bins_camera/image",
            self.sub1_cb,
            qos_profile_sensor_data,
            callback_group=self.cb_group_1,
        )

        self.sub2 = self.create_subscription(
            AdvancedLogicalCameraImage,
            "/ariac/sensors/right_bins_camera/image",
            self.sub2_cb,
            qos_profile_sensor_data,
            callback_group=self.cb_group_1,
        )

        self.sub3 = self.create_subscription(
            AdvancedLogicalCameraImage,
            "/ariac/sensors/kts1_camera/image",
            self.sub3_cb,
            qos_profile_sensor_data,
            callback_group=self.cb_group_1,
        )

        self.sub4 = self.create_subscription(
            AdvancedLogicalCameraImage,
            "/ariac/sensors/kts2_camera/image",
            self.sub4_cb,
            qos_profile_sensor_data,
            callback_group=self.cb_group_1,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_pose_transform(self, parent_frame="world", child_frame=""):
        try:
            t = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {parent_frame} to {child_frame}: {ex}')
            return
        p = Pose()
        p.orientation = t.transform.rotation
        p.position.x = t.transform.translation.x
        p.position.y = t.transform.translation.y
        p.position.z = t.transform.translation.z
        return p

    def sub1_cb(self, msg: AdvancedLogicalCameraImage):
        ariac_env.left_bins_camera = msg

    def sub2_cb(self, msg: AdvancedLogicalCameraImage):
        ariac_env.right_bins_camera = msg

    def sub3_cb(self, msg: AdvancedLogicalCameraImage):
        ariac_env.kts1_camera = msg

    def sub4_cb(self, msg: AdvancedLogicalCameraImage):
        ariac_env.kts2_camera = msg

    def conveyor_agent():
        '''
        this agent will in a thread.
        purpose:
        its job is to list and store all the parts 
        that are detected on the conveyor

        optional:
        keep track of the latest part on the conveyor
        keep track of the speed of the parts on the conveyor
        information about part offset that spawns

        run this thread from constructor
        '''
        pass

# Sensors class ends here


class Robots(Node):
    '''
    robots node
    sub1 - floor robot gripper state
    sub2 - ceiling robot gripper state
    cli1 - floor robot gripper client
    cli2 - ceiling robot gripper client
    cb_group_1 - [ME] gripper state
    '''

    def __init__(self):
        super().__init__("Robots")

        self.gripper_states = {
            True: "enabled",
            False: "disabled",
        }

        self.cb_group_1 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self._ariac_robots = MoveItPy(node_name="ariac_robots_moveit_py")

        self._ariac_robots_state = RobotState(
            self._ariac_robots.get_robot_model())

        self._floor_robot = self._ariac_robots.get_planning_component(
            "floor_robot")
        self._ceiling_robot = self._ariac_robots.get_planning_component(
            "ceiling_robot")
        self._floor_arm = self._ariac_robots.get_planning_component(
            "floor_arm")

        # self._floor_robot_home_quaternion = Quaternion()
        # self._ceiling_robot_home_quaternion = Quaternion()

        self._planning_scene_monitor = self._ariac_robots.get_planning_scene_monitor()

        self._world_collision_objects = []

        # part pose information
        # above

        # camera pose information
        # above

        self.get_cartesian_path_client = self.create_client(
            GetCartesianPath, "compute_cartesian_path")
        self.get_position_fk_client = self.create_client(
            GetPositionFK, "compute_fk")

        self.sub1 = self.create_subscription(
            VacuumGripperState,
            "/ariac/floor_robot_gripper_state",
            self.sub1_cb,
            10,
            callback_group=self.cb_group_1,
        )

        self.cli1 = self.create_client(
            VacuumGripperControl,
            "/ariac/floor_robot_enable_gripper",
        )

        self.cli2 = self.create_client(
            ChangeGripper,
            "/ariac/floor_robot_change_gripper",
        )

    # home
    #  linear_actuator_joint     :  0
    #  floor_shoulder_pan_joint  :  0
    #  floor_shoulder_lift_joint : -1.571
    #  floor_elbow_joint         :  1.571
    #  floor_wrist_1_joint       : -1.571
    #  floor_wrist_2_joint       : -1.571
    #  floor_wrist_3_joint       :  0.0

    # List of functions:
    # 1.  sub1_cb
    # 2.  floor_robot_kitting_PP_part
    # 3.  floor_robot_kitting_PP_tray
    # 4.  floor_robot_equip_tray_gripper
    # 5.  floor_robot_equip_part_gripper
    # 6.  floor_robot_move_home_position
    # 7.  floor_robot_move_test_position
    # 8.  floor_arm_move_to_saved_position
    # 9.  floor_robot_move_to_pose
    # 10. floor_robot_wait_for_attach
    # 11. floor_robot_set_gripper_state
    # 12. log_pose

    def sub1_cb(self, msg: VacuumGripperState):
        self.floor_robot_gripper_state = msg

    def move_around(self):
        self.get_logger().info("moving around")
        pose_goal = Pose()
        pose_goal.position.x = -1.3
        pose_goal.position.y = self._ariac_robots_state.get_pose(
            "robot_base").position.y + (-1.3 - (-1.992176))  # length of link3
        pose_goal.position.z = self._ariac_robots_state.get_pose(
            "floor_gripper").position.z
        pose_goal.orientation = self._ariac_robots_state.get_pose(
            "floor_gripper").orientation
        waypoints = [pose_goal]
        self.floor_robot_move_cartesian(
            waypoints,
            0.3,
            0.3,
            False)

    def floor_robot_pick_part(self, pose: Pose, type: int):
        '''
        move based on moveit planning scene monitor
        '''
        pose: Pose

        self.log_pose("desired pose", pose)

        # refresh _ariac_robots_state by setting it to updated planning scene monitor state
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

        ee_pose = self._ariac_robots_state.get_pose("floor_gripper")

        self.log_pose("ee pose", ee_pose)
        desired_robot_state = RobotState(self._ariac_robots.get_robot_model())
        desired_robot_state.set_joint_group_positions(
            "floor_robot", [-pose.position.y, 0, -1.571, 1.571, -1.571, -1.571, 0])
        self._floor_robot.set_start_state_to_current_state()
        self._floor_robot.set_goal_state(robot_state=desired_robot_state)
        self._plan_and_execute(self._ariac_robots, self._floor_robot,
                               self.get_logger(), "floor_robot", sleep_time=0.0)

        # refresh _ariac_robots_state by setting it to updated planning scene monitor state
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

        # move to part
        pose_goal = deepcopy(pose)
        # + (0.001)*0.5*0.5 # this distance is the threshold for picking
        pose_goal.position.z += ariac_env.part_sizes[type][2]
        # assuming floor robot gripper is currently is parallel to part top surface
        pose_goal.orientation = self._ariac_robots_state.get_pose(
            "floor_gripper").orientation
        self.floor_robot_move_to_pose(pose_goal)

        # refresh _ariac_robots_state by setting it to updated planning scene monitor state
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

        # suck part
        self.floor_robot_set_gripper_state(True)
        # wait for attach to be true
        while self.floor_robot_gripper_state.attached == False:
            self.get_logger().info("waiting for floor gripper attach")
            time.sleep(0.2)
        else:
            self.get_logger().info("successful floor gripper attach")

        # refresh _ariac_robots_state by setting it to updated planning scene monitor state
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

        # retract to upright position with part in gripper
        desired_robot_state = RobotState(self._ariac_robots.get_robot_model())
        desired_robot_state.set_joint_group_positions(
            "floor_robot", [-pose.position.y, 0, -1.571, 1.571, -1.571, -1.571, 0])
        self._floor_robot.set_start_state_to_current_state()
        self._floor_robot.set_goal_state(robot_state=desired_robot_state)
        self._plan_and_execute(self._ariac_robots, self._floor_robot,
                               self.get_logger(), "floor_robot", sleep_time=0.0)

    def floor_robot_flip_held_part(self, type):
        if 0:
            # refresh _ariac_robots_state by setting it to updated planning scene monitor state
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
            # turn toward rail
            pose_goal = Pose()
            pose_goal.position.x = -1.3  # center for side bar
            pose_goal.position.y = self._ariac_robots_state.get_pose(
                "robot_base").position.y + (-1.3 - (-1.992176))  # length of link3
            pose_goal.position.z = self._ariac_robots_state.get_pose(
                "floor_gripper").position.z
            pose_goal.orientation = self._ariac_robots_state.get_pose(
                "floor_gripper").orientation
            # pose move
            # self.floor_robot_move_to_pose(pose_goal)
            # cartesian move
            self.floor_robot_move_cartesian([pose_goal], 0.3, 0.3, False)
            # refresh _ariac_robots_state by setting it to updated planning scene monitor state
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
            # lower
            pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
            # 0.00025# height of side bar + part height
            pose_goal.position.z = 0.93 + ariac_env.part_sizes[type][2] + 0.1
            # pose move
            # self.floor_robot_move_to_pose(pose_goal)
            # cartesian move
            self.floor_robot_move_cartesian([pose_goal], 0.3, 0.3, False)
        else:
            # refresh _ariac_robots_state by setting it to updated planning scene monitor state
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
            # turn toward rail and lower
            pose_goal = Pose()
            pose_goal.position.x = -1.3  # center for side bar
            pose_goal.position.y = self._ariac_robots_state.get_pose(
                "robot_base").position.y + (-1.3 - (-1.992176))  # length of link3
            # side bar prism z + side bar thickness / 2 + height of part
            pose_goal.position.z = 0.93 + \
                (0.1/2) + ariac_env.part_sizes[type][2] + 0.01
            pose_goal.orientation = self._ariac_robots_state.get_pose(
                "floor_gripper").orientation
            # pose move
            # self.floor_robot_move_to_pose(pose_goal)
            # cartesian move
            # self.floor_robot_move_cartesian([pose_goal], 0.3, 0.3, True)
            self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, False)

            # # lerp path
            # current_pose = self._ariac_robots_state.get_pose("floor_gripper")
            # waypoints = []
            # steps = 10
            # for i in range(steps+1):
            #     p = Pose()
            #     p.orientation = current_pose.orientation
            #     p.position.x = current_pose.position.x + (pose_goal.position.x - current_pose.position.x) * (i/steps)
            #     p.position.y = current_pose.position.y + (pose_goal.position.y - current_pose.position.y) * (i/steps)
            #     p.position.z = current_pose.position.z + (pose_goal.position.z - current_pose.position.z) * (i/steps)
            #     waypoints.append(p)
            # for i in range(len(waypoints)):
            #     self.log_pose(f"{i}", waypoints[i])
            #     self.get_logger().info("---")
            # cartesian move
            # self.floor_robot_move_cartesian(waypoints, 0.3, 0.3, True)

        # drop part
        self.floor_robot_set_gripper_state(False)

        # move away
        # refresh _ariac_robots_state by setting it to updated planning scene monitor state
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        # retract to upright position with part in gripper
        pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
        pose_goal.position.z += 0.2
        self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, False)

        # revolve about part
        # refresh _ariac_robots_state by setting it to updated planning scene monitor state
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

        pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
        pose_goal.position.x += 0.1
        pose_goal.position.z = 0.93 + 0.1/2 + 0.05 + 0.01  # 1 cm + radius of gripper
        temp_e = list(ariac_env.rpy_from_quaternion(pose_goal.orientation))
        temp_e[0] += math.pi/2
        pose_goal.orientation = ariac_env.quaternion_from_rpy(
            temp_e[0], temp_e[1], temp_e[2])

        self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, False)

        # approach and suck part
        self.floor_robot_set_gripper_state(True)
        while not self.floor_robot_gripper_state.attached:
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
            pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
            pose_goal.position.x -= 0.005
            self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, False)
            self.get_logger().info("waiting for gripper attach")
            time.sleep(0.2)
        else:
            self.get_logger().info("gottem")

        # lift
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
        pose_goal.position.z += 0.2
        self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, False)

        # rotate -90 abt x(of gripper) first time
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
        pose_goal.position.x = -1.3
        pose_goal.position.z += ariac_env.part_sizes[type][2]/2
        temp_e = list(ariac_env.rpy_from_quaternion(pose_goal.orientation))
        # self.get_logger().info("temp_e base")
        # self.get_logger().info(f"r{temp_e[0]}")
        # self.get_logger().info(f"p{temp_e[1]}")
        # self.get_logger().info(f"y{temp_e[2]}")
        temp_e[0] -= math.pi/2
        # self.get_logger().info("temp_e r1")
        # self.get_logger().info(f"r{temp_e[0]}")
        # self.get_logger().info(f"p{temp_e[1]}")
        # self.get_logger().info(f"y{temp_e[2]}")
        pose_goal.orientation = ariac_env.quaternion_from_rpy(
            temp_e[0], temp_e[1], temp_e[2])
        self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, False)

        # rotate -90 abt x(of gripper) second time
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
        # half width distance away in x
        pose_goal.position.x -= ariac_env.part_sizes[type][1]/2
        # similar but for height
        pose_goal.position.z -= ariac_env.part_sizes[type][2]/2
        temp_e = list(ariac_env.rpy_from_quaternion(pose_goal.orientation))
        # self.get_logger().info("temp_e r1 base")
        # self.get_logger().info(f"r{temp_e[0]}")
        # self.get_logger().info(f"p{temp_e[1]}")
        # self.get_logger().info(f"y{temp_e[2]}")
        temp_e[0] -= math.pi/2 * 1.01  # this is to adjust moveit's mood
        # self.get_logger().info("temp_e r1 base r2")
        # self.get_logger().info(f"r{temp_e[0]}")
        # self.get_logger().info(f"p{temp_e[1]}")
        # self.get_logger().info(f"y{temp_e[2]}")
        pose_goal.orientation = ariac_env.quaternion_from_rpy(
            temp_e[0], temp_e[1], temp_e[2])
        self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, False)

        # lower
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
        # side bar height + side bar thickness/2 + 1 cm + radius of gripper + gripper hitting rail, change order of rotation (90 while placing another 90 while picking)
        pose_goal.position.z = 0.93 + 0.1/2 + 0.05 + 0.01 + 0.1
        self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, True)

        # place
        self.floor_robot_set_gripper_state(False)

        # move away
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        pose_goal = self._ariac_robots_state.get_pose("floor_gripper")
        pose_goal.position.x -= 0.2
        self.floor_arm_move_cartesian([pose_goal], 0.3, 0.3, True)

        # upright
        self.floor_arm_move_to_saved_position("upright")

    def floor_robot_place_part(self):
        pass

    def floor_robot_kitting_PP_part(self, part_validated, agv_number, quadrant):
        # def floor_robot_kitting_PP_part(self, pose, quadrant):
        '''
        move part from bin to agv tray
        '''
        self.get_logger().info(f"floor_robot_kitting_PP_part()")

        part_validated: PartPose

        if 0:
            mod_pose = deepcopy(pose)

            self.log_pose("target pose", pose)

            # self._ariac_robots_state.update()

            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()

                self._ariac_robots_state = scene.current_state

                gripper_current_pose = self._ariac_robots_state.get_pose(
                    "floor_gripper")

                self.log_pose("gripper current pose", gripper_current_pose)

                # increase the height of the gripper in the pose goal
                mod_pose.position.z += 0.3

                mod_pose.orientation = gripper_current_pose.orientation

                self.log_pose("mod_pose", mod_pose)

            self.get_logger().info("--- floor_robot_kitting_PP_part() 1 ---")

            self.floor_robot_move_to_pose(mod_pose)

            mod_pose = deepcopy(pose)

            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()

                self._ariac_robots_state = scene.current_state

                gripper_current_pose = self._ariac_robots_state.get_pose(
                    "floor_gripper")

                self.log_pose("gripper current pose", gripper_current_pose)

                # increase the height of the gripper in the pose goal
                mod_pose.position.z += ariac_env.part_sizes[type][2]  # + 0.002

                mod_pose.orientation = gripper_current_pose.orientation

                self.log_pose("mod_pose", mod_pose)

            self.get_logger().info("--- floor_robot_kitting_PP_part() 2 ---")

            self.floor_robot_move_to_pose(mod_pose)

            self.floor_robot_set_gripper_state(True)

            mod_pose = deepcopy(pose)

            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
                gripper_current_pose = self._ariac_robots_state.get_pose(
                    "floor_gripper")
                self.log_pose("gripper current pose", gripper_current_pose)

                # increase the height of the gripper in the pose goal
                mod_pose.position.z += 0.3

                mod_pose.orientation = gripper_current_pose.orientation

                self.log_pose("mod_pose", mod_pose)

            self.get_logger().info("--- floor_robot_kitting_PP_part() 3 ---")

            self.floor_robot_move_to_pose(mod_pose)

            self.get_logger().info("--- floor_robot_kitting_PP_part() 4 ---")

            self.floor_arm_move_to_saved_position("upright")

            with self._planning_scene_monitor.read_write() as scene:
                self._floor_robot.set_start_state(
                    robot_state=scene.current_state)

                rs = RobotState(self._ariac_robots.get_robot_model())
                rs.set_joint_group_positions(
                    "floor_arm", [1.571, -1.571, 1.571, -1.571, -1.571, 0.0])

                self._floor_robot.set_goal_state(robot_state=rs)

            self.get_logger().info("--- floor_robot_kitting_PP_part() 5 ---")

            self._plan_and_execute(self._ariac_robots, self._floor_arm,
                                   self.get_logger(), "floor_arm", sleep_time=0.0)

            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
                gripper_current_pose = self._ariac_robots_state.get_pose(
                    "floor_gripper")
                self.log_pose("gripper current pose", gripper_current_pose)

                # increase the height of the gripper in the pose goal
                mod_pose = Pose()
                mod_pose.position.x = -1.3
                mod_pose.position.y = 4.8
                mod_pose.position.z = 0.93 + 0.1/2 + \
                    ariac_env.part_sizes[type][2] + 0.01
                mod_pose.orientation = gripper_current_pose.orientation

                self.log_pose("mod_pose", mod_pose)

            self.get_logger().info("--- floor_robot_kitting_PP_part() 6 ---")

            self.floor_robot_move_to_pose(mod_pose)

            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update()
                self._ariac_robots_state = scene.current_state
                gripper_current_pose = self._ariac_robots_state.get_pose(
                    "floor_gripper")
                self.log_pose("gripper current pose", gripper_current_pose)

                # increase the height of the gripper in the pose goal
                mod_pose = Pose()
                mod_pose.position.x = -1.3
                mod_pose.position.y = 4.8
                mod_pose.position.z = 0.93 + 0.1/2 + \
                    ariac_env.part_sizes[type][2] + 0.01

                temp_orientation_rpy = list(
                    ariac_env.rpy_from_quaternion(gripper_current_pose.orientation))
                temp_orientation_rpy[1] += math.pi/2
                temp_orientation_q = ariac_env.quaternion_from_rpy(
                    temp_orientation_rpy[0], temp_orientation_rpy[1], temp_orientation_rpy[2])
                mod_pose.orientation = temp_orientation_q

                self.log_pose("mod_pose", mod_pose)

            self.get_logger().info("--- floor_robot_kitting_PP_part() 7 ---")

            self.floor_robot_move_to_pose(mod_pose)

            # self._floor_robot.set_start_state_to_current_state()

        # move robot base in front of part
        # refresh _ariac_robots_state
        robot_config = [-part_validated.pose.position.y, 0, -1.571, 1.571, -1.571, -1.571, 0]
        self.floor_robot_move_to_robot_state(robot_config)
        # with self._planning_scene_monitor.read_write() as scene:
        #     scene.current_state.update()
        #     self._ariac_robots_state = scene.current_state
        # goal_state = RobotState(self._ariac_robots.get_robot_model())
        # goal_state.set_joint_group_positions(
        #     "floor_robot", [-part_validated.pose.position.y, 0, -1.571, 1.571, -1.571, -1.571, 0])
        # self._floor_robot.set_start_state_to_current_state()
        # self._floor_robot.set_goal_state(robot_state=goal_state)
        # self._plan_and_execute(self._ariac_robots, self._floor_robot,
        #                        self.get_logger(), "floor_robot", sleep_time=0.0)

        # start suck part
        self.floor_robot_set_gripper_state(True)

        # gripper to part top
        # refresh _ariac_robots_state
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        goal_pose = Pose()
        goal_pose.position.x = part_validated.pose.position.x
        goal_pose.position.y = part_validated.pose.position.y
        goal_pose.position.z = part_validated.pose.position.z + \
            ariac_env.part_sizes[part_validated.part.type][2]
        goal_pose.orientation = self._ariac_robots_state.get_pose(
            "floor_gripper").orientation
        self.floor_arm_move_cartesian([goal_pose], 0.3, 0.3, False)

        # raise
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        goal_state = RobotState(self._ariac_robots.get_robot_model())
        goal_state.set_joint_group_positions(
            "floor_robot", [-part_validated.pose.position.y, 0, -1.571, 1.571, -1.571, -1.571, 0])
        self._floor_robot.set_start_state_to_current_state()
        self._floor_robot.set_goal_state(robot_state=goal_state)
        self._plan_and_execute(self._ariac_robots, self._floor_robot,
                               self.get_logger(), "floor_robot", sleep_time=0.0)

        # move to agv
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state
        goal_state = RobotState(self._ariac_robots.get_robot_model())
        goal_state.set_joint_group_positions(
            "floor_robot", [-ariac_env.agv[agv_number]["location"], 0, -1.571, 1.571, -1.571, -1.571, 0])
        self._floor_robot.set_start_state_to_current_state()
        self._floor_robot.set_goal_state(robot_state=goal_state)
        self._plan_and_execute(self._ariac_robots, self._floor_robot,
                               self.get_logger(), "floor_robot", sleep_time=0.0)

        # lower to quadrant
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

        # goal_pose = get_drop_pose(agv, quadrant, part_type)
        goal_pose = Pose()
        goal_pose.orientation = self._ariac_robots_state.get_pose(
            "floor_gripper").orientation
        # agv tray pose: (world)
        #   x: -2.07
        #   y: -4.8, -1.2, 1.2, 4.8
        #   z: 0.76
        goal_pose.position.x = -2.07
        goal_pose.position.z = 0.76
        if agv_number == 1:
            goal_pose.position.y = 4.8
        elif agv_number == 2:
            goal_pose.position.y = 1.2
        elif agv_number == 3:
            goal_pose.position.y = -1.2
        elif agv_number == 4:
            goal_pose.position.y = -4.8

        if quadrant == 1:
            goal_pose.position.x -= ariac_env.tray_size["L"]/4
            goal_pose.position.y -= ariac_env.tray_size["W"]/4
        elif quadrant == 2:
            goal_pose.position.x -= ariac_env.tray_size["L"]/4
            goal_pose.position.y += ariac_env.tray_size["W"]/4
        elif quadrant == 3:
            goal_pose.position.x += ariac_env.tray_size["L"]/4
            goal_pose.position.y -= ariac_env.tray_size["W"]/4
        elif quadrant == 4:
            goal_pose.position.x += ariac_env.tray_size["L"]/4
            goal_pose.position.y += ariac_env.tray_size["W"]/4

        goal_pose.position.z += (
            ariac_env.part_sizes[part_validated.part.type][2])*1.1

        self.floor_arm_move_cartesian([goal_pose], 0.3, 0.3, False)

        # release
        self.floor_robot_set_gripper_state(False)

        # raise

    def floor_robot_kitting_PP_tray(self, tray_id, agv_number):
        '''
        trays need to go from kts{n} to agv{n} only. this functions does that.
        '''
        self.get_logger().info(f"floor_robot_kitting_PP_tray() tray_id: {tray_id} agv_id: {agv_number}")

        # check if tray gripper is equiped

        # PP tray on agv
            # find tray
        tray_found = False
        kts_robot_config = None
        tray_pose = Pose()
        kts = ""
        for tray_pose in ariac_env.kts1_camera.tray_poses:
            if tray_id == tray_pose.id:
                tray_found = True
                kts_robot_config = [5.0,1.57,-1.57,1.57,-1.57,-1.57,0.0]
                tray_pose = ariac_env.multiply_pose(ariac_env.kts1_camera.sensor_pose, tray_pose.pose)
                kts = "kts1"
                break
        if not tray_found:
            for tray_pose in ariac_env.kts2_camera.tray_poses:
                if tray_id == tray_pose.id:
                    tray_found = True
                    kts_robot_config = [-5.0,1.57,-1.57,1.57,-1.57,-1.57,0.0]
                    tray_pose = ariac_env.multiply_pose(ariac_env.kts2_camera.sensor_pose, tray_pose.pose)
                    kts = "kts2"
                    break
        if not tray_found:
            self.get_logger().error(f"\n\n\ntray {tray_id} not found in environment\n\n\n")
            return

        # equip tray gripper
        self.floor_robot_change_gripper("tray", kts)
        
            # move to tray on rail (one end or the other)
        self.floor_robot_move_to_robot_state(robot_config=kts_robot_config)
            # tray approach posiiton
        with self._planning_scene_monitor.read_only() as scene:
            scene.current_state.update()
            tray_pose.orientation = scene.current_state.get_pose("floor_gripper").orientation
        tray_pose.position.z += 0.3
        self.floor_arm_move_cartesian([tray_pose], 0.7, 0.7, False)
            # turn activate gripper
        self.floor_robot_set_gripper_state(True)
            # move to tray surface
        tray_pose.position.z -= 0.3
        self.floor_arm_move_cartesian([tray_pose], 0.7, 0.7, False)   
            # retract to approach position
        tray_pose.position.z += 0.3
        self.floor_arm_move_cartesian([tray_pose], 0.7, 0.7, False)
        self.log_robot_config("floor_robot")
            # move to upright configuration (pre approach position) adn turn toward -x
        kts_robot_config[1] -= math.pi/2
        self.floor_robot_move_to_robot_state(robot_config=kts_robot_config)
        self.log_robot_config("floor_robot")
        
            # move to agv on rail
                # get agv tray world pose
        agv_location = ariac_env.sensors.get_pose_transform("world", f"agv{agv_number}_tray")
                # slide the floor robot to the agv
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._floor_robot.set_start_state(robot_state=scene.current_state)
            joint_values = {
                "linear_actuator_joint": -agv_location.position.y,
            }
            scene.current_state.joint_positions = joint_values
            joint_constraint = construct_joint_constraint(
                robot_state=scene.current_state,
                joint_model_group=self._ariac_robots.get_robot_model().get_joint_model_group("floor_robot"),
            )
            self._floor_robot.set_goal_state(motion_plan_constraints=[joint_constraint])
        self._plan_and_execute(self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot")
            # move to approach position
        self.get_logger().info("placing tray")
        goal_pose = Pose()
        goal_pose.position.x = agv_location.position.x
        goal_pose.position.y = agv_location.position.y
        goal_pose.position.z = agv_location.position.z + 0.3
        with self._planning_scene_monitor.read_only() as scene:
            scene.current_state.update()
            goal_pose.orientation = scene.current_state.get_pose("floor_gripper").orientation
        self.log_pose("goal_pose", goal_pose)
        self.floor_arm_move_cartesian([goal_pose], 1.0, 1.0, False)
            # move to agv surface (tray drop position)
        goal_pose.position.z -= (0.3)*0.9
        self.floor_arm_move_cartesian([goal_pose], 0.3, 0.3, False)
            # deactivate gripper release tray
        self.floor_robot_set_gripper_state(False)
            # retract to approach position
        goal_pose.position.z += 0.3
        self.floor_arm_move_cartesian([goal_pose], 1.0, 1.0, False)
            # 

        # equip part gripper
        self.floor_robot_change_gripper("part", kts)


    # low level movement functions

    def floor_robot_move_home_position(self):
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)
            self._floor_robot.set_goal_state(configuration_name="home")
        self._plan_and_execute(self._ariac_robots, self._floor_robot,
                               self.get_logger(), "floor_robot", sleep_time=0.0)

        # with self._planning_scene_monitor.read_write() as scene:
        #     scene.current_state.update()
        #     self._ariac_robots_state = scene.current_state
        #     self._floor_robot_home_quaternion = self._ariac_robots_state.get_pose("floor_gripper").orientation

    def floor_robot_move_test_position(self):
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)
            self._floor_robot.set_goal_state(configuration_name="sm_test")
        self._plan_and_execute(self._ariac_robots, self._floor_robot,
                               self.get_logger(), "floor_robot", sleep_time=0.0)

        # with self._planning_scene_monitor.read_write() as scene:
        #     scene.current_state.update()
        #     self._ariac_robots_state = scene.current_state
        #     self._floor_robot_home_quaternion = self._ariac_robots_state.get_pose("floor_gripper").orientation

    def floor_arm_move_to_saved_position(self, position_name):
        '''
        floor_arm group positions
        1. upright
        '''
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_arm.set_start_state(robot_state=scene.current_state)
            self._floor_arm.set_goal_state(configuration_name=position_name)
        self._plan_and_execute(self._ariac_robots, self._floor_arm,
                               self.get_logger(), "floor_arm", sleep_time=0.0)

        # with self._planning_scene_monitor.read_write() as scene:
        #     scene.current_state.update()
        #     self._ariac_robots_state = scene.current_state
        #     self._floor_robot_home_quaternion = self._ariac_robots_state.get_pose("floor_gripper").orientation

    def floor_arm_move_to_robot_state(self, arm_config):
        '''
        6 element list for arm joint configuration
        '''
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._floor_robot.set_start_state_to_current_state()
            # self._ariac_robots_state = scene.current_state
        goal_state = RobotState(self._ariac_robots.get_robot_model())
        goal_state.set_joint_group_positions("floor_arm", arm_config)
        # self._floor_robot.set_start_state_to_current_state()
        self._floor_robot.set_goal_state(robot_state=goal_state)
        self._plan_and_execute(self._ariac_robots, self._floor_robot,
                               self.get_logger(), "floor_arm", sleep_time=0.0)

    def floor_robot_move_to_robot_state(self, robot_config):
        '''
        7 element list for arm joint configuration
        '''
        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._floor_robot.set_start_state_to_current_state()
            # self._ariac_robots_state = scene.current_state
        goal_state = RobotState(self._ariac_robots.get_robot_model())
        goal_state.set_joint_group_positions("floor_robot", robot_config)
        # self._floor_robot.set_start_state_to_current_state()
        self._floor_robot.set_goal_state(robot_state=goal_state)
        self._plan_and_execute(self._ariac_robots, self._floor_robot,
                               self.get_logger(), "floor_robot", sleep_time=0.0)

    def floor_robot_move_to_pose(self, pose: Pose):
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)

            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "world"
            pose_goal.pose = pose
            self._floor_robot.set_goal_state(
                pose_stamped_msg=pose_goal, pose_link="floor_gripper")

        while not self._plan_and_execute(self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot"):
            pass
    
    def floor_robot_move_cartesian(self, waypoints, velocity, acceleration, avoid_collision=True):
        trajectory_msg = self._call_get_cartesian_path(
            waypoints, velocity, acceleration, avoid_collision, "floor_robot")
        with self._planning_scene_monitor.read_write() as scene:
            trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())
            trajectory.set_robot_trajectory_msg(
                scene.current_state, trajectory_msg)
            trajectory.joint_model_group_name = "floor_robot"
            scene.current_state.update(True)
            self._ariac_robots_state = scene.current_state
        self._ariac_robots.execute(trajectory, controllers=[])

    def floor_arm_move_cartesian(self, waypoints, velocity, acceleration, avoid_collision=True):
        trajectory_msg = self._call_get_cartesian_path(
            waypoints, velocity, acceleration, avoid_collision, "floor_arm")
        with self._planning_scene_monitor.read_write() as scene:
            trajectory = RobotTrajectory(self._ariac_robots.get_robot_model())
            trajectory.set_robot_trajectory_msg(
                scene.current_state, trajectory_msg)
            trajectory.joint_model_group_name = "floor_arm"
            scene.current_state.update(True)
            self._ariac_robots_state = scene.current_state
        self._ariac_robots.execute(trajectory, controllers=[])

    def _call_get_cartesian_path(
        self,
        waypoints: list,
        max_velocity_scaling_factor: float,
        max_acceleration_scaling_factor: float,
        avoid_collision: bool,
        robot: str
    ):
        self.get_logger().info("getting cartesian path")
        request = GetCartesianPath.Request()
        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
        request.header = header
        with self._planning_scene_monitor.read_write() as scene:
            request.start_state = robotStateToRobotStateMsg(
                scene.current_state)
            if robot == "floor_robot":
                request.group_name = "floor_robot"
                request.link_name = "floor_gripper"
            elif robot == "floor_arm":
                request.group_name = "floor_arm"
                request.link_name = "floor_gripper"
            else:
                request.group_name = "ceiling_robot"
                request.link_name = "ceiling_gripper"
            request.waypoints = waypoints
            request.max_step = 0.1
            request.avoid_collisions = avoid_collision
            request.max_velocity_scaling_factor = max_velocity_scaling_factor
            request.max_acceleration_scaling_factor = max_acceleration_scaling_factor
            future = self.get_cartesian_path_client.call_async(request)
            while not future.done():
                pass
            result: GetCartesianPath.Response
            result = future.result()
            if result.fraction < 0.9:
                self.get_logger().error("unable to plan cartesian trajectory")
            return result.solution

    # basic moveit move function

    def _plan_and_execute(
            self,
            robot,
            planning_component,
            logger,
            robot_type,
            single_plan_parameters=None,
            multi_plan_parameters=None,
            sleep_time=0.0,
    ):
        logger.info("planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        if plan_result:
            logger.info("executing plan")
            with self._planning_scene_monitor.read_write() as scene:
                scene.current_state.update(True)
                self._ariac_robots_state = scene.current_state
                robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=["floor_robot_controller", "linear_rail_controller"]
                          if robot_type == "floor_robot"
                          else ["floor_robot_controller"]
                          if robot_type == "floor_arm"
                          else ["ceiling_robot_controller", "gantry_controller"])
        else:
            logger.error("planning failed")
            return False
        return True

    # gripper functions

    def floor_robot_equip_tray_gripper(self):
        pass

    def floor_robot_equip_part_gripper(self):
        pass

    def floor_robot_wait_for_attach(self, timeout: float):
        pass

    def floor_robot_change_gripper_service(self, gripper_type):
        request = ChangeGripper.Request()
        if gripper_type == "tray":
            request.gripper_type = ChangeGripper.Request.TRAY_GRIPPER
        elif gripper_type == "part":
            request.gripper_type = ChangeGripper.Request.PART_GRIPPER
        future = self.cli2.call_async(request)
        while not future.done():
            pass
        if not future.done():
            raise Error("tool change time out")
        result: ChangeGripper.Response
        result = future.result()
        if not result.success:
            self.get_logger().error("tool change service failed")

    def floor_robot_set_gripper_state(self, state):
        '''
        True/False -> activate/deactivate gripper suck
        '''
        if self.floor_robot_gripper_state.enabled == state:
            self.get_logger().warn(
                f"floor robot: redundant gripper state change to {self.gripper_states[state]}")
            return

        request = VacuumGripperControl.Request()
        request.enable = state

        future = self.cli1.call_async(request)

        while not future.done():
            pass

        if future.result().success:
            self.get_logger().info(
                f"gripper state changed to {self.gripper_states[state]}")
        else:
            self.get_logger().error("failed to change gripper state")

    def floor_robot_change_gripper(self, gripper_type: String, kts="kts1"):
        '''
        this function sets the gripper of the floor robot to the 
        gripper type specified by the gripper_type argument

        gripper_type: ['tray', 'part']

        defaults to kts1
        '''

        # find the closest tool change station

        # equip gripper
            # move to tool change station on rail

            ## 5.0 value is meant to be the value on the floor robot rail 
            ## that corresponds to kts1 
            ## the rail joint values are inverted world y axis values

        robot_config = None
        if kts == "kts1":
            robot_config = [5.0,1.57,-1.57,1.57,-1.57,-1.57,0.0]
        elif kts == "kts2":
            robot_config = [-5.0,1.57,-1.57,1.57,-1.57,-1.57,0.0]

        self.floor_robot_move_to_robot_state(robot_config=robot_config)
            # change gripper type
                # get tool change station world pose
        tcs_world_pose = ariac_env.sensors.get_pose_transform("world", f"{kts}_tool_changer_{gripper_type}s_frame")
                # create pose for robot to move to
        tempQ = list(ariac_env.rpy_from_quaternion(tcs_world_pose.orientation))
                    # rotate 180 about x
        tempQ[0] += math.pi
        tcs_world_pose.orientation = ariac_env.quaternion_from_rpy(tempQ[0], tempQ[1], tempQ[2])
                    # move to approach position
        tcs_world_pose.position.z += 0.5
        self.floor_arm_move_cartesian([tcs_world_pose], 1.0, 1.0, False)
                    # decent to tool change position
        tcs_world_pose.position.z -= 0.5
        self.floor_arm_move_cartesian([tcs_world_pose], 0.3, 0.3, False)
                # call tool change service
        self.floor_robot_change_gripper_service(gripper_type=gripper_type)
                # retract to approach position
        tcs_world_pose.position.z += 0.5
        self.floor_arm_move_cartesian([tcs_world_pose], 1.0, 1.0, False)
            # move to upright configuration
        robot_config = [5.0,1.57,-1.57,1.57,-1.57,-1.57,0.0]
        self.floor_robot_move_to_robot_state(robot_config=robot_config)


    # utility functions

    def log_pose(self, text, pose: Pose):
        '''
        display pose with pose name text
        '''
        self.get_logger().info(f"{text}")
        self.get_logger().info(f"\tp.x {pose.position.x}")
        self.get_logger().info(f"\tp.y {pose.position.y}")
        self.get_logger().info(f"\tp.z {pose.position.z}")
        self.get_logger().info(f"\to.x {pose.orientation.x}")
        self.get_logger().info(f"\to.y {pose.orientation.y}")
        self.get_logger().info(f"\to.z {pose.orientation.z}")
        self.get_logger().info(f"\to.w {pose.orientation.w}")

    def log_robot_config(self, robot_type):
        '''
        display the configuration of the {robot_type}
        '''
        with self._planning_scene_monitor.read_only() as scene:
            scene.current_state.update()
            joint_model_group_names = self._ariac_robots.get_robot_model().joint_model_group_names
            if not robot_type in joint_model_group_names:
                self.get_logger().error(f"robot type {robot_type} not found")
                return
            self.get_logger().info(f"{robot_type} configuration")
            robot_config = scene.current_state.get_joint_group_positions(robot_type)
            for joint in robot_config:
                self.get_logger().info(f"\t{joint}")

    # def log_robot_joints(self, robot_type=""):
    #     '''
    #     joint model group names: ['ceiling_arm', 'ceiling_robot', 'floor_arm', 'floor_robot', 'gantry']
    #     floor_robot:
    #     J: world_to_base
    #     J: linear_actuator_joint
    #     J: floor_base_joint
    #     J: floor_base_link-base_link_inertia
    #     J: floor_shoulder_pan_joint
    #     J: floor_shoulder_lift_joint
    #     J: floor_elbow_joint
    #     J: floor_wrist_1_joint
    #     J: floor_wrist_2_joint
    #     J: floor_wrist_3_joint
    #     J: floor_gripper_joint
    #     L: slide_bar
    #     L: robot_base
    #     L: floor_base_link
    #     L: floor_base_link_inertia
    #     L: floor_shoulder_link
    #     L: floor_upper_arm_link
    #     L: floor_forearm_link
    #     L: floor_wrist_1_link
    #     L: floor_wrist_2_link
    #     L: floor_wrist_3_link
    #     L: floor_gripper

    #     '''
    #     # joint_model_groups = self._ariac_robots.get_robot_model().joint_model_groups
    #     # self.get_logger().info(f"joint model groups: {joint_model_groups}")
    #     # for jmg in joint_model_groups:
    #     #     for i in jmg.joint_model_names:
    #     #         self.get_logger().info(f"J: {i}")
    #     #     for i in jmg.link_model_names:
    #     #         self.get_logger().info(f"L: {i}")
    #     #     self.get_logger().info("---\n")
            
    #     # joint_model_group_names = self._ariac_robots.get_robot_model().joint_model_group_names
    #     # self.get_logger().info(f"joint model group names: {joint_model_group_names}")

    #     with self._planning_scene_monitor.read_only() as scene:
    #         current_state = scene.current_state
    #         joint_positions = current_state.get_joint_group_positions(robot_type)
    #         self.get_logger().info(f"joint positions before scene.update()")
    #         for i in joint_positions:
    #             self.get_logger().info(f"\t{i}")
    #         scene.current_state.update()
    #         current_state = scene.current_state
    #         joint_positions = current_state.get_joint_group_positions(robot_type)
    #         self.get_logger().info(f"joint positions before scene.update()")
    #         for i in joint_positions:
    #             self.get_logger().info(f"\t{i}")



    # convert rpy to quaternion
        # in ariacEnv

    # convert quaternion to rpy
        # in ariacEnv

    # load mesh files

    # add collision object to planning scene

# Robots class ends here


# main
def main(args=None):
    rclpy.init(args=args)

    # initialize ros2 nodes
    smallcatccs = SmallCatCCS()

    orderprocessor = OrderProcessor()

    sensors = Sensors()

    robots = Robots()

    # make robots node a shared object
    ariac_env.robots = robots

    # make sensors node a shared object
    ariac_env.sensors = sensors

    # make a thread pool for the nodes to swim in
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(smallcatccs)
    executor.add_node(orderprocessor)
    executor.add_node(sensors)
    executor.add_node(robots)

    # thread whirlpool
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    # basic node execution
    # rclpy.spin(smallcatccs)
    # rclpy.shutdown()

    # # code testing
    # robots.get_logger().info(f"_ariac_robots")
    # robots.get_logger().info(f"{type(ariac_env.robots._ariac_robots.__dir__())}")
    # for i in ariac_env.robots._ariac_robots.__dir__():
    #     robots.get_logger().info(f"{i}")
    # # robots.get_logger().info(f"")

    # robots.get_logger().info(f"--- _ariac_robots_state.get_planning_scene_monitor()")
    # robots.get_logger().info(f"{type(ariac_env.robots._ariac_robots.get_planning_scene_monitor())}")
    # for i in ariac_env.robots._ariac_robots.get_planning_scene_monitor().__dir__():
    #     robots.get_logger().info(f"{i}")

    # robots.get_logger().info(f"--- _ariac_robots_state.get_planning_scene_monitor().name")
    # robots.get_logger().info(f"{type(ariac_env.robots._ariac_robots.get_planning_scene_monitor().name)}")
    # robots.get_logger().info(ariac_env.robots._ariac_robots.get_planning_scene_monitor().name)

    # # x = ariac_env.robots._ariac_robots.get_planning_scene_monitor().read_only()
    # robots.get_logger().info(f"--- _ariac_robots_state.get_planning_scene_monitor().read_only()")
    # # robots.get_logger().info(f"{type(x)}")
    # # help(x)
    # # robots.get_logger().info(str(help(x)))
    # # y = dir(x)
    # # for i in y:
    # #     robots.get_logger().info(i)

    # # robots.get_logger().info(f"{x.current_state}")

    # with ariac_env.robots._ariac_robots.get_planning_scene_monitor().read_only() as scene:
    #     robots.get_logger().info(f"{scene}")
    #     robots.get_logger().info(f"{dir(scene)}")
    #     a = dir(scene)
    #     for i in a:
    #         robots.get_logger().info(i)
    #     # robots.get_logger().info(f"{dir(scene)}")

    #     robots.get_logger().info("")

    #     robots.get_logger().info(f"{scene.current_state}")
    #     robots.get_logger().info(f"{dir(scene.current_state)}")
    #     b = dir(scene.current_state)
    #     for i in b:
    #         robots.get_logger().info(i)

    #     # robots.get_logger().info(f"{scene.current_state.state_tree}")

    #     robots.get_logger().info(f"--- {scene.current_state.robot_model}")
    #     robots.get_logger().info(f"{type(scene.current_state.robot_model)}")
    #     c = dir(scene.current_state.robot_model)
    #     for i in c:
    #         robots.get_logger().info(f"{i}")
    #     # robots.get_logger().info(f"{scene.current_state.robot_model}")

    # robots.get_logger().info(f"{(scene.current_state.robot_model.joint_model_group_names)}")
    # robots.get_logger().info(f"{(scene.current_state.robot_model.joint_model_groups)}")
    #     for i in scene.current_state.robot_model.joint_model_groups:
    #         robots.get_logger().info(f"{(i.link_model_names)}")
    #         robots.get_logger().info(f"{(i.joint_model_names)}")
    #         robots.get_logger().info(f"---")

    #     d = scene.current_state.get_pose('floor_gripper')
    #     robots.get_logger().info(f"{type(d)}")
    #     robots.get_logger().info(f"{d.position.x}")
    #     robots.get_logger().info(f"{d.position.y}")
    #     robots.get_logger().info(f"{d.position.z}")
    #     robots.get_logger().info(f"{d.orientation.x}")
    #     robots.get_logger().info(f"{d.orientation.y}")
    #     robots.get_logger().info(f"{d.orientation.z}")
    #     robots.get_logger().info(f"{d.orientation.w}")

    # robots.get_logger().info(f"--- _ariac_robots.get_planning_component('floor_robot')")
    # for i in ariac_env.robots._ariac_robots.get_planning_component("floor_robot").__dir__():
    #     robots.get_logger().info(f"{i}")
    #     # robots.get_logger().info(f"{type(i)}")

    # robots.get_logger().info(f"--- _ariac_robots.get_planning_component('floor_robot').named_target_states")
    # robots.get_logger().info(f"{ariac_env.robots._ariac_robots.get_planning_component('floor_robot').named_target_states}")
    # robots.get_logger().info(f"_ariac_robots.get_planning_component('floor_robot').get_named_target_state_values('home').items()")
    # for k, v in ariac_env.robots._ariac_robots.get_planning_component('floor_robot').get_named_target_state_values('home').items():
    #     print(k, ":", v)
    # robots.get_logger().info(f"{ariac_env.robots._ariac_robots.get_planning_component('floor_robot').get_named_target_state_values('home')}")
    # print(f">> {ariac_env.robots._ariac_robots.get_planning_component('floor_robot').__dir__()}")
    # print(f">> {ariac_env.robots._ariac_robots.get_planning_component('floor_robot').planning_group_name}")

    # control ccs state outside of the class
    while rclpy.ok():
        if smallcatccs.competition_state == CompetitionState.IDLE:
            pass

        elif smallcatccs.competition_state == CompetitionState.READY:
            smallcatccs.start_competition_service_call()

        elif smallcatccs.competition_state == CompetitionState.STARTED:
            smallcatccs.start_order_processing()

        elif smallcatccs.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
            if smallcatccs.order_processing_complete == False:
                smallcatccs.start_order_processing()
            if smallcatccs.order_processing_complete == True:
                smallcatccs.end_competition_service_call()

        elif smallcatccs.competition_state == CompetitionState.ENDED:
            rclpy.shutdown()

        smallcatccs.get_logger().info("rclpy.ok()")

        # control the volume or messages
        time.sleep(1)

    # wait for threads to swim away after ros shutdown
    spin_thread.join()

    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()


# ROBOTS
# /agv1_controller/commands
# /agv1_controller/transition_event
# /agv2_controller/commands
# /agv2_controller/transition_event
# /agv3_controller/commands
# /agv3_controller/transition_event
# /agv4_controller/commands
# /agv4_controller/transition_event
# /ariac/agv1_status
# /ariac/agv2_status
# /ariac/agv3_status
# /ariac/agv4_status
# /ceiling_robot_controller/controller_state
# /ceiling_robot_controller/joint_trajectory
# /ceiling_robot_controller/transition_event
# /ceiling_robot_static_controller/transition_event
# /floor_robot_controller/controller_state
# /floor_robot_controller/joint_trajectory
# /floor_robot_controller/transition_event
# /floor_robot_static_controller/transition_event
# /gantry_controller/controller_state
# /gantry_controller/joint_trajectory
# /gantry_controller/transition_event
# /linear_rail_controller/controller_state
# /linear_rail_controller/joint_trajectory
# /linear_rail_controller/transition_event
# /ariac/robot_health

# /controller_manager/robot_description
# /dynamic_joint_states
# /robot_description
# /joint_state_broadcaster/transition_event
# /joint_states

# ARIAC COMPONENTS
# /ariac/assembly_insert_1_assembly_state
# /ariac/assembly_insert_2_assembly_state
# /ariac/assembly_insert_3_assembly_state
# /ariac/assembly_insert_4_assembly_state
# /ariac/bin_parts
# /ariac/ceiling_robot_gripper_state
# /ariac/competition_state
# /ariac/conveyor_parts
# /ariac/conveyor_state
# /ariac/environment_ready
# /ariac/floor_robot_gripper_state
# /ariac/orders
# /ariac/trial_config
# /clock
# /diagnostics
# /parameter_events
# /performance_metrics

# ROS
# /rosout
# /tf
# /tf_static

# SENSORS
# /ariac/sensor_health
# /ariac/sensors/as1_camera/image
# /ariac/sensors/as2_camera/image
# /ariac/sensors/as3_camera/image
# /ariac/sensors/as4_camera/image
# /ariac/sensors/conveyor_breakbeam/change
# /ariac/sensors/conveyor_breakbeam/status
# /ariac/sensors/conveyor_camera/image
# /ariac/sensors/kts1_camera/image
# /ariac/sensors/kts2_camera/image
# /ariac/sensors/left_bins_camera/image
# /ariac/sensors/right_bins_camera/image
