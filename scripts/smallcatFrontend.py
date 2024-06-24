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
    KittingPart
)
from ariac_msgs.srv import (
    SubmitOrder,

)
from std_srvs.srv import Trigger
from std_msgs.msg import (
    String,
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
)

from moveit import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionFK
)

import PyKDL
# shared data


import sys
import signal

# handle ^C
def signal_handler(sig, frame):
    print("you pressed Crtl+C")
    rclpy.shutdown()


class ARIACenv:
    # sensor data
    # raw camera data
    kts_1_camera = []
    kts_2_camera = []

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
    kts_1 = []
    kts_2 = []

    # robots
    # agv location
    # agv tray held
    # agv status?
    agv_1 = []
    agv_2 = []
    agv_3 = []
    agv_4 = []

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

    def __init__(self):
        pass

    def find_tray(self):
        pass

    def find_part(self, part: Part):
        '''
        return the first part found in stored of camera messages
        TODO
        not the closest part, no particular bin/location, not even the part's world coordinate
        '''
        # search left bins
        for part_pose in self.left_bins_camera.part_poses:
            if part_pose.part.type == part.type and part_pose.part.color == part.color:
                # return part_pose
                return self.multiply_pose(self.left_bins_camera.sensor_pose, part_pose.pose)
        
        # search right bins
        for part_pose in self.right_bins_camera.part_poses:
            if part_pose.part.type == part.type and part_pose.part.color == part.color:
                # return part_pose
                return self.multiply_pose(self.right_bins_camera.sensor_pose, part_pose.pose)

        # search conveyor

        # search other AGVs

        # search assembly stations

        # not found until now?
        return False
    
    def multiply_pose(self, p1: Pose, p2: Pose) -> Pose:
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
                # ariac_env.robots.floor_robot_kitting_PP_tray()


            for _part in o.kitting_task.parts:
                _part: KittingPart
                # self.get_logger().info(f"\t{_part.part.color} {_part.part.type}")
                # self.get_logger().info(f"\t{self.colors[_part.part.color]} {self.types[_part.part.type]}")
                # self.get_logger().info(f"\t---")
            
                # check for parts
                part_validated = ariac_env.find_part(_part.part)

                # if part in env, get location
                if part_validated:

                    # equip gripper
                    # ariac_env.robots.floor_robot_equip_gripper()

                    ariac_env.robots.floor_robot_kitting_PP_part(part_validated, _part.quadrant)
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
    sub1 - left bin camera image
    sub2 - right bin camera image
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

    def sub1_cb(self, msg: AdvancedLogicalCameraImage):
        ariac_env.left_bins_camera = msg

    def sub2_cb(self, msg: AdvancedLogicalCameraImage):
        ariac_env.right_bins_camera = msg

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
    def __init__(self):
        super().__init__("Robots")

        self._ariac_robots = MoveItPy(node_name="ariac_robots_moveit_py")
        self._ariac_robots_state = RobotState(
            self._ariac_robots.get_robot_model())

        self._floor_robot = self._ariac_robots.get_planning_component(
            "floor_robot")
        self._ceiling_robot = self._ariac_robots.get_planning_component(
            "ceiling_robot")

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

    def floor_robot_kitting_PP_part(self, pose, quadrant):
        '''
        move to part in bin
        pick part
        move to agv
        place part in quadrant of agv tray
        '''
        pose: Pose

        mod_pose = pose

        self.log_pose("target pose", pose)

        # self._ariac_robots_state.update()

        with self._planning_scene_monitor.read_write() as scene:
            scene.current_state.update()
            self._ariac_robots_state = scene.current_state

            gripper_current_pose = self._ariac_robots_state.get_pose("floor_gripper")
            
            self.log_pose("gripper current pose", gripper_current_pose)

            # increase the height of the gripper in the pose goal
            mod_pose.position.z += 0.3
            mod_pose.orientation = gripper_current_pose.orientation

            self.log_pose("mod_pose", mod_pose)
                        
        self.get_logger().info("--- floor_robot_kitting_PP_part() ---")
        
        self.floor_robot_move_to_pose(mod_pose)


    def floor_robot_kitting_PP_tray(self):
        pass

    def floor_robot_equip_tray_gripper(self):
        pass

    def floor_robot_equip_part_gripper(self):
        pass

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

    def floor_robot_move_to_pose(self, pose: Pose):
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)

            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "world"
            pose_goal.pose = pose
            self._floor_robot.set_goal_state(pose_stamped_msg=pose_goal, pose_link="floor_gripper")

        while not self._plan_and_execute(self._ariac_robots, self._floor_robot, self.get_logger(), "floor_robot"):
            pass
    
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
            robot.execute(robot_trajectory, controllers=["floor_robot_controller", "linear_rail_controller"] if robot_type == "floor_robot" else ["ceiling_robot_controller","gantry_controller"])
        else:
            logger.error("planning failed")
            return False
        return True

    def log_pose(self, text, pose: Pose):
        self.get_logger().info(f"{text}")
        self.get_logger().info(f"\tp.x {pose.position.x}")
        self.get_logger().info(f"\tp.y {pose.position.y}")
        self.get_logger().info(f"\tp.z {pose.position.z}")
        self.get_logger().info(f"\to.x {pose.orientation.x}")
        self.get_logger().info(f"\to.y {pose.orientation.y}")
        self.get_logger().info(f"\to.z {pose.orientation.z}")
        self.get_logger().info(f"\to.w {pose.orientation.w}")        


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

    # make a thread pool for the nodes to swim in
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(smallcatccs)
    executor.add_node(orderprocessor)
    executor.add_node(sensors)
    executor.add_node(robots)

    # whirlpool in the thread pool
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    # what is this?
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
