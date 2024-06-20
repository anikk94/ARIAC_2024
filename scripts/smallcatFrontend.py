#!/usr/bin/env python3
import rclpy
import rclpy.callback_groups
import rclpy.executors
from rclpy.node import Node

import threading
import time

from ariac_msgs.msg import (
    CompetitionState,
    Order,
)
from std_srvs.srv import Trigger
from std_msgs.msg import (
    String,
)

# shared data object


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

        self.cli1 = self.create_client(Trigger, "/ariac/start_competition")    

        self.cli2 = self.create_client(Trigger, "/ariac/end_competition")

    # wait for competition state to be ready
    # competitor lifecycle manager
    def sub1_cb(self, msg: CompetitionState):
        '''
        competition state subscriber callback
        '''
        if self.competition_state == msg.competition_state:
            return
        self.competition_state = msg.competition_state
        self.get_logger().info(f"competition state: {self.competition_state}")

        if self.competition_state == 0:
            self.get_logger().info("waiting")
            # just wait
            pass
        elif self.competition_state == 1:
            # AM is ready
            # call start competition service
            self.get_logger().info("starting competition")
            self.start_competition_service_call()
            # pass
        elif self.competition_state == 2:
            # listen for orders and process orders
            # call order processing service to start work of order processing node
            # don't work on orders in this node
            self.get_logger().info("listening for orders")
            msg = String()
            msg.data = "order_processing_start"
            self.pub1.publish(msg)
            
            # pass
        elif self.competition_state == 3: 
            # continue processing orders until all submitted
            self.get_logger().info("all orders announced")
            # check if orders received are processed
            # TODO
            # if not, reset internal competition state to 2, so this elif is triggered again
            # this should be fixed by making the lifecycle manager run in a while loop in the main function
            
            # TODO
            # check if all orders are processed
            # don't sit in a while loop waiting for order to complete after announcements are complete
            # this is bad code in a callback
            # while self.order_processing_complete == False:
            #     continue
            
            # reset competition state to 2 so that this code is triggered again
            if self.order_processing_complete == False:
                self.competition_state = 2
                # make sure order processing has started
                msg = String()
                msg.data = "order_processing_start"
                self.pub1.publish(msg)
                return
            
            self.get_logger().info("received order processing completed message")

            self.get_logger().info("ending competition")
            self.end_competition_service_call()
            # pass
        elif self.competition_state == 4:
            # call end competition serivce
            self.get_logger().info("shutting down ARIAC Manager")
            rclpy.shutdown()
            # pass
        
    def sub2_cb(self, msg: String):
        if msg.data == "order_processing_complete":
            self.order_processing_complete = True


    # start competition 
    def start_competition_service_call(self):
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
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/ariac/end_competition service not available, waiting again")
        req = Trigger.Request()
        future=self.cli2.call_async(req)
        while not future.done():
            # time.sleep(0.1)
            # self.get_logger().info("comp end: waiting for future")
            continue
        if future.result().success:
            self.get_logger().info("Competition ended")
        else:
            self.get_logger().error("Failed to end competition")
        


   

    # process orders

# SmallCatCCS class ends here

class OrderProcessor(Node):
    '''
    Order Processing Node
    reads global (singleton?) world data shared object (database?) - which holds sensor data and robot functions

    sub1 - orders subscriber
    cb_group_1 - [ME] orders
    '''
    def __init__(self):
        super().__init__("OrderProcessor")

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

        # self.process_orders()


     # listen for orders
    def sub1_cb(self, msg: Order):
        self.orderbook.append(msg)
        self.get_logger(). info(f"Order received. ID: {msg.id}. orderbook size: {len(self.orderbook)}")
        self.orders_received +=1
    
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
        while self.orders_received == 0:
            continue

        while self.orders_completed < self.orders_received: # or competition_state < 3:
            for order in self.orderbook:
                # check if order is completed in order to skip it
                self.get_logger().info(f"working on order {order.id}")
                # work on order
                time.sleep(2)
                # submit order
                self.orders_completed += 1
        
            self.get_logger().info("Completed all orders on book - waiting for more")
        
        self.get_logger().info("All orders processed and submitted")

        # stop order processing thread
        
        # communicate with SmallCatCCS Node
        msg = String()
        msg.data = "order_processing_complete"
        self.pub1.publish(msg)            


# OrderProcessor class ends here


class Sensors(Node):
    '''
    sensor node

    '''
    def __init__(self):
        super().__init__("Sensors")

        # self.cb_group_1 = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # self.sub1 = self.create_subscription(
        #     Order,
        #     "/ariac/sensor_data",
        #     self.sub1_cb,
        #     10,
        #     callback_group=self.cb_group_1,
        # )

    # def sub1_cb(self, msg: Order):
    #     self.get_logger().info("sensor data received")

# main
def main(args=None):
    rclpy.init(args=args)

    smallcatccs = SmallCatCCS()

    orderprocessor = OrderProcessor()

    sensors = Sensors()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(smallcatccs)
    executor.add_node(orderprocessor)
    executor.add_node(sensors)
    
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    # rclpy.spin(smallcatccs)
    # rclpy.shutdown()

    while rclpy.ok():
        time.sleep(1)
        print("smallcatCCS rclpy.ok()")
        # continue
    spin_thread.join()



if __name__== "__main__":
    main()