# -*- coding: utf-8 -*-
"""
Created on Sun May 29 19:25:21 2022

@author: Team 2 robocar1
"""
from custom_interfaces.msg import Detection
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile


# Custom message created for storing detections
# detection.center_x
# detection.center_y
# detection.box_width
# detection.box_height
DETECTION_TOPIC_NAME = '/detections'

# Check actuator package at https://urldefense.proofpoint.com/v2/url?u=https-3A__gitlab.com_ucsd-5Frobocar2_ucsd-5Frobocar-5Factuator2-5Fpkg_-2D_blob_master_ucsd-5Frobocar-5Factuator2-5Fpkg_vesc-5Ftwist-5Fnode.py&d=DwIGAg&c=-35OiAkTchMrZOngvJPOeA&r=RSuX6zNtjR8QyV9bB4B7mpCedielV-h7SUddZpjVIs8&m=dsI64WVV0y1GedjWrwkE-ZyyTn6UHEM3Y1l7i-W6jzU8qI-8zI3LNm1NE7Gn-nf2&s=ZK5G2HJG0eQ6j8moB7jqjsxUXnevRcu09GX80BHj_hg&e= 
# TO RUN THE ACTUATOR ROS PACKAGE
# ros2 launch ucsd_robocar_actuator2_pkg vesc_twist.launch.py
# THIS WILL START THE ACTUATOR NODE LISTENING TO /cmd_vel
ACTUATOR_TOPIC_NAME = '/cmd_vel'


class PID_Node(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('exercise31')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            Detection, DETECTION_TOPIC_NAME, self.detection_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # create a Twist message
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg):
        # REACT TO A DETECTION HERE
        self.x = msg.center_x
        self.y = msg.center_y
        # This publishes the value to the terminal
        self.get_logger().info(str(self.x))
        # This code attaches the values from self.x into steering commands
        #if self.x < 0:
        #self.cmd.linear.x = 0.05
            #self.cmd.angular.x = 3
     
    
    def timer_callback(self):
        # WE WANT TO STOP THE CAR IF NO DETECTIONS FOR A CERTAIN AMOUNT OF TIME

        # Publish driving command example
        cmd = Twist()
        cmd.linear.x = 0.1

        # Publishing the cmd_vel values to topipc
        self.publisher_.publish(cmd)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    pid_node = PID_Node()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(pid_node)
    # Explicity destroy the node
    pid_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == "__main__":
    main()



