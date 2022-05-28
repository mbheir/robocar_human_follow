import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from 


#Custom message created for storing detections
# detection.center_x
# detection.center_y
# detection.box_width
# detection.box_height
DETECTION_TOPIC_NAME = '/detections'
from custom_interfaces.msg import Detection 

# Check actuator package at https://gitlab.com/ucsd_robocar2/ucsd_robocar_actuator2_pkg/-/blob/master/ucsd_robocar_actuator2_pkg/vesc_twist_node.py
ACTUATOR_TOPIC_NAME = '/cmd_vel'
from geometry_msgs.msg import Twist




class PID_Node(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('exercise31')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            Detection, DETECTION_TOPIC_NAME, self.detection_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def detection_callback(self, msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[180]
        self.laser_right = msg.ranges[90]

    def motion(self):
        # print the data
        self.get_logger().info(
            f'Forward {self.laser_forward}\nRight{self.laser_right}\n')
        # Logic of move
        if self.laser_forward < 0.5:
            self.cmd.angular.z = 0.7
        else:
            if self.laser_right > 0.3:
                self.cmd.angular.z = -0.1
            elif self.laser_right < 0.2:
                self.cmd.angular.z = 0.1
            else:  # between 0.3 and 0.2
                self.cmd.angular.z = 0.0

        self.cmd.linear.x = 0.05

        # Publishing the cmd_vel values to topipc
        self.publisher_.publish(self.cmd)
        
if __name__ == "__main__":

