#Custom message created for storing detections
# detection.center_x
# detection.center_y
# detection.box_width
# detection.box_height
from custom_interfaces.msg import Detection 


class PID_Node(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('exercise31')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        self.laser_right = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def move_turtlebot(self, msg):
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

