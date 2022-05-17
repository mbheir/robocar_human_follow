import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
from std_msgs.msg import String

class HumanDetector(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('human_detector')
        # create the publisher object
        self.publisher_ = self.create_publisher(String, 'detections', 10)
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.detect_human)


    def detect_human(self):
        # print the data
        detection = String()
        detection.data = "test string"
        
        # --- INSERT DETECTION ALGO HERE --- 
        
        
        
        # --- 
        
        self.get_logger().info(f'Sending detection')
        # Publishing the cmd_vel values to topipc
        self.publisher_.publish(detection)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    
    
    human_detector = HumanDetector()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(human_detector)
    # Explicity destroy the node
    human_detector.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()

