
# ROS Related Imports
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
from std_msgs.msg import String

# Other Imports
import numpy as np
import cv2


# --- HOG DETECTOR ---
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

cv2.startWindowThread()

# open webcam video stream
cap = cv2.VideoCapture(0)


class HumanDetector(Node):

    def __init__(self, show_video=True):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('human_detector')
        # create the publisher object
        self.publisher_ = self.create_publisher(String, 'detections', 10)
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.detect_human)
        self.show_video = show_video

    def detect_human(self):
        # print the data
        global cap
        detection = String()
        detection.data = "test string"

        # --- INSERT DETECTION ALGO HERE ---
        # Capture frame-by-frame
        ret, frame = cap.read()
        print(frame)
        cv2.imshow('frame', frame)
	
        frame = cv2.resize(frame, (640, 480))

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # for faster detection
        boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8))

        if self.show_video == True:
            for (xA, yA, xB, yB) in boxes:
                # display the detected boxes in the colour picture
                cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
        

        # Publish detection here

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

