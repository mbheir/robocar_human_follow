
# ROS Related Imports
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces import Detection

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
        self.publisher_ = self.create_publisher(Detection, 'detections', 10)
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.detect_human)
        self.show_video = show_video

    def detect_human(self):
        # print the data
        global cap
        detection = Detection()

        # --- INSERT DETECTION ALGO HERE ---
        # Capture frame-by-frame
        ret, frame = cap.read()
        print(type(frame))
        #cv2.imshow('frame', frame)

        frame = cv2.resize(frame, (640, 480))

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # for faster detection
        boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return
        if self.show_video == True:
            for (xA, yA, xB, yB) in boxes:
                # display the detected boxes in the colour picture
                cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
            cv2.imshow('l', np.array(frame, dtype=np.uint8))
        # Publish detection here

        # ---
        if len(boxes) > 0:
            detection.center_x = (xB-xA)//2
            detection.center_y = (yB-yA)//2
            detection.width = (xB-xA)
            detection.height = (yB-yA)
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

