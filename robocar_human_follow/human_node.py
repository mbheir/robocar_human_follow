
# ROS Related Imports
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.msg import Detection

# Other Imports
import numpy as np
import cv2
import argparse

# Import YOLOv5 Stuff
from elements.yolo import OBJ_DETECTION

# Define YOLO Network
Object_classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
                  'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                  'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
                  'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
                  'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                  'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                  'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
                  'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
                  'hair drier', 'toothbrush']
Object_colors = list(np.random.rand(80, 3)*255)
Object_detector = OBJ_DETECTION('weights/yolov5s.pt', Object_classes)


def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


# open webcam video stream
cap = cv2.VideoCapture(0)

# Parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--showVideo', action='store_true',
                    help='whether to show video or not')
opt = parser.parse_args()


class HumanDetector(Node):

    def __init__(self, show_video=opt.showVideo):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('human_detector')
        # create the publisher object
        self.publisher_ = self.create_publisher(Detection, 'detections', 10)
        self.show_video = show_video
        self.x_rez = 640
        self.y_rez = 480

    def detect_human(self):
        # print the data
        if cap.isOpened():
            # Open window to show video
            if self.show_video:
                window_handle = cv2.namedWindow(
                    "CSI Camera", cv2.WINDOW_AUTOSIZE)
            while True:
                # Read from camera and perform YOLO detection
                ret, frame = cap.read()
                if ret:
                    # detection process
                    objs = Object_detector.detect(frame)
                    max_box_size = 0
                    detection = None
                    for obj in objs:
                        label = obj['label']
                        # Check for person
                        if label == 'person':
                            score = obj['score']
                            [(xmin, ymin), (xmax, ymax)] = obj['bbox']
                            dx = xmax - xmin
                            dy = ymax - ymin
                            if dx*dy > max_box_size:
                                max_box_size = dx*dy
                                detection = Detection()
                                detection.center_x = xmin + dx//2 - self.x_rez//2
                                detection.center_y = ymin + dy//2 - self.y_rez//2
                                detection.width = dx
                                detection.height = dy
                                sxmin,sxmax,symin,symax = xmin,xmax,ymin,ymax 
                # Handle detection
                if detection != None:
                    self.get_logger().info(
                        f"Detection (x,y) = ({detection.center_x},{detection.center_y})  \tBox (width,height) = ({detection.width},{detection.height}))")

                    # Publish to topic
                    self.publisher_.publish(detection)

                    # reset detection
                    detection = None

                    # Plotting to Display (for debug information, not important)
                    color = Object_colors[Object_classes.index('person')]
                    frame = cv2.rectangle(
                        frame, (sxmin, symin), (sxmax, symax), color, 2)
                    frame = cv2.putText(frame, f'person ({str(score)})', (
                        sxmin, symin), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 1, cv2.LINE_AA)
                # Display detections
                if self.show_video:
                    cv2.imshow("CSI Camera", frame)
                    keyCode = cv2.waitKey(30)
                    if keyCode == ord('q'):
                        break
            cap.release()
            cv2.destroyAllWindows()
        else:
            print("Unable to open camera")


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    human_detector = HumanDetector()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    human_detector.detect_human()
    # Explicity destroy the node
    human_detector.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
	
