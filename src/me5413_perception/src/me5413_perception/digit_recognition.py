import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, String
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import os
import threading
import numpy as np

# Absolute path of YOLOv8 weights
current_dir = os.path.dirname(os.path.abspath(__file__))
weights_path = os.path.join(current_dir, 'yolov8', 'best_detect.pt')

class DigitRecognizer:
    '''
    Recognition of digits using YOLOv8
    :param: callback_fn
        Callback function for passing the point in the form
        of geometry_msgs.msg.PointStamped
    '''
    def __init__(self, callback_fn):
        self.target = 3
        self.bridge = CvBridge()
        self.latest_image = None
        self.depth_image = None
        self.image_lock = threading.Lock()
        self.depth_image_lock = threading.Lock()
        self.callback_fn = callback_fn
        self.model = YOLO(weights_path)
        self.sub_depth_camera = rospy.Subscriber(
            '/kinect/rgb/image_raw',
            Image, self.image_callback)
        self.sub_camera = rospy.Subscriber(
            '/kinect/depth/image_raw',
            Image, self.depth_image_callback)

    # def set_target(self, target):
    #     self.TARGET = target

    '''
    Callback function listening on camera topic /kinect/rgb/image_raw.
    Internally converts from sensor_msgs.msg.Image to cv::Mat
    :param: data (type: sensor_msgs.msg.Image)
        Image from topic
    '''
    def image_callback(self, data):
        try:
            with self.image_lock:
                self.latest_image = self.bridge.imgmsg_to_cv2(
                    data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    '''
    Callback function listening on camera topic /kinect/depth/image_raw.
    Internally converts from sensor_msgs.msg.Image to cv::Mat
    :param: data (type: sensor_msgs.msg.Image)
        Image from topic
    '''
    def depth_image_callback(self, data):
        try:
            with self.depth_image_lock:
                self.depth_image = self.bridge.imgmsg_to_cv2(
                    data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    '''
    Timer callback running the YOLOv8 pipeline
    '''
    def timer_callback(self, event):
        with self.image_lock:
            if self.latest_image is None:
                return
            image_to_process = self.latest_image.copy()

        with self.depth_image_lock:
            if self.depth_image is None:
                return
            depth_image_to_process = self.depth_image.copy()

        results = self.model.predict(
            source=image_to_process,
            conf=0.7,
            save_conf=True,
            show=True,
            verbose=False)
        if results[0].boxes.xyxy.device.type == 'cuda':
            np_coordinates = results[0].boxes.xyxy.cpu().numpy()
            np_classes = results[0].boxes.cls.cpu().numpy()
        else:
            np_coordinates = results[0].boxes.xyxy.numpy()
            np_classes = results[0].boxes.cls.numpy()

        if not len(np_classes):
            rospy.loginfo(f'No detection')
            return

        index = self.__find_number_in_array(np_classes, self.target)
        if index < 0:
            rospy.loginfo(f'Target {self.target} not detected')
            return

        [x1, y1, x2, y2] = np_coordinates[index, :]
        x_center = int((x1 + x2) / 2)
        y_center = int((y1 + y2) / 2)

        if x_center not in range(0, depth_image_to_process.shape[1]) or \
            y_center not in range(0, depth_image_to_process.shape[0]):
            rospy.logwarn('Index out of bounds')
            return

        depth = depth_image_to_process[y_center, x_center]
        if np.isnan(depth):
            rospy.logwarn("Depth value is NaN at ({}, {})".format(x_center, y_center))
            return
        self.callback_fn(self.__create_coordinate_msg(x_center, y_center, depth))

    '''
    Internal function to create coordinate message of type
    geometry_msgs.msg.PointStamped
    :param: x
        X Coordinate of center point of bounding box
        in camera frame
    :param: y
        Y Coordinate of center point of bounding box
        in camera frame
    :param: z
        Depth value extracted from depth image at (X, Y)
    :return: Message of type geometry_msgs.msg.PointStamped
    '''
    def __create_coordinate_msg(self, x, y, z):
        msg = PointStamped()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/kinect_frame_optical'
        msg.header = header
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        return msg

    '''
    Intrernal function to find a number in a numpy array
    and return index
    :param: array
        Numpy array of numbers
    :param: number
        Number to find
    :returns: Index of the number (if multiple occurances then
        the first index is returned) otherwise -1 if not found
    '''
    def __find_number_in_array(self, array, number):
        indices = np.where(array == number)[0]
        print(indices)
        if len(indices) > 0:
            return indices[0]
        else:
            return -1
