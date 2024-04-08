#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import os
import threading
import torch
import numpy as np
import re

current_dir = os.path.dirname(os.path.abspath(__file__))
weights_path = os.path.join(current_dir, 'yolov8', 'best_detect.pt')

class DigitRecognizer:
    def __init__(self, callback_fn):
        # rospy.init_node('digit_recognizer')
        self.target = 3
        self.bridge = CvBridge()
        self.latest_image = None
        self.depth_image = None
        self.image_lock = threading.Lock()
        self.depth_image_lock = threading.Lock()
        # self.pub_coordinate = None
        # self.sub_camera  = None
        # self.sub_depth_camera = None
        # self.timer = None
        # rospy.Subscriber('/rviz_panel/goal_name', String, self.rviz_goal_callback)
        # self.running = False
        # self.pub_coordinate = None
        self.callback_fn = callback_fn
        # rospy.spin()
        self.model = YOLO(weights_path)
        self.sub_depth_camera = rospy.Subscriber('/kinect/rgb/image_raw', Image, self.image_callback)
        self.sub_camera = rospy.Subscriber('/kinect/depth/image_raw', Image, self.depth_image_callback)

    # def rviz_goal_callback(self, msg):
    #     # print(msg)
    #     if "/box_" in msg.data:
    #         match = re.search(r'box_(\d+)', msg.data)
    #         if match:
    #             TARGET = int(match.group(1))
    #             if self.running == False:
    #                 print("Starting Perception")
    #                 self.set_target(TARGET)
    #                 self.run()
    #             else:
    #                 print("Changing target.")
    #                 self.set_target(TARGET)
    #     else:
    #         if self.running == False:
    #             return
    #         else:
    #             rospy.signal_shutdown("Killing node!!")

    # def set_target(self, target):
    #     self.TARGET = target

    def image_callback(self, data):
        try:
            with self.image_lock:
                self.latest_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def depth_image_callback(self, data):
        try:
            with self.depth_image_lock:
                self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

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
            conf=0.5,
            save_conf=True,
            show=True)
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

        # if 0 <= x_center < depth_image_to_process.shape[1] and 0 <= y_center < depth_image_to_process.shape[0]:
        depth = depth_image_to_process[y_center, x_center]
        if np.isnan(depth):
            rospy.logwarn("Depth value is NaN at ({}, {})".format(x_center, y_center))
            return
        self.callback_fn(self.__create_coordinate_msg(x_center, y_center, depth))
        # else:
        #     rospy.logwarn("Coordinates out of bounds.")
        # else:
        #     rospy.loginfo("Target not found.")

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

    # def coordinates_publisher(self, x, y, z):
    #     msg = PointStamped()
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = "Image frame"
    #     msg.header = header
    #     msg.point.x = x
    #     msg.point.y = y
    #     msg.point.z = z
    #     self.pub_coordinate.publish(msg)

    def __find_number_in_array(self, array, number):
        indices = np.where(array == number)[0]
        print(indices)
        if len(indices) > 0:
            return indices[0]
        else:
            return -1

    # def run(self):
    #     self.running = True
    #     self.model = YOLO(self.weights_path)
    #     self.latest_image = None
    #     self.depth_image = None
    #     self.sub_depth_camera = rospy.Subscriber('/kinect/rgb/image_raw', Image, self.image_callback)
    #     self.sub_camera = rospy.Subscriber('/kinect/depth/image_raw', Image, self.depth_image_callback)
    #     self.pub_coordinate = rospy.Publisher('/target/coordinates', PointStamped, queue_size= 10)
    #     self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

# if __name__ == '__main__':
#     rospy.init_node('digit_recognizer', anonymous=True)
#     recognizer = DigitRecognizer()
