#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import os
import threading
import torch
import numpy as np

class DigitRecognizer:
    TARGET = 3

    def __init__(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(current_dir, '..', 'yolov8', 'best_detect.pt')
        self.model = YOLO(weights_path)
        self.bridge = CvBridge()
        self.latest_image = None
        self.depth_image = None
        self.image_lock = threading.Lock()
        self.depth_image_lock = threading.Lock()
        self.pub_coordinate = None

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
            if self.latest_image is not None:
                image_to_process = self.latest_image.copy()
            else:
                return

        with self.depth_image_lock:
            if self.depth_image is not None:
                depth_image_to_process = self.depth_image.copy()
            else:
                return

        results = self.model.predict(source=image_to_process, conf=0.5, save_conf=True, show=True)
        coordinates = results[0].boxes.xyxy
        classes = results[0].boxes.cls
        np_coordinates = coordinates.numpy()
        np_classes = classes.numpy()
        if not len(np_classes):
            return
        
        index = self.find_number_in_array(np_classes, self.TARGET)
        if index < 0:
            [x1, y1, x2, y2] = np_coordinates[index, :]
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)

            if 0 <= x_center < depth_image_to_process.shape[1] and 0 <= y_center < depth_image_to_process.shape[0]:
                depth = depth_image_to_process[y_center, x_center]
                if np.isnan(depth):
                    rospy.logwarn("Depth value is NaN at ({}, {})".format(x_center, y_center))
                else:
                    self.coordinates_publisher(x_center, y_center, depth)
            else:
                rospy.logwarn("Coordinates out of bounds.")
        else:
            rospy.loginfo("Target not found.")

    def coordinates_publisher(self, x, y, z):
        msg = PointStamped()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "Image frame"
        msg.header = header
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        self.pub_coordinate.publish(msg)

    def find_number_in_array(self, array, number):
        indices = np.where(array == number)[0]
        if indices.size > 0:
            return indices[0]
        else:
            return -1

    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/kinect/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/kinect/depth/image_raw', Image, self.depth_image_callback)
        self.pub_coordinate = rospy.Publisher('/target/coordinates', PointStamped, queue_size= 10)
        timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
        rospy.spin()

if __name__ == '__main__':
    recognizer = DigitRecognizer()
    recognizer.run()
