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
        if self.latest_image is not None:
            with self.image_lock:
                image_to_process = self.latest_image.copy()

            results = self.model.predict(source=image_to_process, conf=0.5, save_conf=True, show=True)

            for r in results:
                box = r.boxes
                box.cpu()
                box.numpy()
                coordinates = box.xywh
                classes = box.cls
            np_coordinates = coordinates.numpy()
            np_classes = classes.numpy()

            index = self.find_number_in_array(np_classes, self.TARGET)
            if index != -1:
                [x1, y1, w, h] = np_coordinates[index, :]
                x_center = int(x1 + w / 2)
                y_center = int(y1 + h / 2)
                if 0 <= x_center < self.depth_image.shape[0] and 0 <= y_center < self.depth_image.shape[1]:
                    depth = self.depth_image[x_center, y_center]
                    # print(f'Depth image coordinates: {depth}')
                    self.coordinates_publisher(x_center, y_center, depth)
                else:
                    print("Out of bounds.")

            else:
                print("Looking for target.")

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
        self.pub_coordinate = rospy.Publisher('/target/coordinates', PointStamped, 10)
        timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
        rospy.spin()

if __name__ == '__main__':
    recognizer = DigitRecognizer()
    recognizer.run()
