#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import os
import threading
import torch
import numpy as np



# Initialize global variables
current_dir = os.path.dirname(os.path.abspath(__file__))
weights_path = os.path.join(current_dir, '..', 'yolov8', 'best_detect.pt')
model = YOLO(weights_path)
latest_image = None
depth_image = None
bridge = CvBridge()
image_lock = threading.Lock()
depth_image_lock = threading.Lock()  # Ensure thread-safe access to the latest_image
TARGET = 3

def image_callback(data):
    global latest_image
    try:
        with image_lock:
            # Convert ROS Image message to CV2 image
            latest_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def depth_image_callback(data):
    global depth_image
    try:
        with depth_image_lock:
            depth_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    

def timer_callback(event):
    global latest_image
    global depth_image
    if latest_image is not None:
        with image_lock:
            # Copy the latest_image to prevent modification during processing
            image_to_process = latest_image.copy()
        
        # Perform prediction
        results = model.predict(source=image_to_process, conf = 0.5, 
                                save_conf=True, show=True)

        for r in results:
            box = r.boxes
            box.cpu()
            box.numpy() 
            coordinates = box.xywh
            classes = box.cls
        np_coordinates = coordinates.numpy()
        # print(f'All detected coordinates are:\n{np_coordinates}')
        np_classes = classes.numpy()
        # print(f'All detected classes are:\n{np_classes}')

        index = find_number_in_array(np_classes, TARGET)
        # print(f'Index of {TARGET}:\n{index}')
        if index == -1:
            print("Looking for target")
        else:
            [x1, y1, w, h] = np_coordinates[index, :]
            print(x1, y1, w, h) 
            x_center = int(x1+w/2)
            y_center = int(y1+h/2)
            if 0 <= x_center < depth_image.shape[0] and 0 <= y_center < depth_image.shape[1]:
                print(f'Depth image coordinates: {depth_image[x_center, y_center]}')
            else:
                print("Index out of bounds.")



def find_number_in_array(array, number):
    indices = np.where(array == number)[0]
    
    if indices.size > 0:
        return indices[0]
    else:
        return -1

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/kinect/rgb/image_raw', Image, image_callback)
    rospy.Subscriber('/kinect/depth/image_raw', Image, depth_image_callback)
    timer = rospy.Timer(rospy.Duration(1), timer_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
