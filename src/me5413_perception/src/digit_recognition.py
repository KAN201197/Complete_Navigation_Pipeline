#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import os
import threading

# Initialize global variables
current_dir = os.path.dirname(os.path.abspath(__file__))
weights_path = os.path.join(current_dir, '..', 'yolov8', 'best_detect.pt')
model = YOLO(weights_path)
latest_image = None
bridge = CvBridge()
image_lock = threading.Lock()  # Ensure thread-safe access to the latest_image

def image_callback(data):
    global latest_image
    try:
        with image_lock:
            # Convert ROS Image message to CV2 image
            latest_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def timer_callback(event):
    global latest_image
    if latest_image is not None:
        with image_lock:
            # Copy the latest_image to prevent modification during processing
            image_to_process = latest_image.copy()
        
        # Perform prediction
        results = model.predict(source=image_to_process, imgsz=[512, 640], conf = 0.5, 
                                save_conf=True, show=True)

        for r in results:
            box = r.boxes
            box.numpy() 

            print(box.xyxy)
            # r.show()


        
        # Visualization or further processing
        cv2.imshow("Image Window", image_to_process)
        cv2.waitKey(3)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/front/image_raw/', Image, image_callback)
    timer = rospy.Timer(rospy.Duration(1), timer_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
