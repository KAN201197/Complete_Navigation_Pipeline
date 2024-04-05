#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
weights_path = os.path.join(current_dir, '..', 'yolov8', 'best.pt')
model = YOLO(weights_path)

def callback(data):
    bridge = CvBridge()
    try:
        # Convert the ROS CompressedImage to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # w, h = cv_image.shape[:2]               #512 640
    # print(f'size {w}  {h}')
    # Display the image
    results = model(source = cv_image, imgsz = [512, 640])
    for r in results:
        r.boxes
        r.show()
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)



# def image_detection(image):
#     result = model(image)
#     boxes = result.boxes  # Boxes object for bounding box outputs
#     masks = result.masks  # Masks object for segmentation masks outputs
#     keypoints = result.keypoints  # Keypoints object for pose outputs
#     probs = result.probs  # Probs object for classification outputs
#     result.show() 
    
    pass


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/front/image_raw/', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()