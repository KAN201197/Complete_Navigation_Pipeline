#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    try:
        # Convert the ROS CompressedImage to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Display the image
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/front/image_raw/', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()