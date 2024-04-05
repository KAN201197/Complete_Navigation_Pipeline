#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    try:
        # Convert the ROS CompressedImage to OpenCV format
        cv_image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    # Display the image
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)

def listener():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/front/image_raw/compressedDepth", CompressedImage, callback)
    
    # Initialize window for display
    cv2.namedWindow("Image Window", 1)
    
    # Spin until node is shutdown
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()
