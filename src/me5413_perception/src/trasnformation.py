#!/usr/bin/env python3
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped

#K Matrix from camera info for transforming coordinate in image frame (with depth) into camera frame 
K_matrix = np.array([[554.25469119,   0.        , 320.5       ],
[  0.        , 554.25469119, 256.5       ],
[  0.  ,   0.        ,   1.        ]])



# Matrix of camera frame in base_link frame
matrix_front_camera_optical_frame_in_base_link_frame = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 0.216],
    [1, 0, 0, -0.242],
    [0, 0, 0, 1]
])

class YoloObjectTransformationToMap:
    def __init__(self):
        rospy.init_node('yolo_object_transformation_to_map', anonymous=True)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/target/coordinates", PointStamped, self.object_position_callback)  # Update when yolo output is locked
        self.rate = rospy.Rate(10.0)  # Control loop rate
        rospy.spin()

    def object_position_callback(self, msg):
        # Get the transform from /base_link to /map
        (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        matrix_base_link_frame_in_map_frame = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans),
            tf.transformations.quaternion_matrix(rot)
        )
        #Change points in image frame in to format for homogeneous transformation

        # Pixel coordinates and depth
        x_pixel, y_pixel, depth = msg.point.x, msg.point.y, msg.point.z

        # Convert pixel coordinates to camera frame using depth and intrinsic matrix
        point_camera = np.dot(np.linalg.inv(K_matrix), np.array([x_pixel, y_pixel, 1]) * depth)
        point_camera_homogeneous = np.append(point_camera, 1)

    
        # Direct transformation of points from camera frame to base_link is applied first
        point_in_base_link = np.dot(matrix_front_camera_optical_frame_in_base_link_frame, point_camera_homogeneous)
        
        # Then, transform the result to the map frame
        point_in_map = np.dot(matrix_base_link_frame_in_map_frame, point_in_base_link)

        # Log the transformed point's coordinates in the map frame
        rospy.loginfo("Object in map frame: (%.2f, %.2f, %.2f)", point_in_map[0], point_in_map[1], point_in_map[2])


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '_main_':
    transformer = YoloObjectTransformationToMap()
    transformer.run()