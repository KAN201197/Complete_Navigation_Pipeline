#!/usr/bin/env python3
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

# K Matrix from camera info for transforming coordinate in image frame (with depth) into camera frame 
K_matrix = np.array([[554.5940455214144, 0.0, 320.5],
                     [0.0, 554.5940455214144, 240.5],
                     [0.0, 0.0, 1.0]])

# Matrix of camera frame in base_link frame
matrix_front_camera_optical_frame_in_base_link_frame = np.array([
    [0, 0, 1, 0],
    [-1, 0, 0, 0.210],
    [0, -1, 0, -0.150],
    [0, 0, 0, 1]
])

class YoloObjectTransformationToMap:
    def __init__(self):
        rospy.init_node('yolo_object_transformation_to_map', anonymous=True)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/target/coordinates", PointStamped, self.object_position_callback)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    def object_position_callback(self, msg):
        try:
            # Get the transform from /base_link to /map
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            matrix_base_link_frame_in_map_frame = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot)
            )
        

            # Pixel coordinates and depth
            x_pixel, y_pixel, depth = msg.point.x, msg.point.y, msg.point.z

            # Convert pixel coordinates to camera frame using depth and intrinsic matrix
            point_camera = np.dot(np.linalg.inv(K_matrix), np.array([x_pixel, y_pixel, 1]) * depth)
            point_camera_homogeneous = np.append(point_camera, 1) #Make homogeneous

            # Direct transformation of points from camera frame to base_link is applied first
            point_in_base_link = np.dot(matrix_front_camera_optical_frame_in_base_link_frame, point_camera_homogeneous)
            
            # Then, transform the result to the map frame
            point_in_map = np.dot(matrix_base_link_frame_in_map_frame, point_in_base_link)

            # Log the transformed point's coordinates in the map frame
            rospy.loginfo("Object in map frame: (%.2f, %.2f, %.2f)", point_in_map[0], point_in_map[1], point_in_map[2])
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = point_in_map[0]
            goal.pose.position.y = point_in_map[1]
            goal.pose.position.z = point_in_map[2]
        
            # Assuming a fixed orientation; adjust if needed
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0

            # Publish the goal
            self.goal_publisher.publish(goal)




        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF transformation error: %s", e)

if __name__ == '__main__':
    transformer = YoloObjectTransformationToMap()
    rospy.spin()  # Keeping the node alive for callbacks

