import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped
from geometry_msgs.msg import PoseStamped

# K Matrix from camera info for transforming coordinate in image frame (with depth) into camera frame
K_matrix = np.array([[554.5940455214144, 0.0, 320.5],
                     [0.0, 554.5940455214144, 240.5],
                     [0.0, 0.0, 1.0]])


class YoloObjectTransformationToMap:
    '''
    Convert a point in camera frame to a pose in map frame. Uses
    TransformListener to extract transformations
    '''
    def __init__(self):
        self.tf_listener = tf.TransformListener()

    '''
    Convert point from camera frame to map frame
    :param: msg (type: geometry_msgs.msg.PointStamped)
        Point in camera frame
    :returns: Point in map frame of type geometry_msgs.msg.PoseStamped
        and None if exception is thrown
    '''
    def object_position_callback(self, msg):
        try:
            # Get the transform from /base_link to /map
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/kinect_frame_optical', rospy.Time(0))
            matrix_camera_frame_in_base_link_frame = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot)
            )

            (trans_1, rot_1) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            matrix_base_link_frame_in_map_frame = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans_1),
                tf.transformations.quaternion_matrix(rot_1)
            )
            # Pixel coordinates and depth
            x_pixel, y_pixel, depth = msg.point.x, msg.point.y, msg.point.z

            # Convert pixel coordinates to camera frame using depth and intrinsic matrix
            point_camera = np.dot(np.linalg.inv(K_matrix), np.array([x_pixel, y_pixel, 1]) * depth)
            point_camera_homogeneous = np.append(point_camera, 1)

            # Direct transformation of points from camera frame to map is applied first
            point_in_base_link = np.dot(matrix_camera_frame_in_base_link_frame, point_camera_homogeneous)

            # Check the y coordinate and adjust x and y accordingly
            if point_in_base_link[1] < 0:
                point_in_base_link[0] = point_in_base_link[0] - 0.4  # x = x - 1
                point_in_base_link[1] = point_in_base_link[1] + 0.4  # y = y + 1
            elif point_in_base_link[1] > 0:
                point_in_base_link[0] = point_in_base_link[0] - 0.4  # x = x - 1
                point_in_base_link[1] = point_in_base_link[1] - 0.4  # y = y - 1
            else:  # y == 0
                point_in_base_link[0] = point_in_base_link[0] - 0.56  # x = x - 1
            # y remains unchanged
            # Log the transformed point's coordinates in the map frame

            point_in_map = np.dot(matrix_base_link_frame_in_map_frame, point_in_base_link)

            rospy.loginfo('Object in map frame: (%.2f, %.2f, %.2f)', point_in_map[0], point_in_map[1], point_in_map[2])
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = point_in_map[0]
            goal.pose.position.y = point_in_map[1]
            goal.pose.position.z = point_in_map[2]

            # Assuming a fixed orientation; adjust if needed
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = -0.7071
            goal.pose.orientation.w = 0.7071
            return goal

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('TF transformation error: %s', e)
