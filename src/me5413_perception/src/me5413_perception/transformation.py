import numpy as np
import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped

# K Matrix from camera info for transforming coordinate
# in image frame (with depth) into camera frame
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
    def object_position_callback(self, msg: PointStamped):
        try:
            # Get the transform from /base_link to /map
            (trans, rot) = self.tf_listener.lookupTransform(
                '/map', '/kinect_frame_optical', rospy.Time(0))
            matrix_camera_frame_in_map_frame = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot))

            # Pixel coordinates and depth
            x_pixel, y_pixel, depth = msg.point.x, msg.point.y, msg.point.z

            # Convert pixel coordinates to camera frame using depth and intrinsic matrix
            point_camera = np.dot(
                np.linalg.inv(K_matrix),
                np.array([x_pixel, y_pixel, 1]) * depth)
            point_camera_homogeneous = np.append(point_camera, 1)

            # Direct transformation of points from camera frame to map is applied first
            point_in_map = np.dot(
                matrix_camera_frame_in_map_frame,
                point_camera_homogeneous)

            # Log the transformed point's coordinates in the map frame
            rospy.loginfo(
                f'Object in map frame: '
                f'{point_in_map[0]},'
                f'{point_in_map[1]},'
                f'{point_in_map[2]}')
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = point_in_map[0]
            goal.pose.position.y = point_in_map[1]
            goal.pose.position.z = point_in_map[2]

            # Assuming a fixed orientation; adjust if needed
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            return goal
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('TF transformation error: %s', e)
            return None
