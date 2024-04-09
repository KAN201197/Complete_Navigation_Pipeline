#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs  # if you need to transform geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class YoloObjectTransformationToMap:
    def __init__(self):
        rospy.init_node('yolo_object_transformation_to_map', anonymous=True)
        
        # Initialize tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))  # Buffer for 10 seconds
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Subscriber("/target/coordinates", PointStamped, self.object_position_callback)

    def object_position_callback(self, msg):
        try:
            # Using the buffer to get the latest available transform
            transform = self.tf_buffer.lookup_transform('map', 'kinect_frame_optical', rospy.Time(0), rospy.Duration(2.0))
            
            # Transform the point from camera frame to map frame using the transform
            point_in_map = tf2_geometry_msgs.do_transform_point(msg, transform).point
            
            rospy.loginfo("Object in map frame: (%.2f, %.2f, %.2f)", point_in_map.x, point_in_map.y, point_in_map.z)

            # Publish the goal
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position = point_in_map
            goal.pose.orientation.w = 1.0  # Assuming a fixed orientation
            self.goal_publisher.publish(goal)
            self.pub_goal_move_base(point_in_map.x, point_in_map.y)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF transformation error: %s", e)

    def pub_goal_move_base(self, x, y):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.x = y
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

if __name__ == '__main__':
    transformer = YoloObjectTransformationToMap()
    rospy.spin()
