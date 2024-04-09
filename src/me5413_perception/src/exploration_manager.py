import json
import yaml
import rospy
import actionlib

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped, PoseStamped

from me5413_perception.digit_recognition import DigitRecognizer
from me5413_perception.transformation import YoloObjectTransformationToMap
from me5413_perception.feasible_path_finder import FeasiblePathFinder

class ExplorationManager:
    def __init__(self):
        rospy.init_node('exploration_manager')
        self.box_goal_name = '/box_3'
        self.frame_id = 'map'
        self.door_entrance_param_name = 'me5413_perception/door_entrance'
        self.door_exit_param_name = 'me5413_perception/door_exit'
        self.point_stamped = None
        self.digit_recognizer = DigitRecognizer(
            self.point_stamped_callback)
        self.yolo_transformation = YoloObjectTransformationToMap()
        self.feasible_path_finder = FeasiblePathFinder()
        self.timer = rospy.Timer(
            rospy.Duration(1.0),
            self.digit_recognizer.timer_callback)
        self.client = actionlib.SimpleActionClient(
            'move_base',
            MoveBaseAction)

    def run(self):
        self.__wait_for_message(self.box_goal_name)
        self.__move_base_client(
            self.__get_goal_coordinates(self.door_entrance_param_name),
            feedback_cb=self.travel_feedback_cb)
        self.__move_base_client(
            self.__get_goal_coordinates(self.door_exit_param_name),
            feedback_cb=self.recognition_feedback_cb)
        box_pose_stamped = self.yolo_transformation.object_position_callback(self.point_stamped)
        if box_pose_stamped is not None:
            feasible_goal_pose_stamped = self.feasible_path_finder.generate_feasible_goal(box_pose_stamped)
            if feasible_goal_pose_stamped is not None:
                self.__move_base_client(
                    self.__pose_stamped_converter(feasible_goal_pose_stamped))

        # if self.point_stamped is not None:
        #     rospy.loginfo(f'Point: {self.point_stamped.point.x}, {self.point_stamped.point.y}, {self.point_stamped.point.z}')

    def point_stamped_callback(self, point_stamped: PointStamped):
        rospy.loginfo(f'Point assigned: {point_stamped.point.x}, {point_stamped.point.y}, {point_stamped.point.z}')
        self.point_stamped = point_stamped

    def travel_feedback_cb(self, data):
        pass

    def recognition_feedback_cb(self, data):
        if self.point_stamped is not None:
            self.client.cancel_goal()

    def __get_goal_coordinates(self, param_name):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = rospy.get_param(param_name+'/position/x')
        goal.target_pose.pose.position.y = rospy.get_param(param_name+'/position/y')
        goal.target_pose.pose.orientation.z = rospy.get_param(param_name+'/orientation/z')
        goal.target_pose.pose.orientation.w = rospy.get_param(param_name+'/orientation/w')
        return goal

    def __move_base_client(self, move_base_goal, feedback_cb=None):
        self.client.wait_for_server()
        rospy.loginfo('Connected')
        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = self.frame_id
        # goal.target_pose.header.stamp = rospy.Time.now()
        # goal.target_pose.pose.position.x = rospy.get_param(param_name+'/position/x')
        # goal.target_pose.pose.position.y = rospy.get_param(param_name+'/position/y')
        # goal.target_pose.pose.orientation.z = rospy.get_param(param_name+'/orientation/z')
        # goal.target_pose.pose.orientation.w = rospy.get_param(param_name+'/orientation/w')

        self.client.send_goal(move_base_goal, feedback_cb=feedback_cb)
        result = self.client.wait_for_result()
        if result:
            rospy.loginfo(self.client.get_goal_status_text())

    def __wait_for_message(self, goal):
        while True:
            rospy.loginfo('Waiting')
            msg = rospy.wait_for_message('rviz_panel/goal_name', String)
            rospy.loginfo(f'Received message: {msg.data}')
            if msg.data == goal:
                break

    def __pose_stamped_converter(self, msg: PoseStamped):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = msg.pose.position.x
        goal.target_pose.pose.position.y = msg.pose.position.y
        goal.target_pose.pose.orientation.z = msg.pose.orientation.z
        goal.target_pose.pose.orientation.w = msg.pose.orientation.w
        return goal

    def __msg_to_json(self, data):
        msg_yaml = yaml.load(str(data))
        return json.dumps(msg_yaml, indent=4)

def main():
    exploration_manager = ExplorationManager()
    exploration_manager.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass