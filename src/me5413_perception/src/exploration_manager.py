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
    '''
    ROS Node to explore a room, find a target and approach it. Target detection is
    carried out using YOLOv8.
    '''
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
            rospy.Duration(1.0), self.digit_recognizer.timer_callback)
        self.client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)

    '''
    Blocking function to run exploration and detection actions
    '''
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

    '''
    Callback function passed to YOLOv8 detector to output a
    point in frame: /kinect_frame_optical
    :param: point_stamped (type: geometry_msgs.msg.PointStamped)
        Calculated point in camera frame
    '''
    def point_stamped_callback(self, point_stamped: PointStamped):
        rospy.loginfo(
            f'Point in frame /kinect_frame_optical:'
            f'{point_stamped.point.x},'
            f'{point_stamped.point.y},'
            f'{point_stamped.point.z}')
        self.point_stamped = point_stamped

    def travel_feedback_cb(self, data):
        pass

    '''
    Callback function to be passed on to move base client for goal
    tracking
    '''
    def recognition_feedback_cb(self, data):
        if self.point_stamped is not None:
            self.client.cancel_goal()

    '''
    Internal function to get goal information form parameter server
    :param: param_name
        Goal identifier
    :return: goal
        Goal message in the form of MoveBaseGoal
    '''
    def __get_goal_coordinates(self, param_name):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = rospy.get_param(
            param_name+'/position/x')
        goal.target_pose.pose.position.y = rospy.get_param(
            param_name+'/position/y')
        goal.target_pose.pose.orientation.z = rospy.get_param(
            param_name+'/orientation/z')
        goal.target_pose.pose.orientation.w = rospy.get_param(
            param_name+'/orientation/w')
        return goal

    '''
    Internal function to connect to move_base action server and send goals
    :param: move_base_goal (type: move_base_msgs.msg.MoveBaseGoal)
        Goal to me sent to move_base server
    :param(optional): feedback_cb
        Callback function to receive goal feedback
    '''
    def __move_base_client(self, move_base_goal, feedback_cb=None):
        self.client.wait_for_server()
        rospy.loginfo('Connected')

        self.client.send_goal(move_base_goal, feedback_cb=feedback_cb)
        result = self.client.wait_for_result()
        if result:
            rospy.loginfo(self.client.get_goal_status_text())

    '''
    One time subscriber to listen on /rviz_panel/goal_name
    :param: goal
        Goal name string to check the current message
    '''
    def __wait_for_message(self, goal):
        while True:
            rospy.loginfo('Waiting')
            msg = rospy.wait_for_message(
                'rviz_panel/goal_name',
                String)
            rospy.loginfo(f'Received message: {msg.data}')
            if msg.data == goal:
                break

    '''
    Internal function to convert from geometry_msgs.msg.PoseStamped
    to move_base_msgs.msg.MoveBaseGoal
    :param: msg (type: geometry_msgs.msg.PoseStamped)
        Message in the form of PoseStamped
    :return: goal
        Message in the form of MoveBaseGoal
    '''
    def __pose_stamped_converter(self, msg: PoseStamped):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = msg.pose.position.x
        goal.target_pose.pose.position.y = msg.pose.position.y
        goal.target_pose.pose.orientation.z = msg.pose.orientation.z
        goal.target_pose.pose.orientation.w = msg.pose.orientation.w
        return goal

    '''
    Debugging function to convert ROS message to JSON
    for printing to console
    :param: data
        ROS message of any type
    :return: Message in the form of string
    '''
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
