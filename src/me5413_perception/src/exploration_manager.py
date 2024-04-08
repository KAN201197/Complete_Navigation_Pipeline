import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

class ExplorationManager:
    def __init__(self):
        rospy.init_node('exploration_manager')
        self.box_goal_name = '/box_3'
        self.frame_id = 'map'
        self.door_entrance_param_name = 'me5413_perception/door_entrance'
        self.door_exit_param_name = 'me5413_perception/door_exit'

    def run(self):
        self.__wait_for_message(self.box_goal_name)
        self.__move_base_client(self.door_entrance_param_name)
        self.__move_base_client(self.door_exit_param_name)

    def __move_base_client(self, param_name):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Waiting for server')
        client.wait_for_server()

        rospy.loginfo('Connected')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = rospy.get_param(param_name+'/position/x')
        goal.target_pose.pose.position.y = rospy.get_param(param_name+'/position/y')
        goal.target_pose.pose.orientation.z = rospy.get_param(param_name+'/orientation/z')
        goal.target_pose.pose.orientation.w = rospy.get_param(param_name+'/orientation/w')

        client.send_goal(goal)
        result = client.wait_for_result()
        if result:
            rospy.loginfo(client.get_goal_status_text())

    def __wait_for_message(self, goal):
        while True:
            rospy.loginfo('Waiting')
            msg = rospy.wait_for_message('rviz_panel/goal_name', String)
            rospy.loginfo(f'Received message: {msg.data}')
            if msg.data == goal:
                break

def main():
    exploration_manager = ExplorationManager()
    exploration_manager.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass