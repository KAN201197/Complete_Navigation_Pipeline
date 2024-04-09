import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
import tf

class FeasiblePathFinder:
    def __init__(self):
        # rospy.init_node('make_plan_caller')
        service_name = '/move_base/make_plan'
        # Service client
        self.plan_service = rospy.ServiceProxy(service_name, GetPlan)
        self.tf_listener = tf.TransformListener()
        # Wait for the service to become available
        rospy.wait_for_service(service_name, timeout=None)

    '''
    This method generates
    '''
    def generate_feasible_goal(self, target_pose: PoseStamped):
        generated_plan = self.generate_feasible_plan(target_pose)
        if generated_plan:
            #Print the generated plan
            #Return the second last pose (-2) because the last index (-1) is always the passed (goal) pose, not the viable found.
            rospy.loginfo(generated_plan[-2])
            return generated_plan[-2]
        else:
            return None


    def generate_feasible_plan(self, target_pose: PoseStamped):
        try:
            # Get current robot position in base_link frame
            self.tf_listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
            position_base_link_map, orientation_base_link_map = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            current_pose = PoseStamped()
            current_pose.header.frame_id = "map"
            current_pose.header.stamp = rospy.Time.now()
            current_pose.pose.position.x = position_base_link_map[0]
            current_pose.pose.position.y = position_base_link_map[1]
            current_pose.pose.position.z = 0.0
            current_pose.pose.orientation.x = orientation_base_link_map[0]
            current_pose.pose.orientation.y = orientation_base_link_map[1]
            current_pose.pose.orientation.z = orientation_base_link_map[2]
            current_pose.pose.orientation.w = orientation_base_link_map[3]

            #Update the header stamp to current time.
            target_pose.header.stamp = rospy.Time.now()

            # Make the service call. NOTE: The huge tolerance is intended.
            plan = self.plan_service(start=current_pose, goal=target_pose, tolerance=5.0)
            rospy.loginfo(f'Generated plan: with {len(plan.plan.poses)} poses (waypoints).')
            rospy.logdebug(plan)
            #return plan
            return plan.plan.poses

        except rospy.ServiceException as error_code:
            #This exception can be thrown when the service is called while move_base
            #is busy (e.g. executing a path: error_code: b)
            rospy.logerr(f"Service call failed: {error_code}")
            return None