import rospy, math
from std_msgs.msg import String
from vigir_footstep_planning_msgs.msg import StepPlan, Step, Foot, StepPlanRequestActionResult, StepPlanRequestActionGoal
# from vigir_footstep_planning_msgs.action import StepPlanRequestActionGoal


callback_done = False

def callback(data):
    # data is whole plan
    # rospy.loginfo(data.result.step_plan)
    uh = [data.result.step_plan.steps]
    rospy.loginfo('Length of uh: %f', len(uh))
    rospy.loginfo('in callback fn')
    for thing in uh:
        rospy.loginfo(thing)
    return
    global callback_done
    distance_centroid_travel = 0.0
    current_centroid = (0.0, 0.0)
    planned_steps = data.result.step_plan
    final_eps = data.result.final_eps
    planning_time = data.result.planning_time
    num_of_exp_states = data.result.number_of_states_expanded
    for x in range(0, len(planned_steps)):
        if x == len(planned_steps):
            break
        left_foot = None
        right_foot = None
        if planned_steps[x].foot.foot_index == 1:
            left_foot = planned_steps[x].foot
            right_foot = planned_steps[x+1].foot
        else:
            right_foot = planned_steps[x].foot
            left_foot = planned_steps[x+1].foot
        left_x = left_foot.pose.position.x
        left_y = left_foot.pose.position.y
        right_x = right_foot.pose.position.x
        right_y = right_foot.pose.position.y
        avg_x = (left_x + right_x) / 2
        avg_y = (left_y + right_y) / 2
        body_centroid = (avg_x, avg_y)
        distance = math.sqrt(sum([(a-b) ** 2 for a,b in zip(body_centroid, current_centroid)]))
        # rospy.loginfo('Distance traveled for step %d: %f', x, distance)
        distance_centroid_travel += distance
        current_centroid = body_centroid
    
    # scale down to grid
    distance_centroid_travel *= 0.02
    rospy.loginfo('Body centroid euclidean distance traveled: %f', distance_centroid_travel)
    rospy.loginfo('Number of steps in plan: %i', len(planned_steps))
    rospy.loginfo('Final epsilon value: %f', final_eps)
    rospy.loginfo('Planning time: %f seconds', planning_time)
    rospy.loginfo('Number of states expanded: %i', num_of_exp_states)
    callback_done = True

def plan_parser():
    # callback_done = False
    rospy.init_node('plan_parser', anonymous=True)
    rospy.Subscriber('/vigir/footstep_planning/step_plan_request/result', StepPlanRequestActionResult, callback)
    rospy.loginfo('Plan_parser started up')
    rospy.spin()



def plan_publisher():
    global callback_done
    # rospy.init_node('plan_publisher', anonymous=True)
    rospy.loginfo('plan_publisher started')
    pub = rospy.Publisher('/vigir/footstep_planning/step_plan_request/goal', StepPlanRequestActionGoal, queue_size=10)
    action_goal = StepPlanRequestActionGoal()
    rospy.loginfo('action_goal created')
    set_action_goal(action_goal, 1)
    rospy.loginfo('set_action_goal returned')
    runner = True
    current_config_number = 1
    pub.publish(action_goal)
    rospy.loginfo('action goal:')
    rospy.loginfo(action_goal)
    rate = rospy.Rate(10) # 10hz
    # while(runner):
    #     rate.sleep()
    #     rospy.loginfo('callback_done: %s', callback_done)
    #     if not callback_done:
    #         continue
    #     else:
    #         if current_config_number == 5:
    #             runner = False
    #             break
    #         callback_done = False
    #         current_config_number += 1
    #         action_goal = set_action_goal(action_goal, current_config_number)
    #         pub.publisher(action_goal)


if __name__ == '__main__':
    plan_parser()
    # plan_publisher()