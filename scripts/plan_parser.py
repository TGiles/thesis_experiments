import rospy, math
from std_msgs.msg import String
from vigir_footstep_planning_msgs.msg import StepPlan, Step, Foot
from vigir_footstep_planning_msgs.action import StepPlanRequestActionGoal


callback_done = False
def callback(data):
    # data is whole plan
    distance_centroid_travel = 0.0
    current_centroid = (0.0, 0.0)
    planned_steps = data.steps
    for x in range(0, len(planned_steps), 2):
        if x == len(planned_steps) or x == len(planned_steps) - 1:
            break
        left_foot = None
        right_foot = None
        if planned_steps[x].foot.foot_index == 1:
            left_foot = planned_steps[x].foot
            right_foot = planned_steps[x+1].foot
        else:
            right_foot = planned_steps[x].foot
            left_foot = planned_steps[x].foot
        left_x = left_foot.pose.position.x
        left_y = left_foot.pose.position.y
        right_x = right_foot.pose.position.x
        right_y = right_foot.pose.position.y
        avg_x = (left_x + right_x) / 2
        avg_y = (left_y + right_y) / 2
        body_centroid = (avg_x, avg_y)
        distance = math.sqrt(sum([(a-b) ** 2 for a,b in zip(body_centroid, current_centroid)]))
        rospy.loginfo('Distance traveled for step %d: %f', x, distance)
        distance_centroid_travel += distance
        current_centroid = body_centroid
    
    # scale down to grid
    distance_centroid_travel *= 0.02
    rospy.loginfo('Body centroid euclidean distance traveled: %f', distance_centroid_travel)
    callback_done = True
    # for step in planned_steps:
    #     current_foot = step.foot
    #     current_pose = current_foot.pose
    #     current_x = current_pose.Point.x
    #     current_y = current_pose.Point.y
    #     curr_centroid = 
    # for thing in data:
    #     rospy.loginfo('What is this thing? %s', thing)

def plan_parser():
    rospy.init_node('plan_parser', anonymous=True)
    rospy.Subscriber('/vigir/footstep_planning/step_plan', StepPlan, callback)
    rospy.spin()

def set_action_defaults(action_goal):
    action_goal.header.frame_id = ""
    action_goal.goal.plan_request.header.frame_id = "world"
    action_goal.goal.plan_request.planning_mode = 0
    action_goal.goal.plan_request.start_step_index = 0
    action_goal.goal.plan_request.start_foot_selection = 0
    action_goal.goal.plan_request.max_planning_time = 0.0
    action_goal.goal.plan_request.max_number_steps = 0.0
    action_goal.goal.plan_request.max_path_length_ratio = 0.0
    action_goal.goal.plan_request.start.header.frame_id = "/world"
    action_goal.goal.plan_request.goal.header.frame_id = "/world"
    action_goal.goal.plan_request.start.left.foot_index = 0
    action_goal.goal.plan_request.start.right.foot_index = 1
    action_goal.goal.plan_request.goal.left.foot_index = 0
    action_goal.goal.plan_request.goal.right.foot_index = 1

    return action_goal

def set_action_goal(action_goal, config_num):
    action_goal = set_action_defaults(action_goal)
    if config_num == 1:
        action_goal.goal.plan_request.start.left.pose.position.x = 0.5
        action_goal.goal.plan_request.start.left.pose.position.y = 1.45
        action_goal.goal.plan_request.start.left.pose.orientation.z = 1.0

        action_goal.goal.plan_request.start.right.pose.position.x = 0.5
        action_goal.goal.plan_request.start.right.pose.position.y = 1.45
        action_goal.goal.plan_request.start.right.pose.orientation.z = 1.0

        action_goal.goal.plan_request.goal.left.pose.position.x = 1.5
        action_goal.goal.plan_request.goal.left.pose.position.y = 1.45
        action_goal.goal.plan_request.goal.left.pose.orientation.z = 1.0

        action_goal.goal.plan_request.goal.right.pose.position.x = 0.5
        action_goal.goal.plan_request.goal.right.pose.position.y = 1.45
        action_goal.goal.plan_request.goal.right.pose.orientation.z = 1.0

    if config_num == 2:
        action_goal.goal.plan_request.start.left.pose.position.x = 2.58136
        action_goal.goal.plan_request.start.left.pose.position.y = 0.665124
        action_goal.goal.plan_request.start.left.pose.orientation.z = 0.71847442
        action_goal.goal.plan_request.start.left.pose.orientation.w = 0.69555338

        action_goal.goal.plan_request.start.right.pose.position.x = 2.681783
        action_goal.goal.plan_request.start.right.pose.position.y = 0.668365
        action_goal.goal.plan_request.start.right.pose.orientation.z = 0.71847442
        action_goal.goal.plan_request.start.right.pose.orientation.w = 0.69555338

        action_goal.goal.plan_request.goal.left.pose.position.x = 2.526384
        action_goal.goal.plan_request.goal.left.pose.position.y = 2.100897
        action_goal.goal.plan_request.goal.left.pose.orientation.z = 0.70958355
        action_goal.goal.plan_request.goal.left.pose.orientation.w = 0.70462131

        action_goal.goal.plan_request.goal.right.pose.position.x = 2.626382
        action_goal.goal.plan_request.goal.right.pose.position.y = 2.101598
        action_goal.goal.plan_request.goal.right.pose.orientation.z = 0.70958355
        action_goal.goal.plan_request.goal.right.pose.orientation.w = 0.70462131

    if config_num == 3:
        action_goal.goal.plan_request.start.left.pose.position.x = 2.476826
        action_goal.goal.plan_request.start.left.pose.position.y = 2.775733
        action_goal.goal.plan_request.start.left.pose.orientation.z = 0.43851306
        action_goal.goal.plan_request.start.left.pose.orientation.w = 0.89872482

        action_goal.goal.plan_request.start.right.pose.position.x = 2.555646
        action_goal.goal.plan_request.start.right.pose.position.y = 2.714191
        action_goal.goal.plan_request.start.right.pose.orientation.z = 0.43851306
        action_goal.goal.plan_request.start.right.pose.orientation.w = 0.89872482

        action_goal.goal.plan_request.goal.left.pose.position.x = 3.323491
        action_goal.goal.plan_request.goal.left.pose.position.y = 3.768600
        action_goal.goal.plan_request.goal.left.pose.orientation.z = 0.48015289
        action_goal.goal.plan_request.goal.left.pose.orientation.w = 0.87718482

        action_goal.goal.plan_request.goal.right.pose.position.x = 3.407727
        action_goal.goal.plan_request.goal.right.pose.position.y = 3.714710
        action_goal.goal.plan_request.goal.right.pose.orientation.z = 0.48015289
        action_goal.goal.plan_request.goal.right.pose.orientation.w = 0.87718482

    if config_num == 4:
        action_goal.goal.plan_request.start.left.pose.position.x = 0.092039
        action_goal.goal.plan_request.start.left.pose.position.y = 0.473287
        action_goal.goal.plan_request.start.left.pose.orientation.z = -0.001306
        action_goal.goal.plan_request.start.left.pose.orientation.w = 0.99999915

        action_goal.goal.plan_request.start.right.pose.position.x = 0.091778
        action_goal.goal.plan_request.start.right.pose.position.y = 0.373288
        action_goal.goal.plan_request.start.right.pose.orientation.z = -0.001306
        action_goal.goal.plan_request.start.right.pose.orientation.w = 0.99999915

        action_goal.goal.plan_request.goal.left.pose.position.x = 0.505778
        action_goal.goal.plan_request.goal.left.pose.position.y = 1.277839
        action_goal.goal.plan_request.goal.left.pose.orientation.z = 0.99999338
        action_goal.goal.plan_request.goal.left.pose.orientation.w = 0.00363932

        action_goal.goal.plan_request.goal.right.pose.position.x = 0.506505
        action_goal.goal.plan_request.goal.right.pose.position.y = 1.377836
        action_goal.goal.plan_request.goal.right.pose.orientation.z = 0.99999338
        action_goal.goal.plan_request.goal.right.pose.orientation.w = 0.00363932
    return action_goal

def plan_publisher():
    rospy.init_node('plan_publisher', anonymous=True)
    pub = rospy.Publisher('/vigir/footstep_planning/step_plan_request/goal', StepPlanRequestActionGoal)
    action_goal = StepPlanRequestActionGoal()
    set_action_goal(action_goal, 0)
    runner = True
    current_config_number = 0
    pub.publish(action_goal)
    rate = rospy.Rate(10) # 10hz
    while(runner):
        rate.sleep()
        if not callback_done:
            continue
        else:
            if current_config_number + 1 == 5:
                runner = False
                break
            callback_done = False
            current_config_number += 1
            action_goal = set_action_goal(action_goal, current_config_number)
            pub.publisher(action_goal)


if __name__ == '__main__':
    plan_publisher()
    plan_parser()