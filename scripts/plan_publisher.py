import rospy, math
from std_msgs.msg import String
from vigir_footstep_planning_msgs.msg import StepPlan, Step, Foot, StepPlanRequestActionResult, StepPlanRequestActionGoal


def set_action_defaults(action_goal):
    action_goal.goal_id.id = str(rospy.Time.now())
    action_goal.header.frame_id = ''
    action_goal.goal.plan_request.header.frame_id = '/world'
    action_goal.goal.plan_request.planning_mode = 0
    action_goal.goal.plan_request.start_step_index = 0
    action_goal.goal.plan_request.start_foot_selection = 0
    action_goal.goal.plan_request.max_planning_time = 0.0
    action_goal.goal.plan_request.max_number_steps = 0.0
    action_goal.goal.plan_request.max_path_length_ratio = 0.0
    action_goal.goal.plan_request.start.header.frame_id = '/world'
    action_goal.goal.plan_request.goal.header.frame_id = '/world'
    action_goal.goal.plan_request.start.left.foot_index = 0
    action_goal.goal.plan_request.start.right.foot_index = 1
    action_goal.goal.plan_request.goal.left.foot_index = 0
    action_goal.goal.plan_request.goal.right.foot_index = 1
    action_goal.goal.plan_request.start.left.header.frame_id = '/world'
    action_goal.goal.plan_request.start.right.header.frame_id = '/world'
    action_goal.goal.plan_request.goal.left.header.frame_id = '/world'
    action_goal.goal.plan_request.goal.right.header.frame_id = '/world'
    action_goal.goal.plan_request.parameter_set_name.data = "Nao"

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
    rospy.loginfo('action goal set')
    return action_goal

def plan_publisher():
    rospy.init_node('plan_publisher', anonymous=True)
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
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo('publisher is still alive')
        r.sleep()


if __name__ == '__main__':
    plan_publisher()