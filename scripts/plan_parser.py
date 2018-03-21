import rospy, math
from std_msgs.msg import String
from vigir_footstep_planning_msgs.msg import StepPlan, Step, Foot, StepPlanRequestActionResult, StepPlanRequestActionGoal
from plan_publisher import *
from plan_writer import *
# from vigir_footstep_planning_msgs.action import StepPlanRequestActionGoal

def set_callback_done(param):
    global callback_done
    callback_done = param

def get_callback_done():
    global callback_done
    return callback_done

def get_file_writer():
    global file_writer
    return file_writer

def get_iteration():
    global iteration
    return iteration


def calc_avgs(result_list):
    length = len(result_list)
    no_path = False
    avg_body_centroid = 0.0
    avg_steps = 0
    avg_final_eps = 0.0
    avg_exp_states = 0
    avg_plan_time = 0.0
    avg_path_cost = 0.0
    if length == 0:
        return [avg_body_centroid, avg_steps, avg_final_eps, avg_exp_states, avg_plan_time, avg_path_cost]
    for thing in result_list:
        no_path = False
        if thing[0] == 0.0:
            no_path = True
        else:
            avg_body_centroid += thing[0]
        if thing[1] == 0:
            no_path = True
        else:
            avg_steps += thing[1]
        if thing[2] > 1000:
            no_path = True
        else:
            avg_final_eps += thing[2]
        if no_path:
            length = length - 1
        else:
            avg_exp_states += thing[3]
            avg_plan_time += thing[4]
            avg_path_cost += thing[5]
    avg_body_centroid = avg_body_centroid / length
    avg_steps = avg_steps / length
    avg_final_eps = avg_final_eps / length
    avg_exp_states = avg_exp_states / length
    avg_plan_time = avg_plan_time / length
    avg_path_cost = avg_path_cost / length
    return [avg_body_centroid, avg_steps, avg_final_eps, avg_exp_states, avg_plan_time, avg_path_cost]

def callback(data):
    global file_writer
    global callback_done
    global run_result
    global setup_id
    global plugin_set_id
    global iteration
    file_writer = get_file_writer()
    distance_centroid_travel = 0.0
    current_centroid = (0.0, 0.0)
    planned_steps = data.result.step_plan.steps
    final_eps = data.result.final_eps
    planning_time = data.result.planning_time
    num_of_exp_states = data.result.number_of_states_expanded
    total_path_cost = data.result.total_path_cost
    for x in range(0, len(planned_steps)):
        if x == len(planned_steps) - 1:
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
    # distance_centroid_travel *= 0.02
    local_iteration = get_iteration()
    rospy.loginfo('--------------------------------------------')
    rospy.loginfo('Body centroid euclidean distance: %f [m]', distance_centroid_travel)
    rospy.loginfo('Number of steps in plan: %i', len(planned_steps))
    rospy.loginfo('Final epsilon value: %f', final_eps)
    rospy.loginfo('Number of states expanded: %i', num_of_exp_states)
    rospy.loginfo('Planning time: %f [seconds]', planning_time)
    rospy.loginfo('Total path cost: %f', total_path_cost)
    rospy.loginfo('Current iteration: %i', local_iteration)
    rospy.loginfo('--------------------------------------------')
    # file_writer, config_id, iteration, body_centroid, num_steps, final_eps, expanded_states, plan_time, path_cost
    config_string = str(setup_id + ' ' + plugin_set_id)
    write_run(file_writer, config_string, get_iteration(), distance_centroid_travel, len(planned_steps), final_eps, num_of_exp_states, planning_time, total_path_cost)
    iteration += 1
    callback_done = True
    run_result.append([distance_centroid_travel, len(planned_steps), final_eps, num_of_exp_states, planning_time, final_eps, total_path_cost])

def plan_parser(setup_id, plugin_set_id):
    # Example of setup id and plugin set
    # ViGIR A
    # means Faulty state generator, euclidean step cost, euclidean heuristic
    global callback_done
    global run_result
    global file_writer
    global iteration
    # global setup_id
    # global plugin_set_id
    callback_done = True
    current_test = create_test_dir()
    current_setup = create_setup_dir(current_test, setup_id, plugin_set_id)
    run_result = []
    rospy.init_node('plan_parser', anonymous=True)
    rospy.Subscriber('/vigir/footstep_planning/step_plan_request/result', StepPlanRequestActionResult, callback)
    rospy.loginfo('Plan_parser started up')
    pub = rospy.Publisher('/vigir/footstep_planning/step_plan_request/goal', StepPlanRequestActionGoal, queue_size=10)
    rospy.loginfo('plan_publisher started')
    rate = rospy.Rate(0.25)
    current_config = 1
    iteration = 0
    file_obj = open(current_setup + '/Scenario' + str(current_config) + '.csv', 'w+')
    file_writer = csv.writer(file_obj, delimiter=',')
    write_header(file_writer, 'Scenario '+ str(current_config))
    write_run_header(file_writer)
    # plan_publisher(pub, current_config)
    # rospy.spin()
    while not rospy.is_shutdown():
        rate.sleep()
        callback_done = get_callback_done()
        if callback_done:
            if current_config == 5:
                rospy.signal_shutdown('End of testing')
                return
            # rospy.signal_shutdown('End of testing')
            plan_publisher(pub, current_config, setup_id, plugin_set_id, iteration)
            set_callback_done(False)
            if iteration == 5:
                avg_results = calc_avgs(run_result)
                write_avg_planning_task(file_writer, str(setup_id + plugin_set_id), 
                avg_results[0], avg_results[1], avg_results[2], avg_results[3], avg_results[4], avg_results[5])

                run_result = []
                current_config += 1
                file_obj = open(current_setup + '/Scenario' + str(current_config)+ '.csv', 'w+')
                file_writer = csv.writer(file_obj, delimiter=',')
                write_header(file_writer, 'Scenario' + str(current_config))
                write_run_header(file_writer)
                iteration = 0
            # else:
            #     iteration += 1
        else:
            continue


def plan_publisher(pub, config_num, setup_id=None, plugin_set_id=None, iteration=None):
    # rospy.init_node('plan_publisher', anonymous=True)
    global callback_done
    action_goal = StepPlanRequestActionGoal()
    rospy.loginfo('action_goal created')
    # set_action_goal(action_goal, config_num, setup_id, plugin_set_id, iteration)
    set_action_goal_straights(action_goal, config_num, setup_id, plugin_set_id, iteration)
    rospy.loginfo('set_action_goal returned: config number %i', config_num)
    rospy.loginfo('Current iteration: %i', iteration)
    pub.publish(action_goal)
    set_callback_done(False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--setup_id', type=str, default=None)
    parser.add_argument('--plugin_set_id', type=str, default=None)
    args = parser.parse_args()

    global callback_done
    global setup_id
    global plugin_set_id
    setup_id = args.setup_id
    plugin_set_id = args.plugin_set_id
    callback_done = True
    plan_parser(args.setup_id, args.plugin_set_id)
    # plan_publisher()