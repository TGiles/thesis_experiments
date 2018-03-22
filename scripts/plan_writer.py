import os
import argparse
import csv
import datetime

def create_test_dir():
    experiment_dir = 'experiment_data'
    test_dir = str(datetime.datetime.now())
    try:
        os.makedirs(experiment_dir + '/' + test_dir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
    os.chmod(experiment_dir + '/' + test_dir, 0776)
    return str(experiment_dir + '/' + test_dir)

def create_setup_dir(current_test_dir, setup_id, plugin_set_id):
    try:
        os.makedirs(current_test_dir + '/' + setup_id + plugin_set_id)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
    os.chmod(current_test_dir + '/' + setup_id + plugin_set_id, 0766)
    return str(current_test_dir + '/' + setup_id + plugin_set_id)


# Need CSV for each scenario, similar to thesis tables

def write_header(file_writer, planning_scenario):
    ''' should write
    Planning Scenario #
    '''
    file_writer.writerow([planning_scenario])

def write_run_header(file_writer):
    file_writer.writerow([
        'Configuration ID',
        'Run #',
        'Body Centroid Distance [m]',
        '# Steps',
        'Final Epsilon',
        '# Expanded States',
        'Planning Time [s]',
        'Path Cost'
    ])

def write_run(file_writer, config_id, iteration, body_centroid, num_steps, final_eps, expanded_states, plan_time, path_cost):
    ''' should write
    Configuration ID
    Run #
    Body Centroid Distance [m]
    # Steps
    Final Epsilon
    # Expanded States
    Planning Time [s]
    Path Cost
    '''
    file_writer.writerow([
        config_id,
        iteration,
        body_centroid,
        num_steps,
        final_eps,
        expanded_states,
        plan_time,
        path_cost
    ])

def write_avg_planning_task(file_writer, config_id, avg_body, avg_steps, avg_final_eps, avg_states_expanded, avg_plan_time, avg_path_cost):
    ''' should write
    Configuration ID
    Average Body Centroid Distance [m]
    Average # Steps
    Average Final Epsilon
    Average # Expanded States
    Average Planning Time [s]
    Average Path Cost
    '''
    file_writer.writerow([
        'Configuration ID',
        '',
        'Average Body Centroid Distance [m]',
        'Average # Steps',
        'Average Final Epsilon',
        'Average # Expanded States',
        'Average Planning Time [s]',
        'Average Path Cost'
    ])
    file_writer.writerow([
        config_id,
        '',
        avg_body,
        avg_steps,
        avg_final_eps,
        avg_states_expanded,
        avg_plan_time,
        avg_path_cost
    ])