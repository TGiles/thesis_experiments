import os
import argparse
import csv
import datetime

def create_test_dir():
    experiment_dir = '../experiment_data'
    test_dir = str(datetime.datetime.now())
    try:
        os.makedirs(experiment_dir + '/' + test_dir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
    os.chdir(experiment_dir + '/' + test_dir, 0776)

# Need CSV for each scenario, similar to thesis tables

def write_header(file_obj, planning_scenario):
    ''' should write
    Planning Scenario #

    '''

def write_run(file_obj):
    ''' should write
    Configuration ID
    Body Centroid Distance [m]
    # Steps
    # Expanded States
    Planning Time [s]
    Path Cost
    '''

def write_avg_planning_task(file_obj):
    ''' should write
    Configuration ID


    '''