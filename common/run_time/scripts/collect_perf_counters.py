#!/usr/bin/env python2.7

import roslib
import rospy
import sys
from mavbench_msgs.msg import control
from std_msgs.msg import Time
from optimizer.opt import Opt
import time
import numpy as np
import math
from optimizer_settings import *
import psutil
import time
import copy
from profiler import *
import json
#from system_profiler import mapping

import os
events_to_collect = ["cache-misses", "instructions"]
perf_log_file = os.getenv('base_dir') +"/stat.log"
stat_file_name = os.getenv('base_dir') + "/src/MAV_apps/data/package_delivery/stats_mu.json"


from pypapi import papi_high
from pypapi import events as papi_events
from profiler import *

def collect_perf_data(events, sampling_period, perf_log_file):
    event_string = ""
    for idx in range(0, len(events)):
        event_string += events[idx]
        if (idx != len(events) - 1):
            event_string += ","

    bash_cmd = "echo rur3dd1 | sudo -S perf stat -x, -e " + event_string + " -a sleep " + str(
        1/sampling_period) + " >>" + perf_log_file + " 2>&1"
    result = os.system(bash_cmd)

"""
def reduce_data(events, perf_log_file):
    result_dict = {}
    result_dict_reduced = {}
    for event in events:
        result_dict[event] = []
    perf_log_file_hndl = open(perf_log_file)
    for line in perf_log_file_hndl:
        if line.split() [0] == "[sudo]":  # fix the sudo issue
            line_ = " ".join(line.split()[4:])
        else:
            line_ = line
        if (len(line_.split(","))<2):
            continue
        event_name = line_.split(",")[2]
        event_value = line_.split(",")[0]
        result_dict[event_name].append(float(event_value))
    for key in result_dict.keys():
        if len(result_dict[key])>0:
            result_dict_reduced[key] = sum(result_dict[key]) / len(result_dict[key])
    os.system("rm " + perf_log_file)
    return result_dict_reduced
"""



def  reduce_data(data_dict, events, reduction_mode="sum"):
    events_ = [event__[0] for event__ in events]
    event_reduced = {}
    for event_idx in range(0, len(events_)):
        event_vals = [cpu_data[0][event_idx] for cpu_data in data_dict.values()]
        event_reduced[events_[event_idx]] = sum(event_vals)
    return event_reduced


end_decision_period = False
end_time_period = False
first_cmd_time_received = False

def collect_sys_prof_callback(collect_sys_prof):
    global end_decision_period
    end_decision_period = True


def first_cmd_time_received_cb(first_cmd_time_):
    global first_cmd_time
    global first_cmd_time_received 
    first_cmd_time = first_cmd_time_.data
    first_cmd_time_received = True


if __name__ == '__main__':
    global stop_collecting 
    global end_time_period
    global end_decision_period
    global first_cmd_time_received
    global first_cmd_time

    rospy.init_node('collect_perf_counters', anonymous=True)
    first_cmd_time = rospy.Time.now()
    print("---first_cmd_time")
    print(first_cmd_time)
    rospy.Subscriber("control_to_crun", control, collect_sys_prof_callback)
    rospy.Subscriber("first_cmd_time_received_topic", Time, first_cmd_time_received_cb)

    #mapping.map_processes()
    time_sampling_rate = rospy.Rate(rospy.get_param("time_sampling_rate"))
    decision_sampling_rate = rospy.Rate(rospy.get_param("decision_sampling_rate"))
    hw_sampling_method = rospy.get_param("hw_sampling_method")


    # --- defining the events
    number_of_cpus = rospy.get_param("number_of_processors")
    events = [['SYSTEMWIDE:PERF_COUNT_HW_INSTRUCTIONS' ], ['SYSTEMWIDE:LLC_REFERENCES'], ['SYSTEMWIDE:CACHE-REFERENCES']]
    perf_list = []
    perf = Profiler(events_groups=events)
    perf.start_counters(0, 0)
    data_dict = {}
    event_dict = {}
    event_dict['time'] = []  # keeping record of sampling time
    # initialize the counters 
    for cpu_id in range(0, number_of_cpus):
        perf_list.append(Profiler(events_groups=events))
        perf_list[cpu_id].start_counters(0, cpu_id)
    
    # iterate and collect
    while not rospy.is_shutdown():
        # ------- sample if time is reached
        if ((hw_sampling_method == "decision_based" and end_decision_period) or (hw_sampling_method == "time_based" and end_time_period)) and first_cmd_time_received: 
            # empty out previous data
            for cpu_id in range(0, number_of_cpus):
                data_dict[cpu_id] = []

            # collect the data 
            for cpu_id in range(0, number_of_cpus):
                blah = perf_list[cpu_id].read_events()
                data_dict[cpu_id].append(perf_list[cpu_id].read_events())
            
            # reduce it statistically
            event_reduced = reduce_data(data_dict, events)
           
            # set params
            for event_name in event_reduced.keys():
                event_name_cleaned_up = event_name.split(":")[1] # get rid of SYSTEMWIDE 
                event_name_cleaned_up_ = event_name_cleaned_up.replace("-","_")
                # sending over param channel. For decision based
                rospy.set_param(event_name_cleaned_up_, float(event_reduced[event_name]))
                # record it locally  # for time_based
                if event_name_cleaned_up_ in event_dict.keys():
                    event_dict[event_name_cleaned_up_].append(float(event_reduced[event_name]))
                else:
                    event_dict[event_name_cleaned_up_] = [float(event_reduced[event_name])]

            event_dict['time'].append((rospy.Time.now() - first_cmd_time).to_sec())

            # reset the events
            for cpu_id in range(0, number_of_cpus):
                perf_list[cpu_id].reset_events()

        # sleep
        if hw_sampling_method == "decision_based":
            end_decision_period = False
            decision_sampling_rate.sleep()
        elif hw_sampling_method == "time_based":
            time_sampling_rate.sleep()
            end_time_period = True 
        else:
            print("hardware mode " + hw_sampling_method + " is not defined")

    with open(stat_file_name, 'w') as file:
        file.write(json.dumps(event_dict))
