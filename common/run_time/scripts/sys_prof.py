#!/usr/bin/env python2.7

import roslib
import rospy
import sys
from mavbench_msgs.msg import control
from optimizer.opt import Opt
import time
import numpy as np
import math
from optimizer_settings import *
import psutil
import time
import copy
from system_profiler import mapping
import os
events_to_collect = ["cache-misses", "instructions"]
perf_log_file = os.getenv('base_dir') +"/stat.log"
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



def collect_sys_prof_callback(collect_sys_prof):
    """
    result_dict = reduce_data(events_to_collect, perf_log_file)
    for event in result_dict.keys():
        rospy.set_param(event.replace("-", "_"), float(result_dict[event]))
    # Starts counters
    papi_high.start_counters([
        papi_events.PAPI_FP_OPS,
        papi_events.PAPI_TOT_CYC
    ])
    """
    #result = papi_high.read_counters()  # -> Flops(rtime, ptime, flpops, mflops)
    #    papi_high.epc(
    #        papi_events.PAPI_L1_DCA)

    #print(papi_high.read_counters())#.cpu_percent(percpu=True)[1:4]

    all_cpu_percents = psutil.cpu_percent(percpu=True)[1:4]
    cpu_utilization_last_round = sum(all_cpu_percents[1:4])/len(all_cpu_percents[1:4])
    rospy.set_param("cpu_utilization_for_last_decision", float(cpu_utilization_last_round) )
    print("right here")
    """ 
    memory_utilization_last_round = psutil.virtual_memory().percent
    rospy.set_param("memory_utilization_for_last_decision", float(memory_utilization_last_round) )
    print(memory_utilization_last_round)
    """



if __name__ == '__main__':
    global events_to_collect, perf_log_file 
    rospy.init_node('sys_prof_thread', anonymous=True)
    #rospy.Subscriber("collect_sys_profs", collect_sys_prof, std_msgs.msg.Bool)
    rospy.Subscriber("control_to_crun", control, collect_sys_prof_callback)
    time.sleep(2)
    mapping.map_processes()
    collection_sampling_rate = 1
    rate = rospy.Rate(collection_sampling_rate) 
    
    while not rospy.is_shutdown():
        #collect_perf_data(events_to_collect, collection_sampling_rate, perf_log_file)
        rate.sleep()
