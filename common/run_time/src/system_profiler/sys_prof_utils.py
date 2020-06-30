import os
from time import *
import subprocess
ctr = 0
#batcmd="dir"



def collect_perf_data(events, sampling_period, perf_log_file):
    event_string =""
    for idx in range(0,len(events)):
        event_string +=events[idx]
        if (idx != len(events) - 1):
            event_string +=","

    bash_cmd = "sudo perf stat -x, -e " + event_string + " -a sleep " + str(sampling_period)  + " >>" + perf_log_file + " 2>&1"
    print(bash_cmd) 
    result = os.system(bash_cmd) 

def reduce_data(events, perf_log_file):
    result_dict = {}
    result_dict_reduced = {} 
    for event in events:
        result_dict[event] = []
    perf_log_file_hndl = open(perf_log_file) 
    for line in perf_log_file_hndl:
        print(line)
        event_name = line.split(",")[2]
        event_value = line.split(",")[0]
        result_dict[event_name].append(int(event_value))

    for key in result_dict.keys():
        result_dict_reduced[key] = sum(result_dict[key])/len(result_dict[key])
    os.system("rm "+ perf_log_file)
    return result_dict_reduced


while(ctr < 10):
    events_to_collect =  ["cache-misses", "instructions"]
    perf_log_file =  "stat.log"
    collect_perf_data(events_to_collect, .1, perf_log_file)
    print(reduce_data(events_to_collect, perf_log_file))
    ctr+=1
