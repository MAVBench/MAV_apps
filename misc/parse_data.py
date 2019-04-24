import sys
import numpy
import math
# gets the json file and get avg of the the variables of intersts

assert(len(sys.argv) == 2), "give me the name of the file that the data resides in"
file_to_parse = sys.argv[1]

# vars you care about collecting data for
var_vals= {"image_to_follow":[], "octomap_integration":[],\
        "motion_planning_kernel":[]}
# vars statistics
var_means = {}

"""
try:
    with  open(file_to_parse, "r")  as f:
        sensor_to_actuator_latency_list = []
        octomap_latency_list = []
        motion_planning_latency_list = []
        
        for line in f:
            if "image_to_follow_time" in line:
                sensor_to_actuator_latency_list.append(float(line.split()[1].split(',')[0]))
            if "octomap_integration" in line:
                octomap_latency_list.append(float(line.split()[1].split(',')[0]))
            if "motion_planning_kernel" in line:
                motion_planning_latency_list.append(float(line.split()[1].split(',')[0]))
        print str(file_to_parse + ":")
        print("sensory_to_actuator_latency:" + str(numpy.mean(filter(lambda x: not(math.isnan(x)),sensor_to_actuator_latency_list ))))
        print("octomap_integration:" + str(numpy.mean(filter(lambda x: not(math.isnan(x)), octomap_latency_list))))
        print("motion_planning_kernel:" + str(numpy.mean(filter(lambda x: not(math.isnan(x)), motion_planning_latency_list))))
except Exception as e:
    print(e) 
"""


try:
    # read the file 
    with  open(file_to_parse, "r")  as f:
        for line in f:
            for key, val in var_vals.items(): 
                if key in line:
                    val.append(float(line.split()[1].split(',')[0]))
    
    # collect statistics
    for key, val in var_vals.items():
        var_means[key] = numpy.mean(filter(lambda x: not(math.isnan(x)), val))
except Exception as e:
    print(e) 

print(var_means)

