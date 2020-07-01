import os
from time import *
import sys
import subprocess
import signal
import psutil
def get_ros_processes(ignore_list):
    ros_process_list = (subprocess.check_output("ps aux | grep ros |grep log", shell=True)).splitlines()
    ros_process_list_filtered = ros_process_list[:]
    #filter the list         
    for process in ros_process_list:  
        for ignore_el in ignore_list:
            if ignore_el in process:
                ros_process_list_filtered.remove(process)
                break
    return ros_process_list_filtered


def map_processes(num_of_processors):
    #process_ignore_list = ["profile_manager", "rosmaster", "rosout", "grep"]
    # setting affinity 
    ros_processes =  get_ros_processes([])
    ros_processes_to_consider = ["occupancy_map", "depth_transforms", "motion_planner", "follow_trajectory", "run_time_thread", "optimizer_node"]
    print("---- setting affitinies")
    task_for_navigation = str(hex((0b1<<num_of_processors) -1))
    task_for_sensors = str(hex((0b1<<num_of_processors) + 0))
    task_for_misc = str(hex((0b1<<num_of_processors+1)))
    
    for process in ros_processes:
        assigned = False
        pid = process.split()[1]
        if "img" in process:
            os.system("taskset -p " +  task_for_sensors + " " +  str(pid)) 
        else:
            for process_to_consider in ros_processes_to_consider:
                if process_to_consider in process:
                    pid = process.split()[1]
                    os.system("taskset -p " + task_for_navigation + " "+ str(pid)) 
                    assigned = True
                    break
            if not assigned:
                    os.system("taskset -p " + task_for_misc +" " + str(pid)) 

    
    # affinities
    print("------affitinies are:")
    ros_processes =  get_ros_processes([])
    for process in ros_processes:
        print("======================") 
        print("process name" + str(process.split()[-1].split("/")[-1]) )
        pid = process.split()[1]
        os.system("taskset -p " + str(pid))



def collect_process_data():
    process_ignore_list = ["profile_manager", "rosmaster", "rosout", "grep"]
    ros_processes = get_ros_processes(process_ignore_list, )
    ros_processes_to_consider = ["occupancy_map", "depth_transforms", "motion_planner", "follow_trajectory" ]
    this_process_list = [] 
    for process in ros_processes:
        for process_to_consider in ros_processes_to_consider:
            if process_to_consider in process:
                pid = process.split()[1]
                this_process = psutil.Process(int(pid))
                this_process_list.append(this_process) 
    
    while(True): 
        for process in this_process_list: 
            print("-------------------------------------------") 
            print(process.cpu_percent())
            pid = process.as_dict(attrs=['pid']).values()[0]
            os.system("ps -p " + str(pid) + " -o %cpu,%mem")
        print(psutil.cpu_percent(percpu=True))
        sleep(.2)



if __name__  == "__main__":
    map_processes()
    #collect_process_data()
