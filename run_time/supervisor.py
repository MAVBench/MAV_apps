import os
from time import *
import sys
import subprocess
import signal


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

def kill_processes(process_list):
    for process in process_list:
        print process 
        pid = process.split()[1] 
        print pid 
        subprocess.Popen("kill -2 "+ pid, shell=True)
        sleep(.1) 
    sleep(3) 

def action_upon_termination():
    ros_node_pid_list = []
    process_ignore_list = ["profile_manager", "rosmaster", "rosout", "grep"]
    
    
    # block till all processes except the ones in the process ignore_list are killed
    ros_process_list_filtered = get_ros_processes(process_ignore_list) 
    kill_processes(ros_process_list_filtered)

    ctr = 0
    ctr_threshold = 100 
    print "==============" 
    while (ctr < ctr_threshold):     
        ros_process_list_filtered = get_ros_processes(process_ignore_list) 
        if(len(ros_process_list_filtered)== 0):#rosmaster doesn't show up
            break
        else:
            print ros_process_list_filtered
        ctr+=1
        sleep(1)

    print("killed all other processes");
    
    # to kill all other processes (including the rosmaster and core and rosout), the
    # easiest way is to kill the roslaunch itself
    roslaunch_process = (subprocess.check_output("ps aux | grep roslaunch |grep opt", shell=True)).splitlines()
    kill_processes(roslaunch_process)
    
    #--- using call cause it's blocking 
    #subprocess.call("kill -INT $(ps aux | grep "+ pid + " | awk '{print $2}')", shell=True
#print roslaunch_process 
    #sye.exit(0)
    #print pid 
    #os.kill(int(pid), 9) 
	     #subprocess.Popen("sudo kill -INT $(ps aux | grep "+ pid + " | awk '{print $2}')", shell=True)
def should_terminate(stat_file):
    try:
        stat_f_hndlr = open(stat_file, "r")
    except IOError:
        handleIOError(stat_file, "source file error")
        exit()
    else:
        with stat_f_hndlr:
            for line in stat_f_hndlr:
                words = line.split(' ')
                #print words[0].strip()
                if words[0].strip() == "kill":
                    stat_f_hndlr.close()
                    return True
            stat_f_hndlr.close()
            return False
                    

def main():
    
    stat_file = "../data/"+"package_delivery"+"/supervisor_mailbox.txt"
    max_run_time = 90000 
    
    polling_freq = 5  #polling 
    if (len(sys.argv) == 4): 
        assert(len(sys.argv) == 4)
        mav_bench_dir = sys.argv[1]
        app =  sys.argv[2]
        max_run_time= sys.argv[3]
        stat_file = mav_bench_dir+"/data/"+app+"/supervisor_mailbox.txt"
        

    stat_f_hndlr = open(stat_file, "w")
    stat_f_hndlr.close()
    initial_time = time() 
    #print stat_file
    #sys.exit(0) 
    #---- body
    while(True): 
        sleep(float(polling_freq))
        time_passed = time() - initial_time
        if (should_terminate(stat_file) or float(time_passed) > float(max_run_time)): 
            action_upon_termination()
            return
        

if __name__ == "__main__":
    main()

