import os
import time
import sys

def action_upon_termination():
    os.system("echo hello > now_here") 
    #--- kill all the ros processes 
    os.system("kill -INT $(ps aux | grep ros | awk '{print $2}')")
    #os.system("rosnode kill --all");
    #os.system("rosnode kill rosout");
def terminate(stat_file):
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
                # if words[0].strip() == "mission_status":
                if "mission_status" in line:
                    stat_f_hndlr.close()
                    return True
            stat_f_hndlr.close()
            return False
                    


def main():
    #sys.argv[1] time based or not
    #sys.argv[2] sleep_time_before checking (optinal)
    SLEEP_TIME_BEFORE_CHECKING = 2 
    assert(len(sys.argv) >= 3)
    stat_file = sys.argv[1]+"data/package_delivery/stats.json"
    # --- populating variables 
    if (len(sys.argv) > 3):
        timeout = float(sys.argv[3])
    else:
        sleep_time_before_checking = SLEEP_TIME_BEFORE_CHECKING
    time_based = sys.argv[2];  
    stat_f_hndlr = open(stat_file, "w")
    stat_f_hndlr.close()
     
    #---- body
    time_spent = 0
    while time_spent < timeout: 
        time.sleep(SLEEP_TIME_BEFORE_CHECKING)
        time_spent += SLEEP_TIME_BEFORE_CHECKING
        if (terminate(stat_file)): 
            action_upon_termination()
            return

    action_upon_termination()
    # with open(stat_file, "a") as stat_f_hndlr:
    #     stat_f_hndlr.write("\nmission_status timeout\n")

if __name__ == "__main__":
    main()

