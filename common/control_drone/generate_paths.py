import time
import sys
def pre_mission(cmds):
    time.sleep(3)
    print("f 0 0 1 5")
    time.sleep(5)

def progressive_increase_velocity():
    return 

def get_op_code(cmd):
    return cmd.split()[0]

def get_operands(cmd):
    return ''.join(cmd.split()[1:])

def run_cmds(cmds):
    for cmd in cmds:
        if get_op_code(cmd) == "sleep":
            time.sleep(int(get_operands(cmd)))
        else:
            #sys.stdout.write(cmd)
            print(cmd) 
            sys.stdout.flush()
            #print(cmd)

#lift off  (total duration: 30 seconds)
def lift_up(cmds):
    cmds.append("sleep 10")
    cmds.append("f 0 0 2 3")
    cmds.append("sleep 20")


#accelerate forward (total of 30 seconds)
def accel_fw(cmds, max_vel):
    for i in range(0,30):
        cmds.append("f 0 " + str(max_vel)+ " 0 2");
        cmds.append("sleep 1");


#accelerate forward (total of 30 seconds)
def accel_uw(cmds, max_vel):
    for i in range(0,30):
        cmds.append("f 0 0 " + str(max_vel) + " 2");
        cmds.append("sleep 1");


#accelerate forward (total of 30 seconds)
def full_stop(cmds): 
    for i in range(0,30):
        cmds.append("f 0 0 0 2");
        cmds.append("sleep 1");


def deccel(cmds, max_vel): 
    for i in range(0,30):
        cmds.append("f 0 " + str(-1*max_vel)+ " 0 2");
        cmds.append("sleep 1");




# lift up, accel up, accel fw, decel
def path_1():
    #max_vel = [30, 20, 10, 8, 6, 4, 2] 
    max_vel = 30
    cmds = [] 
    lift_up(cmds)
    accel_uw(cmds, max_vel)
    accel_fw(cmds, max_vel)
    full_stop(cmds) 
    #deccel(cmds, max_vel)
    return cmds

def main():
    cmds = path_1()    
    print(cmds)  
    run_cmds(cmds) 
    #pre_mission(cmds)

main()

