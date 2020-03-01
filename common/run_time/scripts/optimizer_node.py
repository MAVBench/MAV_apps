#!/usr/bin/env python2.7

import roslib
import rospy
import sys
from mavbench_msgs.msg import control_input
#from optimizer import Opt
import time

def control_input_callback(control_input_):
    #control_inputs = control_input_        
    rospy.set_param("pc_res", .15)
    rospy.set_param("pc_vol_ideal", 8000)
    rospy.set_param("om_to_pl_res", .15)
    rospy.set_param("om_to_pl_vol_ideal", 200000)
    rospy.set_param("ppl_vol_ideal", 40001)
    rospy.set_param("new_control_data", True)

if __name__ == '__main__':
    #op_obj = Opt()
    #opt_obj.opt()
    rospy.init_node('runtime_thread_python', anonymous=True)
    rospy.Subscriber("control_inputs_to_pyrun", control_input, control_input_callback) 
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
	rate.sleep()
