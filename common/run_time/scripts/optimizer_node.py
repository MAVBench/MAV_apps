#!/usr/bin/env python2.7

import roslib
import rospy
import sys
from mavbench_msgs.msg import control_input
from optimizer.opt import Opt
import time
import numpy as np
import math

pc_res_min = .15
om_to_pl_res_min = .15

def run_optimizer(control_inputs):

    r_min = .9*pc_res_min # minimal octomap res
    r_steps = 5
    r_max = (2**r_steps)*r_min

    v_min = 0
    v_max = np.inf
    rt_max = 60

    rt_d = min(control_inputs.sensor_to_actuation_time_budget, rt_max)
    r_gap = max(control_inputs.gap_statistics, r_min)  # can't really see smaller than r_min anyways
    v_sensor_max = control_inputs.sensor_volume_to_digest_estimated

    r_gap_hat = 1 / r_gap
    r_max_hat = 1 / r_min
    r_min_hat = 1 / r_max

    r_min_list = [r_min_hat] * 2
    r_max_list = [r_max_hat] * 2
    v_min_list = [v_min] * 3
    v_max_list = [v_max] * 3


    Q = np.array([[-2.16196038e-05, -2.78515364e-03,  2.86859999e-05],
        [ 2.00720436e-04,  4.60333360e-02, -1.05093373e-05],
        [ 1.34399197e-04,  4.64316885e-02,  1.24233987e-05],
        [ 1.00483609e-01,  1.80366135e-05,  4.71434480e-03]])

    # Constraint matrices #
    G = np.array([[-1,1,0,0,0], [0,0,1,-1,0], [0,0,1,0,0], [-1,0,0,0,0]])
    d = np.array([0, 0, v_sensor_max, -r_gap_hat])

    opt = Opt(Q=Q, 
            r_min=r_min_list,
            r_max=r_max_list,
            v_min=v_min_list,
            v_max=v_max_list,
            G=G,
            d=d)

    # Optimization parameters #
    x0 = np.array([1/0.5, 1/0.5, 5000, 5000, 5000])
    profile = True
    tol = 1e-12
    results = opt.opt(profile=profile, rt_d=rt_d, x0=x0, tol=tol, verbose=False)
    return results
    
def control_input_callback(control_input_):
    control_inputs = control_input_
    results = run_optimizer(control_inputs)
    # some default values
    pc_res = pc_res_min
    pc_vol_ideal = 10000
    om_to_pl_res = om_to_pl_res_min
    om_to_pl_vol_ideal = 200000
    ppl_vol_ideal = 40001
    ppl_budget = .05
    # todo: add guards for minimums and maximus
    # kosherize the resolution to be an eponent of 2* base
    if results.success:
        r0_hat = results.x[0]
        r1_hat = results.x[1]
        v0_hat = results.x[2]
        v1_hat = results.x[3]
        v2_hat = results.x[4]

        r0 = 1.0/r0_hat
        r1 = 1.0/r1_hat
        pc_res = (2 ** math.ceil(math.log(r0/pc_res_min, 2)))*pc_res_min
        om_to_pl_res = (2 ** math.ceil(math.log(r1/om_to_pl_res_min, 2)))*om_to_pl_res_min
        pc_vol_ideal = min(results.x[2], pc_vol_ideal)
        om_to_pl_vol_ideal = min(results.x[3], om_to_pl_vol_ideal)
        ppl_vol_ideal = min(results.x[4], ppl_vol_ideal)
        #ppl_budget = result.


    rospy.set_param("ppl_budget", float(ppl_budget))
    rospy.set_param("pc_res", float(pc_res))
    rospy.set_param("pc_vol_ideal", float(pc_vol_ideal))
    rospy.set_param("om_to_pl_res", float(om_to_pl_res))
    rospy.set_param("om_to_pl_vol_ideal", float(om_to_pl_vol_ideal))
    rospy.set_param("ppl_vol_ideal", float(ppl_vol_ideal))
    rospy.set_param("new_control_data", True)

    """ 
    rospy.set_param("pc_res", .15)
    rospy.set_param("pc_vol_ideal", 8000)
    rospy.set_param("om_to_pl_res", .15)
    rospy.set_param("om_to_pl_vol_ideal", 200000)
    rospy.set_param("ppl_vol_ideal", 40001)
    rospy.set_param("new_control_data", True)
    """

if __name__ == '__main__':
    #op_obj = Opt()
    #opt_obj.opt()
    rospy.init_node('runtime_thread_python', anonymous=True)
    rospy.Subscriber("control_inputs_to_pyrun", control_input, control_input_callback) 
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
