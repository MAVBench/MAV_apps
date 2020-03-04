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

def run_optimizer(control):
    time_budget_left_to_distribute = control.internal_states.sensor_to_actuation_time_budget_to_enforce - misc_latency
    rt_d = min(time_budget_left_to_distribute, rt_max)
    r_gap = max(control.inputs.gap_statistics, r_min)  # can't really see smaller than r_min anyways
    v_sensor_max = control.inputs.sensor_volume_to_digest_estimated

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
        [ pl_to_ppl_ratio*1.00483609e-01,  pl_to_ppl_ratio*1.80366135e-05,  pl_to_ppl_ratio*4.71434480e-03]])

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
    

def control_callback(control):
    results = run_optimizer(control)

    # some default values
    pc_res = pc_res_min
    pc_vol_ideal = 10000
    om_to_pl_res = om_to_pl_res_min
    om_to_pl_vol_ideal = 200000
    ppl_vol_ideal = 200001

    om_latency_expected = .05
    om_to_pl_latency_expected = .05
    ppl_latency_expected = .05
    smoothening_latency_expected = .05

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
        # round up to exponents of 2
        pc_res = (2 ** math.ceil(math.log(r0/pc_res_min, 2)))*pc_res_min
        om_to_pl_res = (2 ** math.ceil(math.log(r1/om_to_pl_res_min, 2)))*om_to_pl_res_min
        pc_vol_ideal = min(results.x[2], pc_vol_ideal)
        om_to_pl_vol_ideal = min(results.x[3], om_to_pl_vol_ideal)
        ppl_vol_ideal = min(results.x[4], ppl_vol_ideal)
        # expected time budgets
        om_latency_expected = results.exp_task_times[0]
        om_to_pl_latency_expected = results.exp_task_times[1]
        ppl_latency_expected = results.exp_task_times[2]/pl_to_ppl_ratio
        smoothening_latency_expected = results.exp_task_times[2]/pl_to_ppl_ratio
    else:
        rospy.set_param("optimizer_succeeded", False)
        return

    rospy.set_param("optimizer_succeeded", True)
    ee_latency_expected = om_latency_expected + om_to_pl_latency_expected + pl_to_ppl_ratio*ppl_latency_expected + misc_latency
    # set the knobs
    rospy.set_param("pc_res", float(pc_res))
    rospy.set_param("pc_vol_ideal", float(pc_vol_ideal))
    rospy.set_param("om_to_pl_res", float(om_to_pl_res))
    rospy.set_param("om_to_pl_vol_ideal", float(om_to_pl_vol_ideal))
    rospy.set_param("ppl_vol_ideal", float(ppl_vol_ideal))

    # time budgets (expected with the above knob settings)
    rospy.set_param("sensor_to_actuation_time_budget_to_enforce", control.internal_states.sensor_to_actuation_time_budget_to_enforce)
    rospy.set_param("om_latency_expected", float(om_latency_expected))
    rospy.set_param("om_to_pl_latency_expected", float(om_to_pl_latency_expected))
    rospy.set_param("ppl_latency_expected", float(ppl_latency_expected))
    rospy.set_param("smoothening_latency_expected", float(smoothening_latency_expected))
    rospy.set_param("ee_latency_expected", float(ee_latency_expected))

    rospy.set_param("new_control_data", True)  # Important: set this one last to ensure all other knobs/vars are set


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
    rospy.Subscriber("control_to_pyrun", control, control_callback)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
