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
rt_max = 10  # this is actually set in the roslaunch file
drone_radius = 3
import time
dist_to_closest_obs_calc_from_octomap = 100

def run_optimizer(control):
    time_budget_left_to_distribute = control.internal_states.sensor_to_actuation_time_budget_to_enforce - misc_latency
    if (time_budget_left_to_distribute < 0):
        print("-----------------**********&&&&budget is less than 0.this really shouldn't happen")
#    else:
#        print("budget in python"+ str(control.internal_states.sensor_to_actuation_time_budget_to_enforce))
    rt_d = min(time_budget_left_to_distribute, rt_max)
    rt_d = max(rt_d, .2)
#    time.sleep(10)
#    r_gap_max = max(control.inputs.gap_statistics_max, r_min)  # can't really see smaller than r_min anyways
    r_gap_max = control.inputs.gap_statistics_max  # can't really see smaller than r_min anyways
    r_gap_avg = control.inputs.gap_statistics_avg
    obs_dist_avg = control.inputs.obs_dist_statistics_avg
    obs_dist_min = min(control.inputs.obs_dist_statistics_min, dist_to_closest_obs_calc_from_octomap)

    v_sensor_max = control.inputs.sensor_volume_to_digest
    v_tree_max = max(control.inputs.cur_tree_total_volume, v_min)#, v_sensor_max)
    #print("before anything"+ str(control.inputs.ppl_vol_min))
    ppl_vol_min = max(control.inputs.ppl_vol_min, 10*v_sensor_max)

    #
    r_min_temp =  max(max(r_gap_avg, obs_dist_avg) - drone_radius, r_min)  # get the minium between the average gap
                                                               # and average obstacle distance
    r_min_ = max(r_min, min(r_min_temp , r_max))  # the outer max term makes sure the resolution is not 0
                                                           # the inner min, is for not exceeding r_max



    r_max_temp = max(min(r_gap_max, obs_dist_min) - drone_radius, r_min) # get the tigher bound of the two:
                                                                                 # aka, maximum gap and closest obstacle
                                                                                 # the intuition is that if
                                                                                 # our resolution is bigger than either
                                                                                 # we can't see the gaps/distance properly
    r_max_ = min(r_max_temp, r_max)
    if r_max_ < r_min_:
        print("-------------------------------------------- bounds inverted -----------------------------------")
        r_min_ = r_max_
#    r_max_ = r_max - drone_radius
    #r_gap_hat = 1 / r_gap
    r_max_hat = 1 / r_min_
    r_min_hat = 1 / r_max_

    r_min_list = [r_min_hat] * 2
    r_max_list = [r_max_hat] * 2
    v_min_list = [min(v_min, .9*v_sensor_max)] * 2 + [ppl_vol_min]
    #print("fufufufufufuf" +str(ppl_vol_min))
    v_max_list = [v_sensor_max, v_tree_max, max(v_max, ppl_vol_min)]


    Q = np.array([[-2.16196038e-05, -2.78515364e-03,  2.86859999e-05],
        [ 2.00720436e-04,  4.60333360e-02, -1.05093373e-05],
        [ 1.34399197e-04,  4.64316885e-02,  1.24233987e-05],
        [ pl_to_ppl_ratio*1.00483609e-01,  pl_to_ppl_ratio*1.80366135e-05,  pl_to_ppl_ratio*4.71434480e-03]])
    # Constraint matrices #

    #  --  w/ r_gap  as constraint
    #G = np.array([[-1,1,0,0,0], [0,0,1,-1,0], [0,0,1,0,0], [-1,0,0,0,0]])
    #d = np.array([0, 0, v_sensor_max, -r_gap_hat])
    # -- w/o r_gap as the constraint (PS: moved the gap constraint directly into the boundary conditions)
    G = np.array([[-1,1,0,0,0], [0,0,1,-1,0], [0,0,1,0,0]])
    d = np.array([0, 0, v_sensor_max])


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
    #pc_res = pc_res_min
    #om_to_pl_res = om_to_pl_res_min
    #om_to_pl_vol_ideal = 200000
    #ppl_vol_ideal = 200001
    #om_latency_expected = .05
    #om_to_pl_latency_expected = .05
    #ppl_latency_expected = .05
    #smoothening_latency_expected = .05

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
        pc_vol_ideal = results.x[2]
        om_to_pl_vol_ideal = results.x[3]
        ppl_vol_ideal = results.x[4]
        # expected time budgets
        om_latency_expected = results.exp_task_times[0]
        om_to_pl_latency_expected = results.exp_task_times[1]
        ppl_latency_expected = results.exp_task_times[2]/pl_to_ppl_ratio
        smoothening_latency_expected = results.exp_task_times[2]/pl_to_ppl_ratio
    else:
        rospy.set_param("optimizer_succeeded", False)
        rospy.set_param("new_control_data", True)  # Important: set this one last to ensure all other knobs/vars are set
        print("====================optimizer failed")
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
    rt_max = rospy.get_param("max_time_budget")
    drone_radius = rospy.get_param("planner_drone_radius")
    while not rospy.is_shutdown():
        dist_to_closest_obs_calc_from_octomap = rospy.get_param("dist_to_closest_obs_calc_from_octomap")
        rate.sleep()
