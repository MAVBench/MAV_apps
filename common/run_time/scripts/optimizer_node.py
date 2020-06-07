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
obs_dist_min_calc_from_octomap = 100

class dummy_output:
    def __init__(self):
        self.success = False

def run_optimizer(control):

    # -- determine the response time destired (rt_d)
    time_budget_left_to_distribute = control.internal_states.sensor_to_actuation_time_budget_to_enforce - misc_latency # the second run_time_latency is for the following iteration
    if (time_budget_left_to_distribute < 0):
        print("-----------------**********&&&&budget is less than 0.this really shouldn't happen")
        results = dummy_output()
        failure_status = 2
        return results, failure_status
    rt_d = min(time_budget_left_to_distribute, rt_max)
    rt_d = max(rt_d, .2)

    # -- determine the resolution
    # intuitively speaking, min value for resolution (r_min) indicate, we don't need to do better than that
    # we can also think of r_min as what regulates the intellignce (i.e, the lower r_min the more intelligence the decision
    # making since we have more free space to make decision based off of)
    # and max value for resolution indicates  that we can't not do worse than this
    r_min_temp = min(control.inputs.gap_statistics_avg/2, control.inputs.obs_dist_statistics_avg/2) - drone_radius  # min to impose the worse case as the
                                                                                                                # determinant

    r_min_temp = min(r_min_temp, r_max_static) # not exceed r_max
    r_min_temp = max(r_min_static, r_min_temp)  # not lower than r_min_static
    r_min_ = r_min_temp
    #obs_dist_min = min(control.inputs.obs_dist_statics_min_from_om)
    r_max_temp = min(control.inputs.gap_statistics_max/2, control.inputs.obs_dist_statistics_min/2) - drone_radius  # min is because we want the tigher bound of the two:
                                                                                                                 # note that if r_max is greater than
                                                                                                                 # r_gap_max, we can't actually see any gaps
    r_max_temp = max(r_max_temp, r_min_static)  # not lower than r_min_static
    r_max_ = min(r_max_temp, r_max_static)  # not aabove r_max_static
    r_max_ = (2 ** math.floor(math.log(round(r_max_ /r_min_static, 2), 2))) * r_min_static  # must get the floor otherwise, when converting (after solving), when we get the floor, we might go over
                                                                                        # the max value

    if r_max_ < r_min_:
        print("-------------------------------------------- bounds inverted -----------------------------------")
        r_min_ = r_max_
    #r_gap_hat = 1 / r_gap
    r_max_hat = 1 / r_min_
    r_min_hat = 1 / r_max_
    r_min_list = [r_min_hat] * 2
    r_max_list = [r_max_hat] * 2

    # --- determine the volume
    v_sensor_max = control.inputs.sensor_volume_to_digest
    v_tree_max = max(control.inputs.cur_tree_total_volume, v_min) + v_sensor_max#, v_sensor_max)
    #print("before anything"+ str(control.inputs.ppl_vol_min))
    ppl_vol_min = max(control.inputs.ppl_vol_min, 10*v_sensor_max)

    v_min_list = [min(v_min, .9*v_sensor_max)] * 2 + [ppl_vol_min]  # not that .9*v_sensor_max is there
                                                                    # because sometimes, we actually might have a smaller
                                                                    # than v_min volume, in which case we need to
                                                                    # provide a volume as a min that is smaller
                                                                    # than what we can already see (v_sennsor_max)
    #print("fufufufufufuf" +str(ppl_vol_min))
    v_max_list = [(r_max_/r_min_static)*v_sensor_max, (r_max_/r_min_static)*v_tree_max, max(v_max, ppl_vol_min)]


    Q = np.array([[-2.16196038e-05, -2.78515364e-03,  2.86859999e-05],
        [ 2.00720436e-04,  4.60333360e-02, -1.05093373e-05],
        [ 1.34399197e-04,  4.64316885e-02,  1.24233987e-05],
        [pl_to_ppl_ratio*1.00483609e-01,  pl_to_ppl_ratio*1.80366135e-05,  pl_to_ppl_ratio*4.71434480e-03]])
    # Constraint matrices #

    #  --  w/ r_gap  as constraint
    #G = np.array([[-1,1,0,0,0], [0,0,1,-1,0], [0,0,1,0,0], [-1,0,0,0,0]])
    #d = np.array([0, 0, v_sensor_max, -r_gap_hat])
    # -- w/o r_gap as the constraint (PS: moved the gap constraint directly into the boundary conditions)
    G = np.array([[-1,1,0,0,0], [0,0,1,-1,0], [0,0,1,0,0]])
    d = np.array([0, 0, (r_max_/r_min_static)*v_sensor_max])


    opt = Opt(method="var5_rhat_volmax",
              Q=Q,
              r_min=r_min_list,
              r_max=r_max_list,
              v_min=v_min_list,
              v_max=v_max_list,
              G=G,
              d=d)

    # Optimization parameters #
    x0 = np.array([1/0.5, 1/0.5, (r_max_/r_min_static)*5000, (r_max_/r_min_static)*5000, r_max_*5000])
    profile = True
    tol = 1e-12
    results = opt.opt(profile=profile, rt_d=rt_d, x0=x0, tol=tol, verbose=False)
    if not results.success:
        failure_status = 1
    else:
        failure_status = 0
    return results, failure_status
    

def control_callback(control):
    print("---- runing the optimizer now")
    results, failure_status = run_optimizer(control)  # failure status key: 1:no valid cofig, 2:not enough time


    # some default values
    #pc_res = r_min_static
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
        """
        blah =  math.log(round(r0/r_min_static,2), 2)
        blah2 = math.log(round(r1/om_to_pl_res_min,2), 2)
        blah_3 =  math.ceil(blah)
        blah4 = math.ceil(blah2)
        """
        pc_res = (2 ** math.ceil(math.log(round(r0/r_min_static,2), 2)))*r_min_static  # round is there, because, the
                                                                                   # float division sometimes gives slightly different resutls (say sometimes 0.0, sometimes 3.6*e**--16
                                                                                   # which results in an assertion error
        om_to_pl_res = (2 ** math.ceil(math.log(round(r1/om_to_pl_res_min, 2), 2)))*om_to_pl_res_min
        assert(om_to_pl_res >= pc_res), "om_to_pl_res should be >= pc_res"
        pc_vol_ideal = results.x[2]
        #pc_vol_ideal = 20000
        om_to_pl_vol_ideal = results.x[3]
        ppl_vol_ideal = results.x[4]
        # expected time budgets
        om_latency_expected = results.exp_task_times[0]
        om_to_pl_latency_expected = results.exp_task_times[1]
        ppl_latency_expected = results.exp_task_times[2]/pl_to_ppl_ratio
        smoothening_latency_expected = results.exp_task_times[2]/pl_to_ppl_ratio
    else:
        rospy.set_param("optimizer_failure_status", failure_status)
        rospy.set_param("optimizer_succeeded", False)
        rospy.set_param("log_control_data", False)
        rospy.set_param("new_control_data", True)  # Important: set this one last to ensure all other knobs/vars are set
        print("====================optimizer failed")
        return

    rospy.set_param("optimizer_succeeded", True)
    rospy.set_param("log_control_data", True)
    ee_latency_expected = om_latency_expected + om_to_pl_latency_expected + pl_to_ppl_ratio*ppl_latency_expected + misc_latency
    # set the knobs
    rospy.set_param("pc_res", float(pc_res))
    rospy.set_param("pc_vol_ideal", float(pc_vol_ideal))
    #rospy.set_param("pc_vol_ideal", 11000)
    rospy.set_param("om_to_pl_res", float(om_to_pl_res))
    rospy.set_param("om_to_pl_vol_ideal", float(om_to_pl_vol_ideal))
    rospy.set_param("ppl_vol_ideal", float(ppl_vol_ideal))
    rospy.set_param("optimizer_failure_status", failure_status)
    # time budgets (expected with the above knob settings)
    rospy.set_param("sensor_to_actuation_time_budget_to_enforce", control.internal_states.sensor_to_actuation_time_budget_to_enforce)
    rospy.set_param("om_latency_expected", float(om_latency_expected))
    rospy.set_param("om_to_pl_latency_expected", float(om_to_pl_latency_expected))
    rospy.set_param("ppl_latency_expected", float(ppl_latency_expected))
    rospy.set_param("smoothening_latency_expected", float(smoothening_latency_expected))
    rospy.set_param("ee_latency_expected", float(ee_latency_expected))

    rospy.set_param("x_coord_while_budgetting", float(control.internal_states.drone_point_while_budgetting.x))
    rospy.set_param("y_coord_while_budgetting", float(control.internal_states.drone_point_while_budgetting.y))
    rospy.set_param("vel_mag_while_budgetting", float(control.internal_states.drone_point_while_budgetting.vel_mag))




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
    v_max = max(rospy.get_param("om_to_pl_vol_ideal_max"), rospy.get_param("ppl_vol_ideal_max"))
    v_min = rospy.get_param("pc_vol_ideal_min")
    r_min_static = rospy.get_param("pc_res_max") # we have actually swapped these mistakenly, i.e., chosen min for max. but launch file is correct and python is incorrect
    r_steps = rospy.get_param("num_of_res_to_try") # we have actually swapped these mistakenly, i.e., chosen min for max. but launch file is correct and python is incorrect
    r_max_static = (2 ** r_steps) * r_min_static
    while not rospy.is_shutdown():
        rate.sleep()
