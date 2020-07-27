#!/usr/bin/env python2.7

import roslib
import rospy
import sys
import os
sys.path.append(os.getenv('base_dir') +"/src/MAV_apps/data_processing/common_utils")
sys.path.append(os.getenv('base_dir') +"/src/MAV_apps/common/run_time/src/model/")
sys.path.append(os.getenv('base_dir') +"/src/MAV_apps/common/run_time/src/model/model_generation")
sys.path.append(os.getenv('base_dir') +"/src/MAV_apps/common/run_time/src/")
from mavbench_msgs.msg import control
from optimizer.opt import Opt
import time
import numpy as np
import math
from optimizer_settings import *
from system_profiler import mapping
import psutil
rt_max = 5  # this is actually set in the roslaunch file
drone_radius = 3
import time
obs_dist_min_calc_from_octomap = 100
import copy
from profiler import *
from data_parser  import DataParser
from models import Model
num_of_processors = 1
from model_generation.model_gen import *
om_popt = np.ndarray((2,2))
om_pl_popt = np.ndarray((2,2))
pp_pl_popt = np.ndarray((2,2))
typical_model = Model(3, 1)
distance_to_goal = 1000
use_gap_map = False
gap_map_gap = 25

def collect_hw_counters():
    try:
        events= [['SYSTEMWIDE:PERF_COUNT_HW_INSTRUCTIONS'],
                ['PERF_COUNT_HW_BRANCH_INSTRUCTIONS','PERF_COUNT_HW_BRANCH_MISSES'],
                ['PERF_COUNT_SW_PAGE_FAULTS']]
        perf= Profiler(program_args= ['/bin/ls','/'], events_groups=events)
        data= perf.run(sample_period= 0.01)
        #print(data)
    except RuntimeError as e:
        print(e.args[0])
    



class dummy_output:
    def __init__(self):
        self.success = False
class SmartQueue:
    def __init__(self, size):
        self.storage = []
        self.size = size 

    def push(self, value): 
        self.storage.append(value)
        if len(self.storage) > self.size:
            storage = copy.deepcopy(self.storage[1:])
            self.storage = storage

    def reduce(self, mode):
        if mode == "min":
            return min(self.storage)
        elif mode == "max":
            return max(self.storage)
        elif mode == "avg":
            return avg(self.storage)
r_max_queue = SmartQueue(5)

motivation_section = False
def run_optimizer(control):
    cmd_rcvd_time = rospy.Time.now()
    global r_max_queue, use_gap_map, motivation_section, r_max_queue
    rt_max = control.inputs.max_time_budget 
    cur_vel_mag = rospy.get_param("velocity_to_budget_on")
    #print("====cur vel amg " + str(cur_vel_mag)) 
    """ 
    print("gap_statistics_max" + str(control.inputs.gap_statistics_max))
    print("gap_statistics_avg" + str(control.inputs.gap_statistics_avg))
    print("obs min" + str(control.inputs.obs_dist_statistics_min))
    print("obs avg" + str(control.inputs.obs_dist_statistics_avg))
    """
    """
    if (failure_ctr_for_pc_res == 5):
        r_max_static = max(r_min_static, r_max_static/2) 
    elif (failure_ctr_for_pc_res == 0):
        r_max_static = max(2*r_max_static, r_max_static_global) 
    """

    #print("rt_max" + str(rt_max))
    # -- determine the response time destired (rt_d)
    time_budget_left_to_distribute = control.internal_states.sensor_to_actuation_time_budget_to_enforce - (back_end_latency + front_end_latency)# the second run_time_latency is for the following iteration
    #time_budget_left_to_distribute = 10
    #print("budget is" + str(time_budget_left_to_distribute))
    if (time_budget_left_to_distribute < runtime_latency):
        #print("-----------------**********&&&&budget is less than 0. budgget given" + str(time_budget_left_to_distribute))
        results = dummy_output()
        failure_status = 2
        return results, failure_status
    rt_d = min(time_budget_left_to_distribute, rt_max)

    # -- determine the resolution
    # intuitively speaking, min value for resolution (r_min) indicate, we don't need to do better than that
    # we can also think of r_min as what regulates the intellignce (i.e, the lower r_min the more intelligence the decision
    # making since we have more free space to make decision based off of)
    # and max value for resolution indicates  that we can't not do worse than this
    r_min_temp = min(control.inputs.gap_statistics_avg/2, control.inputs.obs_dist_statistics_avg) #- drone_radius  # min to impose the worse case as the

    r_min_temp = min(r_min_temp, r_max_static) # not exceed r_max
    r_min_temp = max(r_min_static, r_min_temp)  # not lower than r_min_static
    r_min_ = r_min_temp
    #obs_dist_min = min(control.inputs.obs_dist_statics_min_from_om)
    #print("=================================avg gap size" + str(control.inputs.gap_statistics_avg))
    #r_max_temp = min(control.inputs.gap_statistics_max, control.inputs.obs_dist_statistics_min) # - drone_radius  # min is because we want the tigher bound of the two:
#    r_max_temp = min(control.inputs.gap_statistics_avg, control.inputs.obs_dist_statistics_min) # - drone_radius  # min is because we want the tigher bound of the two:
    r_max_temp = min(control.inputs.gap_statistics_avg, control.inputs.obs_dist_statistics_min) # - drone_radius  # min is because we want the tigher bound of the two:
    #if motivation_section:
    #    r_max_temp = control.inputs.obs_dist_statistics_min # - drone_radius  # min is because we want the tigher bound of the two:

    if (use_gap_map):
        r_max_temp = min(r_max_temp, gap_map_gap)
    global distance_to_goal
    if distance_to_goal < 40:
        r_min_temp = r_max_temp = min(4*r_min_static, r_min_temp, r_max_temp)
        planning_failure_cntr_coeff = 1
    else:
        if control.inputs.planner_consecutive_failure_ctr > 2:  # this is for the scenarios where we are surrounded by free space, but the goal is not
            r_min_temp = r_max_temp = r_min_static 
            planning_failure_cntr_coeff = 2
        if control.inputs.planner_consecutive_failure_ctr > 4:  # this is for the scenarios where we are surrounded by free space, but the goal is not
            planning_failure_cntr_coeff = 4 
        else:
            planning_failure_cntr_coeff = 1
    # r_gap_max, we can't actually see any gaps
    r_max_temp = max(r_max_temp, r_min_static)  # not lower than r_min_static
    r_max_ = min(r_max_temp, r_max_static)  # not aabove r_max_static
    r_max_ = (2 ** math.floor(math.log(round(r_max_ /r_min_static, 2), 2))) * r_min_static  # must get the floor otherwise, when converting (after solving), when we get the floor, we might go over
                                                                                        # the max value
    if not use_gap_map: 
        r_max_queue.push(r_max_)
        r_max_ = r_max_queue.reduce("min")

    if r_max_ < r_min_:
        #print("-------------------------------------------- bounds inverted -----------------------------------")
        r_min_ = r_max_
    #r_gap_hat = 1 / r_gap
    r_max_hat = 1 / r_min_
    r_min_hat = 1 / r_max_
    r_min_list = [r_min_hat] * 2
    r_max_list = [r_max_hat] * 2

    # --- determine the volume
    v_sensor_max = 2*control.inputs.sensor_volume_to_digest
    #print("v sensor max is" + str(v_sensor_max))
    v_tree_max = max(max(control.inputs.cur_tree_total_volume, v_min) + v_sensor_max, 100000)
    #print("before anything"+ str(control.inputs.ppl_vol_min))
    ppl_vol_min = max(max(control.inputs.ppl_vol_min, v_sensor_max), 100000)


    """
    om_response_time_max = calculate_fitted_value(om_popt, r_min_, v_min_list[0], typical_model)
    om_pl_response_time_max = calculate_fitted_value(om_pl_popt, r_min_, v_min_list[1], typical_model)
    pp_pl_response_time_max = calculate_fitted_value(pp_pl_popt, r_min_, v_min_list[2], typical_model)
    if om_response_time_max + om_pl_response_time_max + pp_pl_response_time_max > rt_d:
        print("&&&&&&&&& ------------ this can not happend")
    """
    for i in np.arange(1, 15, .1):
        v_min_list = [min(v_min, .9*v_sensor_max)] * 2 + [ppl_vol_min]  # not that .9*v_sensor_max is there
                                                                        # because sometimes, we actually might have a smaller
                                                                        # than v_min volume, in which case we need to
                                                                        # provide a volume as a min that is smaller
                                                                        # than what we can already see (v_sennsor_max)
        #v_max_list = [(2**i)*(r_max_/r_min_static)*v_sensor_max, (2**i)*(r_max_/r_min_static)*v_tree_max, (2**i)*max(v_max, ppl_vol_min)]

        #v_min_list = [min(v_min, .9*v_sensor_max) , 9*(r_max_/r_min_static)*v_tree_max, 2* max(v_max, 2*ppl_vol_min)]
        v_max_list = [(r_max_/r_min_static)*v_sensor_max, 2*(r_max_/r_min_static)*v_tree_max, 2* max(v_max, 2*ppl_vol_min)]

        # sanity check on response time 
        global om_popt, om_pl_popt, pp_pl_popt, typical_model

        v_min_list_copy = v_min_list[:]
        """ 
        if motivation_section:
            v_min_list = [50000, 150000, 150000]
            v_max_list = [50000, 150000, 150000]
        """

        while True:
            om_response_time_min = calculate_fitted_value(om_popt, r_max_, v_min_list[0], typical_model)
            om_pl_response_time_min = calculate_fitted_value(om_pl_popt, r_max_, v_min_list[1], typical_model)
            pp_pl_response_time_min = calculate_fitted_value(pp_pl_popt, r_max_, v_min_list[2], typical_model)
            if om_response_time_min + om_pl_response_time_min + pp_pl_response_time_min < rt_d:
                break
            v_min_list = [.99*v_min_list[0] , v_min_list[1], .9*v_min_list[2]]
            

            #print(v_min_list) 
            #print(rt_d)
            #print("&&&&&&&&& ------------ this can not happend")


        """
        Q = np.array([[-2.16196038e-05, -2.78515364e-03,  pl_to_ppl_ratio*2.86859999e-05],
            [2.00720436e-04,  4.60333360e-02, pl_to_ppl_ratio*-1.05093373e-05],
            [1.34399197e-04,  4.64316885e-02,  pl_to_ppl_ratio*1.24233987e-05],
            [1.00483609e-01,  1.80366135e-05,  pl_to_ppl_ratio*4.71434480e-03]])
        """
        """
        [-0.01420042  0.01081166 -0.00413957 -0.00029888]
        [9.78020632e-01 -7.03707300e-01   4.03608560e+00   1.36792938e-07]
        [2.31901204e-03 -2.72136219e-02 -7.21721385e-01 -7.93304493e-06]
        """
        """
        Q = np.array([[-0.01420042, 9.78020632e-01,  pl_to_ppl_ratio*2.31901204e-03],
                      [ 0.01081166 , -7.03707300e-01 , pl_to_ppl_ratio*-2.72136219e-02],
                      [-0.00413957, 4.03608560e+00 ,  pl_to_ppl_ratio*-7.21721385e-01],
                      [-0.00029888, 1.36792938e-07,  pl_to_ppl_ratio*-7.93304493e-06]])
        """
        Q = np.array([[-0.00117776, 8.56187525e-05,  pl_to_ppl_ratio*1.98113709e-02],
                      [0.00061975, -1.01646525e-03, pl_to_ppl_ratio*-4.03797137e-02],
                      [-0.00035994 , 1.43034344e-04 ,  pl_to_ppl_ratio*6.16302710e-02],
                      [-0.00375029, -7.02909707e-04,  pl_to_ppl_ratio*7.57338509e-05]])

        """
        [0.00524626 -0.00355227  0.00126292  0.00084077]
        [1.44560156e-06 -1.74231236e-06  4.02383524e-07  1.11044414e-01]
        [1.98113709e-02 -4.03797137e-02  6.16302710e-02  7.57338509e-05]
        """
        # Constraint matrices #

        #  --  w/ r_gap  as constraint
        #G = np.array([[-1,1,0,0,0], [0,0,1,-1,0], [0,0,1,0,0], [-1,0,0,0,0]])
        #d = np.array([0, 0, v_sensor_max, -r_gap_hat])
        # -- w/o r_gap as the constraint (PS: moved the gap constraint directly into the boundary conditions)

        if (control.internal_states.drone_point_while_budgetting.vel_mag < .2):
            pc_om_map_ratio = 1
        else:
            pc_om_map_ratio = .3

        
        G = np.array([[-1,1,0,0,0], [0,0,planning_failure_cntr_coeff,-pc_om_map_ratio,0], [0,0,0, planning_failure_cntr_coeff,-3], [0,0,1,0,0]])
        d = np.array([0, 0, 0, (r_max_/r_min_static)*v_sensor_max])
        if r_min_ == r_max_:
            G = np.array([[0, 0, planning_failure_cntr_coeff, -pc_om_map_ratio, 0],
                          [0, 0, 0, planning_failure_cntr_coeff, -3], [0, 0, 1, 0, 0]])
            d = np.array([0, 0, (r_max_/r_min_static)*v_sensor_max])

            
        opt = Opt(method="var5_rhat_volmax",
                  Q=Q,
                  r_min=r_min_list,
                  r_max=r_max_list,
                  v_min=v_min_list,
                  v_max=v_max_list,
                  G=G,
                  d=d)

        rt_d_ = rt_d/i
        #print("well now 00000000000000000000" +str(rt_d_))
        # Optimization parameters #
        x0 = np.array([1/0.5, 1/0.5, (r_max_/r_min_static)*5000, 3*planning_failure_cntr_coeff*(r_max_/r_min_static)*5000, 3*planning_failure_cntr_coeff*r_max_*5000])
        profile = True
        tol = 1e-12
        results = opt.opt(profile=profile, rt_d=rt_d_, x0=x0, tol=tol, verbose=False)
        """
        if not results.success:
            print("v_min_list" + str(v_min_list))
            print("v_min_list_copy" + str(v_min_list_copy))
            print("v_max_list" + str(v_max_list))
            print("om_vol must be < " + str(pc_om_map_ratio / planning_failure_cntr_coeff) + " om_to_pl")
            print("om_to_pl < " + str(3) + "*ppl_pl_vol")
            print("om_vol < " + str(r_max_ / r_min_static * v_sensor_max))
            print("v_sensor_max" + str(v_sensor_max))
            print("rmin_list" + str(r_min_list))
            print("rmax_hat" + str(r_max_list))
            print("rt_d" + str(rt_d_))
         """
        if results.success:
            break

    if not results.success:
        if (time_budget_left_to_distribute - ((rospy.Time.now() - cmd_rcvd_time).to_sec())) < front_end_latency: # don't have time for another round
            #print("-----------------**********&&&&budget is less than 0.this really shouldn't happen")
            failure_status = 2
        else:
            failure_status = 1
    else:
        failure_status = 0
    return results, failure_status


def control_callback_for_crun(control):
    if budgetting_mode == "static":
        all_cpu_percents = psutil.cpu_percent(percpu=True)[0:num_of_processors]
        cpu_utilization_last_round = sum(all_cpu_percents[0:num_of_processors]) / num_of_processors
        rospy.set_param("cpu_utilization_for_last_decision", float(cpu_utilization_last_round))
    

def control_callback(control):
    all_cpu_percents = psutil.cpu_percent(percpu=True)[0:num_of_processors]
    cpu_utilization_last_round = sum(all_cpu_percents[0:num_of_processors]) / num_of_processors
    rospy.set_param("cpu_utilization_for_last_decision", float(cpu_utilization_last_round))

    
    
    #collect_hw_counters()
    
    #memory_utilization_last_round = psutil.virtual_memory().percent
    #rospy.set_param("memory_utilization_for_last_decision", float(memory_utilization_last_round) )
    #print(memory_utilization_last_round)

    #collect_sys_prof.publish

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
        #print("pc_vol_ideal" + str(pc_vol_ideal))
        om_to_pl_vol_ideal = results.x[3]
        ppl_vol_ideal = results.x[4]
        # expected time budgets
        om_latency_expected = results.exp_task_times[0]
        om_to_pl_latency_expected = results.exp_task_times[1]
        ppl_latency_expected = results.exp_task_times[2]/pl_to_ppl_ratio
        smoothening_latency_expected = results.exp_task_times[2]/pl_to_ppl_ratio
        #print("$$$$$$$$$$$$$$$$ optimizer succeeded with status " + str(failure_status))
    else:
        rospy.set_param("optimizer_failure_status", failure_status)
        rospy.set_param("optimizer_succeeded", False)
        rospy.set_param("log_control_data", False)
        rospy.set_param("new_control_data", True)  # Important: set this one last to ensure all other knobs/vars are set
        #print("====================optimizer failed with status " + str(failure_status))
        return

    rospy.set_param("optimizer_succeeded", True)
    rospy.set_param("log_control_data", True)
    ee_latency_expected = om_latency_expected + om_to_pl_latency_expected + pl_to_ppl_ratio*ppl_latency_expected + back_end_latency 
    # set the knobs
    rospy.set_param("pc_res", float(pc_res))
    #rospy.set_param("pc_res", float(.3))
    rospy.set_param("pc_vol_ideal", float(pc_vol_ideal))
    #rospy.set_param("pc_vol_ideal", 78000)
    rospy.set_param("om_to_pl_res", float(om_to_pl_res))
    rospy.set_param("om_to_pl_vol_ideal", float(om_to_pl_vol_ideal))
    rospy.set_param("ppl_vol_ideal", float(ppl_vol_ideal))
    rospy.set_param("optimizer_failure_status", failure_status)
    # time budgets (expected with the above knob settings)
    #print("printin the S_A_time_buget" + str(control.internal_states.sensor_to_actuation_time_budget_to_enforce))
    rospy.set_param("sensor_to_actuation_time_budget_to_enforce", control.internal_states.sensor_to_actuation_time_budget_to_enforce)
    rospy.set_param("om_latency_expected", float(om_latency_expected))
    rospy.set_param("om_to_pl_latency_expected", float(om_to_pl_latency_expected))
    rospy.set_param("ppl_latency_expected", float(ppl_latency_expected))
    rospy.set_param("smoothening_latency_expected", float(smoothening_latency_expected))
    rospy.set_param("ee_latency_expected", float(ee_latency_expected))

    rospy.set_param("x_coord_while_budgetting", float(control.internal_states.drone_point_while_budgetting.x))
    rospy.set_param("y_coord_while_budgetting", float(control.internal_states.drone_point_while_budgetting.y))
    rospy.set_param("z_coord_while_budgetting", float(control.internal_states.drone_point_while_budgetting.z))
    rospy.set_param("vel_mag_while_budgetting", float(control.internal_states.drone_point_while_budgetting.vel_mag))
    rospy.set_param("closest_unknown_distance", float(control.internal_states.closest_unknown_distance))



    rospy.set_param("new_control_data", True)  # Important: set this one last to ensure all other knobs/vars are set


    """ 
    rospy.set_param("pc_res", .15)
    rospy.set_param("pc_vol_ideal", 8000)
    rospy.set_param("om_to_pl_res", .15)
    rospy.set_param("om_to_pl_vol_ideal", 200000)
    rospy.set_param("ppl_vol_ideal", 40001)
    rospy.set_param("new_control_data", True)
    """

num_of_processors  = 0
if __name__ == '__main__':
    global num_of_processors 
    global om_popt, om_pl_popt, pp_pl_popt,typical_model
    global distance_to_goal, gap_map_gap, use_gap_map, motivation_section,r_max_queue
    
    result_folder = os.getenv('base_dir') + "/src/MAV_apps/common/run_time/src/knob_performance_modeling_all/data_1"
    om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol, pp_pl_response_time_measured = collect_data(result_folder)
    # colecting models model 
    om_popt, om_pl_popt, pp_pl_popt, typical_model = roborun_model_gen(om_res, om_vol, om_response_time_measured,
                                                                       om_pl_res, om_pl_vol,
                                                                       om_pl_response_time_measured, pp_pl_res,
                                                                       pp_pl_vol,
                                                                       pp_pl_response_time_measured)  # for error calculation
    #op_obj = Opt()
    #opt_obj.opt()
    rospy.init_node('runtime_thread_python', anonymous=True)
    rospy.Subscriber("control_to_pyrun", control, control_callback)
    rospy.Subscriber("control_to_crun", control, control_callback_for_crun)

    time.sleep(2)
    rate = rospy.Rate(10) # 10hz
    rt_max = rospy.get_param("max_time_budget")
    drone_radius = rospy.get_param("planner_drone_radius")
    num_of_processors = rospy.get_param("number_of_processors")
    #mapping.map_processes(num_of_processors)
    v_max = max(rospy.get_param("om_to_pl_vol_ideal_max"), rospy.get_param("ppl_vol_ideal_max"))
    v_min = rospy.get_param("pc_vol_ideal_min")
    budgetting_mode = rospy.get_param("budgetting_mode")
    r_min_static = rospy.get_param("pc_res_max") # we have actually swapped these mistakenly, i.e., chosen min for max. but launch file is correct and python is incorrect
    r_steps = rospy.get_param("num_of_res_to_try") # we have actually swapped these mistakenly, i.e., chosen min for max. but launch file is correct and python is incorrect
    r_max_static = (2 ** r_steps) * r_min_static
    use_gap_map = rospy.get_param("use_gap_map")
    rospy.set_param("pyrun_rcvd_models", True) # this is a signal to everyone else to start their run
   
    while not rospy.is_shutdown():
        #rt_max = rospy.get_param("max_time_budget")
        drone_radius = rospy.get_param("planner_drone_radius")
        distance_to_goal = rospy.get_param("distance_to_goal")
        gap_map_gap = rospy.get_param("gap_map_gap")
        motivation_section = rospy.get_param("motivation_section")
        if motivation_section:
            r_max_queue = SmartQueue(1)
        #failure_ctr_for_pc_res = rospy.get_param("failure_ctr_for_pc_res")
        
        rate.sleep()
