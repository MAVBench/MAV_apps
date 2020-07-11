import numpy as np
import sys
sys.path.append('..')
sys.path.append('../../../../../data_processing/common_utils')
#from calc_sampling_time import *
from utils import *
from model_generation.model_gen import *
from model import Model
from data_parser import DataParser
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

   

def calculate_static_values():
    #filtering_time = .45
    #run_diagnostics = .15
    #pc_to_om_overhead = .6
    #run_time_budget = .1 
    drone_radius = 1
    drone_radius_when_hovering =  drone_radius  # set it equal to each other to avoid sub_vmax optimization
    # collect data 
    result_folder = "../../knob_performance_modeling_all/data_1" 
    om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol, pp_pl_response_time_measured = collect_data(result_folder)

    # colecting models model 
    om_popt, om_pl_popt, pp_pl_popt, typical_model = roborun_model_gen(om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol,
            pp_pl_response_time_measured)	 # for error calculation


    om_res_desired = om_pl_res_desired=pp_pl_res_desired = .3
    om_vol_desired = 50000
    om_pl_vol_desired = 3*50000
    pp_pl_vol_desired = 3*50000
    visibility_avg = 12.5 
    
    pc_filtering = .210
    sequencer_latency = .01
    depth_img_conversion = .07
    depth_img_latency = .1
    follow_trajectory_latency = .05

    om_latency = calculate_fitted_value(om_popt, om_res_desired, om_vol_desired, typical_model)
    om_pl_latency = calculate_fitted_value(om_pl_popt, om_pl_res_desired, om_pl_vol_desired, typical_model)
    pp_pl_latency = calculate_fitted_value(pp_pl_popt, pp_pl_res_desired, pp_pl_vol_desired, typical_model)


    max_time_budget = om_latency + om_pl_latency + 2*pp_pl_latency + pc_filtering + sequencer_latency + depth_img_conversion + depth_img_latency + follow_trajectory_latency

    #max_time_budget = pc_filtering + sequencer_latency + depth_img_conversion + depth_img_latency + follow_trajectory_latency + .1 + .1 + .1
    
    velocity_to_budget_on = calc_v_max(max_time_budget, visibility_avg - drone_radius, .1439, .8016)
    print("----om_latency:" + str(om_latency))
    print("--------om_pl_latency:" + str(om_pl_latency))
    print("-------------pp_pl_latency" + str(3*pp_pl_latency))
    print("-----------------controlled_latency" + str(om_latency + om_pl_latency + 2*pp_pl_latency))
    print("----------------------max_budget" + str(max_time_budget))
    print("----------------------v_max" + str(velocity_to_budget_on))

    json_output_file = open("static_knobs.json", "w")  
    json_output_file.write("{\n")
    json_output_file.write("\t\t\"max_time_budget\": "+ str(max_time_budget)+",\n");
    json_output_file.write("\t\t\"om_latency_expected\": " + str(om_latency)+",\n");
    json_output_file.write("\t\t\"om_to_pl_latency_expected\": " + str(om_pl_latency)+",\n");
    json_output_file.write("\t\t\"ppl_latency_expected\": " + str(pp_pl_latency)+",\n");
    json_output_file.write("\t\t\"velocity_to_budget_on\": " + str(velocity_to_budget_on)+",\n");
    json_output_file.write("\t\t\"v_max\": " + str(velocity_to_budget_on)+",\n");
    json_output_file.write("\t\t\"pc_res_max\": " + str(om_res_desired)+",\n");
    json_output_file.write("\t\t\"pc_vol_ideal_max\": " + str(om_vol_desired)+",\n");
    json_output_file.write("\t\t\"om_to_pl_res_max\": " + str(om_pl_res_desired)+",\n");
    json_output_file.write("\t\t\"om_to_pl_vol_ideal_max\": " + str(om_pl_vol_desired)+",\n");
    json_output_file.write("\t\t\"ppl_vol_ideal_max\": " + str(pp_pl_vol_desired)+", \n");
    json_output_file.write("\t\t\"budgetting_mode\": " + str("\"static\"")+",\n");
    json_output_file.write("\t\t\"use_pyrun\": " + str("false")+",\n");
    json_output_file.write("\t\t\"knob_performance_modeling\": " + str("false")+", \n");
    json_output_file.write("\t\t\"planner_drone_radius\": " + str(drone_radius)+", \n");
    json_output_file.write("\t\t\"capture_size\": " + str(1)+", \n");
    json_output_file.write("\t\t\"DEBUG_VIS\": " + str("false")+", \n");
    json_output_file.write("\t\t\"design_mode\": " + str("\"pipelined\"")+", \n");
    json_output_file.write("\t\t\"planner_drone_radius_when_hovering\": " + str(drone_radius_when_hovering)+"\n");
    json_output_file.write("}")
    json_output_file.close()


    script_output_file = open("script1.bash", "w")  
    script_output_file.write("#!/bin/bash\n")
    script_output_file.write("roslaunch package_delivery package_delivery.launch ")
    script_output_file.write("max_time_budget:="+ str(max_time_budget)+" ");
    script_output_file.write("om_latency_expected:=" + str(om_latency) + " ");
    script_output_file.write("om_to_pl_latency_expected:=" + str(om_pl_latency) + " ");
    script_output_file.write("ppl_latency_expected:=" + str(pp_pl_latency)+" ");
    script_output_file.write("velocity_to_budget_on:=" + str(velocity_to_budget_on)+" ");
    script_output_file.write("budgetting_mode:=" + str("\"static\"")+" ");
    script_output_file.write("v_max:=" + str(velocity_to_budget_on)+" ");
    script_output_file.write("use_pyrun:=" + str("false") + " ")
    script_output_file.write("knob_performance_modeling:=" + str("false")+" ")
    script_output_file.write("pc_res_max:=" + str(om_res_desired)+" ");
    script_output_file.write("pc_vol_ideal_max:=" + str(om_vol_desired)+" ");
    script_output_file.write("om_to_pl_res_max:=" + str(om_pl_res_desired)+" ");
    script_output_file.write("om_to_pl_vol_ideal_max:=" + str(om_pl_vol_desired)+" ");
    script_output_file.write("ppl_vol_ideal_max:="+ str(pp_pl_vol_desired) + " ");
    script_output_file.write("planner_drone_radius:="+ str(drone_radius) + " ");
    script_output_file.write("capture_size:="+ str(1) + " ");
    script_output_file.write("design_mode:="+ str("\"pipelined\"") + " ");
    script_output_file.write("DEBUG_VIS:="+ str("false") + " ");
    script_output_file.write("planner_drone_radius_when_hovering:="+ str(drone_radius_when_hovering) + " ");
    script_output_file.close()





## May want to convert this to using absolute paths to make life easier ##
if __name__ == '__main__':
    calculate_static_values() 
