import matplotlib.pyplot as plt
import json
import sys
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.mplot3d import Axes3D 
sys.path.append('../../common_utils')
from data_parsing import *
import seaborn as sns
col = sns.color_palette("Paired", 111111)
stage_of_interests_to_pick_from = ["pc_om", "om_to_pl", "pp_pl", "pc_om_estimation"]

hw_sampling_method = "decision_based"

# which stage are you trying to plot
stage_of_interest = "pp_pl" # pick form ["om_to_pl", "pc_om", "pp_pl", "pc_om_estimation]

assert stage_of_interest in stage_of_interests_to_pick_from

result_folder = "./data_1/system_results"
input_file_name_static = "static/static_stats.json"
input_filepath_static = result_folder + "/" + input_file_name_static

input_file_name_dynamic = "dynamic/dynamic_stats.json"
input_filepath_dynamic = result_folder + "/" + input_file_name_dynamic


time_sampling_data_file = "./data_1/stats_mu.json"

# data to collect
metrics_to_collect_easy = []
metrics_to_collect_hard = ["depthToPCConversionLatency",
"runDiagnosticsLatency",
"runTimeLatency",
"sequencerLatency",
"PCFilteringLatency",
"PCtoOMComOHLatency",
"PCDeserializationLatency",
"PCtoOMTotalLatency",
"OMFilterOutOfRangeLatency",
"insertScanLatency",
"OMFilteringLatency",
"OMSerializationLatency",
"OMtoPlComOHLatency",
"OMDeserializationLatency",
"OMtoPlTotalLatency",
"smoothening_latency",
"ppl_latency",
"pl_to_ft_totalLatency",
"ee_latency",
"blind_latency",
"cpu_utilization_for_last_decision",
"planning_success_ctr",
"traj_gen_failure_ctr",
"planning_ctr",
"decision_ctr",
"runtime_failure_ctr",
"octomap_volume_integrated",
"om_to_pl_volume",
"ppl_volume",
"time_cmd_received",
"pc_to_om_datamovement",
"om_to_pl_datamovement",
"PERF_COUNT_HW_INSTRUCTIONS",
"LLC_REFERENCES",
"CACHE_REFERENCES",
"x_coord_while_budgetting",
"y_coord_while_budgetting",
"z_coord_while_budgetting",
"sensor_to_actuation_time_budget_to_enforce",
"closest_unknown_distance",
"obs_dist_statistics_min",
"vel_mag_while_budgetting",
"pc_res",
"om_to_pl_res",
"gap_statistics_avg"
]

# parse  data

result_dict_static = parse_stat_file_flattened(input_filepath_static, metrics_to_collect_easy, metrics_to_collect_hard)
result_dict_static["pl_to_ft_totalLatency"] =  [0] + result_dict_static["pl_to_ft_totalLatency"][1:]
result_dict_static["cpu_utilization_for_last_decision"] = result_dict_static["cpu_utilization_for_last_decision"][1:] + [100]
result_dict_static["PERF_COUNT_HW_INSTRUCTIONS"] = result_dict_static["PERF_COUNT_HW_INSTRUCTIONS"][1:]+ [100]
result_dict_static["LLC_REFERENCES"] = result_dict_static["LLC_REFERENCES"][1:]+ [100]
result_dict_static["CACHE_REFERENCES"] = result_dict_static["CACHE_REFERENCES"][1:]+ [100]


result_dict_dynamic = parse_stat_file_flattened(input_filepath_dynamic, metrics_to_collect_easy, metrics_to_collect_hard)
result_dict_dynamic["pl_to_ft_totalLatency"] =  [0] + result_dict_dynamic["pl_to_ft_totalLatency"][1:]
result_dict_dynamic["cpu_utilization_for_last_decision"] = result_dict_dynamic["cpu_utilization_for_last_decision"][1:] + [100]
result_dict_dynamic["PERF_COUNT_HW_INSTRUCTIONS"] = result_dict_dynamic["PERF_COUNT_HW_INSTRUCTIONS"][1:]+ [100]
result_dict_dynamic["LLC_REFERENCES"] = result_dict_dynamic["LLC_REFERENCES"][1:]+ [100]
result_dict_dynamic["CACHE_REFERENCES"] = result_dict_dynamic["CACHE_REFERENCES"][1:]+ [100]



mode_result_dict = {}
mode_result_dict["static"] = result_dict_static
mode_result_dict["dynamic"] = result_dict_dynamic


labels = ["depthToPCConversionLatency", "runDiagnosticsLatency", "runTimeLatency", "sequencerLatency", "PCFilteringLatency", "PCtoOMTotalLatency", "OMFilterOutOfRangeLatency",   "insertScanLatency", "OMFilteringLatency", 
        "OMtoPlTotalLatency", "ppl_latency", "smoothening_latency", "pl_to_ft_totalLatency"]

y_lim = 0
for mode in mode_result_dict.keys():
    y_lim = max(y_lim, max(mode_result_dict[mode]["ee_latency"]))
for mode in mode_result_dict.keys():
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])
    fig, ax = plt.subplots()
    ax.stackplot(x, *data_list, labels=labels, colors = col)
    ax.legend(loc='best')
    ax.set_ylim((0,y_lim))
    output_file = "time_breakdown" 
    plt.xlabel("mission progression (s)")
    plt.ylabel("sensor to act latency(s)")
    fig.savefig(result_folder+"/"+output_file + "_"+mode +"_SA.png")
    plt.close(fig)


labels = ["depthToPCConversionLatency", "runDiagnosticsLatency", "runTimeLatency", "sequencerLatency", "PCFilteringLatency", "OMFilterOutOfRangeLatency",  "insertScanLatency", "OMFilteringLatency", 
        "ppl_latency", "smoothening_latency"]

y_lim = 0
for mode in mode_result_dict.keys():
  for el  in labels:
    y_lim = max(y_lim, max(mode_result_dict[mode][el]))
for mode in mode_result_dict.keys():
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])
    fig, ax = plt.subplots()
    ax.stackplot(x, *data_list, labels=labels, colors = col)
    ax.legend(loc='best')
    ax.set_ylim((0,y_lim))
    output_file = "computation_breakdown" 
    plt.xlabel("mission progression (s)")
    plt.ylabel("Computation Break down (s)")
    fig.savefig(result_folder+"/"+output_file + "_"+mode +"_SA.png")
    plt.close(fig)



labels = ["PCtoOMTotalLatency", "OMtoPlTotalLatency", "pl_to_ft_totalLatency"]

y_lim = 0
for mode in mode_result_dict.keys():
  for el  in labels:
    y_lim = max(y_lim, max(mode_result_dict[mode][el]))
for mode in mode_result_dict.keys():
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])
    fig, ax = plt.subplots()
    ax.stackplot(x, *data_list, labels=labels, colors = col)
    ax.legend(loc='best')
    ax.set_ylim((0, y_lim))
    output_file = "communication_breakdown" 
    plt.xlabel("mission progression (s)")
    plt.ylabel("Communication Breakdown (s)")
    fig.savefig(result_folder+"/"+output_file + "_"+mode +"_SA.png")
    plt.close(fig)





# response time
labels = ["ee_latency", "blind_latency"]
y_lim = 0
for mode in mode_result_dict.keys():
    y_lim = max(y_lim, 2*max(mode_result_dict[mode]["ee_latency"]))
for mode in mode_result_dict.keys():
    data_list = []
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    for el in labels:
        data_list.append(mode_result_dict[mode][el][1:])

    fig, ax = plt.subplots()
    ax.stackplot(x[1:], *data_list, labels=labels, colors = col)
    ax.legend(loc='best')
    ax.set_ylim((0,y_lim))
    output_file = "response_time_breakdown"
    plt.xlabel("mission progression (s)")
    plt.ylabel("response time components (s)")
    plt.yticks(np.arange(0, 5, .2))
    fig.savefig(result_folder+"/"+output_file + "_" +mode+ "_SA.png")
    plt.close(fig)


# utilization
y_lim_2 = 0
for mode in mode_result_dict.keys():
    y_lim_2 = max(y_lim_2, max(mode_result_dict[mode]["ee_latency"]))

y_lim_1 = 0
for mode in mode_result_dict.keys():
    y_lim_1 = max(y_lim_1, max(mode_result_dict[mode]["cpu_utilization_for_last_decision"]))

labels = ["cpu_utilization_for_last_decision"]
for mode in mode_result_dict.keys():
    data_list = []
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    for el in labels:
        data_list.append(mode_result_dict[mode][el])

    fig, axs = plt.subplots(2)
    axs[1].set_yscale('linear')
    axs[1].plot(x, mode_result_dict[mode]["ee_latency"])
    axs[1].set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
    axs[1].legend(loc='best')
    axs[1].set_ylim((0,y_lim_2))

    axs[0].plot(x, *data_list, label="cpu_utilization ")
    axs[0].set(xlabel="mission  progression (s)", ylabel="mission progression (s) ")
    axs[0].legend(loc='best')
    axs[0].set_ylim((0, y_lim_1))
    output_file = "utilization" 
    fig.savefig(result_folder+"/"+output_file + "_" + mode + "_SA.png")
    plt.close(fig)


y_lim_2 = 0
for mode in mode_result_dict.keys():
    y_lim_2 = max(y_lim_2, max(mode_result_dict[mode]["ee_latency"]))

y_lim_1 = 0
for mode in mode_result_dict.keys():
    y_lim_1 = max(y_lim_1, max(mode_result_dict[mode]["PERF_COUNT_HW_INSTRUCTIONS"]))

labels = ["PERF_COUNT_HW_INSTRUCTIONS"]
#  HW INSTRUCTIONS
for mode in mode_result_dict.keys():
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])

    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    fig, axs = plt.subplots(2)
    #axs[1].plot(x, ee_latency=" end_to_end_latency")
    axs[1].set_yscale('linear')
    axs[1].plot(x, mode_result_dict[mode]["ee_latency"])
    axs[1].set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
    axs[1].legend(loc='best')
    axs[1].set_ylim((0,y_lim_2))

    axs[0].plot(x, *data_list, label="HW instructions (millions)")
    axs[0].set(xlabel="mission  progression (s)", ylabel="HW instructions (millions) ")
    axs[0].legend(loc='best')
    axs[0].set_ylim((0, y_lim_1))
    output_file = "hw_instructions" 
    fig.savefig(result_folder+"/"+output_file + "_" + mode + "_SA.png")
    plt.close(fig)


labels = ["PERF_COUNT_HW_INSTRUCTIONS"]
#  HW INSTRUCTIONS
for mode in mode_result_dict.keys():
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])

    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    fig, axs = plt.subplots()
    axs.plot(x, *data_list, label="HW instructions (millions)")
    axs.set(xlabel="mission  progression (s)", ylabel="HW instructions (millions) ")
    axs.legend(loc='best')
    axs.set_ylim((0, y_lim_1))
    output_file = "hw_instructions_only" 
    fig.savefig(result_folder+"/"+output_file + "_" + mode + "_SA.png")
    plt.close(fig)








# volume chart
y_lim_2 = 0
for mode in mode_result_dict.keys():
    y_lim_2 = max(y_lim_2, max(mode_result_dict[mode]["ee_latency"]))


labels = ["octomap_volume_integrated", "om_to_pl_volume", "ppl_volume"]
y_lim_1 = 0
for mode in mode_result_dict.keys():
    for el  in labels:
        y_lim_1 = max(y_lim_1, max(mode_result_dict[mode][el]))

for mode in mode_result_dict.keys():
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])

    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    fig, axs = plt.subplots(2)
    axs[1].set_yscale('linear')
    axs[1].plot(x, mode_result_dict[mode]["ee_latency"], label=" end_to_end_latency")
    axs[1].set(xlabel="mission  progression (s)", ylabel="latnecy (s)")
    axs[1].legend(loc='best')
    axs[1].set_ylim((0,y_lim_2))

    for idx, lbl in enumerate(labels):
        axs[0].plot(x, data_list[idx], label=lbl)
        axs[0].set(xlabel="mission  progression (s)", ylabel="time (s)")
        axs[0].set_yscale('log')
        axs[0].legend(loc='best')
    axs[0].set_ylim((40000,y_lim_1))
    output_file = "volume" 
    #plt.show()
    fig.savefig(result_folder+"/"+output_file + "_"+mode+ "_SA.png")
    plt.close(fig)
    #plt.show()


# ------- datamovement chart
y_lim_2 = 0
for mode in mode_result_dict.keys():
    y_lim_2 = max(y_lim_2, max(mode_result_dict[mode]["ee_latency"]))

labels = ["om_to_pl_datamovement", "pc_to_om_datamovement"]
y_lim_1 = 0
for mode in mode_result_dict.keys():
    for el  in labels:
        y_lim_1 = max(y_lim_1, max(mode_result_dict[mode][el]))

for mode in mode_result_dict.keys():
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    fig, axs = plt.subplots(2)
    axs[1].set_yscale('linear')
    axs[1].plot(x, mode_result_dict[mode]["ee_latency"], label=" end_to_end_latency")
    axs[1].set(xlabel="mission  progression (s)", ylabel="latnecy (s)")
    axs[1].legend(loc='best')
    axs[1].set_ylim((0,y_lim_2))
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])

    for idx, lbl in enumerate(labels):
        axs[0].plot(x,[y/1000 for y in data_list[idx]], label=lbl)
    axs[0].set(xlabel="mission  progression (s)", ylabel="dataw movement (KB)")
    axs[0].legend(loc='best')
    output_file = "datamovement" 
    axs[0].set_ylim((0, 1.1*y_lim_1/1000))
    fig.savefig(result_folder+"/"+output_file + "_"+mode + "_SA.png")
    plt.close(fig)




# LLC REFERNCES
for mode in mode_result_dict.keys():
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    fig, axs = plt.subplots(2)
    #axs[1].plot(x, ee_latency=" end_to_end_latency")
    axs[1].set_yscale('linear')
    axs[1].plot(x, mode_result_dict[mode]["ee_latency"])
    axs[1].set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
    axs[1].legend(loc='best')

    labels = ["LLC_REFERENCES"]
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])
    axs[0].plot(x, data_list[0], label=labels[0])
    axs[0].set(xlabel="mission  progression (s)", ylabel="LLC REFERENCES (million) ")
    axs[0].legend(loc='best')
    output_file = "llc_references" 
    fig.savefig(result_folder+"/"+output_file + "_" +mode+ "_SA.png")
    plt.close(fig)


y_lim_1 = 0
for mode in mode_result_dict.keys():
    y_lim_1 = max(y_lim_1, max(mode_result_dict[mode]["CACHE_REFERENCES"]))

# CACHE REFERNCES
for mode in mode_result_dict.keys():
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    fig, axs = plt.subplots()
    labels = ["CACHE_REFERENCES"]
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])
    axs.plot(x, data_list[0], label=labels[0])
    axs.set(xlabel="mission  progression (s)", ylabel="CACHE REFERENCES (million) ")
    axs.legend(loc='best')
    axs.set_ylim((0, 1.1*y_lim_1))
    output_file = "cache_references_only" 
    fig.savefig(result_folder+"/"+output_file + "_" +mode+ "_SA.png")
    plt.close(fig)





for mode in mode_result_dict.keys():
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    fig, axs = plt.subplots(1)
    #axs[1].plot(x, ee_latency=" end_to_end_latency")
    labels = ["pc_res", "om_to_pl_res"]
    data_list = []
    for idx,el in enumerate(labels):
        data_list.append(mode_result_dict[mode][el])
    for idx,el in enumerate(labels):
        axs.plot(x, data_list[idx], label=labels[idx])
        axs.set(xlabel="mission  progression (s)", ylabel="Precision (cm) ")
        axs.legend(loc='best')
        
    output_file = "precision" 
    fig.savefig(result_folder+"/"+output_file + "_" +mode+ "_SA.png")
    plt.close(fig)


# end to end latency
y_lim = 0
for mode in mode_result_dict.keys():
    y_lim = max(y_lim, max(mode_result_dict[mode]["ee_latency"]))

for mode in mode_result_dict.keys():
    x = mode_result_dict[mode]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
    fig, axs = plt.subplots(1)
    #axs[1].plot(x, ee_latency=" end_to_end_latency")
    labels = ["ee_latency"]
    data_list = []
    for el in labels:
        data_list.append(mode_result_dict[mode][el])
    axs.plot(x, data_list[0], label=labels[0])
    axs.set_ylim(0, y_lim) 
    axs.set(xlabel="mission  progression (s)", ylabel="end to end latency (s) ")
    axs.legend(loc='best')
    output_file = "end_to_end_latency" 
    fig.savefig(result_folder+"/"+output_file + "_" +mode+ "_SA.png")
    plt.close(fig)







## CACHE REFERNCES
#fig, axs = plt.subplots(2)
##axs[1].plot(x, ee_latency=" end_to_end_latency")
#axs[1].set_yscale('linear')
#axs[1].plot(x, ee_latency)
#axs[1].set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
#axs[1].legend(loc='best')
#
#time_sampling_dict = []
#if hw_sampling_method == "time_based":
#    with open(time_sampling_data_file) as read_file:
#        time_sampling_dict = json.load(read_file)
#    CACHE_REFERENCES = time_sampling_dict["CACHE_REFERENCES"][1:]+ [100]
#    x = time_sampling_dict["time"]
#else:
#    x = time_cmd_received;#range(0, len(runDiagnosticsLatency))
#axs[0].plot(x, CACHE_REFERENCES, label="CACHE REFERENCES instructions (million)")
#axs[0].set(xlabel="mission  progression (s)", ylabel="CACHE REFERENCES (million) ")
#axs[0].legend(loc='best')
#
#output_file = "cache_references" + "_SA.png"
#fig.savefig(result_folder+"/"+output_file)
#plt.close(fig)
#



# velocity/budget to enforce
x_copy = []
planner_drone_radius = 1.5
sensor_max_range = 25;
sensor_to_actuation_time_budget_to_enforce_copy = []
vel_mag_while_budgetting_copy = []
closest_unknown_distance_copy = []
x = mode_result_dict["dynamic"]["time_cmd_received"];#range(0, len(runDiagnosticsLatency))
for idx, el in enumerate(mode_result_dict["dynamic"]["sensor_to_actuation_time_budget_to_enforce"]):
    if el < .001 or el > 50:
        continue
    else:
        sensor_to_actuation_time_budget_to_enforce_copy.append(el)
        vel_mag_while_budgetting_copy.append(mode_result_dict["dynamic"]["vel_mag_while_budgetting"][idx])
        closest_unknown_distance_copy.append(min(mode_result_dict["dynamic"]["closest_unknown_distance"][idx], sensor_max_range))
        x_copy.append(x[idx])


fig, axs = plt.subplots(2)
#axs[1].plot(x, ee_latency=" end_to_end_latency")
axs[1].set_yscale('linear')
axs[1].plot(x_copy, vel_mag_while_budgetting_copy, marker="o", label="Acutal Dynamic Velocity (m/s)")
axs[1].set(xlabel="Mission  Progression (s)", ylabel=" Velocity(m/s)")
axs[1].legend(loc='best')
axs[1].plot(x_copy, [3.2]*len(vel_mag_while_budgetting_copy), marker="o", label="Assumed Static Velocity (m/s)")
#axs[1].set(xlabel="mission  progression (s)", label = "velocity (m/s)")
axs[1].legend(loc='upper left')



"""
axs[2].set_yscale('linear')
axs[2].plot(x_copy, closest_unknown_distance_copy, label="dynamic visibility", marker="o")
axs[2].plot(x_copy, [8]*len(closest_unknown_distance_copy), label="assumed fix visibility", marker="o")
axs[2].set(xlabel="mission  progression (s)", ylabel="visibility (s) ")
axs[2].legend(loc='best')
"""

axs[0].set_yscale('linear')
axs[0].plot(x_copy, sensor_to_actuation_time_budget_to_enforce_copy, label="Actual Dynamic Time Budget (s)", marker="o")
axs[0].plot(x_copy, [2]*len(sensor_to_actuation_time_budget_to_enforce_copy), label="Assumed Static Time Budget", marker="o")
axs[0].set(xlabel="mission  progression (s)", ylabel="time budget(s) ")
axs[0].legend(loc='upper right')
output_file = "vel_res" + "_SA.png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)





# velocity/budget to enforce
fig, axs = plt.subplots(2)
insertScanLatency_copy = []
obs_dist_statistics_min_copy = []
x_copy = []
planner_drone_radius = 1.5
gap_statistics_avg_copy= []
pc_res_copy = []
min_of_two_space_dist = []
for idx, el in enumerate(mode_result_dict["dynamic"]["insertScanLatency"]):
    if el < .001 or el > 20:
        continue
    else:
        insertScanLatency_copy.append(el)
        obs_dist_statistics_min_copy.append(mode_result_dict["dynamic"]["obs_dist_statistics_min"][idx])
        x_copy.append(x[idx])
        gap_statistics_avg_copy.append(min(mode_result_dict["dynamic"]["gap_statistics_avg"][idx], sensor_max_range))
        pc_res_copy.append(mode_result_dict["dynamic"]["pc_res"][idx])
        min_of_two_space_dist.append(min(min(mode_result_dict["dynamic"]["gap_statistics_avg"][idx], sensor_max_range), abs(mode_result_dict["dynamic"]["obs_dist_statistics_min"][idx])))

#axs[1].plot(x, ee_latency=" end_to_end_latency")
"""
axs[1].set_yscale('linear')
axs[1].plot(x_copy, obs_dist_statistics_min_copy, marker="o", label="dist to closest obstacle")
axs[1].plot(x_copy, [.3]*len(obs_dist_statistics_min_copy), marker="o", label="assumed distance to closest obstacle")
axs[1].set(xlabel="mission  progression (s)", ylabel=" min distance to obst")
axs[1].legend(loc='best')

axs[2].set_yscale('linear')
axs[2].plot(x_copy, gap_statistics_avg_copy, label="average distance between visible obstalces", marker="o")
axs[2].plot(x_copy, [(2+(2*.3))]*len(gap_statistics_avg_copy), label="assumed fix average distance between visible obstacles", marker="o")
axs[2].set(xlabel="mission  progression (s)", ylabel="gap_statistics_avg ")
axs[2].legend(loc='best')

axs[3].set_yscale('linear')
axs[3].plot(x_copy, min_of_two_space_dist, label="", marker="o")
axs[3].set(xlabel="mission  progression (s)", ylabel="min_of_two_space_dist")
axs[3].legend(loc='best')
"""
axs[1].set_yscale('log')
axs[1].plot(x_copy, pc_res_copy, label="Actual Dynamic Precision Demand", marker="o")
axs[1].plot(x_copy, [.6]*len(pc_res_copy), label="Assumed Static Precision Demand (cm)", marker="o")
axs[1].set(xlabel="Mission  Progression (s)", ylabel="Precision Demanded (cm)")
axs[1].legend(loc='upper left')

axs[0].set_yscale('linear')
axs[0].plot(x_copy, insertScanLatency_copy, label="Actual Dynamic Response Time (s) ", marker="o")
axs[0].plot(x_copy, [.7]*len(insertScanLatency_copy), label="Assumed Static Response Time (s)", marker="o")
axs[0].set(xlabel="Mission  Progression (s)", ylabel=" Reponse Time (s)")
axs[0].legend(loc='upper right')

#axs[2].plot(x, pc_res, label="pc_res")
#axs[2].set(xlabel="mission  progression (s)", ylabel=" pc res")
#axs[2].legend(loc='best')
#
output_file = "space_res" + "_SA.png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)


#axs.set_yscale('linear')
#axs.plot(x, ee_latency)
#axs.set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
#axs.legend(loc='best')
#output_file = "ee_latency" + "_SA.png"
#fig.savefig(result_folder+"/"+output_file)
#plt.close(fig)




# ------- making the piechart
ax.set_yscale('linear')
group_names=['replanning', 'assessing']
group_size=[int(mode_result_dict["dynamic"]["planning_ctr"][0]), int(mode_result_dict["dynamic"]["decision_ctr"][0]) - int(mode_result_dict["dynamic"]["planning_ctr"][0])]
subgroup_names=['runtime failure cnt', 'traj gen failure cnt', 'planning success cnt', 'assessing']
subgroup_size= [int(mode_result_dict["dynamic"]['runtime_failure_ctr'][0]), int(mode_result_dict["dynamic"]['traj_gen_failure_ctr'][0]), int(mode_result_dict["dynamic"]['planning_success_ctr'][0]),
        int(mode_result_dict["dynamic"]["decision_ctr"][0]) - int(mode_result_dict["dynamic"]["planning_ctr"][0])]
# Create colors
a, b, c, d,e =[plt.cm.Blues, plt.cm.Reds, plt.cm.Greens, plt.cm.Purples, plt.cm.Greys]

# First Ring (outside)
fig, ax = plt.subplots()
ax.axis('equal')
#mypie, _, _= ax.pie(group_size, radius=1.3, labels=group_names, colors=[d(0.6), a(0.6)], autopct=lambda p:'{:d}'.format(int(p*sum(group_size)/100))) 
mypie, _, _= ax.pie(group_size, radius=1.3, autopct=lambda p:'{:d}'.format(int(p*sum(group_size)/100))) 
plt.setp(mypie, width=0.3, edgecolor='white')
leg1 = ax.legend(mypie, group_names, loc="best")
          
# Second Ring (Inside)
#mypie2, _ = ax.pie(subgroup_size, radius=1.3-0.3, labels=subgroup_names, labeldistance=0.7, colors=[a(0.5), a(0.4), a(0.3), b(0.5), b(0.4), c(0.6), c(0.5), c(0.4), c(0.3), c(0.2)])
mypie2, _,_ = ax.pie(subgroup_size, radius=1.3-0.3,  autopct=lambda p:'{:d}'.format(int(p*sum(group_size)/100)))

ax.legend(mypie2, subgroup_names,
          loc="best"
          )
ax.add_artist(leg1)

plt.setp( mypie2, width=0.4, edgecolor='white')
plt.margins(0,0)
plt.title(label="decision making breakdown")
output_file = "decision_pie_chart" + "_SA.png"
fig.savefig(result_folder+"/"+output_file)
# show it
#plt.show()





