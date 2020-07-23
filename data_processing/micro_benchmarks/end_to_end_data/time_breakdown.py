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

result_folder = "./data_1"
input_file_name = "stats.json"
input_filepath = result_folder + "/" + input_file_name
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
"obs_dist_statistics_min",
"vel_mag_while_budgetting"
]

# parse  data
result_dict = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

depthToPCConversionLatency = result_dict["depthToPCConversionLatency"]
runDiagnosticsLatency = result_dict["runDiagnosticsLatency"]
runTimeLatency = result_dict[ "runTimeLatency"]
sequencerLatency = result_dict["sequencerLatency"]
PCFilteringLatency = result_dict["PCFilteringLatency"]
PCtoOMComOHLatency = result_dict["PCtoOMComOHLatency"]
PCDeserializationLatency = result_dict["PCDeserializationLatency"]
PCtoOMTotalLatency = result_dict["PCtoOMTotalLatency"]
OMFilterOutOfRangeLatency = result_dict["OMFilterOutOfRangeLatency"]
insertScanLatency = result_dict["insertScanLatency"]
OMFilteringLatency = result_dict["OMFilteringLatency"]
OMSerializationLatency = result_dict["OMSerializationLatency"]
OMtoPlComOHLatency = result_dict["OMtoPlComOHLatency"]
OMDeserializationLatency = result_dict["OMDeserializationLatency"]
OMtoPlTotalLatency =  result_dict["OMtoPlTotalLatency"]
ppl_latency= result_dict["ppl_latency"]
smoothening_latency=  result_dict["smoothening_latency"]
pl_to_ft_total_latency =  [0] + result_dict["pl_to_ft_totalLatency"][1:]

end_to_end_latency =  result_dict["ee_latency"]
cpu_utilization_for_last_decision = result_dict["cpu_utilization_for_last_decision"][1:] + [100]
octomap_volume_integrated = result_dict["octomap_volume_integrated"]
om_to_pl_volume = result_dict["om_to_pl_volume"]
ppl_volume = result_dict["ppl_volume"]
time_cmd_received = result_dict["time_cmd_received"]
PERF_COUNT_HW_INSTRUCTIONS = result_dict["PERF_COUNT_HW_INSTRUCTIONS"][1:]+ [100]
LLC_REFERENCES = result_dict["LLC_REFERENCES"][1:]+ [100]
CACHE_REFERENCES = result_dict["CACHE_REFERENCES"][1:]+ [100]

x_coord_while_budgetting = result_dict["x_coord_while_budgetting"]
y_coord_while_budgetting = result_dict["y_coord_while_budgetting"]
z_coord_while_budgetting = result_dict["z_coord_while_budgetting"]
sensor_to_actuation_time_budget_to_enforce = result_dict["sensor_to_actuation_time_budget_to_enforce"]
obs_dist_statistics_min = result_dict["obs_dist_statistics_min"]
vel_mag_while_budgetting = result_dict["vel_mag_while_budgetting"]


pc_to_om_datamovement = result_dict["pc_to_om_datamovement"]
om_to_pl_datamovement = result_dict["om_to_pl_datamovement"]

x = time_cmd_received;#range(0, len(runDiagnosticsLatency))
labels = ["depthToPCConversionLatency", "runDiagnosticsLatency", "runTimeLatency", "sequencerLatency", "PCFilteringLatency", "PCtoOMTotalLatency", "OMFilterOutOfRangeLatency",   "insertScanLatency", "OMFilteringLatency", 
        "OMtoPlTotalLatency", "ppl_latency", "smoothening_latency", "pl_to_ft_total_latency"]

data_list = [depthToPCConversionLatency, runDiagnosticsLatency, runTimeLatency, sequencerLatency, PCFilteringLatency, PCtoOMTotalLatency, OMFilterOutOfRangeLatency, insertScanLatency,
        OMFilteringLatency,
        OMtoPlTotalLatency, ppl_latency, smoothening_latency, pl_to_ft_total_latency]
fig, ax = plt.subplots()
ax.stackplot(x, *data_list, labels=labels, colors = col)
ax.legend(loc='best')
output_file = "time_breakdown" + ".png"
plt.xlabel("mission progression (s)")
plt.ylabel("sensor to act latency(s)")
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)



# for PC
# utilization
fig, axs = plt.subplots(5)
#axs[1].plot(x, end_to_end_latency=" end_to_end_latency")
axs[4].set_yscale('linear')
axs[4].plot(x, depthToPCConversionLatency, label="depthToPCConversionLatency")
axs[4].set(xlabel="mission  progression (s)", ylabel=" depthToPCConversionLatency(s)")
axs[4].legend(loc='best')

axs[3].plot(x, runDiagnosticsLatency, label="runDiagnosticsLatency")
axs[3].set(xlabel="mission  progression (s)", ylabel="runDiagnosticsLatency (s) ")
axs[3].legend(loc='best')

axs[2].plot(x, runTimeLatency , label="runTimeLatency")
axs[2].set(xlabel="mission  progression (s)", ylabel="runTimeLatency (s) ")
axs[2].legend(loc='best')

axs[1].plot(x, sequencerLatency , label="sequencerLatency")
axs[1].set(xlabel="mission  progression (s)", ylabel="sequencerLatency (s) ")
axs[1].legend(loc='best')

axs[0].plot(x, PCFilteringLatency, label="PCFilteringLatency")
axs[0].set(xlabel="mission  progression (s)", ylabel="PCFilteringLatency (s) ")
axs[0].legend(loc='best')

output_file = "PC_time_break_down_separated" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)


# for OM
fig, axs = plt.subplots(4)
axs[3].plot(x, PCtoOMComOHLatency, label="PCtoOMComOHLatency")
axs[3].set(xlabel="mission  progression (s)", ylabel="PCtoOMComOHLatency (s) ")
axs[3].legend(loc='best')

axs[2].plot(x, OMFilterOutOfRangeLatency, label="OMFilterOutOfRangeLatency")
axs[2].set(xlabel="mission  progression (s)", ylabel="OMFilterOutOfRangeLatency(s) ")
axs[2].legend(loc='best')

axs[1].plot(x, insertScanLatency, label="insertScanLatency")
axs[1].set(xlabel="mission  progression (s)", ylabel="insertScanLatency(s) ")
axs[1].legend(loc='best')

axs[0].plot(x, OMFilteringLatency, label="OMFilteringLatency")
axs[0].set(xlabel="mission  progression (s)", ylabel="OMFilteringLatency(s) ")
axs[0].legend(loc='best')

output_file = "OM_time_break_down_separated" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)


# for PL
fig, axs = plt.subplots(3)
axs[2].plot(x, ppl_latency, label="ppl_latency")
axs[2].set(xlabel="mission  progression (s)", ylabel="ppl_latency(s) ")
axs[2].legend(loc='best')

axs[1].plot(x, smoothening_latency, label="smoothening_latency")
axs[1].set(xlabel="mission  progression (s)", ylabel="smoothening_latency(s) ")
axs[1].legend(loc='best')

axs[0].plot(x, pl_to_ft_total_latency, label="pl_to_ft_totalLatency")
axs[0].set(xlabel="mission  progression (s)", ylabel="pl_to_ft_total_latency(s) ")
axs[0].legend(loc='best')

output_file = "PL_time_break_down_separated" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)








