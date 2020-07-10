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
from operator import setitem
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
metrics_to_collect_easy = ["flight_time",  "experiment_number"]
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
"mission_status"
]

# parse  data
result_dict = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard, "separate")


"""
flight_time = result_dict["flight_time"]
mission_status = result_dict["mission_status"]

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


pc_to_om_datamovement = result_dict["pc_to_om_datamovement"]
om_to_pl_datamovement = result_dict["om_to_pl_datamovement"]
"""

# clean up some of the results
for metric in metrics_to_collect_easy + metrics_to_collect_hard:
    if metric in (["PERF_COUNT_HW_INSTRUCTIONS", "LLC_REFERENCES", "CAHCE_REFERENCES",
                  "cpu_utilization_for_last_decision"]):
        for id in range(0, len(result_dict[metric])):
            setitem(result_dict[metric], id, result_dict[metric][id][1:] + [100])
    if metric in (["pl_to_ft_totalLatency"]):
        for id in range(0, len(result_dict[metric])):
            setitem(result_dict[metric], id, [0] + result_dict[metric][id][1:])

# getting average per run
result_dict_avg = {}
result_dict_std = {}
for metric in metrics_to_collect_easy + metrics_to_collect_hard:
    result_dict_avg[metric] = []
    result_dict_std[metric] = []

for metric in metrics_to_collect_easy + metrics_to_collect_hard:
    results = result_dict[metric]
    for el in results:
        result_dict_avg[metric].append(get_avg(el))
        result_dict_std[metric].append(get_std(el))


# getting average over all
result_dict_avg_avg = {}
result_dict_avg_std = {}
for metric in metrics_to_collect_easy + metrics_to_collect_hard:
    result_dict_avg_avg[metric] = []
    result_dict_avg_std[metric] = []

for metric in metrics_to_collect_easy + metrics_to_collect_hard:
    result_dict_avg_avg[metric].append(get_avg(result_dict_avg[metric]))
    result_dict_avg_std[metric].append(get_avg(result_dict_std[metric]))

print(sum(result_dict[flight_time]/len(flight_time)))
print(sum(result_dict[flight_time]/len(flight_time)))









"""
# CACHE REFERNCES
fig, axs = plt.subplots(2)
#axs[1].plot(x, end_to_end_latency=" end_to_end_latency")
axs[1].set_yscale('linear')
axs[1].plot(x, end_to_end_latency)
axs[1].set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
axs[1].legend(loc='best')

time_sampling_dict = []
if hw_sampling_method == "time_based":
    with open(time_sampling_data_file) as read_file:
        time_sampling_dict = json.load(read_file)
    CACHE_REFERENCES = time_sampling_dict["CACHE_REFERENCES"][1:]+ [100]
    x = time_sampling_dict["time"]
else:
    x = time_cmd_received;#range(0, len(runDiagnosticsLatency))
axs[0].plot(x, CACHE_REFERENCES, label="CACHE REFERENCES instructions (million)")
axs[0].set(xlabel="mission  progression (s)", ylabel="CACHE REFERENCES (million) ")
axs[0].legend(loc='best')

output_file = "cache_references" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)
"""




