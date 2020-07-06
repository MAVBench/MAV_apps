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
col = sns.color_palette("Paired", 11)
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
"CACHE_REFERENCES"
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

x = time_cmd_received;#range(0, len(runDiagnosticsLatency))
labels = ["runDiagnosticsLatency", "runTimeLatency", "sequencerLatency", "PCFilteringLatency", "PCtoOMTotalLatency", "OMFilterOutOfRangeLatency",   "insertScanLatency", 
        "OMtoPlTotalLatency", "ppl_latency", "smoothening_latency"]

data_list = [runDiagnosticsLatency, runTimeLatency, sequencerLatency, PCFilteringLatency, PCtoOMTotalLatency, OMFilterOutOfRangeLatency, insertScanLatency,
        OMtoPlTotalLatency, ppl_latency, smoothening_latency]
fig, ax = plt.subplots()
ax.stackplot(x, *data_list, labels=labels, colors = col)
ax.legend(loc='best')
output_file = "time_breakdown" + ".png"
plt.xlabel("mission progression (s)")
plt.ylabel("sensor to act latency(s)")
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)

# utilization
fig, axs = plt.subplots(2)
#axs[1].plot(x, end_to_end_latency=" end_to_end_latency")
axs[1].set_yscale('linear')
axs[1].plot(x, end_to_end_latency)
axs[1].set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
axs[1].legend(loc='best')

axs[0].plot(x, cpu_utilization_for_last_decision, label="cpu_utilization ")
axs[0].set(xlabel="mission  progression (s)", ylabel="mission progression (s) ")
axs[0].legend(loc='best')
output_file = "utilization" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)




#  HW INSTRUCTIONS
fig, axs = plt.subplots(2)
#axs[1].plot(x, end_to_end_latency=" end_to_end_latency")
axs[1].set_yscale('linear')
axs[1].plot(x, end_to_end_latency)
axs[1].set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
axs[1].legend(loc='best')

axs[0].plot(x, PERF_COUNT_HW_INSTRUCTIONS, label="HW instructions (millions)")
axs[0].set(xlabel="mission  progression (s)", ylabel="HW instructions (millions) ")
axs[0].legend(loc='best')
output_file = "hw_instructions" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)




# volume chart
fig, axs = plt.subplots(2)
axs[1].set_yscale('linear')
axs[1].plot(x, end_to_end_latency, label=" end_to_end_latency")
axs[1].set(xlabel="mission  progression (s)", ylabel="latnecy (s)")
axs[1].legend(loc='best')

axs[0].plot(x, octomap_volume_integrated, label="octomap_volume")
axs[0].plot(x, ppl_volume, label="ppl_volume")
axs[0].plot(x, om_to_pl_volume, label="om_to_pl_volume")
axs[0].set(xlabel="mission  progression (s)", ylabel="time (s)")
axs[0].set_yscale('log')
axs[0].legend(loc='best')
output_file = "volume" + ".png"
#plt.show()
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)
#plt.show()



# ------- datamovement chart
fig, axs = plt.subplots(2)
axs[1].set_yscale('linear')
axs[1].plot(x, end_to_end_latency, label=" end_to_end_latency")
axs[1].set(xlabel="mission  progression (s)", ylabel="latnecy (s)")
axs[1].legend(loc='best')

axs[0].plot(x,[y/1000 for y in om_to_pl_datamovement], label="om_to_pl_datamovement")
axs[0].plot(x, [y/1000 for y in pc_to_om_datamovement], label="pc_to_om_datamovement")
axs[0].set(xlabel="mission  progression (s)", ylabel="dataw movement (KB)")
axs[0].legend(loc='best')
output_file = "datamovement" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)



# ------- making the piechart
ax.set_yscale('linear')
group_names=['replanning', 'assessing']
group_size=[int(result_dict["planning_ctr"][0]), int(result_dict["decision_ctr"][0]) - int(result_dict["planning_ctr"][0])]
subgroup_names=['runtime failure cnt', 'traj gen failure cnt', 'planning success cnt', 'assessing']
subgroup_size= [int(result_dict['runtime_failure_ctr'][0]), int(result_dict['traj_gen_failure_ctr'][0]), int(result_dict['planning_success_ctr'][0]),
        int(result_dict["decision_ctr"][0]) - int(result_dict["planning_ctr"][0])]
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
output_file = "decision_pie_chart" + ".png"
fig.savefig(result_folder+"/"+output_file)
# show it
#plt.show()





# LLC REFERNCES
fig, axs = plt.subplots(2)
#axs[1].plot(x, end_to_end_latency=" end_to_end_latency")
axs[1].set_yscale('linear')
axs[1].plot(x, end_to_end_latency)
axs[1].set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
axs[1].legend(loc='best')

axs[0].plot(x, LLC_REFERENCES, label="LLC REFERENCES instructions (million)")
axs[0].set(xlabel="mission  progression (s)", ylabel="LLC REFERENCES (million) ")
axs[0].legend(loc='best')
output_file = "llc_references" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)


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





