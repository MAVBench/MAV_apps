import matplotlib.pyplot as plt
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

# which stage are you trying to plot
stage_of_interest = "pp_pl" # pick form ["om_to_pl", "pc_om", "pp_pl", "pc_om_estimation]

assert stage_of_interest in stage_of_interests_to_pick_from

result_folder = "./data_1"
input_file_name = "stats.json"
input_filepath = result_folder + "/" + input_file_name

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
"time_cmd_received"
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

#width = 10.0/len(depthToPCConversionLatency)
x = time_cmd_received;#range(0, len(runDiagnosticsLatency))
#y = np.vstack([y1, y2, y3])
#y= np.vstack([runDiagnosticsLatency, runTimeLatency, OMtoPlTotalLatency, insertScanLatency])
labels = ["runDiagnosticsLatency", "runTimeLatency", "sequencerLatency", "PCFilteringLatency", "PCtoOMTotalLatency", "OMFilterOutOfRangeLatency",   "insertScanLatency", 
        "OMtoPlTotalLatency", "ppl_latency", "smoothening_latency"]

data_list = [runDiagnosticsLatency, runTimeLatency, sequencerLatency, PCFilteringLatency, PCtoOMTotalLatency, OMFilterOutOfRangeLatency, insertScanLatency,
        OMtoPlTotalLatency, ppl_latency, smoothening_latency]
data_list
fig, ax = plt.subplots()
ax.stackplot(x, *data_list, labels=labels, colors = col)
ax.legend(loc='upper left')
output_file = "time_breakdown" + ".png"
#plt.show()
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)
plt.show()


fig, ax = plt.subplots()
ax.plot(x, end_to_end_latency, label=" end_to_end_latency")
ax.plot(x, cpu_utilization_for_last_decision, label="cpu_utilization ")
ax.legend(loc='upper left')
output_file = "utilization" + ".png"
#plt.show()
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)
plt.show()


fig, ax = plt.subplots()
ax.plot(x, octomap_volume_integrated, label="octomap_volume")
ax.plot(x, ppl_volume, label="ppl_volume")
ax.plot(x, om_to_pl_volume, label="om_to_pl_volume")
ax.legend(loc='upper left')
ax.set_yscale('log')
output_file = "volume" + ".png"
#plt.show()
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)
plt.show()








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
plt.show()






