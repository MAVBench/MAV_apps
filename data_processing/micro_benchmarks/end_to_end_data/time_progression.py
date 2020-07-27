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
"gap_statistics_avg"
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

x_coord_while_budgetting = result_dict["x_coord_while_budgetting"]
y_coord_while_budgetting = result_dict["y_coord_while_budgetting"]
z_coord_while_budgetting = result_dict["z_coord_while_budgetting"]
sensor_to_actuation_time_budget_to_enforce = result_dict["sensor_to_actuation_time_budget_to_enforce"]
closest_unknown_distance= result_dict["closest_unknown_distance"]
obs_dist_statistics_min = result_dict["obs_dist_statistics_min"]
vel_mag_while_budgetting = result_dict["vel_mag_while_budgetting"]
pc_res = result_dict["pc_res"]
gap_statistics_avg= result_dict["gap_statistics_avg"]

end_to_end_latency =  result_dict["ee_latency"]
blind_latency =  result_dict["blind_latency"]
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


# response time
x = time_cmd_received;#range(0, len(runDiagnosticsLatency))
labels = ["ee_latency", "blind_latency"]
data_list = [end_to_end_latency[2:], blind_latency[2:]]
fig, ax = plt.subplots()
ax.stackplot(x[2:], *data_list, labels=labels, colors = col)
ax.legend(loc='best')
output_file = "response_time_breakdown" + ".png"
plt.xlabel("mission progression (s)")
plt.ylabel("response time components (s)")
plt.yticks(np.arange(0, 5, .2))
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



# velocity/budget to enforce
x_copy = []
planner_drone_radius = 1.5
sensor_max_range = 25;
sensor_to_actuation_time_budget_to_enforce_copy = []
vel_mag_while_budgetting_copy = []
closest_unknown_distance_copy = []
for idx, el in enumerate(sensor_to_actuation_time_budget_to_enforce):
    if el < .001 or el > 50:
        continue
    else:
        sensor_to_actuation_time_budget_to_enforce_copy.append(el)
        vel_mag_while_budgetting_copy.append(vel_mag_while_budgetting[idx])
        closest_unknown_distance_copy.append(min(closest_unknown_distance[idx], sensor_max_range))
        x_copy.append(x[idx])


fig, axs = plt.subplots(2)
#axs[1].plot(x, end_to_end_latency=" end_to_end_latency")
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
output_file = "vel_res" + ".png"
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
for idx, el in enumerate(insertScanLatency):
    if el < .001 or el > 20:
        continue
    else:
        insertScanLatency_copy.append(el)
        obs_dist_statistics_min_copy.append(obs_dist_statistics_min[idx])
        x_copy.append(x[idx])
        gap_statistics_avg_copy.append(min(gap_statistics_avg[idx], sensor_max_range))
        pc_res_copy.append(pc_res[idx])
        min_of_two_space_dist.append(min(min(gap_statistics_avg[idx], sensor_max_range), abs(obs_dist_statistics_min[idx])))

#axs[1].plot(x, end_to_end_latency=" end_to_end_latency")
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
output_file = "space_res" + ".png"
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)








fig, axs = plt.subplots()
#axs[1].plot(x, end_to_end_latency=" end_to_end_latency")
axs.set_yscale('linear')
axs.plot(x, end_to_end_latency)
axs.set(xlabel="mission  progression (s)", ylabel=" end to end latency(s)")
axs.legend(loc='best')
output_file = "ee_latency" + ".png"
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





