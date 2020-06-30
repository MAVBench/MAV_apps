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
"cpu_utilization_for_last_decision"]

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
#width = 10.0/len(depthToPCConversionLatency)
x = range(0, len(runDiagnosticsLatency))
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
ax.plot(x, end_to_end_latency, label=" en_to_end_latency")
ax.plot(x, cpu_utilization_for_last_decision, label="cpu_utilization ")
ax.legend(loc='upper left')
output_file = "utilization" + ".png"
#plt.show()
fig.savefig(result_folder+"/"+output_file)
plt.close(fig)
plt.show()



"""
fig, ax = plt.subplots()
ax.stackplot(x, y)
plt.show()width = .36

labels = range(0, len(depthToPCConversionLatency))
#x = plt.subplots()
fig, ax=plt.subplots()
ax.bar(labels, runDiagnosticsLatency, width,  label='runDiagnosticsLatency')
ax.bar(labels, runTimeLatency, width,  bottom=runDiagnosticsLatency, label ="runTimeLatency")
ax.bar(labels, OMtoPlTotalLatency, width,  bottom=runTimeLatency, label ="OMtoPlTotalLatency")
ax.bar(labels, insertScanLatency, width,  bottom=OMtoPlTotalLatency, label ="insertScan")

ax.set_ylabel('latency breakdown')
ax.set_title('time progression progression')
ax.legend()

"""
















#fig = plt.figure()
"""
ax = fig.add_subplot(111, projection='3d')

# -- for point cloud/octomap (data_1/stats.json_om)
if stage_of_interest == "pc_om_estimation": print(len(pc_res)) print(len(pc_vol_estimated)) print(len(octomap_volume_digested)) ax.scatter(pc_vol_estimated[1000:], octomap_volume_digested[1000:])#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs) #ax.scatter(pc_res[100:], pc_vol_estimated[100:], octomap_volume_digested[100:])#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs) print(pc_res) print(pc_vol_estimated) print(octomap_volume_digested) elif stage_of_interest == "pc_om": print(pc_res) print(octomap_integeration_response_time)
    ax.scatter(pc_res, pc_vol_actual, octomap_integeration_response_time)#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
elif stage_of_interest == "om_to_pl":
    ax.scatter(om_to_pl_res, om_to_pl_vol_actual, octomap_to_motion_planner_serialization_to_reception_knob_modeling)#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
    #ax.scatter(om_to_pl_res, potential_volume_to_explore_knob_modeling, octomap_to_planner_com_overhead_knob_modeling)#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
elif stage_of_interest == "pp_pl":
    ax.scatter(om_to_pl_res[:-2], ppl_vol_actual_knob_modeling, piecewise_planner_time_knob_modeling)#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
else:
    print("stage of interest:" + stage_of_interest + "not defined")
    system.exit(0)

# plot
if (stage_of_interest == "pc_om_estimation"):
    ax.set_xlabel('pc_vol_estimation')
    ax.set_ylabel('octomap_volume_digested')
    ax.set_ylabel('resolution')
else:
    ax.set_xlabel('resolution')
    ax.set_ylabel('estimated volume')
    ax.set_zlabel('response time (s)');
ax.legend(loc='best', fontsize="small")
output_file = "knob_performance_modeling" + ".png"
"""

