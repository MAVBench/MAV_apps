import matplotlib.pyplot as plt
import sys
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
sys.path.append('../common_utils')
from data_parsing import *



result_folder = "./data_2"
input_file_name = "stats.json"
input_filepath = result_folder + "/" + input_file_name

metrics_to_collect_easy = ["distance_travelled",
                           "flight_time", "piecewise_planning_budget", "perception_resolution",
                           "smoothening_budget", "experiment_number"]
metrics_to_collect_hard = ["S_A_latency", "S_A_response_time_calculated_from_imgPublisher",
                           "planning_piecewise_failure_rate", "planning_smoothening_failure_rate", "RRT_path_length_normalized_to_direct_path"]


indep_vars = ["piecewise_planning_budget", "perception_resolution",
                           "smoothening_budget", "experiment_number"]

dep_vars = ["distance_travelled", "flight_time", "S_A_latency", "S_A_response_time_calculated_from_imgPublisher",
                           "planning_piecewise_failure_rate", "planning_smoothening_failure_rate", "RRT_path_length_normalized_to_direct_path"]



# parse  data
result_dic = parse_stat_file(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

output_all_file_name = "all_results.csv"
output_all_csv_filepath = result_folder + "/" + output_all_file_name
write_results_to_csv(result_dic, output_all_csv_filepath)

output_all_file_name = "all_results_avged.csv"
output_all_csv_filepath = result_folder + "/" + output_all_file_name
write_results_to_csv(result_dic, output_all_csv_filepath)
avg_result_dic = avg_over_sequence(result_dic, 3)
std_result_dic = std_over_sequence(result_dic, 3,  indep_vars)

# plotting
fig1 = plt.figure(1)
ax1 = fig1.add_subplot(111)

# axis labels
ax1.set_xlabel('time Budget (s)',fontsize=16)
ax1.set_ylabel('piecewise planing failure rate', fontsize=16)


color_ = ['m', 'y']
idx = 0
# plot time budget vs piecewise planning rate
for perception_resolution_val in [.3, .9]:
    avg_result_dic_filtered = filter_based_on_key_value(avg_result_dic, "perception_resolution", perception_resolution_val, "in")
    std_result_dic_filtered = filter_based_on_key_value(std_result_dic, "perception_resolution", perception_resolution_val, "in")
    ax1.errorbar(avg_result_dic_filtered["piecewise_planning_budget"], avg_result_dic_filtered["planning_piecewise_failure_rate"], std_result_dic_filtered["planning_piecewise_failure_rate"], color=color_[idx], marker='o', label = "perception resolution:" + str(perception_resolution_val))
    idx +=1
# save file
output_file_png = "time_budget_piecewise_planning_failure_rate.png"

ax1.legend(loc='best', fontsize ="small")

plt.ylim([-.1,1.1 ])
plt.savefig(result_folder+"/"+output_file_png)
plt.close(fig1)

# plot time budget vs smoothening rate
fig2 = plt.figure(1)
ax2 = fig2.add_subplot(111)

# axis labels
ax2.set_xlabel('time Budget (s)',fontsize=16)
ax2.set_ylabel('smoothening failure rate', fontsize=16)


color_ = ['m', 'y']
idx = 0
# plot time buidget vs piecewise planning rate
for perception_resolution_val in [.3, .9]:
    #result_dic_filtered = filter_based_on_key_value(result_dic, "perception_resolution", perception_resolution_val, "in")
    avg_result_dic_filtered = filter_based_on_key_value(avg_result_dic, "perception_resolution",
                                                        perception_resolution_val, "in")
    std_result_dic_filtered = filter_based_on_key_value(std_result_dic, "perception_resolution",
                                                        perception_resolution_val, "in")
    #ax2.plot(result_dic_filtered["smoothening_budget"], result_dic_filtered["planning_smoothening_failure_rate"], marker='o', label = "perception resolution:" + str(perception_resolution_val))
    ax2.errorbar(avg_result_dic_filtered["smoothening_budget"], avg_result_dic_filtered["planning_smoothening_failure_rate"], std_result_dic_filtered["planning_smoothening_failure_rate"], color=color_[idx], marker='o', label = "perception resolution:" + str(perception_resolution_val))
    idx += 1
# save file
output_file_png = "time_budget_smoothening_failure_rate.png"

ax2.legend(loc='best', fontsize ="small")

plt.ylim([-.2,1.2 ])
plt.savefig(result_folder+"/"+output_file_png)
plt.close(fig2)

# plot time budget vs planing efficienty
fig3 = plt.figure(1)
ax3 = fig3.add_subplot(111)

# axis labels
ax3.set_xlabel('piecewise planning time Budget (s)',fontsize=16)
ax3.set_ylabel('path inefficiency: normalize length to dir path', fontsize=16)

color_ = ['m', 'y']
idx = 0
# plot time buidget vs piecewise planning rate
for perception_resolution_val in [.3, .9]:
    #result_dic_filtered = filter_based_on_key_value(result_dic, "perception_resolution", perception_resolution_val, "in")
    avg_result_dic_filtered = filter_based_on_key_value(avg_result_dic, "perception_resolution",
                                                        perception_resolution_val, "in")
    std_result_dic_filtered = filter_based_on_key_value(std_result_dic, "perception_resolution",
                                                        perception_resolution_val, "in")
#    ax3.plot(result_dic_filtered["piecewise_planning_budget"], result_dic_filtered["RRT_path_length_normalized_to_direct_path"], marker='o', label = "perception resolution:" + str(perception_resolution_val))
    ax3.errorbar(avg_result_dic_filtered["piecewise_planning_budget"], avg_result_dic_filtered["RRT_path_length_normalized_to_direct_path"], std_result_dic_filtered["RRT_path_length_normalized_to_direct_path"], marker='o', color=color_[idx], label = "perception resolution:" + str(perception_resolution_val))
    idx +=1



# save file
output_file_png = "time_budget_path_length_efficiency.png"

ax3.legend(loc='best', fontsize ="small")

plt.ylim([.996,1.026])
plt.savefig(result_folder+"/"+output_file_png)
plt.close(fig3)


fig4 = plt.figure(1)
ax4 = fig4.add_subplot(111)

# axis labels
ax4.set_xlabel('piecewise planning time Budget (s)',fontsize=16)
ax4.set_ylabel('distance travelled (m)', fontsize=16)

idx = 0
color_ = ['m', 'y']
# plot time buidget vs piecewise planning rate
for perception_resolution_val in [.3, .9]:
    #result_dic_filtered = filter_based_on_key_value(result_dic, "perception_resolution", perception_resolution_val, "in")
    avg_result_dic_filtered = filter_based_on_key_value(avg_result_dic, "perception_resolution",
                                                        perception_resolution_val, "in")
    std_result_dic_filtered = filter_based_on_key_value(std_result_dic, "perception_resolution",
                                                        perception_resolution_val, "in")
    cnt = 0
    for el in avg_result_dic_filtered["distance_travelled"]:
        if (el < 3):
            avg_result_dic_filtered["distance_travelled"][cnt] = float("inf")
        cnt +=1
#    ax4.plot(result_dic_filtered["piecewise_planning_budget"], result_dic_filtered["distance_travelled"], marker='o', label = "perception resolution:" + str(perception_resolution_val))
    ax4.errorbar(avg_result_dic_filtered["piecewise_planning_budget"], avg_result_dic_filtered["distance_travelled"], std_result_dic_filtered["distance_travelled"], color=color_[idx], marker='o', label = "perception resolution:" + str(perception_resolution_val))
    idx += 1

# save file
output_file_png = "time_budget_distance_travelled.png"

ax4.legend(loc='best', fontsize ="small")

plt.ylim([470,940])
plt.savefig(result_folder+"/"+output_file_png)
plt.close(fig3)


