import matplotlib.pyplot as plt
import sys
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
sys.path.append('../common_utils')
from data_parsing import *



result_folder = "../lower_res_time_budget"
input_file_name = "stats.json"
input_filepath = result_folder + "/" + input_file_name
output_file_name = "results.csv"
output_filepath = result_folder + "/" + output_file_name
metrics_to_collect_easy = ["distance_travelled",
                           "flight_time", "piecewise_planning_budget", "perception_lower_resolution",
                           "smoothening_budget", "experiment_number"]
metrics_to_collect_hard = ["S_A_latency", "S_A_response_time_calculated_from_imgPublisher",
                           "planning_piecewise_failure_rate", "planning_smoothening_failure_rate"]

# parse  data
result_dic = parse_stat_file(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)
result_dic = avg_over_sequence(result_dic, 3)


# ---- velocity data
#init figs
fig1 = plt.figure(1)
ax1 = fig1.add_subplot(111)

# axis labels
ax1.set_xlabel('time Budget (s)',fontsize=16)
ax1.set_ylabel('piecewise planing failure rate', fontsize=16)

# legend

# velocity acceleration
for perception_resolution_val in [.3, .5, .7, .9]:
    result_dic_filtered = filter_based_on_key_value(result_dic, "perception_lower_resolution", perception_resolution_val, "in")
    ax1.plot(result_dic_filtered["piecewise_planning_budget"], result_dic_filtered["planning_piecewise_failure_rate"], marker='o', label = "perception resolution:" + str(perception_resolution_val))


# save file
output_file = "resolution_time_budget.png"
#plt.savefig(output_file)


# ---- acceleration data
"""
ax1.set_ylabel('quantity',fontsize=16)

ax1.plot(x_values[:-1], acc_vals,marker='o', color='g', label="acceleration")
ax1.plot(subsample(x_values[:-1], 30), subsample(avg_acc,30) ,marker='o', color='orange', label="avg stoppage decceleration (stop cmd to full stop)")
output_file = "blah" + str(int(max(velocity_data))) + ".png"
"""
ax1.legend(loc='upper left', fontsize ="small")

plt.ylim([-.2,1.2 ])
plt.savefig(output_file)


#plt.show()
#fig2 = plt.figure(2)
#ax2 = fig2.add_subplot(211)
#plt.gcf().subplots_adjust(top=0.85,left=0.15,right=0.95)
#ax2.tick_params(axis='x',bottom=False,top=False,labelbottom=False)
#ax2.grid(which='major',linewidth='0.3')
#ax2.set_ylabel('Mission Time \n [s]',fontsize=16)
#ax3 = fig2.add_subplot(212)
#ax3.grid(which='major',linewidth='0.3')
#ax3.set_ylabel('Mission Energy \n [kJ]',fontsize=16)
#plt.xticks(x_pos,bars,weight='bold')
#fig2.subplots_adjust(hspace=0.0001)
#plot
#plt.rc('xtick',labelsize=16)

