import matplotlib.pyplot as plt
import sys
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.mplot3d import Axes3D 
sys.path.append('../common_utils')
from data_parsing import *




result_folder = "./data_1"
input_file_name = "stats.json"
input_filepath = result_folder + "/" + input_file_name

#metrics_to_collect_easy = ["point_cloud_resolution",
#                           "flight_time", "piecewise_planning_budget", "perception_resolution",
#                           "smoothening_budget", "experiment_number"]
metrics_to_collect_easy = []
#metrics_to_collect_hard = ["point_cloud_resolution", "point_cloud_area_to_digest",
#                           "filtering", "point_cloud_volume_to_digest", "octomap_insertCloud_minus_publish_all"]
#metrics_to_collect_hard = ["octomap_exposed_resolution", "point_cloud_estimated_volume", "octomap_insertCloud_minus_publish_all", "octomap_volume_digested", "potential_volume_to_explore_knob_modeling",
#        "resolution_to_explore_knob_modeling", "octomap_to_motion_planner_serialization_to_reception_knob_modeling, piecewise_planner_time_knob_modeling", "piecewise_planner_volume_explored_knob_modeling"]
metrics_to_collect_hard = ["octomap_exposed_resolution", "point_cloud_estimated_volume", "octomap_volume_digested", "potential_volume_to_explore_knob_modeling",
        "resolution_to_explore_knob_modeling", "piecewise_planner_time_knob_modeling", "piecewise_planner_resolution_knob_modeling", "piecewise_planner_volume_explored_knob_modeling",
        "piecewise_planner_time_knob_modeling", "octomap_to_motion_planner_serialization_to_reception_knob_modeling", "octomap_insertCloud_minus_publish_all",
        "octomap_to_planner_com_overhead_knob_modeling", "pc_res", "pc_vol_actual", "om_to_pl_res_knob_modeling", "om_to_pl_vol_actual_knob_modeling", "ppl_vol_actual_knob_modeling", ]

indep_vars = ["piecewise_planning_budget", "perception_resolution",
                           "smoothening_budget", "experiment_number"]

dep_vars = ["distance_travelled", "flight_time", "S_A_latency", "S_A_response_time_calculated_from_imgPublisher",
                           "planning_piecewise_failure_rate", "planning_smoothening_failure_rate", "RRT_path_length_normalized_to_direct_path"]


# parse  data
result_dic = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)
#result_dic = filter_based_on_key_value(result_dic_, "octomap_exposed_resolution", 0.15, "in")
#write_results_to_csv(result_dic, output_all_csv_filepath)
octomap_exposed_resolution = result_dic["octomap_exposed_resolution"]
point_cloud_estimated_volume  = result_dic["point_cloud_estimated_volume"]
#point_cloud_estimated_volume  = result_dic["octomap_volume_digested"]
#point_cloud_volume_to_digest = result_dic["point_cloud_volume_to_digest"]
#filtering = result_dic["filtering"]
octomap_integeration_response_time = result_dic["octomap_insertCloud_minus_publish_all"]
resolution_to_explore_knob_modeling = result_dic["resolution_to_explore_knob_modeling"]
piecewise_planner_resolution_knob_modeling = result_dic["piecewise_planner_resolution_knob_modeling"]
potential_volume_to_explore_knob_modeling= result_dic["potential_volume_to_explore_knob_modeling"]
piecewise_planner_time_knob_modeling = result_dic["piecewise_planner_time_knob_modeling"]
piecewise_planner_volume_explored_knob_modeling = result_dic["piecewise_planner_volume_explored_knob_modeling"]
octomap_to_motion_planner_serialization_to_reception_knob_modeling = result_dic["octomap_to_motion_planner_serialization_to_reception_knob_modeling"]
octomap_to_planner_com_overhead_knob_modeling = result_dic["octomap_to_planner_com_overhead_knob_modeling"]
pc_res = result_dic["pc_res"]
pc_vol_actual = result_dic["pc_vol_actual"]
om_to_pl_res = result_dic["om_to_pl_res_knob_modeling"]
om_to_pl_vol_actual = result_dic["om_to_pl_vol_actual_knob_modeling"]
ppl_vol_actual_knob_modeling = result_dic["ppl_vol_actual_knob_modeling"]


# -- filtering for debugging (probing into the data)
#filtered_results = filter_based_on_keys(result_dic, ["piecewise_planner_resolution_knob_modeling", "piecewise_planner_time_knob_modeling"])
#filtered_results = filter_based_on_key_value(filtered_results, "piecewise_planner_resolution_knob_modeling", 1.2, "in")

fig3 = plt.figure()
#X, Y = np.meshgrid(point_cloud_area_to_digest, point_cloud_resolution)
#Z = filtering;
# ax = plt.axes(projection='3d')
ax3 = fig3.add_subplot(111, projection='3d')
#ax3.contour3D(X, Y, Z, 50, zdir='z', cmap=plt.get_cmap('viridis'))  # 'binary')



# -- for point cloud/octomap (data_1/stats.json_om)
#ax3.scatter(pc_res, pc_vol_actual, octomap_integeration_response_time)#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs)

# -- for octomap to planner (data_1/stats.json_om_to_pl)
#ax3.scatter(om_to_pl_res, om_to_pl_vol_actual, octomap_to_motion_planner_serialization_to_reception_knob_modeling)#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
#ax3.scatter(om_to_pl_res, potential_volume_to_explore_knob_modeling, octomap_to_planner_com_overhead_knob_modeling)#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs)

# -- for piecewise planner (data_1/stats.json_pp_ppl)
ax3.scatter(piecewise_planner_resolution_knob_modeling, ppl_vol_actual_knob_modeling, piecewise_planner_time_knob_modeling)#, zdir='z', c=None, depthshade=True)#(, *args, **kwargs)


ax3.set_xlabel('resolution')
ax3.set_ylabel('estimated volume')
ax3.set_zlabel('response time (s)');
#ax3.set_zscale('log');
#ax3.set_zlim(0, .1);
#ax3.set_xlim(0, .2);

ax3.legend(loc='best', fontsize="small")
output_file = "knob_performance_modeling" + ".png"
plt.show()
plt.savefig(result_folder+"/"+output_file)



# plotting
"""
fig1 = plt.figure(1)
ax1 = fig1.add_subplot(111)
#blah = result_dic["point_cloud_estimated_volume"]
#blah2 = result_dic["octomap_volume_digested"]

ax1.scatter(result_dic["point_cloud_estimated_volume"][:400], result_dic["octomap_volume_digested"][:400])
# axis labels ax1.set_xlabel('octomap interpretation of volume',fontsize=16)
ax1.set_ylabel('octomap interpretation of volume', fontsize=16)
#plt.show()
"""
"""
fig2 = plt.figure(1)
ax2 = fig2.add_subplot(111)
point_cloud_volume = [1000, 2000, 3000, 4000, 4200, 4500, 5000, 5300, 5350, 5360]
forty_five_deg = [1000-500, 2000-500, 3000-500, 4000-500, 4200-500, 4500-500, 5000-500, 5300-500, 5350-500, 5360-500]
octomap_res_04_volume = [428, 862, 1339, 1832, 2000, 2259, 2748, 3142, 3267, 4471]
octomap_res_15_volume = [969, 1835, 2707, 3543, 3810, 4220, 4949, 5496, 5664,7485]

ax2.plot(point_cloud_volume, octomap_res_04_volume, c="r", label ="octomap_res_04")
ax2.plot(point_cloud_volume, octomap_res_15_volume, c="b", label ="octomap_res_15")
ax2.plot(point_cloud_volume, forty_five_deg , c="y", label ="45")
# axis labels ax1.set_xlabel('octomap interpretation of volume',fontsize=16)
ax2.set_ylabel('point cloud interpretation of volume', fontsize=16)
ax2.set_ylabel('octomap interpretation of volume', fontsize=16)

plt.show()
"""


plt.close(fig3)

