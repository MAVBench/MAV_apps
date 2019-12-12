import matplotlib.pyplot as plt
import sys
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset


# plotting
point_cloud_resolution = [.1, .2]
octomap_resolution = [.3, .5]


known_vol_difference_for_map_3 = [float(.054)/(66281 + 5089), float(99)/(66964 + 6517)]
unknown_vol_difference_for_map_3 = [float(5089)/(66281 + 5089), float(6418)/(66964 + 6517)]

known_vol_difference_for_map_5 = [float(74)/(73644 + 1876), float(122)/(3133+ 72597)]
unknown_vol_difference_for_map_5 = [float(1749)/(73644 + 1876), float(3011)/(3133+ 72597)]

total_vol_difference_for_map_3 = [float(5089)/(66281 + 5089), float(6517)/(66964 + 6517)]
total_vol_difference_for_map_5 = [float(1876)/(73644 + 1876), float(3133)/(3133+ 72597)]



fig1 = plt.figure(1)
ax1 = fig1.add_subplot(111)
# axis labels
ax1.plot(point_cloud_resolution, total_vol_difference_for_map_3, marker='o', label = "total vol-diff for OM resolution = .3" )
ax1.plot(point_cloud_resolution, total_vol_difference_for_map_5, marker='o', label = "total vol-dif  for OM resolution = .5" )
ax1.set_xlabel('point cloud resolution (m)',fontsize=16)
ax1.set_ylabel('volume difference ratio', fontsize=16)
output_file_png = "point_cloud_res_vol_diff_total.png"
ax1.legend(loc='upper right', fontsize ="small")
result_folder = "data"
#plt.ylim([-.2,1.2 ])
plt.savefig(result_folder+"/"+output_file_png)
plt.close(fig1)

fig1 = plt.figure(1)
ax1 = fig1.add_subplot(111)
ax1.plot(point_cloud_resolution, unknown_vol_difference_for_map_3, marker='o', label = "unknown vol-diff for OM resolution = .3" )
ax1.plot(point_cloud_resolution, unknown_vol_difference_for_map_5, marker='o', label = "unknown vol-dif  for OM resolution = .5" )
ax1.set_xlabel('point cloud resolution (m)',fontsize=16)
ax1.set_ylabel('volume difference ratio', fontsize=16)
output_file_png = "point_cloud_res_vol_diff_unknown.png"
ax1.legend(loc='upper right', fontsize ="small")
result_folder = "data"
#plt.ylim([-.2,1.2 ])
plt.savefig(result_folder+"/"+output_file_png)
plt.close(fig1)


fig1 = plt.figure(1)
ax1 = fig1.add_subplot(111)
ax1.plot(point_cloud_resolution, known_vol_difference_for_map_3, marker='o', label = "known vol-diff for OM resolution = .3" )
ax1.plot(point_cloud_resolution, known_vol_difference_for_map_5, marker='o', label = "known vol-dif  for OM resolution = .5" )
output_file_png = "point_cloud_res_vol_diff_known.png"
ax1.set_xlabel('point cloud resolution (m)',fontsize=16)
ax1.set_ylabel('volume difference ratio', fontsize=16)
ax1.legend(loc='upper right', fontsize ="small")
result_folder = "data"
#plt.ylim([-.2,1.2 ])
plt.savefig(result_folder+"/"+output_file_png)
plt.close(fig1)



# save file

