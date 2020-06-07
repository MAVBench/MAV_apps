"""
#!/bin/python3
"""
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

#from data_file import coord_tuple, data_values
import matplotlib.pyplot as plt
import sys
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.mplot3d import Axes3D 
sys.path.append('../../common_utils')
from data_parsing import *

result_folder = "./data_1"
input_file_name = "stats.json"
input_filepath = result_folder + "/" + input_file_name

# data to collect
metrics_to_collect_easy = []
metrics_to_collect_hard = ["x_coord_while_budgetting", "y_coord_while_budgetting", "vel_mag_while_budgetting", "sensor_to_actuation_time_budget_to_enforce", "obs_dist_statistics_min"]

# parse  data
result_dic = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)
#result_dic = filter_based_on_key_value(result_dic, "pc_res", 1.200000, "in")
#write_results_to_csv(result_dic, output_all_csv_filepath)
#point_cloud_estimated_volume  = result_dic["octomap_volume_digested"]
#point_cloud_volume_to_digest = result_dic["point_cloud_volume_to_digest"]
#filtering = result_dic["filtering"]


def space_overlay(x,y, metric_values, title, x_bound=(-1,-1), y_bound=(-1,1), x_bin_count=20, y_bin_count=30):
    # bins for x, bins for y
    coord_tuple = [el for el in zip(x, y)]
    coords_nd = np.array(coord_tuple)
    min_bounds =[]
    max_bounds = []
    if x_bound[0] == -1:
        min_bounds.append(np.amin(coords_nd, axis=0)[0])
    else:
        min_bounds.append(x_bound[0])
    
    if x_bound[1] == -1:
        max_bounds.append(np.amax(coords_nd, axis=0)[0])
    else: 
        max_bounds.append(x_bound[1])
    
    if y_bound[0] == -1:
        min_bounds.append(np.amin(coords_nd, axis=0)[1])
    else:    
        min_bounds.append(y_bound[0])
    
    if y_bound[1] == -1:
        max_bounds.append(np.amax(coords_nd, axis=0)[1])
    else:
        max_bounds.append(y_bound[1])
    #max_bounds = np.amax(coords_nd, axis=0)
    #min_bounds = np.amin(coords_nd, axis=0)
    print(max_bounds) 
    print(min_bounds) 
    x_bin_count = int(max_bounds[0] - min_bounds[0])+1 
    y_bin_count = int(max_bounds[1] - min_bounds[1])+1 
    bins = [x_bin_count, y_bin_count]
    space_mat = np.zeros(shape=(bins[1], bins[0]))
    bin_xi = (max_bounds[0] - min_bounds[0]) / bins[0]
    bin_yi = (max_bounds[1] - min_bounds[1]) / bins[1]

    #bin_xi = int(max_bounds[0] - min_bounds[0])+1 
    #bin_yi = int(max_bounds[1] - min_bounds[1]) +1

    coords_i = 0
    epsilon = 0.0001
    for row in coords_nd:
        bin_x = -1
        curr_bx = min_bounds[0]
        for i in range(0, bins[0]):
            curr_bx += bin_xi
            if row[0] <= (curr_bx + epsilon):
                bin_x = i
                break

        bin_y = -1
        curr_by = min_bounds[1]
        for j in range(0, bins[1]):
            curr_by += bin_yi
            if row[1] <= (curr_by + epsilon):
                bin_y = j
                break

        space_mat[bin_y][bin_x] = metric_values[coords_i]
        coords_i += 1



    fig, ax = plt.subplots(figsize=(12,7))


    plt.title(title, fontsize=18)
    ttl = ax.title
    ttl.set_position([0.5, 1.05])

    #ax.set_xticks([])
    #ax.set_yticks([])
    #ax.axis('off')

    sns.set()
    sns.heatmap(space_mat, cmap='RdYlGn', linewidth=0.30, ax=ax)

    ax = plt.gca()
    # start with 0 at bottom and 9 on top
    ax.set_ylim(ax.get_ylim()[::-1])
    plt.savefig(title)
    #return plt
    #plt.savefig(title)
    #plt.show()
    

x_coord_while_budgetting = result_dic["x_coord_while_budgetting"]
y_coord_while_budgetting = result_dic["y_coord_while_budgetting"]
metrics_to_overlay = ["obs_dist_statistics_min", "sensor_to_actuation_time_budget_to_enforce", "vel_mag_while_budgetting"]
for metric_to_overlay in metrics_to_overlay:
    metric_values = result_dic[metric_to_overlay]
    space_overlay(x_coord_while_budgetting,y_coord_while_budgetting, metric_values, metric_to_overlay)

#data_values = obs_dist_statistics_min

