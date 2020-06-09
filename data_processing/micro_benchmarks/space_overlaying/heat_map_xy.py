"""
#!/bin/python3
"""
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from pandas import *
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
from math import *
#result_dic = filter_based_on_key_value(result_dic, "pc_res", 1.200000, "in")
#write_results_to_csv(result_dic, output_all_csv_filepath)
#point_cloud_estimated_volume  = result_dic["octomap_volume_digested"]
#point_cloud_volume_to_digest = result_dic["point_cloud_volume_to_digest"]
#filtering = result_dic["filtering"]


def space_overlay(x,y, metric_values, title, resolution = -1, x_input_bound={"min":-1,"max":-1}, y_input_bound = {"min":-1,"max":-1}):
    if resolution == -1: resolution = 1

    # bins for x, bins for y
    coord_tuple = [el for el in zip(x, y)]

    # setting the bounds
    x_data_bounds = {"min":np.amin(x)/resolution, "max": np.amax(x)/resolution}
    y_data_bounds = {"min": np.amin(y)/resolution, "max": np.amax(y)/resolution}
    x_bounds = {}
    y_bounds = {}
    if x_input_bound["min"] == -1: x_bounds["min"] = x_data_bounds["min"]
    else: x_bounds["min"] = x_input_bound["min"]
    if x_input_bound["max"] == -1: x_bounds["max"] = x_data_bounds["max"]
    else: x_bounds["max"] = x_input_bound["max"]

    if y_input_bound["min"] == -1: y_bounds["min"] = y_data_bounds["min"]
    else: y_bounds["min"] = y_input_bound["min"]

    if y_input_bound["max"] == -1: y_bounds["max"] = y_data_bounds["max"]
    else: y_bounds["max"] = y_input_bound["max"]

    x_offset = -int(x_bounds["min"]) # offset by the min for array index placement
    y_offset = -int(y_bounds["min"]) # offset by the min for array index placement
    x_range = ceil(x_bounds["max"] - x_bounds["min"]) 
    y_range = ceil(y_bounds["max"] - y_bounds["min"])
    space_mat = np.zeros(shape=(y_range , x_range)) 

    # binning the points
    for i in range(0, len(x)):
        x_rounded = int(float(x[i])/resolution) + x_offset
        y_rounded = int(float(y[i])/resolution) + y_offset
        space_mat[y_rounded][x_rounded] = metric_values[i]
       

    # plotting 
    fig, ax = plt.subplots(figsize=(12,7))
    plt.title(title, fontsize=18)
    ttl = ax.title
    ttl.set_position([0.5, 1.05])

    sns.set()
    sns.heatmap(space_mat, cmap='RdYlGn', linewidth=0.30, ax=ax)
    ax = plt.gca()
    ax.set_ylim(ax.get_ylim()[::-1])
    plt.savefig(title)
    
    


result_folder = "./data_1"
input_file_name = "stats.json"
input_filepath = result_folder + "/" + input_file_name

# data to collect
metrics_to_collect_easy = []
metrics_to_collect_hard = ["x_coord_while_budgetting", "y_coord_while_budgetting", "vel_mag_while_budgetting", "sensor_to_actuation_time_budget_to_enforce", "obs_dist_statistics_min"]

# parse  data
result_dic = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

x_coord_while_budgetting = result_dic["x_coord_while_budgetting"]
y_coord_while_budgetting = result_dic["y_coord_while_budgetting"]
metrics_to_overlay = ["obs_dist_statistics_min", "sensor_to_actuation_time_budget_to_enforce", "vel_mag_while_budgetting"]
for metric_to_overlay in metrics_to_overlay:
    metric_values = result_dic[metric_to_overlay]
    space_overlay(x_coord_while_budgetting,y_coord_while_budgetting, metric_values, metric_to_overlay, 5)

#data_values = obs_dist_statistics_min

