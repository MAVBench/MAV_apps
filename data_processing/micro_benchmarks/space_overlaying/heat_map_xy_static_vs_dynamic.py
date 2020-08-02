"""
#!/bin/python3
"""
import numpy as np
import pandas as pd
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
import json
import argparse
import os
import time
from collections import OrderedDict 


#time.sleep(5)

parser = argparse.ArgumentParser()
parser.add_argument('-d', '--datafile_path', type=str)
parser.add_argument('-e', '--envfile_path', type=str)
parser.add_argument('-b', '--bounds', nargs='+', type=int)
parser.add_argument('-m', '--mode', nargs='+', type=str)
args = parser.parse_args()

def dir_path(path):
    if not os.path.isfile(path):
        print("Couldn't find datafile!")
        exit(-1)
    else:
        return os.path.dirname(os.path.realpath(path))

# we dump resulting graphs in same dir as datafile

#input_filepath = args.datafile_path
result_folder = "./data_1"#dir_path(input_filepath)
obs_stats_path = args.envfile_path
data_type = args.mode

bounds = args.bounds
if bounds == None:
    bounds = [-1, -1, -1, -1]
else:
    assert len(bounds) == 4, "bounds must be xmin, xmax, ymin, ymax (4 values)"

def extract_obstacles(obstacle_coords_scaled, x_bounds, y_bounds):
    x_offset = -int(x_bounds["min"]) # offset by the min for array index placement
    y_offset = -int(y_bounds["min"]) # offset by the min for array index placement

    obs_origin = []
    obs_width = []
    obs_height = []

    for i in range(0, len(obstacle_coords_scaled)):
        # obstacle coord bounding boxes are arranged:
        # top right corner, bottom left
        obstacle_bounds = obstacle_coords_scaled[i]
        top_right = obstacle_bounds[0]
        bottom_left = obstacle_bounds[1]
        # checking if obstacle lies within bounds
        if (top_right[0] < x_bounds["max"] and top_right[1] < y_bounds["max"]) or \
                (bottom_left[0] > x_bounds["min"] and bottom_left[1] > y_bounds["min"]):
            obs_origin.append([bottom_left[0] + x_offset, bottom_left[1] + y_offset])
            obs_width.append(top_right[0] - bottom_left[0])
            obs_height.append(top_right[1] - bottom_left[1])

    obs_dict = {}
    obs_dict["origin"] = obs_origin
    obs_dict["width"] = obs_width
    obs_dict["height"] = obs_height
    obs_dict["n_obstacles"] = len(obs_origin)

    return obs_dict

def space_overlay(x,y, metric_values, mode, obstacle_coords, title, resolution = -1, spread_of_obstacles = -1, cbar_min=-1, cbar_max=-1,
        x_input_bound={"min":-1,"max":-1}, y_input_bound = {"min":-1,"max":-1}, larger_is_better=True, show_y_ticks = True):

    if resolution == -1: resolution = 1
    if spread_of_obstacles == -1: spread_of_obstacles = 0

    # bins for x, bins for y
    coord_tuple = [el for el in zip(x, y)]

    # setting the bounds
    x_data_bounds = {"min":np.amin(x)/resolution, "max": np.amax(x)/resolution}
    y_data_bounds = {"min": np.amin(y)/resolution, "max": np.amax(y)/resolution}
    x_bounds = {}
    y_bounds = {}

    # we adjust the data bounds by the spread of obstacles so that the 
    # most relevant part of the map with all the obstacles is displayed

    if x_input_bound["min"] == -1: x_bounds["min"] = x_data_bounds["min"] - spread_of_obstacles/resolution
    else: x_bounds["min"] = x_input_bound["min"]/resolution

    if x_input_bound["max"] == -1: x_bounds["max"] = x_data_bounds["max"] + spread_of_obstacles/resolution
    else: x_bounds["max"] = x_input_bound["max"]/resolution

    if y_input_bound["min"] == -1: y_bounds["min"] = y_data_bounds["min"] - spread_of_obstacles/resolution
    else: y_bounds["min"] = y_input_bound["min"]/resolution

    if y_input_bound["max"] == -1: y_bounds["max"] = y_data_bounds["max"] + spread_of_obstacles/resolution
    else: y_bounds["max"] = y_input_bound["max"]/resolution

    x_offset = -int(x_bounds["min"]) # offset by the min for array index placement
    y_offset = -int(y_bounds["min"]) # offset by the min for array index placement
    x_range = ceil(x_bounds["max"] - x_bounds["min"]) 
    y_range = ceil(y_bounds["max"] - y_bounds["min"])
    space_mat= np.empty(shape=(y_range, x_range))
    space_mat[:] = np.nan # so that coords we don't have data for are greyed out
    space_mat_static = np.empty(shape=(y_range, x_range))
    space_mat_static [:] = np.nan # so that coords we don't have data for are greyed out


    # modify the title and cap at max 
    sensor_max_range = 25 
    min_value = max_metric_value = 1000000
    max_value = min_metric_value = -1
    plot_title = title 
    if (title == "closest_unknown_distance"):
        max_metric_value = sensor_max_range
        plot_title = "Sensor Visibility (m)"
    elif (title== "sensor_to_actuation_time_budget_to_enforce"):
        max_metric_value = 6
        plot_title = "Time Budget (s)"
    elif (title== "pc_res"):
        plot_title = "Precision (Voxel Size in cm)"
    elif (title== "insertScanLatency"):
        plot_title = "Response Time (s)"
    elif (title== "vel_mag_while_budgetting"):
        plot_title = "Acutuation Velocity (m/s)"
        max_metric_value = 6
    elif (title== "octomap_volume_integrated"):
        plot_title = "Space Volume Processed (m3)"


    # binning the points
    for i in range(0, len(x)):
        x_rounded = int(float(x[i])/resolution) + x_offset
        y_rounded = int(float(y[i])/resolution) + y_offset
        if x_rounded < x_range and y_rounded < y_range:
            space_mat[y_rounded][x_rounded] = min(metric_values[i], max_metric_value)
            min_value = min(space_mat[y_rounded][x_rounded], min_value)
            max_value = max(space_mat[y_rounded][x_rounded], max_value)
            #space_mat_static[y_rounded][x_rounded] = static_value
       
    # plotting 
    #space_mat_dict =  OrderedDict()
    #space_mat_dict ["static"] = space_mat_static
    #space_mat_dict ["dynamic"] = space_mat_dynamic
    fig, axs = plt.subplots(1, 1)
    
    # overlaying obstacles on top of plot
    # we pass all the obstacle coords scaled by resolution since the x_bounds
    # and y_bounds are scaled as well
    obstacle_coords_scaled = np.array(obstacle_coords) / resolution
    obs_dict = extract_obstacles(obstacle_coords_scaled, x_bounds, y_bounds)
    n_obs = obs_dict["n_obstacles"]
    ctr = 0 
    plt.suptitle(plot_title) 
    #for key in space_mat_dict.keys():
        #axs.set_title(key, fontsize=10)
        #ttl = axs.title
        #ttl.set_position([0.5, 1.05])
    for i in range(0, n_obs):
        origin = obs_dict['origin'][i]
        width = obs_dict['width'][i]
        height = obs_dict['height'][i]
        axs.add_patch(
                patches.Rectangle(origin, width, height, color="black", linewidth=0)
                )
    sns.set()

    colormap='RdYlGn'
    if not larger_is_better:
        # reverses colormap
        colormap += '_r'

    
    cbar_predicate = True
    #cbar_predicate = False
    if (cbar_min == -1):
        cbar_min = min_value
        cbar_max = max_value 
    sns.heatmap(space_mat, vmin= cbar_min , vmax=cbar_max, cmap=colormap, cbar=cbar_predicate, ax=axs, square=True)
    #axs[ctr] = plt.gca()

    # setting ticks and tick positions on axes

    n_ticks = 15

    x_tick_locs = np.rint(np.linspace(x_bounds["min"], x_bounds["max"], n_ticks)) + x_offset
    y_tick_locs = np.rint(np.linspace(y_bounds["min"], y_bounds["max"], n_ticks)) + y_offset

    # scale back by resolution and round since we want absolute units on the labels
    x_tick_labels = np.rint(np.linspace(x_bounds["min"], x_bounds["max"], n_ticks)) * resolution
    y_tick_labels = np.rint(np.linspace(y_bounds["min"], y_bounds["max"], n_ticks)) * resolution

    axs.set_xticks(x_tick_locs)
    if (show_y_ticks): 
        axs.set_yticks(y_tick_locs)
    else:
        axs.set_yticks([])
    axs.set_xticklabels(x_tick_labels)
    axs.set_yticklabels(y_tick_labels)
    axs.set_ylim(axs.get_ylim()[::-1])
    ctr +=1
    plt.savefig(result_folder + "/" + title + "_" + mode)
    ok = space_mat.min() 
    return min_value, max_value
# data to collect

metrics_to_collect_easy = []
metrics_to_collect_hard = [
        "x_coord_while_budgetting", 
        "y_coord_while_budgetting", 
        "vel_mag_while_budgetting", 
        "sensor_to_actuation_time_budget_to_enforce", 
        "octomap_volume_integrated",
        "pc_res",
        "obs_dist_statistics_min",
        "insertScanLatency",
	"closest_unknown_distance",

]


def parse_data(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard):
# parse  data
    env_gen_stats = {}
    with open(obs_stats_path, "r") as obs_json_file:
        env_gen_stats = json.load(obs_json_file)
        obstacle_coords = env_gen_stats["ObstacleList"]
        result_dic = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard, obs_json_file)
        x_coord_while_budgetting = result_dic["x_coord_while_budgetting"]
        y_coord_while_budgetting = result_dic["y_coord_while_budgetting"]
    return result_dic, x_coord_while_budgetting[0], y_coord_while_budgetting[0], obstacle_coords, env_gen_stats


# adding an offset to show the entire map
grid_resolution = 2

metrics_to_overlay = [
        "vel_mag_while_budgetting", 
        "sensor_to_actuation_time_budget_to_enforce", 
        "octomap_volume_integrated",
        "pc_res",
        "obs_dist_statistics_min",
        "insertScanLatency",
	"closest_unknown_distance",
]

x_input_bound = {"min": bounds[0], "max": bounds[1]}
y_input_bound = {"min": bounds[2], "max": bounds[3]}

static_value_dict = {}
static_value_dict["vel_mag_while_budgetting"] = 5
static_value_dict["sensor_to_actuation_time_budget_to_enforce"] = .1 
static_value_dict["octomap_volume_integrated"] = 46000
static_value_dict["obs_dist_statistics_min"] = 6
static_value_dict["pc_res"] = .3
static_value_dict["insertScanLatency"] = 6.5 
static_value_dict["closest_unknown_distance"] = 6 
# plot dynamic 

# plot resolution, volume
metrics_to_draw = ["pc_res", "octomap_volume_integrated", "insertScanLatency", "vel_mag_while_budgetting", "closest_unknown_distance", "sensor_to_actuation_time_budget_to_enforce"]
cbar_min = -1
cbar_max = 1

example_name ="velocity_"
#example_name ="res_vol_"
for metric_to_overlay in metrics_to_draw:
    for mode in ["dynamic", "static"]:
        input_filepath = result_folder +"/"+example_name + mode+"_stats.json" 
        result_dic, x_coord_while_budgetting, y_coord_while_budgetting, obstacle_coords, env_gen_stats =  parse_data(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

        if (mode == "static"):
            result_dic["vel_mag_while_budgetting"][0] = len(result_dic["vel_mag_while_budgetting"][0])*[static_value_dict["vel_mag_while_budgetting"]]
            result_dic["closest_unknown_distance"][0] = len(result_dic["closest_unknown_distance"][0])*[static_value_dict["closest_unknown_distance"]]
            result_dic["pc_res"][0] = len(result_dic["pc_res"][0])*[static_value_dict["pc_res"]]
            result_dic["octomap_volume_integrated"][0] = len(result_dic["octomap_volume_integrated"][0])*[static_value_dict["octomap_volume_integrated"]]
            result_dic["insertScanLatency"][0] = len(result_dic["insertScanLatency"][0])*[static_value_dict["insertScanLatency"]]
            result_dic["sensor_to_actuation_time_budget_to_enforce"][0] = len(result_dic["sensor_to_actuation_time_budget_to_enforce"][0])*[static_value_dict["sensor_to_actuation_time_budget_to_enforce"]]
        spread_of_obstacles = env_gen_stats["SpreadOfObstacles"]

        # some specific drawing stuff
        if metric_to_overlay in ["pc_res", "vel_mag_while_budgetting"]:
            show_y_ticks = True
        else:
            show_y_ticks = False
        metric_values = result_dic[metric_to_overlay][0]
        larger_is_better_predicate = True
        if metric_to_overlay in ["insertScanLatency"]:
            larger_is_better_predicate = False
        if mode == "dynamic":
            cbar_min = -1
            cbar_max = -1
        cbar_min, cbar_max= space_overlay(x_coord_while_budgetting, y_coord_while_budgetting, 
                metric_values, mode, obstacle_coords,  metric_to_overlay,
                grid_resolution, 
                spread_of_obstacles,
                cbar_min,
                cbar_max,
                x_input_bound=x_input_bound, 
                y_input_bound=y_input_bound,
                larger_is_better = larger_is_better_predicate, show_y_ticks=show_y_ticks
        )


"""

# plot velocity, visbility
metrics_to_draw = ["vel_mag_while_budgetting", "closest_unknown_distance", "sensor_to_actuation_time_budget_to_enforce"]
for metric_to_overlay in metrics_to_draw:
    if (ctr == 0):
        show_y_ticks = True
    else:
        show_y_ticks = False
    metric_values = result_dic[metric_to_overlay]
    larger_is_better_predicate = True
    if metric_to_overlay in ["insertScanLatency"]:
        larger_is_better_predicate = False
    space_overlay(x_coord_while_budgetting, y_coord_while_budgetting, 
            metric_values, obstacle_coords,  metric_to_overlay,
            grid_resolution, 
            spread_of_obstacles,
            x_input_bound=x_input_bound, 
            y_input_bound=y_input_bound,
            larger_is_better = larger_is_better_predicate, show_y_ticks=show_y_ticks
    )
    ctr +=1;






# plot static
#static_value_dict[metric_to_overlay]

# plot resolution, volume



# plot velocity, visbility
"""
