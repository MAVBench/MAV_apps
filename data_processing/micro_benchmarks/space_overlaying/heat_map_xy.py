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

#time.sleep(5)

parser = argparse.ArgumentParser()
parser.add_argument('-d', '--datafile_path', type=str)
parser.add_argument('-e', '--envfile_path', type=str)
parser.add_argument('-b', '--bounds', nargs='+', type=int)
args = parser.parse_args()

def dir_path(path):
    if not os.path.isfile(path):
        print("Couldn't find datafile!")
        exit(-1)
    else:
        return os.path.dirname(os.path.realpath(path))

# we dump resulting graphs in same dir as datafile

input_filepath = args.datafile_path
result_folder = dir_path(input_filepath)
obs_stats_path = args.envfile_path

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

def space_overlay(x,y, metric_values, obstacle_coords, title, resolution = -1, spread_of_obstacles = -1, 
        x_input_bound={"min":-1,"max":-1}, y_input_bound = {"min":-1,"max":-1}, larger_is_better=True):

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
    space_mat = np.empty(shape=(y_range, x_range))
    space_mat[:] = np.nan # so that coords we don't have data for are greyed out

    # binning the points
    for i in range(0, len(x)):
        x_rounded = int(float(x[i])/resolution) + x_offset
        y_rounded = int(float(y[i])/resolution) + y_offset
        if x_rounded < x_range and y_rounded < y_range:
            space_mat[y_rounded][x_rounded] = metric_values[i]
       
    # plotting 
    fig, ax = plt.subplots(figsize=(12,12))
    plt.title(title, fontsize=18)
    ttl = ax.title
    ttl.set_position([0.5, 1.05])

    # overlaying obstacles on top of plot

    # we pass all the obstacle coords scaled by resolution since the x_bounds
    # and y_bounds are scaled as well
    obstacle_coords_scaled = np.array(obstacle_coords) / resolution
    obs_dict = extract_obstacles(obstacle_coords_scaled, x_bounds, y_bounds)
    n_obs = obs_dict["n_obstacles"]

    for i in range(0, n_obs):
        origin = obs_dict['origin'][i]
        width = obs_dict['width'][i]
        height = obs_dict['height'][i]
        ax.add_patch(
                patches.Rectangle(origin, width, height, color="black", linewidth=0)
                )

    sns.set()

    colormap='RdYlGn'
    if not larger_is_better:
        # reverses colormap
        colormap += '_r'

    sns.heatmap(space_mat, cmap=colormap, ax=ax, square=True)
    ax = plt.gca()

    # setting ticks and tick positions on axes

    n_ticks = 15

    x_tick_locs = np.rint(np.linspace(x_bounds["min"], x_bounds["max"], n_ticks)) + x_offset
    y_tick_locs = np.rint(np.linspace(y_bounds["min"], y_bounds["max"], n_ticks)) + y_offset

    # scale back by resolution and round since we want absolute units on the labels
    x_tick_labels = np.rint(np.linspace(x_bounds["min"], x_bounds["max"], n_ticks)) * resolution
    y_tick_labels = np.rint(np.linspace(y_bounds["min"], y_bounds["max"], n_ticks)) * resolution

    ax.set_xticks(x_tick_locs)
    ax.set_yticks(y_tick_locs)
    ax.set_xticklabels(x_tick_labels)
    ax.set_yticklabels(y_tick_labels)

    ax.set_ylim(ax.get_ylim()[::-1])

    plt.savefig(result_folder + "/" + title)
    

# data to collect
metrics_to_collect_easy = []
metrics_to_collect_hard = [
        "x_coord_while_budgetting", 
        "y_coord_while_budgetting", 
        "vel_mag_while_budgetting", 
        "sensor_to_actuation_time_budget_to_enforce", 
        "octomap_volume_integrated",
        "pc_res",
        "obs_dist_statistics_min"
]

# parse  data
result_dic = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

env_gen_stats = {}
with open(obs_stats_path, "r") as obs_json_file:
    env_gen_stats = json.load(obs_json_file)

# adding an offset to show the entire map
spread_of_obstacles = env_gen_stats["SpreadOfObstacles"]
obstacle_coords = env_gen_stats["ObstacleList"]
grid_resolution = 2

x_coord_while_budgetting = result_dic["x_coord_while_budgetting"]
y_coord_while_budgetting = result_dic["y_coord_while_budgetting"]
metrics_to_overlay = [
        "vel_mag_while_budgetting", 
        "sensor_to_actuation_time_budget_to_enforce", 
        "octomap_volume_integrated",
        "pc_res",
        "obs_dist_statistics_min"
]

x_input_bound = {"min": bounds[0], "max": bounds[1]}
y_input_bound = {"min": bounds[2], "max": bounds[3]}

for metric_to_overlay in metrics_to_overlay:
    metric_values = result_dic[metric_to_overlay]
    space_overlay(x_coord_while_budgetting, y_coord_while_budgetting, 
            metric_values, obstacle_coords,  metric_to_overlay, grid_resolution, 
            spread_of_obstacles,
            x_input_bound=x_input_bound, 
            y_input_bound=y_input_bound,
            #larger_is_better=False
    )
