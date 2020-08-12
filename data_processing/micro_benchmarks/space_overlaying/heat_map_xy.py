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
from EnvGenParser import EnvGenParser
from math import *
import json
import argparse
import os
import time
from collections import OrderedDict 

parser = argparse.ArgumentParser()
parser.add_argument('-d', '--datafile_path', type=str)
parser.add_argument('-s', '--staticfile_path', type=str)
parser.add_argument('-e', '--envfile_path', type=str)
parser.add_argument('-r', '--resultdir_path', type=str)
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
obs_stats_path = args.envfile_path
static_path = args.staticfile_path
if args.resultdir_path:
    result_folder = args.resultdir_path
else:
    result_folder = dir_path(input_filepath)

env_gen_parser = EnvGenParser(obs_stats_path)

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

def space_overlay(x,y, metric_values, obstacle_coords, title, static_value = 0, resolution = -1, spread_of_obstacles = -1, 
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
    space_mat_dynamic = np.empty(shape=(y_range, x_range))
    space_mat_dynamic [:] = np.nan # so that coords we don't have data for are greyed out
    space_mat_static = np.empty(shape=(y_range, x_range))
    space_mat_static [:] = np.nan # so that coords we don't have data for are greyed out


    # modify the title and cap at max 
    sensor_max_range = 25 
    max_metric_value = 1000000
    plot_title = title 
    if (title == "closest_unknown_distance"):
        max_metric_value = sensor_max_range
        plot_title = "Sensor Visibility (m)"
    elif (title== "sensor_to_actuation_time_budget_to_enforce"):
        max_metric_value = 15
        plot_title = "Time Budget (s)"
    elif (title== "pc_res"):
        plot_title = "Precision (Voxel Size in cm)"
    elif (title== "insertScanLatency"):
        plot_title = "Response Time (s)"
    elif (title== "vel_mag_while_budgetting"):
        plot_title = "Acutuation Velocity (m/s)"
    elif (title== "octomap_volume_integrated"):
        plot_title = "Space Volume Processed (m3)"


    # binning the points
    for i in range(0, len(x)):
        x_rounded = int(float(x[i])/resolution) + x_offset
        y_rounded = int(float(y[i])/resolution) + y_offset
        if x_rounded < x_range and y_rounded < y_range:
            space_mat_dynamic[y_rounded][x_rounded] = min(metric_values[i], max_metric_value)
            space_mat_static[y_rounded][x_rounded] = static_value
       
    # plotting 
    space_mat_dict =  OrderedDict()
    space_mat_dict ["static"] = space_mat_static
    space_mat_dict ["dynamic"] = space_mat_dynamic
    fig, axs = plt.subplots(1, 2)
    
    # overlaying obstacles on top of plot
    # we pass all the obstacle coords scaled by resolution since the x_bounds
    # and y_bounds are scaled as well
    obstacle_coords_scaled = np.array(obstacle_coords) / resolution
    obs_dict = extract_obstacles(obstacle_coords_scaled, x_bounds, y_bounds)
    n_obs = obs_dict["n_obstacles"]
    ctr = 0 
    plt.suptitle(plot_title) 
    for key in space_mat_dict.keys():
        axs[ctr].set_title(key, fontsize=10)
        ttl = axs[ctr].title
        #ttl.set_position([0.5, 1.05])
        for i in range(0, n_obs):
            origin = obs_dict['origin'][i]
            width = obs_dict['width'][i]
            height = obs_dict['height'][i]
            axs[ctr].add_patch(
                    patches.Rectangle(origin, width, height, color="black", linewidth=0)
                    )
        sns.set()

        colormap='RdYlGn'
        if not larger_is_better:
            # reverses colormap
            colormap += '_r'

        
        if key == "dynamic":
            cbar_predicate = True
        else:
            cbar_predicate = False
        sns.heatmap(space_mat_dict[key], cmap=colormap, cbar=cbar_predicate, ax=axs[ctr], square=True)
        #axs[ctr] = plt.gca()

        # setting ticks and tick positions on axes

        n_ticks = 15

        x_tick_locs = np.rint(np.linspace(x_bounds["min"], x_bounds["max"], n_ticks)) + x_offset
        y_tick_locs = np.rint(np.linspace(y_bounds["min"], y_bounds["max"], n_ticks)) + y_offset

        # scale back by resolution and round since we want absolute units on the labels
        x_tick_labels = np.rint(np.linspace(x_bounds["min"], x_bounds["max"], n_ticks)) * resolution
        y_tick_labels = np.rint(np.linspace(y_bounds["min"], y_bounds["max"], n_ticks)) * resolution

        axs[ctr].set_xticks(x_tick_locs)
        axs[ctr].set_yticks(y_tick_locs)
        axs[ctr].set_xticklabels(x_tick_labels)
        axs[ctr].set_yticklabels(y_tick_labels)
        axs[ctr].set_ylim(axs[ctr].get_ylim()[::-1])
        ctr +=1
    plt.savefig(result_folder + "/" + title)

def space_overlay_congestion_traj(x, y, obstacle_coords, title, resolution = -1, spread_of_obstacles = -1, 
        x_input_bound={"min":-1,"max":-1}, y_input_bound={"min":-1,"max":-1}, x_2=[], y_2=[]):

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
    congestion_mat = np.empty(shape=(y_range, x_range))

    # we use congestion as the metric
    grid_size = 10
    for i in range(x_range):
        # scaling back to actual coordinates
        actual_x = (i - x_offset) * resolution
        for j in range(y_range):
            actual_y = (j - y_offset) * resolution

            # now we check how many of the 9 grids surrounding the current
            # position have obstacles in them
            n_obs_surrounding = 0

            surrounding_grid_bound = [[actual_x + grid_size, actual_y + grid_size], [actual_x - grid_size, actual_y - grid_size]]
            grid_bound_top_right = surrounding_grid_bound[0]
            grid_bound_bottom_left = surrounding_grid_bound[1]

            for coord in obstacle_coords:
                top_right = coord[0]
                bottom_left = coord[1]

                top_right_in_bounds = grid_bound_bottom_left[0] <= top_right[0] <= grid_bound_top_right[0] and \
                        grid_bound_bottom_left[1] <= top_right[1] <= grid_bound_top_right[1]

                bottom_left_in_bounds = grid_bound_bottom_left[0] <= bottom_left[0] <= grid_bound_bottom_left[0] and \
                        grid_bound_bottom_left[1] <= bottom_left[1] <= grid_bound_bottom_left[1]

                if top_right_in_bounds or bottom_left_in_bounds:
                    n_obs_surrounding += 1


            #for grid_i in [-1, 0, 1]:
            #    for grid_j in [-1, 0, 1]:
            #        curr_grid_center_x = actual_x + grid_i * grid_size
            #        curr_grid_center_y = actual_y + grid_j * grid_size

            #        for coord in obstacle_coords:
            #            top_right = coord[0]
            #            bottom_left = coord[1]

            #            #top_right_in_bounds = curr_grid_center_x - grid_size/2 <= top_right[0] <= curr_grid_center_x and \
            #            #        curr_grid_center_y - grid_size/2 <= top_right[1] <= curr_grid_center_y

            #            #bottom_left_in_bounds = curr_grid_center_x <= bottom_left[0] <= curr_grid_center_x + grid_size/2 and \
            #            #        curr_grid_center_y <= bottom_left[1] <= curr_grid_center_y + grid_size/2

            #            top_right_in_bounds = curr_grid_center_x - grid_size/2 <= top_right[0] <= curr_grid_center_x and \
            #                    curr_grid_center_y - grid_size/2 <= top_right[1] <= curr_grid_center_y

            #            bottom_left_in_bounds = curr_grid_center_x <= bottom_left[0] <= curr_grid_center_x + grid_size/2 and \
            #                    curr_grid_center_y <= bottom_left[1] <= curr_grid_center_y + grid_size/2

            #            if top_right_in_bounds or bottom_left_in_bounds:
            #                n_obs_surrounding += 1

            congestion_mat[j][i] = n_obs_surrounding / 9

    # plotting 
    space_mat_dict =  OrderedDict()
    space_mat_dict ["static"] = congestion_mat
    space_mat_dict ["dynamic"] = congestion_mat

    fig, axs = plt.subplots(1, 2)
    plt.suptitle("Map congestion and trajectory") 

    ctr = 0
    for key in space_mat_dict.keys():
        axs[ctr].set_title(key, fontsize=10)
        # overlaying obstacles on top of plot
        # we pass all the obstacle coords scaled by resolution since the x_bounds
        # and y_bounds are scaled as well
        #obstacle_coords_scaled = np.array(obstacle_coords) / resolution
        #obs_dict = extract_obstacles(obstacle_coords_scaled, x_bounds, y_bounds)
        #n_obs = obs_dict["n_obstacles"]
        #
        #for i in range(0, n_obs):
        #    origin = obs_dict['origin'][i]
        #    width = obs_dict['width'][i]
        #    height = obs_dict['height'][i]
        #    ax.add_patch(
        #            patches.Rectangle(origin, width, height, color="black", linewidth=0)
        #            )
        
        obstacle_zones = env_gen_parser.extract_zones()
        scaled_obstacle_zones = np.array(obstacle_zones) / resolution
        scaled_zone_origins = []
        scaled_zone_widths = []
        scaled_zone_heights = []

        for zone_bound in scaled_obstacle_zones:
            zone_bound_top_right = zone_bound[0]
            zone_bound_bottom_left = zone_bound[1]
            scaled_zone_origins.append([zone_bound_bottom_left[0] + x_offset, zone_bound_bottom_left[1] + y_offset])
            scaled_zone_widths.append(zone_bound_top_right[0] - zone_bound_bottom_left[0])
            scaled_zone_heights.append(zone_bound_top_right[1] - zone_bound_bottom_left[1])

        # drawing zones
        for i in range(len(scaled_obstacle_zones)):
            axs[ctr].add_patch(
                    patches.Rectangle(
                        scaled_zone_origins[i],
                        scaled_zone_widths[i],
                        scaled_zone_heights[i],
                        facecolor='none', edgecolor='red', linewidth=3
                    )
            )

        sns.set()

        if key == "dynamic":
            cbar_predicate = True
        else:
            cbar_predicate = False
        colormap='Reds'

        sns.heatmap(space_mat_dict[key], cmap=colormap, ax=axs[ctr], cbar=cbar_predicate, square=True)

        # filtering zeros
        x = [i for i in x if i != 0]
        y = [i for i in y if i != 0]

        # drawing trajectory
        traj_x_coords_scaled = (np.array(x) / resolution) + x_offset
        traj_y_coords_scaled = (np.array(y) / resolution) + y_offset

        # select elements to plot else too many arrows
        markevery = 10

        if key == "static" and x_2 and y_2:
            traj_x_coords_scaled = (np.array(x_2) / resolution) + x_offset
            traj_y_coords_scaled = (np.array(y_2) / resolution) + y_offset

        if key == "dynamic":
            axs[ctr].plot(traj_x_coords_scaled, traj_y_coords_scaled, 'k-^', label="Dynamic trajectory", markevery=markevery)
        elif key == "static":
            axs[ctr].plot(traj_x_coords_scaled, traj_y_coords_scaled, 'b-^', label="Static trajectory", markevery=markevery)

        #traj_x_origin = []
        #traj_y_origin = []
        #traj_x_dir = []
        #traj_y_dir = []
        #for i in range(len(traj_x_coords_scaled)):
        #    if i < (len(traj_x_coords_scaled) - markevery//2) and i % markevery == 0:
        #        traj_x_origin.append(traj_x_coords_scaled[i])
        #        traj_y_origin.append(traj_y_coords_scaled[i])
        #        traj_x_dir.append(traj_x_coords_scaled[i+markevery//2] - traj_x_coords_scaled[i])
        #        traj_y_dir.append(traj_y_coords_scaled[i+markevery//2] - traj_y_coords_scaled[i])

        #traj_x_origin = np.array(traj_x_coords_scaled[:-1])
        #traj_y_origin = np.array(traj_y_coords_scaled[:-1])
        #traj_x_dir = traj_x_coords_scaled[1:] - traj_x_origin
        #traj_y_dir = traj_y_coords_scaled[1:] - traj_y_origin

        #mag = np.sqrt((np.square(traj_x_dir) + np.square(traj_y_dir)))
        #traj_x_dir = np.divide(traj_x_dir, mag)
        #traj_y_dir = np.divide(traj_y_dir, mag)


        #ax.quiver(traj_x_origin, traj_y_origin, traj_x_dir, traj_y_dir)

        # setting ticks and tick positions on axes

        n_ticks = 15

        x_tick_locs = np.rint(np.linspace(x_bounds["min"], x_bounds["max"], n_ticks)) + x_offset
        y_tick_locs = np.rint(np.linspace(y_bounds["min"], y_bounds["max"], n_ticks)) + y_offset

        # scale back by resolution and round since we want absolute units on the labels
        x_tick_labels = np.rint(np.linspace(x_bounds["min"], x_bounds["max"], n_ticks)) * resolution
        y_tick_labels = np.rint(np.linspace(y_bounds["min"], y_bounds["max"], n_ticks)) * resolution

        axs[ctr].set_xticks(x_tick_locs)
        axs[ctr].set_yticks(y_tick_locs)
        axs[ctr].set_xticklabels(x_tick_labels)
        axs[ctr].set_yticklabels(y_tick_labels)
        axs[ctr].set_ylim(axs[ctr].get_ylim()[::-1])
        
        ctr += 1

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
        "obs_dist_statistics_min",
        "insertScanLatency",
	"closest_unknown_distance",

]

# parse  data
result_dic = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

env_gen_stats = env_gen_parser.getEnvGenStats()
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
        "obs_dist_statistics_min",
        "insertScanLatency",
	"closest_unknown_distance"
]

x_input_bound = {"min": bounds[0], "max": bounds[1]}
y_input_bound = {"min": bounds[2], "max": bounds[3]}

static_value_dict = {}
static_value_dict["vel_mag_while_budgetting"] = 3.2
static_value_dict["sensor_to_actuation_time_budget_to_enforce"] = 2 
static_value_dict["octomap_volume_integrated"] = 46000
static_value_dict["obs_dist_statistics_min"] = .3
static_value_dict["pc_res"] = .6
static_value_dict["insertScanLatency"] = .7 
static_value_dict["closest_unknown_distance"] = 8 

for metric_to_overlay in metrics_to_overlay:
    metric_values = result_dic[metric_to_overlay]
    larger_is_better_predicate = True
    if metric_to_overlay in ["insertScanLatency"]:
        larger_is_better_predicate = False
        space_overlay(x_coord_while_budgetting, y_coord_while_budgetting, 
                metric_values, obstacle_coords,  metric_to_overlay, static_value_dict[metric_to_overlay],
                grid_resolution, 
                spread_of_obstacles,
                x_input_bound=x_input_bound, 
                y_input_bound=y_input_bound,
                larger_is_better= larger_is_better_predicate
        )

static_dict = parse_stat_file_flattened(static_path, [], ["x_coord_while_budgetting", "y_coord_while_budgetting"])

# for congestion and trajectory
space_overlay_congestion_traj(x_coord_while_budgetting, y_coord_while_budgetting, 
        obstacle_coords, "map_congestion_trajectory",
        1,
        spread_of_obstacles,
        x_input_bound=x_input_bound, 
        y_input_bound=y_input_bound,
        #x_2=static_dict["x_coord_while_budgetting"],
        #y_2=static_dict["y_coord_while_budgetting"]
)

