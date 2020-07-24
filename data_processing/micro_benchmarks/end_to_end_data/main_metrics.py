import matplotlib.pyplot as plt
import json
import sys
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.mplot3d import Axes3D 
sys.path.append('../../common_utils')
from data_parsing import *
import seaborn as sns
import argparse
import os

col = sns.color_palette("Paired", 111111)

parser = argparse.ArgumentParser()
parser.add_argument('datafile_path')
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
 
# data to collect (pass in the variables that you  are interested in)
# PS: always keep experiement_number inf metrics_to_collect_easy.
# PS: all the other parameters (Except mission_status and collision_count should be added to metrics_to_collect_easy (not hard)
metrics_to_collect_easy = [
        "PeakCongestion", "SpreadOfObstacles", "GapSize", "GoalDistance",
        "flight_time", "distance_travelled", "total_energy_consumed",
        "AverageOfObsDistances", "StddevOfObsDistances",
        "experiment_number", 
]
metrics_to_collect_hard = ["mission_status", "collision_count",
        "planning_ctr", "runtime_failure_ctr", "traj_gen_failure_ctr"
]

# parse  data using parse_stat_file_flattened
result_dict = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

# we generate a measure from spread of obstacles and peak
# congestion at the moment, but later this will come straight
# from the json file
result_dict["DifficultyMeasure"] = \
        np.multiply(result_dict["PeakCongestion"], result_dict["SpreadOfObstacles"])

# filter data based on a key value (the out put is a dictionary that is filtered)
result_dict_filtered_based_on_mission_status = filter_based_on_key_value(result_dict, "mission_status", 1.0, "in")
# filter again. So obviously you can chain filter stuff
result_dict_filtered_based_on_mission_status_and_collision_count = filter_based_on_key_value(result_dict_filtered_based_on_mission_status, "collision_count", 0.0, "in")

mission_status = result_dict["mission_status"]
mission_status_filtered_based_on_mission_status  = result_dict_filtered_based_on_mission_status["mission_status"]
mission_status_filtered_based_on_mission_status_and_filtered_count = result_dict_filtered_based_on_mission_status_and_collision_count["mission_status"]


print(len(mission_status))
print(len(mission_status_filtered_based_on_mission_status))
print(len(mission_status_filtered_based_on_mission_status_and_filtered_count))

# getting filtered variables to be graphed
peak_congestion_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["PeakCongestion"]
spread_of_obstacles_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["SpreadOfObstacles"]
goal_distance_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["GoalDistance"]
#difficulty_measure_based_on_mission_status_and_filtered_count = \
#        result_dict_filtered_based_on_mission_status_and_collision_count["DifficultyMeasure"]
mission_time_filtered_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["flight_time"]
total_distance_filtered_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["distance_travelled"]
mission_energy_filtered_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["total_energy_consumed"]
planning_ctr_filtered_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["planning_ctr"]
runtime_failure_ctr_filtered_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["runtime_failure_ctr"]
traj_gen_failure_ctr_filtered_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["traj_gen_failure_ctr"]

average_velocity_filtered_based_on_mission_status_and_filtered_count = \
        list(np.divide(total_distance_filtered_based_on_mission_status_and_filtered_count,
            mission_time_filtered_based_on_mission_status_and_filtered_count))

average_obs_dist_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["AverageOfObsDistances"]

stddev_obs_dist_based_on_mission_status_and_filtered_count = \
        result_dict_filtered_based_on_mission_status_and_collision_count["StddevOfObsDistances"]

def chain_filter_in(result_dict, filter_in_dict):
    curr_dict = result_dict
    for key in filter_in_dict.keys():
        curr_dict = filter_based_on_key_value(curr_dict, key, filter_in_dict[key], "in")
    return curr_dict

def subplot_2d(x, y, axes_labels, ax_inst, colors=[]):
    if colors == []:
        colors = np.array([0, 0, 1])
    # the atleast 2D is for scatter to stop complaining
    # this will not work with ax.plot
    # ref: https://stackoverflow.com/a/60416579
    ax_inst.scatter(x, y, c=colors)#, alpha=0.5)
    ax_inst.set(xlabel=axes_labels[0], ylabel=axes_labels[1])
    ax_inst.grid(True)
    return ax_inst

def annotate_2d(x, y, annotations, ax_inst, prepend_str="a = "):
    for i, txt in enumerate(annotations):
        ax_inst.annotate(str(i) + ": " + prepend_str + str(txt), (x[i], y[i]))
    return ax_inst

# plotting different ys for same x axis
def combined_plot_2d_const_x_var_y(x, ys, axes, xlabel, ylabels, annotations=[], annotation_prepend_str="a = ", colors=[]):
    for i, y in enumerate(ys):
        ax = axes.flat[i]
        ax = subplot_2d(x, y, [xlabel, ylabels[i]], ax, colors)
        if annotations != []:
            ax = annotate_2d(x, y, annotations, ax, annotation_prepend_str)
    return axes

def combined_plot_2d_var_x_const_y(xs, y, axes, xlabels, ylabel, annotations=[], annotation_prepend_str="a = ", colors=[]):
    for i, x in enumerate(xs):
        ax = axes.flat[i]
        ax = subplot_2d(x, y, [xlabels[i], ylabel], ax, colors)
        if annotations != []:
            ax = annotate_2d(x, y, annotations, ax, annotation_prepend_str)
    return axes

### Comparing E2E stats across diff measures for goal dist 600

result_dict_filtered_based_on_mission_status_and_collision_count_600 = \
        filter_based_on_key_value(result_dict_filtered_based_on_mission_status_and_collision_count,
                "GoalDistance", 600.0, "in")

measure_fields = [
        "DifficultyMeasure",
        "AverageOfObsDistances",
        "StddevOfObsDistances"
]

measures = [result_dict_filtered_based_on_mission_status_and_collision_count_600[measure_field] for measure_field in measure_fields]

measure_labels = ["peak * spread", 
    "Average of obstacle distances",
    "Stddev of obstacle distances"
]
colors = np.random.rand(len(measures[0]))

fig, axes = plt.subplots(nrows=2, ncols=2)
axes = combined_plot_2d_var_x_const_y(
        measures,
        result_dict_filtered_based_on_mission_status_and_collision_count_600["flight_time"],
        axes,
        measure_labels,
        "Mission time",
        result_dict_filtered_based_on_mission_status_and_collision_count_600["GoalDistance"],
        "d = ",
        colors=colors
)

fig.set_size_inches(12, 12)
fig.suptitle("Difficulty measures across GoalDistance = 600", fontsize=12)
fig.tight_layout()
output_file = "end_to_end_stats_600" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

### Comparing E2E stats across diff measures for goal dist 900

result_dict_filtered_based_on_mission_status_and_collision_count_900 = \
        filter_based_on_key_value(result_dict_filtered_based_on_mission_status_and_collision_count,
                "GoalDistance", 900.0, "in")

measure_fields = [
        "DifficultyMeasure",
        "AverageOfObsDistances",
        "StddevOfObsDistances"
]

measures = [result_dict_filtered_based_on_mission_status_and_collision_count_900[measure_field] for measure_field in measure_fields]

measure_labels = ["peak * spread", 
    "Average of obstacle distances",
    "Stddev of obstacle distances"
]
colors = np.random.rand(len(measures[0]))

fig, axes = plt.subplots(nrows=2, ncols=2)
axes = combined_plot_2d_var_x_const_y(
        measures,
        result_dict_filtered_based_on_mission_status_and_collision_count_900["flight_time"],
        axes,
        measure_labels,
        "Mission time",
        result_dict_filtered_based_on_mission_status_and_collision_count_900["GoalDistance"],
        "d = ",
        colors=colors
)

fig.set_size_inches(12, 12)
fig.suptitle("Difficulty measures across GoalDistance = 900", fontsize=12)
fig.tight_layout()
output_file = "end_to_end_stats_900" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

# for graphing the same x and y fields across the same values of filter_in_key
# eg: if you want to graph SpreadOfObstacles against flight_time for 4
# values of GoalDistance
def combined_plot_2d_filter_in(result_dict, x_field, y_field, filter_in_key, filter_in_vals, 
        xlabel, ylabel, annotation_field="", xlabel_append_str="", plot_name="", colors=[]):

    # not supporting more than 4 values for filter_in_key so that setting
    # sane subplot dimensions is easier
    len_filter_in_vals = len(filter_in_vals)
    assert(len_filter_in_vals <= 4)

    if len_filter_in_vals == 1:
        nrows = 1
        ncols = 1
    elif len_filter_in_vals == 2:
        nrows = 1
        ncols = 1
    else:
        nrows = 2
        ncols = 2

    fig, axes = plt.subplots(nrows=nrows, ncols=ncols)

    for i, filter_in_val in enumerate(filter_in_vals):
        result_dict_filtered = filter_based_on_key_value(result_dict, filter_in_key, filter_in_val, "in")

        x = result_dict_filtered[x_field]
        y = result_dict_filtered[y_field]

        if annotation_field != "":
            annotations = result_dict_filtered[annotation_field]

        if xlabel_append_str == "":
            xlabel_append_const_str = "(" + str(filter_in_key) + " = " + str(filter_in_val) + ")"

        ax = axes.flat[i]
        ax = subplot_2d(x, y, [xlabel + xlabel_append_const_str, ylabel], ax, colors)
        
        if annotation_field != "":
            ax = annotate_2d(x, y, annotations, ax, annotation_field[0] + "= ")

    if plot_name == "":
        plot_name = x_field.replace(" ", "_") + "_" + \
                y_field.replace(" ", "_") + "_" + \
                "const_" + filter_in_key.replace(" ", "_")

    fig.set_size_inches(12, 12)
    fig.tight_layout()
    output_file = plot_name + ".png"
    fig.savefig(result_folder + "/" + output_file)
    plt.close(fig)

### graph spread and flight time for set of goal dist values

goal_dist_values = [600.0, 900.0]

combined_plot_2d_filter_in(result_dict_filtered_based_on_mission_status_and_collision_count,
        "SpreadOfObstacles", "flight_time", "GoalDistance", goal_dist_values,
        "Spread", "Flight time", annotation_field="PeakCongestion"
)

combined_plot_2d_filter_in(result_dict_filtered_based_on_mission_status_and_collision_count,
        "PeakCongestion", "flight_time", "GoalDistance", goal_dist_values,
        "Peak", "Flight time", annotation_field="SpreadOfObstacles"
)

### Counter stats

#fig, axes = plt.subplots(nrows=2, ncols=2)
#metrics = [
#        planning_ctr_filtered_based_on_mission_status_and_filtered_count,
#        runtime_failure_ctr_filtered_based_on_mission_status_and_filtered_count,
#        traj_gen_failure_ctr_filtered_based_on_mission_status_and_filtered_count,
#        average_velocity_filtered_based_on_mission_status_and_filtered_count
#]
#metric_labels = [
#        "Total number of plans",
#        "Number of runtime failures",
#        "Number of planner failures",
#        "Average velocity"
#]
#
#axes = combined_plot_2d_const_x_var_y(
#        difficulty_measure_based_on_mission_status_and_filtered_count,
#        metrics,
#        goal_distance_based_on_mission_status_and_filtered_count,
#        axes,
#        "peak * spread",
#        metric_labels,
#        "d = "
#)

fig.tight_layout()
output_file = "ctr_stats" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)


### testing 3D plot

#def subplot_3d(x, y, z, axes_labels, ax_inst):
#    ax_inst.scatter(x, y, z, 'bo')
#    ax_inst.set(xlabel=axes_labels[0], ylabel=axes_labels[1], zlabel=axes_labels[2])
#    ax_inst.grid(True)
#    return ax_inst
#
#def combined_plot_3d(x, y, zs, axes, xlabel, ylabel, zlabels):
#    for i, ax in enumerate(axes.flat):
#        ax = subplot_3d(x, y, zs[i], [xlabel, ylabel, zlabels[i]], ax)
#    return axes
#    
#fig = plt.figure(figsize=(12,6))
#ax1 = fig.add_subplot(1, 2, 1, projection='3d')
#ax2 = fig.add_subplot(1, 2, 2, projection='3d')
#
#axes = combined_plot_3d(
#        spread_of_obstacles_based_on_mission_status_and_filtered_count,
#        goal_distance_based_on_mission_status_and_filtered_count,
#        metrics,
#        np.array([ax1, ax2]),
#        "Spread of obstacles",
#        "Distance to goal",
#        ["Mission time", "Mission energy"]
#)
#
#fig.tight_layout()
#output_file = "end_to_end_stats_3d" + ".png"
#fig.savefig(result_folder + "/" + output_file)
#plt.close(fig)
#
#fig = plt.figure(figsize=(12,12))
#ax1 = fig.add_subplot(2, 2, 1, projection='3d')
#ax2 = fig.add_subplot(2, 2, 2, projection='3d')
#ax3 = fig.add_subplot(2, 2, 3, projection='3d')
#ax4 = fig.add_subplot(2, 2, 4, projection='3d')
#
#metrics = [
#        planning_ctr_filtered_based_on_mission_status_and_filtered_count,
#        runtime_failure_ctr_filtered_based_on_mission_status_and_filtered_count,
#        traj_gen_failure_ctr_filtered_based_on_mission_status_and_filtered_count,
#        average_velocity_filtered_based_on_mission_status_and_filtered_count
#]
#metric_labels = [
#        "Total number of plans",
#        "Number of runtime failures",
#        "Number of planner failures",
#        "Average velocity"
#]
#
#axes = combined_plot_3d(
#        spread_of_obstacles_based_on_mission_status_and_filtered_count,
#        goal_distance_based_on_mission_status_and_filtered_count,
#        metrics,
#        np.array([ax1, ax2, ax3, ax4]),
#        "Spread of obstacles",
#        "Distance to goal",
#        metric_labels
#)
#
#fig.tight_layout()
#output_file = "ctr_stats_3d" + ".png"
#fig.savefig(result_folder + "/" + output_file)
#plt.close(fig)

