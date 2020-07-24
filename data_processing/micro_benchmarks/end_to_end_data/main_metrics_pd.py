import matplotlib.pyplot as plt
import json
import sys
import numpy as np
import pandas as pd
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

result_df = pd.DataFrame(result_dict)

valid_df = result_df.loc[(result_df['mission_status'] == 1.0) & (result_df['collision_count'] == 0.0)]

print(len(result_df))
print(len(valid_df))

df_600 = result_df.loc[(result_df['GoalDistance'] == 600.0) & (result_df['PeakCongestion'] == 0.3)]
df_1200 = result_df.loc[(result_df['GoalDistance'] == 1200.0) & (result_df['PeakCongestion'] == 0.3)]

print(df_600)
print(df_1200)

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
        if len(annotations) != 0:
            ax = annotate_2d(x, y, annotations, ax, annotation_prepend_str)
    return axes

### Comparing E2E stats across diff measures for goal dist 600

df_600 = valid_df.loc[(valid_df['GoalDistance'] == 600.0)]

measure_fields = [
        "DifficultyMeasure",
        "AverageOfObsDistances",
        "StddevOfObsDistances"
]

measures = [df_600[measure_field].to_numpy() for measure_field in measure_fields]

measure_labels = ["peak * spread", 
    "Average of obstacle distances",
    "Stddev of obstacle distances"
]
colors = np.random.rand(len(measures[0]))

fig, axes = plt.subplots(nrows=2, ncols=2)
axes = combined_plot_2d_var_x_const_y(
        measures,
        df_600["flight_time"].to_numpy(),
        axes,
        measure_labels,
        "Mission time",
        df_600["GoalDistance"].to_numpy(),
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

df_900 = valid_df.loc[(valid_df['GoalDistance'] == 900.0)]

measure_fields = [
        "DifficultyMeasure",
        "AverageOfObsDistances",
        "StddevOfObsDistances"
]

measures = [df_900[measure_field].to_numpy() for measure_field in measure_fields]

measure_labels = ["peak * spread", 
    "Average of obstacle distances",
    "Stddev of obstacle distances"
]
colors = np.random.rand(len(measures[0]))

fig, axes = plt.subplots(nrows=2, ncols=2)
axes = combined_plot_2d_var_x_const_y(
        measures,
        df_900["flight_time"].to_numpy(),
        axes,
        measure_labels,
        "Mission time",
        df_900["GoalDistance"].to_numpy(),
        "d = ",
        colors=colors
)

fig.set_size_inches(12, 12)
fig.suptitle("Difficulty measures across GoalDistance = 900", fontsize=12)
fig.tight_layout()
output_file = "end_to_end_stats_900" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

### Comparing E2E stats across diff measures for goal dist 1200

df_1200 = valid_df.loc[(valid_df['GoalDistance'] == 1200.0)]

measure_fields = [
        "DifficultyMeasure",
        "AverageOfObsDistances",
        "StddevOfObsDistances"
]

measures = [df_1200[measure_field].to_numpy() for measure_field in measure_fields]

measure_labels = ["peak * spread", 
    "Average of obstacle distances",
    "Stddev of obstacle distances"
]
colors = np.random.rand(len(measures[0]))

fig, axes = plt.subplots(nrows=2, ncols=2)
axes = combined_plot_2d_var_x_const_y(
        measures,
        df_1200["flight_time"].to_numpy(),
        axes,
        measure_labels,
        "Mission time",
        df_1200["GoalDistance"].to_numpy(),
        "d = ",
        colors=colors
)

fig.set_size_inches(12, 12)
fig.suptitle("Difficulty measures across GoalDistance = 1200", fontsize=12)
fig.tight_layout()
output_file = "end_to_end_stats_1200" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

### Boxplots for peak * spread

df_600 = valid_df.loc[(valid_df['GoalDistance'] == 600.0)]
df_900 = valid_df.loc[(valid_df['GoalDistance'] == 900.0)]
df_1200 = valid_df.loc[(valid_df['GoalDistance'] == 1200.0)]

print(len(df_600))
print(len(df_900))
print(len(df_1200))

fig, axes = plt.subplots(nrows=2, ncols=2)
axes[0][0] = sns.boxplot(
        x="DifficultyMeasure",
        y="flight_time",
        data=df_600,
        ax=axes[0][0]
).set(xlabel="peak * spread for goal distance = 600")
axes[0][1] = sns.boxplot(
        x="DifficultyMeasure",
        y="flight_time",
        data=df_900,
        ax=axes[0][1]
).set(xlabel="peak * spread for goal distance = 900")
axes[1][0] = sns.boxplot(
        x="DifficultyMeasure",
        y="flight_time",
        data=df_1200,
        ax=axes[1][0]
).set(xlabel="peak * spread for goal distance = 1200")

fig.set_size_inches(12, 12)
fig.suptitle("Peak * Spread for different goal distances", fontsize=12)
fig.tight_layout()
output_file = "difficulty_measure_box" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

### Grouped bar plots

valid_df["average_velocity"] = \
        valid_df["distance_travelled"] / valid_df["flight_time"]

fig, axes = plt.subplots(nrows=4, ncols=1)
axes[0] = sns.barplot(
        x="GoalDistance",
        y="flight_time",
        hue="DifficultyMeasure",
        data=valid_df,
        ax=axes[0]
)
axes[1] = sns.barplot(
        x="GoalDistance",
        y="total_energy_consumed",
        hue="DifficultyMeasure",
        data=valid_df,
        ax=axes[1]
)
axes[2] = sns.barplot(
        x="GoalDistance",
        y="distance_travelled",
        hue="DifficultyMeasure",
        data=valid_df,
        ax=axes[2]
)
axes[3] = sns.barplot(
        x="GoalDistance",
        y="average_velocity",
        hue="DifficultyMeasure",
        data=valid_df,
        ax=axes[3]
)

fig.set_size_inches(12, 16)
fig.suptitle("Metrics for different goal distances", fontsize=12)
fig.tight_layout()
output_file = "difficulty_measure_grouped_dist" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)



exit()

###################################################################

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
