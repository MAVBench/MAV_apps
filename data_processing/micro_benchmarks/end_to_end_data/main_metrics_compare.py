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
parser.add_argument('--d', type=str)
parser.add_argument('--s', type=str)
args = parser.parse_args()

def dir_path(path):
    if not os.path.isfile(path):
        print("Couldn't find datafile!")
        exit(-1)
    else:
        return os.path.dirname(os.path.realpath(path))

# we dump resulting graphs in same dir as datafile

input_filepath = args.d
static_filepath = args.s
result_folder = dir_path(input_filepath)
 
# data to collect (pass in the variables that you  are interested in)
# PS: always keep experiement_number inf metrics_to_collect_easy.
# PS: all the other parameters (Except mission_status and collision_count should be added to metrics_to_collect_easy (not hard)
metrics_to_collect_easy = [
        "PeakCongestion", "SpreadOfObstacles", "GapSize", "GoalDistance",
        "flight_time", "distance_travelled", "total_energy_consumed",
        "AverageOfObsDistances", "StddevOfObsDistances",
        "experiment_number"
]
metrics_to_collect_hard = ["mission_status", "collision_count",
        "planning_ctr", "runtime_failure_ctr", "traj_gen_failure_ctr"
]

# parse  data using parse_stat_file_flattened
dynamic_result_dict = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

static_result_dict = parse_stat_file_flattened(static_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

# we generate a measure from spread of obstacles and peak
# congestion at the moment, but later this will come straight
# from the json file
dynamic_result_dict["DifficultyMeasure"] = \
        np.multiply(dynamic_result_dict["PeakCongestion"], dynamic_result_dict["SpreadOfObstacles"])
static_result_dict["DifficultyMeasure"] = \
        np.multiply(static_result_dict["PeakCongestion"], static_result_dict["SpreadOfObstacles"])

dynamic_result_dict["average_velocity"] = \
        np.divide(dynamic_result_dict["distance_travelled"], dynamic_result_dict["flight_time"])
static_result_dict["average_velocity"] = \
        np.divide(static_result_dict["distance_travelled"], static_result_dict["flight_time"])

dynamic_result_df = pd.DataFrame(dynamic_result_dict)
static_result_df = pd.DataFrame(static_result_dict)

dynamic_valid_df = dynamic_result_df.loc[(dynamic_result_df['mission_status'] == 1.0) & (dynamic_result_df['collision_count'] == 0.0)]
static_valid_df = static_result_df.loc[(static_result_df['mission_status'] == 1.0) & (static_result_df['collision_count'] == 0.0)]

barchart_grouping = ["DifficultyMeasure", "GoalDistance"]

dyn_group = dynamic_valid_df.groupby(barchart_grouping)
static_group = static_valid_df.groupby(barchart_grouping)

#print("---------------------DYNAMIC RUNS GROUPED----------------")
#print(dyn_group.agg(['count']))
#print("---------------------STATIC RUNS GROUPED----------------")
#print(static_group.agg(['count']))

def grouped_barplot_metric(metric, ax):
    dyn_stats = dyn_group[metric].agg([np.mean, np.std])
    static_stats = static_group[metric].agg([np.mean, np.std])

    # number of groups on x axis
    ind = np.arange(dynamic_valid_df["GoalDistance"].nunique())
    bar_width = 0.08

    # multiplied by two because we plot each measure separately for static and dynamic
    n_bars = dynamic_valid_df["DifficultyMeasure"].nunique() * 2

    bar_xpos = []
    bar_xpos.append(ind)
    for bar in range(1, n_bars):
        bar_xpos.append([x + bar_width for x in bar_xpos[bar-1]])

    difficulty_measures = np.sort(dynamic_valid_df["DifficultyMeasure"].unique())

    # hack: static currently misses some data points from dynamic, which we'd
    # like to assign to NaNs to prevent graphing and indexing problems.
    # To fill them in, we create a dataframe which is all NaNs from dyn_stats
    # and replace them with whatever points we have in static
    static_stats_copy = dyn_stats.copy()
    for col in static_stats_copy.columns:
        static_stats_copy.values[:] = np.NaN
    static_stats_copy.update(static_stats)

    static_stats = static_stats_copy

    # Plotting dynamic first
    for i in range(n_bars//2):
        means = dyn_stats.loc[difficulty_measures[i], "mean"]
        stddevs = dyn_stats.loc[difficulty_measures[i], "std"]
        ax.bar(
                bar_xpos[i],
                means,
                color="C" + str(i+1),
                width=bar_width,
                yerr=stddevs,
                label="Dynamic, " + str(i+1)
        )

    # static with hatched bars
    for i in range(n_bars//2):
        means = static_stats.loc[difficulty_measures[i], "mean"]
        stddevs = static_stats.loc[difficulty_measures[i], "std"]
        label = "Static, " + str(i)
        ax.bar(
                bar_xpos[i + n_bars//2],
                means,
                color="C" + str(i+1),
                width=bar_width,
                yerr=stddevs,
                hatch='//',
                label="Static, " + str(i+1)
        )

    ax.set_ylabel(metric)
    ax.set_xlabel("GoalDistance")
    ax.set_xticks(ind + (n_bars//2 * bar_width - bar_width/2))
    ax.set_xticklabels(dynamic_valid_df["GoalDistance"].unique())
    ax.legend()

    return ax

fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(24, 30))

metrics = [
        "flight_time",
        "total_energy_consumed",
        "distance_travelled",
        "average_velocity"
]
for i, ax in enumerate(axes.flat):
    if i < len(metrics):
        ax = grouped_barplot_metric(metrics[i], ax)

output_file = "hatched_group_bar_chart" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

exit()

### Stacked plot for plans/replans

# Total plans = planning_success_ctr + runtime_failure_ctr + traj_gen_failure_ctr

dyn_stats = dyn_group.agg([np.mean, np.std])

print(dynamic_result_df)
print(static_result_df)
#print(dyn_stats["flight_time"]["mean"])
exit()

dyn_succ_stats = dyn_group["planning_success_ctr"].agg([np.mean, np.std])
static_succ_stats = static_group["planning_success_ctr"].agg([np.mean, np.std])

dyn_runf_stats = dyn_group["runtime_failure_ctr"].agg([np.mean, np.std])
static_runf_stats = static_group["runtime_failure_ctr"].agg([np.mean, np.std])

dyn_trajf_stats = dyn_group["traj_gen_failure_ctr"].agg([np.mean, np.std])
static_trajf_stats = static_group["traj_gen_failure_ctr"].agg([np.mean, np.std])

fig, ax = plt.subplots(figsize=(24, 8))

# number of groups on x axis
ind = np.arange(dynamic_valid_df["GoalDistance"].nunique())
bar_width = 0.08

# multiplied by two because we plot each measure separately for static and dynamic
n_bars = dynamic_valid_df["DifficultyMeasure"].nunique() * 2

bar_xpos = []
bar_xpos.append(ind)
for bar in range(1, n_bars):
    bar_xpos.append([x + bar_width for x in bar_xpos[bar-1]])

difficulty_measures = dynamic_valid_df["DifficultyMeasure"].unique()

# hack: static currently misses some data points from dynamic, which we'd
# like to assign to NaNs to prevent graphing and indexing problems.
# To fill them in, we create a dataframe which is all NaNs from dyn_stats
# and replace them with whatever points we have in static
static_stats_copy = dyn_stats.copy()
for col in static_stats_copy.columns:
    static_stats_copy.values[:] = np.NaN
static_stats_copy.update(static_stats)

static_stats = static_stats_copy

# Plotting dynamic first
for i in range(n_bars//2):
    means = dyn_stats.loc[difficulty_measures[i], "mean"]
    stddevs = dyn_stats.loc[difficulty_measures[i], "std"]
    ax.bar(
            bar_xpos[i],
            means,
            color="C" + str(i+1),
            width=bar_width,
            yerr=stddevs,
            label="Dynamic, " + str(i+1)
    )

# static with hatched bars
for i in range(n_bars//2):
    means = static_stats.loc[difficulty_measures[i], "mean"]
    stddevs = static_stats.loc[difficulty_measures[i], "std"]
    label = "Static, " + str(i)
    ax.bar(
            bar_xpos[i + n_bars//2],
            means,
            color="C" + str(i+1),
            width=bar_width,
            yerr=stddevs,
            hatch='//',
            label="Static, " + str(i+1)
    )

ax.set_ylabel(metric)
ax.set_xlabel("GoalDistance")
ax.set_xticks(ind + (n_bars//2 * bar_width - bar_width/2))
ax.set_xticklabels(dynamic_valid_df["GoalDistance"].unique())
ax.legend()

exit()

#################################################################

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
