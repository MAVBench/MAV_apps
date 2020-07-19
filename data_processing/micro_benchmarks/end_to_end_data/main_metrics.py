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
col = sns.color_palette("Paired", 111111)

result_folder = "./data_2"
input_file_name = "saturday_morning.json"
input_filepath = result_folder + "/" + input_file_name
 
# data to collect (pass in the variables that you  are interested in)
# PS: always keep experiement_number inf metrics_to_collect_easy.
# PS: all the other parameters (Except mission_status and collision_count should be added to metrics_to_collect_easy (not hard)
metrics_to_collect_easy = ["experiment_number", 
        "PeakCongestion", "SpreadOfObstacles", "GapSize", "GoalDistance",
        "flight_time", "distance_travelled", "total_energy_consumed"
]
metrics_to_collect_hard = ["mission_status", "collision_count",
        "planning_ctr", "runtime_failure_ctr", "traj_gen_failure_ctr"
]

# parse  data using parse_stat_file_flattened
result_dict = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)

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

# we generate a measure from spread of obstacles and peak
# congestion at the moment, but later this will come straight
# from the json file
difficulty_measure_based_on_mission_status_and_filtered_count = \
        np.multiply(
            peak_congestion_based_on_mission_status_and_filtered_count,
            spread_of_obstacles_based_on_mission_status_and_filtered_count)
difficulty_measure_based_on_mission_status_and_filtered_count = \
        list(np.divide(difficulty_measure_based_on_mission_status_and_filtered_count,
            goal_distance_based_on_mission_status_and_filtered_count))


print(mission_energy_filtered_based_on_mission_status_and_filtered_count)

def subplot_2d(x, y, axes_labels, ax_inst):
    ax_inst.plot(x, y, 'bo')
    ax_inst.set(xlabel=axes_labels[0], ylabel=axes_labels[1])
    ax_inst.grid(True)
    return ax_inst

fig, axes = plt.subplots(nrows=2, ncols=1)
axes[0] = subplot_2d(
        difficulty_measure_based_on_mission_status_and_filtered_count, 
        mission_time_filtered_based_on_mission_status_and_filtered_count,
        ["Difficulty measure", "Mission time"],
        axes[0]
)

axes[1] = subplot_2d(
        difficulty_measure_based_on_mission_status_and_filtered_count,
        mission_energy_filtered_based_on_mission_status_and_filtered_count,
        ["Difficulty measure", "Mission energy"],
        axes[1]
)

fig.tight_layout()
output_file = "end_to_end_stats" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

fig, axes = plt.subplots(nrows=2, ncols=2)
axes[0][0] = subplot_2d(
        difficulty_measure_based_on_mission_status_and_filtered_count, 
        planning_ctr_filtered_based_on_mission_status_and_filtered_count,
        ["Difficulty measure", "Total number of plans"],
        axes[0][0]
)

axes[0][1] = subplot_2d(
        difficulty_measure_based_on_mission_status_and_filtered_count,
        runtime_failure_ctr_filtered_based_on_mission_status_and_filtered_count,
        ["Difficulty measure", "Number of runtime failures"],
        axes[0][1]
)

axes[1][0] = subplot_2d(
        difficulty_measure_based_on_mission_status_and_filtered_count, 
        traj_gen_failure_ctr_filtered_based_on_mission_status_and_filtered_count,
        ["Difficulty measure", "Number of planner failures"],
        axes[1][0]
)

axes[1][1] = subplot_2d(
        difficulty_measure_based_on_mission_status_and_filtered_count, 
        average_velocity_filtered_based_on_mission_status_and_filtered_count,
        ["Difficulty measure", "Average velocity"],
        axes[1][1]
)

fig.tight_layout()
output_file = "ctr_stats" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

### testing 3D plot

def subplot_3d(x, y, z, axes_labels, ax_inst):
    ax_inst.scatter(x, y, z, 'bo')
    ax_inst.set(xlabel=axes_labels[0], ylabel=axes_labels[1], zlabel=axes_labels[2])
    ax_inst.grid(True)
    return ax_inst
    
fig = plt.figure(figsize=(12,6))
ax1 = fig.add_subplot(1, 2, 1, projection='3d')
ax1 = subplot_3d(
        spread_of_obstacles_based_on_mission_status_and_filtered_count,
        peak_congestion_based_on_mission_status_and_filtered_count,
        mission_time_filtered_based_on_mission_status_and_filtered_count,
        ["Spread of obstacles", "Peak congestion", "Mission Time"],
        ax1
)

ax2 = fig.add_subplot(1, 2, 2, projection='3d')
ax2 = subplot_3d(
        spread_of_obstacles_based_on_mission_status_and_filtered_count,
        peak_congestion_based_on_mission_status_and_filtered_count,
        mission_energy_filtered_based_on_mission_status_and_filtered_count,
        ["Spread of obstacles", "Peak congestion", "Mission Energy"],
        ax2
)

fig.tight_layout()
output_file = "end_to_end_stats_3d" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

fig = plt.figure(figsize=(12,12))
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1 = subplot_3d(
        spread_of_obstacles_based_on_mission_status_and_filtered_count,
        peak_congestion_based_on_mission_status_and_filtered_count,
        planning_ctr_filtered_based_on_mission_status_and_filtered_count,
        ["Spread of obstacles", "Peak congestion", "Total number of plans"],
        ax1
)

ax2 = fig.add_subplot(2, 2, 2, projection='3d')
ax2 = subplot_3d(
        spread_of_obstacles_based_on_mission_status_and_filtered_count,
        peak_congestion_based_on_mission_status_and_filtered_count,
        runtime_failure_ctr_filtered_based_on_mission_status_and_filtered_count,
        ["Spread of obstacles", "Peak congestion", "Number of runtime failures"],
        ax2
)

ax3 = fig.add_subplot(2, 2, 3, projection='3d')
ax3 = subplot_3d(
        spread_of_obstacles_based_on_mission_status_and_filtered_count,
        peak_congestion_based_on_mission_status_and_filtered_count,
        traj_gen_failure_ctr_filtered_based_on_mission_status_and_filtered_count,
        ["Spread of obstacles", "Peak congestion", "Number of planner failures"],
        ax3
)

ax4 = fig.add_subplot(2, 2, 4, projection='3d')
ax4 = subplot_3d(
        spread_of_obstacles_based_on_mission_status_and_filtered_count,
        peak_congestion_based_on_mission_status_and_filtered_count,
        average_velocity_filtered_based_on_mission_status_and_filtered_count,
        ["Spread of obstacles", "Peak congestion", "Average velocity"],
        ax4
)

fig.tight_layout()
output_file = "ctr_stats_3d" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)
