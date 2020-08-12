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
from EnvGenParser import EnvGenParser
import os
import seaborn as sns
import argparse
import pandas as pd

col = sns.color_palette("Paired", 111111)
stage_of_interests_to_pick_from = ["pc_om", "om_to_pl", "pp_pl", "pc_om_estimation"]

parser = argparse.ArgumentParser()
parser.add_argument('-s', '--staticdata_path', type=str)
parser.add_argument('-d', '--dynamicdata_path', type=str)
parser.add_argument('-e', '--envfile_path', type=str)
parser.add_argument('-r', '--resultdir_path', type=str)
args = parser.parse_args()

def dir_path(path):
    if not os.path.isfile(path):
        print("Couldn't find datafile!")
        exit(-1)
    else:
        return os.path.dirname(os.path.realpath(path))

# we dump resulting graphs in same dir as datafile

input_filepath_static = args.staticdata_path
input_filepath_dynamic = args.dynamicdata_path
obs_stats_path = args.envfile_path
if args.resultdir_path:
    result_folder = args.resultdir_path
else:
    result_folder = dir_path(input_filepath_dynamic)

# which stage are you trying to plot
stage_of_interest = "pp_pl" # pick form ["om_to_pl", "pc_om", "pp_pl", "pc_om_estimation]

assert stage_of_interest in stage_of_interests_to_pick_from

# data to collect
metrics_to_collect_easy = []
metrics_to_collect_hard = [
        "depthToPCConversionLatency",
        "runDiagnosticsLatency",
        "runTimeLatency",
        "sequencerLatency",
        "PCFilteringLatency",
        "PCtoOMComOHLatency",
        "PCDeserializationLatency",
        "PCtoOMTotalLatency",
        "OMFilterOutOfRangeLatency",
        "insertScanLatency",
        "OMFilteringLatency",
        "OMSerializationLatency",
        "OMtoPlComOHLatency",
        "OMDeserializationLatency",
        "OMtoPlTotalLatency",
        "smoothening_latency",
        "ppl_latency",
        "pl_to_ft_totalLatency",
        "ee_latency",
        "blind_latency",
        "cpu_utilization_for_last_decision",
        "planning_success_ctr",
        "traj_gen_failure_ctr",
        "planning_ctr",
        "decision_ctr",
        "runtime_failure_ctr",
        "octomap_volume_integrated",
        "om_to_pl_volume",
        "ppl_volume",
        "time_cmd_received",
        "pc_to_om_datamovement",
        "om_to_pl_datamovement",
        "PERF_COUNT_HW_INSTRUCTIONS",
        "LLC_REFERENCES",
        "CACHE_REFERENCES",
        "x_coord_while_budgetting",
        "y_coord_while_budgetting",
        "z_coord_while_budgetting",
        "sensor_to_actuation_time_budget_to_enforce",
        "closest_unknown_distance",
        "obs_dist_statistics_min",
        "vel_mag_while_budgetting",
        "pc_res",
        "om_to_pl_res",
        "gap_statistics_avg"
]

# parse  data

result_dict_static = parse_stat_file_flattened(input_filepath_static, metrics_to_collect_easy, metrics_to_collect_hard)
result_dict_static["pl_to_ft_totalLatency"] =  [0] + result_dict_static["pl_to_ft_totalLatency"][1:]
result_dict_static["cpu_utilization_for_last_decision"] = result_dict_static["cpu_utilization_for_last_decision"][1:] + [100]
result_dict_static["PERF_COUNT_HW_INSTRUCTIONS"] = result_dict_static["PERF_COUNT_HW_INSTRUCTIONS"][1:]+ [100]
result_dict_static["LLC_REFERENCES"] = result_dict_static["LLC_REFERENCES"][1:]+ [100]
result_dict_static["CACHE_REFERENCES"] = result_dict_static["CACHE_REFERENCES"][1:]+ [100]


result_dict_dynamic = parse_stat_file_flattened(input_filepath_dynamic, metrics_to_collect_easy, metrics_to_collect_hard)
result_dict_dynamic["pl_to_ft_totalLatency"] =  [0] + result_dict_dynamic["pl_to_ft_totalLatency"][1:]
result_dict_dynamic["cpu_utilization_for_last_decision"] = result_dict_dynamic["cpu_utilization_for_last_decision"][1:] + [100]
result_dict_dynamic["PERF_COUNT_HW_INSTRUCTIONS"] = result_dict_dynamic["PERF_COUNT_HW_INSTRUCTIONS"][1:]+ [100]
result_dict_dynamic["LLC_REFERENCES"] = result_dict_dynamic["LLC_REFERENCES"][1:]+ [100]
result_dict_dynamic["CACHE_REFERENCES"] = result_dict_dynamic["CACHE_REFERENCES"][1:]+ [100]

env_gen_parser = EnvGenParser(obs_stats_path)

################## ZONE-WISE STATS #######################

overall_stat_fields = [
        "planning_success_ctr",
        "traj_gen_failure_ctr",
        "planning_ctr",
        "decision_ctr",
        "runtime_failure_ctr"
]

zones = [0, 1, 2]

# Adding decision-based stats to df and removing zeroed coords

dynamic_per_decision_dict = {k:v for k,v in result_dict_dynamic.items() if k not in overall_stat_fields}
dynamic_per_decision_df = pd.DataFrame(dynamic_per_decision_dict)
dynamic_per_decision_df = dynamic_per_decision_df.loc[ \
        # removing zeros
        (dynamic_per_decision_df["x_coord_while_budgetting"] != 0) & (dynamic_per_decision_df["y_coord_while_budgetting"] != 0)
]

static_per_decision_dict = {k:v for k,v in result_dict_static.items() if k not in overall_stat_fields}
static_per_decision_df = pd.DataFrame(static_per_decision_dict)
static_per_decision_df = static_per_decision_df.loc[ \
        (static_per_decision_df["x_coord_while_budgetting"] != 0) & (static_per_decision_df["y_coord_while_budgetting"] != 0)
]

dynamic_per_decision_df.drop(dynamic_per_decision_df.tail(3).index, inplace=True)
static_per_decision_df.drop(static_per_decision_df.tail(3).index, inplace=True)

per_decision_dict = {}
per_decision_dict["dynamic"] = dynamic_per_decision_df
per_decision_dict["static"] = static_per_decision_df

# Marking zones for each entry

for mode in per_decision_dict.keys():
    mode_df = per_decision_dict[mode]
    mode_df["Zone"] = np.NaN
    time_cmd_received = mode_df["time_cmd_received"].to_numpy()
    x_coords = mode_df["x_coord_while_budgetting"].to_numpy()
    y_coords = mode_df["y_coord_while_budgetting"].to_numpy()

    zone_times = env_gen_parser.split_time_by_zone(time_cmd_received, x_coords, y_coords)
    # we enter the first zone at time 0
    zone_times.insert(0, 0)

    for i in range(len(zone_times)):
        if i < len(zone_times) - 1:
            mode_df["Zone"] = \
                    np.where(
                            (mode_df["time_cmd_received"] >= zone_times[i]) & (mode_df["time_cmd_received"] < zone_times[i+1]),
                        i, mode_df["Zone"])
        else:
            mode_df["Zone"] = \
                    np.where(mode_df["time_cmd_received"] >= zone_times[i], i, mode_df["Zone"])

# Summing up communication breakdown components

comm_components = [
        "PCtoOMTotalLatency",
        "OMtoPlTotalLatency",
        "pl_to_ft_totalLatency",
]

for mode in per_decision_dict.keys():
    mode_df = per_decision_dict[mode]
    sum_col = np.zeros(len(mode_df))
    for comp in comm_components:
        sum_col += mode_df[comp]

    mode_df["communication_breakdown"] = sum_col

# Summing up computation breakdown components

comp_components = [
        "depthToPCConversionLatency", 
        "runDiagnosticsLatency", 
        "runTimeLatency", 
        "sequencerLatency", 
        "PCFilteringLatency", 
        "OMFilterOutOfRangeLatency",  
        "insertScanLatency", 
        "OMFilteringLatency", 
        "ppl_latency",
        "smoothening_latency"
]

for mode in per_decision_dict.keys():
    mode_df = per_decision_dict[mode]
    sum_col = np.zeros(len(mode_df))
    for comp in comp_components:
        sum_col += mode_df[comp]

    mode_df["computation_breakdown"] = sum_col

# Summing up response time components

resp_components = [
        "ee_latency", 
        "blind_latency"
]

for mode in per_decision_dict.keys():
    mode_df = per_decision_dict[mode]
    sum_col = np.zeros(len(mode_df))
    for comp in resp_components:
        sum_col += mode_df[comp]

    mode_df["response_time_breakdown"] = sum_col

# Log of ppl_volume

for mode in per_decision_dict.keys():
    mode_df = per_decision_dict[mode]
    mode_df["log_ppl_volume"] = np.log(mode_df["ppl_volume"])

# Combined df for static and dynamic

dyn_df = per_decision_dict["dynamic"].copy()
dyn_df["type"] = "Dynamic"
static_df = per_decision_dict["static"].copy()
static_df["type"] = "Static"

combined_per_decision = pd.concat([dyn_df, static_df])

# Creating zone distance-normalized df

dist_norm_dict = {}

for mode in per_decision_dict.keys():
    mode_df = per_decision_dict[mode]
    dist_norm_df = pd.DataFrame().reindex_like(mode_df).dropna()
    zone_bounds = env_gen_parser.extract_zones()
    for zone in zones:
        # diagonal dist of zone
        zone_bound = zone_bounds[zone]
        zone_dist = np.linalg.norm(np.array(zone_bound[0]) - np.array(zone_bound[1]))
        if zone == 0 or zone == 2:
            # in practice, the start and goal positions are in the middle of
            # the zones
            zone_dist /= 2

        zone_df = mode_df.loc[mode_df["Zone"] == zone]
        zone_df /= zone_dist
        # we reset zone to its original value because we had divided entire df
        zone_df = zone_df.assign(Zone = zone)
        dist_norm_df = pd.concat([dist_norm_df, zone_df], ignore_index=True)

    dist_norm_dict[mode] = dist_norm_df

# Combined df for static and dynamic

dyn_df = dist_norm_dict["dynamic"].copy()
dyn_df["type"] = "Dynamic"
static_df = dist_norm_dict["static"].copy()
static_df["type"] = "Static"

combined_dist_norm = pd.concat([dyn_df, static_df])

# Creating time-normalized df

time_norm_dict = {}

for mode in per_decision_dict.keys():
    mode_df = per_decision_dict[mode]
    time_norm_df = pd.DataFrame().reindex_like(mode_df).dropna()

    time_cmd_received = mode_df["time_cmd_received"].to_numpy()
    x_coords = mode_df["x_coord_while_budgetting"].to_numpy()
    y_coords = mode_df["y_coord_while_budgetting"].to_numpy()

    zone_transitions = env_gen_parser.split_time_by_zone(time_cmd_received, x_coords, y_coords)
    zone_transitions.insert(0, 0)
    zone_transitions.append(time_cmd_received[-1])

    for zone in zones:
        zone_time_spent = zone_transitions[zone+1] - zone_transitions[zone]

        zone_df = mode_df.loc[mode_df["Zone"] == zone]
        zone_df /= zone_time_spent
        # we reset zone to its original value because we had divided entire df
        zone_df = zone_df.assign(Zone = zone)
        time_norm_df = pd.concat([time_norm_df, zone_df], ignore_index=True)

    time_norm_dict[mode] = time_norm_df

# Combined df for static and dynamic

dyn_df = time_norm_dict["dynamic"].copy()
dyn_df["type"] = "Dynamic"
static_df = time_norm_dict["static"].copy()
static_df["type"] = "Static"

combined_time_norm = pd.concat([dyn_df, static_df])

# Grouping by aggregate stats

def decision_based_stats(mode):
    grouped_zone = per_decision_dict[mode].groupby("Zone")
    return grouped_zone.agg([np.mean, np.std, np.sum, np.max, np.min])

def distance_based_stats(mode):
    grouped_zone = dist_norm_dict[mode].groupby("Zone")
    return grouped_zone.agg([np.mean, np.std, np.sum, np.max, np.min])

def time_based_stats(mode):
    grouped_zone = time_norm_dict[mode].groupby("Zone")
    return grouped_zone.agg([np.mean, np.std, np.sum, np.max, np.min])

def box_plot_field(field, ylabel):
    # decision-based

    fig, ax = plt.subplots()
    box_plot = sns.boxplot(
            x="Zone", 
            y=field, 
            hue="type", 
            data=combined_per_decision, 
            ax=ax,
            #showfliers=False,
            whis=10
    )
    ax.set_title(field + ": Decision Based", fontsize=8)
    fig.savefig(result_folder + "/" + field + "_decision_based" + ".png")
    plt.close(fig)

    # distance-based

    #fig, ax = plt.subplots()
    #box_plot = sns.boxplot(
    #        x="Zone", 
    #        y=field, 
    #        hue="type", 
    #        data=combined_dist_norm, 
    #        ax=ax,
    #        #showfliers=False,
    #        whis=100
    #)
    #ax.set_title(field + ": Normalized by Zone Distance", fontsize=8)
    #fig.savefig(result_folder + "/" + field + "_dist_norm" + ".png")
    #plt.close(fig)

    ## time-based

    #fig, ax = plt.subplots()
    #box_plot = sns.boxplot(
    #        x="Zone", 
    #        y=field, 
    #        hue="type", 
    #        data=combined_time_norm, 
    #        ax=ax,
    #        #showfliers=False,
    #        whis=100
    #)
    #ax.set_title(field + ": Normalized by Zone Traversal Time", fontsize=8)
    #fig.savefig(result_folder + "/" + field + "_time_norm" + ".png")
    #plt.close(fig)

def bar_plot_field(field, ylabel):
    # decision-based

    fig, ax = plt.subplots()
    bar_plot = sns.barplot(
            x="Zone", 
            y=field, 
            hue="type", 
            data=combined_per_decision, 
            ax=ax,
    )
    ax.set_title(field + ": Decision Based", fontsize=8)
    fig.savefig(result_folder + "/" + field + "_decision_based" + ".png")
    plt.close(fig)

    # distance-based

    fig, ax = plt.subplots()
    bar_plot = sns.barplot(
            x="Zone", 
            y=field, 
            hue="type", 
            data=combined_dist_norm, 
            ax=ax,
    )
    ax.set_title(field + ": Normalized by Zone Distance", fontsize=8)
    fig.savefig(result_folder + "/" + field + "_dist_norm" + ".png")
    plt.close(fig)

    # time-based

    fig, ax = plt.subplots()
    bar_plot = sns.barplot(
            x="Zone", 
            y=field, 
            hue="type", 
            data=combined_time_norm, 
            ax=ax,
    )
    ax.set_title(field + ": Normalized by Zone Traversal Time", fontsize=8)
    fig.savefig(result_folder + "/" + field + "_time_norm" + ".png")
    plt.close(fig)

fields_to_plot = [
        # Precision
        "pc_res",
        # E2E latency
        "ee_latency",
        # Data movement
        "om_to_pl_datamovement",
        "pc_to_om_datamovement",
        # Volume
        "octomap_volume_integrated",
        "om_to_pl_volume",
        "log_ppl_volume",
        # Time breakdowns
        "communication_breakdown",
        "computation_breakdown",
        "response_time_breakdown",
        # Hardware counters
        "PERF_COUNT_HW_INSTRUCTIONS",
        "LLC_REFERENCES",
        "CACHE_REFERENCES"
]

ylabels = [
        # Precision
        "Precision (cm)",
        # E2E latency
        "End to end latency (s)",
        # Data movement
        "Data movement (B)",
        "Data movement (B)",
        # Volume
        "Volume (m3)",
        "Volume (m3)",
        "Volume (m3)",
        # Time breakdowns
        "Communication breakdown (s)",
        "Computation breakdown (s)",
        "Response time breakdown (s)",
        # Hardware counters
        "HW instructions (millions)",
        "Cache references (millions)",
        "LLC references (millions)"
]

for i, field in enumerate(fields_to_plot):
    box_plot_field(field, ylabels[i])
    #bar_plot_field(field, ylabels[i])

exit()

############# Grouped barplots #################

fields_to_plot = [
        "PERF_COUNT_HW_INSTRUCTIONS",
        "LLC_REFERENCES",
        "CACHE_REFERENCES"
]

ylabels = [
        "HW instructions (millions)",
        "Cache references (millions)",
        "LLC references (millions)"
]

width = 0.3
zones = [0, 1, 2]

### mean + stddev aggregates

fig, axes = plt.subplots(1, 3)

for i, field in enumerate(fields_to_plot):
    for j, mode in enumerate(mode_zone_stats_df_dict.keys()):
        if mode == "dynamic":
            x_offset = - width / 2
        elif mode == "static":
            x_offset = width / 2

        axes[i].bar(
                np.array(zones) + x_offset,
                mode_zone_stats_df_dict[mode].T.loc[field, "mean"], 
                width, 
                yerr=mode_zone_stats_df_dict[mode].T.loc[field, "std"],
                label=mode
        )

        axes[i].set_title(field, fontsize=9)
        axes[i].set_xlabel("Zone")
        axes[i].set_ylabel(ylabels[j])
        axes[i].set_xticks(zones)
        axes[i].set_xticklabels(zones)

handles, labels = axes[2].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper right')
fig.tight_layout()
output_file = "aggregate_stats_zones" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

### distance-normalized aggregates

fig, axes = plt.subplots(1, 3)

for i, field in enumerate(fields_to_plot):
    for j, mode in enumerate(mode_zone_stats_df_dict.keys()):
        # normalize the stats to diagonal distance of zones
        zone_sum = mode_zone_stats_df_dict[mode].T.loc[field, "sum"]
        zone_dists = []
        for zone in zones:
            zone_bound = env_gen_parser.extract_zones()[zone]
            zone_dist = np.linalg.norm(np.array(zone_bound[0]) - np.array(zone_bound[1]))
            if zone == 0 or zone == 2:
                # in practice, the start and goal positions are in the middle of
                # the zones
                zone_dist /= 2
            zone_dists.append(zone_dist)

        zone_stat_norm = np.divide(zone_sum, zone_dists)

        if mode == "dynamic":
            x_offset = - width / 2
        elif mode == "static":
            x_offset = width / 2

        axes[i].bar(
                np.array(zones) + x_offset,
                zone_stat_norm,
                width, 
                label=mode
        )

        axes[i].set_title(field, fontsize=9)
        axes[i].set_xlabel("Zone")
        axes[i].set_ylabel(ylabels[j])
        axes[i].set_xticks(zones)
        axes[i].set_xticklabels(zones)

handles, labels = axes[2].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper right')
fig.tight_layout()
output_file = "normalized_dist_stats_zones" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)

### time-normalized

fig, axes = plt.subplots(1, 3)

for i, field in enumerate(fields_to_plot):
    for j, mode in enumerate(mode_zone_stats_df_dict.keys()):
        mode_df = per_decision_dict[mode]
        time_cmd_received = mode_df["time_cmd_received"].to_numpy()
        x_coords = mode_df["x_coord_while_budgetting"].to_numpy()
        y_coords = mode_df["y_coord_while_budgetting"].to_numpy()
        zone_transitions = env_gen_parser.split_time_by_zone(time_cmd_received, x_coords, y_coords)

        zone_transitions.insert(0, 0)
        zone_transitions.append(time_cmd_received[-1])

        zone_time_spent = []
        for trans_ind in range(1, len(zone_transitions)):
            zone_time_spent.append(zone_transitions[trans_ind] - zone_transitions[trans_ind-1])

        zone_sum = mode_zone_stats_df_dict[mode].T.loc[field, "sum"]

        # normalize the stats to time spent per zone
        zone_stat_norm = np.divide(zone_sum, zone_time_spent)

        if mode == "dynamic":
            x_offset = - width / 2
        elif mode == "static":
            x_offset = width / 2

        axes[i].bar(
                np.array(zones) + x_offset,
                zone_stat_norm,
                width, 
                label=mode
        )

        axes[i].set_title(field, fontsize=9)
        axes[i].set_xlabel("Zone")
        axes[i].set_ylabel(ylabels[j])
        axes[i].set_xticks(zones)
        axes[i].set_xticklabels(zones)

handles, labels = axes[2].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper right')
fig.tight_layout()
output_file = "normalized_time_stats_zones" + ".png"
fig.savefig(result_folder + "/" + output_file)
plt.close(fig)
