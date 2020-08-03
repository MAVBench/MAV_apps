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

metrics_to_collect_easy = [
        "flight_time", 
        "total_energy_consumed",
        "distance_travelled"
]

metrics_to_collect_hard = []

result_dict_static = parse_stat_file_flattened(input_filepath_static, metrics_to_collect_easy, metrics_to_collect_hard)
result_dict_dynamic = parse_stat_file_flattened(input_filepath_dynamic, metrics_to_collect_easy, metrics_to_collect_hard)

result_dict_static["average_velocity"] = np.divide(result_dict_static["distance_travelled"], result_dict_static["flight_time"])
result_dict_dynamic["average_velocity"] = np.divide(result_dict_dynamic["distance_travelled"], result_dict_dynamic["flight_time"])

static_df = pd.DataFrame(result_dict_static)
static_df["type"] = "Static"
dynamic_df = pd.DataFrame(result_dict_dynamic)
dynamic_df["type"] = "Dynamic"

total_df = pd.concat([static_df, dynamic_df])

fields = [
        "flight_time", 
        "total_energy_consumed",
        "distance_travelled",
        "average_velocity"
]

ylabels = [
        "Flight time (s)",
        "Total energy consumed (J)",
        "Distance travelled (m)",
        "Average velocity (m/s)"
]

width = 0.35

fig, axes = plt.subplots(1, len(fields))

for i, field in enumerate(fields):
    axes[i].bar(
        [0 - width/2],
        result_dict_dynamic[field],
        width,
        label="Dynamic"
    )

    axes[i].bar(
        [0 + width/2],
        result_dict_static[field],
        width,
        label="Static"
    )

    axes[i].set_title(field, fontsize=8)
    axes[i].set_xlabel("")
    axes[i].set_ylabel(ylabels[i])
    axes[i].set_xticks([0])
    axes[i].set_xticklabels(["Mission"])

handles, labels = axes[len(fields)-1].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper right')
fig.tight_layout()
fig.savefig(result_folder + "/" + "overall_compare" + ".png")
plt.close(fig)
