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
stage_of_interests_to_pick_from = ["pc_om", "om_to_pl", "pp_pl", "pc_om_estimation"]

hw_sampling_method = "decision_based"

# which stage are you trying to plot
stage_of_interest = "pp_pl" # pick form ["om_to_pl", "pc_om", "pp_pl", "pc_om_estimation]

assert stage_of_interest in stage_of_interests_to_pick_from

result_folder = "./data_1"
input_file_name = "saturday_morning.json"
input_filepath = result_folder + "/" + input_file_name
 
# data to collect (pass in the variables that you  are interested in)
# PS: always keep experiement_number inf metrics_to_collect_easy.
# PS: all the other parameters (Except mission_status and collision_count should be added to metrics_to_collect_easy (not hard)
metrics_to_collect_easy = ["experiment_number"]
metrics_to_collect_hard = ["mission_status", "collision_count"]

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
