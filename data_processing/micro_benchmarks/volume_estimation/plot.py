import matplotlib.pyplot as plt
import sys
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.mplot3d import Axes3D 
sys.path.append('../../common_utils')
from data_parsing import *
from utils import *
import operator


result_folder = "./data_1"
input_file_name = "stats.json"
input_filepath = result_folder + "/" + input_file_name

metrics_to_collect_easy = []
metrics_to_collect_hard = ["sensor_volume_to_digest_estimated", "octomap_volume_digested"]

# parse  data
result_dic = parse_stat_file_flattened(input_filepath, metrics_to_collect_easy, metrics_to_collect_hard)
sensor_volume_to_digest_estimated = result_dic["sensor_volume_to_digest_estimated"]
octomap_volume_digested = result_dic["octomap_volume_digested"]

# the following is there to make sure that the two lists have the same lenght.
# note that -1 (or another number) needs to be encoded based on the stats.json data
sensor_volume_to_digest_estimated = sensor_volume_to_digest_estimated[:-1]

# calculated the "additive" error
tracking_error_before_fitting = map(operator.sub, octomap_volume_digested, sensor_volume_to_digest_estimated)
# model the "additive" error
# at the moment, we use a quadrative function to model the error
a, b, c = fit_func_2(sensor_volume_to_digest_estimated, tracking_error_before_fitting, 2)

# correct  the volume based the modeled error
sensor_volume_to_digest_estimated_corrected = []
for i in sensor_volume_to_digest_estimated:
    sensor_volume_to_digest_estimated_corrected.append(a*pow(i, 2) + b*i + c + i)

# calculate the error again to show how much it's improved
tracking_error_after_fitting = map(operator.sub, octomap_volume_digested, sensor_volume_to_digest_estimated_corrected)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(sensor_volume_to_digest_estimated, octomap_volume_digested, label="estimated vs real")
ax.plot(sensor_volume_to_digest_estimated_corrected, octomap_volume_digested, label ="corrected vs real")
ax.plot(sensor_volume_to_digest_estimated_corrected, tracking_error_before_fitting, label= "corrected vs tracking error before correction")
ax.plot(sensor_volume_to_digest_estimated_corrected, tracking_error_after_fitting, label= "corrected vs tracking error after correction. THIS IS THE IMPORTANT ONE")
ax.set_xlabel('sensor_volume_to_digest_estimated (m^3)')
ax.set_ylabel('octomap_space_volume_digested (m^3)')
ax.legend(loc='best', fontsize="small")
output_file = "sensor_volume_estimated_vs_octomap_volume_digested" + ".png"
plt.show()
plt.savefig(result_folder+"/"+output_file)
plt.close(fig)

