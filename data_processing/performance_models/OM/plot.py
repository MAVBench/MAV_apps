import matplotlib.pyplot as plt
import numpy as np
import sys
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
sys.path.append('../../common_utils')
from utils import *

# data to find the fit (acquired from another experience)
velocity_vals = np.array([25.7, 18.5, 9.6, 7.7, 5.8, 3.9, 2.0])
acceleration_vals = np.array([4.3, 3.55, 2.46, 2.11, 1.73, 1.25, .75])
deg = 1
m, b = fit_func(velocity_vals, acceleration_vals, deg)
print m, b
# data for this experience
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency = np.array([1.409, 1.242, 1.06, 0.928,.76 ])
S_A_response_time = np.array([2.447, 2.09, 1.75, 1.48,.98 ])
max_safe_velocity_calculated = []
for idx in range(0, len(sensor_range)):
    print idx 
    max_safe_velocity_calculated.append(calc_v_max(S_A_response_time[idx], sensor_range[idx], m, b))


fig = plt.figure(1)
ax = fig.add_subplot(111)
# fit a line through the data
ax.set_xlabel('sensor_range (m)',fontsize=16)
ax.set_ylabel('S_A latency (s) ',fontsize=16)
ax.plot(sensor_range, S_A_latency ,marker='o', color='y', label="ok")
plt.savefig("sensor_range_latency.png")
plt.close()


fig = plt.figure(1)
ax = fig.add_subplot(111)
# fit a line through the data
ax.set_xlabel('sensor_range (m)',fontsize=16)
ax.set_ylabel('S_A response_time (s) ',fontsize=16)
ax.plot(sensor_range, S_A_response_time ,marker='o', color='y', label="ok")
plt.savefig("sensor_range_response_time.png")
plt.close()

fig = plt.figure(1)
ax = fig.add_subplot(111)
# fit a line through the data
ax.set_xlabel('sensor_range (m)',fontsize=16)
ax.set_ylabel('max safe velocity (calculated) (m/s) ',fontsize=16)
ax.plot(sensor_range, max_safe_velocity_calculated,marker='o', color='y') 
plt.savefig("sensor_range_max_safe_velocity_calculated.png")
plt.close()












