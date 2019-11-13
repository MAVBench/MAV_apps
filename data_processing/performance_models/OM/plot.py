from data import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset


sensor_range = np.array([25 , 20, 15, 10])
S_A_latency = np.array([1.409, 1.242, 1.06, 0.928])
S_A_response_time = np.array([2.447, 2.09, 1.75, 1.48])

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
sensor_range = np.array([25 , 20, 15, 10, 5])
ax.set_xlabel('sensor_range (m)',fontsize=16)
ax.set_ylabel('S_A response_time (s) ',fontsize=16)
ax.plot(sensor_range, S_A_response_time ,marker='o', color='y', label="ok")
plt.savefig("sensor_range_response_time.png")
plt.close()








