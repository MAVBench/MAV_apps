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

# planning resolution of .25
# point cloud box 25x25
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_25_25 = np.array([3.26, 2.6, 2.15,1.17, 1.08 ])
S_A_response_time_25_25 = np.array([6.09, 4.74, 3.85, 2.78, 1.74])
max_safe_velocity_calculated_25_25 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_25_25.append(calc_v_max(S_A_response_time_25_25[idx], sensor_range[idx], m, b))

# planning resolution of .25
# point cloud box 15x15
sensor_range = np.array([25 , 20,  15, 10, 5])
S_A_latency_25_15 = np.array([3.23, 2.64, 2.16, 1.19,1.10 ])
S_A_response_time_25_15 = np.array([6.02, 4.83, 3.91, 2.83, 1.77])
max_safe_velocity_calculated_25_15 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_25_15.append(calc_v_max(S_A_response_time_25_15[idx], sensor_range[idx], m, b))


# planning resolution of .25
# point cloud box 10x10
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_25_10 = np.array([1.64, 1.41, 1.29, 1.06, .85 ])
S_A_response_time_25_10 = np.array([2.84, 2.46, 2.13, 1.69, 1.26])
max_safe_velocity_calculated_25_10 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_25_10.append(calc_v_max(S_A_response_time_25_10[idx], sensor_range[idx], m, b))

# planning resolution of .25
# point cloud box 5x5
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_25_5 = np.array([.87, .85, .77, .715,.613 ])
S_A_response_time_25_5 = np.array([1.28, 1.27, 1.07, .96, .829])
max_safe_velocity_calculated_25_5 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_25_5.append(calc_v_max(S_A_response_time_25_5[idx], sensor_range[idx], m, b))


# --------------- 
#  switching planning resolution
#----------------
# planning resolution of .45
# point cloud box 25x25
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_45_25 = np.array([1.33, 1.55, 1.32,1.10, .86 ])
S_A_response_time_45_25 = np.array([3.08, 2.72, 2.25, 1.80, 1.33])
max_safe_velocity_calculated_45_25 = []
for idx in range(0, len(sensor_range)):
    print idx 
    max_safe_velocity_calculated_45_25.append(calc_v_max(S_A_response_time_45_25[idx], sensor_range[idx], m, b))


# planning resolution of .45
# point cloud box 15x15
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_45_15 = np.array([1.75, 1.53, 1.36, 1.09, .86 ])
S_A_response_time_45_15 = np.array([3.11, 2.64, 2.31, 1.78,1.32 ])
max_safe_velocity_calculated_45_15 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_45_15.append(calc_v_max(S_A_response_time_45_15[idx], sensor_range[idx], m, b))

#TODO
# planning resolution of .45
# point cloud box 10x10
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_45_10 = np.array([.99, .9, .85, .787, .63])
S_A_response_time_45_10 = np.array([1.61, 1.42, 1.28, 1.17, .88])
max_safe_velocity_calculated_45_10 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_45_10.append(calc_v_max(S_A_response_time_45_10[idx], sensor_range[idx], m, b))

# planning resolution of .45
# point cloud box 5x5
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_45_5 = np.array([.667, .58, .60, .54, .49])
S_A_response_time_45_5 = np.array([.919,.77, .82, .75, .66])
max_safe_velocity_calculated_45_5 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_45_5.append(calc_v_max(S_A_response_time_45_5[idx], sensor_range[idx], m, b))


# --------------- 
#  switching planning resolution
#----------------
# planning resolution of .65
# point cloud box 25x25
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_65_25 = np.array([1.35, 1.24, 1.09, .93, .78])
S_A_response_time_65_25 = np.array([2.3, 2.09, 1.78, 1.47, 1.17])
max_safe_velocity_calculated_65_25 = []
for idx in range(0, len(sensor_range)):
    print idx 
    max_safe_velocity_calculated_65_25.append(calc_v_max(S_A_response_time_65_25[idx], sensor_range[idx], m, b))


# planning resolution of .65
# point cloud box 15x15
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_65_15 = np.array([1.37, 1.23, 1.09,.93,.78])
S_A_response_time_65_15 = np.array([2.36, 2.06, 1.8,1.48,1.16])
max_safe_velocity_calculated_65_15 = []
for idx in range(0, len(sensor_range)):
    print idx 
    max_safe_velocity_calculated_65_15.append(calc_v_max(S_A_response_time_65_15[idx], sensor_range[idx], m, b))


# planning resolution of .65
# point cloud box 10x10
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_65_10 = np.array([.81,.80, .78, .73, .725])
S_A_response_time_65_10 = np.array([1.2, 1.19, 1.16, 1.05, .99])
max_safe_velocity_calculated_65_10 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_65_10.append(calc_v_max(S_A_response_time_65_10[idx], sensor_range[idx], m, b))


# planning resolution of .65
# point cloud box 5x5
sensor_range = np.array([25 , 20, 15, 10, 5])
S_A_latency_65_5 = np.array([.58, .54, .49, .60, .56, .56]) #TODO  last element
S_A_response_time_65_5 = np.array([.75, .72, .79, .8, .81]) # TODO last element
max_safe_velocity_calculated_65_5 = []
for idx in range(0, len(sensor_range)):
    max_safe_velocity_calculated_65_5.append(calc_v_max(S_A_response_time_65_5[idx], sensor_range[idx], m, b))


# ----- plotting
fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_xlabel('sensor_range (m)',fontsize=16)
ax.set_ylabel('S_A response time (s) ',fontsize=16)
ax.plot(sensor_range, S_A_response_time_25_25, marker='o', color='y', label="resolution = .25, pc_box=25")
ax.plot(sensor_range, S_A_response_time_45_25, marker='o', color='orange', label="resolution = .45, pc_box =25x25")
ax.plot(sensor_range, S_A_response_time_65_25, marker='o', color='r', label="resolution = .65, pc_box=25x25")
ax.legend(loc='best', fontsize="small")
plt.savefig("sensor_range_response_time.png")
plt.close()

fig2 = plt.figure(2)
ax = fig2.add_subplot(111)
ax.set_ylabel('sensor_range (m)',fontsize=16)
ax.set_xlabel('max calculaed velocity (m/s) ',fontsize=16)
ax.legend(loc='best', fontsize="small")
ax.plot(max_safe_velocity_calculated_25_25, sensor_range, marker='o', color='y', label="resolution = .25, pc_box = 25x25")
ax.plot(max_safe_velocity_calculated_45_25, sensor_range, marker='o', color='orange', label="resolution = .45, pc_box = 25x25")
ax.plot(max_safe_velocity_calculated_65_25, sensor_range, marker='o', color='r', label="resolution = .65, pc_box = 25x25")
ax.legend(loc='best', fontsize="small")
plt.savefig("calc_velocity_max_sensor_range.png")
plt.close()


fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_xlabel('calculated velocity (m/s) ',fontsize=16)
ax.set_ylabel('max sensor_range (m)',fontsize=16)

ax.plot(max_safe_velocity_calculated_25_25, sensor_range, marker='o', markersize = 15, linestyle = ':', color='gold', label=":: resolution = .25, pc_box=25x25")
ax.plot( max_safe_velocity_calculated_25_15, sensor_range, marker='o', markersize = 10, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=15x15")
ax.plot( max_safe_velocity_calculated_25_10, sensor_range, marker='o', markersize = 5, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=10x10")
ax.plot( max_safe_velocity_calculated_25_5, sensor_range, marker='o', markersize = 2, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=5x5")

ax.plot( max_safe_velocity_calculated_45_25, sensor_range, marker='o', markersize = 15, color='orange', linestyle = "--", label="- - resolution = .45, pc_box25x25")
ax.plot(max_safe_velocity_calculated_45_15, sensor_range, marker='o', markersize = 10, color='orange', linestyle = "--", label="- - resolution = .45, pc_box = 15x15")
ax.plot( max_safe_velocity_calculated_45_10, sensor_range, marker='o', markersize = 5, color='orange', linestyle = "--", label=" - -resolution = .45, pc_box = 10x10")
ax.plot( max_safe_velocity_calculated_45_5, sensor_range, marker='o', markersize = 2, color='orange', linestyle = "--", label="- -resolution = .45, pc_box = 5x5")


ax.plot( max_safe_velocity_calculated_65_25, sensor_range, marker='o', markersize = 15, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_h=25x25")
ax.plot( max_safe_velocity_calculated_65_15, sensor_range, marker='o', markersize = 10, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_box=15x15")
ax.plot( max_safe_velocity_calculated_65_10, sensor_range, marker='o', markersize = 5, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_h=10x10")
ax.plot( max_safe_velocity_calculated_65_5, sensor_range, marker='o', markersize = 2,color='darkred', linestyle = "-", label=" __ resolution = .65, pc_box=5x5")

ax.legend(loc='best', fontsize="small")
plt.savefig("calc_velocity_max_sensor_range_all.png")
plt.close()


fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_xlabel('calculated velocity (m/s) ',fontsize=16)
ax.set_ylabel('max sensor_range (m)',fontsize=16)

ax.plot(sensor_range, S_A_latency_25_25,  marker='o', markersize = 15, linestyle = ':', color='gold', label=":: resolution = .25, pc_box=25x25")
ax.plot(sensor_range, S_A_latency_25_15,  marker='o', markersize = 10, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=15x15")
ax.plot(sensor_range, S_A_latency_25_10,  marker='o', markersize = 5, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=10x10")
ax.plot(sensor_range, S_A_latency_25_5,  marker='o', markersize = 2, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=5x5")

ax.plot(sensor_range, S_A_latency_45_25,  marker='o', markersize = 15, color='orange', linestyle = "--", label="- - resolution = .45, pc_box25x25")
ax.plot(sensor_range, S_A_latency_45_15,  marker='o', markersize = 10, color='orange', linestyle = "--", label="- - resolution = .45, pc_box = 15x15")
ax.plot(sensor_range, S_A_latency_45_10,  marker='o', markersize = 5, color='orange', linestyle = "--", label=" - -resolution = .45, pc_box = 10x10")
ax.plot(sensor_range, S_A_latency_45_5,  marker='o', markersize = 2, color='orange', linestyle = "--", label="- -resolution = .45, pc_box = 5x5")


ax.plot(sensor_range, S_A_latency_65_25,  marker='o', markersize = 15, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_h=25x25")
ax.plot(sensor_range, S_A_latency_65_15,  marker='o', markersize = 10, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_box=15x15")
ax.plot(sensor_range, S_A_latency_65_10,  marker='o', markersize = 5, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_h=10x10")
ax.plot(sensor_range, S_A_latency_65_5,  marker='o', markersize = 2,color='darkred', linestyle = "-", label=" __ resolution = .65, pc_box=5x5")

ax.legend(loc='best', fontsize="small")
plt.savefig("sensor_range_S_A_latency_all.png")
plt.close()


fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_xlabel('calculated velocity (m/s) ',fontsize=16)
ax.set_ylabel('max sensor_range (m)',fontsize=16)

ax.plot(sensor_range, S_A_response_time_25_25,  marker='o', markersize = 15, linestyle = ':', color='gold', label=":: resolution = .25, pc_box=25x25")
ax.plot(sensor_range, S_A_response_time_25_15,  marker='o', markersize = 10, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=15x15")
ax.plot(sensor_range, S_A_response_time_25_10,  marker='o', markersize = 5, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=10x10")
ax.plot(sensor_range, S_A_response_time_25_5,  marker='o', markersize = 2, color='gold', linestyle = ":", label=":: resolution = .25, pc_box=5x5")

ax.plot(sensor_range, S_A_response_time_45_25,  marker='o', markersize = 15, color='orange', linestyle = "--", label="- - resolution = .45, pc_box25x25")
ax.plot(sensor_range, S_A_response_time_45_15,  marker='o', markersize = 10, color='orange', linestyle = "--", label="- - resolution = .45, pc_box = 15x15")
ax.plot(sensor_range, S_A_response_time_45_10,  marker='o', markersize = 5, color='orange', linestyle = "--", label=" - -resolution = .45, pc_box = 10x10")
ax.plot(sensor_range, S_A_response_time_45_5,  marker='o', markersize = 2, color='orange', linestyle = "--", label="- -resolution = .45, pc_box = 5x5")


ax.plot(sensor_range, S_A_response_time_65_25,  marker='o', markersize = 15, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_h=25x25")
ax.plot(sensor_range, S_A_response_time_65_15,  marker='o', markersize = 10, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_box=15x15")
ax.plot(sensor_range, S_A_response_time_65_10,  marker='o', markersize = 5, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_h=10x10")
ax.plot(sensor_range, S_A_response_time_65_5,  marker='o', markersize = 2,color='darkred', linestyle = "-", label=" __ resolution = .65, pc_box=5x5")

ax.legend(loc='best', fontsize="small")
plt.savefig("sensor_range_S_A_response_time_all.png")
plt.close()




#  slice by different point cloud box
fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_xlabel('sensor_range (m)',fontsize=16)
ax.set_ylabel('max calculated velocity (m/s) ',fontsize=16)

ax.plot(sensor_range, max_safe_velocity_calculated_25_25, marker='o', markersize = 15, linestyle = ':', color='gold', label="resolution = .25, pc_box=25x25")
ax.plot(sensor_range, max_safe_velocity_calculated_25_15, marker='o', markersize = 10, color='gold', linestyle = ":", label="resolution = .25, pc_box=15x15")
ax.plot(sensor_range, max_safe_velocity_calculated_25_10, marker='o', markersize = 5, color='gold', linestyle = ":", label="resolution = .25, pc_box=10x10")
ax.plot(sensor_range, max_safe_velocity_calculated_25_5, marker='o', markersize = 2, color='gold', linestyle = ":", label="resolution = .25, pc_box=5x5")
ax.legend(loc='best', fontsize="small")
plt.savefig("sensor_range_max_calc_velocity_diff_pc_b.png")
plt.close()


#  slice by different resolution 
fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_xlabel('sensor_range (m)',fontsize=16)
ax.set_ylabel('max calculated velocity (m/s) ',fontsize=16)

ax.plot(sensor_range, max_safe_velocity_calculated_25_25, marker='o', markersize = 10, linestyle = ':', color='gold', label=": resolution = .25, pc_box=25x25")
ax.plot(sensor_range, max_safe_velocity_calculated_45_25, marker='o', markersize = 10, color='orange', linestyle = "--", label=" - - resolution = .45, pc_box25x25")
ax.plot(sensor_range, max_safe_velocity_calculated_65_25, marker='o', markersize = 10, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_h=25x25")
ax.legend(loc='best', fontsize="small")
plt.savefig("sensor_range_max_calc_velocity_diff_resolution.png")
plt.close()


fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_xlabel('sensor_range (m)',fontsize=16)
ax.set_ylabel('max calculated velocity (m/s) ',fontsize=16)

ax.plot(sensor_range, max_safe_velocity_calculated_65_25, marker='o', markersize = 10, color='darkred', linestyle = "-", label=" __ resolution = .65, pc_h=25x25")
ax.legend(loc='best', fontsize="small")
plt.savefig("sensor_range_max_calc_velocity_range.png")
plt.close()





