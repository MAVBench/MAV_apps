from data import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
import sys
sys.path.append('../common_utils')
from calc_sampling_time import *
from utils import *


# ---- velocity data
#init figs
fig1 = plt.figure(1)
ax1 = fig1.add_subplot(111)

# axis labels
ax1.set_xlabel('time (s)',fontsize=16)
ax1.set_ylabel('velocity (m/s)',fontsize=16)

# legend
#ax1.legend(['acceleration'], loc='best', fontsize = 16)

# velocity acceleration
delta_t = .05
v_max_l = []
velocity_vals = np.array([25.7, 18.5, 9.6, 7.7, 5.8, 3.9, 2.0])
acceleration_vals = np.array([4.3, 3.55, 2.46, 2.11, 1.73, 1.25, .75])
deg = 1
m, b = fit_func(velocity_vals, acceleration_vals, deg)
visibility = 25



# assign values
tick_values = []
tick_val = 0
for val in velocity_data:
    tick_values.append(tick_val)
    tick_val += delta_t

tick_vel_line = ax1.plot(tick_values, velocity_data,marker='o', color='b', label="trip velocity profile")


# dynamically samples
min_latency = 1.55
velocity_data_ = velocity_data[:]
next_sample_time_l_dynamic = iteratively_calc_next_sample_time(velocity_data_, visibility, m, b, min_latency, delta_t)

next_sampling_line = [([sample_time]*20, range(0,20)) for sample_time in next_sample_time_l_dynamic]
cntr = 0
for line in next_sampling_line:
    if cntr == 0:
        ax1.plot(line[0], line[1], color='yellow', label='dyanmic sampling time')
    else:
        ax1.plot(line[0], line[1], color='yellow')
    cntr +=1
cntr = 0
ax1.legend(loc='upper right', fontsize="small")
# save file
output_file = "trip_velocity_profile_dynamically_sampled_" + str(int(max(velocity_data))) + ".png"
plt.savefig(output_file)
plt.close()




# statically sampled
fig2 = plt.figure(1)
ax2 = fig2.add_subplot(111)
# axis labels
tick_vel_line = ax2.plot(tick_values, velocity_data,marker='o', color='b', label="trip velocity profile")
ax2.set_xlabel('time (s)',fontsize=16)
ax2.set_ylabel('velocity (v/s)',fontsize=16)
# save file
next_sample_time_l_static = np.arange(0,len(velocity_data)*delta_t, min_latency)
next_sampling_line = [([sample_time]*20, range(0,20)) for sample_time in next_sample_time_l_static]
cntr = 0
for line in next_sampling_line:
    if cntr == 1:
        ax2.plot(line[0], line[1], color='yellow', label='static sampling_time')
    else:
        ax2.plot(line[0], line[1], color='yellow')
    cntr +=1
cntr = 0
ax2.legend(loc='upper right', fontsize="small")
output_file = "trip_velocity_profile_statically_sampled_" + str(int(max(velocity_data))) + ".png"
plt.savefig(output_file)
plt.close()



# extra time for processing per sample
fig3 = plt.figure(1)
ax3 = fig3.add_subplot(111)
# axis labels
tick_vel_line = ax3.plot(tick_values, velocity_data,marker='o', color='b', label="trip velocity profile")


ax3.set_xlabel('time (s)',fontsize=16)
ax3.set_ylabel('extra time to allocate(s)',fontsize=16)

extra_time_for_processing = []
idx = 0
for sample_time in range(0, len(next_sample_time_l_dynamic) - 1):
    extra_time_for_processing.append(next_sample_time_l_dynamic[idx+1] - next_sample_time_l_dynamic[idx] - min_latency) 
    idx += 1
ax3.plot(next_sample_time_l_dynamic[:-1], extra_time_for_processing, color='green', label="extra time budget to allocate")

next_sampling_line = [([sample_time]*20, range(0,20)) for sample_time in next_sample_time_l_dynamic]
cntr = 0
for line in next_sampling_line:
    if cntr == 0:
        ax3.plot(line[0], line[1], color='yellow', label='dyanmic sampling_time')
    else:
        ax3.plot(line[0], line[1], color='yellow')
    cntr +=1
cntr = 0
ax3.legend(loc='upper right', fontsize="small")
# save file
output_file = "trip_velocity_profile_extra_time_to_allocate" + str(int(max(velocity_data))) + ".png"
plt.savefig(output_file)
plt.close()



