from data import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
"""
def calc_total_t_budget(v, visibility, m, b_):
    return (v**2 - 2*m*visibility*v - 2*b_*visibility)/(-2*m*(v**2) - 2*b_*v) 

def calc_next_stample_time(v, visibility, m, b_, latency):
    return calc_total_t_budget(v, visibility, m, b_) - latency

def fit_func(x, y, deg):
    m, b = np.polyfit(x, y, deg)
    return m,b

def subsample(my_l, subsample_rate = 5):
    cntr = 0 
    result = [] 
    for i in range(len(my_l)):
        if cntr % subsample_rate == 0:
            result.append(my_l[cntr])
        cntr +=1
    return result

def calc_next_sample_time(velocity_data, visibility, m, b_, min_latency, delta_t):
    v = velocity_data[0]
    budget_till_next_stample = calc_next_sample_time(v, visibility, m, b_, min_latency)
    next_sample_time = 0    
    idx = 0 
    while budget_till_next_stample > 0:
        budget_till_next_stample -= delta_t
        next_sample_time += delta_t
        v = velocity_data[idx]
        budget_till_next_stample = calc_next_sample_time(v, visibility, m, b_, min_latency)
        idx +=1
    return idx, next_sample_time


def iteratrively_calc_next_sample_time(velocity_data, visibility, m, b_, min_latency, delta_t):
    idx = 0 
    next_sample_time_l = [] 
    while len(velocity_data)>0:
        idx, next_sample_time = calc_next_stample_time(velocity_data[idx:], visibility, m, b_, min_latency, delta_t)
        next_sample_time_l.append(next_sample_time)
    return  next_sample_time_l  


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
# assign values
tick_values = []
tick_val = 0
for val in velocity_data:
    tick_values.append(tick_val)
    tick_val += delta_t

tick_vel_line = ax1.plot(tick_values, velocity_data,marker='o', color='b', label="trip velocity profile")
ax1.legend(loc='best', fontsize="small")
# save file
output_file = "trip_velocity_profile" + str(int(max(velocity_data))) + ".png"
plt.savefig(output_file)
plt.close()


fig2 = plt.figure(1)
# fit a line through the data
v_max_l = []
velocity_vals = np.array([25.7, 18.5, 9.6, 7.7, 5.8, 3.9, 2.0])
acceleration_vals = np.array([4.3, 3.55, 2.46, 2.11, 1.73, 1.25, .75])
deg = 1
m, b = fit_func(velocity_vals, acceleration_vals, deg)
visibility = 25


# calc theoretical t_budget from v
ax2 = fig2.add_subplot(111)
theoretical_t_budget = [calc_total_t_budget(v+.2, visibility, m, b) for v in velocity_data]
tick_theoretical_t_budget_line = ax2.plot(subsample(tick_values), subsample(theoretical_t_budget) ,marker='o', color='y', label = "dynamic response time budget")

ax2.set_xlabel('time (s)',fontsize=16)
ax2.set_ylabel('time budget (s) ',fontsize=16)

#ax3 = fig2.add_subplot(111)
theoretical_t_budget_min = [calc_total_t_budget(max(velocity_data), visibility, m, b)] 
#print(theoretical_t_budget)
tick_theoretical_min_t_budget_line = ax2.plot(subsample(tick_values, 20), subsample(theoretical_t_budget_min*len(tick_values), 20) ,marker='o', color='g', label = "static response time budget")
#ax2.legend([tick_theoretical_min_t_budget_line, tick_theoretical_t_budget_line], ["min_t_budget_over_all", "t_budget"])
#ax2.legend((tick_theoretical_t_budget_line, tick_theoretical_min_t_budget_line), (["min_t_budget_over_all"], ["t_budget"]))
ax2.legend(loc='best', fontsize="small")
plt.savefig("time_slack.png")

"""

fig3 = plt.figure(4)
ax4 = fig3.add_subplot(111)
# fit a line through the data
planning_budget_vals = np.array([.3, 1])
flight_time_budget_vals = np.array([757, 1296])
ax4.set_xlabel('planning budget (s)',fontsize=16)
ax4.set_ylabel('flight time(s) ',fontsize=16)
ax4.plot(planning_budget_vals, flight_time_budget_vals ,marker='o', color='y')
plt.savefig("blah.png")








