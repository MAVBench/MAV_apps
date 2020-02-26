from data import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset

def subsample(my_l, subsample_rate = 5):
    cntr = 0 
    result = [] 
    for i in range(len(my_l)):
        if cntr % subsample_rate == 0:
            result.append(my_l[cntr])
        cntr +=1
    return result

def find_avg_acc_during_dec(velocity_data, delta_t):
    reached_max = False 
    started_cnting = False
    ctr = 0
    max_vel = round(max (velocity_data), 1)
    for vel in velocity_data:
        if not reached_max: 
            if round(vel, 1) == max_vel:
                reached_max  = True
        elif not started_cnting:
            if round(vel, 1) < max_vel:
                started_cnting = True
        else:
            ctr += 1 
            if round(vel, 1) == 0:
                break
    avg_acc = (0 - max_vel)/(ctr*delta_t)
    print ("max vel" + str(max_vel))
    return avg_acc
                 
# ---- velocity data
#init figs
fig1 = plt.figure(1)
ax1 = fig1.add_subplot(111)

# axis labels
ax1.set_xlabel('time (ms)',fontsize=16)
ax1.set_ylabel('velocity',fontsize=16)

# legend

# velocity acceleration
delta_t = .05
# assign values
x_values = []
x_val = 0
for val in velocity_data:
    x_values.append(x_val)
    x_val += delta_t

ax1.plot(subsample(x_values, 10), subsample(velocity_data, 10),marker='o', color='b', label = "velocity")
# save file
#output_file = "_velocity_for_" + str(int(max(velocity_data))) + ".png"
#plt.savefig(output_file)


# ---- acceleration data
#fig2 = plt.figure(2)
#ax2 = fig1.add_subplot(111)
ax1.set_ylabel('quantity',fontsize=16)

## calculate acceleration
acc_vals = []
for cnt in range(0, len(velocity_data) - 1):
    acc_vals.append((velocity_data[cnt+1] - velocity_data[cnt])/float(delta_t))

# plot time vs acceleration
ax1.plot(x_values[:-1], acc_vals,marker='o', color='g', label="acceleration")
#output_file = "acc_vel" + str(int(max(velocity_data))) + ".png"
#plt.savefig(output_file)


# ---- avg dec data
#ax3 = fig1.add_subplot(111)
ax1.set_ylabel('quantity',fontsize=16)
## calculate acceleration
avg_acc = []
avg_acc_val = find_avg_acc_during_dec(velocity_data, delta_t)
print("avg acc is" + str(avg_acc_val))
for cnt in range(0, len(velocity_data) - 1):
    avg_acc.append(avg_acc_val)

# plot time vs acceleration
ax1.plot(subsample(x_values[:-1], 30), subsample(avg_acc,30) ,marker='o', color='orange', label="avg stoppage decceleration (stop cmd to full stop)")
ax1.legend(loc='lower left', fontsize ="small")
output_file = "acc_vel" + str(int(max(velocity_data))) + ".png"

plt.savefig(output_file)


#plt.show()
#fig2 = plt.figure(2)
#ax2 = fig2.add_subplot(211)
#plt.gcf().subplots_adjust(top=0.85,left=0.15,right=0.95)
#ax2.tick_params(axis='x',bottom=False,top=False,labelbottom=False)
#ax2.grid(which='major',linewidth='0.3')
#ax2.set_ylabel('Mission Time \n [s]',fontsize=16)
#ax3 = fig2.add_subplot(212)
#ax3.grid(which='major',linewidth='0.3')
#ax3.set_ylabel('Mission Energy \n [kJ]',fontsize=16)
#plt.xticks(x_pos,bars,weight='bold')
#fig2.subplots_adjust(hspace=0.0001)
#plot
#plt.rc('xtick',labelsize=16)

