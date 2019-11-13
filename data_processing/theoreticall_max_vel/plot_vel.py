import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calc_poly_deg_2(a, b, c):
    return (-b + np.sqrt(b**2 - 4*a*c))/(2*a)

"""
def calc_v_max(response_time, visibility, m, b_):
    a = 1 + 2*response_time*m
    b = -2*m*visibility + 2*response_time*b_
    c = -2*b_*visibility 
    return calc_poly_deg_2(a, b, c) 
"""
def calc_v_max(response_time, visibility, m, b_):
    a = 1 + 2*response_time*m
    b = -2*m*visibility + 2*response_time*b_
    c = -2*b_*visibility 
    return calc_poly_deg_2(a, b, c)


def fit_func(x, y, deg):
    m, b = np.polyfit(x, y, deg)
    return m,b


# fit a line through the data
v_max_l = []
velocity_vals = np.array([25.7, 18.5, 9.6, 7.7, 5.8, 3.9, 2.0])
acceleration_vals = np.array([4.3, 3.55, 2.46, 2.11, 1.73, 1.25, .75])
deg = 1
m, b = fit_func(velocity_vals, acceleration_vals, deg)

# draw the fitted graph
fig1 = plt.figure(1)
ax2 = fig1.add_subplot(111)
fitted_vel = [i for i in range(0, 30)]
fitted_accel = [m*vel + b  for vel in fitted_vel]
ax2.plot(fitted_vel, fitted_accel,marker='o', color='b', label="fitted_curve")

# draw the actual values
ax1 = fig1.add_subplot(111)
ax1.set_xlabel('velocity ',fontsize=16)
ax1.set_ylabel('acceleration',fontsize=16)
ax1.plot(velocity_vals, acceleration_vals,marker='x', color='g', label="measured_curve")
output_file = "acceleration_velocity_fit.png"
ax1.legend(loc='best', fontsize="small")
plt.savefig(output_file)
plt.close()


# this sould be used for single v_max_calculation
fig2 = plt.figure()
visibility = np.array([25]*4)
response_time = np.array([1.8, 2.35, 2.74, 4.60])
measured_v_max= np.array([10.67, 9.29, 7.56, 6.13])
calculated_v_max = calc_v_max(response_time, visibility, m, b)
ax2 = fig2.add_subplot(111)
ax2.plot(list(response_time), list(calculated_v_max), marker='o', color='b', label="calculated_v_max")
ax2.plot(list(response_time), list(measured_v_max), marker='x', color='g', label="measured_v_max")
ax2.set_xlabel('response time (s)',fontsize=16)
ax2.set_ylabel('max velocity (m/s)',fontsize=16)
output_file = "calculate_vs_measured_vmax.png"
ax2.legend(loc='best', fontsize="small")
#plt.show()
plt.savefig(output_file)
plt.close()


# ---- v_max vs response_time vs visibility for the entire space
fig3 = plt.figure()
response_time = np.linspace(2, 10, 10)
visibility = np.linspace(20, 40, 10)
X, Y = np.meshgrid(response_time, visibility)
Z = calc_v_max(X,Y, m, b)
#ax = plt.axes(projection='3d')
ax3 = fig3.add_subplot(111, projection='3d') 
ax3.contour3D(X, Y, Z, 50, zdir ='z', cmap=plt.get_cmap('viridis'))#'binary')
ax3.set_xlabel('response_time (s)')
ax3.set_ylabel('visibility (m)')
ax3.set_zlabel('v_max (m/s)');
ax3.legend(loc='best', fontsize="small")
output_file = "theoretical_v_max" + ".png"
plt.savefig(output_file)



