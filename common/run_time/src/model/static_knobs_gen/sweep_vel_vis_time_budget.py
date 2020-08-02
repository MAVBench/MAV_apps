from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
import numpy as np
import sys
sys.path.append('..')
sys.path.append('../../../../../data_processing/common_utils')
#from calc_sampling_time import *
from utils import *
from model_generation.model_gen import *
from model import Model
from data_parser import DataParser
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

 

def calc_poly_deg_2(a, b, c):
    return (-b + np.sqrt(b**2 - 4*a*c))/(2*a)

"""
ax = fig.gca(projection='3d')
X=[]
Y=[]
Z=[]

for visibility in range(1, 25):
    for max_time_budget in np.arange(0, 10, .1):
        velocity_to_budget_on = calc_poly_deg_2(.05494, .349+ max_time_budget, -visibility- .2038)
        X.append(visibility)
        Z.append(max_time_budget)
        Y.append(velocity_to_budget_on)
"""


def calc_time_budget(visibility, velocity):
    return (visbility -0.05494*pow(velocity,2) - 0.3549*(velocity)+ 0.2038)/velocity

def vel_vis_buget_sweep():
    visbility = np.linspace(1, 25, 25)                                                              
    velocity = np.linspace(1, 10, 10)                                                                
    X, Y = np.meshgrid(visbility, velocity)                                                       
    Z = calc_time_budget(X,Y)     
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
                                   linewidth=0, antialiased=False)
    ax.set_zlim(0, 25.01)
    fig.colorbar(surf, shrink = .65, aspect=20)

    ax.set_xlabel('Visbility (m)')
    ax.set_ylabel('Velocity (m/s)')                                                                                                                                                                                                                                                                               
    ax.set_zlabel('Time Budget (s)');
    ax.legend(loc='best', fontsize="small")                                                                                                                                                                                                                                                                       
    plt.show()



def res_vol_budget_sweep():
    drone_radius = 1.5
    drone_radius_when_hovering =  drone_radius  # set it equal to each other to avoid sub_vmax optimization
    result_folder = "../../knob_performance_modeling_all/data_1" 
    om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol, pp_pl_response_time_measured = collect_data(result_folder)
    om_popt, om_pl_popt, pp_pl_popt, typical_model = roborun_model_gen(om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol,
            pp_pl_response_time_measured)	 # for error calculation

    om_latency = calculate_fitted_value(om_popt, om_res, om_vol, typical_model)
    res = np.linspace(.3, 2.4, 4)                                                              
    res = [.3, .6, 1.2, 2.4] 
    volume = np.linspace(1000, 400000, 4)                                                           
    latency = [] 
    for res_, vol in zip(res,volume):
        latency.append(calculate_fitted_value(om_popt, res_, vol, typical_model))

    """
    X, Y = np.meshgrid(res, volume)                                                       
    Z = calculate_fitted_value(om_popt, X,Y, typical_model)     
    print(Z)
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
            linewidth=0, antialiased=False)
    ax.set_zlim(0, 25.01)
    fig.colorbar(surf, shrink = .65, aspect=20)
    """ 
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.scatter(res, volume, latency)
    ax.set_xlabel('Precision (m)')
    ax.set_ylabel('Volume (m^3)')
    ax.set_zlabel('Time Budget (s)');
    ax.legend(loc='best', fontsize="small")                                                                                                                                                                                                                                                                       
    plt.show()
    """
    res_vol_budget_sweep()
    #output_file = "theoretical_v_max" + ".png"                                                                                                                                                                                                                                                                      
    #plt.savefig(output_file)                                                                                                                                                                                                                                                                                        
    #$ax.plot_wireframe(X, Y, Z, rstride=10, cstride=10)
    #ax.plot_trisurf(x, y, z, linewidth=0.2, antialiased=True)
    #plt.show()
    """
res_vol_budget_sweep()
