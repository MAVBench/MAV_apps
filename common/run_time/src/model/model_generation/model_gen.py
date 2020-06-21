import numpy as np
import sys
sys.path.append('..')
sys.path.append('../../../../../data_processing/common_utils')
#from calc_sampling_time import *
from utils import *

from model import Model
from data_parser import DataParser
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def calculate_model_error(coeffs, res, vol, latency, typical_model, mode="normalized"):
	assert mode in ["normalized", "absolute"]
	tuple_vals = zip(res, vol, latency)
	error_list = []
	max_abs_error = 0	
	for res_, vol_, latency_ in tuple_vals:
		absolute_error = abs(typical_model.calc_poly(res_, vol_, tuple(coeffs)) - latency_)
		if mode == "absolute": error_list.append(absolute_error)
		elif mode == "normalized": error_list.append(absolute_error/latency_)
		max_abs_error = max(absolute_error, max_abs_error)
		
	avg_error = sum(error_list)/len(error_list)
	return avg_error, max_abs_error	



def calculate_fitted_value(coeffs, res, vol, typical_model):
    return typical_model.calc_poly(res, vol, tuple(coeffs))

def calculate_fitted_values(coeffs, res, vol, typical_model):
    tuple_vals = zip(res, vol)
    result_list = []
    for res_, vol_ in tuple_vals:
        result_list.append(calculate_fitted_value(coeffs, res_, vol_, typical_model))
    return result_list


def collect_data():
    # Location of JSON data relative to current location
    result_folder = '../../knob_performance_modeling_all/data_1'

    # File names with data
    om_fname = 'stats.json_om'
    om_pl_fname = 'stats.json_om_to_pl'
    pp_pl_fname = 'stats.json_pp_pl'

    # Create parser objects
    om_parser = DataParser(result_folder, om_fname)
    om_pl_parser = DataParser(result_folder, om_pl_fname)
    pp_pl_parser = DataParser(result_folder, pp_pl_fname)

    # Get data tuples
    om_res, om_vol, om_response_time_measured = om_parser.parse_om()
    om_pl_res, om_pl_vol, om_pl_response_time_measured = om_pl_parser.parse_om_pl()
    pp_pl_res, pp_pl_vol, pp_pl_response_time_measured = pp_pl_parser.parse_pp_pl()

    return om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol, pp_pl_response_time_measured


def roborun_model_gen(om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol, pp_pl_response_time_measured
):
    # Size of polynomials to use
    res_order = 3
    vol_order = 1

    # Create model objects
    typical_model = Model(res_order, vol_order)
    om_model = Model(res_order, vol_order)
    om_pl_model = Model(res_order, vol_order)
    pp_pl_model = Model(res_order, vol_order)

    # Choose whether to profile curve fitting functions
    profile = False

    # Fit curves and get back model coefficients + other stuff
    om_popt, om_pcov, om_elapsed = om_model.fit(
            res=om_res, 
            vol=om_vol, 
            response_time=om_response_time_measured, 
            profile=profile)

    om_pl_popt, om_pl_pcov, om_pl_elapsed = om_pl_model.fit(
            res=om_pl_res, 
            vol=om_pl_vol, 
            response_time=om_pl_response_time_measured, 
            profile=profile)

    pp_pl_popt, pp_pl_pcov, pp_pl_elapsed = pp_pl_model.fit(
            res=pp_pl_res, 
            vol=pp_pl_vol, 
            response_time=pp_pl_response_time_measured, 
            profile=profile)

    
    return om_popt, om_pl_popt, pp_pl_popt, typical_model
    


## May want to convert this to using absolute paths to make life easier ##
if __name__ == '__main__':
    # collect data 
    om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol, pp_pl_response_time_measured = collect_data()

    # colecting models model 
    om_popt, om_pl_popt, pp_pl_popt, typical_model = roborun_model_gen(om_res, om_vol, om_response_time_measured, om_pl_res, om_pl_vol, om_pl_response_time_measured, pp_pl_res, pp_pl_vol,
            pp_pl_response_time_measured)	 # for error calculation
    
    
    print("pc om models") 
    print(om_popt)
    print("om to pl models") 
    print(om_pl_popt)
    print("pl models") 
    print(pp_pl_popt)

    # calcualting the error 
    om_error, om_max_abs_error = calculate_model_error(om_popt, om_res, om_vol, om_response_time_measured, typical_model, "normalized")
    om_pl_error, om_pl_max_abs_error = calculate_model_error(om_pl_popt, om_pl_res, om_pl_vol, om_pl_response_time_measured, typical_model, "normalized")
    pp_pl_error, pp_pl_max_abs_error = calculate_model_error(pp_pl_popt, pp_pl_res, pp_pl_vol, pp_pl_response_time_measured, typical_model, "normalized")

    print("avg om_error:" + str(om_error) + " max om_error:" + str(om_max_abs_error))
    print("om_pl_error:" + str(om_pl_error) + " max om_pl_error" + str(om_pl_max_abs_error))
    print("pp_pl_error:" + str(pp_pl_error) + " max pp_pl_error" + str(pp_pl_max_abs_error))


    # plot the measured vs fitterd 
    om_response_time_fitted =  calculate_fitted_values(om_popt, om_res, om_vol, typical_model)
    om_pl_response_time_fitted =  calculate_fitted_values(om_pl_popt , om_pl_res, om_pl_vol, typical_model)
    pp_pl_response_time_fitted =  calculate_fitted_values(pp_pl_popt , pp_pl_res, pp_pl_vol, typical_model)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(om_res, om_vol, om_response_time_measured, c="orange")  # , zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
    ax.scatter(om_res, om_vol, om_response_time_fitted)  # , zdir='z', c=None, depthshade=True)#(, *args, **kwargs)


    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.scatter(om_pl_res, om_pl_vol, om_pl_response_time_measured, c="orange")  # , zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
    ax2.scatter(om_pl_res, om_pl_vol, om_pl_response_time_fitted)  # , zdir='z', c=None, depthshade=True)#(, *args, **kwargs)



    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111, projection='3d')
    ax3.scatter(pp_pl_res, pp_pl_vol, pp_pl_response_time_measured, c="orange")  # , zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
    ax3.scatter(pp_pl_res, pp_pl_vol, pp_pl_response_time_fitted)  # , zdir='z', c=None, depthshade=True)#(, *args, **kwargs)
    plt.show(5)







