import numpy as np
import sys
sys.path.append('..')
from model.model import Model
from model.data_parser import DataParser

def calculate_error():
    pass

        
## May want to convert this to using absolute paths to make life easier ##
if __name__ == '__main__':

	# Size of polynomials to use
	res_order = 3
	vol_order = 1

	# Create model objects
	om_model = Model(res_order, vol_order)
	om_pl_model = Model(res_order, vol_order)
	pp_pl_model = Model(res_order, vol_order)

	# Location of JSON data relative to current location
	result_folder = '../knob_performance_modeling_all/data_1'

	# File names with data
	om_fname = 'stats.json_om'
	om_pl_fname = 'stats.json_om_to_pl'
	pp_pl_fname = 'stats.json_pp_pl'

	# Create parser objects
	om_parser = DataParser(result_folder, om_fname)
	om_pl_parser = DataParser(result_folder, om_pl_fname)
	pp_pl_parser = DataParser(result_folder, pp_pl_fname)

	# Get data tuples
	om_res, om_vol, om_response_time = om_parser.parse_om()
	om_pl_res, om_pl_vol, om_pl_response_time = om_pl_parser.parse_om_pl()
	pp_pl_res, pp_pl_vol, pp_pl_response_time = pp_pl_parser.parse_pp_pl()

	# Choose whether to profile curve fitting functions
	profile = False

	# Fit curves and get back model coefficients + other stuff
	om_popt, om_pcov, om_elapsed = om_model.fit(
									res=om_res, 
								 	vol=om_vol, 
								 	response_time=om_response_time, 
								 	profile=profile)

	om_pl_popt, om_pl_pcov, om_pl_elapsed = om_pl_model.fit(
											 res=om_pl_res, 
											 vol=om_pl_vol, 
											 response_time=om_pl_response_time, 
											 profile=profile)

	pp_pl_popt, pp_pl_pcov, pp_pl_elapsed = pp_pl_model.fit(
											 res=pp_pl_res, 
											 vol=pp_pl_vol, 
											 response_time=pp_pl_response_time, 
											 profile=profile)
