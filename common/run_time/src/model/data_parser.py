import numpy as np
import sys
from util.data_parsing import *



class DataParser(object):

	def __init__(self, result_folder, fname):

		self.result_folder = result_folder

		self.easy_metrics = []
		
		self.hard_metrics = ["octomap_exposed_resolution", "point_cloud_estimated_volume", "octomap_volume_digested", "potential_volume_to_explore_knob_modeling",
		        "resolution_to_explore_knob_modeling", "piecewise_planner_time_knob_modeling", "piecewise_planner_resolution_knob_modeling", "piecewise_planner_volume_explored_knob_modeling",
		        "piecewise_planner_time_knob_modeling", "octomap_to_motion_planner_serialization_to_reception_knob_modeling", "octomap_insertCloud_minus_publish_all",
		        "octomap_to_planner_com_overhead_knob_modeling" ]

		self.indep_vars = ["piecewise_planning_budget", "perception_resolution",
		                           "smoothening_budget", "experiment_number"]

		self.dep_vars = ["distance_travelled", "flight_time", "S_A_latency", "S_A_response_time_calculated_from_imgPublisher",
		                           "planning_piecewise_failure_rate", "planning_smoothening_failure_rate", "RRT_path_length_normalized_to_direct_path"]

		self._parse_data(fname)


	# should probably check that fname is a json
	def _parse_data(self, fname):
		#input_file_name = "stats.json"
		fpath = self.result_folder + "/" + fname
		self.result_dic = parse_stat_file_flattened(fpath, self.easy_metrics, self.hard_metrics)


	# Each parse function returns a (res, vol, response_time) tuple

	# Point cloud + Octomap data
	def parse_om(self):
		octomap_exposed_resolution = self.result_dic["octomap_exposed_resolution"]
		point_cloud_estimated_volume  = self.result_dic["point_cloud_estimated_volume"]
		octomap_integration_response_time = self.result_dic["octomap_insertCloud_minus_publish_all"]
		return (octomap_exposed_resolution, point_cloud_estimated_volume, octomap_integration_response_time)

	# Octomap to planner data
	def parse_om_pl(self):
		resolution_to_explore_knob_modeling = self.result_dic["resolution_to_explore_knob_modeling"]
		potential_volume_to_explore_knob_modeling= self.result_dic["potential_volume_to_explore_knob_modeling"]
		octomap_to_motion_planner_serialization_to_reception_knob_modeling = self.result_dic["octomap_to_motion_planner_serialization_to_reception_knob_modeling"]
		return (resolution_to_explore_knob_modeling, potential_volume_to_explore_knob_modeling, octomap_to_motion_planner_serialization_to_reception_knob_modeling)

	# Planner data
	def parse_pp_pl(self):
		piecewise_planner_resolution_knob_modeling = self.result_dic["piecewise_planner_resolution_knob_modeling"]
		piecewise_planner_volume_explored_knob_modeling = self.result_dic["piecewise_planner_volume_explored_knob_modeling"]
		piecewise_planner_time_knob_modeling = self.result_dic["piecewise_planner_time_knob_modeling"]
		return (piecewise_planner_resolution_knob_modeling, piecewise_planner_volume_explored_knob_modeling, piecewise_planner_time_knob_modeling)
