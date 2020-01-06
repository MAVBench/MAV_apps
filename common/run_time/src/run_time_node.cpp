#include "ros/ros.h"
#include <std_msgs/String.h>
//#include "template_library.hpp"
#include <sstream>
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
//#include "controllers/DroneControllerBase.hpp"
//#include "common/Common.hpp"
#include <fstream>
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include <cstring>
#include <string>
#include <geometry_msgs/Vector3.h>
#include <mavbench_msgs/multiDOFpoint.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include "TimeBudgetter.h"
#include <Drone.h>
#include <datacontainer.h>
#include <profile_manager.h>
#include <mavbench_msgs/runtime_debug.h>

using namespace std;
std::string ip_addr__global;
double g_sensor_max_range, g_sampling_interval, g_v_max;
std::deque<double> MacroBudgets;
bool dynamic_budgetting, reactive_runtime;
bool DEBUG_RQT;
int g_capture_size = 600; //set this to 1, if you want to see every data collected separately

typedef struct node_budget_t {
	string node_type;
	double budget;
} NodeBudget;


typedef struct param_val_t {
	string param_name;
	double param_val;
} ParamVal;


typedef struct node_param_t {
	string node_type;
	vector<ParamVal>  param_val_vec;
} NodeParams;

vector<string> node_types;
mavbench_msgs::runtime_debug debug_data;
DataContainer *profiling_container;

vector<NodeBudget> calc_micro_budget(double macro_time_budget){
	vector<NodeBudget> node_budget_vec;
	double node_coeff;
	for (auto &node_type: node_types){

		if (node_type == "point_cloud"){
			node_coeff = .1;
		} else if (node_type == "octomap"){
			node_coeff = .55;
		} else if (node_type == "planning"){
			node_coeff = .35;
		}else{
			ROS_ERROR_STREAM("node:" << node_type << " is not defined for runtime");
			exit(0);
		}
		node_budget_vec.push_back(NodeBudget{node_type, node_coeff*macro_time_budget});
	}

	return node_budget_vec;
}


vector<ParamVal> perf_model(NodeBudget node_budget) {
	vector<ParamVal> results;
	if (node_budget.node_type == "point_cloud"){
		if (node_budget.budget > .2) {
			ROS_ERROR_STREAM("widening point c loud");
			results.push_back(ParamVal{"point_cloud_height", 25});
			results.push_back(ParamVal{"point_cloud_width", 25});
			results.push_back(ParamVal{"point_cloud_resolution", .3});
		}else{
			//ROS_ERROR_STREAM("shrinking point c loud");
			results.push_back(ParamVal{"point_cloud_height", 5});
			results.push_back(ParamVal{"point_cloud_width", 5});
			results.push_back(ParamVal{"point_cloud_resolution", .8});
		}
	} else if (node_budget.node_type == "octomap"){
		if (node_budget.budget > .5) {
			results.push_back(ParamVal{"perception_lower_resolution", .8});
		}else{
			results.push_back(ParamVal{"perception_lower_resolution", .8});
		}
	}else if (node_budget.node_type == "planning"){
		results.push_back(ParamVal{"piecewise_planning_budget", node_budget.budget/2});
		results.push_back(ParamVal{"smoothening_budget", node_budget.budget/2});
	}else{
		ROS_ERROR_STREAM("node:" << node_budget.node_type << " is not defined for runtime");
		exit(0);
	}
	return results;
}


vector<ParamVal> calc_params(NodeBudget node_budget) {
	return perf_model(node_budget);
}


void set_param(ParamVal param) {
	ros::param::set(param.param_name, param.param_val);
}


void convertMavBenchMultiDOFtoMultiDOF(mavbench_msgs::multiDOFpoint copy_from, multiDOFpoint &copy_to){
    	copy_to.vx = copy_from.vx;
    	copy_to.vy = copy_from.vy;
    	copy_to.vz = copy_from.vz;
    	copy_to.ax = copy_from.ax;
    	copy_to.ay = copy_from.ay;
    	copy_to.az = copy_from.az;
    	copy_to.x = copy_from.x;
    	copy_to.y = copy_from.y;
    	copy_to.z = copy_from.z;
}

void next_steps_callback(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg){
    std::deque<multiDOFpoint> traj;
    for (auto point_: msg->points){
    	multiDOFpoint point__;
    	convertMavBenchMultiDOFtoMultiDOF(point_, point__);
    	traj.push_back(point__);
    }

	vector<double> accelerationCoeffs = {.1439,.8016};
	double maxSensorRange = g_sensor_max_range;
	double TimeIncr = g_sampling_interval;
	double maxVelocity = g_v_max;

	double latency = 1.55; //TODO: get this from follow trajectory
	TimeBudgetter MacrotimeBudgetter(maxSensorRange, maxVelocity, accelerationCoeffs, TimeIncr);

	auto macro_time_budgets = MacrotimeBudgetter.calcSamplingTime(traj, latency);
	auto node_budget_vec = calc_micro_budget(macro_time_budgets[0]);

	// calculate each node's params
	vector<vector<ParamVal>> node_params_vec;
	for (auto &node_budget: node_budget_vec) {
		node_params_vec.push_back(calc_params(node_budget));
	}

	//set each node's params
	for (auto &node_params: node_params_vec) {
		for (auto param : node_params)
			;
			//set_param(param);
	}

	MacroBudgets.pop_front();


	return;
}


void initialize_global_params() {
	if(!ros::param::get("/sampling_interval", g_sampling_interval)){
		ROS_FATAL_STREAM("Could not start run_time_node sampling_interval not provided");
        exit(-1);
	}

	if(!ros::param::get("/v_max", g_v_max)) {
		ROS_FATAL_STREAM("Could not start run_time_node v_max not provided");
        exit(-1);
	}

	if(!ros::param::get("/sensor_max_range", g_sensor_max_range)) {
		ROS_FATAL_STREAM("Could not start run_time_node sensor_max_range not provided");
        exit(-1);
	}

	if(!ros::param::get("/ip_addr",ip_addr__global)){
		ROS_FATAL_STREAM("Could not start run_time_node ip_addr not provided");
        exit(-1);
	}
}


int get_point_count(double resolution, vector<std::pair<double, int>>& point_cloud_resolution_point_count){
	for (auto it= point_cloud_resolution_point_count.begin(); it!=point_cloud_resolution_point_count.end(); it++){
		if (it->first >= resolution){
			return it->second;
		}
	}

	ROS_ERROR_STREAM("should have found a resolution greater or equal to the requested one");
	exit(0);
}


void reactive_budgetting(double vel_mag, vector<std::pair<double, int>>& point_cloud_resolution_point_count){
	// -------
	// use velocity as the determiner of the knob values
	// -------

	// -- determine the point cloud resolution and perception resolution together.
	//    this is because
	double max_point_cloud_resolution = .2;
	int num_of_steps_on_y = 4;
	double min_point_cloud_resolution = pow(2, num_of_steps_on_y)*max_point_cloud_resolution;  //this value must be a power of two
	float offset_v_max = g_v_max/num_of_steps_on_y; // this is used to offsset the g_v_max; this is necessary otherwise, the step function basically never reacehs the min_point_cloud_resolution
	double point_cloud_resolution_temp =  (max_point_cloud_resolution - min_point_cloud_resolution)/(0 - (g_v_max-offset_v_max)) * vel_mag + max_point_cloud_resolution;
	int point_cloud_resolution_power_index = int(log2(point_cloud_resolution_temp/max_point_cloud_resolution));
	double point_cloud_resolution =  pow(2, point_cloud_resolution_power_index)*max_point_cloud_resolution;
	double perception_lower_resolution = point_cloud_resolution*2;
	ros::param::set("point_cloud_resolution", point_cloud_resolution);
	ros::param::set("perception_lower_resolution", perception_lower_resolution);
    profiling_container->capture("point_cloud_resolution", "single", point_cloud_resolution, g_capture_size);
    profiling_container->capture("perception_lower_resolution", "single", perception_lower_resolution, g_capture_size);

	// -- determine the number of points within point cloud
	double max_point_cloud_point_count_max_resolution = (double) get_point_count(point_cloud_resolution, point_cloud_resolution_point_count);
	double max_point_cloud_point_count_min_resolution = max_point_cloud_point_count_max_resolution/15;
	double min_point_cloud_point_count = max_point_cloud_point_count_min_resolution;
	//--  calculate the maximum number of points in an unfiltered point cloud as a function of resolution
	//    double modified_max_point_cloud_point_count = (max_point_cloud_point_count_max_resolution - max_point_cloud_point_count_min_resolution)/(max_point_cloud_resolution - min_point_cloud_resolution)*(point_cloud_resolution - max_point_cloud_resolution) + max_point_cloud_point_count_max_resolution;
	//    calucate num of points as function of velocity
	double point_cloud_num_points = (max_point_cloud_point_count_max_resolution - min_point_cloud_point_count)/(0 - g_v_max)*vel_mag + max_point_cloud_point_count_max_resolution;
	ros::param::set("point_cloud_num_points", point_cloud_num_points);
    profiling_container->capture("point_cloud_num_points", "single", point_cloud_num_points, g_capture_size);

	// -- determine how much of the space to keep
	float MapToTransferSideLength = 500 + (500 -40)/(0-g_v_max)*vel_mag;
	ros::param::set("MapToTransferSideLength", MapToTransferSideLength);

	// -- determine the planning budgets
	double piecewise_planning_budget_min = .01;
	double piecewise_planning_budget_max = .5;
//	double piecewise_planning_budget = (piecewise_planning_budget_max - piecewise_planning_budget_min)/(max_point_cloud_resolution- min_point_cloud_resolution)*point_cloud_resolution +
//			piecewise_planning_budget_max;

	vector<double> piecewise_planning_budget_vec{.6, .3, .1, .05, .01};
	double piecewise_planning_budget = piecewise_planning_budget_vec[point_cloud_resolution_power_index];
	ros::param::set("piecewise_planning_budget", piecewise_planning_budget);
	double smoothening_budget = piecewise_planning_budget;
	ros::param::set("smoothening_budget", smoothening_budget);
    profiling_container->capture("piecewise_planning_budget", "single", piecewise_planning_budget, g_capture_size);
    profiling_container->capture("smoothening_budget", "single", smoothening_budget, g_capture_size);

    if (DEBUG_RQT) {
    	debug_data.header.stamp = ros::Time::now();
    	debug_data.point_cloud_resolution = profiling_container->findDataByName("point_cloud_resolution")->values.back();
    	debug_data.perception_lower_resolution = profiling_container->findDataByName("perception_lower_resolution")->values.back();
    	debug_data.point_cloud_num_points =  profiling_container->findDataByName("point_cloud_num_points")->values.back();
    	debug_data.piecewise_planning_budget =  profiling_container->findDataByName("piecewise_planning_budget")->values.back();
    	debug_data.smoothening_budget =  profiling_container->findDataByName("smoothening_budget")->values.back();
    }
    //
}


// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "run_time_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    //signal(SIGINT, sigIntHandler);

    node_types.push_back("point_cloud");
    node_types.push_back("octomap");
    node_types.push_back("planning");


    std::string ns = ros::this_node::getName();
    //ros::Subscriber next_steps_sub = n.subscribe<mavbench_msgs::multiDOFtrajectory>("/next_steps", 1, next_steps_callback);
    initialize_global_params();

    profiling_container = new DataContainer();
    ROS_INFO_STREAM("ip to contact to now"<<ip_addr__global);



    vector<std::pair<double,int>>point_cloud_resolution_point_count;
	//.1, 14600
	//.2, 36700
	//.3 16500
	//.4 9500
	//.5 6000
	//.6 4200
	//.7 3000
	//.8 2400
	//.9 1900
	//1.0 1500
	//1.1 1270
	//1.2 1100
	//1.3 950
	//1.4 800
	//1.5 700
	//1.6 620
	//1.7 520
	//1.8 500
	//1.9 430
	//2.0 400
	//2.1 375
	//2.2 350
	//2.3 300
	//2.4 300
	//2.5 270
	//2.6 250
	//2.7 210
	//2.8 210
	//2.9 210
	//3 200
	//3.5 135
	//4 117
	//4.5 90

    float point_cloud_resolution_array[] = {.1, .2, .3, .4, .5, .6, .7, .8, .9, 1.0, 1.1, 1.2, 1.3, 1.4,
    							1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9,
								3.0, 3.6, 4, 4.5};

    float point_cloud_count_array[] = {146000, 36700, 16500, 9500, 6000, 4200, 3000, 2400, 1900, 1500, 1270,
    		1100, 950 , 800, 700, 620, 520, 500, 430, 400, 375, 350, 300, 300, 270, 250, 210, 210, 210,
								200, 135, 117, 90};

    for (int i=0; i< sizeof(point_cloud_resolution_array)/sizeof(point_cloud_resolution_array[0]); i++){
    	point_cloud_resolution_point_count.push_back(
    			std::make_pair(point_cloud_resolution_array[i], point_cloud_count_array[i]));
    }

    std::string ip_addr, localization_method;
    ros::param::get("/ip_addr", ip_addr);
    uint16_t port = 41451;

    if(!ros::param::get("/localization_method", localization_method)) {
        ROS_FATAL("Could not start occupancy map node. Localization parameter missing!");
        exit(-1);
    }

    if(!ros::param::get("/dynamic_budgetting", dynamic_budgetting)) {
        ROS_FATAL("Could not start occupancy map node. dynamic_budgetting parameter missing!");
        exit(-1);
    }

    if(!ros::param::get("/reactive_runtime", reactive_runtime)) {
        ROS_FATAL("Could not start occupancy map node. reactive_runtime parameter missing!");
        exit(-1);
    }

   if(!ros::param::get("/DEBUG_RQT", DEBUG_RQT)) {
        ROS_FATAL("Could not start run_time_thread node. DEBG_RQT parameter missing!");
        exit(-1);
   }

   if(!ros::param::get("/capture_size", g_capture_size)){
      ROS_FATAL_STREAM("Could not start run_time_thread. capture_size not provided");
      exit(-1);
    }

    ros::Publisher runtime_debug_pub = n.advertise<mavbench_msgs::runtime_debug>("/runtime_debug", 1);

    Drone drone(ip_addr.c_str(), port, localization_method);
    //Drone drone(ip_addr__global.c_str(), port);
	ros::Rate pub_rate(50);

    while (ros::ok())
	{
    	ros::spinOnce();
    	if (dynamic_budgetting){
    		if (reactive_runtime){
    			auto vel = drone.velocity();
    			auto vel_mag = calc_vec_magnitude(vel.linear.x, vel.linear.y, vel.linear.z);
    			reactive_budgetting(vel_mag, point_cloud_resolution_point_count);
    		}
    	}
        if (DEBUG_RQT) {runtime_debug_pub.publish(debug_data);}
    	pub_rate.sleep();
	}

}


