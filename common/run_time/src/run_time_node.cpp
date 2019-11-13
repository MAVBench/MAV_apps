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

using namespace std;
std::string ip_addr__global;
double g_sensor_max_range, g_sampling_interval, g_v_max;
vector<double> g_timeSamples;

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
	TimeBudgetter timeBudgetter(maxSensorRange, maxVelocity, accelerationCoeffs, TimeIncr);
	double latency = 1.55;

	g_timeSamples = timeBudgetter.calcSamplingTime(traj, latency);
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

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "run_time_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    //signal(SIGINT, sigIntHandler);


    std::string ns = ros::this_node::getName();
    ros::Subscriber next_steps_sub = n.subscribe<mavbench_msgs::multiDOFtrajectory>("/next_steps", 1, next_steps_callback);
    initialize_global_params();

    uint16_t port = 41451;

    ROS_INFO_STREAM("ip to contact to now"<<ip_addr__global);
    //Drone drone(ip_addr__global.c_str(), port);
	ros::Rate pub_rate(5);


    while (ros::ok())
	{
        ros::spinOnce();
        pub_rate.sleep();
    }

}


