#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
//#include "controllers/DroneControllerBase.hpp"
//#include "common/Common.hpp"
#include "common.h"
#include <fstream>
#include "Drone.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "control_drone.h"
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include "common.h"
#include <cstring>
#include <string>
#include "profile_manager/start_profiling_srv.h"
#include "profile_manager/profiling_data_srv.h"
#include "profile_manager.h"
#include <datacontainer.h>

using namespace std;
std::string ip_addr__global;

ProfileManager *my_profile_manager;
DataContainer profiling_container;

void log_data_before_shutting_down()
{
    std::string ns = ros::this_node::getName();
    profiling_container.setStatsAndClear();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profile_manager::profiling_data_verbose_srv profiling_data_verbose_srv_inst;

    /*
    for (auto &data: profiling_container.container){
		profiling_data_srv_inst.request.key = data.data_key_name + " last window's avg: ";
		vector<double>* avg_stat = data.getStat("avg");
		if (avg_stat) {
			profiling_data_srv_inst.request.value = avg_stat->back();
		}
		else{
			profiling_data_srv_inst.request.value = nan("");
		}
		profile_manager.clientCall(profiling_data_srv_inst);
    }
	*/
    profiling_data_verbose_srv_inst.request.key = ros::this_node::getName()+"_verbose_data";
    profiling_data_verbose_srv_inst.request.value = "\n" + profiling_container.getStatsInString(); my_profile_manager->verboseClientCall(profiling_data_verbose_srv_inst); }

void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down();
        ros::shutdown();
    }
    exit(0);
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "probe_thread", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandlerPrivate);
 
    std::string ns = ros::this_node::getName();
    uint16_t port = 41451;
    // ros::param::get("/ip_addr",ip_addr__global);
    if(!ros::param::get("/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/ip_addr").c_str());

    return -1;
    }
    ROS_INFO_STREAM("ip to contact to now"<<ip_addr__global);
    Drone drone(ip_addr__global.c_str(), port);
    my_profile_manager = new ProfileManager("client", "/record_profiling_data", "/record_profiling_data_verbose");

    ros::Rate pub_rate(20);
    ros::Duration(5).sleep();

    bool started_cnting = false;

    while (ros::ok())
	{
        geometry_msgs::Twist twist = drone.velocity();
    	double velocity_magnitude =  calc_vec_magnitude(twist.linear.x, twist.linear.y, twist.linear.z);
        geometry_msgs::Accel accel = drone.acceleration();
        double acceleration_magnitude =  calc_vec_magnitude(accel.linear.x, accel.linear.y, accel.linear.z);

        profiling_container.capture("v_x", "single", twist.linear.x, 1); // @suppress("Invalid arguments")
        profiling_container.capture("v_y", "single", twist.linear.y, 1); // @suppress("Invalid arguments")
        profiling_container.capture("velocity", "single", velocity_magnitude, 1); // @suppress("Invalid arguments")
        profiling_container.capture("acceleration", "single", acceleration_magnitude, 1); // @suppress("Invalid arguments")
        if (started_cnting){
        	profiling_container.capture("delta_t", "end", ros::Time::now(), 1);
        }
        profiling_container.capture("delta_t", "start", ros::Time::now(), 1); // @suppress("Invalid arguments")
        started_cnting = true;

        drone.acceleration();
        pub_rate.sleep();
    }

}


