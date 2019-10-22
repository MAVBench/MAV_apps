#include <Drone.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <rosconsole/macros_generated.h>
#include <signal.h>
#include <cstdint>
#include <cstdlib>
#include <string>

#include "motion_planner.h"

MotionPlanner * mp_ptr = nullptr;
void sigIntHandlerPrivate(int signo) {
    if (signo == SIGINT) {
        mp_ptr->log_data_before_shutting_down();
        ros::shutdown();
    }
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_planner_node");
    const std::string blah = "~";
    ros::NodeHandle nh(blah);
    signal(SIGINT, sigIntHandlerPrivate);

    ROS_WARN("This node has no OctoMap instantiated! It's only useful for the scanning application");

    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    
    // Create a Drone object
    std::string ip_addr, localization_method;
    ros::param::get("/ip_addr", ip_addr);
    uint16_t port = 41451;
    if(!ros::param::get("/localization_method", localization_method)) {
        ROS_FATAL("Could not start occupancy map node. Localization parameter missing!");
        exit(-1);
    }
    ros::Duration(1).sleep(); //for img publisher to catch up and publish tf transforms
    Drone drone(ip_addr.c_str(), port, localization_method);
    // Create MotionPlanner
    MotionPlanner mp (nullptr, &drone);
    mp_ptr = &mp;

    while (ros::ok()) {
        ros::spinOnce();
        mp.spinOnce();
    }
}

