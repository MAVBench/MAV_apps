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

ros::Rate set_rate(Drone * drone, double trav_dis_before_replanning, double planner_min_freq, double planner_max_freq, double v_max){
    auto vel = drone->velocity();
    double speed = std::sqrt((vel.linear.x*vel.linear.x + vel.linear.y*vel.linear.y + vel.linear.z*vel.linear.z));
    double freq = ((planner_max_freq - planner_min_freq)/(v_max - 0))* speed + planner_min_freq; //fitting a line through min and max
    ROS_ERROR_STREAM(freq);
    return ros::Rate(freq);
    //return loop_rate;
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
    double trav_dis_before_replanning, planner_min_freq, planner_max_freq, v_max;

    if(!ros::param::get("/localization_method", localization_method)) {
        ROS_FATAL("Could not start occupancy map node. Localization parameter missing!");
        exit(-1);
    }


    if(!ros::param::get("/motion_planner/trav_dis_before_replanning", trav_dis_before_replanning)) {
        ROS_FATAL("Could not start motion_planner node node. trav_dis_before_replanning is missing");
        exit(-1);
    }

    if(!ros::param::get("/v_max", v_max)) {
        ROS_FATAL("Could not start motion_planner node node. v_max is missing");
        exit(-1);
    }


    if(!ros::param::get("/motion_planner/planner_min_freq", planner_min_freq)) {
        ROS_FATAL("Could not start motion_planner node node. planner_min_freq is missing");
        exit(-1);
    }

    if(!ros::param::get("/motion_planner/planner_max_freq", planner_max_freq)) {
        ROS_FATAL("Could not start motion_planner node node. planner_max_freq is missing");
        exit(-1);
    }





    ros::Duration(1).sleep(); //for img publisher to catch up and publish tf transforms
    Drone drone(ip_addr.c_str(), port, localization_method);
    // Create MotionPlanner
    MotionPlanner mp (nullptr, &drone);
    mp_ptr = &mp;
    double cnt = 1;
    ros::Rate loop_rate(30);
    //my_rate.sleep();
    ros::Duration(1).sleep();
    while (ros::ok()) {
    	mp.spinOnce();
    	//auto loop_rate = set_rate(&drone, trav_dis_before_replanning, planner_min_freq, planner_max_freq, v_max);
    	loop_rate.sleep();
    }
}

