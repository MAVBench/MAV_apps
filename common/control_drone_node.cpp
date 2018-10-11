#include "ros/ros.h"
#include <string>
#include <signal.h>
#include "common.h"
#include "Drone.h"
#include "control_drone.h"

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "control_drone_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandler);
 
    uint16_t port = 41451;
    std::string ip_addr;

    ros::param::get("/ip_addr",ip_addr);
    ROS_INFO_STREAM("IP to connect to: " << ip_addr);
    
    Drone drone(ip_addr.c_str(), port);
    ros::Rate pub_rate(5);

    while (ros::ok())
    {
        control_drone(drone);
        pub_rate.sleep();
    }
}
