#include "ros/ros.h"

#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <math.h>
#include <signal.h>
#include <time.h>
#include <sstream>
#include <iterator>
#include <cstdlib>
#include <string>

#include "Drone.h"
#include "common.h"

using namespace std;

std::string ip_addr__global;
void sigIntPrivateHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

// *** F:DN main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_pose", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    
    std::string localization_method; 
    signal(SIGINT, sigIntPrivateHandler);
 
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_topic", 10);
    uint16_t port = 41451;
    ros::param::get("/publish_pose/ip_addr",ip_addr__global);

    ros::Rate pub_rate(20);
    if(!ros::param::get("/publish_pose/localization_method",localization_method))  {
        ROS_FATAL_STREAM("Could not start pubslish pose cause localization_method not provided");
        return -1; 
    }
    
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    while (ros::ok())
    {
        geometry_msgs::PoseWithCovariance  drone_pose = drone.pose_with_covariance();
        geometry_msgs::PoseWithCovarianceStamped drone_pose_stamped; 
        drone_pose_stamped.pose = drone_pose;
        drone_pose_stamped.header.stamp = ros::Time::now();
        drone_pose_stamped.header.frame_id = "world"; //possible change
                
        pose_pub.publish(drone_pose_stamped);
         
        pub_rate.sleep();
    }
}
