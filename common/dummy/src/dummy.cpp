#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <signal.h>
#include <string>
#include <sstream>
#include "common.h"
#include <map>
#include "std_msgs/Bool.h"
using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy");
    ros::NodeHandle nh;
    ros::Publisher dummy_pub = nh.advertise <std_msgs::Bool>("/dummy_topic", 20);
    ros::Rate loop_rate(2);
    int dont_matter; 
    std_msgs::Bool myBool;
    myBool.data = true; 
    while (ros::ok()) {
        cin>>dont_matter;
        dummy_pub.publish(myBool);
    }
    return 0;
}
