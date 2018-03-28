#include "ros/ros.h"
#include <std_msgs/String.h>
//#include "template_library.hpp"
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "controllers/DroneControllerBase.hpp"
//#include "common/Common.hpp"
#include "common.h"
#include <fstream>
#include "Drone.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <time.h>
#include "std_msgs/Bool.h"
#include <signal.h>
#include "common.h"
#include <cstring>
#include <string>
#include "HelperFunctions/QuatRotEuler.h"
using namespace std;
std::string ip_addr__global;
using namespace msr::airlib;

sensor_msgs::Imu create_msg(IMUStats IMU_stats) {
    sensor_msgs::Imu IMU_msg;

	geometry_msgs::Quaternion q = setQuat(IMU_stats.orientation.x(),
                                              IMU_stats.orientation.y(),
                                              IMU_stats.orientation.z(),
                                              IMU_stats.orientation.w());
	tf::Quaternion qOrientation(q.x, q.y, q.z, q.w);
	
	tf::Matrix3x3 rotationMatrix(qOrientation);
	tf::Matrix3x3 rotationMatrixTransposed = rotationMatrix.transpose();
	tf::Vector3 accelerationWorld(IMU_stats.linear_acceleration[0], IMU_stats.linear_acceleration[1], IMU_stats.linear_acceleration[2]);

	
	tf::Transform tfMatrix(qOrientation);
	tf::Vector3 accelerationBody =  accelerationWorld;

    IMU_msg.orientation.x = IMU_stats.orientation.x();
    IMU_msg.orientation.y = IMU_stats.orientation.y();
    IMU_msg.orientation.z = IMU_stats.orientation.z();
    IMU_msg.orientation.w = IMU_stats.orientation.w();
    IMU_msg.orientation_covariance[0] = 0.1;
    IMU_msg.orientation_covariance[4] = 0.1;
    IMU_msg.orientation_covariance[8] = 0.1;

    IMU_msg.angular_velocity.x = IMU_stats.angular_velocity[0];
    IMU_msg.angular_velocity.y = IMU_stats.angular_velocity[1];
    IMU_msg.angular_velocity.z = IMU_stats.angular_velocity[2];
    IMU_msg.angular_velocity_covariance[0] = 0.1;
    IMU_msg.angular_velocity_covariance[4] = 0.1;
    IMU_msg.angular_velocity_covariance[8] = 0.1;

	IMU_msg.linear_acceleration.x = accelerationBody[0]; // IMU_stats.linear_acceleration[0];
    IMU_msg.linear_acceleration.y = accelerationBody[1]; // IMU_stats.linear_acceleration[1];
    IMU_msg.linear_acceleration.z = accelerationBody[2] ; // IMU_stats.linear_acceleration[2];
    IMU_msg.linear_acceleration_covariance[0] = 0.1;
    IMU_msg.linear_acceleration_covariance[4] = 0.1;
    IMU_msg.linear_acceleration_covariance[8] = 0.1;

    IMU_msg.header.stamp = ros::Time(uint32_t(IMU_stats.time_stamp / 1000000000 ), uint32_t(IMU_stats.time_stamp % 1000000000));
    
    return IMU_msg;
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "publish_imu", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    std::string ns = ros::this_node::getName();
    uint16_t port = 41451;
    if (!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL("Could not start mapping. Parameter missing! Looking for %s",
                (ns + "/ip_addr").c_str());
        return -1;
    }
    Drone drone(ip_addr__global.c_str(), port);
    
    int samples = 0;
    int misses = 0;

    ros::Rate pub_rate(110);
    sensor_msgs::Imu IMU_msg, IMU_msg2;
    ros::Publisher IMU_pub = nh.advertise <sensor_msgs::Imu>("imu_topic", 1);
    ros::Publisher IMU_pub2 = nh.advertise <sensor_msgs::Imu>("imu_topic2", 1);

    IMUStats IMU_stats;
    IMUStats IMU_stats2;

    uint64_t last_t = 0;
    uint64_t last_t2 = 0;

    while (ros::ok())
	{
        IMU_stats = drone.getIMUStats();  
        IMU_stats2 = drone.getIMUStats2();  

        IMU_msg = create_msg(IMU_stats);
        IMU_msg2 = create_msg(IMU_stats2);

        samples += 2;
        if (last_t < IMU_stats.time_stamp) {
            last_t = IMU_stats.time_stamp;
            IMU_pub.publish(IMU_msg);
        } else {
            misses++;
        }

        if (last_t2 < IMUStats2.time_stamp) {
            last_t2 = IMU_stats2.time_stamp;
            IMU_pub2.publish(IMU_msg2);
        } else {
            misses++;
        }

        pub_rate.sleep();
    }
}

