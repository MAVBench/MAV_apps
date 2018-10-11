#include "ros/ros.h"

#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <numeric>

#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include "common.h"
#include "Profiling.h"
#include <cstring>
#include <string>
#include "HelperFunctions/QuatRotEuler.h"

using namespace std;
std::string ip_addr__global;
using namespace msr::airlib;

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

    double loop_rate_hz;
    if (!ros::param::get("/publish_imu/loop_rate", loop_rate_hz))
        loop_rate_hz = 100;
    ros::Rate pub_rate(loop_rate_hz);

    sensor_msgs::Imu IMU_msg;
    ros::Publisher IMU_pub = nh.advertise <sensor_msgs::Imu>("imu_topic", 1);
    ros::Publisher error_pub = nh.advertise <std_msgs::Int32>("imu_error_rate", 1);
    
    IMUStats IMU_stats;
    uint64_t last_t = 0;
    
    std::list<double> x_list;
    std::list<double> y_list;
    std::list<double> z_list;
    const int max_list_size = 1;

    while (ros::ok())
    {
        //publish(drone);
        IMU_stats = drone.getIMUStats();  

        geometry_msgs::Quaternion q = setQuat(IMU_stats.orientation.x(),
                                              IMU_stats.orientation.y(),
                                              IMU_stats.orientation.z(),
                                              IMU_stats.orientation.w());
        tf::Vector3 accelerationWorld(IMU_stats.linear_acceleration[0], IMU_stats.linear_acceleration[1], IMU_stats.linear_acceleration[2]);        
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

        IMU_msg.linear_acceleration.x = accelerationBody[0];
        IMU_msg.linear_acceleration.y = accelerationBody[1];
        IMU_msg.linear_acceleration.z = accelerationBody[2];
        IMU_msg.linear_acceleration_covariance[0] = 0.1;
        IMU_msg.linear_acceleration_covariance[4] = 0.1;
        IMU_msg.linear_acceleration_covariance[8] = 0.1;

        IMU_msg.header.stamp = ros::Time(uint32_t(IMU_stats.time_stamp / 1000000000 ), uint32_t(IMU_stats.time_stamp % 1000000000));

        samples++;
        if (last_t < IMU_stats.time_stamp) {
            last_t = IMU_stats.time_stamp;
            IMU_pub.publish(IMU_msg);
        } else {
            misses++;

            std_msgs::Int32 err_rate;
            err_rate.data = (misses*100) / samples;
            error_pub.publish(err_rate);
        }

        pub_rate.sleep();
    }
}

