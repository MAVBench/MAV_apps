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

#include <phoenix_msg/error.h>

using namespace std;
std::string ip_addr__global;
using namespace msr::airlib;

sensor_msgs::Imu create_msg(const IMUStats& IMU_stats) {
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

phoenix_msg::error error_msg;

void error_callback(const phoenix_msg::error::ConstPtr& msg)
{
    error_msg = *msg;
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

    sensor_msgs::Imu IMU_msg, IMU_msg2, IMU_msg_official;
    ros::Publisher IMU_pub = nh.advertise <sensor_msgs::Imu>("imu_topic", 1);
    ros::Publisher IMU_pub2 = nh.advertise <sensor_msgs::Imu>("imu_topic2", 1);
    ros::Publisher IMU_pub_official = nh.advertise <sensor_msgs::Imu>("imu_official", 1);

    // ros::Subscriber error_sub = nh.subscribe<phoenix_msg::error>("error", 1, error_callback);

    double loop_rate_hz;
    if (!ros::param::get("/publish_imu/loop_rate", loop_rate_hz))
        loop_rate_hz = 100;
    ros::Rate pub_rate(loop_rate_hz);
    
    IMUStats IMU_stats;
    IMUStats IMU_stats2;

    uint64_t last_t = 0;
    uint64_t last_t2 = 0;
    uint64_t last_official_t = 0;

    //geometry_msgs::Vector3 linear_acceleration; 
    //geometry_msgs::Vector3 angular_velocity; 
    //geometry_msgs::Quaternion orientation;     
    //float roll, yaw, pitch;
    //msr::airlib::VectorMath::toEulerianAngle(IMU_stats.orientation, pitch, roll, yaw);
    //ROS_INFO_STREAM(yaw*180/M_PI);
    
    // std::list<double> x_list;
    // std::list<double> y_list;
    // std::list<double> z_list;
    // const int max_list_size = 1;

    while (ros::ok())
	{
        IMU_stats = drone.getIMUStats();  
        IMU_stats2 = drone.getIMUStats2();  

        IMU_msg = create_msg(IMU_stats);
        IMU_msg2 = create_msg(IMU_stats2);

        bool imu1_ok = true;
        bool imu2_ok = true;
        samples += 2;
        if (last_t < IMU_stats.time_stamp) {
            last_t = IMU_stats.time_stamp;
            IMU_pub.publish(IMU_msg);
        } else {
            misses++;
            imu1_ok = false;
        }

        if (last_t2 < IMU_stats2.time_stamp) {
            last_t2 = IMU_stats2.time_stamp;
            IMU_pub2.publish(IMU_msg2);
        } else {
            misses++;
            imu2_ok = false;
        }

        // if (!error_msg.imu_0) {
        //     imu1_ok = false;
        // }
        // if (!error_msg.imu_1) {
        //     imu2_ok = false;
        // }

        // average the two IMU msgs if they're both okay
        #define AVG(_field,_m) IMU_msg_official._field._m =\
                               (IMU_msg._field._m + IMU_msg2._field._m) / 2.0
        if (imu1_ok && imu2_ok && IMU_stats.time_stamp == IMU_stats2.time_stamp) {
            IMU_msg_official = IMU_msg;

            AVG(orientation,x);
            AVG(orientation,y);
            AVG(orientation,z);
            AVG(angular_velocity,x);
            AVG(angular_velocity,y);
            AVG(angular_velocity,z);
            AVG(linear_acceleration,x);
            AVG(linear_acceleration,y);
            AVG(linear_acceleration,z);

            if (last_official_t < IMU_stats.time_stamp) {
                IMU_pub_official.publish(IMU_msg_official);
                last_official_t = IMU_stats.time_stamp;
            } else {
                ROS_ERROR("imu message in disorder!");
            }
        } else if (imu1_ok) {
            if (last_official_t < IMU_stats.time_stamp) {
                IMU_pub_official.publish(IMU_msg);
                last_official_t = IMU_stats.time_stamp;
            } else {
                ROS_ERROR("imu message in disorder!");
            }
        } else if (imu2_ok) {
            if (last_official_t < IMU_stats2.time_stamp) {
                IMU_pub_official.publish(IMU_msg2);
                last_official_t = IMU_stats2.time_stamp;
            } else {
                ROS_ERROR("imu message in disorder!");
            }
        }

        ros::spinOnce();
        pub_rate.sleep();
    }
}

