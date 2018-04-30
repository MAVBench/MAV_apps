#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <iostream>
#include <chrono>
#include <math.h>
#include <fstream>
#include <signal.h>

#include <std_msgs/Bool.h>

#include "Drone.h"

using namespace std;

void sigIntHandler(int sig)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "gps_publisher");
  ros::NodeHandle n;
  signal(SIGINT, sigIntHandler);

  double loop_rate_hz;
  if (!ros::param::get("/gps_publisher/loop_rate", loop_rate_hz))
          loop_rate_hz = 100;
  ros::Rate loop_rate(loop_rate_hz);

  tf::TransformBroadcaster br;
  tf::TransformListener listen;

  uint16_t port = 41451;
  string ip_addr;

  if (!ros::param::get("/ip_addr", ip_addr)) {
      ROS_ERROR("gps_publisher: IP address not available");
  }

  Drone drone(ip_addr.c_str(), port);

  // Initialize gps
  for (ros::Time start = ros::Time::now();
          (ros::Time::now() - start).toSec() < 5;) {
      uint64_t timestamp;
      drone.gps(timestamp);
      ros::Duration(0.2).sleep();
  }
  uint64_t offset_timestamp;
  auto offset = drone.gps(offset_timestamp);

  // ros::Publisher debug_pub = n.advertise<std_msgs::Bool>("gps_debug", 1);
  // std_msgs::Bool dummy;
  // dummy.data = true;

  uint64_t gps_last_timestamp = 0;
  while (ros::ok()) {
      uint64_t timestamp;
      auto pos = drone.gps(timestamp);
      // auto imu = drone.getIMUStats();

      // publish if last time stamp is sane
      if (timestamp > gps_last_timestamp) {
          gps_last_timestamp = timestamp;

          tf::StampedTransform transform;

          try {
              listen.lookupTransform("world", "ground_truth", ros::Time(0), transform);

              transform.setOrigin(tf::Vector3(pos.x-offset.x, pos.y-offset.y,
                          pos.z-offset.z));

              // tf::Quaternion q(imu.orientation.x(), imu.orientation.y(),
              //         imu.orientation.z(), imu.orientation.w());
              // transform.setRotation(q);

              uint32_t timestamp_s = uint32_t(timestamp / 1000000000);
              uint32_t timestamp_ns = uint32_t(timestamp % 1000000000);
              ros::Time timestamp_ros(timestamp_s, timestamp_ns);
              br.sendTransform(tf::StampedTransform(transform, timestamp_ros,
                          "world", "gps"));
              br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                          "world", "gps_now"));
          } catch (...) {
          }
      }

      // debug_pub.publish(dummy);
      // ros::spinOnce();

      loop_rate.sleep();
  }

  return 0;
}

