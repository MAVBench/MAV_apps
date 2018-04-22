#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <iostream>
#include <chrono>
#include <math.h>
#include <fstream>
#include <signal.h>

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

  ros::Rate loop_rate(5);

  tf::TransformBroadcaster br;
  tf::TransformListener listen;

  uint16_t port = 41451;
  string ip_addr;

  if (!ros::param::get("/ip_addr", ip_addr)) {
      ROS_ERROR("gps_publisher: IP address not available");
  }

  Drone drone(ip_addr.c_str(), port);

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

              transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));

              // tf::Quaternion q(imu.orientation.x(), imu.orientation.y(),
              //         imu.orientation.z(), imu.orientation.w());
              // transform.setRotation(q);

              uint32_t timestamp_s = uint32_t(timestamp / 1000000000);
              uint32_t timestamp_ns = uint32_t(timestamp % 1000000000);
              ros::Time timestamp_ros(timestamp_s, timestamp_ns);
              br.sendTransform(tf::StampedTransform(transform, timestamp_ros,
                          "world", "gps"));
          } catch (...) {
          }
      }
      loop_rate.sleep();
  }

  return 0;
}

