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

  uint16_t port = 41451;
  string ip_addr__global = "10.157.90.49";
  Drone drone(ip_addr__global.c_str(), port);

  while (ros::ok()) {
      auto pos = drone.gps();
      auto imu = drone.getIMUStats();

      tf::Transform transform;
      transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));

      tf::Quaternion q(imu.orientation.x(), imu.orientation.y(),
              imu.orientation.z(), imu.orientation.w());
      transform.setRotation(q);

      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                  "world", "gps"));

      cout << "Position: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
      loop_rate.sleep();
  }

  return 0;
}
