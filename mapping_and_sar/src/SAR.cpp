/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <visualization_msgs/Marker.h>
#include <thread>
#include <chrono>
#include "coord.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <multiagent_collision_check/Segment.h>

#include <mapping_and_sar/OD.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nbvplanner/nbvp_srv.h>
#include "common/Common.hpp"
#include <fstream>
#include "Drone.h"
#include "control_drone.h"
#include "common.h"

visualization_msgs::Marker path_to_follow_marker;
std::string stats_file_addr;

void OD_callback(const mapping_and_sar::OD::ConstPtr& msg){
    /* 
    if (msg->found) {
        ROS_INFO_STREAM("found the object at the location" << msg->point.x<< " " 
                <<msg->point.y<<" "<< msg->point.z);
            ros::shutdown(); 
    }
    */
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "SAR");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
      > (mav_msgs::default_topics::COMMAND_TRAJECTORY, 5);
  ROS_INFO("Started Search and rescue");
  ros::Subscriber obj_det_sub = nh.subscribe <mapping_and_sar::OD>("/OD_topic", 2, OD_callback);
  
  uint16_t port = 41451;
  std::string ip_addr__global;
  std::string localization_method; 
  std::string ns = ros::this_node::getName();
  if (!ros::param::get("/ip_addr", ip_addr__global)) {
    ROS_FATAL("Could not start SAR. Parameter missing! Looking for %s",
              (ns + "/ip_addr").c_str());
    return -1;
  }
  if(!ros::param::get("/localization_method",localization_method))  {
      ROS_FATAL_STREAM("Could not start SAR cause localization_method not provided");
    return -1; 
  }
  if(!ros::param::get("/stats_file_addr",stats_file_addr)){
      ROS_FATAL("Could not start SAR. Parameter missing! Looking for %s", 
              (ns + "/stats_file_addr").c_str());
  }


    //behzad change for visualization purposes
  ros::Publisher path_to_follow_marker_pub = nh.advertise<visualization_msgs::Marker>("path_to_follow_topic", 1000);
  geometry_msgs::Point p_marker;
  path_to_follow_marker.header.frame_id = "world";
  path_to_follow_marker.type = visualization_msgs::Marker::CUBE_LIST;
  path_to_follow_marker.action = visualization_msgs::Marker::ADD;
  path_to_follow_marker.scale.x = 0.3;


  //ROS_INFO_STREAM("ip address is"<<ip_addr__global); 
  //ROS_ERROR_STREAM("blah"<<ip_addr__global);
  Drone drone(ip_addr__global.c_str(), port, localization_method);

  //dummy segment publisher
  ros::Publisher seg_pub = nh.advertise <multiagent_collision_check::Segment>("evasionSegment", 1);

  std_srvs::Empty srv;
  //bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  /* 
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }
  */
  
  double dt; //= 1.0;
  double yaw_t; 
  //std::string ns = ros::this_node::getName();
  if (!ros::param::get(ns + "/nbvp/dt", dt)) {
    ROS_FATAL("Could not start SAR. Parameter missing! Looking for %s",
              (ns + "/nbvp/dt").c_str());
    return -1;
  }
  
  //behzad change using segment_dedicated_time instead of dt
  //ros::param::get("/follow_trajectory/yaw_t",yaw_t);
  if (!ros::param::get(ns + "/follow_trajectory/yaw_t",yaw_t)){
      ROS_FATAL_STREAM("Could not start SAR. Parameter missing! Looking for"<<
              "/follow_trajectory/yaw_t");
      return -1;
  }
  double t_offset; 
  if (!ros::param::get(ns + "/nbvp/t_offset",t_offset)){
      ROS_FATAL_STREAM("Could not start SAR. Parameter missing! Looking for"<<
              "/nbvp/t_offset");
      return -1;
  }
  
  double segment_dedicated_time = yaw_t + dt;
  control_drone(drone);
 /* 
  ros::param::get("/follow_trajectory/segment_dedicated_time",segment_dedicated_time);
    if (!ros::param::get("/follow_trajectory/segment_dedicated_time",segment_dedicated_time)){
    ROS_FATAL_STREAM("Could not start SAR. Parameter missing! Looking for"<<
              "/follow_trajectory/segment_dedicated_time");
    return -1;
  }
*/


  static int n_seq = 0;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

 // control_drone(drone);
  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  


  // This is the initialization motion, necessary that the known free space allows the planning
  // of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  /*
  for (double i = 0; i <= 1.0; i = i + 0.25) {
    
      
    //nh.param<double>("wp_x", trajectory_point.position_W.x(), 0.0);
    //nh.param<double>("wp_y", trajectory_point.position_W.y(), 0.0);
    //nh.param<double>("wp_z", trajectory_point.position_W.z(), 1.0);
   
    //behzad change, so the starting point is adjusted based on the drone 
    // starting postion (as opposed to being hardcoded)
    trajectory_point.position_W.x() = drone.pose().position.x;
    trajectory_point.position_W.y() = drone.pose().position.y;
    trajectory_point.position_W.z() = drone.pose().position.z;
    //ROS_INFO_STREAM("blah "<< trajectory_point.position_W.x()  <<" " << trajectory_point.position_W.y()  <<" " << trajectory_point.position_W.z());

    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -2*M_PI * i);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(1.0).sleep();
  }
  */
  spin_around(drone);
  // Move back a little bit
  auto cur_pos = drone.position();
  trajectory_point.position_W.x() = cur_pos.x - 1.5;
  trajectory_point.position_W.y() = cur_pos.y - 1.5;
  trajectory_point.position_W.z() = cur_pos.z;
  samples_array.header.seq = n_seq;
  samples_array.header.stamp = ros::Time::now();
  samples_array.points.clear();
  n_seq++;
  mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);


  //ros::shutdown();
  samples_array.points.push_back(trajectory_point_msg);
  trajectory_pub.publish(samples_array);
  ros::Duration(1.0).sleep();


  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 0;
  multiagent_collision_check::Segment dummy_seg;
  
  while (ros::ok()) {
      

    ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
    nbvplanner::nbvp_srv planSrv;
    planSrv.request.header.stamp = ros::Time::now();
    planSrv.request.header.seq = iteration;
    planSrv.request.header.frame_id = "world";
    //if  
    if (ros::service::call("nbvplanner", planSrv)) {
      n_seq++;
      if (planSrv.response.path.size() == 0) {
          ROS_ERROR("path size is zero");
          ros::Duration(1.0).sleep();
      }
      for (int i = 0; i < planSrv.response.path.size(); i++) {
        samples_array.header.seq = n_seq;
        samples_array.header.stamp = ros::Time::now();
        samples_array.header.frame_id = "world";
        samples_array.points.clear();
        tf::Pose pose;
        tf::poseMsgToTF(planSrv.response.path[i], pose);
        double yaw = tf::getYaw(pose.getRotation());
        trajectory_point.position_W.x() = planSrv.response.path[i].position.x;
        trajectory_point.position_W.y() = planSrv.response.path[i].position.y;
        // Add offset to account for constant tracking error of controller
        trajectory_point.position_W.z() = planSrv.response.path[i].position.z + 0.25;
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
        trajectory_point.setFromYaw(tf::getYaw(quat));
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        
        //behzad change for visualization purposes 
        p_marker.x = planSrv.response.path[i].position.x;
        p_marker.y = planSrv.response.path[i].position.y;
        p_marker.z = planSrv.response.path[i].position.z;
        path_to_follow_marker.points.push_back(p_marker);
        ROS_INFO_STREAM("TRAJECTORY PTS:"<< i<< " " << p_marker.x << " " << p_marker.y  << " " << p_marker.z);
        
        std_msgs::ColorRGBA c;
        c.g = 0; c.r = 1; c.b = 1;c.a = 1;
        path_to_follow_marker.colors.push_back(c);
        path_to_follow_marker_pub.publish(path_to_follow_marker);
        
        samples_array.points.push_back(trajectory_point_msg);
        trajectory_pub.publish(samples_array);
        //ros::Duration(1).sleep();
        ros::Duration(t_offset + segment_dedicated_time).sleep(); //changed, make sure segmentation time is smaller
      }
    } else {
      ROS_WARN_THROTTLE(1, "Planner not reachable");
      
      
      
      ros::Duration(t_offset + segment_dedicated_time).sleep(); //changed, make sure segmentation time is smaller
                                    //than 1.5*dt, this way we can finish up the command 
                                    //before sending out another one
    }
    iteration++;
  }
}
