/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/thread.hpp>
#include <depth_image_proc/depth_conversions.h>
#include <signal.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <datacontainer.h>
#include "Profiling.h"
#include "profile_manager/start_profiling_srv.h"
#include "profile_manager/profiling_data_srv.h"
#include <mavbench_msgs/point_cloud_debug.h>
#include <profile_manager.h>
#include <unordered_map>
#include <unordered_set>
#include "boost/functional/hash.hpp"
using namespace std;
#include <mavbench_msgs/point_cloud_debug.h>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

class PointCloudXyzNodelet : public nodelet::Nodelet
{
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  int queue_size_;


  // Publications
  boost::mutex connect_mutex_;
  typedef sensor_msgs::PointCloud2 PointCloud;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pc_debug_pub;

  image_geometry::PinholeCameraModel model_;
    
  //Profiling 
  int pt_cld_ctr;
  long long pt_cld_generation_acc;
  long long img_to_pt_cloud_acc;
  bool CLCT_DATA_;
  bool DEBUG_;
  int data_collection_iteration_freq_; 
  std::string g_supervisor_mailbox; //file to write to when completed
  ros::ServiceClient profile_manager_client;
  bool measure_time_end_to_end;
  float point_cloud_width, point_cloud_height; //point cloud boundary
  int point_cloud_density_reduction; // How much to de-densify the point cloud by
  double point_cloud_resolution; // specifies the minimum distance between the points
  bool first_time = true;
  mavbench_msgs::point_cloud_debug debug_data = {};
  bool DEBUG_RQT = false;
  virtual void onInit();


  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  // Profiling
  //void sigIntHandlerPrivate(int signo);
  void log_data_before_shutting_down();
  ~PointCloudXyzNodelet();
  ProfileManager *profile_manager_;
  DataContainer *profiling_container;
};


void PointCloudXyzNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));
  //profile_manager_ = new ProfileManager("client", "/record_profiling_data", "/record_profiling_data_verbose");
  //signal(SIGINT, sigIntHandlerPrivate);
  
  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);

  profiling_container = new DataContainer();
  profile_manager_ = new ProfileManager("client", "/record_profiling_data", "/record_profiling_data_verbose");

  // Profiling 
  if (!ros::param::get("/DEBUG", DEBUG_)) {
    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for DEBUG");
    return;
  }
  if (!ros::param::get("/data_collection_iteration_freq_ptCld", data_collection_iteration_freq_)) {
    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for data_collection_iteration_freq_ptCld");
    return;
  }
  if (!ros::param::get("/CLCT_DATA", CLCT_DATA_)) {
    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for CLCT_DATA");
    return ;
  } 
  if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
      ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
      return ;
  }

  if (!ros::param::get("/measure_time_end_to_end", measure_time_end_to_end)) {
    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for measure_time_end_to_end");
    return ;
  }
  
  if (!ros::param::get("/point_cloud_width", point_cloud_width)) {
    ROS_FATAL("Could not start img_proc. point_cloud_width Parameter missing! Looking for measure_time_end_to_end");
    return ;
  }
  
  if (!ros::param::get("/point_cloud_height", point_cloud_height)) {
    ROS_FATAL("Could not start img_proc. point_cloud_height Parameter missing! Looking for measure_time_end_to_end");
    return ;
  }

  if (!ros::param::get("/point_cloud_density_reduction", point_cloud_density_reduction)) {
    ROS_FATAL("Could not start img_proc. point_cloud_density_reduction Parameter missing!");
    return ;
  }

  if (!ros::param::get("/point_cloud_resolution", point_cloud_resolution)) {
    ROS_FATAL("Could not start img_proc. point_cloud_resolution Parameter missing!");
    return ;
  }

  if(!ros::param::get("/DEBUG_RQT", DEBUG_RQT)){
      ROS_FATAL_STREAM("Could not start point cloud DEBUG_RQT not provided");
      return ;
   }

  pt_cld_ctr = 0;
  pt_cld_generation_acc = 0;
  img_to_pt_cloud_acc = 0;
  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_point_cloud_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
  pc_debug_pub = nh.advertise<mavbench_msgs::point_cloud_debug>("/point_cloud_debug", 1);
  
  profile_manager_client = 
      private_nh.serviceClient<profile_manager::profiling_data_srv>("/record_profiling_data", true);
}

 PointCloudXyzNodelet::~PointCloudXyzNodelet(){
     log_data_before_shutting_down(); 
     //sigIntHandlerPrivate(4);
 }

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyzNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_point_cloud_.getNumSubscribers() == 0)
  {
    sub_depth_.shutdown();
  }
  else if (!sub_depth_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_ = it_->subscribeCamera("image_rect", queue_size_, &PointCloudXyzNodelet::depthCb, this, hints);
  }
}

void PointCloudXyzNodelet::log_data_before_shutting_down(){
    //std::string ns = ros::this_node::getName();

    /*
    profiling_data_srv_inst.request.key = "img_to_cloud_commun_t";
    profiling_data_srv_inst.request.value = (((double)img_to_pt_cloud_acc)/1e9)/pt_cld_ctr;
    
    if (!profile_manager_client.call(profiling_data_srv_inst)){ 
        ROS_ERROR_STREAM("could not probe data using stats manager in point cloud");
        ros::shutdown();
    }
    
    profiling_data_srv_inst.request.key = "pt_cloud_generation_kernel";
    profiling_data_srv_inst.request.value = (((double)pt_cld_generation_acc)/1e9)/pt_cld_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager in point cloud");
            ros::shutdown();
        }
    }
	*/

    profiling_container->setStatsAndClear();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profile_manager::profiling_data_verbose_srv profiling_data_verbose_srv_inst;
    profiling_data_verbose_srv_inst.request.key = ros::this_node::getName()+"_verbose_data";
    profiling_data_verbose_srv_inst.request.value = "\n" + profiling_container->getStatsInString();
    profile_manager_->verboseClientCall(profiling_data_verbose_srv_inst);

}


/*
void PointCloudXyzNodelet::sigIntHandlerPrivate(int signo){
    log_data_before_shutting_down(); 
    //signal_supervisor(g_supervisor_mailbox, "kill"); 
    //ros::shutdown();
    //exit(0);
}
*/

// int get_entropy(std::unordered_map<double, double> frontier) {
//   int count = 0;
//   for (auto it = frontier.begin(); it != frontier.end(); ++it) {
//     if (it->second < 25) {
//       count += 1;
//     }
//   }
//   std::cout << count << std::endl; 
// }


void PointCloudXyzNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  PointCloud::Ptr cloud_msg(new PointCloud);

  ros::Time start_hook_t = ros::Time::now();
  
  img_to_pt_cloud_acc += (start_hook_t - depth_msg->header.stamp).toSec()*1e9;

  if (measure_time_end_to_end){ 
    cloud_msg->header = depth_msg->header;
  }else{
	  cloud_msg->header = depth_msg->header;
	  cloud_msg->header.stamp = ros::Time::now();
  }
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Update camera model
  model_.fromCameraInfo(info_msg);

  if (depth_msg->encoding == enc::TYPE_16UC1)
  {
    convert<uint16_t>(depth_msg, cloud_msg, model_);
  }
  else if (depth_msg->encoding == enc::TYPE_32FC1)
  {
    convert<float>(depth_msg, cloud_msg, model_);
  }
  else
  {
    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }
   
  ros::Time end_hook_t = ros::Time::now(); 
 
  // Profiling 
   
  if (CLCT_DATA_){
      ros::Duration pt_cloud_generation_t = end_hook_t - start_hook_t;
    if(DEBUG_) {
        //ROS_INFO_STREAM("pt_cloud generation"<<  pt_cloud_generation_t.toSec());
    }
    
    pt_cld_generation_acc += pt_cloud_generation_t.toSec()*1e9; 
    pt_cld_ctr++; 
     
    if ((pt_cld_ctr+1) % data_collection_iteration_freq_ == 0) { // this is because I can't figure out how
    															 // insert the shutting_down function in the sigInt
        log_data_before_shutting_down();       
    }
    
  }
 
  int n_points = cloud_msg->width * cloud_msg->height;
  
  // trim the point cloud by only keeping points of certain positions
  sensor_msgs::PointCloud2Iterator<float> cloud_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> cloud_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> cloud_z(*cloud_msg, "z");
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;

  profiling_container->capture("filtering", "start", ros::Time::now());
  
  // double point_cloud_resolution_in_cubic = std::sqrt(3*point_cloud_resolution*point_cloud_resolution);
  
  // TODO: make collisions less likely? Library to do this?
  auto point_hash_xyz = [](double x, double y, double z) {
    return boost::hash_value(make_tuple(x, y, z));
  };
  auto point_hash_xy = [](double x, double y) {
    return boost::hash_value(make_tuple(x, y));
  };
  auto round_to_resolution = [](double v, double resolution) {
    // add half, then truncate to round to nearest multiple of resolution
    v += boost::math::sign(v) * resolution / 2;
    return v - fmod(v, resolution);
  };

  // map of whether a point has been seen (rounded by resolution)
  std::unordered_set<double> seen;
  seen.clear();

  // TODO: calculate diagnostics for run time - right now just counts number of points within sensor max range
  int sensor_max_range = 25;
  int entropy_diagnostic = 0;

  // bucket points by distance from center for choosing best ones
  const int num_radius_buckets = 100;
  int radius_counters [num_radius_buckets];
  memset(&radius_counters[0], 0,  sizeof(radius_counters));
  double max_radius = sensor_max_range * std::pow(2, 0.5);
  double bucket_width = max_radius / num_radius_buckets;

  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
      // Filter by resolution
      // I don't think variable resolution makes much sense - it leave holes and stuff
      double distance_from_center = abs(*cloud_x); //pow(pow(*cloud_x, 2) + pow(*cloud_y, 2), 0.5);
      double resolution = point_cloud_resolution; // * max(1.0, distance_from_center);
      double rounded_x = round_to_resolution(*cloud_x, resolution);
      double rounded_y = round_to_resolution(*cloud_y, resolution);
      double rounded_z = round_to_resolution(*cloud_z, resolution);
      double hashed_point = point_hash_xyz(rounded_x, rounded_y, rounded_z);
      // printf("x: %f, y: %f, z: %f\n", *cloud_x, *cloud_y, *cloud_z);
      // printf("ROUNDED x: %f, y: %f, z: %f\n", rounded_x, rounded_y, rounded_z);
      // add to filtered cloud only if it is not close to one already seen
      if (seen.find(hashed_point) == seen.end()) {

        seen.insert(hashed_point);
        // printf("x: %f, y: %f, z: %f\n", *cloud_x, *cloud_y, *cloud_z);
			  xs.push_back(*cloud_x);
			  ys.push_back(*cloud_y);
			  zs.push_back(*cloud_z);
        // Add point to radius bucket
        int radius_bucket = (int) (std::pow(std::pow(*cloud_x, 2) + std::pow(*cloud_y, 2), 0.5) / bucket_width);
        radius_counters[radius_bucket] += 1;  
        if (*cloud_z < sensor_max_range) {
          entropy_diagnostic++;
        }
		  }
  }

  /*std::cout << "[ ";
  for (int i = 0; i < num_radius_buckets; i++) {
    std::cout << radius_counters[i] << ", ";
  } 
  std::cout << "]" << std::endl;*/

  // select the most central points to include
  int points_budget = 100;
  points_budget = entropy_diagnostic;  // prototype dynamic adjustment
  int max_radius_bucket = 0;
  int points_to_include = radius_counters[0] + radius_counters[1];
  while (points_to_include <= points_budget && max_radius_bucket < num_radius_buckets) {
    max_radius_bucket++;
    points_to_include += radius_counters[max_radius_bucket + 1];
  }

  // Pick best points
  std::vector<float> xs_fov_filtered;
  std::vector<float> ys_fov_filtered;
  std::vector<float> zs_fov_filtered;

  for (int i = 0; i < xs.size(); i++) {
    /*if (xs[i] > -1*scaled_cloud_width && xs[i] < scaled_cloud_width && 
        ys[i] > -1*scaled_cloud_height && ys[i] < scaled_cloud_height) {
      xs_fov_filtered.push_back(xs[i]);
      ys_fov_filtered.push_back(ys[i]);
      zs_fov_filtered.push_back(zs[i]);
    }*/
    int radius_bucket = (int) (std::pow(std::pow(xs[i], 2) + std::pow(ys[i], 2), 0.5) / bucket_width);
    if (radius_bucket <= max_radius_bucket) {
      xs_fov_filtered.push_back(xs[i]);
      ys_fov_filtered.push_back(ys[i]);
      zs_fov_filtered.push_back(zs[i]);
    }
  }

  if (DEBUG_RQT){
    debug_data.header.stamp = ros::Time::now();
    debug_data.point_cloud_point_cnt = xs_fov_filtered.size();
    pc_debug_pub.publish(debug_data);
  }

  // reset point cloud and load in filtered in points
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  // modifier.resize(0);
  modifier.resize(xs_fov_filtered.size());
  sensor_msgs::PointCloud2Iterator<float> new_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> new_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> new_z(*cloud_msg, "z");

  for(size_t i=0; i<xs_fov_filtered.size(); ++i, ++new_x, ++new_y, ++new_z){
      *new_x = xs_fov_filtered[i];
      *new_y = ys_fov_filtered[i];
      *new_z = zs_fov_filtered[i];
  }


  // ROS_INFO_STREAM("number of points in point cloud " << xs.size());
  // n_points = cloud_msg->width * cloud_msg->height;
  // printf("filtered size: %d \n", n_points);
 
  //cloud_msg->header.stamp = ros::Time::now();
  profiling_container->capture("filtering", "end", ros::Time::now());
  pub_point_cloud_.publish (cloud_msg);

}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::PointCloudXyzNodelet,nodelet::Nodelet);
