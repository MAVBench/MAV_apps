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

#include "Profiling.h"
#include "profile_manager/start_profiling_srv.h"
#include "profile_manager/profiling_data_srv.h"

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

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  // Profiling
  //void sigIntHandlerPrivate(int signo);
  void log_data_before_shutting_down();
  ~PointCloudXyzNodelet();
};



void PointCloudXyzNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  //signal(SIGINT, sigIntHandlerPrivate);
  
  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);
  
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
  
  pt_cld_ctr = 0;
  pt_cld_generation_acc = 0;
  img_to_pt_cloud_acc = 0;
  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_point_cloud_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
  
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
    profile_manager::profiling_data_srv profiling_data_srv_inst;

    std::string ns = ros::this_node::getName();
    
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
}


/*
void PointCloudXyzNodelet::sigIntHandlerPrivate(int signo){
    log_data_before_shutting_down(); 
    //signal_supervisor(g_supervisor_mailbox, "kill"); 
    //ros::shutdown();
    //exit(0);
}
*/

void PointCloudXyzNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  PointCloud::Ptr cloud_msg(new PointCloud);

  ros::Time start_hook_t = ros::Time::now();
  
  img_to_pt_cloud_acc += (start_hook_t - depth_msg->header.stamp).toSec()*1e9;

  cloud_msg->header = depth_msg->header;
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
     
    if ((pt_cld_ctr+1) % data_collection_iteration_freq_ == 0) {
        log_data_before_shutting_down();       
        //profiling_data_srv_inst.request.value = (((double)pt_cld_generation_acc)/1e9)/pt_cld_ctr;
    }
    
  }
 
  //cloud_msg->header.stamp = ros::Time::now();
  pub_point_cloud_.publish (cloud_msg);

}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::PointCloudXyzNodelet,nodelet::Nodelet);
