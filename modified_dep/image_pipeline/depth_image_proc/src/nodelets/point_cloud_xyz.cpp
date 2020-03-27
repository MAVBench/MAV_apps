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
#include <profile_manager.h>
#include <unordered_map>
#include <unordered_set>
#include "boost/functional/hash.hpp"
#include <std_msgs/Bool.h>
#include <tuple>
using namespace std;
#include <mavbench_msgs/point_cloud_debug.h>
#include <string>
#include <mavbench_msgs/point_cloud_meta_data.h>
#include <mavbench_msgs/point_cloud_aug.h>
#include <mavbench_msgs/control.h>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

float sensor_max_range;
double point_cloud_max_z;
string gap_statistic_mode = "avg";
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
  ros::Publisher pub_point_cloud_aug_;
  ros::Publisher point_cloud_meta_data_pub;
  ros::Publisher pc_debug_pub;
  ros::Publisher control_pub;
  ros::Subscriber inform_pc_done_sub;

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
  // TODO: should num_points be the API for runtime? Seems pretty reasonable.
  int point_cloud_num_points; // budget for how many points we can send
  string point_cloud_num_points_filtering_mode; // whether to use raidus based or height based filtering
  float point_cloud_width, point_cloud_height; //point cloud boundary
  int point_cloud_density_reduction; // How much to de-densify the point cloud by
  double pc_res; // specifies the minimum distance between the points
  double pc_vol_ideal;
  double om_to_pl_vol_ideal;
  double om_to_pl_res;
  double ppl_vol_ideal;
  bool first_time = true;
  mavbench_msgs::point_cloud_debug debug_data = {};
  bool DEBUG_RQT = false;
  bool DEBUG_VIS = false;
  double capture_size = 600;
  bool knob_performance_modeling = false;
  bool knob_performance_modeling_for_pc_om = false;
  virtual void onInit();
  double sensor_to_actuation_time_budget_to_enforce;
  double om_latency_expected;
  double om_to_pl_latency_expected;
  double ppl_latency_expected;
  double velocity_to_budget_on;
  double ee_latency_expected;
  ros::Time img_capture_time_stamp; // -- time the snap shot was taken
  bool  new_control_data = false;

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);


  double estimated_to_actual_vol_correction(double estimated_vol);   // for correction the estimated value to real
  double actual_to_estimated_vol_correction(double actual_vol);   // for correction of the actual values to estimated (since pc lives in the estimated world for enforcement)

  // Profiling
  //void sigIntHandlerPrivate(int signo);
  void log_data_before_shutting_down();
  ~PointCloudXyzNodelet();
  void inform_pc_done_cb(std_msgs::Bool);
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
  ros::Duration(1).sleep();
  // Read parameters
  private_nh.param("queue_size", queue_size_, 1);

  profiling_container = new DataContainer();
  profile_manager_ = new ProfileManager("client", "/record_profiling_data", "/record_profiling_data_verbose");

  // Profiling 
  if (!ros::param::get("/DEBUG", DEBUG_)) {
    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for DEBUG");
    exit(0);
  }
  if (!ros::param::get("/data_collection_iteration_freq_ptCld", data_collection_iteration_freq_)) {
    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for data_collection_iteration_freq_ptCld");
    exit(0);
    return;
  }


   if(!ros::param::get("/ppl_latency_expected", ppl_latency_expected)){
	    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for ppl_latency_expected");
	    exit(0);
	    return;
   }

  if (!ros::param::get("/CLCT_DATA", CLCT_DATA_)) {
    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for CLCT_DATA");
    exit(0);
    return ;
  } 
  if(!ros::param::get("/supervisor_mailbox",g_supervisor_mailbox))  {
      ROS_FATAL_STREAM("Could not start mapping supervisor_mailbox not provided");
      exit(0);
      return ;
  }

  if (!ros::param::get("/measure_time_end_to_end", measure_time_end_to_end)) {
    ROS_FATAL("Could not start img_proc. Parameter missing! Looking for measure_time_end_to_end");
      exit(0);
    return ;
  }

  if (!ros::param::get("/point_cloud_num_points", point_cloud_num_points)) {
    ROS_FATAL("Could not start img_proc. point_cloud_num_points Parameter missing!");
    exit(0);
    return ;
  }

  if (!ros::param::get("/pc_vol_ideal", pc_vol_ideal)) {
    ROS_FATAL("Could not start point_Cloud . pc_vol_ideal Parameter missing!");
    exit(0);
    return ;
  }



 if (!ros::param::get("/point_cloud_num_points_filtering_mode", point_cloud_num_points_filtering_mode)) {
    ROS_FATAL("Could not start img_proc. point_cloud_num_points_filtering_mode Parameter missing!");
    exit(0);
    return ;
  }

  
  if (!ros::param::get("/point_cloud_width", point_cloud_width)) {
    ROS_FATAL("Could not start img_proc. point_cloud_width Parameter missing! Looking for measure_time_end_to_end");
    exit(0);
    return ;
  }
  
  if (!ros::param::get("/point_cloud_height", point_cloud_height)) {
    ROS_FATAL("Could not start img_proc. point_cloud_height Parameter missing! Looking for measure_time_end_to_end");
    exit(0);
    return ;
  }

  if (!ros::param::get("/point_cloud_density_reduction", point_cloud_density_reduction)) {
    exit(0);
    return ;
  }

  if (!ros::param::get("/pc_res", pc_res)) {
    ROS_FATAL("Could not start img_proc. pc_res Parameter missing!");
    exit(0);
    return ;
  }


   if(!ros::param::get("/gap_statistic_mode", gap_statistic_mode)){
      ROS_FATAL_STREAM("Could not start point cloud gap_statistic_mode not provided");
      exit(0);
      return ;

   }

  if(!ros::param::get("/DEBUG_RQT", DEBUG_RQT)){
      ROS_FATAL_STREAM("Could not start point cloud DEBUG_RQT not provided");
      exit(0);
      return ;

   }

    if(!ros::param::get("/DEBUG_VIS", DEBUG_VIS)){
      ROS_FATAL_STREAM("Could not start point cloud DEBUG_VIS not provided");
      exit(0);
      return ;

   }

    if(!ros::param::get("/capture_size", capture_size)){
      ROS_FATAL_STREAM("Could not start point cloud capture_size not provided");
      exit(0);
      return ;
   }

    if(!ros::param::get("/knob_performance_modeling", knob_performance_modeling)){
    	ROS_FATAL_STREAM("Could not start point cloud knob_performance_modeling not provided");
    	exit(0);
    	return ;

    }

   if(!ros::param::get("/knob_performance_modeling_for_pc_om", knob_performance_modeling_for_pc_om)){
    	ROS_FATAL_STREAM("Could not start point cloud knob_performance_modeling_for_pc_om not provided");
    	exit(0);
    	return ;

    }




    if(!ros::param::get("/sensor_to_actuation_time_budget_to_enforce", sensor_to_actuation_time_budget_to_enforce)){
    	ROS_FATAL_STREAM("Could not start point cloud sensor_to_actuation_time_budget_to_enforce not provided");
    	exit(0);
    	return ;

    }

    if(!ros::param::get("/velocity_to_budget_on", velocity_to_budget_on)){
    	ROS_FATAL_STREAM("Could not start point cloud velocity_to_budget_on not provided");
    	exit(0);
    	return ;

    }


    if (knob_performance_modeling){
    	capture_size = 1;
    }

    pt_cld_ctr = 0;
    pt_cld_generation_acc = 0;
    img_to_pt_cloud_acc = 0;
    // Monitor whether anyone is subscribed to the output
    //ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzNodelet::connectCb, this);

    // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
    //boost::lock_guard<boost::mutex> lock(connect_mutex_);
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_ = it_->subscribeCamera("image_rect", queue_size_, &PointCloudXyzNodelet::depthCb, this, hints);
    pub_point_cloud_aug_ = nh.advertise<mavbench_msgs::point_cloud_aug>("points_aug", 1);//, connect_cb, connect_cb);
    pub_point_cloud_ = nh.advertise<PointCloud>("points", 1);//, connect_cb, connect_cb);

    //point_cloud_meta_data_pub = nh.advertise<mavbench_msgs::point_cloud_meta_data>("/pc_meta_data", 1);
    pc_debug_pub = nh.advertise<mavbench_msgs::point_cloud_debug>("/point_cloud_debug", 1);
    control_pub = nh.advertise<mavbench_msgs::control>("control_to_crun", 1);
    inform_pc_done_sub =  nh.subscribe("inform_pc_done", 1, &PointCloudXyzNodelet::inform_pc_done_cb, this);

    profile_manager_client =
    		private_nh.serviceClient<profile_manager::profiling_data_srv>("/record_profiling_data", true);
}

 PointCloudXyzNodelet::~PointCloudXyzNodelet(){
     log_data_before_shutting_down(); 
 }

 void PointCloudXyzNodelet::inform_pc_done_cb(std_msgs::Bool msg){
;
	log_data_before_shutting_down();
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



unsigned long int inline point_hash_xyz (double x, double y, double z) {
    return boost::hash_value(make_tuple(x, y, z));
}

unsigned long int inline point_hash_xy(double x, double y) {
    return boost::hash_value(make_tuple(x, y));
}

double inline round_to_resolution(double v, double resolution) {

   /*
	// add half, then truncate to round to nearest multiple of resolution
    v += boost::math::sign(v) * resolution / 2;
    return v - fmod(v, resolution);
    */
	//v= floor(v*100)/100;
	double result = floor((1./resolution)*v)*resolution;
	return result;
}

double inline round_to_resolution_(double v, double resolution) {
    // add half, then truncate to round to nearest multiple of resolution
    //v += boost::math::sign(v) * resolution / 2;
    return v + (resolution - fmod(v, resolution));
}




// use for redundancy removal #smart-flow stuff not for roborun
void filterByResolutionAndEdges(sensor_msgs::PointCloud2Iterator<float> &cloud_x, sensor_msgs::PointCloud2Iterator<float> &cloud_y, sensor_msgs::PointCloud2Iterator<float> &cloud_z,
    std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, int n_points, float resolution){
  // map of whether a point has been seen (rounded by resolution)
  std::unordered_set<double> seen;
  std::map<double, std::tuple<double,double>> min_y;
  std::map<double, std::tuple<double, double>> max_y;
  std::map<double, std::tuple<double,double>> min_x;
  std::map<double, std::tuple<double, double>> max_x;

  seen.clear();

  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
      // Filter by resolution
      // I don't think variable resolution makes much sense - it leave holes and stuff
      double distance_from_center = abs(*cloud_x); //pow(pow(*cloud_x, 2) + pow(*cloud_y, 2), 0.5);
      double rounded_x = round_to_resolution(*cloud_x, resolution);
      double rounded_y = round_to_resolution(*cloud_y, resolution);
      double rounded_z = round_to_resolution(*cloud_z, resolution);
      double hashed_point = point_hash_xyz(rounded_x, rounded_y, rounded_z);

      // printf("x: %f, y: %f, z: %f\n", *cloud_x, *cloud_y, *cloud_z);
      // printf("ROUNDED x: %f, y: %f, z: %f\n", rounded_x, rounded_y, rounded_z);
      // add to filtered cloud only if it is not close to one already seen
      if (seen.find(hashed_point) == seen.end()) {

        seen.insert(hashed_point);

        // add min of y
        auto it = min_y.find(rounded_x);
        if (it != min_y.end()){
        	double y = get<0>(it->second);
        	double z = get<1>(it->second);
        	//tie(y,z) = it->second;
        	it->second = make_tuple(std::min(rounded_y, y),
        		z);
        }else{
        	min_y[rounded_x] = make_tuple(rounded_y, rounded_z);
        }

        auto it_ = max_y.find(rounded_x);
        if (it_ != max_y.end()){
        	double y = get<0>(it->second);
        	double z = get<1>(it->second);
        	//tie(y,z) = it->second;
        	it_->second = make_tuple(std::max(rounded_y, y),
        		z);
        }else{
        	max_y[rounded_x] = make_tuple(rounded_y, rounded_z);
        }

        auto it_min_x = min_x.find(rounded_y);
        if (it_min_x != min_x.end()){
        	double x = get<0>(it->second);
        	double z = get<1>(it->second);
        	//tie(y,z) = it->second;
        	it_min_x->second = make_tuple(std::min(rounded_x, x),
        		z);
        }else{
        	min_x[rounded_y] = make_tuple(rounded_x, rounded_z);
        }

        auto it_max_x = max_x.find(rounded_y);
        if (it_max_x != max_x.end()){
        	double x = get<0>(it->second);
        	double z = get<1>(it->second);
        	//tie(y,z) = it->second;
        	it_max_x->second = make_tuple(std::max(rounded_x, x),
        		z);
        }else{
        	max_x[rounded_y] = make_tuple(rounded_x, rounded_z);
        }

      }
  }

  for (auto it = min_y.begin(); it!=min_y.end(); it++){
		  xs.push_back(it->first);
		  double y,z;
          tie(y,z) = it->second;
		  ys.push_back(y);
		  zs.push_back(z);
  }

  for (auto it = max_y.begin(); it!=max_y.end(); it++){
		  xs.push_back(it->first);
		  double y,z;
          tie(y,z) = it->second;
		  ys.push_back(y);
		  zs.push_back(z);
  }

    for (auto it = min_x.begin(); it!=min_x.end(); it++){
		  ys.push_back(it->first);
		  double x,z;
          tie(x,z) = it->second;
		  xs.push_back(x);
		  zs.push_back(z);
  }

  for (auto it = max_x.begin(); it!=max_x.end(); it++){
		  ys.push_back(it->first);
		  double x,z;
          tie(x,z) = it->second;
		  xs.push_back(x);
		  zs.push_back(z);
  }




}

// to understand the gap size and volume
float** runDiagnosticsUsingGriddedApproach(sensor_msgs::PointCloud2Iterator<float> cloud_x, sensor_msgs::PointCloud2Iterator<float> cloud_y, sensor_msgs::PointCloud2Iterator<float> cloud_z,
  int n_points, const int grid_size, float diagnostic_resolution, double &sensor_volume_to_digest_estimated, double &area_to_digest,
  double &gap_statistics_min, double& gap_statistics_max, double& gap_statistics_avg, double &obs_dist_statistics_min, double  &obs_dist_statistics_avg_from_pc
  ){
  double last_x, last_y, last_z, this_x, this_y, this_z, first_x, first_y, first_z, this_z_under_estimate;
  bool first_itr = true;
  vector<int> gap_ctr_per_row_vec;

  bool gap_started = false;
  bool new_row = false;
  double gap_size = 0;
  int gap_ctr_per_row = 0;
  vector<float> gap_size_vec;
  double last_rounded_x, last_rounded_y, last_rounded_z;
  double this_y_no_norm, last_y_no_norm;
  int cntr= 0;
 int cntr1 = 0;
 bool first_point = true; // first point in the first gap
 obs_dist_statistics_avg_from_pc = 0;
 obs_dist_statistics_min = sensor_max_range;

 //const float diagnostic_resolution = 1;
 //const int point_cloud_wc_side_length = 60;
 // const int grided_volume_size = (int)point_cloud_wc_side_length/diagnostic_resolution;
 // const int grided_size = 60; // int)point_cloud_wc_side_length/diagnostic_resolution;
 float **gridded_volume = 0;
 gridded_volume = new float*[grid_size];
 //memset(gridded_volume, 0, sizeof(gridded_volume[0][0]) * grided_volume_size* grided_volume_size);

 for (int h = 0; h < grid_size; h++)
 {
	 gridded_volume[h] = new float[grid_size];
	 for (int w = 0; w < grid_size; w++)
	 {
		 gridded_volume[h][w] = 0;
	 }
 }

 // iterate through points row by row and get gaps for each row
  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
      this_x = *cloud_x;
      this_y = *cloud_y;
      this_z = *cloud_z;
      this_z_under_estimate = *cloud_z;
      this_y_no_norm = this_y;


      // -- if outside of the sphere with the radius of sensor_max_range, project it back
      double norm = sqrt(this_z*this_z + this_y*this_y + this_x*this_x);
      if (norm > sensor_max_range){
    	  this_x = (sensor_max_range/norm)*this_x;
    	  this_y = (sensor_max_range/norm)*this_y;
    	  this_z = (sensor_max_range/norm)*this_z;
    	  this_z_under_estimate = ((sensor_max_range)/norm)*this_z_under_estimate;
      }
      double new_norm = sqrt(this_z*this_z + this_y*this_y + this_x*this_x);

      if (new_norm < obs_dist_statistics_min){ // -- we might wanna use this to determine the resolution
    	  	  	  	  	  	  	  	    // -- i.e., make sure resolution is bigger than closest_obstacle
    	  obs_dist_statistics_min = new_norm;
      }
      obs_dist_statistics_avg_from_pc += new_norm;

      new_row = this_x < last_x;
      //bool point_is_gap = this_z > (sensor_max_range - resolution);
      float gap_error_margin = .05;
      bool point_is_gap = norm >= sensor_max_range - gap_error_margin*(sensor_max_range);

     /*
      if (!first_itr && this_y_no_norm < last_y_no_norm){
    	  ROS_INFO_STREAM("("<<last_x<<"," <<last_y<<","<<last_z<<")  ("<< this_x<<","<< this_y<<","<< this_z<<")");
    	  cntr++;
      }else{
    	  cntr1++;
      }
      */
      // TODO: figure out whether the projection to the sphare, impact the gap calculation
      // worse case, scenario, we can just not project for the gap diagnostics
      if (new_row && !first_itr){
		  if (gap_started) {
			  gap_size_vec.push_back(gap_size);
			  gap_started = false;
			  gap_size = 0;
			  gap_ctr_per_row +=1;
		  }
		  gap_ctr_per_row_vec.push_back(gap_ctr_per_row);
		  gap_ctr_per_row = 0;
      } else { //within row, or first time
     	  if (point_is_gap) { //if gap, increase the gap size
     		  gap_started = true;
     		  if (!first_point){
     			  gap_size += fabs(this_x - last_x);
     		  }
     		  first_point = false;
     	  }else if (gap_started){ //stop the gap
     		  gap_size_vec.push_back(gap_size);
			  gap_started = false;
			  gap_ctr_per_row +=1;
			  gap_size = 0;
     	  }
      }

      int x_index = ((int)floor(1/diagnostic_resolution)*round_to_resolution(this_x, diagnostic_resolution)) + (int)grid_size/2;
      int y_index = ((int)floor(1/diagnostic_resolution)*round_to_resolution(this_y, diagnostic_resolution)) + (int)grid_size/2;
      //gridded_volume[x_index][y_index] = max(round_to_resolution_(this_z, diagnostic_resolution), (double)gridded_volume[x_index][y_index]);
      if (gridded_volume[x_index][y_index] == 0){
    	  gridded_volume[x_index][y_index] = this_z_under_estimate;
      }else{
    	  gridded_volume[x_index][y_index] = min(this_z_under_estimate, (double)gridded_volume[x_index][y_index]);
      }


      if (!new_row && !first_itr ){
    	  if (fabs(last_z - this_z) < .5){
//    		  sensor_volume_to_digest_estimated += (1.0/3)*pow(diagnostic_resolution,2)*this_z; //approximating using only x
    		  area_to_digest += pow(this_x - last_x, 2); //approximating using only x
    	  }
      }

      last_x = this_x;
      last_y = this_y;
      last_z = this_z;
      last_y_no_norm = this_y_no_norm;
      first_itr = false;
  }
  gap_size_vec.push_back(gap_size);

  for (int i =0; i < grid_size; i++){
	  for (int j =0; j< grid_size; j++){
		 sensor_volume_to_digest_estimated += (1.0/3.0)*pow(diagnostic_resolution,2)*gridded_volume[i][j];
	  }
  }


  float max_gap = 0;
  float avg_gap;
  float min_gap = sensor_max_range;

  // printing stuff out
  if (gap_size_vec.size() != 0){
	  float acc = 0;
  // cout<<"gap_size:";
	  for (auto it = gap_size_vec.begin(); it != gap_size_vec.end(); it++){
  // cout<<*it<<",";
		  acc+=*it;
		  if (*it > max_gap){
			  max_gap = *it;
		  }
		  if (*it < min_gap){
			  min_gap = *it;
		  }
	  }
	  //cout<<endl;
	  avg_gap =  acc/gap_size_vec.size();
	  float std = 0;
	  for (auto it = gap_size_vec.begin(); it != gap_size_vec.end(); it++){
		  std += pow(*it - avg_gap,2);
	  }
	  std = std/gap_size_vec.size();
	  //cout<<"avg:"<< avg<< " std:" << std<<"size"<<gap_size_vec.size()<<endl;
	  //cout<<"max_gap:"<< max_gap<<endl;
  }

  /*
  cout<<"gap_ctr_per_row:";
   for (auto it = gap_ctr_per_row_vec.begin(); it != gap_ctr_per_row_vec.end(); it++){
		  cout<<*it<<",";
   }
   cout<<endl;
*/
  //cout<<endl<<endl;
  //ROS_INFO_STREAM("x_y_not_in_order_cntr"<< x_y_not_in_order_cntr<< "x_not_in_order_cntr"<<x_not_in_order_cntr<< "y_not_in_order_cntr"<<y_not_in_order_cntr<< "x_y_in_order_cntr"<<x_y_in_order_cntr);
//  ROS_INFO_STREAM("counters for outer of order"<<cntr);
 // ROS_INFO_STREAM("------------------------------");


  //if (gap_statistic_mode == "min"){
  gap_statistics_min= min_gap;
//  }else if (gap_statistic_mode == "avg"){
  gap_statistics_avg = avg_gap;
  gap_statistics_max= max_gap;
 // }

  obs_dist_statistics_avg_from_pc /= float(n_points);
  return gridded_volume;
}


// to understand the gap size and volume
void runDiagnostics(sensor_msgs::PointCloud2Iterator<float> cloud_x, sensor_msgs::PointCloud2Iterator<float> cloud_y, sensor_msgs::PointCloud2Iterator<float> cloud_z,
  int n_points, float resolution, double &sensor_volume_to_digest_estimated, double &area_to_digest){
  double last_x, last_y, last_z, this_x, this_y, this_z, first_x, first_y, first_z;
  last_x = sensor_max_range; // -- to make sure, first  iteration is considered a new row
  bool first_itr = true;
  vector<int> gap_ctr_per_row_vec;
  bool gap_started = false;
  bool new_row = false;
  double gap_size = 0;
  int gap_ctr_per_row = 0;
  vector<float> gap_size_vec;
  double last_rounded_x, last_rounded_y, last_rounded_z;
  double this_y_no_norm, last_y_no_norm;
  int cntr= 0;
 int cntr1 = 0;
   double max_x = 0;
   double max_y = 0;
   double max_z = 0;

   double min_x = 1000;
   double min_y = 1000;
   double min_z = 1000;
   double last_row_y;
   double delta_y_row;
   int new_row_cntr = 0;
   bool first_row = true;
   // -- iterate through points row by row and get gaps for each row
  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
      this_x = *cloud_x;
      this_y = *cloud_y;
      this_z = *cloud_z;
      this_y_no_norm = this_y;

      // -- if outside of the sphere with the radius of sensor_max_range, project it back
      double norm = sqrt(this_z*this_z + this_y*this_y + this_x*this_x);
      if (norm > sensor_max_range){
    	  this_x = (sensor_max_range/norm)*this_x;
    	  this_y = (sensor_max_range/norm)*this_y;
    	  this_z = (sensor_max_range/norm)*this_z;
      }
      new_row = this_x < last_x;
      new_row_cntr = new_row ? new_row_cntr+1: new_row_cntr;
      first_row = new_row_cntr <= 1 ? true : false;

      //bool point_is_gap = this_z > (sensor_max_range - resolution);
      bool point_is_gap = norm > sensor_max_range;

      // TODO: figure out whether the projection to the sphare, impact the gap calculation
      // worse case, scenario, we can just not project for the gap diagnostics
      if (new_row && !first_itr){
    	  if (gap_started) {
			  gap_size_vec.push_back(gap_size);
			  gap_started = false;
			  gap_size = 0;
			  gap_ctr_per_row +=1;
		  }
		  gap_ctr_per_row_vec.push_back(gap_ctr_per_row);
		  gap_ctr_per_row = 0;
      } else { //within row, or first time
     	  if (point_is_gap) { //if gap, increase the gap size
     		  gap_started = true;
     		  gap_size += fabs(this_x - last_x);
     	  }else if (gap_started){ //stop the gap
     		  gap_size_vec.push_back(gap_size);
			  gap_started = false;
			  gap_ctr_per_row +=1;
			  gap_size = 0;
     	  }
      }

      if (!new_row && !first_row){
    	  //sensor_volume_to_digest_estimated += (1.0/3)*pow(this_x - last_x,2)*this_z); //approximating using only x
    	  auto blah = fabs(this_y - last_row_y);
    	  auto blah2 = fabs(this_x - last_x);
    	  auto blah3 = fabs(blah2 - blah);
    	  sensor_volume_to_digest_estimated += (1.0/3)*fabs(this_x - last_x)*fabs(this_y - last_row_y)*this_z; //approximating using only x
    	  area_to_digest += pow(this_x - last_x, 2); //approximating using only x
      }else{
    	  //delta_y_row = last_row_y - this_y;
    	  if (new_row){
    		  last_row_y = this_y;
    	  }
      }

      last_x = this_x;
      last_y = this_y;
      last_z = this_z;
      last_y_no_norm = this_y_no_norm;
      first_itr = false;
  }
  gap_size_vec.push_back(gap_size);

  /*
  // printing stuff out
  if (gap_size_vec.size() != 0){
	  float acc = 0;
	  cout<<"gap_size:";
	  for (auto it = gap_size_vec.begin(); it != gap_size_vec.end(); it++){
		  cout<<*it<<",";
		  acc+=*it;
	  }
	  cout<<endl;
	  float avg =  acc/gap_size_vec.size();
	  float std = 0;
	  for (auto it = gap_size_vec.begin(); it != gap_size_vec.end(); it++){
		  std += pow(*it - avg,2);
	  }
	  std = std/gap_size_vec.size();
	  cout<<"avg:"<< avg<< " std:" << std<<"size"<<gap_size_vec.size()<<endl;
  }

   cout<<"gap_ctr_per_row:";
   for (auto it = gap_ctr_per_row_vec.begin(); it != gap_ctr_per_row_vec.end(); it++){
		  cout<<*it<<",";
   }
  cout<<endl<<endl;
  */
  //ROS_INFO_STREAM("x_y_not_in_order_cntr"<< x_y_not_in_order_cntr<< "x_not_in_order_cntr"<<x_not_in_order_cntr<< "y_not_in_order_cntr"<<y_not_in_order_cntr<< "x_y_in_order_cntr"<<x_y_in_order_cntr);
}



// just for debugging purposes. to understand what the point cloud looks like
void run_diagnostics_for_shape(sensor_msgs::PointCloud2Iterator<float> cloud_x, sensor_msgs::PointCloud2Iterator<float> cloud_y, sensor_msgs::PointCloud2Iterator<float> cloud_z,
    int n_points, float resolution){
  // map of whether a point has been seen (rounded by resolution)
  //seen.clear();
  int hit_ctr = 0;
  int total_ctr = 0;
  double last_x, last_y, last_z, this_x, this_y, this_z, first_x, first_y, first_z;
  bool first_itr = true;
  int ordered_in_x_ctr = 0;
  int ordered_in_y_ctr = 0;
  int ordered_in_z_ctr = 0;
  bool last_x_equal = false;
  bool last_y_equal = false;
  bool last_z_equal = false;
  vector<int> ordered_in_x_ctr_vec;
  vector<int> ordered_in_y_ctr_vec;
  vector<int> ordered_in_z_ctr_vec;
  vector<float> x_length;
  vector<float> y_length;
  vector<float> z_length;


  double last_rounded_x, last_rounded_y, last_rounded_z;
  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
	  /*
	  double rounded_x = round_to_resolution(*cloud_x, resolution);
      double rounded_y = round_to_resolution(*cloud_y, resolution);
      double rounded_z = round_to_resolution(*cloud_z, resolution);
      double hashed_point = point_hash_xyz(rounded_x, rounded_y, rounded_z);
      */
      this_x = *cloud_x;
      this_y = *cloud_y;
      this_z = *cloud_z;


      if (fabs(this_x -last_x)<resolution) {
    	  ordered_in_x_ctr +=1;
    	  last_x_equal =true;
      }else if (fabs(this_x - last_x)>resolution && last_x_equal){
    	  ordered_in_x_ctr_vec.push_back(ordered_in_x_ctr);
    	  x_length.push_back(fabs(last_x - first_x));
    	  last_x_equal = false;
    	  ordered_in_x_ctr = 0;
    	  first_x = this_x;
      }else{
    	  last_x_equal = false;
    	  ordered_in_x_ctr = 0;
    	  first_x = this_x;
      }

     // int blah =0 ;
      //if (this_y < last_y){
    	//  blah +=1;
     // }
      if (fabs(this_y- last_y)<resolution) {
    	  ordered_in_y_ctr +=1;
    	  last_y_equal =true;
      }else if (fabs(this_y - last_y) > resolution && last_y_equal){
    	  ordered_in_y_ctr_vec.push_back(ordered_in_y_ctr);
    	  y_length.push_back(fabs(last_y - first_y));
    	  last_y_equal = false;
    	  ordered_in_y_ctr = 0;
    	  first_y = this_y;
      }else{
    	  last_y_equal = false;
    	  ordered_in_y_ctr = 0;
    	  first_y = this_y;
      }

      if (fabs(this_z - last_z) < resolution) {
    	  ordered_in_z_ctr +=1;
    	  last_z_equal =true;
      }else if (fabs(this_z - last_z) < resolution && last_z_equal){
    	  ordered_in_z_ctr_vec.push_back(ordered_in_z_ctr);


    	  last_z_equal = false;
    	  ordered_in_z_ctr = 0;
      }else{
    	  last_z_equal = false;
    	  ordered_in_z_ctr = 0;
      }

      last_x = this_x;
      last_y = this_y;
      last_z = this_z;
      //first_itr = false;
  }

  ordered_in_x_ctr_vec.push_back(ordered_in_x_ctr);
  ordered_in_y_ctr_vec.push_back(ordered_in_y_ctr);
  ordered_in_z_ctr_vec.push_back(ordered_in_z_ctr);


  if (ordered_in_x_ctr_vec.size() != 0){
	  float acc = 0;
	  cout<<"ordered_in_x_ctr_vec:";
	  for (auto it = ordered_in_x_ctr_vec.begin(); it != ordered_in_x_ctr_vec.end(); it++){
		  cout<<*it<<",";
		  acc+=*it;
	  }
	  cout<<endl;
	  float avg =  acc/ordered_in_x_ctr_vec.size();
	  float std = 0;
	  for (auto it = ordered_in_x_ctr_vec.begin(); it != ordered_in_x_ctr_vec.end(); it++){
		  std += pow(*it - avg,2);
	  }
	  std = std/ordered_in_x_ctr_vec.size();
	  cout<<"avg:"<< avg<< " std:" << std<<"size"<<ordered_in_x_ctr_vec.size()<<endl;

	  cout<<"x_length_vec:";
	  for (auto it = x_length.begin(); it != x_length.end(); it++){
		  cout<<*it<<",";
	  }
	  cout<<endl<<endl;
  }



  if (ordered_in_y_ctr_vec.size() != 0){
	  float acc = 0;
	  cout<<"ordered_in_y_ctr_vec:";
	  for (auto it = ordered_in_y_ctr_vec.begin(); it != ordered_in_y_ctr_vec.end(); it++){
		  cout<<*it<<",";
		  acc+=*it;
	  }
	  cout<<endl;
	  float avg =  acc/ordered_in_y_ctr_vec.size();
	  float std = 0;
	  for (auto it = ordered_in_y_ctr_vec.begin(); it != ordered_in_y_ctr_vec.end(); it++){
		  std += pow(*it - avg,2);
	  }

	  std = std/ordered_in_y_ctr_vec.size();
	  cout<<"avg:"<< avg<< " std:" << std<<"size"<<ordered_in_y_ctr_vec.size()<<endl;

	  cout<<"y_length_vec:";
	  for (auto it = y_length.begin(); it != y_length.end(); it++){
		  cout<<*it<<",";
	  }

	  cout<<endl<<endl;
  }

}


// -- just to produce xs, ys, zs (if need be)
void filterByVolumeNoFilter(sensor_msgs::PointCloud2Iterator<float> &cloud_x, sensor_msgs::PointCloud2Iterator<float> &cloud_y, sensor_msgs::PointCloud2Iterator<float> &cloud_z,
		std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, double volume_to_keep, int num_points)
{
	for (int i = 0; i < num_points; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
		xs.push_back(*cloud_x);
		ys.push_back(*cloud_y);
		zs.push_back(*cloud_z);
	}
}


// -- filter based on the desired volume to maintain
void filterByNumOfPoints(sensor_msgs::PointCloud2Iterator<float> &cloud_x, sensor_msgs::PointCloud2Iterator<float> &cloud_y, sensor_msgs::PointCloud2Iterator<float> &cloud_z,
		std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, int num_points, int points_budget)
{
	const int num_radius_buckets = point_cloud_max_z;
	int radius_counters [num_radius_buckets];
	memset(&radius_counters[0], 0, sizeof(radius_counters));
	double max_radius = sensor_max_range * std::pow(2, 0.5);
	double bucket_width = max_radius / num_radius_buckets;

	for (int i = 0; i < num_points; i++){
		// Add point to radius bucket
		int radius_bucket = (int) (std::pow(std::pow(*(cloud_x + i), 2) + std::pow(*(cloud_y+i), 2), 0.5) / bucket_width);
		radius_counters[radius_bucket] += 1;
	}

	// select the most central points to include
	int max_radius_bucket = 0;
	int points_to_include = radius_counters[0] + radius_counters[1];
	while (points_to_include <= points_budget && max_radius_bucket < num_radius_buckets) {
		max_radius_bucket++;
		points_to_include += radius_counters[max_radius_bucket + 1];
	}

	for (int i = 0; i < num_points; i++) {
		int radius_bucket = (int) (std::pow(std::pow(*(cloud_x+i), 2) + std::pow(*(cloud_y+i), 2), 0.5) / bucket_width);
		if (radius_bucket <= max_radius_bucket) {
			xs.push_back(*(cloud_x+i));
			ys.push_back(*(cloud_y+i));
			zs.push_back(*(cloud_z+i));
		}
	}
}

/*
// -- filter based on the desired volume to maintain
void filterByVolume(sensor_msgs::PointCloud2Iterator<float> &cloud_x, sensor_msgs::PointCloud2Iterator<float> &cloud_y, sensor_msgs::PointCloud2Iterator<float> &cloud_z,
		std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, int num_points, double volume_to_keep, double &volume_kept, float **gridded_volume, int grid_volume_row_size, float diagnostic_resolution)
{
	const int num_radius_buckets = 10*sensor_max_range;
	double radius_volume[num_radius_buckets];
	memset(&radius_volume[0], 0, sizeof(radius_volume));
	double max_radius = sensor_max_range* std::pow(2, 0.5);
	double bucket_width = max_radius / num_radius_buckets;
	double this_x, this_y, this_z, last_x, last_y, last_z;
	double last_row_y; // y associated with the first element of last row
	bool new_row = false;
	bool first_itr = true;
	double total_volume = 0;

    for (int i = 0; i < num_points; i++){
		// Add point to radius bucket
	  this_x = *(cloud_x + i);
	  this_y = *(cloud_y + i);
	  this_z = *(cloud_z + i);
	  // -- project x,y,z if they are outside of the sensor range
	  double norm = sqrt(this_z*this_z + this_y*this_y + this_x*this_x);
	  if (norm > sensor_max_range){
    //	  this_x = (sensor_max_range/norm)*this_x;
    //	  this_y = (sensor_max_range/norm)*this_y;
    	  this_z = (sensor_max_range/norm)*this_z;
      }
	  new_row = this_x < last_x;
	  if (!new_row && !first_itr ){
    	   int radius_bucket = (int) (std::pow(std::pow(this_x, 2) + std::pow(this_y, 2), 0.5) / bucket_width);
    	  if (fabs(last_z - this_z) < .5){
    		  //radius_volume[radius_bucket] += (1.0/3)*fabs(this_x-last_x)*fabs(this_y - last_row_y)*this_z;
    		  // using the gridded approach
    		  int x_index = ((int)1/diagnostic_resolution)*round_to_resolution(this_x, diagnostic_resolution) + (int)grid_volume_row_size/2;
    		  int y_index = ((int)1/diagnostic_resolution)*round_to_resolution(this_y, diagnostic_resolution) + (int)grid_volume_row_size/2;
    		  radius_volume[radius_bucket] += (1.0/3)*pow(diagnostic_resolution, 2)*gridded_volume[x_index][y_index];
    		  gridded_volume[x_index][y_index] = 0;
    	  }
	  }else{
		  last_row_y = this_y; // this is not applicable for gridded approach
	  }
      first_itr = false;
      last_x = this_x;
      last_y = this_y;
      last_z = this_z;
	}

	// select the most central points to include
	int max_radius_bucket = 0;
	double volume_included = 0;
	while (volume_included <= volume_to_keep && max_radius_bucket < num_radius_buckets) {
		max_radius_bucket++;
		volume_included += radius_volume[max_radius_bucket];


	}

	for (int i = 0; i < num_points; i++) {
		int radius_bucket = (int) (std::pow(std::pow(*(cloud_x+i), 2) + std::pow(*(cloud_y+i), 2), 0.5) / bucket_width);
		if (radius_bucket <= max_radius_bucket) {
			xs.push_back(*(cloud_x+i));
			ys.push_back(*(cloud_y+i));
			zs.push_back(*(cloud_z+i));
		}
	}
	volume_kept = volume_included;
}
*/

// -- filter based on the desired volume to maintain
void sequencer(sensor_msgs::PointCloud2Iterator<float> &cloud_x, sensor_msgs::PointCloud2Iterator<float> &cloud_y, sensor_msgs::PointCloud2Iterator<float> &cloud_z,
		std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, int num_points, double volume_to_keep, double &volume_kept, float **gridded_volume, int grid_volume_row_size, float diagnostic_resolution)
{
	const int num_radius_buckets = 10*sensor_max_range;
	double radius_volume[num_radius_buckets];
	memset(&radius_volume[0], 0, sizeof(radius_volume));
	vector<double> radius_points[num_radius_buckets];
	//	vector<double> radius_points_x[num_radius_buckets];
//	vector<double> radius_points_y[num_radius_buckets];
//	vector<double> radius_points_z[num_radius_buckets];
//	bool radius_points_valid[num_radius_buckets];
//	memset(&radius_points_valid[0], false, sizeof(radius_points_valid));

	double max_radius = sensor_max_range* std::pow(2, 0.5);
	double bucket_width = max_radius / num_radius_buckets;
	double this_x, this_y, this_z, last_x, last_y, last_z;
	double last_row_y; // y associated with the first element of last row
	bool new_row = false;
	bool first_itr = true;
	double total_volume = 0;

    for (int i = 0; i < num_points; i++){
    	this_x = *(cloud_x + i);
    	this_y = *(cloud_y + i);
    	this_z = *(cloud_z + i);
    	// -- project x,y,z if they are outside of the sensor range
    	double norm = sqrt(this_z*this_z + this_y*this_y + this_x*this_x);
    	if (norm > sensor_max_range){
    		//	  this_x = (sensor_max_range/norm)*this_x;
    		//	  this_y = (sensor_max_range/norm)*this_y;
    		this_z = (sensor_max_range/norm)*this_z;
    	}
    	int radius_bucket = (int) (std::pow(std::pow(this_x, 2) + std::pow(this_y, 2), 0.5) / bucket_width);
    	//radius_volume[radius_bucket] += (1.0/3)*fabs(this_x-last_x)*fabs(this_y - last_row_y)*this_z;
    	// using the gridded approach
    	int x_index = ((int)1/diagnostic_resolution)*round_to_resolution(this_x, diagnostic_resolution) + (int)grid_volume_row_size/2;
    	int y_index = ((int)1/diagnostic_resolution)*round_to_resolution(this_y, diagnostic_resolution) + (int)grid_volume_row_size/2;
    	radius_volume[radius_bucket] += (1.0/3)*pow(diagnostic_resolution, 2)*gridded_volume[x_index][y_index];
    	radius_points[radius_bucket].push_back(i);
    	//radius_points_valid[radius_bucket] = true;
    	gridded_volume[x_index][y_index] = 0;
    }

    for (int bucket_number= 0; bucket_number <num_radius_buckets; bucket_number++){
    	for (int pt_idx= 0; pt_idx< radius_points[bucket_number].size(); pt_idx++){
    		int idx = (radius_points[bucket_number])[pt_idx];
    		xs.push_back(*(cloud_x+ idx));
			ys.push_back(*(cloud_y+idx));
			zs.push_back(*(cloud_z+idx));
    	}
    }

    /*
    // select the most central points to include
	int max_radius_bucket = 0;
	double volume_included = 0;
	while (volume_included <= volume_to_keep && max_radius_bucket < num_radius_buckets) {
		max_radius_bucket++;
		volume_included += radius_volume[max_radius_bucket];


	}

	for (int i = 0; i < num_points; i++) {
		int radius_bucket = (int) (std::pow(std::pow(*(cloud_x+i), 2) + std::pow(*(cloud_y+i), 2), 0.5) / bucket_width);
		if (radius_bucket <= max_radius_bucket) {
			xs.push_back(*(cloud_x+i));
			ys.push_back(*(cloud_y+i));
			zs.push_back(*(cloud_z+i));
		}
	}
	volume_kept = volume_included;
	*/
}



void filterByNumOfPointsHeightWise(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs,
    std::vector<float> &xs_best,  std::vector<float> &ys_best, std::vector<float> &zs_best, int points_budget, double point_cloud_max_z) {

  // bucket points by distance from center for choosing best ones
  const int num_height_buckets = point_cloud_max_z;
  int height_counters [num_height_buckets];
  memset(&height_counters[0], 0, sizeof(height_counters));
  double max_height = 30;
  double bucket_width = max_height / num_height_buckets;

  for (int i = 0; i < xs.size(); i++) {
    int height_bucket_idx = (int) (zs[i]/ bucket_width);
    height_counters[height_bucket_idx] += 1;
  }

  // select the pionts closest to the drone (height wise)
  int max_height_bucket_idx = 0;
  int points_to_include = height_counters[0] + height_counters[1];
  while (points_to_include <= points_budget && max_height_bucket_idx < num_height_buckets) {
    max_height_bucket_idx++;
    points_to_include += height_counters[max_height_bucket_idx + 1];
  }

  for (int i = 0; i < xs.size(); i++) {
      int height_bucket_idx = (int) (zs[i]/ bucket_width);
      if (height_bucket_idx <= max_height_bucket_idx) {
        xs_best.push_back(xs[i]);
        ys_best.push_back(ys[i]);
        zs_best.push_back(zs[i]);
      }
    }
}


// -- just to produce xs_best, ys_best, zs_best (if need be)
void filterByResolutionNoFilter(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs,
    std::vector<float> &xs_best,  std::vector<float> &ys_best, std::vector<float> &zs_best, int points_budget, double point_cloud_max_z, double resolution) {
  // map of whether a point has been seen (rounded by resolution)
  double last_hashed_point = 0;
  //double last_x, last_y, last_z, this_x, this_y, this_z;
  double volume_to_digest_partial = 0; // accumulats the depth values, which is then multipled by a square of resolution side size
  int num_of_points = 0; // -- only for profiling purposes
  double last_rounded_x, last_rounded_y, last_rounded_z;
  for(size_t i=0; i< xs.size() - 1 ; ++i){
	  xs_best.push_back(xs[i]);
	  ys_best.push_back(ys[i]);
	  zs_best.push_back(zs[i]);
  }
}


// -- filter based on resolution using hashing
void filterByResolutionByHashing(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs,
    std::vector<float> &xs_best,  std::vector<float> &ys_best, std::vector<float> &zs_best, int points_budget, double point_cloud_max_z, double resolution) {

	// map of whether a point has been seen (rounded by resolution)
  std::unordered_set<double> seen;
  //seen.clear();
  double last_hashed_point = 0;
  //double last_x, last_y, last_z, this_x, this_y, this_z;
  double volume_to_digest_partial = 0; // accumulats the depth values, which is then multipled by a square of resolution side size
  int num_of_points = 0; // -- only for profiling purposes
  double last_rounded_x, last_rounded_y, last_rounded_z;
  for(size_t i=0; i< xs.size() - 1 ; ++i){
	  double rounded_x = round_to_resolution(xs[i], resolution);
      double rounded_y = round_to_resolution(ys[i], resolution);
      double rounded_z = round_to_resolution(zs[i], resolution);
      double hashed_point = point_hash_xyz(rounded_x, rounded_y, rounded_z);
      /*
      this_x = *cloud_x;
      this_y = *cloud_y;
      this_z = *cloud_z;
	  */

      if (hashed_point != last_hashed_point){
		  if (seen.find(hashed_point) == seen.end()) {
			  seen.insert(hashed_point);
			  xs_best.push_back(rounded_x);
			  ys_best.push_back(rounded_y);
			  zs_best.push_back(rounded_z);
			  num_of_points +=1;
		  }
		  last_hashed_point = hashed_point;
      }
  }
  //ROS_INFO_STREAM("x_y_not_in_order_cntr"<< x_y_not_in_order_cntr<< "x_not_in_order_cntr"<<x_not_in_order_cntr<< "y_not_in_order_cntr"<<y_not_in_order_cntr<< "x_y_in_order_cntr"<<x_y_in_order_cntr);
  //  ROS_INFO_STREAM("num of points "<< num_of_points);
  // volume_to_digest = (1.0/3)*resolution*resolution*volume_to_digest_partial; // -- compilation of  cone volumes
}


/*
void filterByResolutionByHashing(sensor_msgs::PointCloud2Iterator<float> &cloud_x, sensor_msgs::PointCloud2Iterator<float> &cloud_y, sensor_msgs::PointCloud2Iterator<float> &cloud_z,
    std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, int n_points, float resolution, double &volume_to_digest){
  // map of whether a point has been seen (rounded by resolution)
  std::unordered_set<double> seen;
  //seen.clear();
  double last_hashed_point = 0;
  double last_x, last_y, last_z, this_x, this_y, this_z;
  double volume_to_digest_partial = 0; // accumulats the depth values, which is then multipled by a square of resolution side size
  int num_of_points = 0; // -- only for profiling purposes
  double last_rounded_x, last_rounded_y, last_rounded_z;
  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
      double rounded_x = round_to_resolution(*cloud_x, resolution);
      double rounded_y = round_to_resolution(*cloud_y, resolution);
      double rounded_z = round_to_resolution(*cloud_z, resolution);
      double hashed_point = point_hash_xyz(rounded_x, rounded_y, rounded_z);
      this_x = *cloud_x;
      this_y = *cloud_y;
      this_z = *cloud_z;

      if (hashed_point != last_hashed_point){
		  if (seen.find(hashed_point) == seen.end()) {
			  seen.insert(hashed_point);
			  xs.push_back(*cloud_x);
			  ys.push_back(*cloud_y);
			  zs.push_back(*cloud_z);
			  num_of_points +=1;
		  }
		  last_hashed_point = hashed_point;
      }
  }
  //ROS_INFO_STREAM("x_y_not_in_order_cntr"<< x_y_not_in_order_cntr<< "x_not_in_order_cntr"<<x_not_in_order_cntr<< "y_not_in_order_cntr"<<y_not_in_order_cntr<< "x_y_in_order_cntr"<<x_y_in_order_cntr);
//  ROS_INFO_STREAM("num of points "<< num_of_points);
//  volume_to_digest = (1.0/3)*resolution*resolution*volume_to_digest_partial; // -- compilation of  cone volumes
}
*/


/*
struct Hash {
	  size_t operator()(const double *my_tuple) const{
		  size_t index;
		  sscanf((to_string(my_tuple[0]) + to_string(my_tuple[1]) + to_string(my_tuple[2])).c_str(), "%zu", &index);
		  return index;
	  }
};


//didn't perform any better
void filterByResolutionCustomHash(sensor_msgs::PointCloud2Iterator<float> &cloud_x, sensor_msgs::PointCloud2Iterator<float> &cloud_y, sensor_msgs::PointCloud2Iterator<float> &cloud_z,
    std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, int n_points, float resolution){
  // map of whether a point has been seen (rounded by resolution)


	std::unordered_set<double*, Hash> seen;
  //seen.clear();

  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
      //double distance_from_center = abs(*cloud_x); //pow(pow(*cloud_x, 2) + pow(*cloud_y, 2), 0.5);
      double rounded_x = round_to_resolution(*cloud_x, resolution);
      double rounded_y = round_to_resolution(*cloud_y, resolution);
      double rounded_z = round_to_resolution(*cloud_z, resolution);
      //double hashed_point = point_hash_xyz(rounded_x, rounded_y, rounded_z);
      double hashed_point[3] = {rounded_x, rounded_y, rounded_z};
      if (seen.find(hashed_point) == seen.end()) {
    	seen.insert(hashed_point);
        xs.push_back(*cloud_x);
        ys.push_back(*cloud_y);
        zs.push_back(*cloud_z);
      }

  }
}
*/

/*
// populate xs,ys,zs, with the same points, so no filtering. Just used for comparison purposes with filtering
void filterByResolutionNoFilter(sensor_msgs::PointCloud2Iterator<float> &cloud_x, sensor_msgs::PointCloud2Iterator<float> &cloud_y,
		sensor_msgs::PointCloud2Iterator<float> &cloud_z, std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, int n_points, float resolution){
  // map of whether a point has been seen (rounded by resolution)
  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
	xs.push_back(*cloud_x);
	ys.push_back(*cloud_y);
	zs.push_back(*cloud_z);

  }
}
*/

// gets an estimate of how many points are interesting (for runtime to make budget decision)
int getEntropyDiagnostic(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs) {
  //int sensor_max_range = 25;
  int entropy_diagnostic = 0;
  // TODO: improve
  for (int i = 0; i < xs.size(); i++) {
    if (zs[i] < sensor_max_range) {
      entropy_diagnostic++;
    }
  }
  return entropy_diagnostic;
}

// estimates the number of open regions in cloud that fall within specified width
int xGapDiagnostic(std::vector<float> &xs, std::vector<float> &ys, std::vector<float> &zs, 
  double min_gap_size, double max_gap_size, double y_bucket_size, double y_min, double y_max, double max_sensor_range) {
  // bucket points by y value, store x and whether we see obstacle
  std::vector<map<double, bool>> y_buckets;
  const int num_y_buckets = (y_max - y_min) / y_bucket_size + 1;
  y_buckets.resize(num_y_buckets);
  for (int i = 0; i < xs.size(); i ++) {
    int y_bucket = (int)((ys[i] - y_min) / y_bucket_size);
    // printf("bucket number %d\n", y_bucket);
    y_buckets[y_bucket].insert(pair<double,bool>(xs[i], zs[i] < max_sensor_range));

  }

  // count streaks of x values where there are no obstacles
  int num_gaps = 0;
  map<double, bool>::iterator itr;
  for (int bucket_num = 0; bucket_num < num_y_buckets; bucket_num++){
    map<double, bool> bucket = y_buckets[bucket_num];
    bool first_obstacle_seen = false;
    bool is_gap_active = false;
    double current_gap_start = 0;
    for (itr = bucket.begin(); itr != bucket.end(); ++itr) {
      first_obstacle_seen |= itr->second;
      // printf("%d\n", itr->second);
      if (is_gap_active && (itr->second)) {
        // end of gap
        // printf("gap start, end: %f, %f \n", current_gap_start, itr->first);
        double gap_length = itr->first - current_gap_start;
        is_gap_active = false;
        if (gap_length > min_gap_size && gap_length < max_gap_size) {
          // printf("gap length : %f\n", gap_length);
          num_gaps++;
        }
        // printf("gap length: %f\n", gap_length);
      } else if (!is_gap_active && !itr->second && first_obstacle_seen) {
        // printf("gap start: %f\n", itr->first);
        is_gap_active = true;
        current_gap_start = itr->first;
      }
    }
  }
  
  return num_gaps;
}


void filterByNumOfPointsRadiusWise(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs,
    std::vector<float> &xs_best,  std::vector<float> &ys_best, std::vector<float> &zs_best, int points_budget, double point_cloud_max_z) {

  // bucket points by distance from center for choosing best ones
  const int num_radius_buckets = point_cloud_max_z;
  int radius_counters [num_radius_buckets];
  memset(&radius_counters[0], 0, sizeof(radius_counters));
  double max_radius = sensor_max_range * std::pow(2, 0.5);
  double bucket_width = max_radius / num_radius_buckets;

  for (int i = 0; i < xs.size(); i++) {
    // Add point to radius bucket
    int radius_bucket = (int) (std::pow(std::pow(xs[i], 2) + std::pow(ys[i], 2), 0.5) / bucket_width);
    radius_counters[radius_bucket] += 1;
  }

  // select the most central points to include
  int max_radius_bucket = 0;
  int points_to_include = radius_counters[0] + radius_counters[1];
  while (points_to_include <= points_budget && max_radius_bucket < num_radius_buckets) {
    max_radius_bucket++;
    points_to_include += radius_counters[max_radius_bucket + 1];
  }

  for (int i = 0; i < xs.size(); i++) {
      int radius_bucket = (int) (std::pow(std::pow(xs[i], 2) + std::pow(ys[i], 2), 0.5) / bucket_width);
      if (radius_bucket <= max_radius_bucket) {
        xs_best.push_back(xs[i]);
        ys_best.push_back(ys[i]);
        zs_best.push_back(zs[i]);
      }
    }
}


/*
// gets the points that are most central
void filterByNumOfPoints(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs,
    std::vector<float> &xs_best,  std::vector<float> &ys_best, std::vector<float> &zs_best, int points_budget, double point_cloud_max_z, string mode="radius") {
	if (mode == "radius"){
		filterByNumOfPointsRadiusWise(xs, ys, zs, xs_best, ys_best, zs_best, points_budget, point_cloud_max_z);
	} else if (mode == "height"){
		filterByNumOfPointsHeightWise(xs, ys, zs, xs_best, ys_best, zs_best, points_budget, point_cloud_max_z);
	}else{
		ROS_ERROR_STREAM("this mode"<< mode<<" for point cloud num of points filtering is not defined");
		exit(0);
	}
}
*/

/*
// no filter for num of points
void filterNoFilter(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs,
    std::vector<float> &xs_best,  std::vector<float> &ys_best, std::vector<float> &zs_best) {
	for (int i = 0; i < xs.size(); i++) {
		double this_x  = xs[i];
		double this_y = ys[i];
		double this_z  = zs[i];
		double norm = sqrt(this_z*this_z + this_y*this_y + this_x*this_x);
		if (norm > sensor_max_range){
		  this_x = (sensor_max_range/norm)*this_x;
    	  this_y = (sensor_max_range/norm)*this_y;
    	  this_z = (sensor_max_range/norm)*this_z;
		}
		xs_best.push_back(this_x);
		ys_best.push_back(this_y);
		zs_best.push_back(this_z);
	}
}
*/


// no filter for num of points
void filterNoFilter(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs,
    std::vector<float> &xs_best,  std::vector<float> &ys_best, std::vector<float> &zs_best) {
	for (int i = 0; i < xs.size(); i++) {
		xs_best.push_back(xs[i]);
		ys_best.push_back(ys[i]);
		zs_best.push_back(zs[i]);
	}
}


void filterByWidthHeight(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs,
    std::vector<float> &xs_best,  std::vector<float> &ys_best, std::vector<float> &zs_best, int width, int height) {
  for (int i = 0; i < xs.size(); i++) {
    if (xs[i] > -1*width/2 && xs[i] < width/2 && 
        ys[i] > -1*height/2 && ys[i] < height/2) {
        xs_best.push_back(xs[i]);
        ys_best.push_back(ys[i]);
        zs_best.push_back(zs[i]);
    }
  }
}

void calc_avg_worse_point_distance(std::vector<float> &xs,  std::vector<float> &ys, std::vector<float> &zs, double &avg_distance, double &max_min){

  //calculate the avg and worse case distance between points
  for (int i = 0; i<xs.size() ; i++){
	  double min_distance;
	  bool min_distance_collected = false;
	  for (int j = 0; j<xs.size(); j++){
		  if (i == j){
			  continue;
		  }
		  double dx = xs[i] - xs[j];
		  double dy = ys[i] - ys[j];
		  double dz = zs[i] - zs[j];
		  if (min_distance_collected){
			  min_distance = std::min(min_distance, std::sqrt(dx*dx + dy*dy + dz*dz));
		  }else{
			  min_distance =  std::sqrt(dx*dx + dy*dy + dz*dz);
			  min_distance_collected = true;
		  }
	  }
	  if (min_distance_collected){
		  avg_distance += min_distance;
		  max_min = std::max(min_distance, max_min);
	  }
  }
}


/*
//added by Kindell for gap diagnostics
    //{
	  // get diagnostics for runtime
	  // (this function is good for environments with vertical obstacles, if concerned about horizontal
	  // ones then y gap diagnostic may be useful)
	  double max_sensor_range = sensor_max_range; // TODO: use params?
	  double min_gap_size = .5; // use drone height here?
	  double max_gap_size = 50;
	  double y_bucket_size = 3;
	  // could convert this to give an array of values for different max sizes, if multiple calls too expensive.
	  // (for now just seems more confusing to do that than it's worth)
	  double y_min = -50;
	  double y_max = 50;
	  int xGapCount = xGapDiagnostic(xs, ys, zs, min_gap_size, max_gap_size, y_bucket_size, y_min, y_max, max_sensor_range);
	  //printf("x gap diagnostic: %d\n", xGapCount);
  //}

  */
  // double point_cloud_resolution_in_cubic = std::sqrt(3*point_cloud_resolution*point_cloud_resolution);
  // double avg_distance = 0;
  // double max_min = 0; // max of all the mins

  // drop fixed proportion of points - we should delete this, seems it is always bad idea
  /*
  int cntr = 0;
  for(size_t i=0; i< n_points - 1 ; ++i, ++cloud_x, ++cloud_y, ++cloud_z){
	  // filter based on FOV
	  if (*cloud_x > -1*point_cloud_width && *cloud_x < point_cloud_width &&
        *cloud_y > -1*point_cloud_height && *cloud_y < point_cloud_height && ((cntr % point_cloud_density_reduction) == 0)) {
		  xs.push_back(*cloud_x);
		  ys.push_back(*cloud_y);
		  zs.push_back(*cloud_z);
      // add to frontier if it is the closest z for given xy
	  }
	  cntr +=1;
  }
  */

double PointCloudXyzNodelet::estimated_to_actual_vol_correction(double estimated_vol){
	if (knob_performance_modeling && knob_performance_modeling_for_pc_om){ // dont need to do anything because we are trying find the function for correction
		return estimated_vol;
	}else{
		return estimated_vol;
	}
}


double PointCloudXyzNodelet::actual_to_estimated_vol_correction(double actual_vol){
	if (knob_performance_modeling && knob_performance_modeling_for_pc_om){ // dont need to do anything because we are trying find the function for correction
		return actual_vol;
	}else{
		return actual_vol;
	}
}


void PointCloudXyzNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
{
   profiling_container->capture("entire_point_cloud_depth_callback", "start", ros::Time::now());
   ros::param::get("/sensor_max_range", sensor_max_range);
   //ros::param::get("/point_cloud_width", point_cloud_width);
   //ros::param::get("/point_cloud_height", point_cloud_height);
   //ros::param::get("/point_cloud_num_points", point_cloud_num_points);
   //ros::param::get("/point_cloud_num_points_filtering_mode", point_cloud_num_points_filtering_mode);
   //ros::param::get("/point_cloud_density_reduction", point_cloud_density_reduction);
   //ros::param::get("/point_cloud_max_z", point_cloud_max_z);



  PointCloud::Ptr cloud_msg(new PointCloud);

  ros::Time start_hook_t = ros::Time::now();
  
  img_to_pt_cloud_acc += (start_hook_t - depth_msg->header.stamp).toSec()*1e9;

  //img_capture_time_stamp = depth_msg->header.stamp;
  img_capture_time_stamp = ros::Time::now();

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
  double sensor_volume_to_digest_estimated = 0;
  double area_to_digest = 0;

  if (CLCT_DATA_){
   /*
	  pt_cld_ctr++;
    if ((pt_cld_ctr+1) % data_collection_iteration_freq_ == 0) { // this is because I can't figure out how
    															 // insert the shutting_down function in the sigInt
        log_data_before_shutting_down();
    }
    */
  }
 
  // -- convert the msg to vector
  int n_points = cloud_msg->width * cloud_msg->height;
  sensor_msgs::PointCloud2Iterator<float> cloud_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> cloud_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> cloud_z(*cloud_msg, "z");
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> zs;
  double volume_to_digest_2; // -- at the moment, we are not measuing this


  // -- run diagnostics
  profiling_container->capture("diagnostics", "start", ros::Time::now());
  //run_diagnostics_for_shape(cloud_x, cloud_y, cloud_z, n_points, point_cloud_resolution);
  //runDiagnostics(cloud_x, cloud_y, cloud_z, n_points, point_cloud_resolution, volume_to_digest, area_to_digest);
  // -- using the gridded approach
  const int grid_size = 60;
  float **gridded_volume;
  float diagnostic_resolution = 1; // -- point cloud resolution to use for diagnostic purposes
  double gap_statistics_avg, gap_statistics_min, gap_statistics_max, obs_dist_statistics_min_from_pc, obs_dist_statistics_min_from_om, obs_dist_statistics_avg_from_pc;

  gridded_volume = runDiagnosticsUsingGriddedApproach(cloud_x, cloud_y, cloud_z, n_points, grid_size, diagnostic_resolution, sensor_volume_to_digest_estimated, area_to_digest, gap_statistics_min,
		  gap_statistics_max, gap_statistics_avg, obs_dist_statistics_min_from_pc, obs_dist_statistics_avg_from_pc);


  mavbench_msgs::control control;
  double sensor_volume_to_digest =  estimated_to_actual_vol_correction(sensor_volume_to_digest_estimated); // convert to actual, because the runtime makes decision with actual values
  control.inputs.sensor_volume_to_digest = sensor_volume_to_digest;
  double cur_tree_total_volume;
  ros::param::get("cur_tree_total_volume", cur_tree_total_volume);
  ros::param::get("obs_dist_statistics_min_from_om", obs_dist_statistics_min_from_om);
  control.inputs.cur_tree_total_volume = float(cur_tree_total_volume);
  control.inputs.gap_statistics_min = gap_statistics_min;
  control.inputs.gap_statistics_max = gap_statistics_max;
  control.inputs.gap_statistics_avg = gap_statistics_avg;
  control.inputs.obs_dist_statistics_min = min(obs_dist_statistics_min_from_pc, obs_dist_statistics_min_from_om);
  control.inputs.obs_dist_statistics_avg = obs_dist_statistics_avg_from_pc;  // note that we can't really get avg distance from octomap, since there
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	     // so many obstacles in the map

  control_pub.publish(control);
  //ROS_INFO_STREAM("publishing control now");
  ros::param::get("/new_control_data", new_control_data);

  while(!new_control_data){
	  ros::param::get("/new_control_data", new_control_data);
	  ros::Duration(.01).sleep();
	  control_pub.publish(control);
  }
  new_control_data = false;
  ros::param::set("new_control_data", new_control_data);

  bool optimizer_succeeded;
  bool log_control_data; // used to determine whether the data coming from optimizer or manual is valid. This is
  	  	  	  	  	  	 // mainly used to avoid loging bogus data which occurs when the optimizer fails and
  	  	  	  	  	     // hence we don't have reasonable actual cmds to follow (note that a policty such as using the
  	  	  	  	  	     // the last actual cmds might result in bogus data since we don't know if we have for example
  	  	  	  	  	     // enough volume to consume in this round (comparing to last round)
  ros::param::get("/optimizer_succeeded", optimizer_succeeded);
  ros::param::get("/log_control_data", log_control_data);

  /*
  if (!optimizer_succeeded){
	  ROS_INFO_STREAM("----------------------------- OPTIMIZER FAILD<<<<<<<<<<<<<<<<<<<<<");
	  return;
  }
	*/
  //sensor_volume_to_digest_estimated -= volume_correction_coeff;

  profiling_container->capture("diagnostics", "end", ros::Time::now());
  // -- time budget
   ros::param::get("/sensor_to_actuation_time_budget_to_enforce", sensor_to_actuation_time_budget_to_enforce);
   ros::param::get("/om_latency_expected", om_latency_expected);
   ros::param::get("/om_to_pl_latency_expected", om_to_pl_latency_expected);
   ros::param::get("/ppl_latency_expected", ppl_latency_expected);
   ros::param::get("/velocity_to_budget_on", velocity_to_budget_on);
   ros::param::get("/ee_latency_expected", ee_latency_expected);

   // -- point cloud to octomap knobs
   ros::param::get("/pc_res", pc_res);
   ros::param::get("/pc_vol_ideal", pc_vol_ideal);

   // -- octomap to planner
   ros::param::get("/om_to_pl_res", om_to_pl_res);
   ros::param::get("/om_to_pl_vol_ideal", om_to_pl_vol_ideal);


   // -- piecewise planner
   ros::param::get("/ppl_vol_ideal", ppl_vol_ideal);


  // -- start filtering
  profiling_container->capture("filtering", "start", ros::Time::now(), capture_size);
  double pc_vol_estimated = 0;
  double pc_vol_ideal_estimated = actual_to_estimated_vol_correction(pc_vol_ideal); // convert the ideal to estimated because point cloud can only enforce in the estimated world
  //filterByVolume(cloud_x, cloud_y, cloud_z, xs, ys, zs,  n_points, pc_vol_ideal_estimated, pc_vol_estimated, gridded_volume, grid_size, diagnostic_resolution);
  sequencer(cloud_x, cloud_y, cloud_z, xs, ys, zs,  n_points, pc_vol_ideal_estimated, pc_vol_estimated, gridded_volume, grid_size, diagnostic_resolution);

  // -- destroy the grid
  for (int h = 0; h < grid_size; h++)
  {
	  delete [] gridded_volume[h];
  }
  delete gridded_volume;


  //filterByVolumeNoFilter(cloud_x, cloud_y, cloud_z, xs, ys, zs, pc_vol_ideal, n_points);
  //filterByNumOfPoints(cloud_x, cloud_y, cloud_z, xs, ys, zs, n_points, point_cloud_num_points);

  // Pick best points
  std::vector<float> xs_best;
  std::vector<float> ys_best;
  std::vector<float> zs_best;
  //filterByResolutionByHashing(cloud_x, cloud_y, cloud_z, xs, ys, zs, n_points, point_cloud_resolution, volume_to_digest_2);
  filterByResolutionByHashing(xs, ys, zs, xs_best, ys_best, zs_best, point_cloud_num_points, point_cloud_max_z, pc_res);

  //filterByResolutionNoFilter(xs, ys, zs, xs_best, ys_best, zs_best, point_cloud_num_points, point_cloud_max_z, point_cloud_resolution);
  // filterByResolutionNoFilter(cloud_x, cloud_y, cloud_z, xs, ys, zs, n_points, point_cloud_resolution); //for microbehcmark_3 to collect data without resoloution filtering
  //filterByResolutionCustomHash(cloud_x, cloud_y, cloud_z, xs, ys, zs, n_points, point_cloud_resolution);
  //filterByResolutionAndEdges(cloud_x, cloud_y, cloud_z, xs, ys, zs, n_points, point_cloud_resolution);
  //ROS_INFO_STREAM("vol Pc 2:"<<volume_to_digest);


  //filterNoFilter(xs, ys, zs, xs_best, ys_best, zs_best);
  //filterByWidthHeight(xs, ys, zs, xs_best, ys_best, zs_best, point_cloud_width, point_cloud_height);
  //filterByNumOfPoints(xs, ys, zs, xs_best, ys_best, zs_best, point_cloud_num_points, point_cloud_max_z, point_cloud_num_points_filtering_mode);

  // reset point cloud and load in filtered in points
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  modifier.resize(xs_best.size());
  sensor_msgs::PointCloud2Iterator<float> new_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> new_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> new_z(*cloud_msg, "z");

  for(size_t i=0; i<xs_best.size(); ++i, ++new_x, ++new_y, ++new_z){
      *new_x = xs_best[i];
      *new_y = ys_best[i];
      *new_z = zs_best[i];
  }

  profiling_container->capture("filtering", "end", ros::Time::now());

  mavbench_msgs::point_cloud_aug pcl_aug_data;
  pcl_aug_data.header = cloud_msg->header;
  pcl_aug_data.pcl = *cloud_msg;

  pcl_aug_data.controls.cmds.optimizer_succeeded = optimizer_succeeded;
  pcl_aug_data.controls.cmds.log_control_data = log_control_data;
  pcl_aug_data.controls.cmds.pc_res = pc_res;
  pcl_aug_data.controls.cmds.pc_vol = pc_vol_ideal;
  pcl_aug_data.controls.cmds.om_to_pl_vol = om_to_pl_vol_ideal;
  pcl_aug_data.controls.cmds.om_to_pl_res = om_to_pl_res;
  pcl_aug_data.controls.cmds.ppl_vol = ppl_vol_ideal;
  pcl_aug_data.ee_profiles.expected_time.ppl_latency = ppl_latency_expected;
  pcl_aug_data.ee_profiles.expected_time.om_latency = om_latency_expected;
  pcl_aug_data.ee_profiles.expected_time.om_to_pl_latency = om_to_pl_latency_expected;
  double smoothening_latency_expected = ppl_latency_expected;
  pcl_aug_data.ee_profiles.expected_time.smoothening_latency = smoothening_latency_expected;
  pcl_aug_data.ee_profiles.expected_time.ee_latency = ee_latency_expected;

  // -- for profiling purposes
  pcl_aug_data.ee_profiles.space_stats.pc_vol_estimated = pc_vol_estimated; // this is the estimation not the actual. Note that the actuall value can not be determined
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	   // untill PC is intergrated in octomap
  pcl_aug_data.ee_profiles.expected_cmds = pcl_aug_data.controls.cmds;
  //pcl_aug_data.controls.inputs.sensor_volume_to_digest = sensor_volume_to_digest;
  //pcl_aug_data.controls.inputs.cur_tree_total_volume = cur_tree_total_volume;
  pcl_aug_data.controls.inputs =  control.inputs;
  pcl_aug_data.controls.inputs.velocity_to_budget_on = velocity_to_budget_on;

  pcl_aug_data.controls.internal_states.sensor_to_actuation_time_budget_to_enforce = sensor_to_actuation_time_budget_to_enforce;
  profiling_container->capture("entire_point_cloud_depth_callback", "end", ros::Time::now());
  pcl_aug_data.ee_profiles.actual_time.pc_latency =  profiling_container->findDataByName("entire_point_cloud_depth_callback")->values.back();
  pcl_aug_data.ee_profiles.actual_time.pc_pre_pub_time_stamp = ros::Time::now();
  pcl_aug_data.ee_profiles.actual_time.img_capture_time_stamp = img_capture_time_stamp;

  if (DEBUG_VIS){
	  pub_point_cloud_.publish (*cloud_msg);
  }
  pub_point_cloud_aug_.publish (pcl_aug_data);

  //mavbench_msgs::point_cloud_meta_data meta_data_msg;
  //meta_data_msg.header.stamp = ros::Time::now();
  //meta_data_msg.point_cloud_volume_to_digest = pc_vol_actual;
  //meta_data_msg.point_cloud_resolution = point_cloud_resolution;
  //meta_data_msg.point_cloud_area_to_digest = area_to_digest;
  //point_cloud_meta_data_pub.publish(meta_data_msg);

  profiling_container->capture("point_cloud_area_to_digest", "single", area_to_digest, capture_size);
  profiling_container->capture("sensor_volume_to_digest", "single", sensor_volume_to_digest, capture_size);
  profiling_container->capture("point_cloud_resolution", "single", pc_res, capture_size);


  if (DEBUG_RQT){
	  debug_data.header.stamp = ros::Time::now();
	  debug_data.point_cloud_width = point_cloud_width;
	  debug_data.point_cloud_height = point_cloud_height;
	  debug_data.point_cloud_resolution = pc_res;
	  debug_data.point_cloud_point_cnt = xs_best.size();
	  debug_data.point_cloud_filtering_time = profiling_container->findDataByName("filtering")->values.back();
	  debug_data.entire_point_cloud_depth_callback= profiling_container->findDataByName("entire_point_cloud_depth_callback")->values.back();
	  debug_data.sensor_volume_to_digest =  profiling_container->findDataByName("sensor_volume_to_digest")->values.back();
	  debug_data.pc_vol_estimated = pc_vol_estimated;
	  debug_data.point_cloud_area_to_digest = area_to_digest;
	  debug_data.diagnostics = profiling_container->findDataByName("diagnostics")->values.back();;
	  debug_data.sensor_to_actuation_time_budget_to_enforce = sensor_to_actuation_time_budget_to_enforce;
	  debug_data.velocity_to_budget_on = velocity_to_budget_on;

	  pc_debug_pub.publish(debug_data);


  }

}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::PointCloudXyzNodelet,nodelet::Nodelet);
