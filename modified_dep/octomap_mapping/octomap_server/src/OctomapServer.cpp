/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <octomap_server/OctomapServer.h>
#include "octomap_server/maxRangeSrv.h"
//#include <signal.h>
#include "profile_manager/start_profiling_srv.h"
#include "profile_manager/profiling_data_srv.h"

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}
#include <iostream>
namespace octomap_server{

OctomapServer::OctomapServer(ros::NodeHandle private_nh_)
: m_nh(),
  m_pointCloudSub(NULL),
  m_tfPointCloudSub(NULL),
  m_reconfigureServer(m_config_mutex, private_nh_),
  m_octree(NULL),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  m_useHeightMap(true),
  m_useColoredMap(false),
  m_colorFactor(0.8),
  m_latchedTopics(true),
  m_publishFreeSpace(false),
  m_res(0.05),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_pointcloudMinX(-std::numeric_limits<double>::max()),
  m_pointcloudMaxX(std::numeric_limits<double>::max()),
  m_pointcloudMinY(-std::numeric_limits<double>::max()),
  m_pointcloudMaxY(std::numeric_limits<double>::max()),
  m_pointcloudMinZ(-std::numeric_limits<double>::max()),
  m_pointcloudMaxZ(std::numeric_limits<double>::max()),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max()),
  m_minSizeX(0.0), m_minSizeY(0.0),
  m_filterSpeckles(false), m_filterGroundPlane(false),
  m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
  m_compressMap(true),
  m_incrementalUpdate(false),
  CLCT_DATA(false),
  data_collection_iteration_freq(100),
  m_initConfig(true),
  my_profile_manager("client", "/record_profiling_data", "/record_profiling_data_verbose")
{

  double probHit, probMiss, thresMin, thresMax;
  ros::NodeHandle private_nh = private_nh_;
//  ros::NodeHandle private_nh("~");
  //m_nh.setCallbackQueue(&callback_queue_1);
  //private_nh.setCallbackQueue(&callback_queue_2);
  private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
  private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
  private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
  private_nh.param("colored_map", m_useColoredMap, m_useColoredMap);
  private_nh.param("color_factor", m_colorFactor, m_colorFactor);
  private_nh.param("pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  private_nh.param("pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  private_nh.param("pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  private_nh.param("pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  private_nh.param("/occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  private_nh.param("min_x_size", m_minSizeX,m_minSizeX);
  private_nh.param("min_y_size", m_minSizeY,m_minSizeY);
  private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
  private_nh.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
  // distance of points from plane for RANSAC
  private_nh.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
  // angular derivation of found plane:
  private_nh.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  private_nh.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
  private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);
  private_nh.param("/om_res", m_res, m_res);
  private_nh.param("/MapToTransferSideLength", MapToTransferSideLength, MapToTransferSideLength);
  private_nh.param("/gridSliceCountPerSide", gridSliceCountPerSide, gridSliceCountPerSide);
  private_nh.param("/filterOctoMap", filterOctoMap, filterOctoMap);
  private_nh.param("/gridMode", gridMode, gridMode);
  private_nh.param("/om_to_pl_res", om_to_pl_res, om_to_pl_res);
  private_nh.param("/lower_resolution_relative_volume_width", om_to_pl_res_rel_vol_width, om_to_pl_res_rel_vol_width);
  private_nh.param("/lower_resolution_relative_volume_height", om_to_pl_res_rel_vol_height, om_to_pl_res_rel_vol_height);
  private_nh.param("/lower_resolution_relative_volume_length", om_to_pl_res_rel_vol_length, om_to_pl_res_rel_vol_length);
  private_nh.param("/voxel_type_to_publish", voxel_type_to_publish, voxel_type_to_publish);
  private_nh.param("sensor_model/hit", probHit, 0.7);
  private_nh.param("sensor_model/miss", probMiss, 0.4);
  private_nh.param("sensor_model/min", thresMin, 0.12);
  private_nh.param("sensor_model/max", thresMax, 0.97);
  private_nh.param("compress_map", m_compressMap, m_compressMap);
  private_nh.param("incremental_2D_projection", m_incrementalUpdate, m_incrementalUpdate);
  private_nh.param("CLCT_DATA", CLCT_DATA, false);
  private_nh.param("measure_time_end_to_end", measure_time_end_to_end, false);
  private_nh.param("data_collection_iteration_freq_OM", data_collection_iteration_freq, 100);
  private_nh.param("capture_size", capture_size, 600);
  private_nh.param("DEBUG_RQT", DEBUG_RQT, false);
  private_nh.param("/knob_performance_modeling", knob_performance_modeling, false);
  private_nh.param("/knob_performance_modeling_for_om_to_pl", knob_performance_modeling_for_om_to_pl, false);
  private_nh.param("/knob_performance_modeling_for_om_to_pl_no_interference", knob_performance_modeling_for_om_to_pl_no_interference, false);

  if (knob_performance_modeling){
	  capture_size = 1;
  }


  dist_to_closest_obs = m_maxRange;

  profile_manager_client = 
      private_nh.serviceClient<profile_manager::profiling_data_srv>("/record_profiling_data", true);

  profiling_container.capture("high_res_map_volume", "single", 0, 1);
  profiling_container.capture("low_res_map_volume", "single", 0, 1);


  if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)){
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<m_pointcloudMinZ <<", "<< m_pointcloudMaxZ << "], excluding the ground level z=0. "
              << "This will not work.");
  }

  if (m_useHeightMap && m_useColoredMap) {
    ROS_WARN_STREAM("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
    m_useColoredMap = false;
  }

  if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
    ROS_INFO_STREAM("Using RGB color registration (if information available)");
#else
    ROS_ERROR_STREAM("Colored map requested in launch file - node not running/compiled to support colors, please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
#endif
  }


  // initialize octomap object & params
  m_octree = new OcTreeT(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);
  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
  m_gridmap.info.resolution = m_res;


  m_octree_lower_res = new OcTreeT(om_to_pl_res);
  m_octree_lower_res->setProbHit(probHit);
  m_octree_lower_res->setProbMiss(probMiss);
  m_octree_lower_res->setClampingThresMin(thresMin);
  m_octree_lower_res->setClampingThresMax(thresMax);

  closest_obs_coord = point3d(m_maxRange, m_maxRange, m_maxRange);
  dist_to_closest_obs = calc_dist(closest_obs_coord, point3d(0,0,0));


  double r, g, b, a;
  private_nh.param("color/r", r, 0.0);
  private_nh.param("color/g", g, 0.0);
  private_nh.param("color/b", b, 1.0);
  private_nh.param("color/a", a, 1.0);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  private_nh.param("color_free/r", r, 0.0);
  private_nh.param("color_free/g", g, 1.0);
  private_nh.param("color_free/b", b, 0.0);
  private_nh.param("color_free/a", a, 1.0);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  private_nh.param("publish_free_space", m_publishFreeSpace, m_publishFreeSpace);
  last_time_cleared = ros::Time::now();


  private_nh.param("latch", m_latchedTopics, m_latchedTopics);
  if (m_latchedTopics){
    ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
  } else
    ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");

  m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, m_latchedTopics);
  inform_pc_done_pub = m_nh.advertise<std_msgs::Bool>("inform_pc_done", 1, m_latchedTopics);



  octomap_communication_proxy_msg =  m_nh.advertise<std_msgs::Header>("octomap_communication_proxy_msg", 1, m_latchedTopics);
  m_markerLowerResPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array_lower_res", 1, m_latchedTopics);
  //m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, m_latchedTopics);
  m_binaryMapPub = m_nh.advertise<mavbench_msgs::octomap_aug>("octomap_binary", 1, m_latchedTopics);
  //m_binaryMapLowerResPub = m_nh.advertise<Octomap>("octomap_binary_lower_res", 1, m_latchedTopics);
  m_binaryMapLowerResPub = m_nh.advertise<Octomap>("octomap_binary_lower_res", 1, m_latchedTopics);
  m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1, m_latchedTopics);
  m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, m_latchedTopics);
  m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 1, m_latchedTopics);
  m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, m_latchedTopics);

  octomap_debug_pub = m_nh.advertise<mavbench_msgs::octomap_debug>("octomap_debug", 1);

  m_pointCloudSub = new message_filters::Subscriber<mavbench_msgs::point_cloud_aug> (m_nh, "cloud_in", 1);
  m_tfPointCloudSub = new tf::MessageFilter<mavbench_msgs::point_cloud_aug> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 1);
  
  //m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 1);
  //m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 1);
  m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, _1));

  m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &OctomapServer::octomapBinarySrv, this);
  m_octomapFullService = m_nh.advertiseService("octomap_full", &OctomapServer::octomapFullSrv, this);
  m_clearBBXService = private_nh.advertiseService("clear_bbx", &OctomapServer::clearBBXSrv, this);
  m_resetService = private_nh.advertiseService("reset", &OctomapServer::resetSrv, this);


  m_octomapResetMaxRange = private_nh.advertiseService("reset_max_range", &OctomapServer::maxRangecb, this);
  m_octomapHeaderSub = private_nh.subscribe("octomap_header_col_detected", 1, &OctomapServer::OctomapHeaderColDetectedcb, this);
  //m_pc_meta_dataSub = private_nh.subscribe("/pc_meta_data", 1, &OctomapServer::PCMetaDataCb, this);
  m_save_map_pub = private_nh.subscribe("save_map", 1, &OctomapServer::SaveMapCb, this);


  dynamic_reconfigure::Server<OctomapServerConfig>::CallbackType f;
  f = boost::bind(&OctomapServer::reconfigureCallback, this, _1, _2);
  m_reconfigureServer.setCallback(f);

  //  --- repeating the parametet setting cause they get overwritern by setCallback (not sure what their code is written this way
  //      it also seems like, since the parameters get overwritten, the values set by the launch file is not being used,
  //      hence, you need to change the names. For example I changed, pointcloud_max_z to /point_cloud_max_z
  private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
  private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
  private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
  private_nh.param("colored_map", m_useColoredMap, m_useColoredMap);
  private_nh.param("color_factor", m_colorFactor, m_colorFactor);

  private_nh.param("pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  private_nh.param("pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  private_nh.param("pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  private_nh.param("pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh.param("/point_cloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  private_nh.param("/occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  private_nh.param("min_x_size", m_minSizeX,m_minSizeX);
  private_nh.param("min_y_size", m_minSizeY,m_minSizeY);

  private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
  private_nh.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
  private_nh.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
  private_nh.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  private_nh.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
  private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);
  private_nh.param("sensor_model/hit", probHit, 0.7);
  private_nh.param("sensor_model/miss", probMiss, 0.4);
  private_nh.param("sensor_model/min", thresMin, 0.12);
  private_nh.param("sensor_model/max", thresMax, 0.97);
  private_nh.param("/om_to_pl_vol_ideal", om_to_pl_vol_ideal, om_to_pl_vol_ideal);
  private_nh.param("/om_to_pl_res", om_to_pl_res, om_to_pl_res);


  // Profiling
  octomap_integration_acc = 0;
  pt_cld_octomap_commun_overhead_acc = 0; 
  octomap_ctr = 0;


//  private_nh_2.setCallbackQueue(&callback_queue_meta_data);


}



void OctomapServer::spinOnce(){
	// we need two different queues so we can make sure that we can maintain a certain order. Don't mess with the ordering
	ros::spinOnce();
//	callback_queue_2.callAvailable(ros::WallDuration());  // -- first, get the meta data (i.e., resolution, volume)
//	callback_queue_1.callAvailable(ros::WallDuration());  // -- 2nd, get the data (point cloud)
}





OctomapServer::~OctomapServer(){
    if (m_tfPointCloudSub){
    delete m_tfPointCloudSub;
    m_tfPointCloudSub = NULL;
  }

  if (m_pointCloudSub){
    delete m_pointCloudSub;
    m_pointCloudSub = NULL;
  }


  if (m_octree){
    delete m_octree;
    m_octree = NULL;
  }

}

bool OctomapServer::openFile(const std::string& filename){
  if (filename.length() <= 3)
    return false;

  std::string suffix = filename.substr(filename.length()-3, 3);
  if (suffix== ".bt"){
    if (!m_octree->readBinary(filename)){
      return false;
    }
  } else if (suffix == ".ot"){
    AbstractOcTree* tree = AbstractOcTree::read(filename);
    if (!tree){
      return false;
    }
    if (m_octree){
      delete m_octree;
      m_octree = NULL;
    }
    m_octree = dynamic_cast<OcTreeT*>(tree);
    if (!m_octree){
      ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
      return false;
    }

  } else{
    return false;
  }

  ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octree->size());

  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
  m_res = m_octree->getResolution();
  m_gridmap.info.resolution = m_res;
  double minX, minY, minZ;
  double maxX, maxY, maxZ;
  m_octree->getMetricMin(minX, minY, minZ);
  m_octree->getMetricMax(maxX, maxY, maxZ);

  m_updateBBXMin[0] = m_octree->coordToKey(minX);
  m_updateBBXMin[1] = m_octree->coordToKey(minY);
  m_updateBBXMin[2] = m_octree->coordToKey(minZ);

  m_updateBBXMax[0] = m_octree->coordToKey(maxX);
  m_updateBBXMax[1] = m_octree->coordToKey(maxY);
  m_updateBBXMax[2] = m_octree->coordToKey(maxZ);

  publishAll();

  return true;

}


double OctomapServer::calcTreeVolume(OcTreeT* tree){
	double volume = 0;
	for (OcTree::leaf_iterator it = tree->begin_leafs(),
      end = tree->end_leafs();  it != end; ++it){
	  volume += pow(it.getSize(),3);
    }
	return volume;
}


using namespace std; //
//void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
void OctomapServer::insertCloudCallback(const mavbench_msgs::point_cloud_aug::ConstPtr& pcl_aug_data){
//	m_octree->clear();
	pc_capture_time = ros::Time::now();
	const sensor_msgs::PointCloud2 * cloud = &(pcl_aug_data->pcl);
	// -- insert the point cloud into the map
	pc_vol_actual = pcl_aug_data->ee_profiles.actual_cmds.pc_vol; // -- this is a hack, sicne I have overwritten the field value with volume
	pc_res =  pcl_aug_data->controls.cmds.pc_res;
	om_to_pl_res = pcl_aug_data->controls.cmds.om_to_pl_res;
	om_to_pl_vol_ideal = pcl_aug_data->controls.cmds.om_to_pl_vol;
	ppl_vol_ideal = pcl_aug_data->controls.cmds.ppl_vol;

	// -- for profiling
	octomap_aug_data.controls = pcl_aug_data->controls;
	octomap_aug_data.ee_profiles = pcl_aug_data->ee_profiles;
	octomap_aug_data.ee_profiles.actual_time.pc_to_om_ros_oh =  (ros::Time::now() - pcl_aug_data->ee_profiles.actual_time.pc_pre_pub_time_stamp).toSec();

	// -- this is only for knob performance modeling,
	// -- the idea is that since, we don't want the pressure on compute for processing octomap impacts the octomap to planning
	// -- communication, we simply send the map over and return, without integrating any new information
	if (knob_performance_modeling){
		ros::param::get("/knob_performance_modeling_for_om_to_pl", knob_performance_modeling_for_om_to_pl);
		ros::param::get("/knob_performance_modeling_for_om_to_pl_no_interference", knob_performance_modeling_for_om_to_pl_no_interference);
		if (knob_performance_modeling_for_om_to_pl_no_interference){
			profiling_container.capture("octomap_publish_all", "start", ros::Time::now(), capture_size);
			publishAll(ros::Time::now());
			profiling_container.capture("octomap_publish_all", "end", ros::Time::now(), capture_size);
			return;
		}
	}

    // python dummy value get-set
	float dummy_val;
	ros::param::get("optimizer_node/dummy_val", dummy_val);
	dummy_val++;
	ros::param::set("optimizer_node/dummy_val", dummy_val);

    if(CLCT_DATA) {
		ros::Time start_time = ros::Time::now();
		pt_cld_octomap_commun_overhead_acc +=  (start_time - cloud->header.stamp).toSec()*1e9;
		octomap_ctr++;
	}
	rcvd_point_cld_time_stamp = cloud->header.stamp;


	if (measure_time_end_to_end){
		profiling_container.capture("sensor_to_octomap_time", "start", cloud->header.stamp, capture_size);
		profiling_container.capture("sensor_to_octomap_time", "end", ros::Time::now(), capture_size);
	}else{
		profiling_container.capture("point_cloud_to_octomap_communication_time", "start", cloud->header.stamp, capture_size);
		profiling_container.capture("point_cloud_to_octomap_communication_time", "end", ros::Time::now(), capture_size);
	}

	profiling_container.capture(std::string("octomap_insertCloud_minus_publish_all"), "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")

	// ground filtering in base frame
	PCLPointCloud pc; // input cloud for filtering and ground-detection

	profiling_container.capture(std::string("point_cloud_deserialization"), "start", ros::Time::now(), capture_size);

	sensor_msgs::PointCloud2 cloud_ = *cloud;
	cloud_.fields[0].count = 1;
	pcl::fromROSMsg(cloud_, pc);
	profiling_container.capture(std::string("point_cloud_deserialization"), "end", ros::Time::now(), capture_size);
	//ROS_INFO_STREAM(std::string("octomap deserialization time")<<this->profiling_container.findDataByName("point_cloud_deserialization")->values.back());

	profiling_container.capture("octomap_filter", "start", ros::Time::now(), capture_size); //collect data
	tf::StampedTransform sensorToWorldTf;
	try {
		m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
	} catch(tf::TransformException& ex){
		ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);



	// set up filter for height range, also removes NANs:
	pcl::PassThrough<PCLPoint> pass_x;
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
	pcl::PassThrough<PCLPoint> pass_y;
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
	pcl::PassThrough<PCLPoint> pass_z;
	pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

  PCLPointCloud pc_ground; // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  if (m_filterGroundPlane){
    tf::StampedTransform sensorToBaseTf, baseToWorldTf;
    try{
      m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
      m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
      m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, baseToWorldTf);


    }catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
    		  "You need to set the base_frame_id or disable filter_ground.");
    }


    Eigen::Matrix4f sensorToBase, baseToWorld;
    pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
    pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensorToBase);
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);
    filterGroundPlane(pc, pc_ground, pc_nonground);

    // transform clouds to world frame for insertion
    pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
    pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
  } else {
    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    // just filter height range:
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }
  profiling_container.capture("octomap_filter", "end", ros::Time::now(), capture_size);
  //ROS_INFO_STREAM("octomap filter time"<<this->profiling_container.findDataByName("octomap_filter")->values.back());


  profiling_container.capture("sensor_volume_to_digest_estimated", "single", pcl_aug_data->controls.inputs.sensor_volume_to_digest_estimated, capture_size);

  profiling_container.capture("insertScan", "start", ros::Time::now(), capture_size);
  insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);
  profiling_container.capture("insertScan", "end", ros::Time::now(), capture_size);
  profiling_container.capture("perceived_closest_obstacle", "single", dist_to_closest_obs, capture_size);

  //ROS_INFO_STREAM("octomap insertCloud time"<<this->profiling_container.findDataByName("octomap_insertCloud")->values.back());

  //ROS_INFO_STREAM("-------------- exposed volume"<<exposed_volume);
  profiling_container.capture("octomap_insertCloud_minus_publish_all", "end", ros::Time::now(), capture_size);
  profiling_container.capture("pc_vol_actual", "single", pc_vol_actual, capture_size);
  profiling_container.capture("pc_res", "single", pc_res, capture_size);


  profiling_container.capture("octomap_publish_all", "start", ros::Time::now(), capture_size);
  if (measure_time_end_to_end){ publishAll(cloud->header.stamp);}
  else{publishAll(ros::Time::now());}
  profiling_container.capture("octomap_publish_all", "end", ros::Time::now(), capture_size);


  /*// for debugging purposes. Don't delete, just comment out
  {
	  ROS_ERROR_STREAM("--------- this is very expensive operation, deactive the block when not needed");
	  double high_res_volume  = calcTreeVolume(m_octree);
	  double low_res_volume  = calcTreeVolume(m_octree_lower_res);
	  profiling_container.capture("high_res_map_volume", "single", high_res_volume, 1);
	  profiling_container.capture("low_res_map_volume", "single", low_res_volume, 1);

  }
*/


  if (m_save_map){ // for micro benchmarking purposes
	  m_octree->write("high_res_map.ot");
	  m_octree_lower_res->write("low_res_map.ot");
	  m_save_map = false; //reset m_save_map to avoid rewriting
  }

  if (DEBUG_RQT) {
	  	debug_data.header.stamp = ros::Time::now();
	    debug_data.octomap_insertScan = profiling_container.findDataByName("insertScan")->values.back();
	  	debug_data.octomap_insertCloud_minus_publish_all = profiling_container.findDataByName("octomap_insertCloud_minus_publish_all")->values.back();
	  	debug_data.octomap_publish_all = profiling_container.findDataByName("octomap_publish_all")->values.back();
	  	debug_data.pc_vol_actual = profiling_container.findDataByName("pc_vol_actual")->values.back();
	  	//debug_data.octomap_memory_usage =  m_octree->memoryUsage();
	  	//debug_data.octomap_low_res_memory_usage =  m_octree_lower_res->memoryUsage();
	  	debug_data.perceived_closest_obs_distance =  profiling_container.findDataByName("perceived_closest_obstacle")->values.back();
//	  	debug_data.octomap_serialization_high_res_time =  profiling_container.findDataByName("octomap_serialization_high_res_time")->values.back();
//	  	debug_data.octomap_serialization_low_res_time =  profiling_container.findDataByName("octomap_serialization_low_res_time")->values.back();
//	  	debug_data.high_res_map_volume =  profiling_container.findDataByName("high_res_map_volume")->values.back();
//	  	debug_data.low_res_map_volume =  profiling_container.findDataByName("low_res_map_volume")->values.back();
	  	if (filterOctoMap) { debug_data.octomap_filtering_time =  profiling_container.findDataByName("octomap_filtering_time")->values.back();}
	  	else{debug_data.octomap_filtering_time =  0;}
	  	octomap_debug_pub.publish(debug_data);
  }
}

void OctomapServer::SaveMapCb(std_msgs::Bool msg){
	m_save_map = true;
}


void OctomapServer::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& ground, const PCLPointCloud& nonground){

   point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);
   double temp_res;
   ros::param::get("/om_res", temp_res);
   if (temp_res != m_res) { //construct a new map
	   ROS_INFO_STREAM("--- request was placed to change resolution. This is costly, so avoid overusing it!");
	   profiling_container.capture("construct_diff_res_map_latency", "start", ros::Time::now(), capture_size);
	   construct_diff_res_map(temp_res, sensorOrigin);
	   profiling_container.capture("construct_diff_res_map_latency", "end", ros::Time::now(), capture_size);
	    cout<<"constructing map took:"<<profiling_container.findDataByName("construct_diff_res_map_latency")->values.back()<<endl;;
	   m_res = temp_res;
	   return;
   }


   if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
    || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  {
    ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
  }

#ifdef COLOR_OCTOMAP_SERVER
  unsigned char* colors = new unsigned char[3];
#endif


  if (first_time_scanning) {
	  originalSensorOrigin = sensorOrigin; // to ensure the shrunk tree maintains the same origin as the normal tree
	  first_time_scanning = false;
	  ROS_ERROR_STREAM("getting sensor origin");
  }

  //update the closest obstacle
  dist_to_closest_obs = calc_dist(closest_obs_coord, sensorOrigin);

  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells, occupied_cells;
  // insert ground points only as free:
  for (PCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
      point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
    }

    // only clear space (ground points)
    if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
    	free_cells.insert(m_keyRay.begin(), m_keyRay.end());
    }

    octomap::OcTreeKey endKey;
    if (m_octree->coordToKeyChecked(point, endKey)){
      updateMinKey(endKey, m_updateBBXMin);
      updateMaxKey(endKey, m_updateBBXMax);
    } else{
      ROS_ERROR_STREAM("Could not generate Key for endpoint "<<point);
    }
  }

  profiling_container.capture("octomap_calc_hash", "start", ros::Time::now(), capture_size);
  // all other points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

      // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
    	  free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)){
        occupied_cells.insert(key);

        updateMinKey(key, m_updateBBXMin);
        updateMaxKey(key, m_updateBBXMax);

#ifdef COLOR_OCTOMAP_SERVER // NB: Only read and interpret color if it's an occupied node
        const int rgb = *reinterpret_cast<const int*>(&(it->rgb)); // TODO: there are other ways to encode color than this one
        colors[0] = ((rgb >> 16) & 0xff);
        colors[1] = ((rgb >> 8) & 0xff);
        colors[2] = (rgb & 0xff);
        m_octree->averageNodeColor(it->x, it->y, it->z, colors[0], colors[1], colors[2]);
#endif
      }
    } else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
    	  free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)){
          free_cells.insert(endKey);
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
        } else{
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }


      }
    }
  }
  profiling_container.capture("octomap_calc_hash", "end", ros::Time::now(), capture_size);
  if (DEBUG_RQT) {
	  	debug_data.header.stamp = ros::Time::now();
	    debug_data.octomap_calc_hash= profiling_container.findDataByName("octomap_calc_hash")->values.back();
		//octomap_debug_pub.publish(debug_data);
  }

  profiling_container.capture("octomap_calc_disjoint_and_update", "start", ros::Time::now(), capture_size);

  sensorOrigin_ = sensorOrigin;
  // mark free cells only if not seen occupied in this cloud
  double update_low_res_total = 0;
  int free_cells_cnt = 0; // -- keep track of number of free cells (can't get this from free_cell.size() because of the overlap between occupied and free cell)
  float depth_acc_touched = 0;
  int cell_touched_cnt = 0;
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
	  if (occupied_cells.find(*it) == occupied_cells.end()){
    	const OcTreeKey my_key = *it;
    	//auto my_key_ = m_octree->adjustKeyAtDepth(my_key,10);
    	auto high_res_node = m_octree->updateNode((OcTreeKey) my_key, false, false);

    	//auto coordinate = m_octree->keyToCoord(*it);
    	//auto high_res_node = m_octree->updateNode(coordinate.x(), coordinate.y(), coordinate.z(), false);
    	//cell_touched_cnt +=1;
    	//ros::Time low_res_start = ros::Time::now() ;
    	//update_lower_res_map(coordinate, high_res_node);
    	//update_low_res_total += (ros::Time::now() - low_res_start).toSec();
    	free_cells_cnt +=1;
	  }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
	  const OcTreeKey my_key = *it;
	  //auto my_key_ = m_octree->adjustKeyAtDepth(my_key,10);
	  auto high_res_node = m_octree->updateNode((OcTreeKey) my_key, true, false);

	  // calculate the closest obstacle distance
	  auto coordinate = m_octree->keyToCoord(*it);
	  tree_max_x = max(tree_max_x, coordinate.x());
	  tree_max_y = max(tree_max_y, coordinate.y());
	  tree_max_z = max(tree_max_z, coordinate.z());

	  update_closest_obstacle(coordinate, sensorOrigin);

	  //auto high_res_node = m_octree->updateNode(coordinate.x(), coordinate.y(), coordinate.z(), true);
	  //lower resolution map handling
	  //ros::Time low_res_start = ros::Time::now() ;
	  //depth_acc_touched += m_octree->depth_touched; //comment this if your octomap is unmodified
	  //cell_touched_cnt +=1;
	  //auto coordinate = m_octree->keyToCoord(*it);
	  //update_lower_res_map(coordinate, high_res_node);
	  //update_low_res_total += (ros::Time::now() - low_res_start).toSec();
  }

 // auto free_cell_volume = free_cells_cnt*pow(pc_res, 3); // -- we use pc_res instead of octomap->resolution() because exposed resolution is how point cloud is spaced out,
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //    hence, tehnically, that's the volume we cover
  //auto occupied_cell_volume = occupied_cells.size()*pow(pc_res, 3);


  auto free_cell_volume = free_cells_cnt*pow(m_octree->getResolution(), 3); // -- we use pc_res instead of octomap->resolution() because exposed resolution is how point cloud is spaced out,
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //    hence, tehnically, that's the volume we cover
  auto occupied_cell_volume = occupied_cells.size()*pow(m_octree->getResolution(), 3);

  profiling_container.capture(std::string("octomap_avg_depth_touched"), "single", (float) depth_acc_touched/cell_touched_cnt, capture_size);
  profiling_container.capture(std::string("update_lower_res_map"), "single", update_low_res_total , capture_size);
  profiling_container.capture("perceived_closest_obs_distance", "single", dist_to_closest_obs, capture_size);
  profiling_container.capture("octomap_calc_disjoint_and_update", "end", ros::Time::now(), capture_size);

  // TODO: eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  //m_octree->updateInnerOccupancy();
  octomap::point3d minPt, maxPt;
  ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

  // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
//   if (m_maxTreeDepth < 16)
//   {
//      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth); // this should give us the first key at depth m_maxTreeDepth that is smaller or equal to m_updateBBXMin (i.e. lower left in 2D grid coordinates)
//      OcTreeKey tmpMax = getIndexKey(m_updateBBXMax, m_maxTreeDepth); // see above, now add something to find upper right
//      tmpMax[0]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      tmpMax[1]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      tmpMax[2]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
//      m_updateBBXMin = tmpMin;
//      m_updateBBXMax = tmpMax;
//   }

  // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
  minPt = m_octree->keyToCoord(m_updateBBXMin);
  maxPt = m_octree->keyToCoord(m_updateBBXMax);
  ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);


  profiling_container.capture("octomap_prune_in_octomap_server", "start", ros::Time::now(), capture_size);

  profiling_container.capture(std::string("construct_lower_res_map"), "start", ros::Time::now());
  //construct_lower_res_map(om_to_pl_res, point3d(0,0,0));//, sensorToWorldTf.getOrigin()));
  profiling_container.capture(std::string("construct_lower_res_map"), "end", ros::Time::now());

   //attempt to limite the map size
   /*
   point3d drone_cur_pos(sensorOrigin.x(), sensorOrigin.y(), sensorOrigin.z());
   double high_res_max_x, high_res_max_y, high_res_max_z;
   double high_res_min_x, high_res_min_y, high_res_min_z;
   m_octree->getMetricMax(high_res_max_x, high_res_max_y, high_res_max_z);
   m_octree->getMetricMin(high_res_min_x, high_res_min_y, high_res_min_z);
   double low_res_max_x, low_res_max_y, low_res_max_z;
   double low_res_min_x, low_res_min_y, low_res_min_z;
   low_res_max_x = drone_cur_pos.x() + om_to_pl_res_rel_vol_width;
   low_res_max_y = drone_cur_pos.y() + om_to_pl_res_rel_vol_length; //into the screen
   low_res_max_z = drone_cur_pos.z() + om_to_pl_res_rel_vol_height;
   low_res_min_x = drone_cur_pos.x() - om_to_pl_res_rel_vol_width;
   low_res_min_y = drone_cur_pos.y() - om_to_pl_res_rel_vol_length; //into the screen
   low_res_min_z = drone_cur_pos.z() - om_to_pl_res_rel_vol_height;

   auto bbxMin = octomap::point3d(std::max(low_res_min_x, high_res_min_x), std::max(low_res_min_y, high_res_min_y), std::max(low_res_min_z, high_res_min_z));
   auto bbxMax = octomap::point3d(std::min(low_res_max_x, high_res_max_x), std::min(low_res_max_y, high_res_max_y), std::min(low_res_max_z, high_res_max_z));

   auto new_tree = new OcTreeT(m_res);
   new_tree->setProbHit(m_octree->getProbHit());
   new_tree->setProbMiss(m_octree->getProbMiss());
   new_tree->setClampingThresMin(m_octree->getClampingThresMin());
   new_tree->setClampingThresMax(m_octree->getClampingThresMax());
   new_tree->setClampingThresMax(m_octree->getClampingThresMax());

   for(typename OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(bbxMin,bbxMax), end=m_octree->end_leafs_bbx(); it!= end; ++it){
		new_tree->updateNode(it.getCoordinate(), it->getLogOdds(), true);
   }
   delete m_octree;
   m_octree = new_tree;
   */


  if (m_compressMap){
    //m_octree->prune();
    //m_octree_lower_res->prune();
  }

  profiling_container.capture("octomap_prune_in_octomap_server", "end", ros::Time::now(), capture_size);
  profiling_container.capture("octomap_volume_digested", "single", free_cell_volume + occupied_cell_volume, capture_size);


  if (DEBUG_RQT) {
	  	debug_data.header.stamp = ros::Time::now();
	    debug_data.update_lower_resolution_map = profiling_container.findDataByName("update_lower_res_map")->values.back();
	  	debug_data.octomap_calc_disjoint_and_update = profiling_container.findDataByName("octomap_calc_disjoint_and_update")->values.back();
		debug_data.perceived_closest_obs_distance = profiling_container.findDataByName("perceived_closest_obs_distance")->values.back();
		debug_data.octomap_avg_depth_touched= profiling_container.findDataByName("octomap_avg_depth_touched")->values.back();
		debug_data.octomap_prune_in_octomap_server = profiling_container.findDataByName("octomap_prune_in_octomap_server")->values.back();
		debug_data.octomap_construct_lower_res_map= profiling_container.findDataByName("construct_lower_res_map")->values.back();
		debug_data.octomap_space_volume_digested =  free_cell_volume + occupied_cell_volume;
		debug_data.integrated_volume_tracking_error =  (free_cell_volume + occupied_cell_volume) - profiling_container.findDataByName("sensor_volume_to_digest_estimated")->values.back();

		//octomap_debug_pub.publish(debug_data);
  }
  ROS_INFO_STREAM("OM vol"<<free_cell_volume + occupied_cell_volume);

  //profiling_container.capture("octomap_exposed_volume", "single", free_cell_volume + occupied_cell_volume, capture_size);


#ifdef COLOR_OCTOMAP_SERVER
  if (colors)
  {
    delete[] colors;
    colors = NULL;
  }
#endif
}

// calculate dist between two points
double OctomapServer::calc_dist(point3d point1, point3d point2) {
	double dx = point1.x() - point2.x();
	double dy = point1.y() - point2.y();
	double dz = point1.z() - point2.z();
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}


// this function is conserverative, that's if doesn't not correct for obstacles that are
// labeled as occupied before, but not occupied now anymore
void OctomapServer::update_closest_obstacle(point3d coordinate, point3d sensorOrigin) {
	double dist_to_this_obs = calc_dist(coordinate, sensorOrigin);
	dist_to_closest_obs = calc_dist(closest_obs_coord, sensorOrigin);
	if (dist_to_this_obs <= dist_to_closest_obs){
		dist_to_closest_obs = dist_to_this_obs;
		closest_obs_coord = coordinate;
	}
}


void OctomapServer::update_lower_res_map(point3d coordinate, OcTreeNode* high_res_node){
    //lower resolution update
    //auto node_to_look_at = m_octree_lower_res->search(coordinate, m_treeDepth);
    //auto high_res_node = m_octree->search(coordinate);
    bool occupancy = m_octree->isNodeOccupied(high_res_node);
    m_octree_lower_res->updateNode(coordinate, occupancy, true);
   /*
    if (node_to_look_at){ //if node already exist
    	currently_occupied = m_octree_lower_res->isNodeOccupied(node_to_look_at);
    }else{
    	currently_occupied = false;
    }

    if ((!currently_occupied && set_occupied) || (currently_occupied && !set_occupied)){ //avoid redundant updates, overwrite if the values don't match
    	auto high_res_node = m_octree->search(coordinate);
    	bool occupancy = m_octree->isNodeOccupied(high_res_node);
    	m_octree_lower_res->updateNode(coordinate, occupancy, set_occupied);
    }
    */
}

// generate the offsets from thet current drone's position to sample octomap from
void generateOffSets(vector<point3d> & offset_vals, float GridSize, float zGridSize, int GridCount, string mode ="2d", string bias_mode = "none"){
	int lowerBoundX,  lowerBoundY, lowerBoundZ, upperBoundX, upperBoundY, upperBoundZ;
	if (mode == "3d"){
		if (bias_mode == "pos"){ lowerBoundX = lowerBoundY = lowerBoundZ = 0;}
		else{lowerBoundX = lowerBoundY = lowerBoundZ = -1*int(pow(GridCount, float(1)/3)/2);}

		upperBoundX = upperBoundY = upperBoundZ = int(pow(GridCount, float(1)/3)/2) + 1;
		//lowerBoundZ = -1*int(pow(ZGridCount, float(1)/3)/2);
		//upperBoundZ = int(pow(ZGridCount, float(1)/3)/2) + 1;
	}else{ // 2d

		if (bias_mode == "pos"){ lowerBoundX = lowerBoundY = lowerBoundZ = 0;}
		else{lowerBoundX = lowerBoundY = -1*int(pow(GridCount, float(1)/2)/2);}
		upperBoundX = upperBoundY =  int(pow(GridCount, float(1)/2)/2) + 1;
		upperBoundZ = 1;
		lowerBoundZ = 0;
	}

	for (int i = lowerBoundX; i < upperBoundX; i++){
		for (int j = lowerBoundY; j < upperBoundY; j++){
			for (int k = lowerBoundZ; k < upperBoundZ; k++){
				offset_vals.push_back(point3d(i*GridSize, j*GridSize, k*zGridSize));  // note that I hald z, because I am making a decision that maintaing z is not as important
			}
		}
	}
}


void OctomapServer::gen_coordinates_to_consider(const OcTreeT::leaf_bbx_iterator &it, OcTreeT* cur_octree, vector<octomap::point3d> &coord_vec){
	//cout<<"depth is" <<it.getDepth();
	if (it.getDepth() != cur_octree->getTreeDepth()) {
		int cubeSize = 1 << (cur_octree->getTreeDepth() - it.getDepth());
		octomap::OcTreeKey key=it.getIndexKey();
		for(int dx = 0; dx < cubeSize; dx++){
			for(int dy = 0; dy < cubeSize; dy++){
				for(int dz = 0; dz < cubeSize; dz++){
					unsigned short int tmpx = key[0]+dx;
					unsigned short int tmpy = key[1]+dy;
					unsigned short int tmpz = key[2]+dz;
					coord_vec.push_back(m_octree->keyToCoord(octomap::OcTreeKey(tmpx, tmpy, tmpz)));
				}
			}
		}
	}else{
		coord_vec.push_back(it.getCoordinate());
	}
	/*
	if (coord_vec.size() > 1) {
		cout<<"vec size"<<coord_vec.size()<<endl;
	}
	*/
}

// traverse the current tree entirely, expand leafs (if not at max depth possible) and sample arround them properly
// to generate the different resolution tree
void OctomapServer::construct_diff_res_multiple_of_two_map(double diff_res, OcTreeT* m_octree_temp){
	double cur_res_max_x, cur_res_max_y, cur_res_max_z;
	double cur_res_min_x, cur_res_min_y, cur_res_min_z;
	m_octree->getMetricMax(cur_res_max_x, cur_res_max_y, cur_res_max_z);
	m_octree->getMetricMin(cur_res_min_x, cur_res_min_y, cur_res_min_z);
	auto bbxMin = octomap::point3d(cur_res_min_x, cur_res_min_y, cur_res_min_z);
	auto bbxMax = octomap::point3d(cur_res_max_x, cur_res_max_y, cur_res_max_z);
	for(typename OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(bbxMin,bbxMax), end=m_octree->end_leafs_bbx(); it!= end; ++it){
		;
		/*
		//cout<<it.getDepth()<<endl;
		// generate coordinates to iterate over for this it
		coord_vec.clear();
		gen_coordinates_to_consider(it, m_octree, coord_vec);
		*/
	}
}

// traverse the current tree entirely, expand leafs (if not at max depth possible) and sample arround them properly
// to generate the different resolution tree
void OctomapServer::construct_diff_res_non_multiple_of_two_map(double diff_res, OcTreeT* m_octree_temp){
	m_octree_temp->setProbHit(m_octree->getProbHit());
	m_octree_temp->setProbMiss(m_octree->getProbMiss());
	m_octree_temp->setClampingThresMin(m_octree->getClampingThresMin());
	m_octree_temp->setClampingThresMax(m_octree->getClampingThresMax());

	auto key = m_octree->coordToKey(originalSensorOrigin);
	auto m_octree_node = m_octree->search(key);
	m_octree_temp->updateNode(m_octree->coordToKey(originalSensorOrigin), m_octree->isNodeOccupied(m_octree_node));


	//m_octree->calcMinMax();
	double cur_res_max_x, cur_res_max_y, cur_res_max_z;
	double cur_res_min_x, cur_res_min_y, cur_res_min_z;
	m_octree->getMetricMax(cur_res_max_x, cur_res_max_y, cur_res_max_z);
	m_octree->getMetricMin(cur_res_min_x, cur_res_min_y, cur_res_min_z);

	/*
	// logic to filter in a cerat volume around drone's current position
	double low_res_max_x, low_res_max_y, low_res_max_z;
	double low_res_min_x, low_res_min_y, low_res_min_z;
	low_res_max_x = drone_cur_pos.x() + om_to_pl_res_rel_vol_width;
	low_res_max_y = drone_cur_pos.y() + om_to_pl_res_rel_vol_length; //into the screen
	low_res_max_z = drone_cur_pos.z() + om_to_pl_res_rel_vol_height;

	low_res_min_x = drone_cur_pos.x() - om_to_pl_res_rel_vol_width;
	low_res_min_y = drone_cur_pos.y() - om_to_pl_res_rel_vol_length; //into the screen
	low_res_min_z = drone_cur_pos.z() - om_to_pl_res_rel_vol_height;

	//	auto bbxMin = octomap::point3d(std::max(low_res_min_x, cur_res_min_x), std::max(low_res_min_y, cur_res_min_y), std::max(low_res_min_z, cur_res_min_z));
	//	auto bbxMax = octomap::point3d(std::min(low_res_max_x, cur_res_max_x), std::min(low_res_max_y, cur_res_max_y), std::min(low_res_max_z, cur_res_max_z));
	 */

	auto bbxMin = octomap::point3d(cur_res_min_x, cur_res_min_y, cur_res_min_z);
	auto bbxMax = octomap::point3d(cur_res_max_x, cur_res_max_y, cur_res_max_z);


	// upsample (if necessary) a current coordinate to fill the holes (generated due to increasing resolution)
	// to do this, we calculate offsets form the coordinate and later on sample the cur resolution tree with them
	vector<point3d> offset_vals;
	double grid_size = diff_res;
	// note: pow(2,3) is for include both forward and backward in 3 axis.
	int macroGridCount = pow(2,3); // 8 macro grids, basically 8 quadrants (rahter octoants)
	int microGridCount = int(pow(2,3))*int(pow(int(m_res/diff_res)+1, 3));  // +1 is to make sure if the two resolutions
																			 // do not align, we sample a bit more to
																		     // cover everything
	generateOffSets(offset_vals, grid_size, grid_size, macroGridCount*microGridCount, gridMode) ;

	vector<point3d> coord_vec; // vector containing the coordinates from tree one to incoorperate (generated from specific
							   // iterator. This is handy when the leaf iterator is not at max depth
							   // and we need to expan the leaf
	for(typename OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(bbxMin,bbxMax), end=m_octree->end_leafs_bbx(); it!= end; ++it){
		// generate coordinates to iterate over for this it
		coord_vec.clear();
		// expand the leafs that do not have max depth and generate their coordinates
		gen_coordinates_to_consider(it, m_octree, coord_vec);
		for (auto it_coord = coord_vec.begin(); it_coord != coord_vec.end(); it_coord++){
			for (auto it_ = offset_vals.begin(); it_ != offset_vals.end(); it_++) {
				auto node_to_look_at = m_octree->search(*it_ + *it_coord);
				if (!node_to_look_at){
					continue;
				}
				float occupancy_value = node_to_look_at->getLogOdds();
				m_octree_temp->updateNode(*it_ + it.getCoordinate(), occupancy_value, true);
			}
		}
	}
}

// construct another map from the first, by traversing the entire tree
// and upsampling (if necessary); this operation is computationaly very expensive
void OctomapServer::construct_diff_res_map(double diff_res, point3d drone_cur_pos){
	auto m_octree_temp = new OcTreeT(diff_res);
	if (diff_res == m_res){ //if resolutions are the same, copy the pointer
		//m_octree_lower_res = m_octree;
		delete m_octree_temp;
		return;
	} else if (int(diff_res/m_res) == 2) {
		construct_diff_res_multiple_of_two_map(diff_res, m_octree_temp);
	} else{
		construct_diff_res_non_multiple_of_two_map(diff_res, m_octree_temp);

	}

	delete m_octree;
	m_octree = m_octree_temp;
}


void OctomapServer::publishAll(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
  bool publishMarkerArrayLowerRes = (m_latchedTopics || m_markerLowerResPub.getNumSubscribers() > 0);

  bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
  bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
  bool publishBinaryLowerResMap = (m_latchedTopics || m_binaryMapLowerResPub.getNumSubscribers() > 0);
  bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
  m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

  // init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth+1);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> pclCloud;

  /*
  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
      end = m_octree->end(); it != end; ++it)
  {
    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (inUpdateBBX)
      handleNodeInBBX(it);

    if (m_octree->isNodeOccupied(*it)){
      double z = it.getZ();
      if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        // Ignore speckles in the map:
        if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
          ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        } // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (inUpdateBBX)
          handleOccupiedNodeInBBX(it);


        //create marker:
        if (publishMarkerArray){
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
          if (m_useHeightMap){
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }

#ifdef COLOR_OCTOMAP_SERVER
          if (m_useColoredMap) {
            std_msgs::ColorRGBA _color; _color.r = (r / 255.); _color.g = (g / 255.); _color.b = (b / 255.); _color.a = 1.0; // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
            occupiedNodesVis.markers[idx].colors.push_back(_color);
          }
#endif
        }

        // insert into pointcloud:
        if (publishPointCloud) {
#ifdef COLOR_OCTOMAP_SERVER
          PCLPoint _point = PCLPoint();
          _point.x = x; _point.y = y; _point.z = z;
          _point.r = r; _point.g = g; _point.b = b;
          pclCloud.push_back(_point);
#else
          pclCloud.push_back(PCLPoint(x, y, z));
#endif
        }

      }
    } else{ // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        handleFreeNode(it);
        if (inUpdateBBX)
          handleFreeNodeInBBX(it);

        if (m_publishFreeSpace){
          double x = it.getX();
          double y = it.getY();

          //create marker for free space:
          if (publishFreeMarkerArray){
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }

      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      if (!m_useColoredMap)
        occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_markerPub.publish(occupiedNodesVis);
  }


  // finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;


      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_fmarkerPub.publish(freeNodesVis);
  }


  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
//    cloud.header.stamp2 = rostime;
    m_pointCloudPub.publish(cloud);
  }
  */



  if (publishBinaryMap){
	  if (filterOctoMap) { // if not set, simply pass the entire map
		  //publishFilteredBinaryOctoMap(rostime, sensorOrigin_);
		  publishFilteredByVolumeBySamplingBinaryOctoMap(rostime, sensorOrigin_);
	  }else{
		  publishBinaryOctoMap(rostime);
	  }

	  if (measure_time_end_to_end) { // just a proxy so we can still know the communication overhead
		  	  	  	  	  	  	     // this is necessary since if we want to measure end-to-end
		  	  	  	  	  	  	     // we can't directly measure the communication overhead
		  std_msgs::Header msg;
		  msg.stamp = ros::Time::now();
		  octomap_communication_proxy_msg.publish(msg);
	  }
  }

  profiling_container.capture("octomap_serialization_low_res_time", "start", ros::Time::now(), capture_size);
  if (publishBinaryLowerResMap){
	  publishBinaryLowerResOctoMap(rostime);
  }
  profiling_container.capture("octomap_serialization_low_res_time", "end", ros::Time::now(), capture_size);


   if (publishMarkerArray){
	publish_octomap_vis(m_octree, m_markerPub);
   }

  if (publishMarkerArrayLowerRes){
	publish_octomap_vis(m_octree_lower_res, m_markerLowerResPub);
  }


  if (publishFullMap)
    publishFullOctoMap(rostime);


  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

}


bool OctomapServer::octomapBinarySrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(*m_octree, res.map))
    return false;

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool OctomapServer::octomapFullSrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();


  if (!octomap_msgs::fullMapToMsg(*m_octree, res.map))
    return false;

  return true;
}

bool OctomapServer::maxRangecb(octomap_server::maxRangeSrv::Request& req, octomap_server::maxRangeSrv::Response& resp){
  this->m_maxRange = req.max_range; 
  ROS_INFO_STREAM("changed max_range to "<<this->m_maxRange); 
  return true;
}



/*
void OctomapServer::PCMetaDataCb(mavbench_msgs::point_cloud_meta_data msg) {
	point_cloud_estimated_volume = msg.point_cloud_volume_to_digest;
	pc_res = msg.point_cloud_resolution;
  //ROS_INFO_STREAM("exposed volume"<<exposed_volume);
}
*/


void OctomapServer::OctomapHeaderColDetectedcb(std_msgs::Int32ConstPtr header) {
    /* 
    for (auto el : header_timeStamp_vec) {
        if (header.data == el) {
            ROS_INFO_STREAM("ptcloud recieved for collision detection "<< e.time_stamp);
        }
        head_timeStamp_vec.clear();
    }
    
    ROS_INFO_STREAM("COOOOOOOOOOOOOOOOOOOOL");
    */
}

/*
bool OctomapServer::changeResolution(octomap::point3d bbxMin, octomap::point3d bbxMax, int treeDepth){
	auto boundingBoxMinKey = m_octree->coordToKey(bbxMin);
	auto boundingBoxMaxKey = m_octree->coordToKey(bbxMax);
	auto offsetX = -boundingBoxMinKey[0];
	auto offsetY = -boundingBoxMinKey[1];
	auto offsetZ = -boundingBoxMinKey[2];

	int _sizeX = boundingBoxMaxKey[0] - boundingBoxMinKey[0] + 1;
	int _sizeY = boundingBoxMaxKey[1] - boundingBoxMinKey[1] + 1;
	int _sizeZ = boundingBoxMaxKey[2] - boundingBoxMinKey[2] + 1;

	//initializeEmpty(_sizeX, _sizeY, _sizeZ, false);


	 for(typename OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(bbxMin,bbxMax), end=m_octree->end_leafs_bbx(); it!= end; ++it){
		 if(m_octree->isNodeOccupied(*it)){
			 int nodeDepth = it.getDepth();
			 if( nodeDepth == treeDepth){
				 insertMaxDepthLeafAtInitialize(it.getKey());
			 } else {
				 int cubeSize = 1 << (treeDepth - nodeDepth);
				 octomap::OcTreeKey key=it.getIndexKey();
				 for(int dx = 0; dx < cubeSize; dx++)
					 for(int dy = 0; dy < cubeSize; dy++)
						 for(int dz = 0; dz < cubeSize; dz++){
							 unsigned short int tmpx = key[0]+dx;
							 unsigned short int tmpy = key[1]+dy;
							 unsigned short int tmpz = key[2]+dz;

							 if(boundingBoxMinKey[0] > tmpx || boundingBoxMinKey[1] > tmpy || boundingBoxMinKey[2] > tmpz)
								 continue;
							 if(boundingBoxMaxKey[0] < tmpx || boundingBoxMaxKey[1] < tmpy || boundingBoxMaxKey[2] < tmpz)
								 continue;

							 insertMaxDepthLeafAtInitialize(octomap::OcTreeKey(tmpx, tmpy, tmpz));
						 }
			 }
		 }
	 }
}
*/


bool OctomapServer::clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp){
  point3d min = pointMsgToOctomap(req.min);
  point3d max = pointMsgToOctomap(req.max);

  double thresMin = m_octree->getClampingThresMin();
  for(OcTreeT::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min,max),
      end=m_octree->end_leafs_bbx(); it!= end; ++it){

    it->setLogOdds(octomap::logodds(thresMin));
    //			m_octree->updateNode(it.getKey(), -6.0f);
  }
  // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
  m_octree->updateInnerOccupancy();

  publishAll(ros::Time::now());

  return true;
}




bool OctomapServer::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth +1);
  ros::Time rostime = ros::Time::now();
  m_octree->clear();
  // clear 2D map:
  m_gridmap.data.clear();
  m_gridmap.info.height = 0.0;
  m_gridmap.info.width = 0.0;
  m_gridmap.info.resolution = 0.0;
  m_gridmap.info.origin.position.x = 0.0;
  m_gridmap.info.origin.position.y = 0.0;

  ROS_INFO("Cleared octomap");
  publishAll(rostime);

  publishBinaryOctoMap(rostime);
  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){

    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  m_markerPub.publish(occupiedNodesVis);

  visualization_msgs::MarkerArray freeNodesVis;
  freeNodesVis.markers.resize(m_treeDepth +1);

  for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){

    freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
    freeNodesVis.markers[i].header.stamp = rostime;
    freeNodesVis.markers[i].ns = "map";
    freeNodesVis.markers[i].id = i;
    freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  m_fmarkerPub.publish(freeNodesVis);

  return true;
}


template <class OctomapT>
static inline bool binaryMapToMsgModified(const OctomapT& octomap, Octomap& msg, double &volume_communicated_in_unit_cubes, double resolution =0, int depth_limit = 0){
	if (depth_limit == 0){
		depth_limit =  octomap.getTreeDepth();
	}
	if (resolution == 0){
		resolution = octomap.getResolution();
	}
	msg.resolution = resolution; //octomap.getResolution();
	msg.id = octomap.getTreeType();
	msg.binary = true;

	std::stringstream datastream;
	if (!octomap.writeBinaryData(datastream, depth_limit, volume_communicated_in_unit_cubes))
		return false;



	std::string datastring = datastream.str();
	msg.data = std::vector<int8_t>(datastring.begin(), datastring.end());

	return true;
}


// filter octomap based On Volume publishing
void OctomapServer::publishFilteredByVolumeBinaryOctoMap(const ros::Time& rostime, point3d sensorOrigin) {

  int lower_res_map_depth = m_octree->getTreeDepth() - int(log2(om_to_pl_res/m_res));
  float volume_to_communicate = 0;
  Octomap map;
  map.header.frame_id = m_worldFrameId;
  profiling_container.capture("octomap_filtering_time", "start", ros::Time::now(), capture_size);
 // ros::param::get("/om_to_pl_vol_ideal", om_to_pl_vol_ideal);
//  ros::param::get("/om_to_pl_res", om_to_pl_res);
  //ros::param::get("/ppl_vol_ideal", ppl_vol_ideal);
  assert(om_to_pl_res >= m_res);


//  float potential_volume_to_explore = PotentialVolumeToExplore;
  int depth_to_look_at = 16;

  // -- put the original sensorOrigin in the shrunk_tree
  OcTreeT* m_octree_shrunk = new OcTreeT(m_octree->getResolution());
  m_octree_shrunk->setProbHit(m_octree->getProbHit());
  m_octree_shrunk->setProbMiss(m_octree->getProbMiss());
  m_octree_shrunk->setClampingThresMin(m_octree->getClampingThresMin());
  m_octree_shrunk->setClampingThresMax(m_octree->getClampingThresMax());
  auto key = m_octree->coordToKey(originalSensorOrigin);
  auto m_octree_node = m_octree->search(key);
  bool node_occupancy;
  if (!m_octree_node){
	 node_occupancy = false;
  }else{
	  node_occupancy =  m_octree->isNodeOccupied(m_octree_node);
  }

  m_octree_shrunk->updateNode(m_octree->coordToKey(originalSensorOrigin), node_occupancy);


  key = m_octree->coordToKey(sensorOrigin + point3d(10,10,10));
  int depth_found_at = 0;
  unsigned int pos;
  unsigned int pos_to_include;
  OcTreeNode* m_octree_node_to_include;
  while(depth_to_look_at > 0){
	  //m_octree_node = m_octree->search(key, depth_to_look_at);
	  m_octree_node = m_octree->search_with_pos_and_depth_return_(key, pos, depth_found_at, depth_to_look_at);
	  if (m_octree_node){ // node exist
		  depth_to_look_at = depth_found_at;
		  float block_volume = m_octree_node->getVolumeInUnitCube();
		  if (block_volume < om_to_pl_vol_ideal){
//			  depth_to_look_at --;
			  volume_to_communicate = block_volume;
			  m_octree_node_to_include = m_octree_node;
			  pos_to_include = pos;
		  }else{
			  depth_to_look_at ++;
			  break; // went over the limit
		  }
	  }
	  else{
		  depth_to_look_at --;
	  }
  }
  int parent_depth_found_at = 0;
  unsigned int parent_pos_found_at = 0;
  
  OcTreeNode* m_octree_shrunk_node;
  //int x = 10;
  //auto m_octree_block = m_octree->search_with_pos_and_depth_return_(key, pos, depth_found_at, x);
  //m_octree_shrunk_node = m_octree_shrunk->updateNode(key, m_octree_block->getValue(), true); // inject a node
  //auto m_octree_shrunk_block_parent = m_octree_shrunk->search_with_pos_and_depth_return_(key, parent_pos_found_at, parent_depth_found_at, x-1); //parent
  //m_octree_shrunk->setChild(m_octree_shrunk_block_parent, m_octree_block, pos);

  m_octree_shrunk->updateNode(key, m_octree_node_to_include->getValue(), true);

  if (depth_to_look_at != 0) {
	  auto m_octree_shrunk_block_parent = m_octree_shrunk->search_with_pos_and_depth_return_(key, parent_pos_found_at, parent_depth_found_at, depth_to_look_at); // get the parent node
	  if (parent_depth_found_at != depth_to_look_at - 1){ //this is a sanity check to make sure we don't encounter situations where the depth requested is different than
		  //acquired
		  ROS_ERROR_STREAM("parent should be right above child");
	  }
	  m_octree_shrunk->setChild(m_octree_shrunk_block_parent, m_octree_node_to_include, pos_to_include);
  }else{
	  m_octree_shrunk = m_octree;
  }
  profiling_container.capture("octomap_filtering_time", "end", ros::Time::now(), capture_size);


  profiling_container.capture("octomap_filtering_time", "end", ros::Time::now(), capture_size);
  double volume_communicated_in_unit_cubes = 0;
  // serialize
  if (binaryMapToMsgModified(*m_octree_shrunk, map, volume_communicated_in_unit_cubes, m_octree->getResolution(), lower_res_map_depth)){
	  int serialization_length = ros::serialization::serializationLength(map);
	  profiling_container.capture("octomap_serialization_load_in_BW", "single", (double) serialization_length, capture_size);
	  map.header.stamp = rostime;
	  octomap_aug_data.header.stamp = rostime;
	  octomap_aug_data.oct = map;
	  //octomap_aug_data.blah = -1;
	  m_binaryMapPub.publish(octomap_aug_data);
  }
  else
    ROS_ERROR("Error serializing OctoMap");
  auto total_volume = volume_communicated_in_unit_cubes*pow(om_to_pl_res, 3);

  debug_data.octomap_volume_communicated =    total_volume;
  profiling_container.capture(std::string("octomap_potential_exploration_resolution"), "single", om_to_pl_res, capture_size); // @suppress("Invalid arguments")
  profiling_container.capture(std::string("octomap_potential_exploration_volume"), "single", total_volume, capture_size); // @suppress("Invalid arguments")
}


// filter octomap before publishing based on volume
// we use a sampling method to make sure that we continously include the map
void OctomapServer::publishFilteredByVolumeBySamplingBinaryOctoMap(const ros::Time& rostime, point3d sensorOrigin) {

  ros::param::get("/MapToTransferSideLength", MapToTransferSideLength);
 // ros::param::get("/om_to_pl_res", om_to_pl_res);
//  ros::param::get("/om_to_pl_vol_ideal", om_to_pl_vol_ideal);
  //ros::param::get("/ppl_vol_ideal", ppl_vol_ideal);


  assert(om_to_pl_res >= m_res);
  unsigned int pos;
  int depth_found_at;
  // take care of space gridding for filtering octomap
  int grid_coeff;
  if (gridMode == "2d"){ grid_coeff = 2;}
  else{grid_coeff = 3;}

 // -- determine teh side length to transfer
 MapToTransferSideLength  = 1.2*m_maxRange; // -- 20  percent more
 vector<point3d> offset_vals;
 //double potential_volume_to_keep = PotentialVolumeToExplore;
 double last_om_to_pl_vol_actual = 0; // -- to rewind back the volume kept to
 	 	 	 	 	 	 	  // -- previously held value, after exceeding the threshold

 vector<OcTreeNode *> octomap_block_vec;
 double volume_increment; // -- volume to add per iteration

 // ros::Duration(10).sleep();
 // -- max_length is max of dimension and twice the sensor range. I want to have a bit more
 // -- left over for max_length to include all the voxels (since gridifying can cause looinsg
 // -- a chunk
 float map_max_length = max(max(max(tree_max_x, tree_max_y), tree_max_z), (float)(1.2*m_maxRange));

 bool first_itr = true;
 // -- expand MapToTransferSideLength untill you hit the volume target
 while (MapToTransferSideLength <= max(map_max_length, (float) (1.2*m_maxRange))){
	 om_to_pl_vol_actual = 0;
	 octomap_block_vec.clear();

	 // -- gridify the space up to MapToTransferSideLength
	 // -- and see if we can cover up to "volume_to_keep"
	 gridSideLength = float(MapToTransferSideLength)/gridSliceCountPerSide;
	 gridSliceCountToInclude =  int(pow(2, grid_coeff)*pow(gridSliceCountPerSide, grid_coeff)); //9(2d grid), 27 (3d grid)
	 MapToTransferBorrowedDepth = m_octree->getTreeDepth() - int(log2(gridSideLength/m_res)) - 1;
	 generateOffSets(offset_vals, gridSideLength, gridSideLength/4, gridSliceCountToInclude, gridMode) ;
	 for (auto it = offset_vals.begin(); it != offset_vals.end(); it++) {
		 point3d offset_point = *it;
		 auto point_to_consider = sensorOrigin + offset_point;
		 auto key = m_octree->coordToKey(point_to_consider);
		 auto m_octree_leaf_node = m_octree->search(key);
		 auto m_octree_block = m_octree->search_with_pos_and_depth_return_(key, pos, depth_found_at, MapToTransferBorrowedDepth);
		 if (!m_octree_block){
			continue;
		 }else{
			 if (std::find(octomap_block_vec.begin(), octomap_block_vec.end(),m_octree_block) !=octomap_block_vec.end()) {
				 continue;
			 }
			 om_to_pl_vol_actual +=  (m_octree_block->getVolumeInUnitCube()*pow(m_octree->getResolution(), 3));
			 octomap_block_vec.push_back(m_octree_block);
		 }
	 }
	 // -- if exceed the threshold, break
	 if (om_to_pl_vol_actual > om_to_pl_vol_ideal){
		 if (!first_itr){ // -- if first time, keep the lenght. We should set the volume to keep high enough
			 	 	 	   // --- that the first time always is smaller or equal.
			 om_to_pl_vol_actual = last_om_to_pl_vol_actual;
			 MapToTransferSideLength /= 2;
		 }
		 break;
	 }
	 first_itr = false;
	 last_om_to_pl_vol_actual = om_to_pl_vol_actual;
	 MapToTransferSideLength *= 2;
 }

 ROS_INFO_STREAM("volume kept" << om_to_pl_vol_actual);

 MapToTransferSideLength = min(MapToTransferSideLength, map_max_length); // -- incase we exceeded the threshold when doubling
 gridSideLength = float(MapToTransferSideLength)/(4*gridSliceCountPerSide); // -- to be more accurate, we quadruple the number of slices
 gridSliceCountToInclude =  int(pow(2, grid_coeff)*pow(4*gridSliceCountPerSide, grid_coeff)); //9(2d grid), 27 (3d grid)
 MapToTransferBorrowedDepth = m_octree->getTreeDepth() - int(log2(gridSideLength/m_res)) - 1;

  // -- now that we found the MapToTrasnferSideLength associated with the volume target
  // --  sample and add to the shrunk tree
  int lower_res_map_depth = m_octree->getTreeDepth() - int(log2(om_to_pl_res/m_res));
  Octomap map;
  map.header.frame_id = m_worldFrameId;
  //map.header.stamp = rostime;
  profiling_container.capture("octomap_filtering_time", "start", ros::Time::now(), capture_size);
  // create a smaller tree (comparing to the tree maintained for octomap), to reduce the communication (serialization, OM->MP and deserealization) overhead
  OcTreeT* m_octree_shrunk = new OcTreeT(m_octree->getResolution());
  m_octree_shrunk->setProbHit(m_octree->getProbHit());
  m_octree_shrunk->setProbMiss(m_octree->getProbMiss());
  m_octree_shrunk->setClampingThresMin(m_octree->getClampingThresMin());
  m_octree_shrunk->setClampingThresMax(m_octree->getClampingThresMax());
  //m_octree_shrunk->s(m_octree->getClampingThresMakx());

  // insert the original points (added to the main octomap) so that
  // both trees have the same origin

  auto key = m_octree->coordToKey(originalSensorOrigin);
  auto m_octree_node = m_octree->search(key);
  bool node_occupancy;
  if (!m_octree_node){
	 node_occupancy = false;
  }else{
	  node_occupancy =  m_octree->isNodeOccupied(m_octree_node);
  }
  m_octree_shrunk->updateNode(m_octree->coordToKey(originalSensorOrigin), node_occupancy);
  int parentBorrowedDepth = MapToTransferBorrowedDepth - 1;
  generateOffSets(offset_vals, gridSideLength, gridSideLength/4, gridSliceCountToInclude, gridMode) ;
  vector<point3d> point_to_consider_vec;
  //ros::Duration(10).sleep();
  // iteratve and add slices TODO: possibly add a prune, but probably not necessary
  for (auto it = offset_vals.begin(); it != offset_vals.end(); it++) {
      point3d offset_point = *it;

      auto point_to_consider = sensorOrigin + offset_point;
      if (point_to_consider.z() <= m_occupancyMinZ) {
    	  point_to_consider.z() = m_occupancyMinZ + m_res;
      }else if (point_to_consider.z() >= m_occupancyMaxZ) {
    	  point_to_consider.z() = m_occupancyMaxZ - m_res;
      }

     /*
      if (std::find(point_to_consider_vec.begin(), point_to_consider_vec.end(),point_to_consider)!=point_to_consider_vec.end()) {
    //	  ROS_ERROR_STREAM("already inclucded the point");
    	  continue;
      }
	*/

      point_to_consider_vec.push_back(point_to_consider);

      auto key = m_octree->coordToKey(point_to_consider);
	  auto m_octree_block = m_octree->search_with_pos_and_depth_return_(key, pos, depth_found_at, MapToTransferBorrowedDepth);
	  auto m_octree_leaf_node = m_octree->search(key);
	  if (!m_octree_block){
		  continue;
	  }
	  // now we just insert a leaf (of depth max) as a place holder to replace later
	  // we delete this node later
	  OcTreeNode* m_octree_shrunk_node;
	  if (!m_octree_leaf_node){ //if the node doesn't exist, add it
		  m_octree_shrunk_node = m_octree_shrunk->updateNode(key, false, false);
	  }else{
		  m_octree_shrunk_node = m_octree_shrunk->updateNode(key, m_octree_leaf_node->getValue(), true);
	  }

	  //ROS_INFO_STREAM(sizeof(*m_octree_shrunk_node));
	  //float ok = m_octree_shrunk_node->to_delete;


  	 //m_octree_shrunk->updateNode(key, m_octree_leaf_node->getValue(), true);

	  int parent_depth_found_at;
	  unsigned int parent_pos_found_at;
	  auto m_octree_shrunk_block_parent = m_octree_shrunk->search_with_pos_and_depth_return_(key, parent_pos_found_at, parent_depth_found_at, parentBorrowedDepth); //parent

	  if (parent_depth_found_at != depth_found_at - 1){ //this is a sanity check to make sure we don't encounter situations where the depth requested is different than
		  	  	  	  	  	  	  	  	  	  	  	    //acquired
		  ROS_ERROR_STREAM("parent should be right above child");
	  }

	  m_octree_shrunk->setChild(m_octree_shrunk_block_parent, m_octree_block, pos);
	  //delete m_octree_shrunk_node;
  }

  profiling_container.capture("octomap_filtering_time", "end", ros::Time::now(), capture_size);
  double volume_communicated_in_unit_cubes = 0;
  // serialize
  profiling_container.capture("octomap_serialization_time", "start", ros::Time::now(), capture_size);
  if (binaryMapToMsgModified(*m_octree_shrunk, map, volume_communicated_in_unit_cubes, m_octree->getResolution(), lower_res_map_depth)){
	  int serialization_length = ros::serialization::serializationLength(map);
	  profiling_container.capture("octomap_serialization_load_in_BW", "single", (double) serialization_length, capture_size);
	  map.header.stamp = rostime;

	  octomap_aug_data.header.stamp = rostime;
	  octomap_aug_data.oct = map;
	  octomap_aug_data.ee_profiles.actual_cmds.om_to_pl_vol = om_to_pl_vol_actual;
	  octomap_aug_data.ee_profiles.actual_time.om_latency = (ros::Time::now()  - pc_capture_time).toSec();
	  octomap_aug_data.ee_profiles.actual_time.om_pre_pub_time_stamp =  ros::Time::now();
	  m_binaryMapPub.publish(octomap_aug_data);
  }
  else
    ROS_ERROR("Error serializing OctoMap");
  auto total_volume = volume_communicated_in_unit_cubes*pow(om_to_pl_res, 3);
  profiling_container.capture("octomap_serialization_time", "end", ros::Time::now(), capture_size);
  octomap_aug_data.ee_profiles.actual_time.om_serialization_time =  profiling_container.findDataByName("octomap_serialization_time")->values.back();

  debug_data.octomap_volume_communicated =    total_volume;
  debug_data.octomap_serialization_time =  profiling_container.findDataByName("octomap_serialization_time")->values.back();
  profiling_container.capture(std::string("octomap_potential_exploration_resolution"), "single", om_to_pl_res, capture_size); // @suppress("Invalid arguments")
  profiling_container.capture(std::string("octomap_potential_exploration_volume"), "single", total_volume, capture_size); // @suppress("Invalid arguments")
}




// filter octomap before publishing
void OctomapServer::publishFilteredBinaryOctoMap(const ros::Time& rostime, point3d sensorOrigin) {

  ros::param::get("/MapToTransferSideLength", MapToTransferSideLength);
  ros::param::get("/om_to_pl_res", om_to_pl_res);


  assert(om_to_pl_res >= m_res);
  int lower_res_map_depth = m_octree->getTreeDepth() - int(log2(om_to_pl_res/m_res));


  // take care of space gridding for filtering octomap
  int grid_coeff;
  if (gridMode == "2d"){ grid_coeff = 2;}
  else{grid_coeff = 3;}
  gridSideLength = float(MapToTransferSideLength)/gridSliceCountPerSide;
  gridSliceCountToInclude =  int(pow(2, grid_coeff)*pow(gridSliceCountPerSide, grid_coeff)); //9(2d grid), 27 (3d grid)
  MapToTransferBorrowedDepth = m_octree->getTreeDepth() - int(log2(gridSideLength/m_res)) - 1;

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  //map.header.stamp = rostime;
  profiling_container.capture("octomap_filtering_time", "start", ros::Time::now(), capture_size);
  // create a smaller tree (comparing to the tree maintained for octomap), to reduce the communication (serialization, OM->MP and deserealization) overhead
  OcTreeT* m_octree_shrunk = new OcTreeT(m_octree->getResolution());
  m_octree_shrunk->setProbHit(m_octree->getProbHit());
  m_octree_shrunk->setProbMiss(m_octree->getProbMiss());
  m_octree_shrunk->setClampingThresMin(m_octree->getClampingThresMin());
  m_octree_shrunk->setClampingThresMax(m_octree->getClampingThresMax());
  //m_octree_shrunk->s(m_octree->getClampingThresMakx());

  // insert the original points (added to the main octomap) so that
  // both trees have the same origin

  auto key = m_octree->coordToKey(originalSensorOrigin);
  auto m_octree_node = m_octree->search(key);
  bool node_occupancy;
  if (!m_octree_node){
	 node_occupancy = false;
  }else{
	  node_occupancy =  m_octree->isNodeOccupied(m_octree_node);
  }

  m_octree_shrunk->updateNode(m_octree->coordToKey(originalSensorOrigin), node_occupancy);
  int parentBorrowedDepth = MapToTransferBorrowedDepth - 1;

  vector<point3d> offset_vals;
  generateOffSets(offset_vals, gridSideLength, gridSideLength/4, gridSliceCountToInclude, gridMode) ;
  vector<point3d> point_to_consider_vec;
  //ros::Duration(10).sleep();
  // iteratve and add slices TODO: possibly add a prune, but probably not necessary
  for (auto it = offset_vals.begin(); it != offset_vals.end(); it++) {
      point3d offset_point = *it;

      auto point_to_consider = sensorOrigin + offset_point;
      if (point_to_consider.z() <= m_occupancyMinZ) {
    	  point_to_consider.z() = m_occupancyMinZ + m_res;
      }else if (point_to_consider.z() >= m_occupancyMaxZ) {
    	  point_to_consider.z() = m_occupancyMaxZ - m_res;
      }


     /*
      if (std::find(point_to_consider_vec.begin(), point_to_consider_vec.end(),point_to_consider)!=point_to_consider_vec.end()) {
    //	  ROS_ERROR_STREAM("already inclucded the point");
    	  continue;
      }
	*/

      point_to_consider_vec.push_back(point_to_consider);

      auto key = m_octree->coordToKey(point_to_consider);
	  unsigned int pos;
	  int depth_found_at;
	  auto m_octree_block = m_octree->search_with_pos_and_depth_return_(key, pos, depth_found_at, MapToTransferBorrowedDepth);
	  auto m_octree_leaf_node = m_octree->search(key);
	  if (!m_octree_block){
		  continue;
	  }




	  // now we just insert a leaf (of depth max) as a place holder to replace later
	  // we delete this node later
	  OcTreeNode* m_octree_shrunk_node;
	  if (!m_octree_leaf_node){ //if the node doesn't exist, add it
		  m_octree_shrunk_node = m_octree_shrunk->updateNode(key, false, false);
	  }else{
		  m_octree_shrunk_node = m_octree_shrunk->updateNode(key, m_octree_leaf_node->getValue(), true);
	  }

	  //ROS_INFO_STREAM(sizeof(*m_octree_shrunk_node));
	  //float ok = m_octree_shrunk_node->to_delete;


  	 //m_octree_shrunk->updateNode(key, m_octree_leaf_node->getValue(), true);

	  int parent_depth_found_at;
	  unsigned int parent_pos_found_at;
	  auto m_octree_shrunk_block_parent = m_octree_shrunk->search_with_pos_and_depth_return_(key, parent_pos_found_at, parent_depth_found_at, parentBorrowedDepth); //parent

	  if (parent_depth_found_at != depth_found_at - 1){ //this is a sanity check to make sure we don't encounter situations where the depth requested is different than
		  	  	  	  	  	  	  	  	  	  	  	    //acquired
		  ROS_ERROR_STREAM("parent should be right above child");
	  }

	  m_octree_shrunk->setChild(m_octree_shrunk_block_parent, m_octree_block, pos);
	  //delete m_octree_shrunk_node;
  }

  profiling_container.capture("octomap_filtering_time", "end", ros::Time::now(), capture_size);
  double volume_communicated_in_unit_cubes = 0;
  // serialize
  if (binaryMapToMsgModified(*m_octree_shrunk, map, volume_communicated_in_unit_cubes, m_octree->getResolution(), lower_res_map_depth)){
	  int serialization_length = ros::serialization::serializationLength(map);
	  profiling_container.capture("octomap_serialization_load_in_BW", "single", (double) serialization_length, capture_size);
	  map.header.stamp = rostime;

	  octomap_aug_data.header.stamp = rostime;
	  octomap_aug_data.oct = map;
	  octomap_aug_data.ee_profiles.actual_cmds.om_to_pl_vol = om_to_pl_vol_actual;
	  octomap_aug_data.ee_profiles.actual_time.om_latency = (ros::Time::now()  - pc_capture_time).toSec();
	  octomap_aug_data.ee_profiles.actual_time.om_pre_pub_time_stamp =  ros::Time::now();
	  m_binaryMapPub.publish(octomap_aug_data);
  }
  else
    ROS_ERROR("Error serializing OctoMap");
  auto total_volume = volume_communicated_in_unit_cubes*pow(om_to_pl_res, 3);

  debug_data.octomap_volume_communicated =    total_volume;
  profiling_container.capture(std::string("octomap_potential_exploration_resolution"), "single", om_to_pl_res, capture_size); // @suppress("Invalid arguments")
  profiling_container.capture(std::string("octomap_potential_exploration_volume"), "single", total_volume, capture_size); // @suppress("Invalid arguments")
}


void OctomapServer::publishBinaryOctoMap(const ros::Time& rostime) {
  Octomap map;
  map.header.frame_id = m_worldFrameId;

  //map.header.stamp = rostime;
  //map.header.stamp = ros::Time::now();



  // publish part of the tree
  if (octomap_msgs::binaryMapToMsg(*m_octree, map)){
	  int serialization_length = ros::serialization::serializationLength(map);
//	  ROS_INFO_STREAM("serialization length is:"<< serialization_length);
	  profiling_container.capture("octomap_serialization_load_in_BW", "single", (double) serialization_length, capture_size);
	  map.header.stamp = rostime;

	  octomap_aug_data.header.stamp = rostime;
	  octomap_aug_data.oct = map;
	  octomap_aug_data.ee_profiles.actual_cmds.om_to_pl_vol = om_to_pl_vol_actual;
	  octomap_aug_data.ee_profiles.actual_time.om_latency = (ros::Time::now()  - pc_capture_time).toSec();
	  octomap_aug_data.ee_profiles.actual_time.om_pre_pub_time_stamp =  ros::Time::now();
	  //octomap_aug_data.blah = -1;

	  m_binaryMapPub.publish(octomap_aug_data);
  }
  else
    ROS_ERROR("Error serializing OctoMap");
}

void OctomapServer::publishBinaryLowerResOctoMap(const ros::Time& rostime) {

    Octomap map;
  map.header.frame_id = m_worldFrameId;

  map.header.stamp = rostime;
  //map.header.stamp = ros::Time::now();


  if (octomap_msgs::binaryMapToMsg(*m_octree_lower_res, map)){
	  int serialization_length = ros::serialization::serializationLength(map);
//	  ROS_INFO_STREAM("serialization length is:"<< serialization_length);
	  profiling_container.capture("octomap_serialization_low_res_load_in_BW", "single", (double) serialization_length, capture_size);
	  m_binaryMapLowerResPub.publish(map);
  }
  else
    ROS_ERROR("Error serializing OctoMap");
}


void OctomapServer::publish_octomap_vis(octomap::OcTree *m_octree_, ros::Publisher& om_pub){
   size_t octomapSize = m_octree_->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }
  //double m_treeDepth = 10;//m_octree_->getTreeDepth();
  double m_treeDepth = m_octree_->getTreeDepth();

  // init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth+1);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  typedef octomap::OcTree OcTreeT;

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = m_octree_->begin(m_treeDepth),
      end = m_octree_->end(); it != end; ++it)
  {
    bool inUpdateBBX = true;

    bool publish_voxel = voxel_type_to_publish == "free" ? !m_octree_->isNodeOccupied(*it):  m_octree_->isNodeOccupied(*it);
    if (publish_voxel){
      double z = it.getZ();
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
        /*
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
        */
        int r = 0;
        int g = 0;
        int b = 0;

        // Ignore speckles in the map:
        //create marker:
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree_->getMetricMin(minX, minY, minZ);
            m_octree_->getMetricMax(maxX, maxY, maxZ);

            //double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            std_msgs::ColorRGBA color;
            color.a = 1.0; color.r = 1; color.g = 1; color.b = 1;
            occupiedNodesVis.markers[idx].colors.push_back(color);
        }
  }

    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octree_->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = "world";
      occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      //occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    om_pub.publish(occupiedNodesVis);
}


void OctomapServer::publishFullOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    m_fullMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");

}


void OctomapServer::filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const{
  ground.header = pc.header;
  nonground.header = pc.header;

  if (pc.size() < 50){
    ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
    nonground = pc;
  } else {
    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<PCLPoint> seg;
    seg.setOptimizeCoefficients (true);
    // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold (m_groundFilterDistance);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(m_groundFilterAngle);


    PCLPointCloud cloud_filtered(pc);
    // Create the filtering object
    pcl::ExtractIndices<PCLPoint> extract;
    bool groundPlaneFound = false;

    while(cloud_filtered.size() > 10 && !groundPlaneFound){
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){
        ROS_INFO("PCL segmentation did not find any plane.");

        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative (false);
        extract.filter (ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          PCLPointCloud cloud_out;
          extract.filter(cloud_out);
          nonground += cloud_out;
          cloud_filtered = cloud_out;
        }

        groundPlaneFound = true;
      } else{
        ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        pcl::PointCloud<PCLPoint> cloud_out;
        extract.setNegative (false);
        extract.filter(cloud_out);
        nonground +=cloud_out;
        // debug
        //            pcl::PCDWriter writer;
        //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        } else{
          cloud_filtered.points.clear();
        }
      }

    }
    // TODO: also do this if overall starting pointcloud too small?
    if (!groundPlaneFound){ // no plane found or remaining points too small
      ROS_WARN("No ground plane found in scan");

      // do a rough fitlering on height to prevent spurious obstacles
      pcl::PassThrough<PCLPoint> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
      second_pass.setInputCloud(pc.makeShared());
      second_pass.filter(ground);

      second_pass.setFilterLimitsNegative (true);
      second_pass.filter(nonground);
    }

    // debug:
    //        pcl::PCDWriter writer;
    //        if (pc_ground.size() > 0)
    //          writer.write<PCLPoint>("ground.pcd",pc_ground, false);
    //        if (pc_nonground.size() > 0)
    //          writer.write<PCLPoint>("nonground.pcd",pc_nonground, false);

  }


}

void OctomapServer::handlePreNodeTraversal(const ros::Time& rostime){
  if (m_publish2DMap){
    // init projected 2D map:
    m_gridmap.header.frame_id = m_worldFrameId;
    m_gridmap.header.stamp = rostime;
    nav_msgs::MapMetaData oldMapInfo = m_gridmap.info;

    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
    octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);

    ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

    // add padding if requested (= new min/maxPts in x&y):
    double halfPaddedX = 0.5*m_minSizeX;
    double halfPaddedY = 0.5*m_minSizeY;
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    minPt = octomap::point3d(minX, minY, minZ);
    maxPt = octomap::point3d(maxX, maxY, maxZ);

    OcTreeKey paddedMaxKey;
    if (!m_octree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)){
      ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }
    if (!m_octree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)){
      ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }

    ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
    m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
    m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;

    int mapOriginX = minKey[0] - m_paddedMinKey[0];
    int mapOriginY = minKey[1] - m_paddedMinKey[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
    double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
    m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
    m_gridmap.info.resolution = gridRes;
    m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
    m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;
    if (m_maxTreeDepth != m_treeDepth){
      m_gridmap.info.origin.position.x -= m_res/2.0;
      m_gridmap.info.origin.position.y -= m_res/2.0;
    }

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (m_maxTreeDepth < m_treeDepth)
      m_projectCompleteMap = true;


    if(m_projectCompleteMap){
      ROS_DEBUG("Rebuilding complete 2D map");
      m_gridmap.data.clear();
      // init to unknown:
      m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

    } else {

       if (mapChanged(oldMapInfo, m_gridmap.info)){
          ROS_DEBUG("2D grid map size changed to %dx%d", m_gridmap.info.width, m_gridmap.info.height);
          adjustMapData(m_gridmap, oldMapInfo);
       }
       nav_msgs::OccupancyGrid::_data_type::iterator startIt;
       size_t mapUpdateBBXMinX = std::max(0, (int(m_updateBBXMin[0]) - int(m_paddedMinKey[0]))/int(m_multires2DScale));
       size_t mapUpdateBBXMinY = std::max(0, (int(m_updateBBXMin[1]) - int(m_paddedMinKey[1]))/int(m_multires2DScale));
       size_t mapUpdateBBXMaxX = std::min(int(m_gridmap.info.width-1), (int(m_updateBBXMax[0]) - int(m_paddedMinKey[0]))/int(m_multires2DScale));
       size_t mapUpdateBBXMaxY = std::min(int(m_gridmap.info.height-1), (int(m_updateBBXMax[1]) - int(m_paddedMinKey[1]))/int(m_multires2DScale));

       assert(mapUpdateBBXMaxX > mapUpdateBBXMinX);
       assert(mapUpdateBBXMaxY > mapUpdateBBXMinY);

       size_t numCols = mapUpdateBBXMaxX-mapUpdateBBXMinX +1;

       // test for max idx:
       uint max_idx = m_gridmap.info.width*mapUpdateBBXMaxY + mapUpdateBBXMaxX;
       if (max_idx  >= m_gridmap.data.size())
         ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]", max_idx, m_gridmap.data.size(), m_gridmap.info.width, m_gridmap.info.height, mapUpdateBBXMinX, mapUpdateBBXMinY, mapUpdateBBXMaxX, mapUpdateBBXMaxY);

       // reset proj. 2D map in bounding box:
       for (unsigned int j = mapUpdateBBXMinY; j <= mapUpdateBBXMaxY; ++j){
          std::fill_n(m_gridmap.data.begin() + m_gridmap.info.width*j+mapUpdateBBXMinX,
                      numCols, -1);
       }

    }



  }

}

void OctomapServer::handlePostNodeTraversal(const ros::Time& rostime){

  if (m_publish2DMap)
    m_mapPub.publish(m_gridmap);
}

void OctomapServer::handleOccupiedNode(const OcTreeT::iterator& it){

  if (m_publish2DMap && m_projectCompleteMap){
    update2DMap(it, true);
  }
}

void OctomapServer::handleFreeNode(const OcTreeT::iterator& it){

  if (m_publish2DMap && m_projectCompleteMap){
    update2DMap(it, false);
  }
}

void OctomapServer::handleOccupiedNodeInBBX(const OcTreeT::iterator& it){

  if (m_publish2DMap && !m_projectCompleteMap){
    update2DMap(it, true);
  }
}

void OctomapServer::handleFreeNodeInBBX(const OcTreeT::iterator& it){

  if (m_publish2DMap && !m_projectCompleteMap){
    update2DMap(it, false);
  }
}

void OctomapServer::update2DMap(const OcTreeT::iterator& it, bool occupied){

  // update 2D map (occupied always overrides):

  if (it.getDepth() == m_maxTreeDepth){
    unsigned idx = mapIdx(it.getKey());
    if (occupied)
      m_gridmap.data[mapIdx(it.getKey())] = 100;
    else if (m_gridmap.data[idx] == -1){
      m_gridmap.data[idx] = 0;
    }

  } else{
    int intSize = 1 << (m_maxTreeDepth - it.getDepth());
    octomap::OcTreeKey minKey=it.getIndexKey();
    for(int dx=0; dx < intSize; dx++){
      int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
      for(int dy=0; dy < intSize; dy++){
        unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/m_multires2DScale);
        if (occupied)
          m_gridmap.data[idx] = 100;
        else if (m_gridmap.data[idx] == -1){
          m_gridmap.data[idx] = 0;
        }
      }
    }
  }


}



bool OctomapServer::isSpeckleNode(const OcTreeKey&nKey) const {
  OcTreeKey key;
  bool neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        if (key != nKey){
          OcTreeNode* node = m_octree->search(key);
          if (node && m_octree->isNodeOccupied(node)){
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return neighborFound;
}

void OctomapServer::reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level){
	if (m_maxTreeDepth != unsigned(config.max_depth))
    m_maxTreeDepth = unsigned(config.max_depth);
  else{
	m_pointcloudMinZ            = config.pointcloud_min_z;
    m_pointcloudMaxZ            = config.pointcloud_max_z;
    m_occupancyMinZ             = config.occupancy_min_z;
    m_occupancyMaxZ             = config.occupancy_max_z;
    m_filterSpeckles            = config.filter_speckles;
    m_filterGroundPlane         = config.filter_ground;
    m_compressMap               = config.compress_map;
    m_incrementalUpdate         = config.incremental_2D_projection;

    // Parameters with a namespace require an special treatment at the beginning, as dynamic reconfigure
    // will overwrite them because the server is not able to match parameters' names.
    if (m_initConfig){
		// If parameters do not have the default value, dynamic reconfigure server should be updated.
		if(!is_equal(m_groundFilterDistance, 0.04))
          config.ground_filter_distance = m_groundFilterDistance;
		if(!is_equal(m_groundFilterAngle, 0.15))
          config.ground_filter_angle = m_groundFilterAngle;
	    if(!is_equal( m_groundFilterPlaneDistance, 0.07))
          config.ground_filter_plane_distance = m_groundFilterPlaneDistance;
        if(!is_equal(m_maxRange, -1.0))
          config.sensor_model_max_range = m_maxRange;
        if(!is_equal(m_octree->getProbHit(), 0.7))
          config.sensor_model_hit = m_octree->getProbHit();
	    if(!is_equal(m_octree->getProbMiss(), 0.4))
          config.sensor_model_miss = m_octree->getProbMiss();
		if(!is_equal(m_octree->getClampingThresMin(), 0.12))
          config.sensor_model_min = m_octree->getClampingThresMin();
		if(!is_equal(m_octree->getClampingThresMax(), 0.97))
          config.sensor_model_max = m_octree->getClampingThresMax();
        m_initConfig = false;

	    boost::recursive_mutex::scoped_lock reconf_lock(m_config_mutex);
        m_reconfigureServer.updateConfig(config);
    }
    else{
	  m_groundFilterDistance      = config.ground_filter_distance;
      m_groundFilterAngle         = config.ground_filter_angle;
      m_groundFilterPlaneDistance = config.ground_filter_plane_distance;
      m_maxRange                  = config.sensor_model_max_range;
      m_octree->setClampingThresMin(config.sensor_model_min);
      m_octree->setClampingThresMax(config.sensor_model_max);

     // Checking values that might create unexpected behaviors.
      if (is_equal(config.sensor_model_hit, 1.0))
		config.sensor_model_hit -= 1.0e-6;
      m_octree->setProbHit(config.sensor_model_hit);
	  if (is_equal(config.sensor_model_miss, 0.0))
		config.sensor_model_miss += 1.0e-6;
      m_octree->setProbMiss(config.sensor_model_miss);
	}
  }
  publishAll();
}

void OctomapServer::adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const{
  if (map.info.resolution != oldMapInfo.resolution){
    ROS_ERROR("Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off = int((oldMapInfo.origin.position.x - map.info.origin.position.x)/map.info.resolution +0.5);
  int j_off = int((oldMapInfo.origin.position.y - map.info.origin.position.y)/map.info.resolution +0.5);

  if (i_off < 0 || j_off < 0
      || oldMapInfo.width  + i_off > map.info.width
      || oldMapInfo.height + j_off > map.info.height)
  {
    ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  nav_msgs::OccupancyGrid::_data_type oldMapData = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  nav_msgs::OccupancyGrid::_data_type::iterator fromStart, fromEnd, toStart;

  for (int j =0; j < int(oldMapInfo.height); ++j ){
    // copy chunks, row by row:
    fromStart = oldMapData.begin() + j*oldMapInfo.width;
    fromEnd = fromStart + oldMapInfo.width;
    toStart = map.data.begin() + ((j+j_off)*m_gridmap.info.width + i_off);
    copy(fromStart, fromEnd, toStart);

//    for (int i =0; i < int(oldMapInfo.width); ++i){
//      map.data[m_gridmap.info.width*(j+j_off) +i+i_off] = oldMapData[oldMapInfo.width*j +i];
//    }

  }

}


std_msgs::ColorRGBA OctomapServer::heightMapColor(double h) {

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}
}



