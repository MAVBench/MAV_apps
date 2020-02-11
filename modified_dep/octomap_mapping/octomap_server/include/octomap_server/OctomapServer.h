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

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_H
#define OCTOMAP_SERVER_OCTOMAPSERVER_H
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <octomap_server/maxRangeSrv.h>
#include <std_msgs/Int32.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/CollisionMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>
#include <mavbench_msgs/octomap_debug.h>
#include <mavbench_msgs/point_cloud_meta_data.h>
#include <mavbench_msgs/octomap_aug.h>
#include <mavbench_msgs/point_cloud_aug.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include "profile_manager/start_profiling_srv.h"
#include "profile_manager/profiling_data_srv.h"
#include <profile_manager/profiling_data_verbose_srv.h>
#include <profile_manager.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <datacontainer.h>
#include <ros/callback_queue.h>
//#define COLOR_OCTOMAP_SERVER // turned off here, turned on identical ColorOctomapServer.h - easier maintenance, only maintain OctomapServer and then copy and paste to ColorOctomapServer and change define. There are prettier ways to do this, but this works for now

#ifdef COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif
namespace octomap_server {
class OctomapServer {

public:
  ros::CallbackQueue callback_queue_1; // -- queues for m_nh
  ros::CallbackQueue callback_queue_2; // -- queues for private_nh

//  ros::NodeHandle private_nh;//("~");
	ros::CallbackQueue callback_queue_meta_data; // -- only meta_data
  double point_cloud_estimated_volume; // -- the estimated volume of the information coming from point cloud
 double exposed_resolution; // -- the resolution of the information coming from point cloud
 std::string voxel_type_to_publish;
 DataContainer profiling_container;
 ProfileManager my_profile_manager;
 long long octomap_integration_acc;
 long long pt_cld_octomap_commun_overhead_acc; 
 int octomap_ctr;
 bool measure_time_end_to_end;
 ros::Time rcvd_point_cld_time_stamp;
 ros::Publisher inform_pc_done_pub;

 //void log_data_before_shutting_down();
 void sigIntHandlerPrivate(int);
 long long g_pt_cld_to_octomap_commun_olverhead_acc;
 int capture_size = 600;

 //typedef typename OcTreeT::leaf_bbx_iterator leaf_itr;

#ifdef COLOR_OCTOMAP_SERVER
  typedef pcl::PointXYZRGB PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
  typedef octomap::ColorOcTree OcTreeT;
#else
  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef octomap::OcTree OcTreeT;
#endif
  typedef octomap_msgs::GetOctomap OctomapSrv;
  typedef octomap_msgs::BoundingBoxQuery BBXSrv;

  OcTreeT *tree_ptr() const { return m_octree; }

  mavbench_msgs::octomap_debug debug_data;
  mavbench_msgs::octomap_aug octomap_aug_data;

  bool DEBUG_RQT;
  bool knob_performance_modeling;
  bool knob_performance_modeling_for_om_to_pl = false;
  bool knob_performance_modeling_for_om_to_pl_no_interference = false;  // -- it turnes of insertPoint cloud to get rid of the computational overhead of point cloud insertion

  OctomapServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
  //ros::NodeHandle private_nh_2;
  virtual ~OctomapServer();
  void spinOnce();
  virtual bool octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  virtual bool octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
  void publish_octomap_vis(octomap::OcTree *m_octree, ros::Publisher& om_pub); //for now only publishes for lower_res octomap, i.e, the publishing topic is fixed


  //bool changeResolution(octomap::point3d bbxMin, octomap::point3d bbxMax);
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
  void OctomapHeaderColDetectedcb(std_msgs::Int32ConstPtr header) ;
  void PCMetaDataCb(mavbench_msgs::point_cloud_meta_data header) ;
  bool maxRangecb(octomap_server::maxRangeSrv::Request& req, octomap_server::maxRangeSrv::Response& resp);
  virtual void insertCloudCallback(const mavbench_msgs::point_cloud_aug::ConstPtr& pcl_aug_data);
  virtual bool openFile(const std::string& filename);
  void log_data_before_shutting_down();



protected:
  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
    for (unsigned i = 0; i < 3; ++i)
      min[i] = std::min(in[i], min[i]);
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };

  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
    return (key[0] + voxelWidth >= m_updateBBXMin[0]
            && key[1] + voxelWidth >= m_updateBBXMin[1]
            && key[0] <= m_updateBBXMax[0]
            && key[1] <= m_updateBBXMax[1]);
  }

  void reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level);
  void publishBinaryOctoMap(const ros::Time& rostime = ros::Time::now()) ;
  void publishFilteredBinaryOctoMap(const ros::Time& rostime, octomap::point3d sensorOrigin);
  void publishFilteredByVolumeBinaryOctoMap(const ros::Time& rostime, octomap::point3d sensorOrigin);
  void publishFilteredByVolumeBySamplingBinaryOctoMap(const ros::Time& rostime, octomap::point3d sensorOrigin);
  void publishFilteredBinaryOctoMapTesting(const ros::Time& rostime, octomap::point3d sensorOrigin);

  void publishBinaryLowerResOctoMap(const ros::Time& rostime = ros::Time::now()) ;
  void publishFullOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  //virtual void publishAll(const ros::Time& rostime = ros::Time::now(), octomap::point3d = octomap::point3d(0,0,0));
  virtual void publishAll(const ros::Time& rostime = ros::Time::now());
  octomap::point3d sensorOrigin_; // helps avoid having to overleading the publishAll function by remembering sensorOrigin (TODO: overload later)
  octomap::point3d originalSensorOrigin; // register the first sensorOrigin to ensure
  	  	  	  	  	  	  	  	  	  	 // bothe shrunk and normal tree has the same original point
  bool first_time_scanning = true; //used to register originalSensorOrigin (only once obviously)

  /**
  * @brief update occupancy map with a scan labeled as ground and nonground.
  * The scans should be in the global map frame.
  *
  * @param sensorOrigin origin of the measurements for raycasting
  * @param ground scan endpoints on the ground plane (only clear space)
  * @param nonground all other endpoints (clear up to occupied endpoint)
  */
  virtual void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

  /// label the input cloud "pc" into ground and nonground. Should be in the robot's fixed frame (not world!)
  void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const;

  /**
  * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
  * @param key
  * @return
  */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  /// hook that is called before traversing all nodes
  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
  virtual void handleNode(const OcTreeT::iterator& it) {};

  // expand the non_max depth tree leafs and push their coordinates into a vector
  void gen_coordinates_to_consider(const OcTreeT::leaf_bbx_iterator &it, OcTreeT* cur_octree, std::vector<octomap::point3d> &coord_vec);
  void construct_diff_res_multiple_of_two_map(double diff_res, OcTreeT* m_octree_temp);
  void construct_diff_res_non_multiple_of_two_map(double diff_res, OcTreeT* m_octree_temp);


  /// hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
  virtual void handleNodeInBBX(const OcTreeT::iterator& it) {};

  /// hook that is called when traversing occupied nodes of the updated Octree
  virtual void handleOccupiedNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing occupied nodes in the updated area (updates 2D map projection here)
  virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes of the updated Octree
  virtual void handleFreeNode(const OcTreeT::iterator& it);


  double calcTreeVolume(OcTreeT* tree);

  /// hook that is called when traversing free nodes in the updated area (updates 2D map projection here)
  virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  inline unsigned mapIdx(int i, int j) const {
    return m_gridmap.info.width * j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
    return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                  (key[1] - m_paddedMinKey[1]) / m_multires2DScale);

  }


  inline double calc_dist(octomap::point3d point1, octomap::point3d point2);
  /**
   * Adjust data of map due to a change in its info properties (origin or size,
   * resolution needs to stay fixed). map already contains the new map info,
   * but the data is stored according to oldMapInfo.
   */

  void construct_diff_res_map(double resolution, octomap::point3d drone_cur_pos); //construct a lower resolution map from a higher one


  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;
  void SaveMapCb(std_msgs::Bool msg);


  void update_lower_res_map(octomap::point3d , octomap::OcTreeNode* node);
  inline void update_closest_obstacle(octomap::point3d coordinate, octomap::point3d sensorOrigin); //update the cloests obstacle distance and coordinates. It's conservate (read description in .cpp)

  inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo) {
    return (    oldMapInfo.height != newMapInfo.height
                || oldMapInfo.width != newMapInfo.width
                || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
  }

  static std_msgs::ColorRGBA heightMapColor(double h);
  ros::NodeHandle m_nh;
  ros::Publisher  m_markerPub, m_markerLowerResPub, m_binaryMapPub, m_binaryMapLowerResPub, m_fullMapPub, m_pointCloudPub, m_collisionObjectPub, m_mapPub, m_cmapPub, m_fmapPub, m_fmarkerPub, octomap_debug_pub, octomap_communication_proxy_msg;



  message_filters::Subscriber<mavbench_msgs::point_cloud_aug>* m_pointCloudSub;

 ros::Subscriber m_save_map_pub;
 bool m_save_map = false;

  tf::MessageFilter<mavbench_msgs::point_cloud_aug>* m_tfPointCloudSub;
  ros::ServiceServer m_octomapBinaryService, m_octomapFullService, m_clearBBXService, m_resetService, m_octomapResetMaxRange;
  ros::Subscriber m_octomapHeaderSub;  
  ros::Subscriber m_pc_meta_dataSub;

  tf::TransformListener m_tfListener;
  boost::recursive_mutex m_config_mutex;
  dynamic_reconfigure::Server<OctomapServerConfig> m_reconfigureServer;

  OcTreeT* m_octree;
  OcTreeT* m_octree_lower_res;
  ros::Time last_time_cleared;

octomap::KeyRay m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  double m_maxRange;
  std::string m_worldFrameId; // the map frame
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  bool m_useHeightMap;
  std_msgs::ColorRGBA m_color;
  std_msgs::ColorRGBA m_colorFree;
  double m_colorFactor;
  double dist_to_closest_obs; //distant to the closest obstacle perceived
  int depth_to_transfer;
  int MapToTransferGridSize; // how big the grid (side of the cubic grid) for sampling the octomap is
  float PotentialVolumeToExploreThreshold; // -- size of the map that is sent to the planner (hence, the map size potentially to explore)
  float VolumeToExploreThreshold;  // -- size of the map that the planner is allowed to plan through. todo: we shouldn't need this. we simply need to pass this from point cloud.
  int MapToTransferGridCount; // Number of grids (with the side size of MapToTransferGridSize) to include in the transfered map
  int MapToTransferBorrowedDepth; // the depth that is borrowed (attached) from the main octomap
  int closest_obs_coordinate_vect_size = 20;
  octomap::point3d closest_obs_coord; // closeset obstacle (coordinate) perceived.

  int gridSliceCountPerSide = 2; // how many gridslice per Map to transfer Side lenght (in other words, how many time should we slice the Side lenght). This is necessary since the space is gridded, so we
  	  	  	  	  	  	  	  	 // we can never exactly maintain the map side length, but rather we need to include some extra. Note that if we increase
  	  	  	  	  	  	  	     // this value, we can reduce the extra space overhead we need to carry over
  float MapToTransferSideLength;
  std::string gridMode; // 2d or 3d
  bool filterOctoMap = true; // if true, we grid the space and filter how much of the map to be communicated over to planner
  float gridSideLength; // the bigger this is, the more of the map we keep the map
  float gridSliceCountToInclude; // How many grid slices to include (centered around the current drone's position)


  float tree_max_x = 0;
  float tree_max_y = 0;
  float tree_max_z = 0;

  bool m_latchedTopics;
  bool m_publishFreeSpace;

  double m_res, m_lower_res;
  double m_lower_res_rel_vol_height, m_lower_res_rel_vol_width, m_lower_res_rel_vol_length; //width and height of the lower resolution with respect to the current drone position
  unsigned m_treeDepth;
  unsigned m_maxTreeDepth;

  double m_pointcloudMinX;
  double m_pointcloudMaxX;
  double m_pointcloudMinY;
  double m_pointcloudMaxY;
  double m_pointcloudMinZ;
  double m_pointcloudMaxZ;
  double m_occupancyMinZ;
  double m_occupancyMaxZ;
  double m_minSizeX;
  double m_minSizeY;
  bool m_filterSpeckles;



  bool m_filterGroundPlane;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;

  bool m_compressMap;

  bool m_initConfig;

  // downprojected 2D map:
  bool m_incrementalUpdate;
  nav_msgs::OccupancyGrid m_gridmap;
  bool m_publish2DMap;
  bool m_mapOriginChanged;
  octomap::OcTreeKey m_paddedMinKey;
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
  bool m_useColoredMap;

 // Profiling
 bool CLCT_DATA;
 int data_collection_iteration_freq; 
 ros::ServiceClient profile_manager_client;

};
}

#endif
