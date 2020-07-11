#include "motion_planner.h"

#include <coord.h>
#include <datat.h>
#include <Drone.h>
#include <Eigen/src/Core/Matrix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <octomap/math/Vector3.h>
#include <octomap/octomap_types.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeNode.h>
#include <octomap_msgs/Octomap.h>
#include <profile_manager/profiling_data_verbose_srv.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/message_forward.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <ros/subscribe_options.h>
#include <ros/this_node.h>
#include <ros/time.h>
#include <ros/transport_hints.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>
#include <mavbench_msgs/motion_planning_debug.h>
#include <mavbench_msgs/octomap_aug.h>
#include <common.h>
#include <mavbench_msgs/response_time_capture.h>
#include "../../deps/mav_comm/mav_msgs/include/mav_msgs/eigen_mav_msgs.h"
#include "../../deps/mav_trajectory_generation/mav_trajectory_generation/include/mav_trajectory_generation/impl/polynomial_optimization_linear_impl.h"
#include "../../deps/mav_trajectory_generation/mav_trajectory_generation/include/mav_trajectory_generation/motion_defines.h"
#include "../../deps/mav_trajectory_generation/mav_trajectory_generation/include/mav_trajectory_generation/segment.h"
#include "../../deps/mav_trajectory_generation/mav_trajectory_generation/include/mav_trajectory_generation/vertex.h"
#include "../../deps/mav_trajectory_generation/mav_trajectory_generation_ros/include/mav_trajectory_generation_ros/ros_visualization.h"
#include "../../deps/mav_trajectory_generation/mav_visualization/include/mav_visualization/helpers.h"
#include "../../deps/mav_trajectory_generation/mav_trajectory_generation/include/mav_trajectory_generation/motion_defines.h"
#include "filterqueue.h"
string clct_data_mode_;
double max_time_budget;
double follow_trajectory_loop_rate;
double follow_trajectory_worse_case_latency;
ros::Time ppl_start_time;
double SA_time_budget_to_enforce; // end to end budget
float g_ppl_time_budget;
double total_collision_func = 0;
double potential_distance_to_explore = 0; // the distance that the planner ask to expore (note that this can be bigger than the volume since the volume calculation stops after hitting a an obstacle)
double volume_explored_in_unit_cubes = 0; // volume explored within the piecewise planner
double ppl_vol_idealInUnitCube;
double volume_explored_in_unit_cube_last = 0;
int stagnation_ctr = 0;
ros::Time g_planning_start_time;
bool ppl_vol_maximum_underestimated;
ros::Time planning_start_time_stamp;
ros::Time deadline_setting_time; // time when image was captured
/*
void planner_termination_func(double &volume_explored_so_far, double &volume_explored_threshold){
	bool res = volume_explored_so_far > volume_explored_threshold;
	return res;
}
*/



bool planner_termination_func(){
	bool volume_explored_exceeded = (volume_explored_in_unit_cubes > ppl_vol_idealInUnitCube);
	bool time_exceeded = ((SA_time_budget_to_enforce - follow_trajectory_worse_case_latency) - (ros::Time::now() - deadline_setting_time).toSec()) < 0;  // whatever is left of the budget

	if (volume_explored_in_unit_cube_last < volume_explored_in_unit_cubes){
		volume_explored_in_unit_cube_last = volume_explored_in_unit_cubes;
		stagnation_ctr = 0;
	}else{
		stagnation_ctr++;
	}
	bool explored_new_teritory = stagnation_ctr < 20;
	if (explored_new_teritory){
		ppl_vol_maximum_underestimated = false;
	}
	if (time_exceeded){
		ROS_INFO_STREAM("----time exceeded");
	}
	return volume_explored_exceeded || time_exceeded || !explored_new_teritory;

}

bool planner_termination_func_knob_performance_modeling(){
	bool volume_explored_exceeded = (volume_explored_in_unit_cubes > ppl_vol_idealInUnitCube);
	//bool SA_time_exceeded = (SA_time_budget_to_enforce- (ros::Time::now() - deadline_setting_time).toSec()) < 0;  // whatever is left of the budget
	bool ppl_time_exceeded = max_time_budget - ((ros::Time::now() - ppl_start_time).toSec()) < 0;
	if (ppl_time_exceeded || volume_explored_exceeded){
		cout<<" unit cube"<<volume_explored_in_unit_cubes<<endl;
	}
	return  volume_explored_exceeded || ppl_time_exceeded;// || ppl_time_exceeded;
}


bool planner_termination_func_2(){
	//bool volume_explored_exceeded = (volume_explored_in_unit_cubes > ppl_vol_idealInUnitCube);
	bool SA_time_exceeded = ((SA_time_budget_to_enforce/2 - follow_trajectory_worse_case_latency) - (ros::Time::now() - deadline_setting_time).toSec()) < 0;  // whatever is left of the budget
	//bool ppl_time_exceeded = ((ros::Time::now() - ppl_start_time).toSec() > g_ppl_time_budget);
	return  SA_time_exceeded;// || ppl_time_exceeded;
}

template<class PlannerType>
MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_plan(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree, int& status)
{

   stagnation_ctr = 0;
	//publish_dummy_octomap_vis(octree);
	namespace ob = ompl::base;
    namespace og = ompl::geometric;

    piecewise_trajectory result;

    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // Set bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, std::min(x__low_bound__global, g_start_pos.x));
    bounds.setHigh(0, std::max(x__high_bound__global, g_start_pos.x));
    bounds.setLow(1, std::min(y__low_bound__global, g_start_pos.y));
    bounds.setHigh(1, std::max(y__high_bound__global, g_start_pos.y));
    bounds.setLow(2, std::min(z__low_bound__global, g_start_pos.z));
    bounds.setHigh(2, std::max(z__high_bound__global, g_start_pos.z));

    space->setBounds(bounds);

    og::SimpleSetup ss(space);


	bool SA_time_exceeded = ((SA_time_budget_to_enforce/2 - follow_trajectory_worse_case_latency) - (ros::Time::now() - deadline_setting_time).toSec()) < 0;  // whatever is left of the budget
	bool ppl_time_exceeded = ((ros::Time::now() - ppl_start_time).toSec() > g_ppl_time_budget);
	if (SA_time_exceeded){ //|| ppl_time_exceeded){
		status = 0;
		piecewise_planning = false;
		return result;
	}

    // Setup collision checker
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    si->setStateValidityChecker([this] (const ompl::base::State * state) {
        return this->OMPLStateValidityChecker(state);
    });
    si->setMotionValidator(std::make_shared<OMPLMotionValidator>(this, si));
    si->setup();

    // Set planner
    //auto blah =  new PlannerType(si) ;
    //blah->setApproximate(10.0);
    ob::PlannerPtr planner(new PlannerType(si));
    ss.setPlanner(planner);

    ob::ScopedState<> start_state(space);
    start_state[0] = start.x;
    start_state[1] = start.y;
    start_state[2] = start.z;

    ob::ScopedState<> goal_state(space);
    goal_state[0] = goal.x;
    goal_state[1] = goal.y;
    goal_state[2] = goal.z;

    double distance_to_goal = calc_vec_magnitude(start.x - goal.x, start.y - goal.y,  start.z - goal.z);
    double ideal_distance_to_goal = max(distance_to_goal/10, distance_to_goal_margin);
    ss.setStartAndGoalStates(start_state, goal_state, ideal_distance_to_goal);
    ss.setup();


	profiling_container.capture("OMPL_planning_time", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
    // Solve for path
    //ob::PlannerStatus solved = ss.solve(g_ppl_time_budget);

	piecewise_planning = true;
	auto planner_termination_obj = ompl::base::PlannerTerminationCondition(planner_termination_func_2);

	if (knob_performance_modeling){
		planner_termination_obj = ompl::base::PlannerTerminationCondition(planner_termination_func_knob_performance_modeling);
	}
	ob::PlannerStatus solved;



	SA_time_exceeded = ((SA_time_budget_to_enforce/2 - follow_trajectory_worse_case_latency) - (ros::Time::now() - deadline_setting_time).toSec()) < 0;  // whatever is left of the budget
	//ppl_time_exceeded = ((ros::Time::now() - ppl_start_time).toSec() > g_ppl_time_budget);
	if (SA_time_exceeded){// || ppl_time_exceeded){
		status = 0;
		piecewise_planning = false;
		return result;
	}

    //ROS_INFO_STREAM("insie before solved");
	solved = ss.solve(planner_termination_obj);
    ROS_INFO_STREAM("after solved");
	//if (knob_performance_modeling_for_piecewise_planner){
    //}else
	//ob::PlannerStatus solved = ss.solve(planner_termination_func(volume_explored_in_unit_cubes, ppl_vol_idealInUnitCube));
    if (solved == ob::PlannerStatus::INVALID_START) {
    	status = 2;
    }else if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION){
    	status = 3;
    	ROS_INFO_STREAM("APPPROXIMATE SOLUTION");
    }
    else if (solved == ob::PlannerStatus::EXACT_SOLUTION){
    	status = 1;
    } else {
    	status = 0;
    }
    piecewise_planning = false;

    // profiling, debugging
    profiling_container.capture("OMPL_planning_time", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
	if (DEBUG_RQT) {
		debug_data.header.stamp = ros::Time::now();
		debug_data.OMPL_planning_time = profiling_container.findDataByName("OMPL_planning_time")->values.back();
		motion_planning_debug_pub.publish(debug_data);
	}

	if (status == 1 || status == 3) //only take exact solution
    {

    	profiling_container.capture("OMPL_simplification_time", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
    	if (status ==1) {
    		ROS_INFO("Solution found Exactly!");
    	}else if (status == 3){
    		ROS_INFO("Solution found Approximately!");
    	}
        //ss.simplifySolution();

        for (auto state : ss.getSolutionPath().getStates()) {
            const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

            double x = pos->values[0];
            double y = pos->values[1];
            double z = pos->values[2];

            result.push_back({x, y, z});
        }

        //profiling/debugging
        profiling_container.capture("OMPL_simplification_time", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
    	if (DEBUG_RQT) {
    		debug_data.header.stamp  = ros::Time::now();
    		debug_data.OMPL_simplification_time = profiling_container.findDataByName("OMPL_simplification_time")->values.back();
    		motion_planning_debug_pub.publish(debug_data);
		}
    }
    else
        ROS_ERROR("Path not found!");

    return result;
}






MotionPlanner::MotionPlanner(octomap::OcTree * octree_, Drone * drone_):
        octree(octree_),
		drone(drone_),
		profile_manager("client", "/record_profiling_data", "/record_profiling_data_verbose")
		{


	vmax_filter_queue = new FilterQueue(3);
	motion_planning_initialize_params();
    g_goal_pos.x = g_goal_pos.y = g_goal_pos.z = nan("");
	// Create a new callback queue
	nh.setCallbackQueue(&callback_queue);

	// Topics and services
	get_trajectory_srv_server = nh.advertiseService("/get_trajectory_srv", &MotionPlanner::get_trajectory_fun, this);

	future_col_sub = nh.subscribe("/col_coming", 1, &MotionPlanner::future_col_callback, this);
	runtime_failure_sub = nh.subscribe("/runtime_failure", 1, &MotionPlanner::runtime_failure_cb, this);
	octomap_communication_proxy_msg = nh.subscribe("/octomap_communication_proxy_msg", 1, &MotionPlanner::octomap_communication_proxy_msg_cb, this);
	next_steps_sub = nh.subscribe("/next_steps", 1, &MotionPlanner::next_steps_callback, this);

	//octomap_sub = nh.subscribe("/octomap_binary", 1, &MotionPlanner::octomap_callback, this);
	octomap_sub = nh.subscribe("/octomap_binary", 1, &MotionPlanner::octomap_callback, this);

	traj_pub = nh.advertise<mavbench_msgs::multiDOFtrajectory>("multidoftraj", 1);
    timing_msg_from_mp_pub = nh.advertise<mavbench_msgs::response_time_capture> ("/timing_msgs_from_mp", 1);
    motion_planning_debug_pub = nh.advertise<mavbench_msgs::motion_planning_debug>("/motion_planning_debug", 1);

    smoothener_collision_marker_pub = nh.advertise<visualization_msgs::Marker>("smoothener_collision_marker", 1);//, connect_cb, connect_cb);


	// for stress testing
	octomap_dummy_pub = nh.advertise<octomap_msgs::Octomap>("octomap_binary_2", 2);

	// for visualizatrion (rviz)
	m_markerPub = nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array_dumy", 1);
	smooth_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
	piecewise_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
	collision_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("collision_waypoints", 1);
	closest_unknown_pub = nh.advertise<mavbench_msgs::planner_info>("closest_unknown_point", 1);
	goal_rcv_service = nh.advertiseService("goal_rcv", &MotionPlanner::goal_rcv_call_back, this);
	//re = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    if (knob_performance_modeling){
    	capture_size = 1;
    }

    time_budgetter = new TimeBudgetter(10.0, 10.0, accelerationCoeffs, 0.1, 10.0, drone_radius__global);  // non of the hardcoded values actually matter for motion planner

		}



Drone* MotionPlanner::get_drone() {
	return drone;
}

bool MotionPlanner::goal_rcv_call_back(package_delivery::point::Request &req, package_delivery::point::Response &res){
	ROS_INFO_STREAM("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ first planning false");
	first_time_planning_succeeded = false;
	g_goal_pos = req.goal;
	goal_known = true;
}

void MotionPlanner::publish_dummy_octomap_vis(octomap::OcTree *m_octree){
	ros::param::get("/voxel_type_to_publish", voxel_type_to_publish);
	size_t octomapSize = m_octree->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }
  double m_treeDepth = m_octree->getTreeDepth();
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
  for (OcTreeT::iterator it = m_octree->begin(),
      end = m_octree->end(); it != end; ++it)
  {
    bool inUpdateBBX = true;

    bool publish_voxel = voxel_type_to_publish == "free" ? !m_octree->isNodeOccupied(*it):  m_octree->isNodeOccupied(*it);
    if (voxel_type_to_publish == "all") {
    		publish_voxel =true;
    }



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
            m_octree->getMetricMin(minX, minY, minZ);
            m_octree->getMetricMax(maxX, maxY, maxZ);

            //double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            std_msgs::ColorRGBA color;


             if (voxel_type_to_publish == "free"){
            	color.a = .3; color.r = 0; color.g = 1; color.b = 0;
            }else if (voxel_type_to_publish == "occupied"){
            	color.a = .3; color.r = 1; color.g = 0; color.b = 0;
            }else if (voxel_type_to_publish == "all") {
            	if (m_octree->isNodeOccupied(*it)){
            		color.a = .3; color.r = 1; color.g = 0; color.b = 0;
            	}else{
            		color.a = .3; color.r = 0; color.g = 1; color.b = 0;
            	}
            }
//            color.a = 1; color.r = 1; color.g = 1; color.b = 1;
            occupiedNodesVis.markers[idx].colors.push_back(color);
        }
  }

    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octree->getNodeSize(i);

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

    m_markerPub.publish(occupiedNodesVis);
}


// debugging
void MotionPlanner::publish_dummy_octomap(octomap::OcTree *m_octree){
	octomap_msgs::Octomap_<std::allocator<void> > map;

	if (octomap_msgs::binaryMapToMsg(*m_octree, map)){
		octomap_dummy_pub.publish(map);
	}
	//publish_dummy_octomap_vis(m_octree);
}

double dist(coord t, graph::node m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.x)*(t.x-m.x) + (t.y-m.y)*(t.y-m.y) + (t.z-m.z)*(t.z-m.z));
}

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.x)*(t.x-m.x) + (t.y-m.y)*(t.y-m.y) + (t.z-m.z)*(t.z-m.z));
}

void MotionPlanner::runtime_failure_cb(const mavbench_msgs::runtime_failure_msg & msg){
	//res.multiDOFtrajectory.planning_status = Short_time_failure;
	mavbench_msgs::multiDOFtrajectory traj;
	traj.header.stamp = msg.header.stamp;
	traj.trajectory_seq = trajectory_seq_id;
	traj.planning_status = "piecewise_planning_failed";
    traj.reverse = false;
    traj.stop = true;
    traj.controls = msg.controls;
    traj.ee_profiles = msg.ee_profiles;
    trajectory_seq_id++;
    mavbench_msgs::planner_info closest_unknown_way_point;
    closest_unknown_way_point.x = nan("");
    closest_unknown_way_point.y = nan("");
    closest_unknown_way_point.z = nan("");
    closest_unknown_way_point.planning_status = "runtime_failure";
	closest_unknown_pub.publish(closest_unknown_way_point);
	ros::param::set("/set_closest_unknown_point", true);
	traj.closest_unknown_point = closest_unknown_way_point;
    traj_pub.publish(traj);
    runtime_failure_last_time = true;
}

double updated_budget_till_next_unknown;
bool got_updated_budget = false;


double calculate_budget(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg, std::deque<multiDOFpoint> *traj_);

// determine whether it's time to replan
bool MotionPlanner::shouldReplan(const octomap_msgs::Octomap& msg){
	bool replan;
	//std_msgs::Header msg_for_follow_traj;
    //mavbench_msgs::response_time_capture msg_for_follow_traj;
	msg_for_follow_traj.header.stamp = msg.header.stamp;

	auto cur_velocity = drone->velocity().linear;
    auto cur_vel_mag = calc_vec_magnitude(cur_velocity.x, cur_velocity.y, cur_velocity.z);
    auto time_diff_duration = (ros::Time::now() - dist_to_closest_obs_time_stamp).toSec();
    //double dist_to_closest_obs_recalculated =  dist_to_closest_obs - time_diff_duration*cur_vel_mag;
    double dist_to_closest_obs_recalculated =  dist_to_closest_obs;
    double renewed_v_max_temp = max(v_max_min + (dist_to_closest_obs_recalculated/sensor_max_range)*(v_max_max - v_max_min), v_max_min);
    //bool sub_optimal_v_max = (renewed_v_max >  (v_max__global+2) || (renewed_v_max <  (v_max__global- 2));
    vmax_filter_queue->push(renewed_v_max_temp);
    //cout<<"renewd_v_max_before"<<renewed_v_max<<endl;
    renewed_v_max = vmax_filter_queue->reduce("min");
    //cout<<"renewd_v_max_after"<<renewed_v_max<<endl;

    sub_optimal_v_max = (renewed_v_max >  1.5*v_max__global) || (renewed_v_max <  v_max__global/1.5);
    //bool sub_optimal_v_max = (renewed_v_max >  (v_max__global+1.4) || (renewed_v_max <  (v_max__global- 1.4)));
    //bool sub_optimal_v_max = false;

    planned_approximately = false; // for now setting to false, to see how much full approximate planning  following is effective
    bool SA_time_exceeded = ((SA_time_budget_to_enforce - follow_trajectory_worse_case_latency) - (ros::Time::now() - deadline_setting_time).toSec()) < 0;  // whatever is left of the budget
    if (SA_time_exceeded){
		replanning_reason = Collision_detected;
    	replan = true;
    }if(!first_time_planning_succeeded) {
		msg_for_follow_traj.closest_unknown_point.planning_status = "first_time_planning";
		//msg_for_follow_traj.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
		//msg_for_follow_traj.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
		//msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
		timing_msg_from_mp_pub.publish(msg_for_follow_traj); //send a msg to make sure we update response time
		replanning_reason = First_time_planning;
		replan = true;
	} else if (got_new_next_steps_since_last_attempted_plan){ // only can decide on replanning, if we have the new position of the drone on the track
		if (runtime_failure_last_time){
			replanning_reason = Runtime_failure;
			runtime_failure_last_time = false;;
			ROS_ERROR_STREAM("runtime failed last time , so replan");
			replan = true;
		}
		else if (failed_to_plan_last_time) {
		//ROS_ERROR_STREAM("failed to plan last time , so replan");
		replanning_reason = Failed_to_plan_last_time;
		replan = true;
		} else if (!planned_optimally && (ros::Time::now() - this->last_planning_time).toSec() < 3) {
			replan = true;
		}else if(planned_approximately){
			replanning_reason = Last_plan_approximate;
			replan = true;
			ROS_ERROR_STREAM("approximate planning");
		}else if (sub_optimal_v_max && budgetting_mode != "static"){
			replanning_reason = Last_plan_approximate;
			replan = true;
			ROS_ERROR_STREAM("sub_optimal v_max");
		}
		else if( (ros::Time::now() - this->last_planning_time).toSec() > (float)1/planner_min_freq) {
			replanning_reason = Min_freq_passed;
			//ROS_ERROR_STREAM("long time since last planning, so replan");
			replan = true;
		}
		else if (next_steps_msg_size == 0 && dist(drone->position(), g_goal_pos) > distance_to_goal_margin){
			ROS_INFO_STREAM("==============================================================         now it's zero----------------------------=================================");
			ROS_INFO_STREAM("==============================================================         now it's zero----------------------------================================="<< dist(drone->position(), g_goal_pos));
			ROS_INFO_STREAM("==============================================================         now ----------------------------=================================");
			ROS_INFO_STREAM("==============================================================         DONE ----------------------------=================================");
			replan = true;
		}
		else {
			profiling_container.capture("collision_check_for_replanning", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
			mavbench_msgs::planner_info closest_unknown_way_point;
			bool collision_coming = this->traj_colliding(&g_next_steps_msg, closest_unknown_way_point);
			profiling_container.capture("collision_check_for_replanning", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
			debug_data.collision_check_for_replanning = profiling_container.findDataByName("collision_check_for_replanning")->values.back();


			multiDOFpoint point_of_interest;
			if (collision_coming){ // use closest obstacle if there is a collision
				point_of_interest.x = closest_obstacle_on_path_way_point.x;
				point_of_interest.y = closest_obstacle_on_path_way_point.y;
				point_of_interest.z = closest_obstacle_on_path_way_point.z;
			}else{ // use closest unknown
				point_of_interest.x = closest_unknown_way_point.x;
				point_of_interest.y = closest_unknown_way_point.y;
				point_of_interest.z = closest_unknown_way_point.z;
			}

			multiDOFpoint cur_point;
			cur_point.x = drone->position().x;
			cur_point.y = drone->position().y;
			cur_point.z = drone->position().z;
			coord drone_pos;
			drone_pos.x  = cur_point.x;
			drone_pos.y  = cur_point.y;
			drone_pos.z  = cur_point.z;
			//updated_budget_till_next_unknown = time_budgetter->calc_budget_till_closest_unknown(cur_point, point_of_interest);

			std::deque<multiDOFpoint> traj;
			//updated_budget_till_next_unknown = calculate_budget(g_next_steps_msg, &traj);
			//double time_budget = time_budgetter->calc_budget(g_next_steps_msg, &traj, point_of_interest, drone_pos);
			//got_updated_budget = true;
			//bool got_enough_budget = (updated_budget_till_next_unknown > .6); //TODO change .6 to the front-end latency
			bool got_enough_budget = true;

			if (collision_coming || !got_enough_budget){
				replanning_reason = Collision_detected;
				//ROS_INFO_STREAM("there is a collision");
				profiling_container.capture("replanning_due_to_collision_ctr", "counter", 0, capture_size); // @suppress("Invalid arguments")
				//ROS_ERROR_STREAM("collision comming, so replan");
				ROS_ERROR_STREAM(" collision detected");
				replan = true;
			} else{ //this case is for profiling. We send this over to notify the follow trajectory that we made a decision not to plan
				replanning_reason = No_need_to_plan;
				replan = false;
				closest_unknown_way_point.planning_status = "no_planning_needed";
				closest_unknown_pub.publish(closest_unknown_way_point);
				ros::param::set("/set_closest_unknown_point", true);
				msg_for_follow_traj.closest_unknown_point = closest_unknown_way_point;
				ROS_ERROR_STREAM(" not collision detected");
			}
		}
	}


	// for profiling
	if (!replan) { //notify the follow trajectory to erase up to this msg
		msg_for_follow_traj.closest_unknown_point.planning_status = "no_planning_needed";
		ppl_vol_actual = volume_explored_in_unit_cubes*pow(map_res, 3);
		ppl_vol_unit_cube_actual = volume_explored_in_unit_cubes;
		msg_for_follow_traj.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
		msg_for_follow_traj.ee_profiles.actual_time.smoothening_latency = 0;
		msg_for_follow_traj.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
		msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
		msg_for_follow_traj.ee_profiles.space_stats.ppl_vol_maxium_underestimated = ppl_vol_maximum_underestimated;
		msg_for_follow_traj.ee_profiles.space_stats.ppl_vol_unit_cube =  ppl_vol_unit_cube_actual;
		msg_for_follow_traj.ee_profiles.control_flow_path = .5;
		timing_msg_from_mp_pub.publish(msg_for_follow_traj); //send a msg to make sure we update response time
	}else{
		profiling_container.capture("planning_count", "counter", 0, capture_size); // @suppress("Invalid arguments")
	}

	return replan;
}



// call back to register the octomap to motion planner communication overhead
void MotionPlanner::octomap_communication_proxy_msg_cb(const std_msgs::Header& msg)
{
	if (measure_time_end_to_end) { // if measuring end_to_end use the proxy msg to measure the overhead
		profiling_container.capture("octomap_to_motion_planner_communication_overhead", "single",
				(ros::Time::now() - msg.stamp).toSec(), capture_size);
    	debug_data.octomap_to_motion_planner_communication_overhead = profiling_container.findDataByName("octomap_to_motion_planner_communication_overhead")->values.back();
	}
}


// octomap callback
void MotionPlanner::octomap_callback(const mavbench_msgs::octomap_aug::ConstPtr& msg)
{
	got_updated_budget = false;
	ros::param::get("/v_max", v_max__global);
    ppl_vol_maximum_underestimated = true;
	msg_for_follow_traj.controls = msg->controls;
	msg_for_follow_traj.ee_profiles = msg->ee_profiles;
	msg_for_follow_traj.ee_profiles.actual_time.om_to_pl_ros_oh = (ros::Time::now() - msg->ee_profiles.actual_time.om_pre_pub_time_stamp).toSec();


	planner_consecutive_failure_ctr = msg->controls.inputs.planner_consecutive_failure_ctr;
	//g_ppl_time_budget = msg->ee_profiles.expected_time.ppl_latency;
	//g_smoothening_budget = msg->ee_profiles.expected_time.ppl_latency;  // -- for now setting it equal to ppl_budget
															   // -- todo: this can be another knob

	deadline_setting_time = msg->ee_profiles.actual_time.deadline_setting_time_stamp;
	SA_time_budget_to_enforce= msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce;

	 dist_to_closest_obs = msg->dist_to_closest_obs;
	 dist_to_closest_obs_time_stamp = msg->dist_to_closest_obs_time_stamp;

	// reset all the profiling values
	potential_distance_to_explore = 0;
    volume_explored_in_unit_cubes = 0;
    volume_explored_in_unit_cube_last = 0;
    debug_data.motion_planning_collision_check_volume_explored = 0;
    debug_data.motion_planning_piecewise_volume_explored = 0;
    debug_data.motion_planning_smoothening_volume_explored = 0;
    debug_data.motion_planning_piece_wise_time = 0;
    debug_data.collision_func = 0;

    ppl_vol_ideal = msg->controls.cmds.ppl_vol;


    map_res = msg->controls.cmds.om_to_pl_res;
    ppl_vol_idealInUnitCube = ppl_vol_ideal/(pow(msg->controls.cmds.om_to_pl_res, 3));
    //ppl_vol_idealInUnitCube = 4000;
    profiling_container.capture("entire_octomap_callback", "start", ros::Time::now(), capture_size);
    profiling_container.capture("ppl_vol_ideal", "single", msg->ee_profiles.actual_cmds.ppl_vol, capture_size);
    profiling_container.capture("om_to_pl_vol_actual", "single", msg->ee_profiles.actual_cmds.om_to_pl_vol, capture_size);
    profiling_container.capture("om_to_pl_res", "single", msg->controls.cmds.om_to_pl_res, capture_size);

    if (octree != nullptr) {
        delete octree;
	}
    //ROS_INFO_STREAM("octomap communication time"<<(ros::Time::now() - msg.header.stamp).toSec());
	if (measure_time_end_to_end){
		profiling_container.capture("sensor_to_motion_planner_time", "single",
				(ros::Time::now() - msg->header.stamp).toSec(), capture_size);
	}else{
		profiling_container.capture("octomap_to_motion_planner_communication_overhead", "single",
				(ros::Time::now() - msg->header.stamp).toSec(), capture_size);
	}


	//if(!measure_time_end_to_end) profiling_container.capture("octomap_to_motion_planner_communication_overhead","end", ros::Time::now());

	// -- deserialize the map
	profiling_container.capture("OMDeserializationLatency", "start", ros::Time::now(), capture_size);
    octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(msg->oct);
    profiling_container.capture("OMDeserializationLatency", "end", ros::Time::now(), capture_size);
    // -- cast the map
    profiling_container.capture("octomap_dynamic_casting", "start", ros::Time::now(), capture_size);
    octree = dynamic_cast<octomap::OcTree*> (tree);
    profiling_container.capture("octomap_dynamic_casting", "end", ros::Time::now(), capture_size);
//    octree->setResolution(msg->controls.cmds.om_to_pl_res):



    msg_for_follow_traj.ee_profiles.time_stats.OMtoPlComOHLatency = msg_for_follow_traj.ee_profiles.actual_time.om_to_pl_ros_oh;

    msg_for_follow_traj.ee_profiles.time_stats.OMDeserializationLatency = profiling_container.findDataByName("OMDeserializationLatency")->values.back() +
			profiling_container.findDataByName("octomap_dynamic_casting")->values.back();
    msg_for_follow_traj.ee_profiles.time_stats.OMtoPlTotalLatency = msg_for_follow_traj.ee_profiles.time_stats.OMtoPlComOHLatency + msg->ee_profiles.time_stats.OMSerializationLatency + msg_for_follow_traj.ee_profiles.time_stats.OMDeserializationLatency;

    msg_for_follow_traj.ee_profiles.actual_time.om_to_pl_latency =  msg->ee_profiles.actual_time.om_serialization_time +
    		msg_for_follow_traj.ee_profiles.actual_time.om_to_pl_ros_oh +
    		profiling_container.findDataByName("OMDeserializationLatency")->values.back() +
			profiling_container.findDataByName("octomap_dynamic_casting")->values.back();


    if (DEBUG_VIS){
    	publish_dummy_octomap_vis(octree);
    }

	planning_start_time_stamp = ros::Time::now();
    // -- purelly for knob_performance_modeling;
    // -- only collect data when the knob is set.
    // -- We'd like to have fine grain control over the collection so
    // -- as to only start the collection when octomap is not inserting
    // -- points into the point cloud (to avoid data contamination due to the
    // -- resources being used by octomap
	if (knob_performance_modeling){
		ros::param::get("/knob_performance_modeling_for_om_to_pl", knob_performance_modeling_for_om_to_pl);
		ros::param::get("/knob_performance_modeling_for_piecewise_planner", knob_performance_modeling_for_piecewise_planner);
		// -- om to pl
		if (knob_performance_modeling_for_om_to_pl){ // -- start collecting data when this is set
			profiling_container.capture("octomap_to_motion_planner_serialization_to_reception_knob_modeling", "single",
					msg_for_follow_traj.ee_profiles.time_stats.OMtoPlComOHLatency + msg->ee_profiles.time_stats.OMSerializationLatency + msg_for_follow_traj.ee_profiles.time_stats.OMDeserializationLatency, capture_size);
			profiling_container.capture("om_to_pl_res_knob_modeling", "single",
					msg->controls.cmds.om_to_pl_res, capture_size);
			profiling_container.capture("om_to_pl_vol_actual_knob_modeling", "single",
					msg->ee_profiles.actual_cmds.om_to_pl_vol, capture_size);
			profiling_container.capture("OMDeserializationLatency_knob_modeling", "single",
					profiling_container.findDataByName("OMDeserializationLatency")->values.back(), capture_size);
			profiling_container.capture("octomap_dynamic_casting_knob_modeling", "single",
					profiling_container.findDataByName("octomap_dynamic_casting")->values.back(), capture_size);
			double octomap_to_planner_com_overhead_knob_modeling = profiling_container.findDataByName("octomap_dynamic_casting_knob_modeling")->values.back() +
					profiling_container.findDataByName("OMDeserializationLatency_knob_modeling")->values.back() +
					profiling_container.findDataByName("octomap_to_motion_planner_serialization_to_reception_knob_modeling")->values.back();
			profiling_container.capture("octomap_to_planner_com_overhead_knob_modeling", "single",
					octomap_to_planner_com_overhead_knob_modeling, capture_size);
			debug_data.octomap_to_motion_planner_serialization_to_reception_knob_modeling = profiling_container.findDataByName("octomap_to_motion_planner_serialization_to_reception_knob_modeling")->values.back();
		}
		// -- piecewise planner
		else if (knob_performance_modeling_for_piecewise_planner){
				profiling_container.capture("om_to_pl_res_knob_modeling", "single",
					msg->controls.cmds.om_to_pl_res, capture_size);
		}
	}


    if (DEBUG_RQT) {
    		debug_data.header.stamp = ros::Time::now();
    		debug_data.OMDeserializationLatency = profiling_container.findDataByName("OMDeserializationLatency")->values.back();
    		if (!measure_time_end_to_end){
    			debug_data.octomap_to_motion_planner_communication_overhead = profiling_container.findDataByName("octomap_to_motion_planner_communication_overhead")->values.back();
    		}
    		//motion_planning_debug_pub.publish(debug_data);
    }

    // check whether the goal has already been provided or not, if not, return
    if (!this->goal_known) { //
    	return;
    }

    auto blah = octree->getResolution();
    ppl_start_time = ros::Time::now();
    if (!shouldReplan(msg->oct) && !knob_performance_modeling_for_piecewise_planner){
    	debug_data.motion_planning_collision_check_volume_explored = volume_explored_in_unit_cubes*pow(map_res, 3);
    	debug_data.motion_planning_potential_distance_to_explore = potential_distance_to_explore;
    	ROS_INFO_STREAM("don't need to replan");
    	return;
    }
    last_unknown_pt_ctr = 0;

    this->last_planning_time = ros::Time::now();
    g_planning_start_time = this->last_planning_time;



    // if already have a plan, but not colliding, plan again
    profiling_container.capture("motion_planning_time_total", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
    bool planning_succeeded = this->motion_plan_end_to_end(msg->header.stamp, g_goal_pos);
    profiling_container.capture("motion_planning_time_total", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
    /*
    if (unknown_budget_failed){
    	debug_data.motion_planning_collision_check_volume_explored = volume_explored_in_unit_cubes*pow(map_res, 3);
    	debug_data.motion_planning_potential_distance_to_explore = potential_distance_to_explore;
    	unknown_budget_failed = false;
    	return;
    }
    */
    //ROS_INFO_STREAM("motion_Planning_time_total"<<profiling_container.findDataByName("motion_planning_time_total")->values.back());

    if (knob_performance_modeling_for_piecewise_planner){
    	auto resolution = profiling_container.findDataByName("om_to_pl_res_knob_modeling")->values.back();
    	profiling_container.capture("ppl_res_knob_modeling", "single", resolution,
    			capture_size);
    	profiling_container.capture("piecewise_planner_time_knob_modeling", "single", profiling_container.findDataByName("motion_planning_piece_wise_time")->values.back(),
    			capture_size);
    	profiling_container.capture("ppl_vol_actual_knob_modeling", "single", ppl_vol_actual, capture_size);
    }


    if (!planning_succeeded) {
		profiling_container.capture("planning_failure_count", "counter", 0);
	}

    if (!first_time_planning_succeeded){
    	if (planning_succeeded){
    		first_time_planning_succeeded = true;
    	}
	}

    profiling_container.capture("entire_octomap_callback", "end", ros::Time::now(), capture_size);
    debug_data.motion_planning_total_time = profiling_container.findDataByName("motion_planning_time_total")->values.back();
    debug_data.octomap_dynamic_casting = profiling_container.findDataByName("octomap_dynamic_casting")->values.back();
    debug_data.entire_octomap_callback = profiling_container.findDataByName("entire_octomap_callback")->values.back();
    debug_data.motion_planning_potential_distance_to_explore = potential_distance_to_explore;
}

octomap::OcTree* MotionPlanner::getOctree() {
	return this->octree;
}

//*** F:DN getting the smoothened trajectory
bool MotionPlanner::get_trajectory_fun(package_delivery::get_trajectory::Request &req, package_delivery::get_trajectory::Response &res){
	ros::Duration ppl_latency_so_far;

	volume_explored_in_unit_cubes = 0; // reset the value for piecewise planning
	//-----------------------------------------------------------------
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    piecewise_trajectory piecewise_path;
	smooth_trajectory smooth_path;

    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 
    auto hook_end_t_2 = ros::Time::now(); 

	//ros::param::get("/ppl_time_budget", g_ppl_time_budget);
	//ros::param::get("/smoothening_budget", g_smoothening_budget);

    //auto hook_start_t = ros::Time::now();
    //g_start_pos = req.start;
    //g_goal_pos = req.goal;
    mavbench_msgs::planner_info closest_unknown_way_point;
    closest_unknown_way_point.x = nan("");
    closest_unknown_way_point.y = nan("");
    closest_unknown_way_point.z = nan("");


    //hard code values for now
    if (g_always_randomize_end_point || knob_performance_modeling_for_piecewise_planner){
    	g_goal_pos.x = rand() % int(x__high_bound__global);
    	g_goal_pos.y = rand() % int(y__high_bound__global);
    	g_goal_pos.z = rand() % int(z__high_bound__global);

    	req.goal.x = g_goal_pos.x;
    	req.goal.y = g_goal_pos.y;
    	req.goal.z = g_goal_pos.z;
    }
    /*
    //g_goal_pos.x = 60;
    //g_goal_pos.y = 60;
    //g_goal_pos.z = 5;
    req.goal.x = g_goal_pos.x;
    req.goal.y = g_goal_pos.y;
    req.goal.z = g_goal_pos.z;
	*/


    //publish_dummy_octomap_vis(octree);
    int status;

    total_collision_func =0;
    profiling_container.capture("motion_planning_piece_wise_time", "start", ros::Time::now(), capture_size);
    double last_time_planning_duration;

    if (got_updated_budget){
    	SA_time_budget_to_enforce = updated_budget_till_next_unknown;
    	ROS_INFO_STREAM("00000000000000000000000000000000000000000 updated_budget is "<< updated_budget_till_next_unknown);
    }

    g_ppl_time_budget = (SA_time_budget_to_enforce - (ros::Time::now() - deadline_setting_time).toSec())/3;  // distribute the rest of the budget between smoothening and ppl
	g_smoothening_budget = g_ppl_time_budget;

   //ROS_INFO_STREAM("--budget to impose on ppl"<< SA_time_budget_to_enforce/2);

    //bool time_exceeded = false;
    bool SA_time_exceeded = ((SA_time_budget_to_enforce/2 - follow_trajectory_worse_case_latency) - (ros::Time::now() - deadline_setting_time).toSec()) < 0;  // whatever is left of the budget

    bool ppl_time_exceeded = ((ros::Time::now() - ppl_start_time).toSec() > g_ppl_time_budget);
    if (SA_time_exceeded){// || ppl_time_exceeded){
    	status = 0;
    } else {
    //while((piecewise_path.empty() || status == 3 || !got_enough_budget_for_next_SA_itr(piecewise_path) || !ppl_inbound_check(piecewise_path))){
    	ros::Time this_time_ppl_start_time = ros::Time::now();
    	piecewise_path = motion_planning_core(req.start, req.goal, req.width, req.length, req.n_pts_per_dir, octree, status);
    	ppl_latency_so_far = (ros::Time::now() - ppl_start_time);
    	last_time_planning_duration = (ros::Time::now() - this_time_ppl_start_time).toSec();
    	//}
	}

    profiling_container.capture("motion_planning_piece_wise_time", "end", ros::Time::now(), capture_size);
    if (DEBUG_RQT) {
    		debug_data.header.stamp = ros::Time::now();
    		debug_data.motion_planning_piece_wise_time = profiling_container.findDataByName("motion_planning_piece_wise_time")->values.back();
    		//motion_planning_debug_pub.publish(debug_data);
    }


    debug_data.motion_planning_piecewise_volume_explored = volume_explored_in_unit_cubes*pow(map_res,3);
    debug_data.collision_func = total_collision_func;
    ppl_vol_actual = volume_explored_in_unit_cubes*pow(map_res, 3);
    ppl_vol_unit_cube_actual = volume_explored_in_unit_cubes;
    //ROS_INFO_STREAM("actuall volume explored"<< ppl_vol_actual << " volume explored in unit cubes"<< volume_explored_in_unit_cubes<<  "volume expected in unit cube"<<ppl_vol_idealInUnitCube << " volume expected"<< ppl_vol_ideal<< "octree res"<< map_res);
    //ROS_INFO_STREAM("status is "<< status);

    //ROS_INFO_STREAM("already flew backward"<<already_flew_backward);
    bool take_approximate = false;
    if (status == 3){
    	graph::node last_node = piecewise_path[-1];
    	//auto appx_distance_to_end_position = distance(last_node.x - drone->position().x, last_node.y - drone->position().y, last_node.z - drone->position().z);
    	auto appx_distance_to_end_position = distance(last_node.x - req.goal.x, last_node.y - req.goal.y, last_node.z - req.goal.z);
    	auto cur_distance_to_end_position = distance(req.goal.x - drone->position().x, req.goal.y - drone->position().y, req.goal.z - drone->position().z);
    	if (appx_distance_to_end_position < cur_distance_to_end_position){
    	 take_approximate = true;
    	}else{
    		cout<<"difference between approx and current distance"<< cur_distance_to_end_position - appx_distance_to_end_position<<endl;
    	}
    }
    if (piecewise_path.empty() || (status == 3 && !take_approximate) || (status == 0)) {
		msg_for_follow_traj.closest_unknown_point = closest_unknown_way_point;
		msg_for_follow_traj.closest_unknown_point.planning_status = "ppl_failed";
		closest_unknown_way_point.planning_status = "ppl_failed";
		closest_unknown_pub.publish(closest_unknown_way_point);
		ros::param::set("/set_closest_unknown_point", true);
		msg_for_follow_traj.ee_profiles.control_flow_path = 1;
    	profiling_container.capture("motion_planning_piecewise_failure_cnt", "counter", 0, capture_size); // @suppress("Invalid arguments")
    	if (notified_failure){ //so we won't fly backward multiple times
    		//std_msgs::Header msg_for_follow_traj;
    		if (!measure_time_end_to_end) { msg_for_follow_traj.header.stamp = ros::Time::now(); }
			else{ msg_for_follow_traj.header.stamp = req.header.stamp; }
			msg_for_follow_traj.planning_status = "piecewise_planning_failed";
			msg_for_follow_traj.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
			msg_for_follow_traj.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
			msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
    		msg_for_follow_traj.ee_profiles.space_stats.ppl_vol_maxium_underestimated = ppl_vol_maximum_underestimated;
			msg_for_follow_traj.ee_profiles.space_stats.ppl_vol_unit_cube = ppl_vol_unit_cube_actual;
			timing_msg_from_mp_pub.publish(msg_for_follow_traj); //send a msg to make sure we update responese timne
    		return false;
    	}

    	//ROS_ERROR("Empty path returned.status is");
    	//ROS_INFO_STREAM("Status is "<<status);
    	res.path_found = false;

        res.multiDOFtrajectory.future_collision_seq = future_col_seq_id;
        res.multiDOFtrajectory.trajectory_seq = trajectory_seq_id;
        trajectory_seq_id++;

        res.multiDOFtrajectory.append = false;
        if (status == 2) {
        	//res.multiDOFtrajectory.reverse = false;
        	res.multiDOFtrajectory.reverse = true;
        	res.multiDOFtrajectory.stop = false;
        	res.multiDOFtrajectory.planning_status = "piecewise_planning_failed";
        	ROS_INFO_STREAM("curent state, x:"<<drone->position().x<< ",  y:"<< drone->position().y<< ", z:"<< drone->position().z);
        }else if (status == 0 ){ //if couldn't find an exact path within the time frame reverse
        	//res.multiDOFtrajectory.planning_status = Short_time_failure;
        	res.multiDOFtrajectory.planning_status = "piecewise_planning_failed";
        	res.multiDOFtrajectory.reverse = false;
        	res.multiDOFtrajectory.stop = true;
        }
        else if ( status == 3){ //if couldn't find an exact path within the time frame reverse
        	//res.multiDOFtrajectory.planning_status = Short_time_failure;
        	res.multiDOFtrajectory.planning_status = "piecewise_planning_failed";
        	res.multiDOFtrajectory.reverse = false;
        	res.multiDOFtrajectory.stop = true;
        	draw_piecewise(piecewise_path, status, &piecewise_traj_markers);
        	piecewise_traj_vis_pub.publish(piecewise_traj_markers);
        }
        else{
        	cout<<status<<endl;
        	ROS_INFO_STREAM("this state shouldn't happpen"<< status);
        	exit(0);
        }

        notified_failure = true;
        if (!measure_time_end_to_end) { res.multiDOFtrajectory.header.stamp = ros::Time::now(); }
        else{ res.multiDOFtrajectory.header.stamp = req.header.stamp; }
        res.multiDOFtrajectory.closest_unknown_point = msg_for_follow_traj.closest_unknown_point;
        res.multiDOFtrajectory.planning_status = "piecewise_planning_failed";
        res.multiDOFtrajectory.controls = msg_for_follow_traj.controls;
        res.multiDOFtrajectory.ee_profiles = msg_for_follow_traj.ee_profiles;
        res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
        res.multiDOFtrajectory.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
        res.multiDOFtrajectory.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
    	res.multiDOFtrajectory.ee_profiles.space_stats.ppl_vol_maxium_underestimated = ppl_vol_maximum_underestimated;
        res.multiDOFtrajectory.ee_profiles.space_stats.ppl_vol_unit_cube = ppl_vol_unit_cube_actual;
        res.multiDOFtrajectory.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
        res.multiDOFtrajectory.ee_profiles.control_flow_path = 1;
        traj_pub.publish(res.multiDOFtrajectory);
        return false;
    }

    profiling_container.capture("RRT_path_length_normalized_to_direct_path", "single",
    		calculate_path_length(piecewise_path)/calc_vec_magnitude(drone->position().x - req.goal.x, drone->position().y - req.goal.y, drone->position().z - req.goal.z),
    		capture_size);

    /*
    if (motion_planning_core_str != "lawn_mower") {
    	profiling_container.capture("piecewise_path_post_process", "start", ros::Time::now(), capture_size);
    	postprocess(piecewise_path);
    	profiling_container.capture("piecewise_path_post_process", "end", ros::Time::now(), capture_size);
    }
	*/


    res.multiDOFtrajectory.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
    res.multiDOFtrajectory.ee_profiles.space_stats.ppl_vol_maxium_underestimated = ppl_vol_maximum_underestimated;
    res.multiDOFtrajectory.ee_profiles.space_stats.ppl_vol_unit_cube = ppl_vol_unit_cube_actual;
    res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
	msg_for_follow_traj.ee_profiles.space_stats.ppl_vol_unit_cube = ppl_vol_unit_cube_actual;
    msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
    msg_for_follow_traj.ee_profiles.space_stats.ppl_vol_maxium_underestimated = ppl_vol_maximum_underestimated;
    volume_explored_in_unit_cubes = 0;

    auto ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
    draw_piecewise(piecewise_path, status, &piecewise_traj_markers);
    piecewise_traj_vis_pub.publish(piecewise_traj_markers);

    // Smoothen the path and build the multiDOFtrajectory response
    //ROS_INFO("Smoothenning...");
    profiling_container.capture("motion_planning_smoothening_time", "start", ros::Time::now(), capture_size);
    /*
    auto cur_velocity = drone->velocity().linear;
    auto cur_vel_mag = calc_vec_magnitude(cur_velocity.x, cur_velocity.y, cur_velocity.z);
    auto time_diff_duration = (ros::Time::now() - dist_to_closest_obs_time_stamp).toSec();
    double renewed_v_max = min(max((dist_to_closest_obs - time_diff_duration*cur_vel_mag), 1.0), 6.0);
    ROS_INFO_STREAM("v_max ===="<< renewed_v_max);
    //ros::param::set("v_max", renewed_v_max);
    //v_max__global = renewed_v_max;
	*/
 	auto cur_velocity = drone->velocity().linear;
    auto cur_vel_mag = calc_vec_magnitude(cur_velocity.x, cur_velocity.y, cur_velocity.z);
    auto time_diff_duration = (ros::Time::now() - dist_to_closest_obs_time_stamp).toSec();
    double dist_to_closest_obs_recalculated =  max(dist_to_closest_obs - time_diff_duration*cur_vel_mag, 0.0);
    //double dist_to_closest_obs_recalculated =  max(dist_to_closest_obs *cur_vel_mag, 0.0);
    //double renewed_v_max = v_max_min + (dist_to_closest_obs_recalculated/sensor_max_range)*(v_max_max - v_max_min);
    //double renewed_v_max = max(v_max_min + (dist_to_closest_obs_recalculated/sensor_max_range)*(v_max_max - v_max_min), v_max_min);
    //bool sub_optimal_v_max = (renewed_v_max >  1.5*v_max__global) || (renewed_v_max <  v_max__global/1.5);
    //bool sub_optimal_v_max = (renewed_v_max >  (v_max__global+1.4) || (renewed_v_max <  (v_max__global- 1.4)));
    //ROS_INFO_STREAM("renewed_v_max:" << renewed_v_max << "  v_max__golbal (current v_max):"<<v_max__global << " dist_to_closest_obs:"<<dist_to_closest_obs);
    if (sub_optimal_v_max && budgetting_mode != "static"){
    	ros::param::set("v_max", renewed_v_max);
    	v_max__global = renewed_v_max;
    }
    ros::Time smoothening_start_time_stamp_ = ros::Time::now();

    smooth_path = smoothen_the_shortest_path(piecewise_path, octree, 
                                    Eigen::Vector3d(req.twist.linear.x,
                                        req.twist.linear.y,
                                        req.twist.linear.z), 
                                    Eigen::Vector3d(req.acceleration.linear.x,
                                                    req.acceleration.linear.y,
                                                    req.acceleration.linear.z), closest_unknown_way_point);
    /*
    if (unknown_budget_failed){
    	return false;
    }
	*/

    profiling_container.capture("motion_planning_smoothening_time", "end", ros::Time::now(), capture_size);
    if (DEBUG_RQT) {
    		debug_data.header.stamp = ros::Time::now();
    		debug_data.motion_planning_smoothening_time = profiling_container.findDataByName("motion_planning_smoothening_time")->values.back();
    		//motion_planning_debug_pub.publish(debug_data);
	}
    debug_data.motion_planning_smoothening_volume_explored = volume_explored_in_unit_cubes*pow(map_res,3);
    piecewise_traj_vis_pub.publish(piecewise_traj_markers);
    closest_unknown_pub.publish(closest_unknown_way_point);
	ros::param::set("/set_closest_unknown_point", true);
    if (smooth_path.empty()) {
		msg_for_follow_traj.closest_unknown_point = closest_unknown_way_point;
    	msg_for_follow_traj.ee_profiles.control_flow_path = 1.5;
    	ROS_ERROR("Path could not be smoothened successfully");
    	profiling_container.capture("motion_planning_smoothening_failure_cnt", "counter", 0, capture_size); // @suppress("Invalid arguments")
    	if (notified_failure){ //so we won't fly backward multiple times
    		//mavbench_msgs::response_time_capture msg_for_follow_traj;
    		//std_msgs::Header msg_for_follow_traj;
			if (!measure_time_end_to_end) { msg_for_follow_traj.header.stamp = ros::Time::now(); }
			else{ msg_for_follow_traj.header.stamp = req.header.stamp; }
			msg_for_follow_traj.closest_unknown_point.planning_status = "smoothening_failed";
			msg_for_follow_traj.ee_profiles.actual_time.ppl_latency= ppl_latency;
			msg_for_follow_traj.ee_profiles.actual_time.smoothening_latency = (ros::Time::now() - smoothening_start_time_stamp_).toSec();
			msg_for_follow_traj.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
//			msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
			timing_msg_from_mp_pub.publish(msg_for_follow_traj); //send a msg to make sure we update responese timne
    		return false;
    	}

        res.path_found = false;

        res.multiDOFtrajectory.future_collision_seq = future_col_seq_id;
        res.multiDOFtrajectory.trajectory_seq = trajectory_seq_id;
        trajectory_seq_id++;

        res.multiDOFtrajectory.append = false;
        res.multiDOFtrajectory.reverse = false;
        res.multiDOFtrajectory.stop = true;
        ROS_INFO_STREAM("asking FT to stop");
        res.multiDOFtrajectory.planning_status = "smoothening_failed"; //profiling
        notified_failure = true;
        if (!measure_time_end_to_end) { res.multiDOFtrajectory.header.stamp = ros::Time::now(); }
        else{ res.multiDOFtrajectory.header.stamp = req.header.stamp; }
        res.multiDOFtrajectory.controls = msg_for_follow_traj.controls;
        res.multiDOFtrajectory.ee_profiles = msg_for_follow_traj.ee_profiles;
        res.multiDOFtrajectory.closest_unknown_point= msg_for_follow_traj.closest_unknown_point;
        //res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
        res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = ppl_latency;
        res.multiDOFtrajectory.ee_profiles.actual_time.smoothening_latency = (ros::Time::now() - smoothening_start_time_stamp_).toSec();
        res.multiDOFtrajectory.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
        //res.multiDOFtrajectory.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
		res.multiDOFtrajectory.closest_unknown_point.planning_status = "smoothening_failed";
        res.multiDOFtrajectory.ee_profiles.control_flow_path = 1.5;
        traj_pub.publish(res.multiDOFtrajectory);
        return false;
    }
    notified_failure = false;

    create_response(res, smooth_path);

    // Publish the trajectory (for debugging purposes)
    if (!measure_time_end_to_end) { res.multiDOFtrajectory.header.stamp = ros::Time::now(); }
    else{ res.multiDOFtrajectory.header.stamp = req.header.stamp;}
    res.multiDOFtrajectory.replanning_reason = replanning_reason;
    res.multiDOFtrajectory.planning_status = "success";
    got_new_next_steps_since_last_attempted_plan = false;
    res.multiDOFtrajectory.controls = msg_for_follow_traj.controls;
    res.multiDOFtrajectory.closest_unknown_point= msg_for_follow_traj.closest_unknown_point;
    res.multiDOFtrajectory.closest_unknown_point.planning_status = "success";
    res.multiDOFtrajectory.ee_profiles = msg_for_follow_traj.ee_profiles;
    res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = ppl_latency;
    //res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
	res.multiDOFtrajectory.ee_profiles.actual_time.smoothening_latency = (ros::Time::now() - smoothening_start_time_stamp_).toSec();
    res.multiDOFtrajectory.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
    //res.multiDOFtrajectory.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
	//res.multiDOFtrajectory.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
    //res.multiDOFtrajectory.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
	res.multiDOFtrajectory.ee_profiles.control_flow_path = 2;
    traj_pub.publish(res.multiDOFtrajectory);
    smooth_traj_vis_pub.publish(smooth_traj_markers);

    g_number_of_planning++; 
    res.path_found = true;
    planned_approximately = (status == 3);
//    ROS_INFO_STREAM("this was approximate so planned approximately is" << planned_approximately);
    if (planned_approximately || !first_time_planning_succeeded) {
		profiling_container.capture("approximate_plans_count", "counter", 0); // @suppress("Invalid arguments")
    }

    return true;
}


double MotionPlanner::calculate_path_length(piecewise_trajectory piecewise_path){
	double total_path_length = 0;
	for (auto it = piecewise_path.begin(); it != piecewise_path.end() - 1 ; it++) {
    	total_path_length += distance(it->x - (it+1)->x, it->y - (it+1)->y, it->z - (it+1)->z);
	}
	return total_path_length;
}


void MotionPlanner::get_start_in_future(Drone& drone,
        geometry_msgs::Point& start, geometry_msgs::Twist& twist,
        geometry_msgs::Twist& acceleration)
{

	if (g_next_steps_msg.points.empty()) { //|| g_next_steps_msg.reverse) {
		//ROS_INFO_STREAM("start in the future is now");
		auto pos = drone.position();
        start.x = pos.x; start.y = pos.y; start.z = pos.z; 
        twist.linear.x = twist.linear.y = twist.linear.z = 0;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
       // ROS_ERROR_STREAM("only if I have to reverse");
        return;
    }

    //Data *look_ahead_time;
    //profiling_container.findDataByName("motion_planning_time_total", &look_ahead_time);
    //multiDOFpoint mdofp = trajectory_at_time(g_next_steps_msg, look_ahead_time->values.back());
    multiDOFpoint mdofp = trajectory_at_time(g_next_steps_msg, 2*g_ppl_time_budget);
	//multiDOFpoint mdofp = trajectory_at_time(g_next_steps_msg, .2);


    // Shift the drone's planned position at time "g_ppl_time_budget" seconds
    // by its current position
    auto current_pos = drone.position();
    auto planned_point = g_next_steps_msg.points[0];
    if (planned_point.vx == 0 && planned_point.vy == 0 && planned_point.vz == 0){
    	mdofp.x = current_pos.x;
    	mdofp.y = current_pos.y;
    	mdofp.z = current_pos.z;
    }else if (next_steps_msg_size == 0){
    	ROS_INFO_STREAM("101010-----------10101010   using current position");
    	mdofp.x = current_pos.x;
    	mdofp.y = current_pos.y;
    	mdofp.z = current_pos.z;
    	mdofp.vx =  mdofp.vy = mdofp.vz = 0;
    }
    else{
    	mdofp.x += current_pos.x - planned_point.x;
    	mdofp.y += current_pos.y - planned_point.y;
    	mdofp.z += current_pos.z - planned_point.z;
    	//mdofp.x = current_pos.x;
    	//mdofp.y = current_pos.y;
    	//mdofp.z = current_pos.z;


    }

    start.x = mdofp.x; start.y = mdofp.y; start.z = mdofp.z; 

    twist.linear.x = mdofp.vx;
    twist.linear.y = mdofp.vy;
    twist.linear.z = mdofp.vz;

    acceleration.linear.x = mdofp.ax;
    acceleration.linear.y = mdofp.ay;
    acceleration.linear.z = mdofp.az;
}


void MotionPlanner::future_col_callback(const mavbench_msgs::future_collision::ConstPtr& msg)
{
    ROS_INFO("motion_planner: New collision noticed");

    if (msg->future_collision_seq > future_col_seq_id)
        future_col_seq_id = msg->future_collision_seq;
    else
        return;

    // If neccessary, plan a new path for the drone
    if (g_next_steps_msg.future_collision_seq <= future_col_seq_id) {
        // "Call the get_trajectory_fun function right here, without waiting
        // for the package_delivery node to make a request
    	motion_plan_end_to_end(msg->header.stamp, g_goal_pos);
    	/*
    	package_delivery::get_trajectory::Request req;
        package_delivery::get_trajectory::Response res;

        get_start_in_future(*drone, req.start, req.twist, req.acceleration);
        req.goal = g_goal_pos;
        req.header.stamp = msg->header.stamp;
        req.call_func = 1;  //used for debugging purposes
        get_trajectory_fun(req, res);
        */
    }
}

bool MotionPlanner::haveExistingTraj(mavbench_msgs::multiDOFtrajectory *traj){
	if (traj == nullptr){
		return false;
	}else if (traj->points.size() == 0) {
		return false;
	}
	return true;
}


bool MotionPlanner::motion_plan_end_to_end(ros::Time invocation_time, geometry_msgs::Point g_goal){

	package_delivery::get_trajectory::Request req;
	package_delivery::get_trajectory::Response res;

	get_start_in_future(*drone, req.start, req.twist, req.acceleration);
	req.goal = g_goal;
	req.header.stamp = invocation_time;
	req.call_func = 1;  //used for debugging purposes
	failed_to_plan_last_time = !get_trajectory_fun(req, res);
	return !failed_to_plan_last_time;

	/*
	if (failed_to_plan_last_time || first_time_planning){
		profiling_container.capture("planning_failure_count", "counter", 0); // @suppress("Invalid arguments")
	}

    if (!failed_to_plan_last_time && first_time_planning){
    	first_time_planning_succeeded = true;
    }
	*/
}


void MotionPlanner::next_steps_callback(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg)
{
	got_new_next_steps_since_last_attempted_plan = true;
	g_next_steps_msg = *msg;
	next_steps_msg_size  = msg->points.size();
	if (!next_steps_msg_size){
		ROS_INFO_STREAM("msg sizees are 00000000000000000000");
	}
}

void MotionPlanner::motion_planning_initialize_params()
{
	if(!ros::param::get("/DEBUG_RQT", DEBUG_RQT)){
      ROS_FATAL_STREAM("Could not start motion_planning DEBUG_RQT not provided");
      return ;
    }


	if(!ros::param::get("/budgetting_mode", budgetting_mode)){
      ROS_FATAL_STREAM("Could not start motion_planning budgetting_mode not provided");
      return ;
    }


	if(!ros::param::get("/voxel_type_to_publish", voxel_type_to_publish)){
      ROS_FATAL_STREAM("Could not start motion_planner voxel_type_to_publish not provided");
      return ;
    }


	if(!ros::param::get("/DEBUG_VIS", DEBUG_VIS)){
      ROS_FATAL_STREAM("Could not start motion_planning DEBUG_VIS not provided");
      return ;
    }





	if(!ros::param::get("/ppl_time_budget", g_ppl_time_budget)){
      ROS_FATAL_STREAM("Could not start pkg delivery ppl_time_budget not provided");
      return ;
    }

    if(!ros::param::get("/capture_size", capture_size)){
      ROS_FATAL_STREAM("Could not start pkg delivery capture_size not provided");
      return ;
    }

    if(!ros::param::get("/smoothening_budget", g_smoothening_budget)){
      ROS_FATAL_STREAM("Could not start pkg delivery smoothening_budget not provided");
      return ;
    }

    if(!ros::param::get("/distance_to_goal_margin",distance_to_goal_margin))  {
      ROS_FATAL_STREAM("Could not start mapping goal_distance_margin not provided");
      return ;
    }



    ros::param::get("/clct_data_mode", clct_data_mode_);
    ros::param::get("/motion_planner/max_roadmap_size", max_roadmap_size__global);
    ros::param::get("/sampling_interval", sampling_interval__global);
    ros::param::get("/motion_planner/rrt_step_size", rrt_step_size__global);
    ros::param::get("/motion_planner/rrt_bias", rrt_bias__global);
    ros::param::get("/motion_planner/x_dist_to_sample_from__low_bound", x__low_bound__global);
    ros::param::get("/motion_planner/x_dist_to_sample_from__high_bound", x__high_bound__global);
    ros::param::get("/motion_planner/always_randomize_end_point", g_always_randomize_end_point);

    ros::param::get("/motion_planner/y_dist_to_sample_from__low_bound", y__low_bound__global);
    ros::param::get("/motion_planner/y_dist_to_sample_from__high_bound", y__high_bound__global);
    ros::param::get("/motion_planner/z_dist_to_sample_from__low_bound", z__low_bound__global);
    ros::param::get("/motion_planner/z_dist_to_sample_from__high_bound", z__high_bound__global);
    ros::param::get("/motion_planner/nodes_to_add_to_roadmap", nodes_to_add_to_roadmap__global);
    ros::param::get("/motion_planner/max_dist_to_connect_at", max_dist_to_connect_at__global);

    ros::param::get("/motion_planner/drone_radius", drone_radius__global);
    ros::param::get("/motion_planner/drone_height", drone_height__global);
    ros::param::get("/planner_drone_radius_when_hovering", planner_drone_radius_when_hovering);
    ros::param::get("/planner_drone_height_when_hovering", planner_drone_height_when_hovering);



    ros::param::get("/v_max", v_max__global);
    ros::param::get("/v_max_max", v_max_max);
    ros::param::get("/pc_res_max", pc_res_max);

    ros::param::get("/v_max_min", v_max_min);
    ros::param::get("/sensor_max_range", sensor_max_range);

    ros::param::get("/motion_planner/a_max", a_max__global);

    ros::param::get("/knob_performance_modeling", knob_performance_modeling);
    ros::param::get("/knob_performance_modeling_for_om_to_pl", knob_performance_modeling_for_om_to_pl);
    ros::param::get("/knob_performance_modeling_for_om_to_pl", knob_performance_modeling_for_om_to_pl);


    ros::param::get("/motion_planner/measure_time_end_to_end", measure_time_end_to_end);

    ros::param::get("/follow_trajectory_loop_rate", follow_trajectory_loop_rate);
    follow_trajectory_worse_case_latency = 2*(1/follow_trajectory_loop_rate);

    ros::param::get("ros_DEBUG", DEBUG__global);
    
    if(!ros::param::get("/motion_planner/planner_min_freq", planner_min_freq)) {
        ROS_FATAL("Could not start motion_planner node node. planner_min_freq is missing");
        exit(-1);
    }
    if(!ros::param::get("/max_time_budget", max_time_budget)) {
        ROS_FATAL("Could not start motion_planner node node. max_time_budget is missing");
        exit(-1);
    }


    ros::param::get("/motion_planner/motion_planning_core", motion_planning_core_str);
    if (motion_planning_core_str == "lawn_mower")
        motion_planning_core = [this] (geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status) {
            return this->lawn_mower(start, goal, width, length, n_pts_per_dir, octree);
        };
    else if (motion_planning_core_str == "OMPL-RRT")
        motion_planning_core = [this] (geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status) {
            return this->OMPL_RRT(start, goal, width, length, n_pts_per_dir, octree, status);
        };
    else if (motion_planning_core_str == "OMPL-RRTStar")
        motion_planning_core = [this] (geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status) {
            return this->OMPL_RRTStar(start, goal, width, length, n_pts_per_dir, octree, status);
        };
    else if (motion_planning_core_str == "OMPL-PRM")
        motion_planning_core = [this] (geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status) {
            return this->OMPL_PRM(start, goal, width, length, n_pts_per_dir, octree, status);
        };
    else {
        std::cout<<"This motion planning type is not defined"<<std::endl;
        exit(0);
    }


	profiling_container.capture("planning_failure_count", "counter", 0);
	profiling_container.capture("approximate_plans_count", "counter", 0); // @suppress("Invalid arguments")
	profiling_container.capture("planning_count", "counter", 0, capture_size); // @suppress("Invalid arguments")
    profiling_container.capture("motion_planning_smoothening_failure_cnt", "counter", 0, capture_size); // @suppress("Invalid arguments")
    profiling_container.capture("motion_planning_piecewise_failure_cnt", "counter", 0, capture_size); // @suppress("Invalid arguments")
}

void MotionPlanner::log_data_before_shutting_down()
{
	if (clct_data_mode_ =="only_end_to_end"){
		return;
	}

	// post processing for some statistics
	int total_planning_count = profiling_container.findDataByName("planning_count")->values.back();
	int approximate_planning_count = profiling_container.findDataByName("approximate_plans_count")->values.back();
	int planning_failure_count = profiling_container.findDataByName("planning_failure_count")->values.back();
	profiling_container.capture("planning_failure_rate", "single", float(planning_failure_count)/total_planning_count);
	profiling_container.capture("approximate_planning_rate", "single", float(approximate_planning_count)/total_planning_count);
	int  piecewise_failure_cnt =  profiling_container.findDataByName("motion_planning_piecewise_failure_cnt")->values.back();
	profiling_container.capture("planning_piecewise_failure_rate", "single", float(piecewise_failure_cnt)/total_planning_count);

	int  smoothening_failure_cnt =  profiling_container.findDataByName("motion_planning_smoothening_failure_cnt")->values.back();
	profiling_container.capture("planning_smoothening_failure_rate", "single", float(smoothening_failure_cnt)/total_planning_count);

	std::string ns = ros::this_node::getName();
    profiling_container.setStatsAndClear();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profile_manager::profiling_data_verbose_srv profiling_data_verbose_srv_inst;

    /*
    for (auto &data: profiling_container.container){
		profiling_data_srv_inst.request.key = data.data_key_name + " last window's avg: ";
		vector<double>* avg_stat = data.getStat("avg");
		if (avg_stat) {
			profiling_data_srv_inst.request.value = avg_stat->back();
		}
		else{
			profiling_data_srv_inst.request.value = nan("");
		}
		profile_manager.clientCall(profiling_data_srv_inst);
    }
	*/


    profiling_data_verbose_srv_inst.request.key = ros::this_node::getName()+"_verbose_data";
    profiling_data_verbose_srv_inst.request.value = "\n" + profiling_container.getStatsInString();
    profile_manager.verboseClientCall(profiling_data_verbose_srv_inst);

    /*
    profiling_data_srv_inst.request.key = "number_of_plannings";
    profiling_data_srv_inst.request.value = g_number_of_planning;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "motion_planning_acc";
    profiling_data_srv_inst.request.value = (double)g_planning_without_OM_PULL_time_acc/1e9;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
    
    profiling_data_srv_inst.request.key = "motion_planning_kernel";
    profiling_data_srv_inst.request.value = ((double)g_planning_without_OM_PULL_time_acc/1e9)/g_number_of_planning;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }
	*/
}

static double dist(const graph::node& n1, const graph::node& n2)
{
	return std::sqrt((n1.x-n2.x)*(n1.x-n2.x) + (n1.y-n2.y)*(n1.y-n2.y) + (n1.z-n2.z)*(n1.z-n2.z));
}


bool MotionPlanner::occupied(octomap::OcTree * octree, double x, double y, double z)
{
	const double OCC_THRESH = 0.5;

	octomap::OcTreeNode * otn = octree->search(x, y, z);

	return otn != nullptr && otn->getOccupancy() >= OCC_THRESH;
}


bool MotionPlanner::known(octomap::OcTree * octree, double x, double y, double z)
{
	return octree->search(x, y, z) != nullptr;
}

bool is_between(double x, double min, double max)
{
    return x >= min && x <= max;
}

bool MotionPlanner::out_of_bounds(const graph::node& pos)
{
    bool x_correct = is_between(pos.x, x__low_bound__global, x__high_bound__global);
    bool y_correct = is_between(pos.y, y__low_bound__global, y__high_bound__global);
    bool z_correct = is_between(pos.z, z__low_bound__global, z__high_bound__global);

    if (x_correct && y_correct && z_correct)
        return false;

    bool x_start_correct = is_between(g_start_pos.x, x__low_bound__global, x__high_bound__global);
    bool y_start_correct = is_between(g_start_pos.y, y__low_bound__global, y__high_bound__global);
    bool z_start_correct = is_between(g_start_pos.z, z__low_bound__global, z__high_bound__global);

    if (x_start_correct && y_start_correct && !z_start_correct) {
        if (is_between(pos.x, g_start_pos.x - g_out_of_bounds_allowance,
                    g_start_pos.x + g_out_of_bounds_allowance)
                && is_between(pos.y, g_start_pos.y - g_out_of_bounds_allowance,
                    g_start_pos.y + g_out_of_bounds_allowance)) {
            if (g_start_pos.z < z__low_bound__global && pos.z >= g_start_pos.z)
                return false;
            else if (g_start_pos.z > z__high_bound__global && pos.z <= g_start_pos.z)
                return false;
        }
    }

    return true;
}


bool MotionPlanner::out_of_bounds_strict(const graph::node& pos)
{
    bool x_correct = is_between(pos.x, x__low_bound__global, x__high_bound__global);
    bool y_correct = is_between(pos.y, y__low_bound__global, y__high_bound__global);
    bool z_correct = is_between(pos.z, z__low_bound__global, z__high_bound__global);

    return !x_correct || !y_correct || !z_correct;
}


bool MotionPlanner::out_of_bounds_lax(const graph::node& pos)
{
    double x_low = std::min(g_start_pos.x, x__low_bound__global);
    double x_high = std::max(g_start_pos.x, x__high_bound__global);
    double y_low = std::min(g_start_pos.y, y__low_bound__global);
    double y_high = std::max(g_start_pos.y, y__high_bound__global);
    double z_low = std::min(g_start_pos.z, z__low_bound__global);
    double z_high = std::max(g_start_pos.z, z__high_bound__global);

    bool x_correct = is_between(pos.x, x_low, x_high);
    bool y_correct = is_between(pos.y, y_low, y_high);
    bool z_correct = is_between(pos.z, z_low, z_high);
    if (!x_correct || !y_correct || !z_correct){
    	ROS_INFO_STREAM("fuuuuuuuuuuuuuuuuuck");
    	ROS_ERROR_STREAM("here are the posistion"<<pos.x<< "" << pos.y<< " "<<pos.z);
    }

    return !x_correct || !y_correct || !z_correct;
}

/*
bool MotionPlanner::collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, string mode, graph::node * end_ptr)
{
	profiling_container.capture("collision_func", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")

	if (motion_planning_core_str == "lawn_mower")
        return false;

	if (volume_explored_in_unit_cubes > ppl_vol_idealInUnitCube && piecewise_planning){
		return true;
	}

    RESET_TIMER();

    // First, check if anything goes too close to the ground
    if (n1.z <= drone_height__global || n2.z <= drone_height__global)
        return true;

    // Next, check if it goes out-of-bounds
    bool z_out_of_bounds = !is_between(n1.z, z__low_bound__global, z__high_bound__global)
        || !is_between(n2.z, z__low_bound__global, z__high_bound__global);
    if (z_out_of_bounds) {
        if (!is_between(n1.x, n2.x - g_out_of_bounds_allowance, n2.x + g_out_of_bounds_allowance)
                || !is_between(n1.y, n2.y - g_out_of_bounds_allowance, n2.y + g_out_of_bounds_allowance)) {
            if (end_ptr != nullptr) {
                end_ptr->x = n1.x;
                end_ptr->y = n1.y;
                end_ptr->z = n1.z;
            }
            return true;
        }
    }

    // Create a bounding box representing the drone
    double height = drone_height__global;
    double radius = drone_radius__global;

    octomap::point3d min(n1.x-radius, n1.y-radius, n1.z-height/2);
    octomap::point3d max(n1.x+radius, n1.y+radius, n1.z+height/2);

    // Create a direction vector over which to check for collisions
	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    potential_distance_to_explore += distance;
    octomap::point3d direction(dx, dy, dz);

    // Make sure the direction vector isn't just (0,0,0)
    // Otherwise, we'll get a bunch of really annoying error messages
    // if (distance == 0) {
    //     if (occupied(octree, n1.x, n1.y, n1.z)) {
    //         if (end_ptr != nullptr) {
    //             end_ptr->x = n1.x;
    //             end_ptr->y = n1.y;
    //             end_ptr->z = n1.z;
    //         }
    //         return true;
    //     } else
    //         return false;
    // }

    // Finally, loop over the drone's bounding box to search for collisions
    octomap::point3d end;
    for (auto it = octree->begin_leafs_bbx(min, max),
            end_it = octree->end_leafs_bbx(); it != end_it; ++it)
    {
        octomap::point3d start (it.getCoordinate());

        // std::cout << distance << " (" << start.x() << " " << start.y() << " " << start.z() << ") (" << direction.x() << " " << direction.y() << " " << direction.z() << ")" << std::endl;
        double volume_explored_in_unit_cubes_ = 0;
        //int resolution_ratio = (int)(map_res/octree->getResolution());
        //int depth_to_look_at = 16 - (int)log2((double)resolution_ratio);

        if (octree->castRayAndCollectVolumeTraveresed(start, direction, end, true, distance, volume_explored_in_unit_cubes_, 1)) {
            if (end_ptr != nullptr) {
                end_ptr->x = end.x();
                end_ptr->y = end.y();
                end_ptr->z = end.z();
            }
            volume_explored_in_unit_cubes += volume_explored_in_unit_cubes_;
            //ROS_INFO_STREAM("right here"<< volume_explored_in_unit_cubes);
            profiling_container.capture("collision_func", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
            //ROS_INFO_STREAM("direction"<<direction<< " Addded volume this round"<< volume_explored_in_unit_cubes_<< "total volume so far"<< volume_explored_in_unit_cubes);
            total_collision_func +=  profiling_container.findDataByName("collision_func")->values.back();
            return true;
        }
        volume_explored_in_unit_cubes += volume_explored_in_unit_cubes_;
        //ROS_INFO_STREAM("right here outside"<< volume_explored_in_unit_cubes);
//        ROS_INFO_STREAM("Addded volume this round"<< volume_explored_in_unit_cubes_<< "total volume so far"<< volume_explored_in_unit_cubes);
    }

	profiling_container.capture("collision_func", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
	total_collision_func +=  profiling_container.findDataByName("collision_func")->values.back();
	//LOG_ELAPSED(motion_planner);
	return false;
}
*/


bool MotionPlanner::coord_on_drone(octomap::point3d point){
	if ((point.x() < drone->position().x + drone_radius__global && point.x()> drone->position().x - drone_radius__global )&&
			(point.y() < drone->position().y + drone_radius__global && point.y()> drone->position().y - drone_radius__global )&&
			(point.z() < drone->position().z + drone_height__global && point.z()> drone->position().z - drone_height__global )){
		return true;
	}
	return false;
}

bool MotionPlanner::collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, mavbench_msgs::planner_info &closest_unknown_point, graph::node * end_ptr)
{
	profiling_container.capture("collision_func", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")

	if (motion_planning_core_str == "lawn_mower")
        return false;

	if (volume_explored_in_unit_cubes > ppl_vol_idealInUnitCube && piecewise_planning){
		return true;
	}

	octomap::point3d n1_point; // to convert n1 from graph::node to octomap::point
    n1_point(0) = n1.x;
    n1_point(1) = n1.y;
    n1_point(2) = n1.z;
    closest_unknown_point.x = nan(""); // intialize for scenarios that we exceed the bound
    closest_unknown_point.y = nan(""); // intialize for scenarios that we exceed the bound
    closest_unknown_point.z = nan(""); // intialize for scenarios that we exceed the bound
    double min_dist_to_unknown_so_far = std::numeric_limits<double>::max();

    RESET_TIMER();

    // First, check if anything goes too close to the ground
    if (n1.z <= drone_height__global || n2.z <= drone_height__global){
    	return true;
    }

    // Next, check if it goes out-of-bounds
    bool z_out_of_bounds = !is_between(n1.z, z__low_bound__global, z__high_bound__global)
        || !is_between(n2.z, z__low_bound__global, z__high_bound__global);
    if (z_out_of_bounds) {
        if (!is_between(n1.x, n2.x - g_out_of_bounds_allowance, n2.x + g_out_of_bounds_allowance)
                || !is_between(n1.y, n2.y - g_out_of_bounds_allowance, n2.y + g_out_of_bounds_allowance)) {
            if (end_ptr != nullptr) {
                end_ptr->x = n1.x;
                end_ptr->y = n1.y;
                end_ptr->z = n1.z;
            }
            return true;
        }
    }

    // Create a bounding box representing the drone
    //double height = drone_height__global;
    //double radius = drone_radius__global;

    /*
    if (cur_velocity.x < .1 && cur_velocity.y <.1 && cur_velocity.z<.1){ // if stopped, no need to expand the radius since there
    																	 // the deviation from track is zero (so no saftely halo is necessary)
    	height = planner_drone_height_when_hovering;
    	radius = planner_drone_radius_when_hovering;
    }
    */
    auto cur_velocity = drone->velocity().linear;
    auto cur_vel_mag = calc_vec_magnitude(cur_velocity.x, cur_velocity.y, cur_velocity.z);
    double reduced_halo = 0; // used for the scenarios that the drone think it's so close to the obstacles that it thinks it's on obstacles.
    						 // we pull back the halo to fix this dellusion
    /*
    if (planner_consecutive_failure_ctr > 4) {
    	reduced_halo = pc_res_max + .25;
    }
	*/


    double height = -1*correct_distance(cur_vel_mag, v_max__global, drone_height__global, planner_drone_height_when_hovering, 0);
    double radius = -1*correct_distance(cur_vel_mag, v_max__global, drone_radius__global, planner_drone_radius_when_hovering, 0);
    	//    double height = max((cur_vel_mag/v_max__global)*drone_height__global, planner_drone_height_when_hovering - reduced_halo);
 //   double radius = max((cur_vel_mag/v_max__global)*drone_radius__global, planner_drone_radius_when_hovering - reduced_halo);



    octomap::point3d min(n1.x-radius, n1.y-radius, n1.z-height/2);
    octomap::point3d max(n1.x+radius, n1.y+radius, n1.z+height/2);

    // Create a direction vector over which to check for collisions
	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    potential_distance_to_explore += distance;
    octomap::point3d direction(dx, dy, dz);

    // Make sure the direction vector isn't just (0,0,0)
    // Otherwise, we'll get a bunch of really annoying error messages
    // if (distance == 0) {
    //     if (occupied(octree, n1.x, n1.y, n1.z)) {
    //         if (end_ptr != nullptr) {
    //             end_ptr->x = n1.x;
    //             end_ptr->y = n1.y;
    //             end_ptr->z = n1.z;
    //         }
    //         return true;
    //     } else
    //         return false;
    // }

    // Finally, loop over the drone's bounding box to search for collisions
    int resolution_ratio = (int)(map_res/octree->getResolution());
    int depth_to_look_at = 16 - (int)log2((double)resolution_ratio);
    octomap::point3d end;

   // unknown calculation
    octomap::OcTreeKey current_key;
    if (!octree->coordToKeyChecked(n1_point, current_key) ) { // if out of bound
    	closest_unknown_point.x = n1_point.x();
    	closest_unknown_point.y = n1_point.y();
    	closest_unknown_point.z = n1_point.z();
    }else{
    	if (!octree->search(current_key, depth_to_look_at)) { // if node doesn't exist
    		if (!coord_on_drone(n1_point)){ // if it's on the drone, it can not be an unknown
    			closest_unknown_point.x = n1_point.x();
    			closest_unknown_point.y = n1_point.y();
    			closest_unknown_point.z = n1_point.z();
    		}
    	}
    }

    //    octomap::point3d start = min;

    //for (auto it = octree->begin_leafs_bbx(min, max),
   //         end_it = octree->end_leafs_bbx(); it != end_it; ++it)

    auto it = octree->begin_leafs_bbx(min, max);
    auto end_it = octree->end_leafs_bbx();
    while(true) {
    	if (it == end_it){
        	break;
        }

    	octomap::point3d start(it.getCoordinate());
        double volume_explored_in_unit_cubes_ = 0;

        if (octree->castRayAndCollectVolumeTraveresed(start, direction, end, true, distance, volume_explored_in_unit_cubes_, resolution_ratio, depth_to_look_at)) {
            if (end_ptr != nullptr) {
                end_ptr->x = end.x();
                end_ptr->y = end.y();
                end_ptr->z = end.z();
            }
            volume_explored_in_unit_cubes += volume_explored_in_unit_cubes_;
            profiling_container.capture("collision_func", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
            total_collision_func +=  profiling_container.findDataByName("collision_func")->values.back();
            return true;
        }
        volume_explored_in_unit_cubes += volume_explored_in_unit_cubes_;
        ++it;
    }

	profiling_container.capture("collision_func", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
	total_collision_func +=  profiling_container.findDataByName("collision_func")->values.back();
	//LOG_ELAPSED(motion_planner);
	return false;
}


bool MotionPlanner::collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
{
	profiling_container.capture("collision_func", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")

	if (motion_planning_core_str == "lawn_mower")
        return false;

	if (volume_explored_in_unit_cubes > ppl_vol_idealInUnitCube && piecewise_planning){
		return true;
	}

    RESET_TIMER();

    // First, check if anything goes too close to the ground
    if (n1.z <= drone_height__global || n2.z <= drone_height__global)
        return true;

    // Next, check if it goes out-of-bounds
    bool z_out_of_bounds = !is_between(n1.z, z__low_bound__global, z__high_bound__global)
        || !is_between(n2.z, z__low_bound__global, z__high_bound__global);
    if (z_out_of_bounds) {
        if (!is_between(n1.x, n2.x - g_out_of_bounds_allowance, n2.x + g_out_of_bounds_allowance)
                || !is_between(n1.y, n2.y - g_out_of_bounds_allowance, n2.y + g_out_of_bounds_allowance)) {
            if (end_ptr != nullptr) {
                end_ptr->x = n1.x;
                end_ptr->y = n1.y;
                end_ptr->z = n1.z;
            }
            return true;
        }
    }

    // Create a bounding box representing the drone
    //double height = drone_height__global;
   // double radius = drone_radius__global;
    auto cur_velocity = drone->velocity().linear;
    auto cur_vel_mag = calc_vec_magnitude(cur_velocity.x, cur_velocity.y, cur_velocity.z);
    double reduced_halo = 0; // used for the scenarios that the drone think it's so close to the obstacles that it thinks it's on obstacles.
    						 // we pull back the halo to fix this dellusion
    /*
    if (planner_consecutive_failure_ctr > 4) {
    	reduced_halo = pc_res_max + .25;
    }
    */
    //double height = max((cur_vel_mag/v_max__global)*drone_height__global, planner_drone_height_when_hovering - reduced_halo);
    //double radius = max((cur_vel_mag/v_max__global)*drone_radius__global, planner_drone_radius_when_hovering - reduced_halo);
    double height = -1*correct_distance(cur_vel_mag, v_max__global, drone_height__global, planner_drone_height_when_hovering, 0);
    double radius = -1*correct_distance(cur_vel_mag, v_max__global, drone_radius__global, planner_drone_radius_when_hovering, 0);


    /*
    auto cur_velocity = drone->velocity().linear;
    if (cur_velocity.x < .1 && cur_velocity.y <.1 && cur_velocity.z<.1){ // if stopped, no need to expand the radius since there
    																	 // the deviation from track is zero (so no saftely halo is necessary)
    	height = planner_drone_height_when_hovering;
    	radius = planner_drone_radius_when_hovering;
    }
	*/

    octomap::point3d min(n1.x-radius, n1.y-radius, n1.z-height/2);
    octomap::point3d max(n1.x+radius, n1.y+radius, n1.z+height/2);

    // Create a direction vector over which to check for collisions
	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    potential_distance_to_explore += distance;
    octomap::point3d direction(dx, dy, dz);

    // Make sure the direction vector isn't just (0,0,0)
    // Otherwise, we'll get a bunch of really annoying error messages
    // if (distance == 0) {
    //     if (occupied(octree, n1.x, n1.y, n1.z)) {
    //         if (end_ptr != nullptr) {
    //             end_ptr->x = n1.x;
    //             end_ptr->y = n1.y;
    //             end_ptr->z = n1.z;
    //         }
    //         return true;
    //     } else
    //         return false;
    // }

    // Finally, loop over the drone's bounding box to search for collisions
    int resolution_ratio = (int)((map_res)/octree->getResolution());
    int depth_to_look_at = 16 - (int)log2((double)resolution_ratio);
    octomap::point3d end;
    for (auto it = octree->begin_leafs_bbx(min, max),
            end_it = octree->end_leafs_bbx(); it != end_it; ++it)
    {
        octomap::point3d start (it.getCoordinate());
        
        // std::cout << distance << " (" << start.x() << " " << start.y() << " " << start.z() << ") (" << direction.x() << " " << direction.y() << " " << direction.z() << ")" << std::endl;
        double volume_explored_in_unit_cubes_ = 0;


        if (octree->castRayAndCollectVolumeTraveresed(start, direction, end, true, distance, volume_explored_in_unit_cubes_, resolution_ratio, depth_to_look_at)) {
            if (end_ptr != nullptr) {
                end_ptr->x = end.x();
                end_ptr->y = end.y();
                end_ptr->z = end.z();
            }
            volume_explored_in_unit_cubes += volume_explored_in_unit_cubes_;
            profiling_container.capture("collision_func", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
            //ROS_INFO_STREAM("direction"<<direction<< " Addded volume this round"<< volume_explored_in_unit_cubes_<< "total volume so far"<< volume_explored_in_unit_cubes);
          //  if (piecewise_planning){
            //ROS_INFO_STREAM("right here"<< volume_explored_in_unit_cubes);
         //   }
//        ROS_INFO_STREAM("Addded volume this round"<< volume_explored_in_unit_cubes_<< "total volume so far"<< volume_explored_in_unit_cubes);
            total_collision_func +=  profiling_container.findDataByName("collision_func")->values.back();
            return true;
        }
        volume_explored_in_unit_cubes += volume_explored_in_unit_cubes_;
        //if (piecewise_planning){
        //ROS_INFO_STREAM("right here outside"<< volume_explored_in_unit_cubes);
        //}
        //        ROS_INFO_STREAM("Addded volume this round"<< volume_explored_in_unit_cubes_<< "total volume so far"<< volume_explored_in_unit_cubes);
    }

	profiling_container.capture("collision_func", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
	total_collision_func +=  profiling_container.findDataByName("collision_func")->values.back();
	//LOG_ELAPSED(motion_planner);
	return false;
}


void MotionPlanner::create_response(package_delivery::get_trajectory::Response &res, const smooth_trajectory& smooth_path)
{
    const double safe_radius = 1.0;

	// Sample trajectory
	mav_msgs::EigenTrajectoryPoint::Vector states;
	// double sampling_interval__global;
	// ros::param::get("/motion_planner/sampling_interval__global", sampling_interval__global);
	mav_trajectory_generation::sampleWholeTrajectory(smooth_path, sampling_interval__global, &states);

    // Get starting position
    graph::node start = {states[0].position_W.x(), states[0].position_W.y(), states[0].position_W.z()};

	// Convert sampled trajectory points to MultiDOFJointTrajectory response
    res.unknown = -1;

    int state_index = 0;
    int seq_ctr = 0;
    // for (const auto& s : states) {
    for (int i = 0; i < states.size() - 1; i++) {
        const auto& s = states[i];
        const auto& s_next = states[i+1];

		mavbench_msgs::multiDOFpoint point;

        graph::node current;
		point.x = current.x = s_next.position_W.x();
		point.y = current.y = s_next.position_W.y();
		point.z = current.z = s_next.position_W.z();

		point.vx = s.velocity_W.x();
		point.vy = s.velocity_W.y();
		point.vz = s.velocity_W.z();
        //ROS_INFO_STREAM("-----vx"<<point.vx<<"vy"<<point.vy<<"vz"<<point.vz) ;

		point.ax = s.acceleration_W.x();
		point.ay = s.acceleration_W.y();
		point.az = s.acceleration_W.z();
		point.pt_ctr = seq_ctr;
		seq_ctr +=1;

        point.yaw = yawFromVelocity(point.vx, point.vy);
        point.blocking_yaw = false;

	    point.duration = double(s_next.time_from_start_ns - s.time_from_start_ns) / 1e9;

        if (res.unknown != -1 &&
                !known(octree, current.x, current.y, current.z)
                && dist(start, current) > safe_radius) {
            ROS_WARN("Trajectory enters unknown space.");
            res.unknown = state_index;
        }

		res.multiDOFtrajectory.points.push_back(point);
        state_index++;
	}

    res.multiDOFtrajectory.append = false;
    res.multiDOFtrajectory.reverse = false;
    res.multiDOFtrajectory.stop = false;

    // Mark the trajectory with the correct sequence id's
    res.multiDOFtrajectory.trajectory_seq = trajectory_seq_id;
    trajectory_seq_id++;

    res.multiDOFtrajectory.future_collision_seq = future_col_seq_id;
}

void MotionPlanner::spinOnce() {
	callback_queue.callAvailable(ros::WallDuration());
    motion_planning_debug_pub.publish(debug_data);
}



bool MotionPlanner::traj_colliding(mavbench_msgs::multiDOFtrajectory *traj, mavbench_msgs::planner_info &closest_unknown_way_point)
{
	if (octree == nullptr || traj->points.size() < 1) {
        return false;
    }

    bool col = false;
    mavbench_msgs::planner_info closest_unknown_point;
    bool first_unknown_collected = false; // to only allow writing into the closest_unknown_way_point once
    graph::node *end_ptr = new graph::node();
    //ROS_INFO_STREAM("map_res is "<<map_res);
    int resolution_ratio = (int)((map_res)/octree->getResolution());
    int depth_to_look_at = 16 - (int)log2((double)resolution_ratio);
    auto last_key = octree->coordToKey(octomap::point3d(-100, -100, -100), depth_to_look_at);
    for (int i = 0; i < traj->points.size() - 1; ++i) {
        auto& pos1 = traj->points[i];
        auto& pos2 = traj->points[i+1];
        graph::node n1 = {pos1.x, pos1.y, pos1.z};
        graph::node n2 = {pos2.x, pos2.y, pos2.z};
		auto this_key = octree->coordToKey(octomap::point3d(n1.x, n1.y, n1.z), depth_to_look_at);
		if (i == 0 || this_key != last_key ){ // skip if you can
			last_key = this_key;
		}else{
			continue;
		}

        if (collision(octree, n1, n2, closest_unknown_point, end_ptr)) {
        	// TODO: this is not unknown anymore, but nevertheless should be used to determine the budget
        	closest_obstacle_on_path_way_point.x = pos1.x;
        	closest_obstacle_on_path_way_point.y = pos1.y;
        	closest_obstacle_on_path_way_point.z = pos1.z;
        	last_unknown_pt_ctr  = pos1.pt_ctr;
        	col = true;
            break;
        }else{
        	if (!isnan(closest_unknown_point.x) && !first_unknown_collected){
        		if (pos1.pt_ctr >= last_unknown_pt_ctr){  // if smaller, what that means is that the unknown is discovered because we lowered the voxel
        												 // size, however, this is really not an issue because in our subsampling, we would
        											     // made sure to include any point that would be been an obstacle
        												 // so we won't be mistakenly thinking that an unknown is free because of subsampling
					closest_unknown_way_point.x = pos1.x;
					closest_unknown_way_point.y = pos1.y;
					closest_unknown_way_point.z = pos1.z;
					last_unknown_pt_ctr  = pos1.pt_ctr;
					first_unknown_collected = true;
        		}else{
        		;
        			//ROS_INFO_STREAM("closest uknown didn't change");
        		}
        	}
        }
    }

    //ROS_INFO_STREAM("volume"<<volume_explored_in_unit_cubes);
    return col;




}




int MotionPlanner::find_optimum_vec_index(mavbench_msgs::multiDOFpoint cur_point, piecewise_trajectory& vertecis, int cur_vertex_idx) {
	if (cur_vertex_idx + 2 >= vertecis.size()){
		return cur_vertex_idx;
	}else{
		if (
		(distance(cur_point.x - vertecis[cur_vertex_idx].x, cur_point.y - vertecis[cur_vertex_idx].y, cur_point.z - vertecis[cur_vertex_idx].z) +
		 distance(cur_point.x - vertecis[cur_vertex_idx+1].x, cur_point.y - vertecis[cur_vertex_idx+1].y, cur_point.z - vertecis[cur_vertex_idx+1].z)) <
		(distance(cur_point.x - vertecis[cur_vertex_idx+1].x, cur_point.y - vertecis[cur_vertex_idx+1].y, cur_point.z - vertecis[cur_vertex_idx+1].z) +
		 distance(cur_point.x - vertecis[cur_vertex_idx+2].x, cur_point.y - vertecis[cur_vertex_idx+2].y, cur_point.z - vertecis[cur_vertex_idx+2].z))){
			return cur_vertex_idx;
		}else{
			return cur_vertex_idx +1;
		}
	}
}

// traverse the entire trajectory and find the number of optimums up to certain vertex
int MotionPlanner::get_num_of_optimums(piecewise_trajectory& piecewise, mav_trajectory_generation::Trajectory &traj, int vertex_num) {
	mav_msgs::EigenTrajectoryPoint::Vector states;
	double sampling_interval = .1;
	mav_trajectory_generation::sampleWholeTrajectory(traj, sampling_interval, &states);
	mavbench_msgs::multiDOFpoint cur_point;
	mavbench_msgs::multiDOFpoint next_point;
	int optimum_cnt = 0;
	vector<int> optimum_vec;
	for (int i =0; i <piecewise.size(); i++){
		optimum_vec.push_back(0);
	}
	int cur_vertex_idx = 0;
	for (int i = 0; i < states.size() - 1; i++) {
        const auto& s = states[i];
        const auto& s_next = states[i+1];


        cur_point.vx = s.velocity_W.x();
		cur_point.vy = s.velocity_W.y();
		cur_point.vz = s.velocity_W.z();

		next_point.vx = s_next.velocity_W.x();
		next_point.vy = s_next.velocity_W.y();
		next_point.vz = s_next.velocity_W.z();

		// if found an optimum, store it in the
		// optimum_vec according to the index of the vertex
		// of the first of two points that it's closest to (distance wise
		// this means if the optimum is between vertex two and three (of the piecewise path)
		// add incremenet the optimum_vec[2]
		if (next_point.vx * cur_point.vx < 0  ||
			next_point.vy * cur_point.vy < 0  ||
			next_point.vz * cur_point.vz < 0 ) {
			cur_vertex_idx = find_optimum_vec_index(cur_point, piecewise, cur_vertex_idx);
//			cout<<"cur_vertex_idx"<< cur_vertex_idx<<" "<<optimum_vec.size()<<endl;
			optimum_vec[cur_vertex_idx] += 1;
		}
    }
	for (auto it=optimum_vec.begin(); it!= optimum_vec.end(); it++){
		cout<< *it << " ";
	}

	int accumulated_num_of_optimums= 0;
	int idx = 0;
	for (auto it= optimum_vec.begin(); it!=optimum_vec.end() || idx < vertex_num; it++){
		accumulated_num_of_optimums += *it;
		idx +=1;
	}
	return accumulated_num_of_optimums;
}


void setMarkerProperties(const std_msgs::Header& header, double life_time,
                         const visualization_msgs::Marker::_action_type& action,
                         visualization_msgs::MarkerArray* markers) {
  CHECK_NOTNULL(markers);
  int count = 0;
  for (visualization_msgs::Marker& marker : markers->markers) {
    marker.header = header;
    marker.action = action;
    marker.id = count;
    marker.lifetime = ros::Duration(life_time);
    ++count;
  }
}

void drawVerticesModified(const mav_trajectory_generation::Vertex::Vector& vertices, const std::string& frame_id,
                  visualization_msgs::MarkerArray* marker_array, int status) {
  CHECK_NOTNULL(marker_array);
  marker_array->markers.resize(1);
  visualization_msgs::Marker& marker = marker_array->markers.front();

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  if (status == 3 ){
	  marker.color = mav_visualization::Color::Orange();
  }else{
	  marker.color = mav_visualization::Color::Green();
  }
  marker.scale.x = 1;
  marker.ns = "straight_path";

  for (const mav_trajectory_generation::Vertex& vertex : vertices) {
    if (vertex.D() != 3) {
      ROS_ERROR("Vertex has dimension %d but should have dimension 3.",
                vertex.D());
      return;
    }

    if (vertex.hasConstraint(mav_trajectory_generation::derivative_order::POSITION)) {
      Eigen::VectorXd position = Eigen::Vector3d::Zero();
      vertex.getConstraint(mav_trajectory_generation::derivative_order::POSITION, &position);
      geometry_msgs::Point constraint_msg;
      tf::pointEigenToMsg(position, constraint_msg);
      marker.points.push_back(constraint_msg);
    } else
      ROS_WARN("Vertex does not have a position constraint, skipping.");
  }

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  setMarkerProperties(header, 0.0, visualization_msgs::Marker::ADD,
                                marker_array);
}

void MotionPlanner::draw_piecewise(piecewise_trajectory& piecewise_path, int status, visualization_msgs::MarkerArray* marker_array){
	mav_trajectory_generation::Vertex::Vector vertices;
	const int dimension = 3;
	std::string frame_id = "world";
	for (auto it = piecewise_path.begin(); it+1 != piecewise_path.end(); ++it) {
		mav_trajectory_generation::Vertex v(dimension);
		v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(it->x, it->y, it->z));
		vertices.push_back(v);
	}
	drawVerticesModified(vertices, frame_id, marker_array, status);
}


// blah now
// determine wheter we have enough budget to  run SA again
bool MotionPlanner::got_enough_budget_for_next_SA_itr(piecewise_trajectory& piecewise_path)	{
	bool got_enough_budget_for_next_itr = false;
	for (auto it=piecewise_path.begin(); it !=piecewise_path.end(); it++){
		graph::node n1 = {it->x, it->y, it->z};
		graph::node n2 = {(it+1)->x, (it+1)->y, (it+1)->z};
		mavbench_msgs::planner_info closest_unknown_way_point;
		collision(octree, n1, n2, closest_unknown_way_point);
		multiDOFpoint cur_point;
		cur_point.x = it->x;
		cur_point.y = it->y;
		cur_point.z = it->z;
		cur_point.vx = drone->velocity().linear.x;
		cur_point.vy = drone->velocity().linear.y;
		cur_point.vz = drone->velocity().linear.z;
		multiDOFpoint closest_unknown_point;
		closest_unknown_point.x = closest_unknown_way_point.x;
		closest_unknown_point.y = closest_unknown_way_point.y;
		closest_unknown_point.z = closest_unknown_way_point.z;
		double budget_till_next_unknown = time_budgetter->calc_budget_till_closest_unknown(cur_point, closest_unknown_point);
		/*
		if (budget_till_next_unknown < (map_res/2)){
			unknown_budget_failed = true;
			ROS_ERROR_STREAM("--------------------------- didn't got teh budget");
			return false;
		}
		*/
	}
	return true;
}


bool MotionPlanner::got_enough_budget_for_next_SA_itr(mavbench_msgs::planner_info closest_unknown_way_point, mav_trajectory_generation::Trajectory smoothened_traj){
	multiDOFpoint cur_point;
	cur_point.x = drone->position().x;
	cur_point.y = drone->position().y;
	cur_point.z = drone->position().z;
//	cur_point.vx = drone->velocity().linear.x;
//	cur_point.vy = drone->velocity().linear.y;
//	cur_point.vz = drone->velocity().linear.z;
	coord drone_position = {cur_point.x, cur_point.y, cur_point.z};
	multiDOFpoint closest_unknown_point;
	closest_unknown_point.x = closest_unknown_way_point.x;
	closest_unknown_point.y = closest_unknown_way_point.y;
	closest_unknown_point.z = closest_unknown_way_point.z;
	//double budget_till_next_unknown = time_budgetter->calc_budget_till_closest_unknown(cur_point, closest_unknown_point);

	// iterate through all the points and find the budget till next unknown
	trajectory_t traj_converted;
	// Sample trajectory
	mav_msgs::EigenTrajectoryPoint::Vector states;
	mav_trajectory_generation::sampleWholeTrajectory(smoothened_traj, sampling_interval__global, &states);
    graph::node start = {states[0].position_W.x(), states[0].position_W.y(), states[0].position_W.z()};
    for (int i = 0; i < states.size() - 1; i++) {
        const auto& s = states[i];
        const auto& s_next = states[i+1];

		multiDOFpoint point;

        graph::node current;
		point.x = current.x = s.position_W.x();
		point.y = current.y = s.position_W.y();
		point.z = current.z = s.position_W.z();

		point.vx = s.velocity_W.x();
		point.vy = s.velocity_W.y();
		point.vz = s.velocity_W.z();
        //ROS_INFO_STREAM("-----vx"<<point.vx<<"vy"<<point.vy<<"vz"<<point.vz) ;

		point.ax = s.acceleration_W.x();
		point.ay = s.acceleration_W.y();
		point.az = s.acceleration_W.z();
        point.yaw = yawFromVelocity(point.vx, point.vy);
	    point.duration = double(s_next.time_from_start_ns - s.time_from_start_ns) / 1e9;
		traj_converted.push_back(point);
	}
	double budget_till_next_unknown = time_budgetter->calc_budget_till_closest_unknown(traj_converted, closest_unknown_point, drone_position);
	//ROS_ERROR_STREAM("================budget till next unknown"<< budget_till_next_unknown);

	bool got_budget = budget_till_next_unknown > .6;
	if (!got_budget){
		//unknown_budget_failed = true;
		ROS_ERROR_STREAM("--------------------------- didn't got teh budget");
	}
	return got_budget;
}




/*
MotionPlanner::get_drones_distance_to_closeset_unknown(piecewise_trajectory& piecewise_path)	{
	double distance_to_unknown = nan("");
	for (auto it=piecewise_path.begin(); it+1 !=piecewise_path.end(); it++){
		graph::node n1 = {it->x, it->y, it->z};
		graph::node n2 = {(it+1)->x, (it+1)->y, (it+1)->z};
		geometry_msgs::Point closest_unknown_point;
		collision(octree, n1, n2, closest_unknown_point);
		distance_to_unkonwn = calc_vec_magnitude(drone->position().x - closest_unknown_point.x, drone->position().y - closest_unknown_point.y, drone->position().z - closest_unknown_point.z);
		if (!isnan(distance_to_uknown)) {
			return distance_to_unknown;
		}
	}
	return distance_to_unknown;
}
*/
bool  MotionPlanner::ppl_inbound_check(piecewise_trajectory& piecewise_path){
	for (auto it=piecewise_path.begin(); it!=piecewise_path.end(); it++){
		graph::node n1 = {it->x, it->y, it->z};
		if (out_of_bounds_lax(n1)){
			return false;
		}
	}
	return true;
}



MotionPlanner::smooth_trajectory MotionPlanner::smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree, Eigen::Vector3d initial_velocity, Eigen::Vector3d initial_acceleration,
	mavbench_msgs::planner_info &closest_unknown_way_point)
{

	//ROS_INFO_STREAM("begining of smoothening");
    ros::Time smoothening_start_time = ros::Time::now();
	// Variables for visualization for debugging purposes
	double distance = 0.5; 
	std::string frame_id = "world";

	// deep copy the piecewise trajectory
	vector<graph::node> piecewise_path_copy_of_original;
	for (auto it=piecewise_path.begin(); it!=piecewise_path.end(); it++){
		piecewise_path_copy_of_original.push_back(*it);
	}

	// Setup optimizer
	mav_trajectory_generation::Vertex::Vector vertices;
	const int dimension = 3;
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
	
	// Convert roadmap path to optimizer's path format
	mav_trajectory_generation::Vertex start_v(dimension), end_v(dimension);
	//start_v.makeStartOrEnd(Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z), derivative_to_optimize);
   	start_v.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, initial_velocity);
   	//start_v.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(3, 0, 0));
    start_v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z));
    
    end_v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(piecewise_path.back().x, piecewise_path.back().y, piecewise_path.back().z));
   	end_v.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    ///Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z));
    //end_v.makeStartOrEnd(Eigen::Vector3d(piecewise_path.back().x, piecewise_path.back().y, piecewise_path.back().z), derivative_to_optimize);

	vertices.push_back(start_v);
	for (auto it = piecewise_path.begin()+1; it+1 != piecewise_path.end(); ++it) {
		mav_trajectory_generation::Vertex v(dimension);
		v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(it->x, it->y, it->z));
		vertices.push_back(v);
	}
	vertices.push_back(end_v);

	// Parameters used to calculate how quickly the drone can move between vertices
	const double magic_fabian_constant = 6.5; // A tuning parameter.

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);

	// Optimize until no collisions are present
	bool col;
    int smoothening_ctr = 0;	
    ros::Duration smoothening_time_so_far;
    //blah reactive the bellow after data collection
    g_smoothening_budget = (SA_time_budget_to_enforce - follow_trajectory_worse_case_latency) - (ros::Time::now() - deadline_setting_time).toSec();  // whatever is left of the budget
//    ROS_INFO_STREAM("--------------- remaning time is "<< g_smoothening_budget);
    //g_smoothening_budget = min(3.0,  SA_time_budget_to_enforce - (ros::Time::now() - deadline_setting_time).toSec());

	bool first_unknown_collected = false;
	bool too_close_to_unknown = false;
	//bool got_enough_budget_for_next_SA_itr = false;
    int resolution_ratio = (int)(map_res/octree->getResolution());
    int depth_to_look_at = 16 - (int)log2((double)resolution_ratio);
	Eigen::VectorXd first_segment;
	mav_trajectory_generation::Trajectory traj;
	bool break_all = false;
	do {

		smoothening_time_so_far = (ros::Time::now() - smoothening_start_time);
		if(smoothening_time_so_far.toSec() > ros::Duration(g_smoothening_budget).toSec()){ break;}

		first_unknown_collected = false;
		col = false;
		auto segment_times = estimateSegmentTimes(vertices, v_max__global, a_max__global, magic_fabian_constant);
		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();

		smoothening_time_so_far = (ros::Time::now() - smoothening_start_time);
		if(smoothening_time_so_far.toSec() > ros::Duration(g_smoothening_budget).toSec()){ break;}

		mav_trajectory_generation::Segment::Vector segments;
		opt.getSegments(&segments);


		smoothening_time_so_far = (ros::Time::now() - smoothening_start_time);
		if(smoothening_time_so_far.toSec() > ros::Duration(g_smoothening_budget).toSec()){ break;}

		// Loop through the vector of segments looking for collisions
		graph::node n1, n2;
		graph::node collision_n1, collision_n2;
		auto last_key = octree->coordToKey(octomap::point3d(-100, -100, -100), depth_to_look_at);

		double	planned_path_in_future_time  = 0;
		for (int i = 0; !col && i < segments.size(); ++i) {
			// ROS_INFO("Looping through segments...");
			const double time_step = 0.1;
			double segment_len = segments[i].getTime();


			smoothening_time_so_far = (ros::Time::now() - smoothening_start_time);
			if(smoothening_time_so_far.toSec() > ros::Duration(g_smoothening_budget).toSec()){ break_all = true; break;}

			auto segment_start = *(piecewise_path.begin() + i);
			auto segment_end = *(piecewise_path.begin() + i + 1);

			// Step through each individual segment, at increments of "time_step" seconds, looking for a collision
			mavbench_msgs::planner_info closest_unknown_point;
			for (double t = 0; t < segment_len - time_step; t += time_step) {
                // ROS_INFO("Stepping through individual...");
				if (t ==0 && i ==0){
					first_segment = segments[i].evaluate(0);
				}
				auto pos1 = segments[i].evaluate(t);
				auto pos2 = segments[i].evaluate(t + time_step);

				n1 = {pos1.x(), pos1.y(), pos1.z()};
				n2 = {pos2.x(), pos2.y(), pos2.z()};
				if (pos1.x() == pos2.x() &&
						pos1.y() == pos2.y() &&
						pos1.z() == pos2.z()){
					ROS_ERROR_STREAM("EQUAL positions detection");
				}

				n1 = {pos1.x(), pos1.y(), pos1.z()};
				n2 = {pos2.x(), pos2.y(), pos2.z()};
				// Check for a collision between two near points on the segment

                //if (out_of_bounds_lax(n1) || out_of_bounds_lax(n2) || collision(octree, n1, n2, "smoothener")) {
				// use some caching to avoid traversing the tree
				auto this_key = octree->coordToKey(octomap::point3d(n1.x, n1.y, n1.z), depth_to_look_at);
				if ( this_key != last_key || (i == 0 && t == 0)){ // first found skip
					last_key = this_key;
				}else{
					continue;
				}


				if (out_of_bounds_lax(n1) || out_of_bounds_lax(n2) || collision(octree, n1, n2, closest_unknown_point)) {
                	/*
                	if (out_of_bounds_lax(n1) || out_of_bounds_lax(n2)){
                		ROS_INFO_STREAM("out of bound n1");
                	}else{
                		ROS_INFO_STREAM("collision");
                		collision(octree, n1, n2);
                	}
                	*/
                	// Add a new vertex in the middle of the segment we are currently on
					mav_trajectory_generation::Vertex middle(dimension);

					double middle_x = (segment_start.x + segment_end.x) / 2;
					double middle_y = (segment_start.y + segment_end.y) / 2;
					double middle_z = (segment_start.z + segment_end.z) / 2;



					middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(middle_x, middle_y, middle_z));

					vertices.insert(vertices.begin()+i+1, middle);

                    // Add a new node to the piecewise path where the vertex is
                    graph::node middle_node = {middle_x, middle_y, middle_z};
					piecewise_path.insert(piecewise_path.begin()+i+1, middle_node);

					col = true;

					break;
				}else{
					if (!isnan(closest_unknown_point.x) && !first_unknown_collected){
						first_unknown_collected = true;
						closest_unknown_way_point.x = closest_unknown_point.x;
						closest_unknown_way_point.y = closest_unknown_point.y;
						closest_unknown_way_point.z = closest_unknown_point.z;
					}
				}
                /*
                else{
                		ROS_INFO_STREAM("succeed at smoothening some parts");
				}
                */
                //}
			}

			planned_path_in_future_time += segments[i].getTime();
			if (planned_path_in_future_time > max_time_budget){ // if we have lookinto future long enough that we have passed the worst case computation time,
				break;
			}
		}

		if (break_all){
			break;
		}
		smoothening_ctr++;
     smoothening_time_so_far = (ros::Time::now() - smoothening_start_time);
     /*
     if (smoothening_time_so_far.toSec() > ros::Duration(g_smoothening_budget).toSec()){
    	 bool now = 1;
     }
	*/

     //double distance_to_unknown = calc_vec_magnitude(drone->position().x - closest_unknown_way_point.x, drone->position().y - closest_unknown_way_point.y, drone->position().z - closest_unknown_way_point.z);
     //too_close_to_unknown = distance_to_unknown < 2;
     // making sure we have enough budget for the next iterations to prevent stoppage
         //got_enough_budget_for_next_SA_itr = time_budgetter.calc_budget_till_closest_unknown(cur_point, closest_unknown_point);


     if (col){ // only for debugging
    	 multiDOFpoint smoothener_collision_wp_1;
    	 /*
    	 smoothener_collision_wp_1.x = n1.x;
    	 smoothener_collision_wp_1.y = n1.y;
    	 smoothener_collision_wp_1.z = n1.z;

    	 multiDOFpoint smoothener_collision_wp_2;
    	 smoothener_collision_wp_2.x = n2.x;
    	 smoothener_collision_wp_2.y = n2.y;
    	 smoothener_collision_wp_2.z = n2.z;
//    	 auto marker = get_marker(smoothener_collision_wp_1, smoothener_collision_wp_2);

    	 */
    	 piecewise_trajectory collision_path;
    	 collision_path.push_back(n1);
    	 n2.x += 1;
    	 n2.y += 1;
    	 n2.z += 1;
    	 collision_path.push_back(n2);
    	 collision_path.push_back(n2);
    	 visualization_msgs::MarkerArray collision_traj_markers;
    	 draw_piecewise(collision_path, 1, &collision_traj_markers);
    	 collision_traj_vis_pub.publish(collision_traj_markers);
    	 //mav_trajectory_generation::Trajectory collision_traj;
    	 //mav_trajectory_generation::drawMavTrajectory(collision_traj, distance, frame_id, &collision_smooth_traj_markers);

    	 //smoothener_collision_marker_pub.publish(marker);
     }
     //ROS_ERROR_STREAM("smoothening_ctr"<<smoothening_ctr << " smoothening tim so far"<<smoothening_time_so_far.toSec()<<" while budget is"<<ros::Duration(g_smoothening_budget).toSec());
	 opt.getTrajectory(&traj);

	 if((ros::Time::now() - smoothening_start_time).toSec() < ros::Duration(g_smoothening_budget).toSec()){ // if enough budget for this iteration

		 if(!got_enough_budget_for_next_SA_itr(closest_unknown_way_point, traj)){
			 if (drone->velocity().linear.x < .1 && drone->velocity().linear.y <.1 && drone->velocity().linear.z <.1){ // already stopped so no path
				 closest_unknown_way_point.planning_status = "smoothener_failed";
				 closest_unknown_pub.publish(closest_unknown_way_point);
			 }else{
				 closest_obstacle_on_path_way_point.planning_status = "smoothener_failed";
				 closest_unknown_pub.publish(closest_obstacle_on_path_way_point);
			 }
			 ros::param::set("/set_closest_unknown_point", true);
			 //unknown_budget_failed= true;
			 ROS_INFO_STREAM("sending empty trajectory");
			 return smooth_trajectory();
		 }
		 if (col){
			 continue;
		 }else{
			 break;
		 }
	 }else{
		 break;
	 }
	} while(true);
	/*
	while ((smoothening_time_so_far.toSec() < ros::Duration(g_smoothening_budget).toSec()) &&
			(col || !got_enough_budget_for_next_SA_itr(closest_unknown_way_point, traj)));
//    		smoothening_ctr < 15);
            //ros::Time::now() < g_start_time+ros::Duration(g_ppl_time_budget));
	*/



    smoothening_time_so_far = (ros::Time::now() - smoothening_start_time);
	if (col || smoothening_time_so_far.toSec() >= ros::Duration(g_smoothening_budget).toSec()) {
		closest_unknown_way_point.x = nan("");
		closest_unknown_way_point.y = nan("");
		closest_unknown_way_point.z = nan("");
		closest_unknown_way_point.planning_status = "smoothener_failed";
		return smooth_trajectory();
    }else {
		graph::node n1 = {drone->position().x, drone->position().y, drone->position().z};
		graph::node n2 = {first_segment.x(), first_segment.y(), first_segment.z()};
		auto dummy_graph_node_ptr = new graph::node();
		double dx = n2.x - n1.x;
		double dy = n2.y - n1.y;
		double dz = n2.z - n1.z;
		double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
		if (distance > map_res){
			if(collision(octree, n1, n2, dummy_graph_node_ptr)){
			closest_unknown_way_point.x = nan("");
			closest_unknown_way_point.y = nan("");
			closest_unknown_way_point.z = nan("");
			closest_unknown_way_point.planning_status = "smoothener_failed";
			return smooth_trajectory();
			}
		}

    }


    // Return the collision-free smooth trajectory
	opt.getTrajectory(&traj);
	int num_of_optimums_till_idx = 3; // return the accumulated number of optimums until this index. The idea is that
									  // we are only concern with suboptimal (oscilating) paths at the intial vertecis of the trajectory. Such behavior is the result of sudden
									  // direction changes to dampen the high velocity while meeting intricate maneuvers. Note that if we don't replan the path is very suboptimal which
									  // means that we'll loose the benefits of high speed capability

	// figure out how many extra vertecies were (artifically) added (to avoid the obstacle collision introduced by smoothening) up to the
	// num_of_optimums_till_idx and add the value when passiging to  get_num_of_optimums.
	int idx_of_interest = std::min(int(piecewise_path.size() - 1), num_of_optimums_till_idx-1);
	int starting_idx = idx_of_interest;
	int counting_idx = starting_idx;
	for (auto it=piecewise_path.begin()+ starting_idx; it!=piecewise_path.end(); it++){
		if (it->x == piecewise_path_copy_of_original[idx_of_interest].x && it->y == piecewise_path_copy_of_original[idx_of_interest].y
				&& it->z == piecewise_path_copy_of_original[idx_of_interest].z){
			break;
		}else{
			counting_idx +=1;
		}
	}

	closest_unknown_way_point.planning_status = "success";
	/*
	cout<<"--------------------------"<<endl;
	cout<<"====================== modified"<<endl;
	for (auto it=piecewise_path.begin(); it!=piecewise_path.end(); it++){
		cout<< "("<<it->x<< "," <<it->y<<","<<it->z<< ") ";
	}
	cout<<endl;
	for (auto it=piecewise_path_copy_of_original.begin(); it!=piecewise_path_copy_of_original.end(); it++){
		cout<< "("<<it->x<< "," <<it->y<<","<<it->z<< ") ";
	}
	cout<<endl;
	*/

	int num_of_optimums = get_num_of_optimums(piecewise_path, traj, counting_idx);
	ROS_INFO_STREAM("Smoothened path! with "<< num_of_optimums << " num of optimums up to " << counting_idx << " index");
	if (num_of_optimums <= 3*(num_of_optimums_till_idx-1)) {
		planned_optimally = true;
	}else{
		ROS_INFO_STREAM("---planning suboptimally--");
		planned_optimally = false;
	}

	// Visualize path for debugging purposes
	mav_trajectory_generation::drawMavTrajectory(traj, distance, frame_id, &smooth_traj_markers);
	mav_trajectory_generation::drawVertices(vertices, frame_id, &piecewise_traj_markers);
//	assert(first_unknown_collected);
	return traj;
}




void MotionPlanner::postprocess(piecewise_trajectory& path)
{
    // We use a greedy approach to shorten the path here.
    // We connect non-adjacent nodes in the path that do not have collisions.
    for (auto it = path.begin(); it != path.end()-1; ) {
        bool shortened = false;
        for (auto it2 = path.end()-1; it2 != it+1 && !shortened; --it2) {
            if (!collision(octree, *it, *it2)
                && out_of_bounds_strict(*it) == out_of_bounds_strict(*it2)) {
                it = path.erase(it+1, it2);
                shortened = true;
            }
        }

        if (!shortened)
            ++it;
    }
}


static graph create_lawnMower_path(geometry_msgs::Point start, int width, int length, int n_pts_per_dir, octomap::OcTree *octree, graph::node_id &start_id, graph::node_id &goal_id)

{
	
    // *** F:DN variables 
    graph roadmap;
	bool success = true;
    double x_step = double(length)/double(n_pts_per_dir);
    double y_step = double(width)/double(n_pts_per_dir);
    graph::node_id cur_node_id, prev_node_id;
    double x = start.x;
    double y = start.y;


	//ROS_INFO("starting piecewise_path");
	ROS_INFO("starting x,y is %f %f", x, y);
	//start_id = -1, goal_id = -2;
	
    //*** F:DN generate all the nodes
        for (int i = 0 ; i < n_pts_per_dir; i++) {
            for (int j = 0 ; j < n_pts_per_dir; j++) {

                ROS_INFO("%f %f", x, y);
                if (i==0 && j==0) {
                    graph::node_id cur_node_id = roadmap.add_node(
                            x, y, start.z);
                }
                else{
                    graph::node_id cur_node_id = roadmap.add_node(x, y, start.z);
                    roadmap.connect(cur_node_id, prev_node_id, 
                            dist(roadmap.get_node(cur_node_id), 
                                roadmap.get_node(prev_node_id)));
                }
                ROS_INFO("id is %d", int(cur_node_id)); 
                y +=y_step;
                prev_node_id = cur_node_id; 
            }
	   
            /*
            if ((i+1)  < n_pts_per_dir) { 
                    
                    ROS_INFO("%f %f", x, y);
		    x += (x_step/3); 
	            cur_node_id = roadmap.add_node(x, y, start.z);
		    roadmap.connect(cur_node_id, prev_node_id, 
				    dist(roadmap.get_node(cur_node_id), 
					    roadmap.get_node(prev_node_id)));
		    prev_node_id = cur_node_id; 
                    ROS_INFO("%f %f", x, y);
		    x += (x_step/3); 
		    cur_node_id = roadmap.add_node(x, y, start.z);
		    roadmap.connect(cur_node_id, prev_node_id, 
				    dist(roadmap.get_node(cur_node_id), 
					    roadmap.get_node(prev_node_id)));
		    prev_node_id = cur_node_id; 
		    x += (x_step/3);
	    } 
             */          
	    x += x_step;
	    y_step *= -1;
    }
   
    // ***F:DN returning back the the origin
    cur_node_id = roadmap.add_node(start.x, start.y, start.z);
    roadmap.connect(cur_node_id, prev_node_id, 
            dist(roadmap.get_node(cur_node_id), 
                roadmap.get_node(prev_node_id)));
    /*   
	if (occupied(octree, start.x, start.y, start.z)) {
		ROS_ERROR("Start is already occupied!");
		success = false;
	}
    */ 
    return roadmap;
}


MotionPlanner::piecewise_trajectory MotionPlanner::lawn_mower(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree)
{
	//----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    piecewise_trajectory result;
	graph::node_id start_id, goal_id;
	auto generate_shortest_path = keep_roadmap_intact_plan; // TODO: parameter

    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 
    graph roadmap = create_lawnMower_path(start, width, length, n_pts_per_dir, octree, start_id, goal_id);

    if (roadmap.size() == 0) {
    	ROS_ERROR("PRM could not be initialized.");
    	return result;
    }

    // publish_graph(roadmap); // A debugging function used to publish the roadmap generated, so it can be viewed in rviz

    // Search for a path to the goal in the PRM
    result = generate_shortest_path(roadmap);

    return result;
}


bool MotionPlanner::OMPLStateValidityChecker(const ompl::base::State * state)
{
    namespace ob = ompl::base;

    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

    double x = pos->values[0];
    double y = pos->values[1];
    double z = pos->values[2];

    return !out_of_bounds({x,y,z}) && !occupied(octree, x, y, z);
}


MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status)
{
    return OMPL_plan<ompl::geometric::RRT>(start, goal, octree, status);
}


MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_RRTStar(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status)
{
	//publish_dummy_octomap_vis(octree);
	return OMPL_plan<ompl::geometric::RRTstar>(start, goal, octree, status);
}


MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status)
{
    return OMPL_plan<ompl::geometric::PRM>(start, goal, octree, status);
}

