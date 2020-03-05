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


double total_collision_func = 0;
double potential_distance_to_explore = 0; // the distance that the planner ask to expore (note that this can be bigger than the volume since the volume calculation stops after hitting a an obstacle)
double volume_explored_in_unit_cubes = 0; // volume explored within the piecewise planner
double ppl_vol_idealInUnitCube;
ros::Time g_planning_start_time;
/*
void planner_termination_func(double &volume_explored_so_far, double &volume_explored_threshold){
	bool res = volume_explored_so_far > volume_explored_threshold;
	return res;
}
*/

bool planner_termination_func(){
	bool taking_to_long = (ros::Time::now() - g_planning_start_time).toSec() > 20; // -- this is just to make sure the volume is not gonna be too big
																				   // -- so that we will return
	return (volume_explored_in_unit_cubes > ppl_vol_idealInUnitCube) || taking_to_long;

}

template<class PlannerType>
MotionPlanner::piecewise_trajectory MotionPlanner::OMPL_plan(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree, int& status)
{

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

    // Setup collision checker
    ob::SpaceInformationPtr si = ss.getSpaceInformation();
    si->setStateValidityChecker([this] (const ompl::base::State * state) {
        return this->OMPLStateValidityChecker(state);
    });
    si->setMotionValidator(std::make_shared<OMPLMotionValidator>(this, si));
    si->setup();

    // Set planner
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

    ss.setStartAndGoalStates(start_state, goal_state);

    ss.setup();


	profiling_container.capture("OMPL_planning_time", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
    // Solve for path
    //ob::PlannerStatus solved = ss.solve(g_ppl_time_budget);

	auto planner_termination_obj = ompl::base::PlannerTerminationCondition(planner_termination_func);
	ob::PlannerStatus solved;
    solved = ss.solve(planner_termination_obj);
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

    // profiling, debugging
    profiling_container.capture("OMPL_planning_time", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
	if (DEBUG_RQT) {
		debug_data.header.stamp = ros::Time::now();
		debug_data.OMPL_planning_time = profiling_container.findDataByName("OMPL_planning_time")->values.back();
		motion_planning_debug_pub.publish(debug_data);
	}

	if (status == 1) //only take exact solution
    {

    	profiling_container.capture("OMPL_simplification_time", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
    	ROS_INFO("Solution found!");
        ss.simplifySolution();

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

	motion_planning_initialize_params();
    g_goal_pos.x = g_goal_pos.y = g_goal_pos.z = nan("");

	// Create a new callback queue
	nh.setCallbackQueue(&callback_queue);

	// Topics and services
	get_trajectory_srv_server = nh.advertiseService("/get_trajectory_srv", &MotionPlanner::get_trajectory_fun, this);

	future_col_sub = nh.subscribe("/col_coming", 1, &MotionPlanner::future_col_callback, this);
	octomap_communication_proxy_msg = nh.subscribe("/octomap_communication_proxy_msg", 1, &MotionPlanner::octomap_communication_proxy_msg_cb, this);
	next_steps_sub = nh.subscribe("/next_steps", 1, &MotionPlanner::next_steps_callback, this);

	//octomap_sub = nh.subscribe("/octomap_binary", 1, &MotionPlanner::octomap_callback, this);
	octomap_sub = nh.subscribe("/octomap_binary", 1, &MotionPlanner::octomap_callback, this);

	traj_pub = nh.advertise<mavbench_msgs::multiDOFtrajectory>("multidoftraj", 1);
    timing_msg_from_mp_pub = nh.advertise<mavbench_msgs::response_time_capture> ("/timing_msgs_from_mp", 1);
    motion_planning_debug_pub = nh.advertise<mavbench_msgs::motion_planning_debug>("/motion_planning_debug", 1);

	// for stress testing
	octomap_dummy_pub = nh.advertise<octomap_msgs::Octomap>("octomap_binary_2", 1);

	// for visualizatrion (rviz)
	m_markerPub = nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array_dumy", 1);
	smooth_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
	piecewise_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    goal_rcv_service = nh.advertiseService("goal_rcv", &MotionPlanner::goal_rcv_call_back, this);
	//re = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    if (knob_performance_modeling){
    	capture_size = 1;
    }
}



Drone* MotionPlanner::get_drone() {
	return drone;
}

bool MotionPlanner::goal_rcv_call_back(package_delivery::point::Request &req, package_delivery::point::Response &res){
    first_time_planning_succeeded = false;
	g_goal_pos = req.goal;
	goal_known = true;
}

void MotionPlanner::publish_dummy_octomap_vis(octomap::OcTree *m_octree){
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

    if (m_octree->isNodeOccupied(*it)){
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
            color.a = 1.0; color.r = 1; color.g = 1; color.b = 1;
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


// determine whether it's time to replan
bool MotionPlanner::shouldReplan(const octomap_msgs::Octomap& msg){
	bool replan;
	//std_msgs::Header msg_for_follow_traj;
    //mavbench_msgs::response_time_capture msg_for_follow_traj;
	msg_for_follow_traj.header.stamp = msg.header.stamp;
	if(!first_time_planning_succeeded) {
		msg_for_follow_traj.planning_status = "first_time_planning";
		//msg_for_follow_traj.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
		//msg_for_follow_traj.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
		//msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
		timing_msg_from_mp_pub.publish(msg_for_follow_traj); //send a msg to make sure we update response time
		replanning_reason = First_time_planning;
		replan = true;
	} else if (got_new_next_steps_since_last_attempted_plan){ // only can decide on replanning, if we have the new position of the drone on the track
		if (failed_to_plan_last_time) {
		//ROS_ERROR_STREAM("failed to plan last time , so replan");
		replanning_reason = Failed_to_plan_last_time;
		replan = true;
		} else if (!planned_optimally && (ros::Time::now() - this->last_planning_time).toSec() < 3) {
			replan = true;
		}else if(planned_approximately){
			replanning_reason = Last_plan_approximate;
			replan = true;
			ROS_ERROR_STREAM("approximate planning");
		}
		else if( (ros::Time::now() - this->last_planning_time).toSec() > (float)1/planner_min_freq) {
			replanning_reason = Min_freq_passed;
			//ROS_ERROR_STREAM("long time since last planning, so replan");
			replan = true;
		} else {
			profiling_container.capture("collision_check_for_replanning", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
			bool collision_coming = this->traj_colliding(&g_next_steps_msg);
			profiling_container.capture("collision_check_for_replanning", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
			debug_data.collision_check_for_replanning = profiling_container.findDataByName("collision_check_for_replanning")->values.back();
			if (collision_coming){
				replanning_reason = Collision_detected;
				//ROS_INFO_STREAM("there is a collision");
				profiling_container.capture("replanning_due_to_collision_ctr", "counter", 0, capture_size); // @suppress("Invalid arguments")
				//ROS_ERROR_STREAM("collision comming, so replan");
				replan = true;
			} else{ //this case is for profiling. We send this over to notify the follow trajectory that we made a decision not to plan
				replanning_reason = No_need_to_plan;
				replan = false;
			}
		}
	}

	// for profiling
	if (!replan) { //notify the follow trajectory to erase up to this msg
		msg_for_follow_traj.planning_status = "no_planning_needed";
		ppl_vol_actual = volume_explored_in_unit_cubes*pow(map_res, 3);
		msg_for_follow_traj.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
		msg_for_follow_traj.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
		msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
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

	msg_for_follow_traj.controls = msg->controls;
	msg_for_follow_traj.ee_profiles = msg->ee_profiles;
	msg_for_follow_traj.ee_profiles.actual_time.om_to_pl_ros_oh = (ros::Time::now() - msg->ee_profiles.actual_time.om_pre_pub_time_stamp).toSec();
	planning_start_time_stamp = ros::Time::now();
	g_ppl_time_budget = msg->ee_profiles.expected_time.ppl_latency;
	g_smoothening_budget = msg->ee_profiles.expected_time.ppl_latency;  // -- for now setting it equal to ppl_budget
															   // -- todo: this can be another knob

	// reset all the profiling values
	potential_distance_to_explore = 0;
    volume_explored_in_unit_cubes = 0;
    debug_data.motion_planning_collision_check_volume_explored = 0;
    debug_data.motion_planning_piecewise_volume_explored = 0;
    debug_data.motion_planning_smoothening_volume_explored = 0;
    debug_data.motion_planning_piece_wise_time = 0;
    debug_data.collision_func = 0;

    ppl_vol_ideal = msg->controls.cmds.ppl_vol;

    map_res = msg->controls.cmds.om_to_pl_res;
    ppl_vol_idealInUnitCube = ppl_vol_ideal/(pow(msg->controls.cmds.om_to_pl_res, 3));
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
	profiling_container.capture("octomap_deserialization_time", "start", ros::Time::now(), capture_size);
    octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(msg->oct);
    profiling_container.capture("octomap_deserialization_time", "end", ros::Time::now(), capture_size);
    // -- cast the map
    profiling_container.capture("octomap_dynamic_casting", "start", ros::Time::now(), capture_size);
    octree = dynamic_cast<octomap::OcTree*> (tree);
    profiling_container.capture("octomap_dynamic_casting", "end", ros::Time::now(), capture_size);

    msg_for_follow_traj.ee_profiles.actual_time.om_to_pl_latency =  msg->ee_profiles.actual_time.om_serialization_time +
    		msg_for_follow_traj.ee_profiles.actual_time.om_to_pl_ros_oh +
    		profiling_container.findDataByName("octomap_deserialization_time")->values.back() +
			profiling_container.findDataByName("octomap_dynamic_casting")->values.back();



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
					(ros::Time::now() - msg->header.stamp).toSec(), capture_size);
			profiling_container.capture("om_to_pl_res_knob_modeling", "single",
					msg->controls.cmds.om_to_pl_res, capture_size);
			profiling_container.capture("om_to_pl_vol_actual_knob_modeling", "single",
					msg->ee_profiles.actual_cmds.om_to_pl_vol, capture_size);
			profiling_container.capture("octomap_deserialization_time_knob_modeling", "single",
					profiling_container.findDataByName("octomap_deserialization_time")->values.back(), capture_size);
			profiling_container.capture("octomap_dynamic_casting_knob_modeling", "single",
					profiling_container.findDataByName("octomap_dynamic_casting")->values.back(), capture_size);
			double octomap_to_planner_com_overhead_knob_modeling = profiling_container.findDataByName("octomap_dynamic_casting_knob_modeling")->values.back() +
					profiling_container.findDataByName("octomap_deserialization_time_knob_modeling")->values.back() +
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


	if (DEBUG_VIS){
    	ROS_INFO_STREAM("publishing octomap in motion planner is heavy. It's just used for debuuging. so comment out this block");
    	publish_dummy_octomap_vis(octree);
    }

    if (DEBUG_RQT) {
    		debug_data.header.stamp = ros::Time::now();
    		debug_data.octomap_deserialization_time = profiling_container.findDataByName("octomap_deserialization_time")->values.back();
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
	if (!shouldReplan(msg->oct) && !knob_performance_modeling_for_piecewise_planner){
    	debug_data.motion_planning_collision_check_volume_explored = volume_explored_in_unit_cubes*pow(map_res, 3);
    	debug_data.motion_planning_potential_distance_to_explore = potential_distance_to_explore;
    	return;
    }

    this->last_planning_time = ros::Time::now();
    g_planning_start_time = this->last_planning_time;
    // if already have a plan, but not colliding, plan again
    profiling_container.capture("motion_planning_time_total", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
    bool planning_succeeded = this->motion_plan_end_to_end(msg->header.stamp, g_goal_pos);
    profiling_container.capture("motion_planning_time_total", "end", ros::Time::now(), capture_size); // @suppress("Invalid arguments")
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

	ros::param::get("/ppl_time_budget", g_ppl_time_budget);
	ros::param::get("/smoothening_budget", g_smoothening_budget);

    //auto hook_start_t = ros::Time::now();
    //g_start_pos = req.start;
    //g_goal_pos = req.goal;


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
    piecewise_path = motion_planning_core(req.start, req.goal, req.width, req.length, req.n_pts_per_dir, octree, status);
    profiling_container.capture("motion_planning_piece_wise_time", "end", ros::Time::now(), capture_size);
    if (DEBUG_RQT) {
    		debug_data.header.stamp = ros::Time::now();
    		debug_data.motion_planning_piece_wise_time = profiling_container.findDataByName("motion_planning_piece_wise_time")->values.back();
    		//motion_planning_debug_pub.publish(debug_data);
    }


    debug_data.motion_planning_piecewise_volume_explored = volume_explored_in_unit_cubes*pow(map_res,3);
    debug_data.collision_func = total_collision_func;
    ppl_vol_actual = volume_explored_in_unit_cubes*pow(map_res, 3);
    ROS_INFO_STREAM("actuall volume"<< ppl_vol_actual << " volume explored in unit cubes"<< volume_explored_in_unit_cubes<<  "volume expected in unit cube"<<ppl_vol_idealInUnitCube << " volume expected"<< ppl_vol_ideal<< "octree res"<< map_res);

    //ROS_INFO_STREAM("already flew backward"<<already_flew_backward);
    if (piecewise_path.empty()) {
    	profiling_container.capture("motion_planning_piecewise_failure_cnt", "counter", 0, capture_size); // @suppress("Invalid arguments")
    	if (notified_failure){ //so we won't fly backward multiple times
    		//std_msgs::Header msg_for_follow_traj;
    		if (!measure_time_end_to_end) { msg_for_follow_traj.header.stamp = ros::Time::now(); }
			else{ msg_for_follow_traj.header.stamp = req.header.stamp; }
			msg_for_follow_traj.planning_status = "piecewise_planning_failed";
			msg_for_follow_traj.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
			msg_for_follow_traj.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
			msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
			timing_msg_from_mp_pub.publish(msg_for_follow_traj); //send a msg to make sure we update responese timne
    		return false;
    	}

    	ROS_ERROR("Empty path returned");
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
        }else if (status == 0 || status == 3){ //if couldn't find an exact path within the time frame reverse
        	//res.multiDOFtrajectory.planning_status = Short_time_failure;
        	res.multiDOFtrajectory.planning_status = "piecewise_planning_failed";
        	res.multiDOFtrajectory.reverse = false;
        	res.multiDOFtrajectory.stop = true;
        }else{
        	cout<<status<<endl;
        	ROS_INFO_STREAM("this state shouldn't happpen"<< status);
        	exit(0);
        }

        notified_failure = true;
        if (!measure_time_end_to_end) { res.multiDOFtrajectory.header.stamp = ros::Time::now(); }
        else{ res.multiDOFtrajectory.header.stamp = req.header.stamp; }
        res.multiDOFtrajectory.planning_status = "piecewise_planning_failed";
        res.multiDOFtrajectory.controls = msg_for_follow_traj.controls;
        res.multiDOFtrajectory.ee_profiles = msg_for_follow_traj.ee_profiles;
        res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
        res.multiDOFtrajectory.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
        res.multiDOFtrajectory.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
        traj_pub.publish(res.multiDOFtrajectory);
        return false;
    }

    profiling_container.capture("RRT_path_length_normalized_to_direct_path", "single",
    		calculate_path_length(piecewise_path)/calc_vec_magnitude(drone->position().x - req.goal.x, drone->position().y - req.goal.y, drone->position().z - req.goal.z),
    		capture_size);

    if (motion_planning_core_str != "lawn_mower") {
    	profiling_container.capture("piecewise_path_post_process", "start", ros::Time::now(), capture_size);
    	postprocess(piecewise_path);
    	profiling_container.capture("piecewise_path_post_process", "end", ros::Time::now(), capture_size);
    }


    res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
    volume_explored_in_unit_cubes = 0;
    // Smoothen the path and build the multiDOFtrajectory response
    //ROS_INFO("Smoothenning...");
    profiling_container.capture("motion_planning_smoothening_time", "start", ros::Time::now(), capture_size);
    smooth_path = smoothen_the_shortest_path(piecewise_path, octree, 
                                    Eigen::Vector3d(req.twist.linear.x,
                                        req.twist.linear.y,
                                        req.twist.linear.z), 
                                    Eigen::Vector3d(req.acceleration.linear.x,
                                                    req.acceleration.linear.y,
                                                    req.acceleration.linear.z));
    profiling_container.capture("motion_planning_smoothening_time", "end", ros::Time::now(), capture_size);
    if (DEBUG_RQT) {
    		debug_data.header.stamp = ros::Time::now();
    		debug_data.motion_planning_smoothening_time = profiling_container.findDataByName("motion_planning_smoothening_time")->values.back();
    		//motion_planning_debug_pub.publish(debug_data);
	}
    debug_data.motion_planning_smoothening_volume_explored = volume_explored_in_unit_cubes*pow(map_res,3);

    if (smooth_path.empty()) {
    	ROS_ERROR("Path could not be smoothened successfully");
    	profiling_container.capture("motion_planning_smoothening_failure_cnt", "counter", 0, capture_size); // @suppress("Invalid arguments")
    	if (notified_failure){ //so we won't fly backward multiple times
    		//mavbench_msgs::response_time_capture msg_for_follow_traj;
    		//std_msgs::Header msg_for_follow_traj;
			if (!measure_time_end_to_end) { msg_for_follow_traj.header.stamp = ros::Time::now(); }
			else{ msg_for_follow_traj.header.stamp = req.header.stamp; }
			msg_for_follow_traj.planning_status = "smoothening_failed";
			//msg_for_follow_traj.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
			msg_for_follow_traj.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
			msg_for_follow_traj.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
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
        res.multiDOFtrajectory.planning_status = "smoothening_failed"; //profiling
        notified_failure = true;
        if (!measure_time_end_to_end) { res.multiDOFtrajectory.header.stamp = ros::Time::now(); }
        else{ res.multiDOFtrajectory.header.stamp = req.header.stamp; }
        res.multiDOFtrajectory.controls = msg_for_follow_traj.controls;
        res.multiDOFtrajectory.ee_profiles = msg_for_follow_traj.ee_profiles;
        //res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
        res.multiDOFtrajectory.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
        res.multiDOFtrajectory.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
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
    res.multiDOFtrajectory.ee_profiles = msg_for_follow_traj.ee_profiles;
    res.multiDOFtrajectory.ee_profiles.actual_time.ppl_latency = (ros::Time::now() - planning_start_time_stamp).toSec();
    //res.multiDOFtrajectory.ee_profiles.actual_time.pl_pre_pub_time_stamp =  ros::Time::now();
    res.multiDOFtrajectory.ee_profiles.actual_cmds.ppl_vol = ppl_vol_actual;
    traj_pub.publish(res.multiDOFtrajectory);
    smooth_traj_vis_pub.publish(smooth_traj_markers);
    piecewise_traj_vis_pub.publish(piecewise_traj_markers);

    g_number_of_planning++; 
    res.path_found = true;
    planned_approximately = (status == 3);
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
    multiDOFpoint mdofp = trajectory_at_time(g_next_steps_msg, g_smoothening_budget + g_ppl_time_budget);
	//multiDOFpoint mdofp = trajectory_at_time(g_next_steps_msg, .2);


    // Shift the drone's planned position at time "g_ppl_time_budget" seconds
    // by its current position
    auto current_pos = drone.position();
    auto planned_point = g_next_steps_msg.points[0];
    if (planned_point.vx == 0 && planned_point.vy == 0 && planned_point.vz == 0){
    	mdofp.x = current_pos.x;
    	mdofp.y = current_pos.y;
    	mdofp.z = current_pos.z;
    }else{
    	mdofp.x += current_pos.x - planned_point.x;
    	mdofp.y += current_pos.y - planned_point.y;
    	mdofp.z += current_pos.z - planned_point.z;
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
}

void MotionPlanner::motion_planning_initialize_params()
{
	if(!ros::param::get("/DEBUG_RQT", DEBUG_RQT)){
      ROS_FATAL_STREAM("Could not start motion_planning DEBUG_RQT not provided");
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
    ros::param::get("/motion_planner/v_max", v_max__global);
    ros::param::get("/motion_planner/a_max", a_max__global);

    ros::param::get("/knob_performance_modeling", knob_performance_modeling);
    ros::param::get("/knob_performance_modeling_for_om_to_pl", knob_performance_modeling_for_om_to_pl);
    ros::param::get("/knob_performance_modeling_for_om_to_pl", knob_performance_modeling_for_om_to_pl);


    ros::param::get("/motion_planner/measure_time_end_to_end", measure_time_end_to_end);
    ros::param::get("ros_DEBUG", DEBUG__global);
    
    if(!ros::param::get("/motion_planner/planner_min_freq", planner_min_freq)) {
        ROS_FATAL("Could not start motion_planner node node. planner_min_freq is missing");
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

    return !x_correct || !y_correct || !z_correct;
}


bool MotionPlanner::collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
{
	profiling_container.capture("collision_func", "start", ros::Time::now(), capture_size); // @suppress("Invalid arguments")

	if (motion_planning_core_str == "lawn_mower")
        return false;

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

        if (octree->castRayAndCollectVolumeTraveresed(start, direction, end, true, distance, volume_explored_in_unit_cubes_)) {
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



bool MotionPlanner::traj_colliding(mavbench_msgs::multiDOFtrajectory *traj)
{
	if (octree == nullptr || traj->points.size() < 1) {
        return false;
    }

    bool col = false;
    graph::node *end_ptr = new graph::node();
    for (int i = 0; i < traj->points.size() - 1; ++i) {
        auto& pos1 = traj->points[i];
        auto& pos2 = traj->points[i+1];
        graph::node n1 = {pos1.x, pos1.y, pos1.z};
        graph::node n2 = {pos2.x, pos2.y, pos2.z};
        if (collision(octree, n1, n2, end_ptr)) {
            col = true;
            break;
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




MotionPlanner::smooth_trajectory MotionPlanner::smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree, Eigen::Vector3d initial_velocity, Eigen::Vector3d initial_acceleration)
{

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
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
	
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
    do {
		col = false;

		// Estimate the time the drone should take flying between each node
		auto segment_times = estimateSegmentTimes(vertices, v_max__global, a_max__global, magic_fabian_constant);

        // for (auto& el : segment_times)
        //     el *= 0.5;

		// Optimize and create a smooth path from the vertices
		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();

		// Return all the smooth segments in the path
		// (Each segment goes from one of the original nodes to the next one in the path)
		mav_trajectory_generation::Segment::Vector segments;
		opt.getSegments(&segments);

		// Loop through the vector of segments looking for collisions
		for (int i = 0; !col && i < segments.size(); ++i) {
            // ROS_INFO("Looping through segments...");
			const double time_step = 0.1;
			double segment_len = segments[i].getTime();

			auto segment_start = *(piecewise_path.begin() + i);
			auto segment_end = *(piecewise_path.begin() + i + 1);

			// Step through each individual segment, at increments of "time_step" seconds, looking for a collision
			for (double t = 0; t < segment_len - time_step; t += time_step) {
                // ROS_INFO("Stepping through individual...");
				auto pos1 = segments[i].evaluate(t);
				auto pos2 = segments[i].evaluate(t + time_step);

				graph::node n1 = {pos1.x(), pos1.y(), pos1.z()};
				graph::node n2 = {pos2.x(), pos2.y(), pos2.z()};

				// Check for a collision between two near points on the segment

                //if (motion_planning_core_str != "lawn_mower") {
                if (out_of_bounds_lax(n1) || out_of_bounds_lax(n2) || collision(octree, n1, n2)) {

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
				}
                /*
                else{
                		ROS_INFO_STREAM("succeed at smoothening some parts");
				}
                */
                //}
			}
		}
     smoothening_ctr++;	
     smoothening_time_so_far = (ros::Time::now() - smoothening_start_time);
     if (smoothening_time_so_far.toSec() > ros::Duration(g_smoothening_budget).toSec()){
    	 bool now = 1;
     }

     ROS_ERROR_STREAM("smoothening_ctr"<<smoothening_ctr << " smoothening tim so far"<<smoothening_time_so_far.toSec());
    } while (col &&
            smoothening_time_so_far.toSec() < ros::Duration(g_smoothening_budget).toSec());
    		//smoothening_ctr < 5);
            //ros::Time::now() < g_start_time+ros::Duration(g_ppl_time_budget));

    if (col || smoothening_time_so_far.toSec() >= ros::Duration(g_smoothening_budget).toSec()) {
        return smooth_trajectory();
    }

	// Return the collision-free smooth trajectory
	mav_trajectory_generation::Trajectory traj;
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

