#include <ros/ros.h>

// Standard headers
#include <cstdlib>
#include <cmath>
#include <random>
#include <vector>
#include <iostream>
#include <functional>
#include <limits>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
// MAVBench headers
#include "common.h"
#include "graph.h"
#include "global_planner.h"
#include "package_delivery/get_trajectory.h"
#include "package_delivery/point.h"
#include "timer.h"

#include <profile_manager.h>
#include <profile_manager/profiling_data_srv.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>
#include <mavbench_msgs/motion_planning_debug.h>
#include <mavbench_msgs/octomap_aug.h>

// Misc messages
#include <geometry_msgs/Point.h>

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

// Trajectory smoothening headers
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

// OMPL specific headers
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

#include <datacontainer.h>
#include <iostream>
#include <vector>

using namespace std;

class MotionPlanner
{
friend class OMPLMotionValidator;

public:
    // Type definitions
	using piecewise_trajectory = vector<graph::node>;
    using smooth_trajectory = mav_trajectory_generation::Trajectory;


    int get_num_of_optimums(piecewise_trajectory& piecewise, mav_trajectory_generation::Trajectory &traj, int num_of_optimums_till_idx);
    int find_optimum_vec_index(mavbench_msgs::multiDOFpoint cur_point, piecewise_trajectory& vertecis, int cur_vertex_idx);

    MotionPlanner(octomap::OcTree * octree_, Drone * drone_);
    void log_data_before_shutting_down();

    // does  what ROS::spinceOnce which is to "call all the callback functions if there is any packets in the queues"
    void spinOnce();
    // planning end to end
    bool motion_plan_end_to_end(ros::Time invocation_time, geometry_msgs::Point goal);
    octomap::OcTree *getOctree();
    Drone *get_drone();

    double calculate_path_length(piecewise_trajectory piecewise_path); //calculate the length of the path generated by RRT


private:
	mavbench_msgs::motion_planning_debug debug_data = {};
    ProfileManager profile_manager;
    DataContainer profiling_container;
    bool measure_time_end_to_end;
    bool got_new_next_steps_since_last_attempted_plan = false; //only plan if you have recieved new next steps otherwise, we'll predict wrong

    ros::Publisher motion_planning_debug_pub;
    // ***F:DN call back for octomap msgs
    void octomap_callback(const mavbench_msgs::octomap_aug::ConstPtr& msgs);

    // generating the trajectory (including piecewise generation and smooothening)
    bool get_trajectory_fun(package_delivery::get_trajectory::Request &req, package_delivery::get_trajectory::Response &res);

    // ***F:DN Plans new paths when collisions are detected
    void future_col_callback(const mavbench_msgs::future_collision::ConstPtr& msg);

    // ***F:DN Keep track of the drone's next moves
    void next_steps_callback(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg);

    // is the trajecotry colliding
    bool traj_colliding(mavbench_msgs::multiDOFtrajectory *traj);


    void octomap_communication_proxy_msg_cb(const std_msgs::Header& msg);

    // ***F:DN Find where the drone will be in a few seconds
    void get_start_in_future(Drone& drone, geometry_msgs::Point& start, geometry_msgs::Twist& twist, geometry_msgs::Twist& acceleration);

    // ***F:DN Create a grid-based lawn-mower-like path
    piecewise_trajectory lawn_mower(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree);

    // ***F:DN Use the RRT sampling method from OMPL to find a piecewise path
    piecewise_trajectory OMPL_RRT(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status);

    // ***F:DN Use bi-directonal RRT from OMPL to find a piecewise path
    piecewise_trajectory OMPL_RRTStar(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status);

    // ***F:DN Use the PRM sampling method from OMPL to find a piecewise path
    piecewise_trajectory OMPL_PRM(geometry_msgs::Point start, geometry_msgs::Point goal, int width, int length, int n_pts_per_dir, octomap::OcTree * octree, int &status);

    // *** F:DN Checks whether a cell in the occupancy grid is occupied.
    bool occupied(octomap::OcTree * octree, double x, double y, double z);

    // *** F:DN Checks whether a cell in the occupancy grid is known.
    bool known(octomap::OcTree * octree, double x, double y, double z);

    // *** F:DN Checks whether there is a collision between two nodes in the occupancy grid.
    bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr = nullptr);

    // whether the drone is already following a trajectory
    bool haveExistingTraj(mavbench_msgs::multiDOFtrajectory *traj);


    bool shouldReplan(const octomap_msgs::Octomap& msg);

    // *** F:DN Checks whether there is a collision between two nodes in the occupancy grid.
//    bool collision(octomap::OcTree * octree, const multiDOFpoint &n1, const multiDOFpoint &n2, graph::node * end_ptr);

    // *** F:DN Checks whether a cell in the occupancy grid is occupied.
    bool out_of_bounds(const graph::node& pos);

    // *** F:DN Checks whether a cell in the occupancy grid is occupied.
    bool out_of_bounds_strict(const graph::node& pos);

    // *** F:DN Checks whether a cell in the occupancy grid is occupied.
    bool out_of_bounds_lax(const graph::node& pos);

    // *** F:DN Optimize and smoothen a piecewise path without causing any new collisions.
    smooth_trajectory smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree, Eigen::Vector3d initial_velocity, Eigen::Vector3d initial_acceleration);

    // ***F:DN Build the response to the service from the smooth_path
    void create_response(package_delivery::get_trajectory::Response &res, const smooth_trajectory& smooth_path);

    // ***F:DN Post-process piecewise path to optimize it.
    void postprocess(piecewise_trajectory& path);

    // *** F:DN initializing all the global variables 
    void motion_planning_initialize_params();

    // *** F:DN Check the validity of states for OMPL planners
    bool OMPLStateValidityChecker(const ompl::base::State * state);

    // *** F:DN A flexible wrapper for OMPL planners
    template<class PlannerType>
    piecewise_trajectory OMPL_plan(geometry_msgs::Point start, geometry_msgs::Point goal, octomap::OcTree * octree, int &status);


    // dummy publishers for debugging/microbenchmarking
    void publish_dummy_octomap(octomap::OcTree *m_octree);
    void publish_dummy_octomap_vis(octomap::OcTree *m_octree);
    bool goal_rcv_call_back(package_delivery::point::Request &req, package_delivery::point::Response &res);
    double planner_min_freq;
    ros::Time last_planning_time;
    bool failed_to_plan_last_time = false;
    bool planned_optimally = true; // if smoothener spits out suboptimal paths, use this to replan
    Drone * drone = nullptr;
    int capture_size = 600; //set this to 1 if you want to see every data captured separately
private:
    ros::NodeHandle nh;
    ros::CallbackQueue callback_queue;
    ros::Publisher smooth_traj_vis_pub, piecewise_traj_vis_pub, octomap_dummy_pub, m_markerPub;
    bool knob_performance_modeling = false;
    bool knob_performance_modeling_for_om_to_pl = false;
    ros::Subscriber future_col_sub, next_steps_sub, octomap_sub, octomap_communication_proxy_msg;
    ros::ServiceServer get_trajectory_srv_server, goal_rcv_service;
    ros::Publisher traj_pub;
    ros::Publisher timing_msg_from_mp_pub;
    octomap::OcTree * octree = nullptr; int future_col_seq_id = 0;
    ros::Publisher motion_plannineg_debug_pub;
    int trajectory_seq_id = 0;
    mavbench_msgs::multiDOFtrajectory g_next_steps_msg;
    bool first_time_planning = true;
    bool first_time_planning_succeeded = false;
    bool DEBUG_RQT;

    geometry_msgs::Point g_start_pos;
    geometry_msgs::Point g_goal_pos;
    bool goal_known = false;
    ros::Time g_start_time{0};
	enum planning_reason_enum {No_need_to_plan, Collision_detected, Last_plan_approximate, Failed_to_plan_last_time, Min_freq_passed, First_time_planning};
	enum planning_status {Short_time_failure, Initial_state_failure, Success};
	bool planned_approximately = false; //if true, we replan again


    // Parameters
    std::string motion_planning_core_str;
    bool DEBUG__global;
    double drone_height__global;
    double drone_radius__global;
    double rrt_step_size__global;
    int rrt_bias__global;
    double x__low_bound__global, x__high_bound__global;
    double y__low_bound__global, y__high_bound__global;
    double z__low_bound__global, z__high_bound__global;
    bool g_always_randomize_end_point; // for micro benchmarking
    int nodes_to_add_to_roadmap__global;
    double max_dist_to_connect_at__global;
    double sampling_interval__global;
    double v_max__global, a_max__global;
    bool notified_failure = false;
    int max_roadmap_size__global;
    std::function<piecewise_trajectory (geometry_msgs::Point, geometry_msgs::Point, int, int , int, octomap::OcTree *, int &status)> motion_planning_core;
    long long g_planning_without_OM_PULL_time_acc = 0;
    int g_number_of_planning = 0 ;
    float g_piecewise_planning_budget;
    float g_smoothening_budget;
    float g_out_of_bounds_allowance = 5;
    int replanning_reason;
    // The following block of variables only exist for debugging purposes
    visualization_msgs::MarkerArray smooth_traj_markers;
    visualization_msgs::MarkerArray piecewise_traj_markers;
};


// A class to validate proposed motions for OMPL
class OMPLMotionValidator : public ompl::base::MotionValidator
{
public:
    OMPLMotionValidator(MotionPlanner * mp_, const ompl::base::SpaceInformationPtr &si)
        : ompl::base::MotionValidator(si), mp(mp_)
    {
    }

    bool checkMotion(const ompl::base::State *s1,
            const ompl::base::State *s2) const override
    {
        namespace ob = ompl::base;

        const auto *pos1 = s1->as<ob::RealVectorStateSpace::StateType>();
        const auto *pos2 = s2->as<ob::RealVectorStateSpace::StateType>();

        double x1 = pos1->values[0], x2 = pos2->values[0];
        double y1 = pos1->values[1], y2 = pos2->values[1];
        double z1 = pos1->values[2], z2 = pos2->values[2];

        return !mp->collision(mp->octree, {x1,y1,z1}, {x2,y2,z2});
    }

    bool checkMotion(const ompl::base::State *s1,
            const ompl::base::State *s2,
            std::pair<ompl::base::State*, double>& lastValid) const override
    {
        namespace ob = ompl::base;

        const auto *pos1 = s1->as<ob::RealVectorStateSpace::StateType>();
        const auto *pos2 = s2->as<ob::RealVectorStateSpace::StateType>();

        double x1 = pos1->values[0], x2 = pos2->values[0];
        double y1 = pos1->values[1], y2 = pos2->values[1];
        double z1 = pos1->values[2], z2 = pos2->values[2];

        graph::node end;
        bool valid = !mp->collision(mp->octree, {x1,y1,z1}, {x2,y2,z2}, &end);

        if (!valid) {
            auto *end_pos = lastValid.first->as<ob::RealVectorStateSpace::StateType>();
            end_pos->values[0] = end.x;
            end_pos->values[1] = end.y;
            end_pos->values[2] = end.z;

            double dx = x2-x1, dy = y2-y1, dz = z2-z1;
            double end_dx = end.x-x1, end_dy = end.y-y1, end_dz = end.z-z1;

            if (dx != 0)
                lastValid.second = end_dx / dx;
            else if (dy != 0)
                lastValid.second = end_dy / dy;
            else if (dz != 0)
                lastValid.second = end_dz / dz;
            else
                lastValid.second = 0;
        }

        return valid;
    }

private:
    MotionPlanner * mp = nullptr;
};



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
    ob::PlannerStatus solved = ss.solve(g_piecewise_planning_budget);
    if (solved == ob::PlannerStatus::INVALID_START) {
    	status = 2;
    }else if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION){
    	status = 3;
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

