#include <ros/ros.h>

// Standard headers
#include <cstdlib>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include <iostream>
#include <thread>
#include <functional>
#include <limits>
#include <signal.h>

// My headers
#include "common.h"
#include "graph.h"
#include "global_planner.h"
#include "package_delivery/get_trajectory.h"
#include "timer.h"

// Misc messages
#include <geometry_msgs/Point.h>

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_world/octomap_world.h>

// TF specific headers
#include <tf/transform_datatypes.h>

// Pointcloud headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Trajectory smoothening headers
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

// OMPL specific headers

// Type-defs
using piecewise_trajectory = std::vector<graph::node>;
using smooth_trajectory = mav_trajectory_generation::Trajectory;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;


// Parameters
graph::node_id start_id,  goal_id;
std::string motion_planning_core_str;
bool DEBUG__global;
double drone_height__global;
double drone_radius__global;
double rrt_step_size__global;
int rrt_bias__global;
double x__low_bound__global, x__high_bound__global;
double y__low_bound__global, y__high_bound__global;
double z__low_bound__global, z__high_bound__global;
int nodes_to_add_to_roadmap__global;
double max_dist_to_connect_at__global;
double sampling_interval__global;
double v_max__global, a_max__global;
int max_roadmap_size__global;
std::function<piecewise_trajectory (geometry_msgs::Point, geometry_msgs::Point, int, int , int, octomap::OcTree *)> motion_planning_core;


//*** F:DN global variables
octomap::OcTree * octree = nullptr;
trajectory_msgs::MultiDOFJointTrajectory traj_topic;
ros::ServiceClient octo_client;

// The following block of global variables only exist for debugging purposes
visualization_msgs::MarkerArray smooth_traj_markers;
visualization_msgs::MarkerArray piecewise_traj_markers;
octomap_msgs::Octomap omp;
PointCloud::Ptr pcl_ptr{new pcl::PointCloud<pcl::PointXYZ>};
visualization_msgs::Marker graph_conn_list;
ros::Publisher graph_conn_pub;
// #define INFLATE

// *** F:DN calculating the distance between two nodes in the graph.
double dist(const graph::node& n1, const graph::node& n2);


// *** F:DN Checks whether a cell in the occupancy grid is occupied.
bool occupied(octomap::OcTree * octree, double x, double y, double z);


// *** F:DN Checks whether a cell in the occupancy grid is known.
bool known(octomap::OcTree * octree, double x, double y, double z);


// *** F:DN Checks whether there is a collision between two nodes in the occupancy grid.
bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr = nullptr);


// *** F:DN Checks whether a cell in the occupancy grid is occupied.
bool out_of_bounds(const graph::node& pos);


// *** F:DN find all neighbours within "max_dist" meters of node
std::vector<graph::node_id> nodes_in_radius(/*const*/ graph& g, graph::node_id n, double max_dist, octomap::OcTree * octree);


// *** F:DN Request an octomap from the octomap_server
void request_octomap();


// *** F:DN Clear area in bounding box
void clear_octomap_bbx(const graph::node& pos);


// *** F:DN Generate and inflate an octomap from a message
void generate_octomap(const octomap_msgs::Octomap& msg);


// ***F:DN FOR MICRO-BENCHMARK
piecewise_trajectory circle(geometry_msgs::Point start, geometry_msgs::Point goal);


// *** F:DN Optimize and smoothen a piecewise path without causing any new collisions.
smooth_trajectory smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree);


// ***F:DN Build the response to the service from the smooth_path
void create_response(package_delivery::get_trajectory::Response &res, smooth_trajectory& smooth_path);


// ***F:DN Temporary debugging function that publishes the roadmap (without the connections) for visualization.
void publish_graph(graph& g);


// ***F:DN Post-process piecewise path to optimize it.
void postprocess(piecewise_trajectory& path);


//*** F:DN getting the smoothened trajectory
bool get_trajectory_fun(package_delivery::get_trajectory::Request &req, package_delivery::get_trajectory::Response &res)
{
	//----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
	piecewise_trajectory piecewise_path;
	smooth_trajectory smooth_path;

    //----------------------------------------------------------------- 
    // *** F:DN Body 
    //----------------------------------------------------------------- 

    request_octomap();
    if (octree == nullptr) {
    	ROS_ERROR("Octomap is not available.");
    	return false;
    }
    clear_octomap_bbx({req.start.x, req.start.y, req.start.z});

    // octomap_msgs::binaryMapToMsg(*octree, omp);
    octree->writeBinary("/home/ubuntu/octomap.bt");

    // piecewise_path = motion_planning_core(req.start, req.goal, req.width, req.length ,req.n_pts_per_dir, octree);
    //piecewise_path = motion_planning_core(req.start, req.goal, octree);
    piecewise_path = circle(req.start, req.goal);

    if (piecewise_path.size() == 0) {
        ROS_ERROR("Empty path returned");
        return false;
    }

    ROS_INFO("Path size: %u. Now post-processing...", piecewise_path.size());

    if (motion_planning_core_str != "lawn_mower") {
        postprocess(piecewise_path);
    }

    ROS_INFO("Path size: %u. Now smoothening...", piecewise_path.size());

    // Smoothen the path and build the multiDOFtrajectory response
    smooth_path = smoothen_the_shortest_path(piecewise_path, octree);
	
    create_response(res, smooth_path);

    // Publish the trajectory (for debugging purposes)
    traj_topic = res.multiDOFtrajectory;

	return true;
}


// *** F:DN initializing all the global variables 
void motion_planning_initialize_params() {

    ros::param::get("motion_planner/max_roadmap_size", max_roadmap_size__global);
    ros::param::get("/motion_planner/sampling_interval", sampling_interval__global);
    ros::param::get("/motion_planner/rrt_step_size", rrt_step_size__global);
    ros::param::get("/motion_planner/rrt_bias", rrt_bias__global);
    ros::param::get("/motion_planner/x_dist_to_sample_from__low_bound", x__low_bound__global);
    ros::param::get("/motion_planner/x_dist_to_sample_from__high_bound", x__high_bound__global);

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
    ros::param::get("ros_DEBUG", DEBUG__global);
    //std::cout<<"max_dist_to_"<<max_dist_to_connect_at__global<<std::endl;
    
    ros::param::get("/motion_planner/motion_planning_core", motion_planning_core_str);
}



int main(int argc, char ** argv)
{
    //----------------------------------------------------------------- 
    // *** F:DN variables	
    //----------------------------------------------------------------- 
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    motion_planning_initialize_params();
    signal(SIGINT, sigIntHandler);

    // *** F:DN topics and services
    ros::ServiceServer service = nh.advertiseService("get_trajectory_srv", get_trajectory_fun);
    ros::Publisher smooth_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
    ros::Publisher piecewise_traj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("multidoftraj", 1);
    ros::Publisher octo_pub = nh.advertise<octomap_msgs::Octomap>("omap", 1);
    ros::Publisher pcl_pub = nh.advertise<PointCloud> ("graph", 1);
    graph_conn_pub = nh.advertise<visualization_msgs::Marker>("graph_conns", 100);
    // ros::Subscriber octomap_sub = nh.subscribe("octomap_full", 1, generate_octomap);
    octo_client = nh.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");
	
    pcl_ptr->header.frame_id = graph_conn_list.header.frame_id = "world";
    graph_conn_list.type = visualization_msgs::Marker::LINE_LIST;
    graph_conn_list.action = visualization_msgs::Marker::ADD;
    graph_conn_list.scale.x = 0.1;
    graph_conn_list.pose.orientation.w = 1;
    graph_conn_list.color.r = 1;
    graph_conn_list.color.a = 1;

    /* //TODO place a sanity check making sure that panic distance is smaller than halo
    float panic_distance = ros::param::get("/panic_pcl/safe_distance",panic_distance);
    float  
    */

    

    //----------------------------------------------------------------- 
    // *** F:DN BODY
    //----------------------------------------------------------------- 
	ros::Rate pub_rate(5);
	while (ros::ok())
	{
        if (DEBUG__global) { //if debug, publish markers to be seen by rviz
            smooth_traj_vis_pub.publish(smooth_traj_markers);
            piecewise_traj_vis_pub.publish(piecewise_traj_markers);
            graph_conn_pub.publish(graph_conn_list);
            octo_pub.publish(omp);
            pcl_pub.publish(pcl_ptr);
        }
        traj_pub.publish(traj_topic);
		ros::spinOnce();
		pub_rate.sleep();
	}

    return 0;
}


double dist(const graph::node& n1, const graph::node& n2)
{
	return std::sqrt((n1.x-n2.x)*(n1.x-n2.x) + (n1.y-n2.y)*(n1.y-n2.y) + (n1.z-n2.z)*(n1.z-n2.z));
}


bool occupied(octomap::OcTree * octree, double x, double y, double z)
{
	const double OCC_THRESH = 0.5;

	octomap::OcTreeNode * otn = octree->search(x, y, z);

	return otn != nullptr && otn->getOccupancy() >= OCC_THRESH;
}


bool known(octomap::OcTree * octree, double x, double y, double z)
{
	return octree->search(x, y, z) != nullptr;
}


bool out_of_bounds(const graph::node& pos) {
    return (pos.x < x__low_bound__global
            || pos.x > x__high_bound__global
            || pos.y < y__low_bound__global
            || pos.y > y__high_bound__global
            || pos.z < z__low_bound__global
            || pos.z > z__high_bound__global);
}


#ifdef INFLATE
bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
{
    RESET_TIMER();
    // First, check if anything goes underground
    if (n1.z <= 0 || n2.z <= 0)
        return true;
            
	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;

	double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    octomap::point3d start(n1.x, n1.y, n1.z);
	octomap::point3d direction(dx, dy, dz);
	octomap::point3d end;

    bool collided = octree->castRay(start, direction, end, true, distance);

    if (end_ptr != nullptr && collided) {
        end_ptr->x = end.x();
        end_ptr->y = end.y();
        end_ptr->z = end.z();
    }

	LOG_ELAPSED(motion_planner);
	return collided;
}
#else
bool collision(octomap::OcTree * octree, const graph::node& n1, const graph::node& n2, graph::node * end_ptr)
{
    RESET_TIMER();
    // First, check if anything goes underground
    if (n1.z <= 0 || n2.z <= 0)
        return true;
            
    const double pi = 3.14159265359;

	// The drone is modeled as a cylinder.
	// Angles are in radians and lengths are in meters.
    
    double height = drone_height__global; 
    double radius = drone_radius__global; 

	const double angle_step = pi/4;
	const double radius_step = radius/3;
	const double height_step = height/2;

	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;
	double dz = n2.z - n1.z;

	double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

	octomap::point3d direction(dx, dy, dz);
	octomap::point3d end;

	for (double h = -height/2; h <= height/2; h += height_step) {
		for (double r = 0; r <= radius; r += radius_step) {
			for (double a = 0; a <= pi*2; a += angle_step) {
				octomap::point3d start(n1.x + r*std::cos(a), n1.y + r*std::sin(a), n1.z + h);

				if (octree->castRay(start, direction, end, true, distance)) {

                    if (end_ptr != nullptr) {
                        end_ptr->x = end.x();
                        end_ptr->y = end.y();
                        end_ptr->z = end.z();
                    }

					LOG_ELAPSED(motion_planner);
					return true;
                }
			}
		}
	}

	LOG_ELAPSED(motion_planner);
	return false;
}
#endif

std::vector<graph::node_id> nodes_in_radius(/*const*/ graph& g, graph::node_id n, double max_dist, octomap::OcTree * octree)
{
	auto node_ids = g.node_ids();
	node_ids.erase(n);
	std::vector<graph::node_id> result;

	for (const auto& n2 : node_ids) {
		if (dist(g.get_node(n), g.get_node(n2)) <= max_dist && !collision(octree, g.get_node(n), g.get_node(n2))) {
			result.push_back(n2);
		}
	}

	return result;
}


void request_octomap()
{
    octomap_msgs::GetOctomap srv;
    
    if (octo_client.call(srv))
        generate_octomap(srv.response.map);
    else
        ROS_ERROR("Octomap service request failed");
}


void clear_octomap_bbx(const graph::node& pos)
{
    if (occupied(octree, pos.x, pos.y, pos.z)) {
        ROS_WARN("Start is already occupied!");
    }

    // Free space around the start since is assumed to be open
    const double epsilon = 0.1;
    double r = drone_radius__global + epsilon;
    double h = drone_height__global + epsilon;

    octomap::point3d min(pos.x-r, pos.y-r, pos.z-h/2);
    octomap::point3d max(pos.x+r, pos.y+r, pos.z+h/2);

    double threshMin = octree->getClampingThresMin();
    for (auto it = octree->begin_leafs_bbx(min, max),
            end = octree->end_leafs_bbx(); it != end; ++it) {
        it->setLogOdds(octomap::logodds(threshMin));
    }
    octree->updateInnerOccupancy();
}


void generate_octomap(const octomap_msgs::Octomap& msg)
{
    RESET_TIMER();
    if (octree != nullptr) {
        delete octree;
    }

    ROS_INFO("Requesting octomap...");

#ifdef INFLATE
    // Inflate Octomap
    ROS_INFO("Inflating..");
    volumetric_mapping::OctomapWorld ocworld;
    ocworld.setOctomapFromMsg(msg);
    Eigen::Vector3d safety_radius(drone_radius__global, drone_radius__global,
            drone_radius__global);
    ocworld.inflateOccupied(safety_radius);

    // Convert inflated OctomapWorld to Octree
    octomap_msgs::Octomap inflated_msg;
    ocworld.getOctomapBinaryMsg(&inflated_msg);
	octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(inflated_msg);
#else
    octomap::AbstractOcTree * tree = octomap_msgs::msgToMap(msg);
#endif
	octree = dynamic_cast<octomap::OcTree*> (tree);

    if (octree == nullptr) {
        ROS_ERROR("Octree could not be pulled.");
    }

    LOG_ELAPSED(motion_planner_pull);
}



void create_response(package_delivery::get_trajectory::Response &res, smooth_trajectory& smooth_path)
{
    const double safe_radius = 1.0;

	// Sample trajectory
	mav_msgs::EigenTrajectoryPoint::Vector states;
	//double sampling_interval__global;
	//ros::param::get("/motion_planner/sampling_interval__global", sampling_interval__global);
	mav_trajectory_generation::sampleWholeTrajectory(smooth_path, sampling_interval__global, &states);

    // Get starting position
    graph::node start = {states[0].position_W.x(), states[0].position_W.y(), states[0].position_W.z()};

	// Convert sampled trajectory points to MultiDOFJointTrajectory response
	res.multiDOFtrajectory.joint_names.push_back("base");
    res.unknown = -1;

    int state_index = 0;
	for (const auto& s : states) {
		trajectory_msgs::MultiDOFJointTrajectoryPoint point;

		geometry_msgs::Transform pos;
        graph::node current;
		pos.translation.x = current.x = s.position_W.x();
		pos.translation.y = current.y = s.position_W.y();
		pos.translation.z = current.z = s.position_W.z();

		geometry_msgs::Twist vel;
		vel.linear.x = s.velocity_W.x();
		vel.linear.y = s.velocity_W.y();
		vel.linear.z = s.velocity_W.z();

		geometry_msgs::Twist acc;
		acc.linear.x = s.acceleration_W.x();
		acc.linear.y = s.acceleration_W.y();
		acc.linear.z = s.acceleration_W.z();

		ros::Duration dur(float(s.time_from_start_ns) / 1e9);

		point.transforms.push_back(pos);
		point.velocities.push_back(vel);
		point.accelerations.push_back(acc);
		point.time_from_start = dur;

        if (res.unknown != -1 &&
                !known(octree, current.x, current.y, current.z)
                && dist(start, current) > safe_radius) {
            ROS_WARN("Trajectory enters unknown space.");
            res.unknown = state_index;
        }

		res.multiDOFtrajectory.points.push_back(point);

        state_index++;
	}
}


smooth_trajectory smoothen_the_shortest_path(piecewise_trajectory& piecewise_path, octomap::OcTree* octree)
{
    // Variables for visualization for debugging purposes
	double distance = 0.5; 
	std::string frame_id = "world";

	// Setup optimizer
	mav_trajectory_generation::Vertex::Vector vertices;
	const int dimension = 3;
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
	
	// Convert roadmap path to optimizer's path format
	mav_trajectory_generation::Vertex start_v(dimension), end_v(dimension);
	start_v.makeStartOrEnd(Eigen::Vector3d(piecewise_path.front().x, piecewise_path.front().y, piecewise_path.front().z), derivative_to_optimize);
	end_v.makeStartOrEnd(Eigen::Vector3d(piecewise_path.back().x, piecewise_path.back().y, piecewise_path.back().z), derivative_to_optimize);

	vertices.push_back(start_v);
	for (auto it = piecewise_path.begin()+1; it+1 != piecewise_path.end(); ++it) {
		mav_trajectory_generation::Vertex v(dimension);
		v.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(it->x, it->y, it->z));
		vertices.push_back(v);
	}
	vertices.push_back(end_v);

	// Parameters used to calculate how quickly the drone can move between vertices
	const double magic_fabian_constant = 6.5; // A tuning parameter.

	//double v_max__global, a_max__global;
	//ros::param::get("/motion_planner/v_max__global", v_max__global);
	//ros::param::get("/motion_planner/a_max__global", a_max__global);

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);

	// Optimize until no collisions are present
	bool col;
	do {
		ROS_INFO("Checking for collisions...");
		col = false;

		// Estimate the time the drone should take flying between each node
		auto segment_times = estimateSegmentTimes(vertices, v_max__global, a_max__global, magic_fabian_constant);
	
		// Optimize and create a smooth path from the vertices
		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();

		// Return all the smooth segments in the path
		// (Each segment goes from one of the original nodes to the next one in the path)
		mav_trajectory_generation::Segment::Vector segments;
		opt.getSegments(&segments);
	} while (col);

	// Return the collision-free smooth trajectory
	mav_trajectory_generation::Trajectory traj;
	opt.getTrajectory(&traj);

	ROS_INFO("Smoothened path!");

	// Visualize path for debugging purposes
	mav_trajectory_generation::drawMavTrajectory(traj, distance, frame_id, &smooth_traj_markers);
	mav_trajectory_generation::drawVertices(vertices, frame_id, &piecewise_traj_markers);

	return traj;
}


void publish_graph(graph& g)
{
	pcl_ptr->clear();
    graph_conn_list.points.clear();

	for (auto n_id : g.node_ids()) {
		graph::node n = g.get_node(n_id);
		pcl_ptr->points.push_back (pcl::PointXYZ(n.x, n.y, n.z));

        // Publish edges between nodes
        for (const auto& e : g.adjacent_edges(n_id)) {
            geometry_msgs::Point p1, p2;

            p1.x = g.get_node(e.n1).x;
            p1.y = g.get_node(e.n1).y;
            p1.z = g.get_node(e.n1).z;

            p2.x = g.get_node(e.n2).x;
            p2.y = g.get_node(e.n2).y;
            p2.z = g.get_node(e.n2).z;

            graph_conn_list.points.push_back(p1);
            graph_conn_list.points.push_back(p2);
        }
	}

    if (DEBUG__global) {
        // Visualize graph immediately (a temporary debugging technique)
        graph_conn_pub.publish(graph_conn_list);
        ros::spinOnce();
    }
}


void postprocess(piecewise_trajectory& path)
{
    return;
    // We use a greedy approach to shorten the path here.
    // We connect non-adjacent nodes in the path that do not have collisions.
    
    for (auto it = path.begin(); it != path.end()-1; ) {
        bool shortened = false;
        for (auto it2 = path.end()-1; it2 != it+1 && !shortened; --it2) {
            if (!collision(octree, *it, *it2)) {
                it = path.erase(it+1, it2);
                shortened = true;
            }
        }

        if (!shortened)
            ++it;
    }
}


piecewise_trajectory circle(geometry_msgs::Point start, geometry_msgs::Point goal)
{
    piecewise_trajectory result;

    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double dz = goal.z - start.z;
    double d = std::sqrt(dx*dx + dy*dy + dz*dz);
    double r = d/2;
    double yaw = std::atan2(dx, dy);
    const double pi = std::atan(1)*4;

    graph::node start_node{start.x, start.y, start.z};
    graph::node goal_node{goal.x, goal.y, goal.z};
    graph::node mid{(start.x+goal.x)/2.0, (start.y+goal.y)/2.0, (start.z+goal.z)/2.0};

    std::cout << mid.x << ", " << mid.y << ", " << mid.z << std::endl;

    result.push_back(start_node);
    for (double a = 0; a <= 180;) {
        graph::node n;
        n.x = mid.x - r*std::sin(a/180 * pi);
        n.y = mid.y - r*std::cos(a/180 * pi);
        n.z = start.z + a/180 * (goal.z-start.z);
        result.push_back(n);

        if (a < 5 || a > 175)
            a += 0.05;
        else
            a += 5;
    }
    result.push_back(goal_node);

    return result;
}

