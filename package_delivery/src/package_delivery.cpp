#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <chrono>
#include <thread>
#include <deque>
#include <limits>
#include <signal.h>

#include "control_drone.h"
#include "common/Common.hpp"
#include "Drone.h"
#include "package_delivery/get_trajectory.h"
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include "common.h"
#include "timer.h"

using namespace std;
bool should_panic = false;
bool col_imminent = false;
bool col_coming = false;
bool slam_lost = false;
geometry_msgs::Vector3 panic_direction;
string ip_addr__global;
string localization_method;
string stats_file_addr;
string ns;
double v_max;
double angular_vel;
	
enum State { setup, waiting, flying, completed, failed, invalid };

double dist(coord t, geometry_msgs::Point m)
{
    // We must convert between the two coordinate systems
    return std::sqrt((t.x-m.x)*(t.x-m.x) + (t.y-m.y)*(t.y-m.y) + (t.z-m.z)*(t.z-m.z));
}

// *** F:DN call back function for the panic_topic subscriber
void panic_callback(const std_msgs::Bool::ConstPtr& msg) {
    should_panic = msg->data;
}

void panic_dir_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    panic_direction = *msg;
}

void col_imminent_callback(const std_msgs::Bool::ConstPtr& msg) {
    col_imminent = msg->data;
}

void col_coming_callback(const std_msgs::Bool::ConstPtr& msg) {
    col_coming = msg->data;
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}

void package_delivery_initialize_params() {
    if(!ros::param::get("/package_delivery/ip_addr",ip_addr__global)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/ip_addr").c_str());
      return; 
    }
    if(!ros::param::get("/package_delivery/localization_method",localization_method)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/localization_method").c_str());
       return; 
    }
    if(!ros::param::get("/stats_file_addr",stats_file_addr)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/stats_file_addr").c_str());
     return; 
    }
    if(!ros::param::get("/motion_planner/v_max", v_max)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/v_max").c_str());
     return; 
    }
    if(!ros::param::get("/angular_vel", angular_vel)){
        ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s", 
                (ns + "/angular_vel").c_str());
     return;
    }
}

geometry_msgs::Point get_start(Drone& drone) {
    geometry_msgs::Point start;

    // Get current position from drone
    auto drone_pos = drone.position();
    start.x = drone_pos.x; start.y = drone_pos.y; start.z = drone_pos.z;
    std::cout << "Current position is " << drone_pos.x << " " << drone_pos.y << " " << drone_pos.z << std::endl;

    return start;
}

geometry_msgs::Point get_goal() {
    geometry_msgs::Point goal;

    // Get intended destination from user
    std::cout << "Please input your destination in x,y,z format." << std::endl;
    double input_x, input_y, input_z;
    std::cin >> input_x >> input_y >> input_z;
    goal.x = input_x; goal.y = input_y; goal.z = input_z;

    return goal;
}

trajectory_t request_trajectory(ros::ServiceClient& client, geometry_msgs::Point start, geometry_msgs::Point goal) {
    // Request the actual trajectory from the motion_planner node
    package_delivery::get_trajectory srv;
    srv.request.start = start;
    srv.request.goal = goal;

    if (client.call(srv)) {
        ROS_INFO("Received trajectory.");
    } else {
        ROS_ERROR("Failed to call service.");
        return trajectory_t();
    }

    return create_trajectory(srv.response.multiDOFtrajectory);
}

trajectory_t mod_trajectory(const trajectory_t& trajectory) {
    ROS_ERROR("v_max is %f", v_max);

    trajectory_t result;

    float yaw = -90;

    for (multiDOFpoint p : trajectory) {
        double v = std::sqrt(p.vx*p.vx + p.vy*p.vy + p.vz*p.vz);
        double scale = v_max / v;

        p.vx *= scale;
        p.vy *= scale;
        p.vz *= scale;
        p.duration /= scale;
        p.yaw = yaw;

        if (p.vx != p.vx || p.vy != p.vy || p.vz != p.vz || p.duration == 0)
            continue;
        yaw += p.duration * angular_vel;
        yaw = yaw <= 180 ? yaw : yaw-360;
        result.push_back(p);
    }

    // Reverse path to build full circle
    trajectory_t back_path;
    for (multiDOFpoint p : result) {
        p.vx = -p.vx;
        p.vy = -p.vy;
        p.vz = -p.vz;
        p.yaw = yaw;

        yaw += p.duration * angular_vel;
        yaw = yaw <= 180 ? yaw : yaw-360;

        back_path.push_back(p);
    }
    result.insert(result.end(), back_path.begin(), back_path.end());

    return result;
}

bool trajectory_done(const trajectory_t& trajectory) {
    return trajectory.size() == 0;
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "package_delivery", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandler);
    ns = ros::this_node::getName();
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    package_delivery_initialize_params();
    geometry_msgs::Point start, goal;

    // Flight queues
    trajectory_t normal_traj, rev_normal_traj;
    trajectory_t panic_traj;
    trajectory_t slam_loss_traj;
    trajectory_t future_col_traj;

    bool created_future_col_traj = false;
    bool created_slam_loss_traj = false;

    uint16_t port = 41451;
    // Drone drone(ip_addr__global.c_str(), port, localization_method);
    Drone drone(ip_addr__global.c_str(), port, "ground_truth");
    bool delivering_mission_complete = false; //if true, we have delivered the 
                                              //pkg and successfully returned to origin
    // *** F:DN subscribers,publishers,servers,clients
	ros::ServiceClient get_trajectory_client = 
        nh.serviceClient<package_delivery::get_trajectory>("get_trajectory_srv");
	ros::ServiceClient stats_output_client = 
        nh.serviceClient<std_srvs::Trigger>("output_stats");

    ros::Subscriber panic_sub = 
		nh.subscribe<std_msgs::Bool>("panic_topic", 1, panic_callback);
    ros::Subscriber panic_dir_sub = 
		nh.subscribe<geometry_msgs::Vector3>("panic_direction", 1, panic_dir_callback);
    ros::Subscriber col_imminent_sub = 
		nh.subscribe<std_msgs::Bool>("col_imminent", 1, col_imminent_callback);
    ros::Subscriber col_coming_sub = 
		nh.subscribe<std_msgs::Bool>("col_coming", 1, col_coming_callback);
	ros::Subscriber slam_lost_sub = 
		nh.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);

    //----------------------------------------------------------------- 
	// *** F:DN knobs(params)
	//----------------------------------------------------------------- 
    const float goal_s_error_margin = 1e9; //ok distance to be away from the goal.
                                           //this is b/c it's very hard 
                                           //given the issues associated with
                                           //flight controler to land exactly
                                           //on the goal

    msr::airlib::FlightStats init_stats, end_stats;
    std::string mission_status = "timeout";
    //----------------------------------------------------------------- 
	// *** F:DN Body
	//----------------------------------------------------------------- 
    
    // Wait for SLAM to come online
    tf::TransformListener tfListen;
    while(1) {
        try {
            tf::StampedTransform transform;
            tfListen.lookupTransform("/world", "/"+localization_method, ros::Time(0), transform);
            break;
        } catch(tf::TransformException& ex) {
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    //update_stats_file(stats_file_addr,"\n\n# NEW\n# Package delivery\n###\nTime: ");
    //log_time(stats_file_addr);
    //update_stats_file(stats_file_addr,"###\n");

    for (State state = setup; ros::ok(); ) {
        ros::spinOnce();
        State next_state = invalid;

        if (state == setup)
        {
            control_drone(drone);

            goal = get_goal();
            start = get_start(drone);

            init_stats = drone.getFlightStats();
            // spin_around(drone);
            drone.set_yaw(-90);
            
            next_state = waiting;
        }
        else if (state == waiting)
        {
            ROS_INFO("Waiting to receive trajectory...");

            start = get_start(drone);
            normal_traj = request_trajectory(get_trajectory_client, start, goal);
            normal_traj = mod_trajectory(normal_traj);

            // Pause a little bit so that future_col can be updated
            col_coming = col_imminent = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(150));

            if (!normal_traj.empty())
                next_state = flying;
            else
                next_state = failed;
        }
        else if (state == flying)
        {
            trajectory_t * forward_traj = nullptr;
            trajectory_t * rev_traj = nullptr;
            bool check_position = false;
            yaw_strategy_t yaw_strategy = follow_yaw; // face_forward; // ignore_yaw;

            // Handle panic queue
            if (should_panic) {
                ROS_ERROR("Panicking!");
                panic_traj = create_panic_trajectory(drone, panic_direction);
                check_position = false;
                
                normal_traj.clear(); // Replan a path once we're done
            } else {
                panic_traj.clear();
            }

            // Handle SLAM loss queue
            if (slam_lost) {
                ROS_WARN("SLAM lost!");
                if (!created_slam_loss_traj)
                    slam_loss_traj = create_slam_loss_trajectory(drone, normal_traj, rev_normal_traj);

                future_col_traj.clear(); // No need to keep this if we're planning to replan anyway. Keeping it could cause collisions in some cases

                created_slam_loss_traj = true;
            } else {
                slam_loss_traj.clear();
                created_slam_loss_traj = false;
            }

            // Handle future_collision queue
            if (col_coming) {
                ROS_WARN("Future collision appeared on trajectory!");

                if (!created_future_col_traj)
                    future_col_traj = create_future_col_trajectory(normal_traj, 3);

                created_future_col_traj = true;

                check_position = false;

                ROS_WARN_STREAM("Future col length " << future_col_traj.size());

                normal_traj.clear(); // Replan the normal path once we're done
            } else {
                future_col_traj.clear();
                created_future_col_traj = false;
            }

            // Choose correct queue to use
            if (!panic_traj.empty()) {
                // ROS_ERROR("Chose panic trajectory");
                forward_traj = &panic_traj;
                rev_traj = nullptr;
            } else if (!slam_loss_traj.empty()) {
                // ROS_WARN("Chose SLAM loss trajectory");
                forward_traj = &slam_loss_traj;
                rev_traj = &normal_traj;
                yaw_strategy = ignore_yaw;
            } else if (!future_col_traj.empty()) {
                // ROS_WARN("Chose future collision trajectory");
                forward_traj = &future_col_traj;
                rev_traj = &rev_normal_traj;
            } else {
                // ROS_INFO("Chose normal path");
                forward_traj = &normal_traj;
                rev_traj = &rev_normal_traj;
            }

            // multiDOFpoint p = forward_traj->front();
            // std::cout << p.vx << " " << p.vy << " " << p.vz << " " << p.duration << " " << std::sqrt(p.vx*p.vx+p.vy*p.vy+p.vz*p.vz) << " " <<  p.yaw << std::endl;

            follow_trajectory(drone, forward_traj, rev_traj, yaw_strategy, check_position);

            // Choose next state (failure, completion, or more flying)
            if (slam_lost)
                next_state = failed;
            else if (slam_lost && created_slam_loss_traj && trajectory_done(slam_loss_traj))
                next_state = failed;
            else if (trajectory_done(*forward_traj))
                next_state = completed;
            else
                next_state = flying;
        }
        else if (state == completed)
        {
            drone.fly_velocity(0, 0, 0);

            if (dist(drone.position(), goal) < goal_s_error_margin) {
                ROS_INFO("Delivered the package and returned!");
                mission_status = "completed"; 

                end_stats = drone.getFlightStats();
                std_srvs::Trigger srv;
                if (!stats_output_client.call(srv)) {
                    ROS_ERROR("Failed to output some info.");
                }
                output_flight_summary(init_stats, end_stats, mission_status, stats_file_addr);


                break;
                //update_stats_file(stats_file_addr,"mission_status completed");

                next_state = setup;
            } else { //If we've drifted too far off from the destination
                ROS_WARN("We're a little off...");

                auto pos = drone.position();
                std::cout << "Pos: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
                std::cout << "Goal: " << goal.x << " " << goal.y << " " << goal.z << std::endl;
                std::cout << "Dist: " << dist(pos, goal) << std::endl;

                start = get_start(drone);
                next_state = waiting;
            }
        }
        else if (state == failed) {
            ROS_ERROR("Failed to reach destination");
            mission_status = "failed"; 

            end_stats = drone.getFlightStats();
            std_srvs::Trigger srv;
            if (!stats_output_client.call(srv)) {
                ROS_ERROR("Failed to output some info.");
            }
            output_flight_summary(init_stats, end_stats, mission_status, stats_file_addr);

            break;

            //update_stats_file(stats_file_addr,"mission_status failed");
            next_state = setup;
        }
        else
        {
            ROS_ERROR("Invalid FSM state!");
            break;
        }

        state = next_state;
    }

    return 0;
}

