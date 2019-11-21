#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <limits>
#include <geometry_msgs/Vector3.h>
#include <mavbench_msgs/multiDOFpoint.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include "Profiling.h"
#include <mavbench_msgs/follow_traj_debug.h>
#include "Drone.h"

void sigIntHandler(int sig);


// Functions and classes to manipulate and follow trajectories
struct multiDOFpoint {
    double x, y, z;
    double vx, vy, vz;
    double ax, ay, az; // Currently, the acceleration values are ignored
    double yaw;
    bool blocking_yaw;
    double duration;
};


struct debug_follow_trajectory_data {
	double vx, vy, vz;
	double vx_error, vy_error, vz_error;
	double x_error, y_error, z_error;
};


typedef std::deque<multiDOFpoint> trajectory_t;
enum yaw_strategy_t { ignore_yaw, face_forward, face_backward, follow_yaw };

trajectory_t create_trajectory_from_msg(const mavbench_msgs::multiDOFtrajectory&);
mavbench_msgs::multiDOFtrajectory create_trajectory_msg(const trajectory_t&, Drone *drone);
multiDOFpoint trajectory_at_time(const trajectory_t& traj, double t);
multiDOFpoint trajectory_at_time(const mavbench_msgs::multiDOFtrajectory& traj, double t);
trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second);

double follow_trajectory(Drone& drone, trajectory_t * traj,
        trajectory_t * reverse_traj,
		mavbench_msgs::follow_traj_debug &debug_data,
		yaw_strategy_t yaw_strategy = ignore_yaw,
        bool check_position = true,
        float max_speed = std::numeric_limits<double>::infinity(),
        //float max_speed = 3,
        float time = 2, float p_vx = .5, float p_vy = .5, float p_vz = .5,
		float I_px = .5, float I_py = .5, float I_pz=.5,
		float d_px=.4, float d_py=.5, float d_pz=.5, bool reset_PID= false);
//		,debug_follow_trajectory_data debug_data = {});


// Recovery methods
trajectory_t create_slam_loss_trajectory(Drone& drone, trajectory_t& normal_traj, const trajectory_t& rev_normal_traj);
bool reset_slam(Drone& drone, const std::string& topic);


void zero_trajectory_msg(mavbench_msgs::multiDOFtrajectory &traj, coord current_pos);
// Spinning commands
void spin_around(Drone &drone);
void scan_around(Drone &drone, int angle);


// Utility functions
void modify_backward_traj(trajectory_t *traj, float backup_duration, float stay_in_place_duration_for_stop , float stay_in_place_duration_for_reverse, bool stop);

trajectory_t shift_trajectory(const trajectory_t &traj, Drone *drone);
float distance(float x, float y, float z);
double calc_vec_magnitude(double x, double y, double z);
float yawFromQuat(geometry_msgs::Quaternion q);
float yawFromVelocity(float vx, float vy);
void waitForLocalization(std::string method);

#endif

