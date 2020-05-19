#include <ros/ros.h>
#include <signal.h>

#include <iostream>
#include <math.h>
#include <stdio.h>
#include "common.h"
#include "Drone.h"
#include <std_msgs/Bool.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>
#include <mavbench_msgs/follow_traj_debug.h>
#include "profile_manager/profiling_data_srv.h"
#include <profile_manager/profiling_data_verbose_srv.h>
#include <profile_manager.h>
#include <datacontainer.h>
#include <cmath>
#include <mavbench_msgs/response_time_capture.h>
using namespace std;

// Trajectories
trajectory_t trajectory;
trajectory_t reverse_trajectory;
bool fly_backward = false;
bool stop = false;
// Messages from other nodes
bool slam_lost = false;
ros::Time future_collision_time{0}; // The moment in time when the drone should stop because of an upcoming collision
int future_collision_seq = 0;
int trajectory_seq = 0;
geometry_msgs::Point closest_unknown_point;
// Parameters
float g_v_max;
double p_vx_original, p_vy_original, p_vz_original, I_px, I_py, I_pz, d_px, d_py, d_pz; // I and P factor for the PID controller in follow_trajectory function
double g_grace_period = 0; // How much time the drone will wait for a new path to be calculated when a collision is near, before pumping the breaks
double g_time_to_come_to_full_stop = 0;
double g_fly_trajectory_time_out;
long long g_planning_time_including_ros_overhead_acc  = 0;
float g_max_yaw_rate;
float g_max_yaw_rate_during_flight;
bool g_trajectory_done = false;
bool g_got_new_trajectory = false;
double g_sampling_time_interval;
ros::Time last_new_trajectory_time;
float PID_correction_time; //the time within which we enforce PID (since the last  new trajectory received)
float g_follow_trajectory_loop_rate; //the time within which we enforce PID (since the last  new trajectory received)
float g_backup_duration, g_stay_in_place_duration_for_stop, g_stay_in_place_duration_for_reverse;
int g_capture_size = 600; //set this to 1, if you want to see every data collected separately
bool DEBUG_RQT = false; //to publish for rqt_plotter

// Profiling
std::string g_supervisor_mailbox; //file to write to when completed
float g_localization_status = 1.0;
long long g_rcv_traj_to_follow_traj_acc_t = 0;
bool CLCT_DATA, DEBUG;
int g_follow_ctr = 0;
long long g_img_to_follow_acc = 0;
ros::Time g_msg_time_stamp;
long long g_pt_cld_to_futurCol_commun_acc = 0;
int g_traj_ctr = 0; 
ros::Time g_recieved_traj_t;
double g_max_velocity_reached = 0;
bool new_trajectory_since_backed_up = false;
bool backed_up = false;
ros::Time  new_traj_msg_time_stamp;
ProfileManager *profile_manager_client;
DataContainer *profiling_container;
bool measure_time_end_to_end;
std::vector<ros::Time> timing_msgs_vec; //to collect the timing msgs that are send from imgPublisher for profiling SA latency and throughput profiling
int timing_msgs_channel_size = -1; //used to ensure we haven't dropped any timing_msgs (this is b/c if we drop msgs, then there is a chance, we can't calculate the
				  // the SA end to end accurately.
int timing_msgs_cntr = 0; //counting the number of msgs in the timing_msg topic before it's consumed
ros::Time timing_msgs_begin_el_time;
int replanning_reason;
string planning_status;
int SA_response_time_capture_ctr = 0; //counting the number of response_time captures. This is used to filter out the first
    									  // planning response time, since there is a big lag from the beginning of the
    									  // game execution and first planning which will skew the results


string this_response_time_collected_type;

bool micro_benchmark_signaled_supervisor = false; //if signaled, then don't capture the velocity anymore
//micro benchmarking
bool micro_benchmark;
int micro_benchmark_number;

mavbench_msgs::follow_traj_debug debug_data = {};

template <class P1, class P2>
double distance(const P1& p1, const P2& p2) {
    double x_diff = p2.x - p1.x;
    double y_diff = p2.y - p1.y;
    double z_diff = p2.z - p1.z;

    return distance(x_diff, y_diff, z_diff);
}


void log_data_before_shutting_down()
{
    std::cout << "\n\nMax velocity reached by drone: " << g_max_velocity_reached << "\n" << std::endl;
    profiling_container->setStatsAndClear();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profile_manager::profiling_data_verbose_srv profiling_data_verbose_srv_inst;

    profiling_data_verbose_srv_inst.request.key = ros::this_node::getName()+"_verbose_data";
    profiling_data_verbose_srv_inst.request.value = "\n" + profiling_container->getStatsInString();
    profile_manager_client->verboseClientCall(profiling_data_verbose_srv_inst);


    /*
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profiling_data_srv_inst.request.key = "localization_status";
    profiling_data_srv_inst.request.value = g_localization_status;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "img_to_follow_traj_commun_t";
    profiling_data_srv_inst.request.value = (((double)g_pt_cld_to_futurCol_commun_acc)/1e9)/g_traj_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "traj_rcv_to_follow";
    profiling_data_srv_inst.request.value = (((double)g_rcv_traj_to_follow_traj_acc_t)/1e9)/g_follow_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "image_to_follow_time";
    profiling_data_srv_inst.request.value = (((double)g_img_to_follow_acc)/1e9)/g_follow_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "max_velocity_reached";
    profiling_data_srv_inst.request.value = g_max_velocity_reached;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    */


}

//erase the vector up to the msg with certain header
// TODO: this won't work if we have multiple motion planner workers. I believe
//we'll need a separate channel/timing_msgs_vec for each one of them; THINK MORE
void erase_up_to_msg(const std_msgs::Header &msg_s_header, string caller){
	if (!measure_time_end_to_end || timing_msgs_vec.size() == 0){
		return;
	}

	// search throug hthe timeing_msgs to find the msg of interest (with the same header)
	vector<ros::Time>::iterator it ;
	for (it = timing_msgs_vec.begin(); it != timing_msgs_vec.end(); it++){
		if (*it == msg_s_header.stamp) {
			break;
		}
	}

	//uppon finding, log it, and erase everything before it (to improve performance and space)
	// if call back is motion planner, then erase up to msg (
	if (it == timing_msgs_vec.end()){
		ROS_ERROR_STREAM("couldn't find a header with the same time stamp to erase the elements before of");
		//exit(0);
	}else{
	    timing_msgs_begin_el_time = *(timing_msgs_vec.begin());
		if (caller == "callback_trajectory"){ // only make sense to calculate SA metrics when there is a trajectory
	    	if (SA_response_time_capture_ctr >=1 && !micro_benchmark_signaled_supervisor) profiling_container->capture("S_A_waiting_time", "single",
	    			(*it - *timing_msgs_vec.begin()).toSec(), g_capture_size);
	    ;
	    	//ROS_INFO_STREAM("========== S_A response time"<<(ros::Time::now() - *timing_msgs_vec.begin()).toSec());
	    }
		timing_msgs_vec.erase(timing_msgs_vec.begin(), it);
	}
}

//call back function  to receive timing msgs. These msgs are collectd for profiling the SA latnecy and throughput purposes
void timing_msgs_callback(const std_msgs::Header::ConstPtr& msg) {
	if (!measure_time_end_to_end){
		return;
	}
	timing_msgs_cntr += 1;
	//sanity check ensuring no msg is dropped from the queue
	if (timing_msgs_cntr == timing_msgs_channel_size){
		ROS_ERROR_STREAM("timing_msgs channel is full; either increase the follow trajectory rate, or decrease the imgPublisher rate");
	}
	timing_msgs_vec.push_back(msg->stamp);
}

//call back function to receive timing msgs_from_mp. Using this, we reset the vector because this means
//that we have already made a decision to not make a traj for those imgs
void timing_msgs_from_mp_callback(const mavbench_msgs::response_time_capture::ConstPtr& msg) {
	closest_unknown_point = msg->closest_unknown_point;
	erase_up_to_msg(msg->header, "timing_msgs_from_mp_callback");
    if (msg->planning_status == "first_time_planning") {
    	return;
    }else if (SA_response_time_capture_ctr >=1 and !micro_benchmark_signaled_supervisor) { //>=1 cause the first one is really big due to pre_mission steps
    	profiling_container->capture("S_A_response_time", "single",
    			(ros::Time::now() - timing_msgs_begin_el_time).toSec(), g_capture_size); //ignoring the first planning since it takse forever
    	profiling_container->capture("S_A_response_time_" + msg->planning_status, "single",
    			(ros::Time::now() - timing_msgs_begin_el_time).toSec(), g_capture_size); //ignoring the first planning since it takse forever
    }

    this_response_time_collected_type = "S_A_response_time_" + msg->planning_status;
    if(measure_time_end_to_end && !micro_benchmark_signaled_supervisor) profiling_container->capture("S_A_latency", "single", (ros::Time::now() - msg->header.stamp).toSec(), g_capture_size);


    SA_response_time_capture_ctr++;

	debug_data.header.stamp = ros::Time::now();
    debug_data.controls = msg->controls;
    debug_data.ee_profiles = msg->ee_profiles;
	debug_data.ee_profiles.actual_time.pl_to_ft_ros_oh = (ros::Time::now() - msg->ee_profiles.actual_time.pl_pre_pub_time_stamp).toSec();
	debug_data.ee_profiles.actual_time.ee_latency = (ros::Time::now() - msg->ee_profiles.actual_time.img_capture_time_stamp).toSec();
	debug_data.ee_profiles.control_flow_path =  msg->ee_profiles.control_flow_path;

	debug_data.error.time.ppl_latency = -1;
	debug_data.error.time.smoothening_latency = -1;
	debug_data.error.time.ee_latency = -1;
	debug_data.error.space.ppl_vol = -1;
	debug_data.slack.expected.failure = -1;
	debug_data.slack.expected.data_flow =  -1;
	debug_data.slack.expected.control_flow =  -1;
	debug_data.slack.actual.failure = -1;
	debug_data.slack.expected.total = -1;
	debug_data.slack.actual.data_flow =  -1;
	debug_data.slack.actual.control_flow =  -1;
	debug_data.slack.actual.total = -1;

	if (!debug_data.controls.cmds.log_control_data){
		return;
	}


	// calculate the error
	// time error is caused by a combination of models(governer) and enforcement(operators)
	if (msg->ee_profiles.control_flow_path >= 1) { // made it up to end of planning
		debug_data.error.time.ppl_latency = fabs(msg->ee_profiles.actual_time.ppl_latency -  msg->ee_profiles.expected_time.ppl_latency);
	}
	if (msg->ee_profiles.control_flow_path >= 1.5) { // made it up to end of  smootheing
		debug_data.error.time.smoothening_latency = fabs(msg->ee_profiles.actual_time.smoothening_latency -  msg->ee_profiles.expected_time.smoothening_latency);
	}
	if (msg->ee_profiles.control_flow_path == 2) { // if sucess all the way
		debug_data.error.time.ee_latency = fabs(msg->ee_profiles.actual_time.ee_latency -  msg->ee_profiles.expected_time.ee_latency);
	}
	// the next two always run
	debug_data.error.time.om_to_pl_latency = fabs(msg->ee_profiles.actual_time.om_to_pl_latency-  msg->ee_profiles.expected_time.om_to_pl_latency);
	debug_data.error.time.om_latency= fabs(msg->ee_profiles.actual_time.om_latency -  msg->ee_profiles.expected_time.om_latency);

	// space
	// space error is caused by the enforcement (operators)
	debug_data.error.space.pc_res= fabs(msg->ee_profiles.actual_cmds.pc_res-  msg->ee_profiles.expected_cmds.pc_res)/msg->ee_profiles.expected_cmds.pc_res;
	debug_data.error.space.pc_vol= fabs(msg->ee_profiles.actual_cmds.pc_vol -  msg->ee_profiles.expected_cmds.pc_vol)/msg->ee_profiles.expected_cmds.pc_vol;
	debug_data.error.space.om_to_pl_res = fabs(msg->ee_profiles.actual_cmds.om_to_pl_res -  msg->ee_profiles.expected_cmds.om_to_pl_res)/msg->ee_profiles.expected_cmds.om_to_pl_res;
	debug_data.error.space.om_to_pl_vol = fabs(msg->ee_profiles.actual_cmds.om_to_pl_vol -  msg->ee_profiles.expected_cmds.om_to_pl_vol)/msg->ee_profiles.expected_cmds.om_to_pl_vol;

	if (msg->ee_profiles.control_flow_path >= 1) { // up to  smootheing planning success
		debug_data.error.space.ppl_vol = fabs(msg->ee_profiles.actual_cmds.ppl_vol -  msg->ee_profiles.expected_cmds.ppl_vol)/msg->ee_profiles.expected_cmds.ppl_vol;
	}
	// slack
	// forced is caused by dataflow control
	if (msg->ee_profiles.control_flow_path == .5) { // if no collision
		double expected_total_slack = fabs(msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce - msg->ee_profiles.expected_time.ee_latency);
		debug_data.slack.expected.total = expected_total_slack;
		debug_data.slack.expected.control_flow = msg->ee_profiles.expected_time.ppl_latency + msg->ee_profiles.expected_time.smoothening_latency;
		debug_data.slack.expected.data_flow = max(expected_total_slack -  debug_data.slack.expected.control_flow, 0.0); // can't allow it to be less than zero

		double actual_total_slack = fabs(msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce - msg->ee_profiles.actual_time.ee_latency);
		debug_data.slack.actual.total = actual_total_slack;
		debug_data.slack.actual.control_flow = msg->ee_profiles.expected_time.ppl_latency + msg->ee_profiles.expected_time.smoothening_latency;  // have to use expected time, cause we
																																				 // the actual is zero
		debug_data.slack.actual.data_flow = max(actual_total_slack -  debug_data.slack.actual.control_flow, 0.0); // can't allow it to be less than zero
	}else if(msg->ee_profiles.control_flow_path != 2) { // if failure
		debug_data.slack.actual.failure =  1;
		debug_data.slack.expected.failure =  1;
	}else{
		double expected_total_slack  = fabs(msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce - msg->ee_profiles.expected_time.ee_latency);
		debug_data.slack.expected.total = expected_total_slack;
		debug_data.slack.expected.data_flow = expected_total_slack;
		debug_data.slack.expected.control_flow = 0;
		double actual_total_slack  = fabs(msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce - msg->ee_profiles.actual_time.ee_latency);
		debug_data.slack.actual.total = actual_total_slack;
		debug_data.slack.actual.data_flow = actual_total_slack;
		debug_data.slack.actual.control_flow = 0;
	}



}

/*
void timing_msgs_from_pd_callback(const std_msgs::Header::ConstPtr& msg) {
    erase_up_to_msg(*msg, "timing_msgs_from_pd_callback");
}
*/

// call back uppon future collision msgs received. deprecated
void future_collision_callback(const mavbench_msgs::future_collision::ConstPtr& msg) {
    if (msg->future_collision_seq > future_collision_seq) {
        future_collision_seq = msg->future_collision_seq;

        if (g_grace_period+g_time_to_come_to_full_stop < msg->time_to_collision)
            future_collision_time = ros::Time::now() + ros::Duration(g_grace_period);
        else
            future_collision_time = ros::Time::now();
    }
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}


// used for connecting the current trajectory to the newly arrived trajectory.
// to smoothen the transition between the two trajectorys (currently not used)
template<class P1, class P2>
trajectory_t straight_line_trajectory(const P1& start, const P2& end, geometry_msgs::Twist twist, double dt)
{
    trajectory_t result;
    double correction_in_x = end.x - start.x;
    double correction_in_y = end.y - start.y;
    double correction_in_z = end.z - start.z;
    double v = distance(twist.linear.x, twist.linear.y, twist.linear.z);

    double correction_distance = distance(correction_in_x, correction_in_y, correction_in_z);
    double correction_time = correction_distance / v;

    //double disc = st0d::min((dt * v) / correction_distance, correction_distance); // The proportion of the correction_distance taken up by each g_dt time step
    double disc = (dt * v);

    double vx = correction_in_x / correction_time;
    double vy = correction_in_y / correction_time;
    double vz = correction_in_z / correction_time;

    double yaw = yawFromVelocity(vx, vy);

    for (double it = disc; it <= correction_distance; it += disc) {
        multiDOFpoint p;

        p.x = start.x + it*correction_in_x;
        p.y = start.y + it*correction_in_y;
        p.z = start.z + it*correction_in_z;

        p.vx = twist.linear.x;
        p.vy = twist.linear.y;
        p.vz = twist.linear.z;

        p.yaw = yaw;
        p.blocking_yaw = false;

        p.duration = dt;

        result.push_back(p);
    }

    return result;
}

// used for backing up
template<class P1, class P2>
trajectory_t straight_line_trajectory(const P1& start, const P2& end, double v)
{
    trajectory_t result;

    const double dt = 0.1;

    double correction_in_x = end.x - start.x;
    double correction_in_y = end.y - start.y;
    double correction_in_z = end.z - start.z;

    double correction_distance = distance(correction_in_x, correction_in_y, correction_in_z);
    double correction_time = correction_distance / v;

    double disc = std::min((dt * v) / correction_distance, 1.0); // The proportion of the correction_distance taken up by each g_dt time step

    double vx = correction_in_x / correction_time;
    double vy = correction_in_y / correction_time;
    double vz = correction_in_z / correction_time;

    double yaw = yawFromVelocity(vx, vy);

    for (double it = 0; it <= 1.0; it += disc) {
        multiDOFpoint p;

        p.x = start.x + it*correction_in_x;
        p.y = start.y + it*correction_in_y;
        p.z = start.z + it*correction_in_z;

        p.vx = vx;
        p.vy = vy;
        p.vz = vz;

        p.yaw = yaw;
        p.blocking_yaw = false;

        p.duration = dt;

        result.push_back(p);
    }

    return result;
}


// for micro_benchmarking purposes
// this is called uppon setting the micro_benchmark_flag
void micro_benchmark_func(int micro_benchmark_number, int replanning_reason, Drone *drone){
	if (micro_benchmark_number == 1){ // microbenchmark description: to stop fully once saw an obstacle and quit
		if (replanning_reason == 1) {

			//collect infomation
			float sleep_duration = .01;
			int cntr = 0;
			geometry_msgs::Twist twist = drone->velocity();
			double velocity_magnitude =  calc_vec_magnitude(twist.linear.x, twist.linear.y, twist.linear.z);
			if (!micro_benchmark_signaled_supervisor) profiling_container->capture("velocity_before_stoppping", "single", velocity_magnitude, g_capture_size);

			drone->fly_velocity(0, 0, 0, 10);
			ROS_WARN("stopping the dronne");
			while (std::round(100*velocity_magnitude) != 0){
				ros::Duration(sleep_duration).sleep();
				cntr +=1;
				twist = drone->velocity();
				velocity_magnitude =  calc_vec_magnitude(twist.linear.x, twist.linear.y, twist.linear.z);
			}
			if (!micro_benchmark_signaled_supervisor) profiling_container->capture("stoppage_time", "single", cntr*sleep_duration, g_capture_size);
			signal_supervisor(g_supervisor_mailbox, "kill");
			micro_benchmark_signaled_supervisor = true;
		}
	}
}


void callback_trajectory(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg, Drone * drone)
{
	//for profiling SA
	erase_up_to_msg(msg->header, "callback_trajectory"); //erase the predecessors of the msg that currently reside in the timing_msgs_vec
	closest_unknown_point = msg->closest_unknown_point;
    // Check for trajectories that arrive out of order
	if (msg->trajectory_seq < trajectory_seq) {
        ROS_ERROR("follow_trajectory: Trajectories arrived out of order! New seq: %d, old seq: %d", msg->trajectory_seq, trajectory_seq);
        return;
    } else
        trajectory_seq = msg->trajectory_seq;

    // Check for trajectories that are not updated to the latest collision detection
    if (msg->future_collision_seq < future_collision_seq) {
        ROS_ERROR("Proposed trajectory does not consider latest detected collision");
        return;
    } else {
        future_collision_seq = msg->future_collision_seq;
        future_collision_time = ros::Time(0);
    }

    trajectory_t new_trajectory = create_trajectory_from_msg(*msg);

//    bool optimizer_succeeded;
 //   ros::param::get("/optimizer_succeeded", optimizer_succeeded);

    if (msg->reverse) {
    	ROS_INFO_STREAM("reversing now");
    	fly_backward = true;
    	stop = false;
    } else if (msg->stop ) {// !optimizer_succeeded){
    	fly_backward = false;
    	stop = true;
    }
    else if (trajectory.empty() && !new_trajectory.empty()) {
    	ROS_INFO_STREAM("traj empty but new traj not empty");
    	// Add drift correction if the drone is currently idling (because it will float around while idling)
        const double max_idling_drift_distance = 0.5;
        trajectory_t idling_correction_traj;

        if (distance(drone->position(), new_trajectory.front()) > max_idling_drift_distance)
             idling_correction_traj = straight_line_trajectory(drone->position(), new_trajectory.front(), 1.0);

        trajectory = append_trajectory(idling_correction_traj, new_trajectory);
        fly_backward = false;
        stop = false;
    } else if (msg->append) {
    	ROS_INFO_STREAM("append crap");
    	trajectory = append_trajectory(trajectory, new_trajectory);
        fly_backward = false;
        stop = false;
    } else {
    	ROS_INFO_STREAM("else cond");
    	const double max_idling_drift_distance = 0.5;
        trajectory_t idling_correction_traj;
        trajectory_t trajectory_annex;
        /*
        if (distance(drone->position(), new_trajectory.front()) > max_idling_drift_distance)
             trajectory_annex = straight_line_trajectory(drone->position(), new_trajectory.front(), drone->velocity(), g_sampling_time_interval);
        //trajectory = append_trajectory(trajectory_annex, new_trajectory);
        */
        trajectory = new_trajectory;
        fly_backward = false;
        stop = false;
    }

    g_got_new_trajectory = true;
    last_new_trajectory_time = ros::Time::now();
    new_trajectory_since_backed_up = !(new_trajectory.empty());

    // profiling
    if (measure_time_end_to_end) {
    	new_traj_msg_time_stamp = msg->header.stamp;
    	profiling_container->capture("sensor_to_follow_trajectorytime", "single", (ros::Time::now() - msg->header.stamp).toSec(), g_capture_size);
    	ROS_INFO_STREAM("end to end"<< (ros::Time::now() - msg->header.stamp).toSec());
    }else{
    	profiling_container->capture("motion_planner_to_follow_traj_com", "start", msg->header.stamp, g_capture_size);
    	profiling_container->capture("motion_planner_to_follow_traj_com", "end", ros::Time::now(), g_capture_size);
    }
    replanning_reason = msg->replanning_reason; // @suppress("Field cannot be resolved")
    planning_status = msg->planning_status;


	if(measure_time_end_to_end && !micro_benchmark_signaled_supervisor) profiling_container->capture("S_A_latency", "start", new_traj_msg_time_stamp, g_capture_size);
	if (measure_time_end_to_end && !micro_benchmark_signaled_supervisor) profiling_container->capture("S_A_latency", "end", ros::Time::now(), g_capture_size);
	if (SA_response_time_capture_ctr >=1 and !micro_benchmark_signaled_supervisor) { //>=1 cause the first one is really big due to pre_mission steps
		profiling_container->capture("S_A_response_time", "single",
				(ros::Time::now() - timing_msgs_begin_el_time).toSec(), g_capture_size); //ignoring the first planning since it takse forever
		profiling_container->capture("S_A_response_time_" + planning_status, "single",
				(ros::Time::now() - timing_msgs_begin_el_time).toSec(), g_capture_size); //ignoring the first planning since it takse forever
		this_response_time_collected_type = "S_A_response_time_" + planning_status;
	}
	SA_response_time_capture_ctr++;

	debug_data.header.stamp = ros::Time::now();
	debug_data.controls = msg->controls;
	debug_data.ee_profiles = msg->ee_profiles;
	debug_data.ee_profiles.actual_time.pl_to_ft_ros_oh = (ros::Time::now() - msg->ee_profiles.actual_time.pl_pre_pub_time_stamp).toSec();
	debug_data.ee_profiles.actual_time.ee_latency = (ros::Time::now() - msg->ee_profiles.actual_time.img_capture_time_stamp).toSec();
	debug_data.ee_profiles.control_flow_path =  msg->ee_profiles.control_flow_path;




	debug_data.error.time.ppl_latency = -1;
	debug_data.error.time.smoothening_latency = -1;
	debug_data.error.time.ee_latency = -1;
	debug_data.error.space.ppl_vol = -1;
	debug_data.slack.expected.failure = -1;
	debug_data.slack.expected.data_flow =  -1;
	debug_data.slack.expected.control_flow =  -1;
	debug_data.slack.expected.total = -1;
	debug_data.slack.actual.failure = -1;
	debug_data.slack.actual.data_flow =  -1;
	debug_data.slack.actual.control_flow =  -1;
	debug_data.slack.actual.total = -1;


	if (!debug_data.controls.cmds.log_control_data){
		return;
	}



	// calculate the error
	// time error is caused by a combination of models(governer) and enforcement(operators)
	if (msg->ee_profiles.control_flow_path >= 1) { // made it up to end of planning
		debug_data.error.time.ppl_latency = fabs(msg->ee_profiles.actual_time.ppl_latency -  msg->ee_profiles.expected_time.ppl_latency);
	}
	if (msg->ee_profiles.control_flow_path >= 1.5) { // made it up to end of  smootheing
		debug_data.error.time.smoothening_latency = fabs(msg->ee_profiles.actual_time.smoothening_latency -  msg->ee_profiles.expected_time.smoothening_latency);
	}
	if (msg->ee_profiles.control_flow_path == 2) { // if sucess all the way
		debug_data.error.time.ee_latency = fabs(msg->ee_profiles.actual_time.ee_latency -  msg->ee_profiles.expected_time.ee_latency);
	}
	// the next two always run
	debug_data.error.time.om_to_pl_latency = fabs(msg->ee_profiles.actual_time.om_to_pl_latency-  msg->ee_profiles.expected_time.om_to_pl_latency);
	debug_data.error.time.om_latency= fabs(msg->ee_profiles.actual_time.om_latency -  msg->ee_profiles.expected_time.om_latency);

	// space
	// space error is caused by the enforcement (operators)
	debug_data.error.space.pc_res= fabs(msg->ee_profiles.actual_cmds.pc_res-  msg->ee_profiles.expected_cmds.pc_res)/msg->ee_profiles.expected_cmds.pc_res;
	debug_data.error.space.pc_vol= fabs(msg->ee_profiles.actual_cmds.pc_vol -  msg->ee_profiles.expected_cmds.pc_vol)/msg->ee_profiles.expected_cmds.pc_vol;
	debug_data.error.space.om_to_pl_res = fabs(msg->ee_profiles.actual_cmds.om_to_pl_res -  msg->ee_profiles.expected_cmds.om_to_pl_res)/msg->ee_profiles.expected_cmds.om_to_pl_res;
	debug_data.error.space.om_to_pl_vol = fabs(msg->ee_profiles.actual_cmds.om_to_pl_vol -  msg->ee_profiles.expected_cmds.om_to_pl_vol)/msg->ee_profiles.expected_cmds.om_to_pl_vol;



	if (msg->ee_profiles.control_flow_path >= 1) { // up to  smootheing planning success
		debug_data.error.space.ppl_vol = fabs(msg->ee_profiles.actual_cmds.ppl_vol -  msg->ee_profiles.expected_cmds.ppl_vol)/msg->ee_profiles.expected_cmds.ppl_vol;
	}
	// slack
	// forced is caused by dataflow control
	if (msg->ee_profiles.control_flow_path == .5) { // if no collision
		double expected_total_slack = fabs(msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce - msg->ee_profiles.expected_time.ee_latency);
		debug_data.slack.expected.total = expected_total_slack;
		debug_data.slack.expected.control_flow = msg->ee_profiles.expected_time.ppl_latency + msg->ee_profiles.expected_time.smoothening_latency;
		debug_data.slack.expected.data_flow = max(expected_total_slack -  debug_data.slack.expected.control_flow, 0.0); // can't allow it to be less than zero

		double actual_total_slack = fabs(msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce - msg->ee_profiles.actual_time.ee_latency);
		debug_data.slack.actual.total = actual_total_slack;
		debug_data.slack.actual.control_flow = msg->ee_profiles.expected_time.ppl_latency + msg->ee_profiles.expected_time.smoothening_latency;  // have to use expected time, cause we
																																				 // the actual is zero
		debug_data.slack.actual.data_flow = max(actual_total_slack -  debug_data.slack.actual.control_flow, 0.0); // can't allow it to be less than zero
	}else if(msg->ee_profiles.control_flow_path != 2) { // if failure
		debug_data.slack.actual.failure =  1;
		debug_data.slack.expected.failure =  1;
	}else{
		double expected_total_slack  = fabs(msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce - msg->ee_profiles.expected_time.ee_latency);
		debug_data.slack.expected.total = expected_total_slack;
		debug_data.slack.expected.data_flow = expected_total_slack;
		debug_data.slack.expected.control_flow = 0;
		double actual_total_slack  = fabs(msg->controls.internal_states.sensor_to_actuation_time_budget_to_enforce - msg->ee_profiles.actual_time.ee_latency);
		debug_data.slack.actual.total = actual_total_slack;
		debug_data.slack.actual.data_flow = actual_total_slack;
		debug_data.slack.actual.control_flow = 0;
	}
}


bool trajectory_done(const trajectory_t& trajectory) {
//    trajectory.size() == 0;
    g_trajectory_done = (trajectory.size() == 0);
    return g_trajectory_done;
}


void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        signal_supervisor(g_supervisor_mailbox, "kill"); 
        ros::shutdown();
    }
    exit(0);
}


void initialize_global_params() {
	ros::param::get("p_vx", p_vx_original);
	ros::param::get("p_vy", p_vy_original);
	ros::param::get("p_vz", p_vz_original);
	ros::param::get("I_px", I_px);
	ros::param::get("I_py", I_py);
	ros::param::get("I_pz", I_pz);
	ros::param::get("d_px", d_px);
	ros::param::get("d_py", d_py);
	ros::param::get("d_pz", d_pz);

	ros::param::get("PID_correction_time", PID_correction_time);

	 if(!ros::param::get("/DEBUG_RQT", DEBUG_RQT)){
      ROS_FATAL_STREAM("Could not start follow_trajectory DEBUG_RQT not provided");
      return ;
    }

    if(!ros::param::get("/capture_size", g_capture_size)){
      ROS_FATAL_STREAM("Could not start pkg delivery capture_size not provided");
      return ;
    }

	if(!ros::param::get("backup_duration", g_backup_duration))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory backup_duration not provided");
        exit(-1);
	}

	if(!ros::param::get("stay_in_place_duration_for_stop", g_stay_in_place_duration_for_stop))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory stay_in_place_duration_for_stop not provided");
        exit(-1);
	}

	if(!ros::param::get("stay_in_place_duration_for_reverse", g_stay_in_place_duration_for_reverse))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory stay_in_place_duration_for_reverse not provided");
        exit(-1);
	}



	if(!ros::param::get("follow_trajectory_loop_rate", g_follow_trajectory_loop_rate))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory follow_trajectory_loop_rate not provided");
        exit(-1);
    }

	if(!ros::param::get("micro_benchmark_number", micro_benchmark_number))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory micro_benchmark_number not provided");
        exit(-1);
    }

	if(!ros::param::get("/sampling_interval", g_sampling_time_interval))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory sampling_time  interval not provided");
        exit(-1);
    }

	if(!ros::param::get("micro_benchmark", micro_benchmark))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory micro_benchmark not provided");
        exit(-1);
    }

	if(!ros::param::get("supervisor_mailbox", g_supervisor_mailbox))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory supervisor_mailbox not provided");
        exit(-1);
    }


	if(!ros::param::get("v_max", g_v_max))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory vmax not provided");
        exit(-1);
    }

    if(!ros::param::get("max_yaw_rate",g_max_yaw_rate))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate not provided");
        exit(-1);
    }

    if(!ros::param::get("max_yaw_rate_during_flight",g_max_yaw_rate_during_flight))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory max_yaw_rate_during_flight not provided");
        exit(-1);
    }

    if(!ros::param::get("CLCT_DATA",CLCT_DATA))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory CLCT_DATA not provided");
        exit(-1);
    }
    if(!ros::param::get("DEBUG",DEBUG))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory DEBUG not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/fly_trajectory_time_out", g_fly_trajectory_time_out)){
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! fly_trajectory_time_out is not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/grace_period", g_grace_period)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! grace_period is not provided");
        exit(-1);
    }
    double a_max;
    if(!ros::param::get("a_max", a_max))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory amax not provided");
        exit(-1);
    }

    if(!ros::param::get("measure_time_end_to_end", measure_time_end_to_end))  {
        ROS_FATAL_STREAM("Could not start follow_trajectory amax not provided");
        exit(-1);
    }

    g_time_to_come_to_full_stop = g_v_max / a_max;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_trajectory", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, sigIntHandlerPrivate);

    double p_vx = p_vx_original;
    double p_vy = p_vy_original;
    double p_vz = p_vz_original;
    initialize_global_params();

    // Initialize the drone
    std::string localization_method;
    std::string ip_addr;
    const uint16_t port = 41451;

    profile_manager_client = new ProfileManager("client", "/record_profiling_data", "/record_profiling_data_verbose");
    profiling_container = new DataContainer();

    ros::param::get("/follow_trajectory/ip_addr", ip_addr); // @suppress("Invalid arguments")
    ros::param::get("/follow_trajectory/localization_method", localization_method);
    Drone drone(ip_addr.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight);

    // Initialize publishers and subscribers
    ros::Publisher next_steps_pub = n.advertise<mavbench_msgs::multiDOFtrajectory>("/next_steps", 1);

    ros::Subscriber slam_lost_sub = n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    ros::Subscriber col_coming_sub = n.subscribe<mavbench_msgs::future_collision>("/col_coming", 1, future_collision_callback);
    ros::Subscriber timing_msg_sub = n.subscribe<std_msgs::Header> ("/timing_msgs", timing_msgs_channel_size, timing_msgs_callback);
    ros::Subscriber timing_msg_from_mp_sub = n.subscribe<mavbench_msgs::response_time_capture> ("/timing_msgs_from_mp", timing_msgs_channel_size, timing_msgs_from_mp_callback);
    //ros::Subscriber timing_msg_from_pd_sub = n.subscribe<std_msgs::Header> ("/timing_msgs_from_pd", timing_msgs_channel_size, timing_msgs_from_pd_callback);

    // ros::Subscriber traj_sub = n.subscribe<mavbench_msgs::multiDOFtrajectory>("normal_traj", 1, callback_trajectory);
    ros::Subscriber traj_sub = n.subscribe<mavbench_msgs::multiDOFtrajectory>("multidoftraj", 1, boost::bind(callback_trajectory, _1, &drone));


    // for debuggin
    ros::Publisher follow_traj_debug_pub = n.advertise<mavbench_msgs::follow_traj_debug>("/follow_traj_debug", 1);

    // Begin execution loop
    bool app_started = false;  //decides when the first planning has occured
                               //this allows us to activate all the
                               //functionaliy in follow_trajectory accordingly


    bool reset_PID = false; //if set, PID history is reset

    /*
    // lambda function for debugging
    auto publish_debug_data = [&] () {
           i++;
    };
	*/
    bool modeling_first_itr = true;
    bool knob_performance_modeling_for_piecewise_planner = false;
    ros::Rate loop_rate(g_follow_trajectory_loop_rate);
    ros::Time last_time_following = ros::Time::now();
    while (ros::ok()) {
    	profiling_container->capture("entire_follow_trajectory", "start", ros::Time::now(), g_capture_size);
    	ros::param::get("/knob_performance_modeling_for_piecewise_planner", knob_performance_modeling_for_piecewise_planner);

    	if (knob_performance_modeling_for_piecewise_planner){
    		if (modeling_first_itr){ //  --- the following command causes the drone to dip, so to avoid this, we just stop the drone once
    			drone.fly_velocity(0, 0, 0,3);
    		}
    		ros::Duration(5).sleep();
    		modeling_first_itr = false;
    		continue;
    	}

    	ros::spinOnce();

        if ((ros::Time::now() - last_time_following).toSec() < g_fly_trajectory_time_out && !g_got_new_trajectory) {
        	loop_rate.sleep();
			continue;
        }
        last_time_following = ros::Time::now();


        timing_msgs_cntr = 0 ; //resetting the cntr, this is just for profiling

        if (trajectory.size() > 0) {
            app_started = true;
        }

        if (app_started) {
            // Profiling
            if (CLCT_DATA) {
                if (g_got_new_trajectory) {
                    g_rcv_traj_to_follow_traj_acc_t +=
                        (ros::Time::now() - g_recieved_traj_t).toSec()*1e9;
                    if (g_msg_time_stamp.sec != 0) {
                        if ((ros::Time::now() - g_msg_time_stamp).toSec() < 5){
                            g_img_to_follow_acc += (ros::Time::now() - g_msg_time_stamp).toSec()*1e9;
                            g_follow_ctr++;
                        }
                    }
                }
            }
        }
        if (trajectory.size() == 0){
        	continue;
        }

        // Figure out which direction we will fly in
        trajectory_t * forward_traj;
        trajectory_t * rev_traj;

        //yaw_strategy_t yaw_strategy = follow_yaw;
        yaw_strategy_t yaw_strategy = follow_closest_unknown;
        float max_velocity = g_v_max;

        /*
        if (fly_backward && ((ros::Time::now() - last_to_fly_backward).toSec() > 2)) { //prevent continous backward movement
        	ROS_ERROR_STREAM("stop backing off  cause backed off already");

        	drone.fly_velocity(0, 0, 0);
            drone.fly_velocity(0, 0, 0);
            drone.fly_velocity(0, 0, 0);

            mavbench_msgs::multiDOFtrajectory trajectory_msg = create_trajectory_msg(*forward_traj);
			trajectory_msg.future_collision_seq = future_collision_seq;
			trajectory_msg.trajectory_seq = trajectory_seq;
			trajectory_msg.reverse = fly_backward;
			zero_trajectory_msg(trajectory_msg, drone.position()); //zero out, to get the same position as the current position
			next_steps_pub.publish(trajectory_msg);
            loop_rate.sleep();
			continue;
        }
		*/

        if (new_trajectory_since_backed_up || !backed_up){
        	if (!fly_backward && !stop) {
        		forward_traj = &trajectory;
        		rev_traj = &reverse_trajectory;
        		backed_up = false;
        	} else {
        		backed_up = true;
        		forward_traj = &reverse_trajectory;
        		rev_traj = &trajectory;

        		modify_backward_traj(forward_traj, g_backup_duration, g_stay_in_place_duration_for_stop, g_stay_in_place_duration_for_reverse, stop);

        		trajectory_t blah = *forward_traj;
        		yaw_strategy = face_backward;
        		//max_velocity = 3;
        	}
        }

         if (trajectory_done(*forward_traj)){
            loop_rate.sleep();
            continue;
        }

        //bool check_position = (ros::Time::now() - last_new_trajectory_time).toSec() < PID_correction_time;
        bool check_position = true; //always check position now

        // profiling
        /*
        if (g_got_new_trajectory){
        	if(measure_time_end_to_end && !micro_benchmark_signaled_supervisor) profiling_container->capture("S_A_latency", "start", new_traj_msg_time_stamp, g_capture_size);
        	if (measure_time_end_to_end && !micro_benchmark_signaled_supervisor) profiling_container->capture("S_A_latency", "end", ros::Time::now(), g_capture_size);
        	if (SA_response_time_capture_ctr >=1 and !micro_benchmark_signaled_supervisor) { //>=1 cause the first one is really big due to pre_mission steps
        		profiling_container->capture("S_A_response_time", "single",
        				(ros::Time::now() - timing_msgs_begin_el_time).toSec(), g_capture_size); //ignoring the first planning since it takse forever
        	    		profiling_container->capture("S_A_response_time_" + planning_status, "single",
        	    				(ros::Time::now() - timing_msgs_begin_el_time).toSec(), g_capture_size); //ignoring the first planning since it takse forever
        	}
        	SA_response_time_capture_ctr++;
        }
        */
        if (DEBUG_RQT) {
			if (measure_time_end_to_end && !micro_benchmark_signaled_supervisor) debug_data.S_A_latency = profiling_container->findDataByName("S_A_latency")->values.back();
			if (measure_time_end_to_end && !micro_benchmark_signaled_supervisor && profiling_container->findDataByName("S_A_response_time")) debug_data.S_A_response_time = profiling_container->findDataByName("S_A_response_time")->values.back();
			debug_data.S_A_response_time_no_planning_needed = 0;
			debug_data.S_A_response_time_success = 0;
			debug_data.S_A_response_time_piecewise_planning_failed = 0;
			debug_data.S_A_response_time_smoothening_failed = 0;
			if (measure_time_end_to_end && !micro_benchmark_signaled_supervisor && profiling_container->findDataByName("S_A_response_time_no_planning_needed") && this_response_time_collected_type == "S_A_response_time_no_planning_needed") {
				debug_data.S_A_response_time_no_planning_needed = profiling_container->findDataByName("S_A_response_time_no_planning_needed")->values.back();
			}
			if (measure_time_end_to_end && !micro_benchmark_signaled_supervisor && SA_response_time_capture_ctr>=2  && profiling_container->findDataByName("S_A_response_time_success") && this_response_time_collected_type == "S_A_response_time_success") {
				debug_data.S_A_response_time_success = profiling_container->findDataByName("S_A_response_time_success")->values.back();
			}
			if (measure_time_end_to_end && !micro_benchmark_signaled_supervisor && SA_response_time_capture_ctr>=2  && profiling_container->findDataByName("S_A_response_time_piecewise_planning_failed") && this_response_time_collected_type == "S_A_response_time_piecewise_planning_failed"){
				debug_data.S_A_response_time_piecewise_planning_failed = profiling_container->findDataByName("S_A_response_time_piecewise_planning_failed")->values.back();
			}
			if (measure_time_end_to_end && !micro_benchmark_signaled_supervisor && SA_response_time_capture_ctr>=2  && profiling_container->findDataByName("S_A_response_time_smoothening_failed") && this_response_time_collected_type == "S_A_response_time_smoothening_failed"){
				debug_data.S_A_response_time_smoothening_failed = profiling_container->findDataByName("S_A_response_time_smoothening_failed")->values.back();
			}
        }


        // microbenchmarks
        if (micro_benchmark){
    	 micro_benchmark_func(micro_benchmark_number, replanning_reason, &drone);
        }


        // dampening the impact of P controller uppon arrival of a new trajectory
        if ((ros::Time::now() - last_new_trajectory_time).toSec() < 1) {
            double scale = 1/(ros::Time::now() - last_new_trajectory_time).toSec();
        	p_vx /= scale;
        	p_vy /= scale;
        	p_vz /= scale;
        }else{
        	p_vx = p_vx_original;
        	p_vy = p_vy_original;
        	p_vz = p_vz_original;
        }

        // reset PID to avoid violent moves uppon stoping or backing up
        if( fly_backward || stop) {
        	reset_PID = true;
        }else{
        	reset_PID = false;
        }

        double max_velocity_reached = follow_trajectory(drone, forward_traj,
        		// @suppress("Invalid arguments")
                rev_traj,
        		debug_data,
				closest_unknown_point,
				yaw_strategy, check_position , max_velocity,
                g_fly_trajectory_time_out, p_vx, p_vy, p_vz, I_px, I_py, I_pz, d_px, d_py, d_pz, reset_PID);

        profiling_container->capture("velocity", "single", max_velocity_reached, g_capture_size);
        if (max_velocity_reached > g_max_velocity_reached) {
            g_max_velocity_reached = max_velocity_reached;
            //ROS_INFO_STREAM("max speed is"<<max_velocity_reached);
        }
        profiling_container->capture("max_velocity", "single", g_max_velocity_reached, g_capture_size);
        // Publish the remainder of the trajectory
        mavbench_msgs::multiDOFtrajectory trajectory_msg = create_trajectory_msg(*forward_traj, &drone);
        trajectory_msg.future_collision_seq = future_collision_seq;
        trajectory_msg.trajectory_seq = trajectory_seq;
        trajectory_msg.reverse = fly_backward;
        trajectory_msg.stop = stop;


//        if (trajectory_msg.points.size() != 0){
        	next_steps_pub.publish(trajectory_msg);
 //       }

    	profiling_container->capture("entire_follow_trajectory", "end", ros::Time::now(), g_capture_size);
        // debugging
        debug_data.new_plan = g_got_new_trajectory;
        debug_data.fly_backward = fly_backward;
        debug_data.stop = stop;
        debug_data.header.stamp = ros::Time::now();
        debug_data.replanning_reason = replanning_reason;
        //debug_data.planning_failure_short_time = (planning_status == 0) ;
        //debug_data.planning_failure_inital_state = (planning_status == 1);
        //debug_data.planning_success = (planning_status == 2);
        debug_data.vel_magnitude = profiling_container->findDataByName("velocity")->values.back();
        debug_data.entire_follow_trajectory = profiling_container->findDataByName("entire_follow_trajectory")->values.back();




        if (DEBUG_RQT) {follow_traj_debug_pub.publish(debug_data);}
        replanning_reason = 0; //zero it out for the next round

        if (slam_lost){
            ROS_INFO_STREAM("slam loss");
            log_data_before_shutting_down();
            g_localization_status = 0;
            signal_supervisor(g_supervisor_mailbox, "kill");
            ros::shutdown();
        } else if (future_collision_time != ros::Time(0) && ros::Time::now() >= future_collision_time) {
            // Stop the drone if we haven't been able to come up with a new plan in our budgetted time
            ROS_WARN("Motion planner took too long to propose a new path, so the drone is being stopped!");
            drone.fly_velocity(0, 0, 0);
            trajectory.clear();
            future_collision_time = ros::Time(0);
        }
        g_got_new_trajectory = false;
    }

    return 0;
}

