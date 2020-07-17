#include <common.h>
#include <coord.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/time.h>
#include <ros/topic.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <tf/exceptions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <vector>
int clock_speed = 1;
using namespace std;

static const int angular_vel = 15;

static multiDOFpoint reverse_point(multiDOFpoint mdp);

template <class T>
static T magnitude(T a, T b, T c) {
    return std::sqrt(a*a + b*b + c*c);
}


template <class T>
static T last_msg (std::string topic) {
    // Return the last message of a latched topic
    return *(ros::topic::waitForMessage<T>(topic));
}


visualization_msgs::Marker get_marker(multiDOFpoint point_1, multiDOFpoint point_2){
	uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	//marker.ns = "basic_shapes";
	marker.ns = "straight_path";
    marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = point_1.x;
	marker.pose.position.y = point_1.y;
	marker.pose.position.z = point_1.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = .5;
	marker.scale.y = .5;
	marker.scale.z = .5;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	
    marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = point_2.x;
	marker.pose.position.y = point_2.y;
	marker.pose.position.z = point_2.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = .5;
	marker.scale.y = .5;
	marker.scale.z = .5;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration(.5);
	return marker;
}


visualization_msgs::Marker get_marker(multiDOFpoint closest_unknown_point){
	uint32_t shape = visualization_msgs::Marker::SPHERE;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = closest_unknown_point.x;
	marker.pose.position.y = closest_unknown_point.y;
	marker.pose.position.z = closest_unknown_point.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = .5;
	marker.scale.y = .5;
	marker.scale.z = .5;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(.5);
	return marker;
}



void sigIntHandler(int sig)
{
    //ros::shutdown();
    exit(0);
}


trajectory_t create_slam_loss_trajectory(Drone& drone, trajectory_t& normal_traj, const trajectory_t& rev_normal_traj)
{
    trajectory_t result;
    float current_yaw = drone.get_yaw();
    auto current_pos = drone.position();

    // Add pause to trajectory
    multiDOFpoint pause_p;
    pause_p.x = current_pos.x;
    pause_p.y = current_pos.y;
    pause_p.z = current_pos.z;
    pause_p.vx = pause_p.vy = pause_p.vz = 0;
    pause_p.duration = 2.0;
    pause_p.yaw = current_yaw;

    result.push_back(pause_p);

    // Add spinning around to trajectory
    const float scanning_width = 45;

    multiDOFpoint scan_p;
    scan_p.x = current_pos.x;
    scan_p.y = current_pos.y;
    scan_p.z = current_pos.z;
    scan_p.vx = scan_p.vy = scan_p.vz = 0;
    scan_p.duration = scanning_width / drone.maxYawRateDuringFlight();

    float yaws[] = {current_yaw - scanning_width, current_yaw, current_yaw + scanning_width, current_yaw};

    for (float y : yaws) {
        scan_p.yaw = y;
        result.push_back(scan_p);
    }

    // Add backtrack to trajectory
    double distance_left = 500.0; // TODO: make this a reasonable number
    const double safe_v = 1.0;

    for (multiDOFpoint p : rev_normal_traj) {
        double v = magnitude(p.vx, p.vy, p.vz);

        if (v*p.duration > distance_left) {
            p.duration = distance_left / v;
        }

        distance_left -= v*p.duration;
        
        double scale = v > safe_v ? safe_v/v : 1;

        p.vx *= scale;
        p.vy *= scale;
        p.vz *= scale;
        p.duration /= scale;

        result.push_back(p);

        if (distance_left <= 0)
            break;
    }

    // Add one last pause
    multiDOFpoint last_pause_p;
    last_pause_p.x = result.back().x;
    last_pause_p.y = result.back().y;
    last_pause_p.z = result.back().z;
    last_pause_p.vx = last_pause_p.vy = last_pause_p.vz = 0;
    last_pause_p.duration = 2.0;
    last_pause_p.yaw = YAW_UNCHANGED;

    result.push_back(last_pause_p);

    // Slow down normal_traj
    const double max_a = 1.0;
    double max_v = safe_v;

    for (multiDOFpoint& p : normal_traj) {
        double v = magnitude(p.vx, p.vy, p.vz);

        double scale = v > max_v ? max_v/v : 1;

        p.vx *= scale;
        p.vy *= scale;
        p.vz *= scale;
        p.duration /= scale;

        max_v += max_a*p.duration;
    }

    return result;
}

bool reset_slam(Drone& drone, const std::string& topic) {
    ros::NodeHandle nh;
	ros::ServiceClient reset_client = nh.serviceClient<std_srvs::Trigger>("/slam_reset");
    std_srvs::Trigger srv;

    // Reset the SLAM map
    if (reset_client.call(srv)) {
        ROS_INFO("SLAM resetted succesfully");
    } else {
        ROS_ERROR("Failed to reset SLAM");
        return false;
    }

    // Move around a little to initialize SLAM
    drone.fly_velocity(-0.5, 0, 0, 2);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    drone.fly_velocity(0.5, 0, 0, 4);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    drone.fly_velocity(-0.5, 0, 0, 2);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std_msgs::Bool is_lost = last_msg<std_msgs::Bool>(topic);
    return !is_lost.data;
}


float distance(float x, float y, float z) {
  return std::sqrt(x*x + y*y + z*z);
}

double calc_vec_magnitude(double x, double y, double z) {
  return std::sqrt(x*x + y*y + z*z);
}

float calc_vec_magnitude(float x, float y, float z) {
  return std::sqrt(x*x + y*y + z*z);
}

void scan_around(Drone &drone, int angle) {
    float init_yaw = drone.get_yaw();
    ROS_INFO("Scanning around from %f degrees...", init_yaw);

    if (angle > 90) {
		ROS_INFO("we don't have support for angles greater than 90");
        exit(0);
	}

    drone.set_yaw(init_yaw+angle <= 180 ? init_yaw + angle : init_yaw + angle - 360);
    drone.set_yaw(init_yaw);
    drone.set_yaw(init_yaw-angle >= -180 ? init_yaw - angle : init_yaw - angle + 360);
    drone.set_yaw(init_yaw);
}


void spin_around(Drone &drone) {
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ROS_INFO("Spinning around...");
    ros::Time last_time;
    float init_yaw = drone.get_yaw();
    double start_z = drone.pose().position.z; // Get drone's current position

    int angle_corrected;
    for (int i = 0; i <= 360; i += 90) {
        int angle = init_yaw + i;
        angle_corrected  = (angle <= 180 ? angle : angle - 360);
        drone.set_yaw_at_z(angle_corrected, start_z);
        //drone.set_yaw(angle <= 180 ? angle : angle - 360);
    }

    // to correct 
    double dz = start_z - drone.pose().position.z;
    double vz = dz > 0 ? 1 : -1;
    double dt = dz > 0 ? dz : -dz;
    
    drone.fly_velocity(0, 0, vz, YAW_UNCHANGED, dt);
    std::this_thread::sleep_for(std::chrono::milliseconds(int(dt*1000.0)));
    drone.fly_velocity(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


// Follows trajectory, popping commands off the front of it and returning those commands in reverse order
// Also returns the maximum speed reached while flying along trajectory
double follow_trajectory(Drone& drone, trajectory_t * traj,
        trajectory_t * reverse_traj,
		mavbench_msgs::follow_traj_debug& debug_data,
		mavbench_msgs::planner_info closest_unknown_point,
		yaw_strategy_t yaw_strategy,
        bool check_position, float max_speed, float time,
		float p_vx, float p_vy, float p_vz, float I_px, float I_py, float I_pz,
		float d_px, float d_py, float d_pz, bool reset_PID) {
		//debug_follow_trajectory_data& debug_data) {


	//max_speed = min(max_speed, (float)6.5);
	trajectory_t reversed_commands;
    double max_speed_so_far = 0;

    // I controller
    static double accumulated_x_error = 0;
    static double accumulated_y_error = 0;
    static double accumulated_z_error = 0;

    static double prev_x_error = 0;
    static double prev_y_error = 0;
    static double prev_z_error = 0;
    static double prev_vx_error = 0;
    static double prev_vy_error = 0;
    static double prev_vz_error = 0;

    double current_x_error, current_y_error, current_z_error, position_deriv_error_x,  position_deriv_error_y,  position_deriv_error_z;
    double current_vx_error, current_vy_error, current_vz_error;
    double target_x, target_y, target_z;
    double target_vx, target_vy, target_vz;



    static ::geometry_msgs::Vector3 prev_linear_vel;
    static ::geometry_msgs::Vector3 prev_linear_acc;
    static ros::Time last_time_sample = ros::Time::now();

	auto pos = drone.position();
	auto vel = drone.velocity();
	auto acc = drone.acceleration();

	while (time > 0 && traj->size() > 0) {
        multiDOFpoint p = traj->front();


        // for debugging
//        debug_data.vx = p.vx; debug_data.vy = p.vy; debug_data.vz = vz;
       debug_data.velocity.x = p.vx; debug_data.velocity.y = p.vy; debug_data.velocity.z = p.vz;


        // Calculate the velocities we should be flying at
        double v_x = p.vx;
        double v_y = p.vy;
        double v_z = p.vz;

        //ROS_INFO_STREAM("before ~~~~~~vx"<<v_x<<"vy"<<v_y<<"vz"<<v_z) ;

        // reset all the PID values, to avoid over shooting
        if (reset_PID){
        	accumulated_x_error = 0;
			accumulated_y_error = 0;
			accumulated_z_error = 0;

			prev_x_error = 0;
			prev_y_error = 0;
			prev_z_error = 0;
			prev_vx_error = 0;
			prev_vy_error = 0;
			prev_vz_error = 0;

        }

        // use PID for velocity commands (by correcting for position and velocity errors)

        else if (check_position) {

        	//target_x = p.x + .5*p.ax*pow(std::min((double)time, p.duration),2) + p.vx*std::min((double)time, p.duration);  //corrected target, since we slice the way points further
        	//target_y = p.y + .5*p.ay*pow(std::min((double)time, p.duration),2) + p.vy*std::min((double)time, p.duration);  //corrected target, since we slice the way points further
        	//target_z = p.z + .5*p.az*pow(std::min((double)time, p.duration),2) + p.vz*std::min((double)time, p.duration);  //corrected target, since we slice the way points further

        	target_vx = p.vx + p.ax*std::min((double)time, p.duration);
        	target_vy = p.vy + p.ay*std::min((double)time, p.duration);
        	target_vz = p.vz + p.az*std::min((double)time, p.duration);

        	target_x = p.x + target_vx*std::min((double)time, p.duration);
        	target_y = p.y + target_vy*std::min((double)time, p.duration);
        	target_z = p.z + target_vz*std::min((double)time, p.duration);


        	target_vx = p.vx;
        	target_vy = p.vy;
        	target_vz = p.vz;

        	/*
        	target_x = p.x;
        	target_y = p.y;
        	target_z = p.z;
			*/
        	target_x = (*traj)[1].x;
        	target_y = (*traj)[1].y;
        	target_z = (*traj)[1].z;


        	// collecting debug data
        	debug_data.target_acceleration.x = prev_linear_acc.x;
        	debug_data.target_acceleration.y = prev_linear_acc.y;
        	debug_data.target_acceleration.z = prev_linear_acc.z;

        	debug_data.measured_acceleration.x = acc.linear.x;
        	debug_data.measured_acceleration.y = acc.linear.y;
        	debug_data.measured_acceleration.z = acc.linear.z;


        	debug_data.time_between_samples = (ros::Time::now() - last_time_sample).toSec();
        	debug_data.calc_acceleration.x = (vel.linear.x - prev_linear_vel.x)/debug_data.time_between_samples;
        	debug_data.calc_acceleration.y = (vel.linear.y - prev_linear_vel.y)/debug_data.time_between_samples;
        	debug_data.calc_acceleration.z = (vel.linear.z - prev_linear_vel.z)/debug_data.time_between_samples;
        	last_time_sample = ros::Time::now();

        	debug_data.target_velocity.x = target_vx;
        	debug_data.target_velocity.y = target_vy;
        	debug_data.target_velocity.z = target_vz;

        	debug_data.measured_velocity.x = vel.linear.x;
        	debug_data.measured_velocity.y = vel.linear.y;
        	debug_data.measured_velocity.z = vel.linear.z;

        	debug_data.target_position.x = p.x;
        	debug_data.target_position.y = p.y;
        	debug_data.target_position.z = p.z;

        	debug_data.measured_position.x = pos.x;
        	debug_data.measured_position.y = pos.y;
        	debug_data.measured_position.z = pos.z;




        	/*
        	target_x = p.x + p.vx*std::min((double)time, p.duration);
        	target_y = p.y + p.vy*std::min((double)time, p.duration);
        	target_z = p.z + p.vz*std::min((double)time, p.duration);
			*/

        	current_x_error = target_x - pos.x;
        	current_y_error = target_y - pos.y;
        	current_z_error = target_z - pos.z;

        	accumulated_x_error += current_x_error;
        	accumulated_y_error += current_y_error;
        	accumulated_z_error += current_z_error;

        	/*
        	current_vx_error =  p.vx - vel.linear.x;
        	current_vy_error =  p.vy - vel.linear.y;
        	current_vz_error =  p.vz - vel.linear.z;
			*/

        	current_vx_error =  target_vx - vel.linear.x;
        	current_vy_error =  target_vy - vel.linear.y;
        	current_vz_error =  target_vz - vel.linear.z;

        	position_deriv_error_x = current_x_error - prev_x_error;
        	position_deriv_error_y = current_y_error - prev_y_error;
        	position_deriv_error_z = current_z_error - prev_z_error;


        	// tracking velocity, PID is on velocity
        	/*
        	v_x = d_px*(current_vx_error - prev_vx_error) + p_vx*(current_vx_error) + I_px*(current_x_error);
        	v_y = d_py*(current_vy_error - prev_vy_error) + p_vy*(current_vy_error) + I_py*(current_y_error);
        	v_z = d_pz*(current_vz_error - prev_vz_error) + p_vz*(current_vz_error) + I_pz*(current_z_error);
			*/
        	
        	//tracking velocity, PID is on position

        	v_x += d_px*(position_deriv_error_x) + p_vx*(current_x_error) + I_px*(accumulated_x_error);
        	v_y += d_py*(position_deriv_error_y) + p_vy*(current_y_error) + I_py*(accumulated_y_error);
        	v_z +=  d_pz*(position_deriv_error_z) + p_vz*(current_z_error) + I_pz*(accumulated_z_error);


        	//tracking position, PID is on position
           /*
            v_x =  d_px*(current_x_error - prev_x_error)  + p_vx*(current_x_error) + I_px*(accumulated_x_error);
        	v_y = d_py*(current_y_error - prev_y_error) +  p_vy*(current_y_error) + I_py*(accumulated_y_error);
        	v_z = d_pz*(current_z_error - prev_z_error) + p_vz*(current_z_error) + I_pz*(accumulated_z_error);
           */

        	prev_x_error = current_x_error;
        	prev_y_error = current_y_error;
        	prev_z_error = current_z_error;

        	prev_vx_error = current_vx_error;
        	prev_vy_error = current_vy_error;
        	prev_vz_error = current_vz_error;

        	// printing stuff
        	//ROS_INFO_STREAM("==============p.x"<<p.x<<"p.y"<<p.y<<"p.z"<<p.z<< " current vels"<<  pos.x<< " " << pos.y<< "  " << pos.z) ;
        	//ROS_INFO_STREAM("~~~~~~~~~~ error of px "<<p.x - pos.x<<"error py"<<p.y - pos.y<<"error of pz"<<p.z - pos.z);
        	//ROS_INFO_STREAM("~~~~~~~~~~ error of py "<<p.y - pos.y);
        	//ROS_INFO_STREAM("p.vx"<<p.vx<<"p.vy"<<p.vy<<"p.vz"<<p.vz<< " current vels"<<  vel.linear.x<< " " << vel.linear.y<< "  " << vel.linear.z) ;
        	//ROS_INFO_STREAM("error vx "<<current_vx_error<<"error vy"<<current_vy_error<<"error of vz"<<current_vz_error);
        	//ROS_INFO_STREAM("current error x "<<current_x_error<<"current error y"<<current_y_error <<"current z erro "<<current_z_error);
        	//ROS_INFO_STREAM("error vy"<<current_vy_error);

        	// for debugging
        	debug_data.velocity_error.x = current_vx_error;
        	debug_data.velocity_error.y = current_vy_error;
        	debug_data.velocity_error.z = current_vz_error;
        	debug_data.position_error.x = current_x_error;
        	debug_data.position_error.y = current_y_error;
        	debug_data.position_error.z = current_z_error;
        	debug_data.position_acc_error.x = accumulated_x_error;
        	debug_data.position_acc_error.y = accumulated_y_error;
        	debug_data.position_acc_error.z = accumulated_z_error;

        	debug_data.position_deriv_error.x = position_deriv_error_x;
        	debug_data.position_deriv_error.y = position_deriv_error_y;
        	debug_data.position_deriv_error.z = position_deriv_error_z;






        	/*
        	debug_data.velocity_corrected.x = p.vx;
        	debug_data.velocity_corrected.y = p.vy;
        	debug_data.velocity_corrected.z = p.vz;
        */
        	/*
        	debug_data.target_velocity_uncorrected.x = p.vx;
        	debug_data.target_velocity_uncorrected.y = p.vy;
        	debug_data.target_velocity_uncorrected.z = p.vz;
			*/


        	/*
        	debug_data.x_error = current_x_error; debug_data.y_error = current_y_error; debug_data.z_error = current_z_error;
        	debug_data.vx_error = current_vx_error; debug_data.vy_error = current_vy_error; debug_data.vz_error = current_vz_error;
        	debug_data.vx_corrected = v_x;   debug_data.vy_corrected = v_y; debug_data.vz_corrected = v_z;
			*/

        	//ROS_INFO_STREAM("after ========= vx"<<v_x<<"vy"<<v_y<<"vz"<<v_z) ;
        }

        prev_linear_vel.x = vel.linear.x;
        prev_linear_vel.y = vel.linear.y;
        prev_linear_vel.z = vel.linear.z;

        prev_linear_acc.x = p.ax;
        prev_linear_acc.y = p.ay;
        prev_linear_acc.z = p.az;


        // Calculate the yaw we should be flying with
       // float yaw = p.yaw;

       float yaw;
        if (yaw_strategy == ignore_yaw)
            yaw = YAW_UNCHANGED;
        else if (yaw_strategy == face_forward)
            yaw = FACE_FORWARD;
        else if (yaw_strategy == face_backward)
            yaw = FACE_BACKWARD;
        else if (yaw_strategy == follow_closest_unknown){
        	if (isnan(closest_unknown_point.x) || isnan(closest_unknown_point.y)){
        		yaw = p.yaw;
        	}
        	yaw =  90 - atan2(closest_unknown_point.y - pos.y, closest_unknown_point.x - pos.x)*180.0/3.14;
        }
        else
        	yaw = p.yaw;

        // Check whether the yaw needs to be set before we fly
        if (p.blocking_yaw)
            drone.set_yaw(p.yaw);

        // Make sure we're not going over the maximum speed
        double speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
        double scale = 1;

        // make sure that speed doesn't supercede the maximum value

        if (speed > max_speed) {
            scale = max_speed / speed;
            
            v_x *= scale;
            v_y *= scale;
            v_z *= scale;
            speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
        }


        if (speed > max_speed_so_far){
        	max_speed_so_far = speed;
        }

        // Calculate the time for which these flight commands should run
        double flight_time = p.duration <= time ? p.duration : time;
        double scaled_flight_time = flight_time / scale;

        // Fly for flight_time seconds
        auto segment_start_time = std::chrono::system_clock::now();
        drone.fly_velocity(v_x, v_y, v_z, yaw, scaled_flight_time + 1);
        //drone.fly_position(p.y, p.x, -p.z, calc_vec_magnitude(p.vx, p.vy, p.vz), scaled_flight_time); //doesn't work
        
        // uncomment this for sleeping in this thread which is required if we want to update time
        //std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(scaled_flight_time));

        // Push completed command onto reverse-command stack
        multiDOFpoint rev_point = reverse_point(p);
        rev_point.duration = flight_time;
        reversed_commands.push_front(rev_point);

        // Update trajectory
        traj->front().duration -= flight_time/clock_speed;
        traj->front().x = target_x;
        traj->front().y = target_y;
        traj->front().z = target_z;
        traj->front().vx  = target_vx;
        traj->front().vy = target_vy;
        traj->front().vz = target_vz;

        if (traj->front().duration <= 0){
        	traj->pop_front(); // @suppress("Method cannot be resolved")
//        	ROS_INFO_STREAM("pop");
        	break; //if done with the way point, break to
        }

        time -= flight_time;
    }

    if (reverse_traj != nullptr)
        *reverse_traj = append_trajectory(reversed_commands, *reverse_traj);



    return max_speed_so_far;
}


static multiDOFpoint reverse_point(multiDOFpoint mdp) {
    multiDOFpoint point = mdp;

    // scaling everything down (normalizing) to avoid a very fast
    // backing off (and correction on it, since on the way back we start
    // from zero velocity, so error correction would go crazy

    double scale = 1/magnitude(point.vx, point.vy, point.vz); //scale down to slow down to prevent overshooting
    point.vx = -mdp.vx*scale;
    point.vy = -mdp.vy*scale;
    point.vz = -mdp.vz*scale;
    point.ax = -mdp.ax*scale;
    point.ay = -mdp.ay*scale;
    point.az = -mdp.az*scale;

    return point;
}


trajectory_t append_trajectory (trajectory_t first, const trajectory_t& second) {
    first.insert(first.end(), second.begin(), second.end());
	auto it=first.begin();                                                                          while((it!=first.end() - 1) && it !=(first.end())) {                                             
		if (it->x == (it+1)->x &&                                                                    
				it->y == (it+1)->y &&                                                                
				it->z == (it+1)->z){                                                                 
			it->duration += (it+1)->duration;                                                        
			it = first.erase(it+1);                                                                  
		}else{                                                                                       
			it++;                                                                                    
		}                                                                           
    }    
	return first;
}

float yawFromQuat(geometry_msgs::Quaternion q)
{
	float roll, pitch, yaw;

	// Formulas for roll, pitch, yaw
	// roll = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y) );
	// pitch = asin(2*(q.w*q.y - q.z*q.x));
	yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    yaw = (yaw*180)/3.14159265359;

    return (yaw <= 180 ? yaw : yaw - 360);
}

trajectory_t shift_trajectory(const trajectory_t &traj, Drone *drone){
	trajectory_t shifted_traj = traj;
	auto first_point = traj.front();
	coord cur_point = drone->position();
	multiDOFpoint shift_by;
	shift_by.x = cur_point.x - first_point.x;
	shift_by.y = cur_point.y - first_point.y;
	shift_by.z = cur_point.z - first_point.z;

	for (auto &point: shifted_traj){
		point.x += shift_by.x;
		point.y += shift_by.y;
		point.z += shift_by.z;
	}
	return shifted_traj;
}

trajectory_t create_trajectory_from_msg(const mavbench_msgs::multiDOFtrajectory& t)
{

    trajectory_t result;
    for (const auto& mdp_msg : t.points) {
        multiDOFpoint mdp;

        mdp.x = mdp_msg.x;
        mdp.y = mdp_msg.y;
        mdp.z = mdp_msg.z;

        mdp.vx = mdp_msg.vx;
        mdp.vy = mdp_msg.vy;
        mdp.vz = mdp_msg.vz;
//        ROS_INFO_STREAM("=====vx"<<mdp.vx<<"vy"<<mdp.vy<<"vz"<<mdp.vz) ;

        mdp.ax = mdp_msg.ax;
        mdp.ay = mdp_msg.ay;
        mdp.az = mdp_msg.az;

        mdp.yaw = mdp_msg.yaw;
        mdp.blocking_yaw = mdp_msg.blocking_yaw;
        mdp.pt_ctr = mdp_msg.pt_ctr;
        mdp.duration = mdp_msg.duration;

        result.push_back(mdp);
    }

    return result;
}


double correct_distance( double cur_vel_mag, double max_vel, double max_drone_radius, double min_drone_radius, double distance){
	return distance - max((cur_vel_mag/max_vel)*max_drone_radius, min_drone_radius);
}



void clear_traj(trajectory_t *traj, float backup_duration, float stay_in_place_duration_for_stop , float stay_in_place_duration_for_reverse, bool stop){
	double fly_down_vel = 2;
	double time = 0;
	int ctr = 0;
	int sign = 1;
	double duration_incr = traj->front().duration;
	double first_point_x = traj->front().x;
	double first_point_y = traj->front().y;
	double first_point_z = traj->front().z;
	float stay_still_duration = 0;
	traj->clear();
}


// set the velocity of the trajectory after certain duration to zero
// set the position of the trajectory waypoints after certain duratrion to the last point (immediately before the end of the duration)





// set the the z velocity after certain duration  to -1 to come down and avoid moving out of the boundary
void modify_backward_traj(trajectory_t *traj, float backup_duration, float stay_in_place_duration_for_stop , float stay_in_place_duration_for_reverse, bool stop){
	double fly_down_vel = 2;
	double time = 0;
	int ctr = 0;
	int sign = 1;
	double duration_incr = traj->front().duration;
	double first_point_x = traj->front().x;
	double first_point_y = traj->front().y;
	double first_point_z = traj->front().z;
	float stay_still_duration = 0;


	if (stop) {
		stay_still_duration = stay_in_place_duration_for_stop;
	} else{
		stay_still_duration = stay_in_place_duration_for_reverse;
	}

	// only reverse up to backup_duration
	// and then stop
	multiDOFpoint last_point, cur_point;
	for (auto &point: *traj){

		if (isnan(point.vx) || isnan(point.vy) || isnan(point.vz) ){
			point.vx = 0;
			point.vy = 0;
			point.vz = 0;
		}

		auto cur_point = point;
		if(time < backup_duration) {
			last_point = cur_point;
		}else{ //zero out the velocity of the rest of the points
			// zero out vs and as
			point.vx = 0;
			point.vy = 0;
			point.vz = 0;

			point.ax = 0 ;
			point.ay = 0;
			point.az = 0;
			point.x = last_point.x + sign*.75;  //need this otherwise, we'll get ray tracing errors
			point.y = last_point.y + sign*.75;
			point.z = last_point.z + sign*.75;
			sign *= -1;
		}
		time += point.duration;
	}

	// stop before reversing, so zero out the velocities
	for (time = 0; time < stay_still_duration; time +=duration_incr){
		multiDOFpoint point;

		// zero out vs and as
		point.vx = 0;
		point.vy = 0;
		point.vz = 0;

		point.ax = 0 ;
		point.ay = 0;
		point.az = 0;
		point.x = first_point_x + sign*.75;  //need this otherwise, we'll get ray tracing errors
		point.y = first_point_y + sign*.75;
		point.z = first_point_z + sign*.75;

		point.duration = duration_incr;
		traj->push_front(point);
		sign *= -1;
	}
}


void zero_trajectory_msg(mavbench_msgs::multiDOFtrajectory &traj, coord current_pos){
    for (int i = 0; i < traj.points.size() - 1; ++i) {
		traj.points[i].x = current_pos.x;
		traj.points[i].y = current_pos.y;
		traj.points[i].z = current_pos.z;

		// zero out vs and as
		traj.points[i].vx = 0;
		traj.points[i].vy = 0;
		traj.points[i].vz = 0;

		traj.points[i].ax = 0 ;
		traj.points[i].ay = 0;
		traj.points[i].az = 0;
	}
}
mavbench_msgs::multiDOFtrajectory create_trajectory_msg(const trajectory_t& t, Drone *drone)
{

	trajectory_t traj, shifted_traj;
	/*
	// not using it any more, since the trajectory blindely shifted keeps colliding
	// instead, are correcting the initial position on the planning side
	if (t.size() != 0){
		shifted_traj = shift_trajectory(t, drone);
		traj = shifted_traj;
	}
	*/
	traj = t;
	mavbench_msgs::multiDOFtrajectory result;

    result.append = false;
    result.reverse = false;

    for (const auto& mdp : traj) {
        mavbench_msgs::multiDOFpoint mdp_msg;

        mdp_msg.x = mdp.x;
        mdp_msg.y = mdp.y;
        mdp_msg.z = mdp.z;

        mdp_msg.vx = mdp.vx;
        mdp_msg.vy = mdp.vy;
        mdp_msg.vz = mdp.vz;

        mdp_msg.ax = mdp.ax;
        mdp_msg.ay = mdp.ay;
        mdp_msg.az = mdp.az;

        mdp_msg.yaw = mdp.yaw;
        mdp_msg.blocking_yaw = mdp.blocking_yaw;
        mdp_msg.pt_ctr = mdp.pt_ctr;
        mdp_msg.duration = mdp.duration;

        result.points.push_back(mdp_msg);
    }

    return result;
}

void waitForLocalization(std::string method)
{
    // Wait for the localization method to come online
    tf::TransformListener tfListen;
    while(1) {
        try {
            tf::StampedTransform tf;
            tfListen.lookupTransform("/world", "/"+method, ros::Time(0), tf);
            break;
        } catch(tf::TransformException& ex) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

float yawFromVelocity(float vx, float vy)
{
    if (vx == 0 && vy == 0)
        return YAW_UNCHANGED;
    return 90 - atan2(vy, vx)*180.0/3.14;
}

multiDOFpoint trajectory_at_time(const trajectory_t& traj, double t)
{
    for (const auto& mdofp : traj) {
        t -= mdofp.duration;

        if (t <= 0)
            return mdofp;
    }

    return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0};
}

multiDOFpoint trajectory_at_time(const mavbench_msgs::multiDOFtrajectory& traj, double t)
{
    for (const auto& mdofp : traj.points) {
        t -= mdofp.duration;

        if (t <= 0)
            return {mdofp.x, mdofp.y, mdofp.z, mdofp.vx, mdofp.vy, mdofp.vz,
                mdofp.ax, mdofp.ay, mdofp.az, mdofp.yaw, mdofp.blocking_yaw,
                mdofp.duration};
    }


    //ROS_INFO_STREAM("this should not happen");
    //exit(0);
    return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0};
}

