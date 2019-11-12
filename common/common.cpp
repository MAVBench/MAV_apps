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
        trajectory_t * reverse_traj, yaw_strategy_t yaw_strategy,
        bool check_position, float max_speed, float time, float p_vx, float p_vy, float p_vz, float I_px, float I_py, float I_pz, float d_px, float d_py, float d_pz) {

    trajectory_t reversed_commands;
    double max_speed_so_far = 0;
    ros::Time start_hook_t;

    // I controller
    static double accumulated_x_error = 0;
    static double accumulated_y_error = 0;
    static double accumulated_z_error = 0;

    double prev_x_error = 0;
    double prev_y_error = 0;
    double prev_z_error = 0;
    double prev_vx_error = 0;
    double prev_vy_error = 0;
    double prev_vz_error = 0;

    double current_x_error, current_y_error, current_z_error;
    double current_vx_error, current_vy_error, current_vz_error;
    double target_x, target_y, target_z;

    while (time > 0 && traj->size() > 0) {
        start_hook_t = ros::Time::now();  
        multiDOFpoint p = traj->front();

        // Calculate the velocities we should be flying at
        double v_x = p.vx;
        double v_y = p.vy;
        double v_z = p.vz;
        //ROS_INFO_STREAM("~~~~~~vx"<<v_x<<"vy"<<v_y<<"vz"<<v_z) ;

        if (check_position) {
        	auto pos = drone.position();
        	auto vel = drone.velocity();

        	target_x = p.x + p.vx*std::min((double)time, p.duration);  //corrected target, since we slice the way points further
        	target_y = p.y + p.vy*std::min((double)time, p.duration);
        	target_z = p.z + p.vz*std::min((double)time, p.duration);

        	current_x_error = target_x - pos.x;
        	current_y_error = target_y - pos.y;
        	current_z_error = target_z - pos.z;

        	accumulated_x_error += current_x_error;
        	accumulated_y_error += current_y_error;
        	accumulated_z_error += current_z_error;

        	current_vx_error =  p.vx - vel.linear.x;
        	current_vy_error =  p.vy - vel.linear.y;
        	current_vz_error =  p.vz - vel.linear.z;

        	// tracking velocity, PID is on velocity
        	/*
        	v_x += p_vx*(current_vx_error) + I_px*(current_x_error) + d_px*(current_vx_error - prev_vx_error);
        	v_y += p_vy*(current_vy_error) + I_py*(current_y_error) + d_py*(current_vy_error - prev_vy_error);
        	v_z += p_vz*(current_vz_error) + I_pz*(current_z_error) + d_pz*(current_vz_error - prev_vz_error);
        	 */

        	/*
        	//tracking position, PID is on velocity
        	v_x +=  p_vx*(current_x_error) + I_px*(accumulated_x_error) + d_px*(current_x_error - prev_x_error);
        	v_y +=  p_vy*(current_y_error) + I_py*(accumulated_y_error)+ d_py*(current_y_error - prev_y_error);
        	v_z +=  p_vz*(current_z_error) + I_pz*(accumulated_z_error) + d_pz*(current_z_error - prev_z_error);
        	 */

        	//tracking position, PID is on position
        	v_x =  p_vx*(current_x_error) + I_px*(accumulated_x_error) + d_px*(current_x_error - prev_x_error);
        	v_y =  p_vy*(current_y_error) + I_py*(accumulated_y_error)+ d_py*(current_y_error - prev_y_error);
        	v_z =  p_vz*(current_z_error) + I_pz*(accumulated_z_error) + d_pz*(current_z_error - prev_z_error);

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
        	//ROS_INFO_STREAM("error vy"<<current_vy_error);
        }

        // Calculate the yaw we should be flying with
        float yaw = p.yaw;
        if (yaw_strategy == ignore_yaw)
            yaw = YAW_UNCHANGED;
        else if (yaw_strategy == face_forward)
            yaw = FACE_FORWARD;
        else if (yaw_strategy == face_backward)
            yaw = FACE_BACKWARD;

        // Check whether the yaw needs to be set before we fly
        if (p.blocking_yaw)
            drone.set_yaw(p.yaw);

        // Make sure we're not going over the maximum speed
        double speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
        double scale = 1;

        // make sure that speed doesn't supercede the maximum value
        /*
        if (speed > max_speed && !check_position) {
            scale = max_speed / speed;
            
            v_x *= scale;
            v_y *= scale;
            v_z *= scale;
            speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
        }
		*/

        if (speed > max_speed_so_far){
        	max_speed_so_far = speed;
        }

        // Calculate the time for which these flight commands should run
        double flight_time = p.duration <= time ? p.duration : time;
        double scaled_flight_time = flight_time / scale;

        // Fly for flight_time seconds
        auto segment_start_time = std::chrono::system_clock::now();
        drone.fly_velocity(v_x, v_y, v_z, yaw, scaled_flight_time);
        //drone.fly_position(p.y, p.x, -p.z, calc_vec_magnitude(p.vx, p.vy, p.vz), scaled_flight_time); //doesn't work
        
        std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(scaled_flight_time));

        // Push completed command onto reverse-command stack
        multiDOFpoint rev_point = reverse_point(p);
        rev_point.duration = flight_time;
        reversed_commands.push_front(rev_point);

        // Update trajectory
        traj->front().duration -= flight_time;
        traj->front().x = target_x;
        traj->front().y = target_y;
        traj->front().z = target_z;
        if (traj->front().duration <= 0){
        	traj->pop_front(); // @suppress("Method cannot be resolved")
        	break; //if done with the way point, break to
        }

        time -= flight_time;
    }

    if (reverse_traj != nullptr)
        *reverse_traj = append_trajectory(reversed_commands, *reverse_traj);

    return max_speed_so_far;
}


static multiDOFpoint reverse_point(multiDOFpoint mdp) {
    multiDOFpoint result = mdp;

    result.vx = -mdp.vx;
    result.vy = -mdp.vy;
    result.vz = -mdp.vz;

    return result;
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
        //ROS_INFO_STREAM("=====vx"<<mdp.vx<<"vy"<<mdp.vy<<"vz"<<mdp.vz) ;

        mdp.ax = mdp_msg.ax;
        mdp.ay = mdp_msg.ay;
        mdp.az = mdp_msg.az;

        mdp.yaw = mdp_msg.yaw;
        mdp.blocking_yaw = mdp_msg.blocking_yaw;

        mdp.duration = mdp_msg.duration;

        result.push_back(mdp);
    }

    return result;
}

mavbench_msgs::multiDOFtrajectory create_trajectory_msg(const trajectory_t& t)
{
    mavbench_msgs::multiDOFtrajectory result;

    result.append = false;
    result.reverse = false;

    for (const auto& mdp : t) {
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

    return {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0};
}

