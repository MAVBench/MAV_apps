#include "common.h"

#include <iostream>
#include <fstream>
#include <string>
#include <deque>
#include <algorithm>
#include <cmath>
#include <cstdarg>

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "Drone.h"
#include "autoencoder.h"

using namespace std;

static const int angular_vel = 15;
std::vector<double> g_x = {0.0, 0.0};
std::vector<double> g_y = {0.0, 0.0};
std::vector<double> g_z = {0.0, 0.0};
std::vector<double> g_vx = {0.0, 0.0};
std::vector<double> g_vy = {0.0, 0.0};
std::vector<double> g_vz = {0.0, 0.0};
std::vector<double> g_ax = {0.0, 0.0};
std::vector<double> g_ay = {0.0, 0.0};
std::vector<double> g_az = {0.0, 0.0};
std::vector<double> g_yaw = {0.0, 0.0};
std::vector<double> g_duration = {0.0, 0.0};
std::vector<double> g_drone_vx = {0.0, 0.0};
std::vector<double> g_drone_vy = {0.0, 0.0};
std::vector<double> g_drone_vz = {0.0, 0.0};
std::vector<double> g_drone_duration = {0.0, 0.0};
bool recompute;
//double threshold = 0.08;
//double threshold = 0.6;
double scale_threshold = 1;

//std::list<float> error_list;

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

std::vector<float> inference(std::vector<double>& tmp, Drone& drone, int detect){
    /*auto result = drone->encoder.model_all.predict(
        {fdeep::tensor(fdeep::tensor_shape(15),
        fdeep::float_vec{tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], tmp[9], tmp[10], tmp[11], tmp[12], tmp[13], tmp[14]})});
    return result[0].to_vector();*/
    auto result = drone.encoder.inference(tmp, detect);
    return result;
}
void autoencoder_detect(Drone& drone, int detect){
    std::vector<double> tmp;
    //std::vector<float> *result_vec;
    double accum;
    recompute = false;
    fdeep::float_vec input;
    tmp.clear();
    for (int j=0; j < 15; j++){
        if(j == 0)
            tmp.push_back(g_x[1] - g_x[0]);
        else if (j==1)
            tmp.push_back(g_y[1] - g_y[0]);
        else if (j==2)    
            tmp.push_back(g_z[1] - g_z[0]);
        else if (j==3)    
            tmp.push_back(g_vx[1] - g_vx[0]);
        else if (j==4)    
            tmp.push_back(g_vy[1] - g_vy[0]);
        else if (j==5)    
            tmp.push_back(g_vz[1] - g_vz[0]);
        else if (j==6)    
            tmp.push_back(g_ax[1] - g_ax[0]);
        else if (j==7)    
            tmp.push_back(g_ay[1] - g_ay[0]);
        else if (j==8)    
            tmp.push_back(g_az[1] - g_az[0]);
        /*else if (j==9){ 
            if(g_yaw[1] == YAW_UNCHANGED){
                g_yaw[1] = g_yaw[0];
                tmp.push_back(0);
            } 
            else{
                tmp.push_back(g_yaw[1] - g_yaw[0]);
            }
        }
        else if (j==10)   
            tmp.push_back(g_duration[1] - g_duration[0]);*/
        else if (j==11)
            tmp.push_back(g_drone_vx[1] - g_drone_vx[0]);
        else if (j==12)
            tmp.push_back(g_drone_vy[1] - g_drone_vy[0]);
        else if (j==13)
            tmp.push_back(g_drone_vz[1] - g_drone_vz[0]);
        else if (j==14)
            tmp.push_back(g_drone_duration[1] - g_drone_duration[0]);
    }
    drone.x_list.push_back(g_x[1] - g_x[0]);
    drone.y_list.push_back(g_y[1] - g_y[0]);
    drone.z_list.push_back(g_z[1] - g_z[0]);
    drone.vx_list.push_back(g_vx[1] - g_vx[0]);
    drone.vy_list.push_back(g_vy[1] - g_vy[0]);
    drone.vz_list.push_back(g_vz[1] - g_vz[0]);
    drone.ax_list.push_back(g_ax[1] - g_ax[0]);
    drone.ay_list.push_back(g_ay[1] - g_ay[0]);
    drone.az_list.push_back(g_az[1] - g_az[0]);
    drone.yaw_list.push_back(g_yaw[1] - g_yaw[0]);
    drone.duration_list.push_back(g_duration[1] - g_duration[0]);
    drone.drone_vx_list.push_back(g_drone_vx[1] - g_drone_vx[0]);
    drone.drone_vy_list.push_back(g_drone_vy[1] - g_drone_vy[0]);
    drone.drone_vz_list.push_back(g_drone_vz[1] - g_drone_vz[0]);
    drone.drone_duration_list.push_back(g_drone_duration[1] - g_drone_duration[0]);
    //std::cout << "g_x[1]: " << g_x[1] << "g_x[0]"<< g_x[0] << " tmp: " << tmp[0] << endl;
    //std::cout << "g_x[0]"<< g_x[0] << endl;
    //result_vec = result.to_vector();
    //std::cout << "result size: " << result.size() << std::endl;
    auto result_vec = inference(tmp, drone, detect);
    accum = 0;
    /*if(detect == 3){
        threshold = 0.08;
    }
    else{
        threshold = 1.3;
    }*/
    for(int element = 0; element < result_vec.size(); element++){
        accum = accum + (result_vec[element] - tmp[element]) * (result_vec[element] - tmp[element]);  
        if(accum > (drone.threshold)){
            std::cout <<"element: " << element << " construction error: " << accum << "result_vec[element]: " << result_vec[element] << "tmp[element]): "<< tmp[element] << "threshold: " << drone.threshold << "\n";
        }
    }
    /*std::cout << "result: ";
    for(int element = 0; element < result_vec.size(); element++){
        std::cout << result_vec[element] <<" ,";
    }
    std::cout << "\n";
    std::cout << "input: ";
    for(int element = 0; element < result_vec.size(); element++){
        std::cout << tmp[element] <<" ,";
    }
    std::cout << "\n";*/
    result_vec.clear();
    drone.error_list.push_back(accum);
    if(accum > (drone.threshold)){
        recompute = true;
        std::cout <<" construction error: " << accum << "\n";
    }
    //std::cout << fdeep::show_tensors(result) << std::endl;
    //encoder.inference(input);
    ros::Time end_ros;
    if(recompute){
        end_ros = ros::Time::now();
        drone.recompute_time.push_back((end_ros - drone.start_ros).toSec());
        //num_recompute++;
        std::cout << "recompute: " << recompute << "\n";
        std::cout << "\n";
    }
}

// Follows trajectory, popping commands off the front of it and returning those commands in reverse order
// Also returns the maximum speed reached while flying along trajectory
double follow_trajectory(Drone& drone, trajectory_t * traj,
        trajectory_t * reverse_traj, yaw_strategy_t yaw_strategy,
        bool check_position, float max_speed, float time, int noise_select, int range_select_drone, int detect) {
    /*g_x = {0.0, 0.0};
    g_y = {0.0, 0.0};
    g_z = {0.0, 0.0};
    g_vx = {0.0, 0.0};
    g_vy = {0.0, 0.0};
    g_vz = {0.0, 0.0};
    g_ax = {0.0, 0.0};
    g_ay = {0.0, 0.0};
    g_az = {0.0, 0.0};
    g_yaw = {0.0, 0.0};
    g_duration = {0.0, 0.0};
    g_drone_vx = {0.0, 0.0};
    g_drone_vy = {0.0, 0.0};
    g_drone_vz = {0.0, 0.0};
    g_drone_duration = {0.0, 0.0};*/
    trajectory_t reversed_commands;

    double max_speed_so_far = 0;
    int traj_cur_size = traj->size();
    int pre_traj_cur_size = 0;
    ros::Time start_hook_t;
    bool start = true;
    int safety_critical = 0;
    while (time > 0 && traj->size() > 0 && safety_critical < 2) {
        start_hook_t = ros::Time::now();  
        auto segment_start_time = std::chrono::system_clock::now();
        double flight_time;
        double scaled_flight_time;
        multiDOFpoint p;
        drone.recompute = true;
        recompute = true;
        while((drone.recompute || start || recompute) && safety_critical < 2){
            p = traj->front();
            traj_cur_size = traj->size();
            // Calculate the velocities we should be flying at
            double v_x = p.vx;
            double v_y = p.vy;
            double v_z = p.vz;
             
            if (check_position) {
                auto pos = drone.position();
                v_x += 0.2*(p.x-pos.x);
                v_y += 0.2*(p.y-pos.y);
                v_z += 0.5*(p.z-pos.z);
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
            if (speed > max_speed) {
                scale = max_speed / speed;
                
                v_x *= scale;
                v_y *= scale;
                v_z *= scale;
                speed = std::sqrt((v_x*v_x + v_y*v_y + v_z*v_z));
            }

            if (speed > max_speed_so_far)
                max_speed_so_far = speed;

            // Calculate the time for which these flight commands should run
            flight_time = p.duration <= time ? p.duration : time;
            scaled_flight_time = flight_time / scale;

            /*drone.x_list.push_back(p.x);
            drone.y_list.push_back(p.y);
            drone.z_list.push_back(p.z);
            drone.vx_list.push_back(p.vx);
            drone.vy_list.push_back(p.vy);
            drone.vz_list.push_back(p.vz);
            drone.ax_list.push_back(p.ax);
            drone.ay_list.push_back(p.ay);
            drone.az_list.push_back(p.az);
            drone.yaw_list.push_back(p.yaw);
            drone.duration_list.push_back(p.duration);
            drone.drone_vx_list.push_back(v_x);
            drone.drone_vy_list.push_back(v_y);
            drone.drone_vz_list.push_back(v_z);
            drone.drone_duration_list.push_back(flight_time);*/
            // Fly for flight_time seconds
            segment_start_time = std::chrono::system_clock::now();



            recompute = false;
            if(start){
                g_x[1]= p.x;
                g_y[1]= p.y;
                g_z[1]= p.z;
                g_vx[1] = p.vx;
                g_vy[1] = p.vy;
                g_vz[1] = p.vz;
                g_ax[1] = p.ax;
                g_ay[1] = p.ay;
                g_az[1] = p.az;
                g_yaw[1] = p.yaw;
                g_duration[1] = p.duration;
                g_drone_vx[1] = v_x;
                g_drone_vy[1] = v_y;
                g_drone_vz[1] = v_z;
                g_drone_duration[1] = flight_time;
                g_x[0]= g_x[1];
                g_y[0]= g_y[1];
                g_z[0]= g_z[1];
                g_vx[0] = g_vx[1];
                g_vy[0] = g_vy[1];
                g_vz[0] = g_vz[1];
                g_ax[0] = g_ax[1];
                g_ay[0] = g_ay[1];
                g_az[0] = g_az[1];
                g_yaw[0] = g_yaw[1];
                g_duration[0] = g_duration[1];
                g_drone_vx[0] = g_drone_vx[1];
                g_drone_vy[0] = g_drone_vy[1];
                g_drone_vz[0] = g_drone_vz[1];
                g_drone_duration[0] = g_drone_duration[1];
            }
            //if((detect == 3||detect==4 ||detect==5 || detect == 6) && pre_traj_cur_size != traj_cur_size && !start){
            if((detect == 3||detect==4 ||detect==5 || detect == 6) && !start){
                g_x[1]= p.x;
                g_y[1]= p.y;
                g_z[1]= p.z;
                g_vx[1] = p.vx;
                g_vy[1] = p.vy;
                g_vz[1] = p.vz;
                g_ax[1] = p.ax;
                g_ay[1] = p.ay;
                g_az[1] = p.az;
                g_yaw[1] = p.yaw;
                g_duration[1] = p.duration;
                g_drone_vx[1] = v_x;
                g_drone_vy[1] = v_y;
                g_drone_vz[1] = v_z;
                g_drone_duration[1] = flight_time;
                ros::Time detection_start = ros::Time::now();
                autoencoder_detect(drone, detect);
                drone.detection_time += (ros::Time::now() - detection_start).toSec();
                //std::cout << "detection time: " << drone.detection_time << "\n";
                g_x[0]= g_x[1];
                g_y[0]= g_y[1];
                g_z[0]= g_z[1];
                g_vx[0] = g_vx[1];
                g_vy[0] = g_vy[1];
                g_vz[0] = g_vz[1];
                g_ax[0] = g_ax[1];
                g_ay[0] = g_ay[1];
                g_az[0] = g_az[1];
                g_yaw[0] = g_yaw[1];
                g_duration[0] = g_duration[1];
                g_drone_vx[0] = g_drone_vx[1];
                g_drone_vy[0] = g_drone_vy[1];
                g_drone_vz[0] = g_drone_vz[1];
                g_drone_duration[0] = g_drone_duration[1];
                /*if(recompute){
                    scale_threshold = 5;
                }
                else{
                    scale_threshold = 1;
                }*/
            }
            if(!recompute){
                drone.fly_velocity(v_x, v_y, v_z, yaw, scaled_flight_time, noise_select, range_select_drone, start, detect); 
            }
            else{
                safety_critical++;
                drone.num_recompute++;
                if(safety_critical >= 2){
                    std::cout << "stop the drone\n";
                    drone.fly_velocity(0, 0, 0, yaw, scaled_flight_time, noise_select, range_select_drone, start, detect);
                    traj->clear();
                }
            }
            start = false;

            if(drone.recompute){
                traj->pop_front();
            }
            pre_traj_cur_size = traj->size();
        }
        if(safety_critical < 2){
            std::this_thread::sleep_until(segment_start_time + std::chrono::duration<double>(scaled_flight_time));
        

            // Push completed command onto reverse-command stack
            multiDOFpoint rev_point = reverse_point(p);
            rev_point.duration = flight_time;
            reversed_commands.push_front(rev_point);

            // Update trajectory
            traj->front().duration -= flight_time;
            if (traj->front().duration <= 0){
                traj->pop_front();
            }

            time -= flight_time;
        }
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
	auto it=first.begin();                                                                          
    while((it!=first.end() - 1) && it !=(first.end())) {                                             
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
    //cout << "atan2(vy, vx):" << vy << "," << vx <<", yaw: " << 90.0 - atan2(vy, vx)*180.0/3.14 << "\n";
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

