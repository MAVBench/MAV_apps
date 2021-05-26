#include "Drone.h"
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <cstring>
#include "common/Common.hpp"
#include "coord.h"
#include <geometry_msgs/Point.h>
#include "timer.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include "common/VectorMath.hpp"
Drone::Drone() : client(0)
{
	connect();

	auto pos = client->getPosition();
    initial_fc_pos = {pos.y(), pos.x(), -pos.z()};

    this->localization_method = "ground_truth";
}

Drone::Drone(const std::string& ip_addr, uint16_t port) : client(0), collision_count(0), 
    localization_method("ground_truth")
{
	connect(ip_addr, port);

	auto pos = client->getPosition();
    initial_fc_pos = {pos.y(), pos.x(), -pos.z()};
}

Drone::Drone(const std::string& ip_addr, uint16_t port, std::string localization_method) : client(0), collision_count(0)
{
	connect(ip_addr, port);

	auto pos = client->getPosition();
    initial_fc_pos = {pos.y(), pos.x(), -pos.z()};

    this->localization_method = localization_method;
}

Drone::Drone(const std::string& ip_addr, uint16_t port, 
             std::string localization_method, float max_yaw_rate, 
             float max_yaw_rate_during_flight) : client(0), collision_count(0)
{
	connect(ip_addr, port);

	auto pos = client->getPosition();
    initial_fc_pos = {pos.y(), pos.x(), -pos.z()};
    this->localization_method = localization_method;
    this->max_yaw_rate_during_flight = max_yaw_rate_during_flight;
    this->max_yaw_rate = max_yaw_rate;
}

Drone::Drone(const std::string& ip_addr, uint16_t port, 
             std::string localization_method, float max_yaw_rate, 
             float max_yaw_rate_during_flight, int num_fault_drone) : client(0), collision_count(0)
{
    connect(ip_addr, port);

    auto pos = client->getPosition();
    initial_fc_pos = {pos.y(), pos.x(), -pos.z()};
    this->localization_method = localization_method;
    this->max_yaw_rate_during_flight = max_yaw_rate_during_flight;
    this->max_yaw_rate = max_yaw_rate;
    num_fault = num_fault_drone;
}

Drone::~Drone()
{
	if (client != 0)
		delete client;
}

void Drone::connect()
{
	if (client != 0)
		delete client;
	client = new msr::airlib::MultirotorRpcLibClient();
    client->enableApiControl(true);
}

void Drone::set_localization_method(std::string localization_method) {
    this->localization_method = localization_method;
}
void Drone::connect(const std::string& ip_addr, uint16_t port)
{
	if (client != 0)
		delete client;
	client = new msr::airlib::MultirotorRpcLibClient(ip_addr, port);
    client->enableApiControl(true);
}

void Drone::arm()
{
	client->armDisarm(true);
}

void Drone::disarm()
{
	client->armDisarm(false);
}

bool Drone::takeoff(double h)
{
    auto pos = position();
    const double small_margin = 0.2;
    const double large_margin = 0.5;
    int tries_left = 5;

    while (pos.z < h-large_margin || pos.z > h+large_margin) {
        float p = 0.75, max_speed = 1;

        for (; pos.z < h-small_margin || pos.z > h+small_margin; pos = position()) {
            float dist = h - pos.z;

            float speed = dist*p;
            if (speed > max_speed)
                speed = max_speed;

            fly_velocity(0, 0, speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            p += 0.05;
            max_speed += 0.1;
        }

        fly_velocity(0,0,0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        pos = position();

        if (tries_left == 0)
            return false;
        tries_left--;
    }

    return true;
}

/*
static double cap_vz (double vz)
{
    if (vz < -10)
        vz = -10;
    else if (vz > 10)
        vz = 10;
    // else if (vz < -5)
    //     vz = -5;
    // else if (vz > 5)
    //     vz = 5;
    // else if (vz < -2)
    //     vz = -2;
    // else if (vz > 2)
    //     vz = 2;
    // else if (vz < -1)
    //     vz = -1;
    // else if (vz > 1)
    //     vz = 1;
    // else if (vz < -0.1)
    //     vz = -0.1;
    // else if (vz > 0.1)
    //     vz = 0.1;

    return vz;
}
*/

bool Drone::set_yaw_at_z (int y, double z)
{
    int pos_dist = (y - int(get_yaw()) + 360) % 360;
    int yaw_diff = pos_dist <= 180 ? pos_dist : pos_dist - 360;

    float duration = yaw_diff / max_yaw_rate;
    if (duration < 0)
        duration = -duration;

    float yaw_rate = max_yaw_rate;
    if (yaw_diff < 0)
        yaw_rate = -yaw_rate;

	try {
        auto drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        msr::airlib::YawMode yawmode(true, yaw_rate);

        auto t = std::chrono::system_clock::now();
        auto end_t = t + std::chrono::milliseconds(int(duration*1000));
        const double t_step = 0.05; // 50 ms
        const auto t_step_ms = std::chrono::milliseconds(int(t_step*1000));

        for (; t < end_t; t += t_step_ms) {
            double current_z = pose().position.z;
            double v_z = (z - current_z) * 0.5;

            // if (std::abs(start_z - current_z) < 0.25)
            //     v_z = 0;
            // else
            //     v_z = cap_vz(v_z);

            // ROS_INFO("%f\t%f\t%f", start_z, current_z, v_z);

            client->moveByVelocity(0, 0, -v_z, duration, drivetrain, yawmode);

            std::this_thread::sleep_until(t);
        }
        
        client->moveByVelocity(0, 0, 0, 1);
	} catch(...) {
		std::cerr << "set_yaw failed" << std::endl;
		return false;
	}
}


bool Drone::set_yaw(int y)
{
    int pos_dist = (y - int(get_yaw()) + 360) % 360;
    int yaw_diff = pos_dist <= 180 ? pos_dist : pos_dist - 360;

    float duration = yaw_diff / max_yaw_rate;
    if (duration < 0)
        duration = -duration;

    float yaw_rate = max_yaw_rate;
    if (yaw_diff < 0)
        yaw_rate = -yaw_rate;

	try {
        auto drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
        auto yawmode = msr::airlib::YawMode(true, yaw_rate);

        client->moveByVelocity(0, 0, 0, duration, drivetrain, yawmode);

        int duration_ms = duration*1000;
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
	}catch(...){
		std::cerr << "set_yaw failed" << std::endl;
		return false;
	}
}

/*
bool Drone::set_yaw(float y, bool slow)
{
    float angular_vel = 15;

	try {
        if (slow) {
            float init_yaw = get_yaw();
            int direction;
            if (y >= init_yaw)
                direction = 1;
            else
                direction = -1;

            angular_vel *= direction;

            for (float yaw = get_yaw(); (direction == 1 && yaw <= y) || (direction == -1 && yaw >= y);) {
                // ROS_ERROR("yaw: %f, y: %f", yaw, y);

                client->rotateToYaw(yaw, 60, 5);

                yaw += angular_vel;

                if (direction == 1 && yaw > y)
                    yaw = y+1;
                else if (direction == -1 && yaw < y)
                    yaw = y-1;

                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        } else {
            client->rotateToYaw(y, 60, 5);
        }
	}catch(...){
		std::cerr << "set_yaw failed" << std::endl;
		return false;
	}

	return true;
}
*/

bool Drone::set_yaw_based_on_quaternion(geometry_msgs::Quaternion q)
{
	float pitch, roll, yaw;
    Eigen::Quaternion<float,Eigen::DontAlign> q_airsim_style;
    q_airsim_style.x() = q.x;
    q_airsim_style.y() = q.y;
    q_airsim_style.z() = q.z;
    q_airsim_style.w() = q.w;
    msr::airlib::VectorMath::toEulerianAngle(q_airsim_style, pitch, roll, yaw);
	
    try {
        this->set_yaw(yaw*180 / M_PI);
    } catch(...) {
		std::cerr << "set_yaw_based_on_quaternion failed" << std::endl;
		return false;
	}
	return true;
}

static float xy_yaw(double x, double y) {
    if (x == 0 && y == 0)
        return YAW_UNCHANGED;
    return 90 - atan2(y, x)*180.0/3.14;
}

void Drone::fault_injection_double(double &original){
    int noise; 
    int pick;
    int select;
    //original_float = original;
    bool success = false;
    std::vector<int> record;
    //is_float = true;

   // write inputted data into the file.
    //std::cout << "print float64: " << original << std::endl;
    for(int i = 0; i < num_fault; i++){
        while(!success){
            if(range_select_drone == 1){
                select = 63;
            }
            else if(range_select_drone == 2){
                select = rand() % 11 + 52;
            }
            else if(range_select_drone == 3){
                select = rand() % 52;
            }
            noise = select / 8;
            pick = select % 8;
            success = true;
            if(!record.empty()){
                for (int j = 0; j < record.size(); ++j){
                    if(record[j] == (select)){
                        success = false;
                    }              
                }
            }
        }
        char* bits = reinterpret_cast<char*>(&original);
        for(std::size_t n = 0; n < sizeof original; ++n){
            std::bitset<8> byte (bits[n]);
            if(n == noise){
                byte.flip(pick);
                bits[n] = static_cast<unsigned char>(byte.to_ulong());
                record.push_back(select);
            }
        }
        
        success = false;
    }
    //after_float = original;
    record.clear();
    //difference_float = after_float - original_float;
    //std::cout << "after injection print float64: " << original << std::endl;
    //std::cout << "difference: " << difference_float << std::endl;

}

void Drone::fault_injection_float(float &original){
    int noise; 
    int pick;
    int select;
    //original_float32 = original;
    bool success = false;
    std::vector<float> record;
    //is_float = false;

   // write inputted data into the file.
    //std::cout << "print float32: " << original << std::endl;
    for(int i = 0; i < num_fault; i++){
        while(!success){
            if(range_select_drone == 1){
                select = 31;
            }
            else if(range_select_drone == 2){
                select = rand() % 8 + 23;
            }
            else if(range_select_drone == 3){
                select = rand() % 23;
            }
            noise = select / 8;
            pick = select % 8;
            success = true;
            if(!record.empty()){
                for (int j = 0; j < record.size(); ++j){
                    if(record[j] == (select)){
                        success = false;
                    }              
                }
            }
        }
        char* bits = reinterpret_cast<char*>(&original);
        for(std::size_t n = 0; n < sizeof original; ++n){
            std::bitset<8> byte (bits[n]);
            if(n == noise){
                byte.flip(pick);
                bits[n] = static_cast<unsigned char>(byte.to_ulong());
                record.push_back(select);
            }
        }
        
        success = false;
    }
    //after_float32 = original;
    record.clear();
    //difference_float32 = after_float32 - original_float32;
    //std::cout << "after injection print float32: " << original << std::endl;
    //std::cout << "difference: " << difference_float32 << std::endl;

}

void Drone::detect_double(std::vector<double> &points, double mean, double stddev){
    double upper_b = mean + num_sigma * stddev;
    double lower_b = mean - num_sigma * stddev;
    double delta;

    /*for(int i = 0; i < points.size(); i++){
        std::cout << i << ": " << points[i] << " ";
    }*/
    delta = points[1] - points[0];
    if(delta != 0){
        if(delta > upper_b || delta < lower_b){
            std::cout << " upper_b: " << upper_b << " lower_b: " << lower_b << "\n";
            std::cout << "drone delta: " << delta << "\n";
            recompute = true;
        } 
    }


}
void Drone::detect_float(std::vector<float> &points, double mean, double stddev){
    float upper_b = mean + num_sigma * stddev;
    float lower_b = mean - num_sigma * stddev;
    float delta;

    if(!recompute){
        delta = points[1] - points[0];
        if(delta != 0){
            if(delta > upper_b || delta < lower_b){
                recompute = true;
            } 
            for(int i = 0; i < points.size(); i++){
                if (points[i] == FACE_FORWARD || points[i] == FACE_BACKWARD || points[i] == YAW_UNCHANGED){
                    recompute = false;
                }
                if (points[i] == 90 || points[i] == -90 || points[i] == 180 || points[i] == -180){
                    recompute = false;
                }
            }
        }
    }
    

}
void Drone::detect_all(bool is_yaw_rate){
    // Starting from index = 1
    std::vector<double> mean = {-4.08598e-05, -6.49413e-07, -0.000127048};
    //std::vector<double> stddev = {1.7164e-07, 0.00198019, 0.0131262};
    std::vector<double> stddev = {0.0486682, 0.00198019, 0.0131262};
    //std::vector<double> mean = {7.49104e-06, -6.49413e-07, -0.000127048};
    //std::vector<double> stddev = {0.0886682, 0.00198019, 0.0131262};
    std::vector<double> tmp;
    int num_detect = 4.0 * detect_percentage;
    int detect_run = 0;
    int select;
    bool success = false;
    if(record_detect.empty()){
        if(detect_percentage == 1){
            for(int i = 0; i< 4; i++){
                record_detect.push_back(i);
            }
        }
        else{
            while(detect_run < num_detect){
                success = false;
                while(!success){
                    select = rand() % 4;
                    success = true;
                    for (int j = 0; j < record_detect.size(); ++j){
                        if(record_detect[j] == (select)){
                            success = false;
                        }              
                    }
                }
                record_detect.push_back(select);
                detect_run++;
            }
        }
        std::cout << "control: ";
        for(int i = 0; i < record_detect.size(); i++){
            std::cout << record_detect[i] << ", ";
        }
        std::cout << "\n";
    }

    for(int i = 0; i < record_detect.size(); i++){
        select = record_detect[i];
        if(select == 0){
            detect_double(g_vx, mean[0], stddev[0]);
        }
        else if(select == 1){
            detect_double(g_vy, mean[0], stddev[0]);
        }
        else if(select == 2){
            if(count > 10){
                detect_double(g_vz, mean[0], stddev[0]);
            }
        }
        else if(select == 3){
            if(is_yaw_rate){
                detect_float(g_yaw_rate, mean[2], stddev[2]);
            }
        }
    }
    /*detect_double(g_duration, mean[1], stddev[1]);
    if(recompute){
        std::cout << "duration has error" << "\n";
    }*/


    if(recompute){
        end_ros = ros::Time::now();
        recompute_time.push_back((end_ros - start_ros).toSec());
        std::cout << "recompute: " << recompute << "\n";
        num_recompute++;
        std::cout << "\n";
    }
    if(injected && one_time_injection == false){
        one_time_injection = true;
        std::cout << "fault injected!!!" << "\n";
        std::cout << "recompute: " << recompute << "\n";
        if(recompute){
            injected_detected++;
        }
        else{
            injected_not_detected++;
        }
    }
    else{
        if(recompute){
            not_injected_detected++;
        }
        else{
            not_injected_not_detected++;
        }
    }


}
bool Drone::fly_velocity(double vx, double vy, double vz, float yaw, double duration, int noise_select, int range_select, bool start, int detect)
{
    //getCollisionInfo();

	try {
        /*if(start){
            g_vx[0] = 0.0;
            g_vy[0] = 0.0;
            g_vz[0] = 0.0;
            g_duration[0] = 0.0;
            g_yaw_rate[0] = 0.0;
        }*/
        range_select_drone = range_select;
        if (yaw != YAW_UNCHANGED) {
            float target_yaw = yaw;
            if (yaw == FACE_FORWARD)
                target_yaw = xy_yaw(vx, vy);
            else if (yaw == FACE_BACKWARD) {
                target_yaw = xy_yaw(vx, vy);
                target_yaw += 180;
                target_yaw = target_yaw <= 180 ? target_yaw : target_yaw-360;
            }

            float yaw_diff = (int(target_yaw - get_yaw()) + 360) % 360;
            yaw_diff = yaw_diff <= 180 ? yaw_diff : yaw_diff - 360;
            
            if (yaw_diff >= 5)
                yaw_diff -= 5;
            else if (yaw_diff <= -5)
                yaw_diff += 5;
            else
                yaw_diff = 0;

            float yaw_rate = yaw_diff / duration;

            if (yaw_rate > max_yaw_rate_during_flight)
                yaw_rate = max_yaw_rate_during_flight;
            else if (yaw_rate < -max_yaw_rate_during_flight)
                yaw_rate = -max_yaw_rate_during_flight;

            auto drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
            auto yawmode = msr::airlib::YawMode(true, yaw_rate);
            drone_yaw_list.push_back(yaw_rate);
            //std::cout << "yaw_rate: " << yaw_rate << "\n";
            int inject = rand() % 100;
            if(inject == 0 && noise_select != 0){
                // noise injection
                if(!injected){
                    injected = true;
                    if(noise_select == 1){
                        fault_injection_double(vx);
                        fault_injection_double(vy);
                        fault_injection_double(vz);
                    }
                    else if(noise_select == 2){
                        fault_injection_double(duration);
                    }
                    else if(noise_select == 3){
                        fault_injection_float(yaw_rate);
                    }
                    else{
                        injected = false;
                    }
                }
            }
            g_vx[1] = vx;
            g_vy[1] = vy;
            g_vz[1] = vz;
            g_duration[1] = duration;
            g_yaw_rate[1] = yaw_rate;
            // Initialize the recompute as don't need
            recompute = false;
            if(detect == 1){
                detect_all(true);
            }

            g_vx[0] = g_vx[1];
            g_vy[0] = g_vy[1];
            g_vz[0] = g_vz[1];
            g_duration[0] = g_duration[1];
            g_yaw_rate[0] = g_yaw_rate[1];
            if(!recompute){
                client->moveByVelocity(vy, vx, -vz, duration, drivetrain, yawmode);
            }
        } else {
            int inject = rand() % 1000;
            if(inject == 0){
                // noise injection
                if(!injected){
                    injected = true;
                    if(noise_select == 1){
                        fault_injection_double(vx);
                        fault_injection_double(vy);
                        fault_injection_double(vz);
                    }
                    else if(noise_select == 2){
                        fault_injection_double(duration);
                    }
                    else{
                        injected = false;
                    }
                }
            }
            drone_yaw_list.push_back(g_yaw_rate[0]);
            g_vx[1] = vx;
            g_vy[1] = vy;
            g_vz[1] = vz;
            g_duration[1] = duration;
            g_yaw_rate[1] = g_yaw_rate[0];

            // Initialize the recompute as don't need
            recompute = false;
            if(detect == 1){
                detect_all(false);
            }

            g_vx[0] = g_vx[1];
            g_vy[0] = g_vy[1];
            g_vz[0] = g_vz[1];
            g_duration[0] = g_duration[1];
            g_yaw_rate[0] = g_yaw_rate[1];
            if(!recompute){
                client->moveByVelocity(vy, vx, -vz, duration);
            }
        }
    } catch(...) {
		std::cerr << "fly_velocity failed" << std::endl;
		return false;
	}

	return true;
}

bool Drone::fly_velocity_at_z(double vx, double vy, double z, float yaw, double duration)
{
    auto t = std::chrono::system_clock::now();
    auto end_t = t + std::chrono::milliseconds(int(duration*1000));
    const double t_step = 0.05; // 50 ms
    const auto t_step_ms = std::chrono::milliseconds(int(t_step*1000));

    for (; t < end_t; t += t_step_ms) {
        double current_z = pose().position.z;
        double vz = (z - current_z) * 0.5;

        if (!fly_velocity(vx, vy, vz, yaw, duration))
            return false;

        std::this_thread::sleep_until(t);
    }

    fly_velocity(0, 0, 0);

    return true;
}

bool Drone::land()
{
    auto pos = position();

    for (; pos.z > 0.05; pos = position()) {
        const float speed = -0.5;
        fly_velocity(0, 0, speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    fly_velocity(0,0,0);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    return true;
}

coord Drone::gps(uint64_t& timestamp)
{
    // https://stackoverflow.com/questions/3024404/transform-longitude-latitude-into-meters
    coord result;

	// getCollisionInfo();
	auto currentGPS = client->getGPSStats();
	auto homeGPS = client->getHomeGeoPoint();

    double deltaLat = currentGPS.latitude - homeGPS.latitude;
    double deltaLong = currentGPS.longitude - homeGPS.longitude;

    double latCircumference = 40075160 * std::cos(homeGPS.latitude * M_PI / 180);

    result.x = deltaLong * latCircumference / 360;
    result.y = deltaLat * 40008000 / 360;
    result.z = currentGPS.altitude - homeGPS.altitude;

    timestamp = currentGPS.time_stamp;

    return result;
}

geometry_msgs::Pose Drone::pose()
{
    geometry_msgs::Pose result;

    /*
    if (this->localization_method == "gps") {
        auto p = client->getPosition();
	    auto q = client->getOrientation();
        result.position.x = client->getPosition().y() - initial_gps.x;
        result.position.y = client->getPosition().x() - initial_gps.y;
        result.position.z = -1*client->getPosition().z() - initial_gps.z;
        result.orientation.x = q.x();
        result.orientation.y = q.y();
        result.orientation.z = q.z();
        result.orientation.w = q.w();
    }else{
    */
    //std::cout<<"localization method is"<<this->localization_method<<std::endl;
    
    tf::StampedTransform transform;
        try{
            tfListen.lookupTransform("/world", "/"+(this->localization_method),
                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());

            auto p = client->getPosition();
            auto q = client->getOrientation();
            result.position.x = client->getPosition().y() - initial_fc_pos.x;
            result.position.y = client->getPosition().x() - initial_fc_pos.y;
            result.position.z = -1*client->getPosition().z() - initial_fc_pos.z;
            result.orientation.x = q.x();
            result.orientation.y = q.y();
            result.orientation.z = q.z();
            result.orientation.w = q.w();

            return result;
        }

        tf::Vector3 tf_translation = transform.getOrigin();
        result.position.x = tf_translation.x();
        result.position.y = tf_translation.y();
        result.position.z = tf_translation.z();

        tf::Quaternion tf_rotation = transform.getRotation();
        result.orientation.x = tf_rotation.x();
        result.orientation.y = tf_rotation.y();
        result.orientation.z = tf_rotation.z();
        result.orientation.w = tf_rotation.w();
    //}
    return result;
}

coord Drone::position() {
    coord result;
    geometry_msgs::Pose result_pose = pose();
    result.x = result_pose.position.x;
    result.y = result_pose.position.y;
    result.z = result_pose.position.z;
    return result;
}


geometry_msgs::PoseWithCovariance Drone::pose_with_covariance()
{
    geometry_msgs::PoseWithCovariance result;

    result.pose = pose();

    for (int i = 0; i <36; i++) { 
        //https://answers.ros.org/question/181689/computing-posewithcovariances-6x6-matrix/ 
        result.covariance[i] = 0;
    }

    return result;
}

float Drone::get_yaw()
{
	//auto q = client->getOrientation();
	auto pose = this->pose();
    msr::airlib::Quaternionr q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return -y*180 / M_PI;
}

float Drone::get_roll()
{
	//auto q = client->getOrientation();
	auto pose = this->pose();
   msr::airlib::Quaternionr q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    
    float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return r*180 / M_PI;
}

/*
geometry_msgs::Pose Drone::get_geometry_pose(){
    geometry_msgs::Pose pose;
	auto q = client->getOrientation();
	//auto p = client->getPosition();
    pose.position.x = client->getPosition().y();
    pose.position.y = client->getPosition().x();
    pose.position.z = -1*client->getPosition().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}
*/
/*
geometry_msgs::PoseWithCovariance Drone::get_geometry_pose_with_coveraiance(){
    geometry_msgs::PoseWithCovariance pose_with_covariance;
    geometry_msgs::Pose pose;
	
    auto q = client->getOrientation();
    pose.position.x = client->getPosition().y();
    pose.position.y = client->getPosition().x();
    pose.position.z = -1*client->getPosition().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_with_covariance.pose = pose;
    for (int i = 0; i <36; i++) { 
        //https://answers.ros.org/question/181689/computing-posewithcovariances-6x6-matrix/ 
        pose_with_covariance.covariance[i] = 0;
    } 
    return pose_with_covariance;
}
*/

float Drone::get_pitch()
{
	
    auto pose = this->pose();
    msr::airlib::Quaternionr q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
     //auto q = client->getOrientation();
	
    
    float p, r, y;
    msr::airlib::VectorMath::toEulerianAngle(q, p, y, r);

	return p*180 / M_PI;
}

/*
msr::airlib::CollisionInfo Drone::getCollisionInfo()
{
  auto col_info = client->getCollisionInfo();
  if (col_info.has_collided) {
     collision_count ++;
  }
  if (collision_count > 10) {
    land();
    sleep(5);
    disarm();
    fprintf(stderr, "Drone Crashed!\n");
    LOG_TIME(crashed);
    ros::shutdown();
    exit(0);
  }

  if (col_info.has_collided) {
  printf("CollisionInfo %s, count %ld, collison %d, penetrate %f\n",
          col_info.has_collided ? "has collided" : "none", collision_count, col_info.collison_count,
          col_info.penetration_depth);
  }

  return col_info;
}
*/

msr::airlib::FlightStats Drone::getFlightStats()
{
    try {
        return client->getFlightStats();
    } catch (...) {
        std::cerr << "getFlightStats() failed!" << std::endl;
        return msr::airlib::FlightStats();
    }
}


msr::airlib::IMUStats Drone::getIMUStats()
{
    try {
        return client->getIMUStats();
    } catch (const std::exception& e) {
        std::cerr << "getIMUStats() failed!" << e.what()<<std::endl;
        return msr::airlib::IMUStats();
    }
}


float Drone::maxYawRate()
{
    return max_yaw_rate;
}

float Drone::maxYawRateDuringFlight()
{
    return max_yaw_rate_during_flight;
}

