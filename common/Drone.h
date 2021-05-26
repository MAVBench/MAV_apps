#ifndef DRONE_H
#define DRONE_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <stdint.h>
#include <string>
#include <limits>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <geometry_msgs/Pose.h>
#include "coord.h"
#include "common/VectorMath.hpp"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "coord.h"
#include <vector>
#include "autoencoder.h"
#include "fdeep.hpp"

//#include "configs.h"
// Control functions

const float FACE_FORWARD = std::numeric_limits<float>::infinity();
const float FACE_BACKWARD = -std::numeric_limits<float>::infinity();
const float YAW_UNCHANGED = -1e9;

class Drone {
public:
    Drone();
    Drone(const std::string& ip_addr, uint16_t port);
    Drone(const std::string& ip_addr, uint16_t port, std::string localization_method);
    Drone(const std::string& ip_addr, uint16_t port, std::string localization_method, 
         float max_yaw_rate, float max_yaw_rate_during_flight);
    Drone(const std::string& ip_addr, uint16_t port, std::string localization_method, 
         float max_yaw_rate, float max_yaw_rate_during_flight, int num_fault);


    ~Drone();

    // *** F:DN Connection functions
    void connect();
    void connect(const std::string& ip_addr, uint16_t port);
    void set_localization_method(std::string localization_method);

	// *** F:DN Control functions
    void arm();
    void disarm();
    bool takeoff(double h);
    bool set_yaw(int y);
    bool set_yaw_at_z(int y, double z);
    bool fly_velocity(double vx, double vy, double vz, float yaw = YAW_UNCHANGED, double duration = 3, int noise_select = 0, int range_select_drone = 0, bool start = false, int detect = 0);
    bool fly_velocity_at_z(double vx, double vy, double z, float yaw = YAW_UNCHANGED, double duration = 3);
    bool land();
    bool set_yaw_based_on_quaternion(geometry_msgs::Quaternion q);
    void fault_injection_double(double &original);
    void fault_injection_float(float &original);
    void detect_float(std::vector<float> &points, double mean, double stddev);
    void detect_double(std::vector<double> &points, double mean, double stddev);
    void detect_all(bool is_yaw_rate);

    // *** F:DN Localization functions
    coord position(); 
    geometry_msgs::Pose pose();
    geometry_msgs::PoseWithCovariance pose_with_covariance();
    float get_pitch();
    float get_yaw();
    float get_roll();
    coord gps(uint64_t& timestamp);

    // *** F:DN Stats functions
    msr::airlib::FlightStats getFlightStats();
    
    msr::airlib::IMUStats getIMUStats();
    
    // *** F:DN Collison functions
    msr::airlib::CollisionInfo getCollisionInfo();

    // *** F:DN Drone parameters functions
    float maxYawRate();
    float maxYawRateDuringFlight();
    // number of bit flip faults
    bool injected = false;
    int num_fault = 0;
    int range_select_drone;
    int count = 0;
    double detect_percentage = 0.0;
    // Error threshold for construction error of autoencoder-based anomaly detection
    double threshold = 10.0;
    // Magnitude difference after fault injection
    /*double difference_float;
    double original_float;
    double after_float;
    float difference_float32;
    float original_float32;
    float after_float32;
    // Whether inject to float or int
    bool is_float;*/

    // for saving results
    std::list<double> drone_vx_list;
    std::list<double> drone_vy_list;
    std::list<double> drone_vz_list;
    std::list<float> drone_yaw_list;
    std::list<double> drone_duration_list;

    std::list<double> x_list;
    std::list<double> y_list;
    std::list<double> z_list;
    std::list<double> vx_list;
    std::list<double> vy_list;
    std::list<double> vz_list;
    std::list<double> ax_list;
    std::list<double> ay_list;
    std::list<double> az_list;
    std::list<double> yaw_list;
    std::list<double> duration_list;
    std::list<int> trajectory_seq_list;
    std::list<int> future_collision_seq_list;

    std::list<float> recompute_time;
    std::list<float> error_list;

    std::vector<double> g_vx = {0.0, 0.0};
    std::vector<double> g_vy = {0.0, 0.0};
    std::vector<double> g_vz = {0.0, 0.0};
    std::vector<double> g_duration = {0.0, 0.0};
    std::vector<float> g_yaw_rate = {0.0, 0.0};
    bool recompute = false;
    std::vector<int> record_detect;
    // make sure only inject once
    bool one_time_injection = false;
    double num_sigma = 3.0;
    int injected_detected = 0;
    int injected_not_detected = 0;
    int not_injected_detected = 0;
    int not_injected_not_detected = 0;
    int num_recompute = 0;
    ros::Time start_ros;
    ros::Time end_ros;
    Autoencoder encoder;
    double detection_time = 0;



private:
    msr::airlib::MultirotorRpcLibClient * client;

    std::string localization_method; 
    tf::TransformListener tfListen;

    uint64_t collision_count;

    float max_yaw_rate = 90.0;
    //float max_yaw_rate_during_flight = 90.0;
    float max_yaw_rate_during_flight = 90.0;
    // Initial position as determined by the flight-controller 
    coord initial_fc_pos;
};

#endif

