#include "ros/ros.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cmath>

#include <Eigen/Dense>

#include "common.h"

using namespace std;

// Functions to calculate SLAM error
// Explanation of formula: "A Benchmark for the Evaluation of RGB-D SLAM Systems"
void transformToSE3Matrix(const tf::StampedTransform& transform, Eigen::Matrix4d& mat);
double relativePoseError(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q);
Eigen::Matrix4d Ei(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q, int i, int delta); 
double rmse_E_delta(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q, int delta);
double absoluteTrajectoryError(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q);

// *** F:DN main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "stats_manager");
    ros::NodeHandle n;

    // Variables for SLAM accuracy measurements
    tf::TransformListener tfListen;
    std::vector<Eigen::Matrix4d> P;
    std::vector<Eigen::Matrix4d> Q;

    // Parameters
    uint16_t port = 41451;
    string ip_addr;
    string stats_fname;
    string localization_method;

    ros::param::get("/ip_addr", ip_addr);
    ros::param::get("/stats_file_addr", stats_fname);
    ros::param::get("/airsim_imgPublisher/localization_method", localization_method);

    Drone drone(ip_addr.c_str(), port);

	ros::Rate loop_rate(10);
    while (ros::ok()) {
        try {
            tf::StampedTransform ground_truth_tf, estimated_tf;

            ros::Time now = ros::Time::now();
            tfListen.waitForTransform("/world", "/ground_truth", now, ros::Duration(1.0));
            tfListen.waitForTransform("/world", "/"+localization_method, now, ros::Duration(1.0));

            tfListen.lookupTransform("/world", "/ground_truth", now, ground_truth_tf);
            tfListen.lookupTransform("/world", "/"+localization_method, now, estimated_tf);
            
            Eigen::Matrix4d Pi(4,4), Qi(4,4);

            transformToSE3Matrix(estimated_tf, Pi);
            transformToSE3Matrix(ground_truth_tf, Qi);

            P.push_back(Pi);
            Q.push_back(Qi);
        } catch(tf::TransformException& ex) {
            // ROS_ERROR("Stats manager failed to read transform");
        }

        loop_rate.sleep();
    }

    update_stats_file(stats_fname, "\nFlightSummaryEnd: ");
    output_flight_summary(drone, stats_fname);

    stringstream slam_error_ss;
    slam_error_ss << "SLAM_relative_pose_error: " << relativePoseError(P, Q) << endl;
    slam_error_ss << "SLAM_absolute_trajectory_error: " << absoluteTrajectoryError(P, Q) << endl;
    update_stats_file(stats_fname, slam_error_ss.str());

    return 0;
}


void transformToSE3Matrix(const tf::StampedTransform& transform, Eigen::Matrix4d& mat)
{
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 v = transform.getOrigin();

    mat << 1-2*q.y()*q.y()-2*q.z()*q.z(),
        2*q.x()*q.y()-2*q.z()*q.w(),
        2*q.x()*q.z()+2*q.y()*q.w(),
        v.x(),

        2*q.x()*q.y()+2*q.z()*q.w(),
        1-2*q.x()*q.x()-2*q.z()*q.z(),
        2*q.y()*q.z()-2*q.x()*q.w(),
        v.y(),
        
        2*q.x()*q.z()-2*q.y()*q.w(),
        2*q.y()*q.z()+2*q.x()*q.w(),
        1-2*q.x()*q.x()-2*q.y()*q.y(),
        v.z(),
        
        0, 0, 0, 1;
}


double relativePoseError(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q)
{
    if (P.size() != Q.size()) {
        ROS_ERROR("P and Q are different sizes");
        return numeric_limits<double>::quiet_NaN();
    }

    double sum = 0;
    for (int delta = 1; delta < P.size(); delta++)
        sum += rmse_E_delta(P, Q, delta);

    return std::sqrt(sum / P.size());
}


Eigen::Matrix4d Ei(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q, int i, int delta)
{
    return (Q[i].inverse()*Q[i+delta]).inverse()*(P[i].inverse()*P[i+delta]);
}


double rmse_E_delta(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q, int delta)
{
    double sum = 0;
    int m = P.size() - delta;

    for (int i = 0; i < m; i++) {
        Eigen::Matrix4d Ei_ = Ei(P, Q, i, delta);

        Eigen::Vector3d trans_Ei;
        trans_Ei << Ei_(0,3), Ei_(1,3), Ei_(2,3);

        double trans_Ei_magnitude = trans_Ei.norm();
        sum += trans_Ei_magnitude*trans_Ei_magnitude;
    }

    return std::sqrt(sum / m);
}


double absoluteTrajectoryError(const std::vector<Eigen::Matrix4d>& P, const std::vector<Eigen::Matrix4d>& Q)
{
    if (P.size() != Q.size()) {
        ROS_ERROR("P and Q are different sizes");
        return numeric_limits<double>::quiet_NaN();
    }

    double sum = 0;
    for (int i = 1; i < P.size(); i++) {
        Eigen::Matrix4d Fi = Q[i].inverse() * P[i];

        Eigen::Vector3d trans_Fi;
        trans_Fi << Fi(0,3), Fi(1,3), Fi(2,3);

        double trans_Fi_magnitude = trans_Fi.norm();
        sum += trans_Fi_magnitude*trans_Fi_magnitude;
    }

    return std::sqrt(sum / P.size());
}

