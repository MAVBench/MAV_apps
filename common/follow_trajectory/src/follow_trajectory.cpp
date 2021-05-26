#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "common.h"
#include "Drone.h"
#include <std_msgs/Bool.h>
#include "std_msgs/Int32.h"
#include <signal.h>
#include <sys/prctl.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
#include <mavbench_msgs/future_collision.h>
#include <mavbench_msgs/rosfi.h>

#include <fstream>
#include <typeinfo>
#include <vector>
#include <list>

using namespace std;

// Trajectories
trajectory_t trajectory;
trajectory_t reverse_trajectory;
bool fly_backward = false;

// Messages from other nodes
bool slam_lost = false;
ros::Time future_collision_time{0}; // The moment in time when the drone should stop because of an upcoming collision
int future_collision_seq = 0;
int trajectory_seq = 0;

// Parameters
float g_v_max;
double g_grace_period = 0; // How much time the drone will wait for a new path to be calculated when a collision is near, before pumping the breaks
double g_time_to_come_to_full_stop = 0;
double g_fly_trajectory_time_out;
long long g_planning_time_including_ros_overhead_acc  = 0;
float g_max_yaw_rate;
float g_max_yaw_rate_during_flight;
bool g_trajectory_done = false;
bool g_got_new_trajectory = false;

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
ros::Time last_to_fly_backward;
double g_max_velocity_reached = 0;
int noise_select;
int num_fault;
int range_select_drone;
double num_sigma;
int detect;
double detect_percentage;
double error_threshold;
int num_recompute;
// Magnitude difference after fault injection
/*double difference_float;
double original_float;
double after_float;
float difference_float32;
float original_float32;
float after_float32;
bool is_float;
*/
// for saving results
std::list<double> drone_vx_list;
std::list<double> drone_vy_list;
std::list<double> drone_vz_list;
std::list<float> drone_yaw_list;
std::list<double> drone_duration_list;
std::list<double> recompute_time;

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
std::list<float> error_list;


int injected_detected = 0;
int injected_not_detected = 0;
int not_injected_detected = 0;
int not_injected_not_detected = 0;

double detection_time;
int FI_PID;
bool ready = false;

void FIPIDCallback(const std_msgs::Int32::ConstPtr& msg)
{
    FI_PID = msg->data;
  
  ready = true;
  ROS_INFO("FI_PID: [%d]", msg->data);
}

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
    // write the variable vaules into .txt files
    ofstream outfile;
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/drone_vx.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:drone_vx_list){
        outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/drone_vy.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:drone_vy_list){
        outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/drone_vz.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:drone_vz_list){
        outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/drone_yaw.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:drone_yaw_list){
        outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/drone_duration.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:drone_duration_list){
        outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/recompute.txt", std::ios_base::app);
    outfile << "control stage: "<< num_recompute << "\n";
    outfile.close();
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/recompute_time.txt", std::ios_base::app);
    outfile << 3 << "\n";
    for (auto v:recompute_time){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_x.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:x_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_y.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:y_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_z.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:z_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_vx.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:vx_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_vy.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:vy_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_vz.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:vz_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_ax.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:ax_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_ay.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:ay_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_az.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:az_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_yaw.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:yaw_list){
         outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/multi_duration.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:duration_list){
         outfile << v << "\n";}
    outfile.close();
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/error_list.txt", std::ios_base::app);
    for (auto v:error_list){
         outfile << v << "\n";}
    outfile.close();
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/detection_time.txt", std::ios_base::app);
    outfile << "control stage: "<< detection_time << "\n";
    outfile.close();
    // write the location date into a .txt file
    /*std::ofstream outfile;
    if(is_float){
        outfile.open("/home/nvidia/MAVBench_base/src/MAV_apps/package_delivery/src/drone_float64_var_" + std::to_string(noise_select)
            + "_range_" + std::to_string(range_select_drone) + ".txt", std::ios_base::app);
        outfile << original_float << ", " << after_float << ", " <<  difference_float << "\n";
    }
    else {
        outfile.open("/home/nvidia/MAVBench_base/src/MAV_apps/package_delivery/src/drone_float32_var_" + std::to_string(noise_select)
            + "_range_" + std::to_string(range_select_drone) + ".txt", std::ios_base::app);
        outfile << original_float32 << ", " << after_float32 << ", " <<  difference_float32 << "\n";
    }
    outfile.close();*/
    //write the variable vaules into .txt files
    /*std::ofstream outfile;
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/detect_control_summary.txt", std::ios_base::app);
    outfile << injected_detected << ", " << injected_not_detected << ", " << not_injected_detected << ", " << not_injected_not_detected << "\n";
    outfile.close();

    std::cout << "control stage:" << "\n";
    std::cout << injected_detected << ", " << injected_not_detected << ", " << not_injected_detected << ", " << not_injected_not_detected << "\n";
    */
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profiling_data_srv_inst.request.key = "localization_status";
    profiling_data_srv_inst.request.value = g_localization_status;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "img_to_follow_traj_commun_t";
    profiling_data_srv_inst.request.value = (((double)g_pt_cld_to_futurCol_commun_acc)/1e9)/g_traj_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "traj_rcv_to_follow";
    profiling_data_srv_inst.request.value = (((double)g_rcv_traj_to_follow_traj_acc_t)/1e9)/g_follow_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "image_to_follow_time";
    profiling_data_srv_inst.request.value = (((double)g_img_to_follow_acc)/1e9)/g_follow_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "max_velocity_reached";
    profiling_data_srv_inst.request.value = g_max_velocity_reached;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }
}

void future_collision_callback(const mavbench_msgs::future_collision::ConstPtr& msg) {
    if (msg->future_collision_seq > future_collision_seq) {
        future_collision_seq = msg->future_collision_seq;

        if (g_grace_period+g_time_to_come_to_full_stop < msg->time_to_collision)
            future_collision_time = ros::Time::now() + ros::Duration(g_grace_period);
        else
            future_collision_time = ros::Time::now();
    }
    //cout <<"-----"<<endl;
    //cout << int(msg->collision) <<endl;
    //cout << msg->time_to_collision <<endl;
    //cout << msg->future_collision_seq <<endl;
}

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    slam_lost = msg->data;
}


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


void callback_trajectory(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg, Drone * drone)
{
    //ROS_INFO_STREAM("recieved a new trajectory -------- in ft"); 
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

    if (msg->reverse) {
        fly_backward = true;
        last_to_fly_backward = ros::Time::now(); 
    } else if (trajectory.empty() && !new_trajectory.empty()) {
        // Add drift correction if the drone is currently idling (because it will float around while idling)
        const double max_idling_drift_distance = 0.5;
        trajectory_t idling_correction_traj;

        if (distance(drone->position(), new_trajectory.front()) > max_idling_drift_distance)
             idling_correction_traj = straight_line_trajectory(drone->position(), new_trajectory.front(), 1.0);

        trajectory = append_trajectory(idling_correction_traj, new_trajectory);
        fly_backward = false;
    } else if (msg->append) {
        trajectory = append_trajectory(trajectory, new_trajectory);
        fly_backward = false;
    } else {
        trajectory = new_trajectory;
        fly_backward = false;
    }

    g_got_new_trajectory = true;

    // Profiling
    if (CLCT_DATA){
        g_recieved_traj_t = ros::Time::now();
        g_msg_time_stamp = msg->header.stamp;
        if (g_msg_time_stamp.sec != 0) {
            g_pt_cld_to_futurCol_commun_acc += (ros::Time::now() - msg->header.stamp).toSec()*1e9;
            //std::cout<<"g_pt_cld_to_futurCol_commun_acc:"<<g_pt_cld_to_futurCol_commun_acc<<endl;
            //exit(0); 
            g_traj_ctr++;
        } 
    }
}


bool trajectory_done(const trajectory_t& trajectory) {
    trajectory.size() == 0;
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
    if(!ros::param::get("/follow_trajectory/num_fault_drone", num_fault)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! num_fault is not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/var_choose_drone", noise_select)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! noise_select is not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/range_select_drone", range_select_drone)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! range_select_drone is not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/num_sigma_drone", num_sigma)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! num_sigma_drone is not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/detect", detect)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! detect is not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/detect_percentage", detect_percentage)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! detect_percentage is not provided");
        exit(-1);
    }
    if(!ros::param::get("/follow_trajectory/error_threshold", error_threshold)) {
        ROS_FATAL("Could not start follow_trajectory. Parameter missing! error_threshold is not provided");
        exit(-1);
    }
    
    double a_max;
    if(!ros::param::get("a_max", a_max))  {
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
    srand(time(0));
    initialize_global_params();

    ros::Subscriber sub = n.subscribe("/FIPID", 1, FIPIDCallback);
    ros::Publisher PID_pub = n.advertise<mavbench_msgs::rosfi>("/PID", 1);
    ros::Publisher traced_process_pub = n.advertise<std_msgs::Int32>("/traced_process", 1);
    // Initialize the drone
    std::string localization_method;
    std::string ip_addr;
    const uint16_t port = 41451;

    ros::param::get("/follow_trajectory/ip_addr", ip_addr);
    ros::param::get("/follow_trajectory/localization_method", localization_method);
    Drone drone(ip_addr.c_str(), port, localization_method,
                g_max_yaw_rate, g_max_yaw_rate_during_flight, num_fault);

    // Initialize publishers and subscribers
    ros::Publisher next_steps_pub = n.advertise<mavbench_msgs::multiDOFtrajectory>("/next_steps", 1);

    ros::Subscriber slam_lost_sub = n.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
    ros::Subscriber col_coming_sub = n.subscribe<mavbench_msgs::future_collision>("/col_coming", 1, future_collision_callback);
    // ros::Subscriber traj_sub = n.subscribe<mavbench_msgs::multiDOFtrajectory>("normal_traj", 1, callback_trajectory);
    ros::Subscriber traj_sub = n.subscribe<mavbench_msgs::multiDOFtrajectory>("multidoftraj", 1, boost::bind(callback_trajectory, _1, &drone));

    // Begin execution loop
    bool app_started = false;  //decides when the first planning has occured
                               //this allows us to activate all the
                               //functionaliy in follow_trajectory accordingly

    ros::Rate loop_rate(50);
    bool sent = false;
    std_msgs::Int32 ready_msg;
    ready_msg.data = 0;
    mavbench_msgs::rosfi msg;
    sleep(1);
    prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY);
    while (ros::ok()) {
        if(!sent){
            drone.start_ros = ros::Time::now();
            msg.pid = getpid();
            msg.name = "follow_trajectory";
            PID_pub.publish(msg);
            ROS_INFO("publish follow_trajectory_node's id: %d", msg.pid);
            sent = true;
        }
        //sleep(1);
        if(ready){
            ready_msg.data = 1;
            traced_process_pub.publish(ready_msg);
            ready = false;
        }
        ros::spinOnce();

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
                            ROS_WARN_STREAM("========================================="<<(g_img_to_follow_acc/1e9)/g_follow_ctr<<endl); 
                            ROS_WARN_STREAM("this instance"<<(ros::Time::now() - g_msg_time_stamp).toSec());
                        }
                    
                    
                    }
                    if (DEBUG) {
                        //ROS_INFO_STREAM("follow_traj_cb to  func" << g_rcv_traj_to_follow_traj_t);
                        //ROS_INFO_STREAM("whatevs------- " << g_img_to_follow_acc);
                    }
                }
            }
        }

        // Figure out which direction we will fly in
        trajectory_t * forward_traj;
        trajectory_t * rev_traj;

        yaw_strategy_t yaw_strategy = follow_yaw;
        float max_velocity = g_v_max;

        
        if (fly_backward && ((ros::Time::now() - last_to_fly_backward).toSec() > 5)) { //prevent continous backward movement
            ROS_INFO_STREAM("setting flying back_ward to false"); 
            drone.fly_velocity(0, 0, 0);
            drone.fly_velocity(0, 0, 0);
            drone.fly_velocity(0, 0, 0);
            fly_backward = false;
        }

        if (!fly_backward) {
            forward_traj = &trajectory;
            rev_traj = &reverse_trajectory;
        } else {
            forward_traj = &reverse_trajectory;
            rev_traj = &trajectory;

            yaw_strategy = face_backward;
            max_velocity = g_v_max;
        }
        drone.num_sigma = num_sigma;
        drone.detect_percentage = detect_percentage;
        drone.threshold = error_threshold; //Threshold for construction error in autoencoder-based anomaly detection
        double max_velocity_reached = follow_trajectory(drone, forward_traj,
                rev_traj, yaw_strategy, true, max_velocity,
                g_fly_trajectory_time_out, noise_select, range_select_drone, detect);
        // Record histogram
        for (auto v:drone.drone_vx_list){
            drone_vx_list.push_back(v);
        }
        for (auto v:drone.drone_vy_list){
            drone_vy_list.push_back(v);
        }
        for (auto v:drone.drone_vz_list){
            drone_vz_list.push_back(v);
        }
        for (auto v:drone.drone_yaw_list){
            drone_yaw_list.push_back(v);
        }
        for (auto v:drone.drone_duration_list){
            drone_duration_list.push_back(v);
        }
        for (auto v:drone.drone_yaw_list){
            drone_yaw_list.push_back(v);
        }
        for (auto v:drone.recompute_time){
            recompute_time.push_back(v);
        }

        for (auto v:drone.x_list){
            x_list.push_back(v);
        }
        for (auto v:drone.y_list){
            y_list.push_back(v);
        }
        for (auto v:drone.z_list){
            z_list.push_back(v);
        }
        for (auto v:drone.vx_list){
            vx_list.push_back(v);
        }
        for (auto v:drone.vy_list){
            vy_list.push_back(v);
        }
        for (auto v:drone.vz_list){
            vz_list.push_back(v);
        }
        for (auto v:drone.ax_list){
            ax_list.push_back(v);
        }
        for (auto v:drone.ay_list){
            ay_list.push_back(v);
        }
        for (auto v:drone.az_list){
            az_list.push_back(v);
        }
        for (auto v:drone.yaw_list){
            yaw_list.push_back(v);
        }
        for (auto v:drone.duration_list){
            duration_list.push_back(v);
        }
        for (auto v:drone.error_list){
            error_list.push_back(v);
        }

        drone.drone_vx_list.clear();
        drone.drone_vy_list.clear();
        drone.drone_vz_list.clear();
        drone.drone_yaw_list.clear();
        drone.drone_duration_list.clear();
        drone.recompute_time.clear();

        drone.x_list.clear();
        drone.y_list.clear();
        drone.z_list.clear();
        drone.vx_list.clear();
        drone.vy_list.clear();
        drone.vz_list.clear();
        drone.ax_list.clear();
        drone.ay_list.clear();
        drone.az_list.clear();
        drone.yaw_list.clear();
        drone.duration_list.clear();
        drone.error_list.clear();

        num_recompute = drone.num_recompute;
        detection_time = drone.detection_time;
        // Record after fault injection
        //injected_detected = drone.injected_detected;
        //injected_not_detected = drone.injected_not_detected;
        //not_injected_detected = drone.not_injected_detected;
        //not_injected_not_detected = drone.not_injected_not_detected;

        /*difference_float = drone.difference_float;
        original_float = drone.original_float;
        after_float = drone.after_float;
        difference_float32 = drone.difference_float32;
        original_float32 = drone.original_float32;
        after_float32 = drone.after_float32;
        is_float = drone.is_float;*/
        if (max_velocity_reached > g_max_velocity_reached)
            g_max_velocity_reached = max_velocity_reached;

        // Publish the remainder of the trajectory
        mavbench_msgs::multiDOFtrajectory trajectory_msg = create_trajectory_msg(*forward_traj);
        trajectory_msg.future_collision_seq = future_collision_seq;
        trajectory_msg.trajectory_seq = trajectory_seq;
        trajectory_msg.reverse = fly_backward;

        next_steps_pub.publish(trajectory_msg);
        //std::cout << "output trajectory in follow_trajectory" << "\n";
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
        } else if (trajectory_done(*forward_traj)){
            loop_rate.sleep();
        }

        g_got_new_trajectory = false;
    }

    return 0;
}

