#include "ros/ros.h"

#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <thread>
#include <fstream>
#include <cstdlib>
#include <time.h>
#include <signal.h>
#include <string>

#include "pid.h"
#include "Drone.h"
#include "common.h"
#include "bounding_box.h"
#include "follow_the_leader/bounding_box_msg.h"
#include "follow_the_leader/cmd_srv.h"
#include <profile_manager/profiling_data_srv.h>
#include "error.h"

using namespace std;
std::string ip_addr__global;
std::string localization_method;
std::queue<bounding_box> bb_queue; // Used to buffer imags while detection is running
float height_ratio;

//long long g_error_accumulate = 0;
//int g_error_ctr = 0;

int image_w__global;// = 400;
int  image_h__global; //= 400; //this must be equal to the img being pulled in from airsim

float vx__P__global; //= (float)2.0/(image_h/2); 
float vy__P__global; //= (float)3.0/(image_w/2); 
float vz__P__global; //= (float)1.0;

float vx__I__global; //= .05;
float vy__I__global; //= .05;
float vz__I__global; //= .05;

float vx__D__global; //= .1;
float vy__D__global; //= .1;
float vz__D__global; //= .1;

void log_data_before_shutting_down(){
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
}

void sigIntHandlerPrivate(int signo){
    if (signo == SIGINT) {
        log_data_before_shutting_down(); 
        ros::shutdown();
    }
    exit(0);
}

void bb_cb(const follow_the_leader::bounding_box_msg::ConstPtr& msg) {
    bounding_box bb;
    bb.x = msg->x;
    bb.y = msg->y;
    bb.w = msg->w;
    bb.h = msg->h;
    bb.conf  = msg->conf;
    bb_queue.push(bb);
}

void fly_towards_target(Drone& drone, const bounding_box& bb,
        int img_height, int img_width, PID& pid_vx, PID& pid_vy, PID& pid_vz,
        double dt)
{
    static float hover_height = drone.pose().position.z;

    auto yaw = drone.get_yaw();
    if (yaw > 15 || yaw < -15) {
        cout << "Correcting yaw\n";
        drone.set_yaw(0);
    }
    
    double img__cntr =  img_width / 2;
    double vy = pid_vy.calculate(height_ratio, bb.h/img_height,  dt); 
    double vx = pid_vx.calculate(bb.x + bb.w/2, img_width/2, dt); 
    double vz = pid_vz.calculate(1*(double)img_height/2, bb.y + bb.h/2, dt); 
    
    if (vy >=3 || vy<=-3) {
        ROS_INFO_STREAM("vy:"<<vy);
    }
    drone.fly_velocity(vx, vy, vz);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, sigIntHandlerPrivate);
 
    uint16_t port = 41451;

    ros::Subscriber bb_sub = nh.subscribe("/bb_topic", 4, bb_cb);
    
    if(!ros::param::get("/pid_node/ip_addr__global",ip_addr__global) ||
            !ros::param::get("/pid_node/vx__P__global",vx__P__global)||
            !ros::param::get("/pid_node/vy__P__global",vy__P__global)||
            !ros::param::get("/pid_node/vz__P__global",vz__P__global)||

            !ros::param::get("/pid_node/vx__I__global",vx__I__global)||
            !ros::param::get("/pid_node/vy__I__global",vy__I__global)||
            !ros::param::get("/pid_node/vz__I__global",vz__I__global)||

            !ros::param::get("/pid_node/vx__D__global",vx__D__global)||
            !ros::param::get("/pid_node/vy__D__global",vy__D__global)||
            !ros::param::get("/pid_node/vz__D__global",vz__D__global)||

            !ros::param::get("/image_w__global",image_w__global)||
            !ros::param::get("/image_h__global",image_h__global)||
            !ros::param::get("/localization_method",localization_method)||
            !ros::param::get("/height_ratio",height_ratio)
            ){
        ROS_FATAL("you did not specify all the parameters");
        return -1; 
    }
    
    int loop_rate = 30; 
    ros::Rate pub_rate(loop_rate);
    float dt = ((float)1)/(float)loop_rate;
    Drone drone(ip_addr__global.c_str(), port, localization_method);
    PID pid_vx(vx__P__global, vx__I__global, vx__D__global, 3.5, -3.5);
    PID pid_vy(vy__P__global, vy__I__global, vy__D__global, 3.5, -3.5);
    PID pid_vz(vz__P__global, vz__I__global, vz__D__global, 3.5, -3.5); //at the moment only for keeping drone stable

    while (ros::ok())
    {
        while(!bb_queue.empty()) {
            auto bb = bb_queue.front(); 
            bb_queue.pop(); 
            fly_towards_target(drone, bb, image_h__global, image_w__global, pid_vx, pid_vy, pid_vz, dt); // dt is not currently used
        }
        ros::spinOnce(); 
    }
}
