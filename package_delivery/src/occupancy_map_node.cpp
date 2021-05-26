#include "ros/ros.h"

#include <stdio.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <std_srvs/Empty.h>
#include <ros/duration.h>
#include <sys/ptrace.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <sys/prctl.h>
#include <iostream>
#include <fstream>
// Standard headers
#include <string>
#include <signal.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ptrace.h>
#include <sys/types.h>
#include <sys/user.h>

// Octomap specific headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

// MAVBench headers
#include "Drone.h"
#include "timer.h"
#include "motion_planner.h"
#include <future_collision/future_collision.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>

// Octomap server headers
#include <octomap_server/OctomapServer.h>

#include <mavbench_msgs/rosfi.h>


FutureCollisionChecker * fcc_ptr = nullptr;
MotionPlanner * mp_ptr = nullptr;
double total_octo = 0.0;
int count_octo = 0;
double total_fcc = 0.0;
int count_fcc = 0;
double total_mp = 0.0;
int count_mp = 0;


int FI_PID;
bool start_FI = false;
bool ready = false;

void FIPIDCallback(const std_msgs::Int32::ConstPtr& msg)
{
    FI_PID = msg->data;
  
  ready = true;
  ROS_INFO("FI_PID: [%d]", msg->data);
}
void StartCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if(msg->data >=1){
    ROS_INFO("traced_process start!");
    start_FI = true;
  }
  

}

void sigIntHandlerPrivate(int signo) {
    if (signo == SIGINT) {
        std::cout << "octomap takes " << total_octo / count_octo << "\n";
        std::cout << "fcc takes " << total_fcc / count_fcc << "\n";
        std::cout << "mp takes " << total_mp / count_mp << "\n";
        fcc_ptr->log_data_before_shutting_down();
        mp_ptr->log_data_before_shutting_down();
        ros::shutdown();
    }
    exit(0);
}
void sigCONTHandlerPrivate(int signo) {
    if (signo == SIGCONT) {
      raise(SIGCONT);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_map_node");
    ros::NodeHandle nh("~");
    std::string mapFilename(""), mapFilenameParam("");
    signal(SIGINT, sigIntHandlerPrivate);
    signal(SIGCONT, sigCONTHandlerPrivate);

    ros::Subscriber sub = nh.subscribe("/FIPID", 1, FIPIDCallback);
    //ros::Publisher Start_pub = nh.advertise<std_msgs::Int32>("/start", 1);

    ros::Publisher PID_pub = nh.advertise<mavbench_msgs::rosfi>("/PID", 1);
    ros::Publisher traced_process_pub = nh.advertise<std_msgs::Int32>("/traced_process", 1);

    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 
    // Create a Drone object
    std::string ip_addr, localization_method;
    ros::param::get("/ip_addr", ip_addr);
    uint16_t port = 41451;
    if(!ros::param::get("/localization_method", localization_method)) {
        ROS_FATAL("Could not start occupancy map node. Localization parameter missing!");
        exit(-1);
    }
    Drone drone(ip_addr.c_str(), port, localization_method);
    
    // Create an octomap server
    octomap_server::OctomapServer server;
    octomap::OcTree * octree = server.tree_ptr();

    if (nh.getParam("map_file", mapFilenameParam)) {
        if (mapFilename != "") {
            ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'", mapFilename.c_str(), mapFilenameParam.c_str(), mapFilename.c_str());
        } else {
            mapFilename = mapFilenameParam;
        }
    }

    if (mapFilename != "") {
        if (!server.openFile(mapFilename)){
            ROS_ERROR("Could not open file %s", mapFilename.c_str());
            exit(1);
        }
    }

    // Create FutureCollisionChecker
    FutureCollisionChecker fcc (octree, &drone);
    fcc.setOctomapServer(&server); // This is only used for profiling purposes.
    fcc_ptr = &fcc;
    // Create MotionPlanner
    MotionPlanner mp (octree, &drone);
    mp_ptr = &mp;
    bool start;

    sleep(1);
    prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY);
    int count = 0;
    int accum = 0;
    bool sent = false;
    int succeed = -2;
    bool once = false;
    std_msgs::Int32 ready_msg;
    ready_msg.data = 0;
    mavbench_msgs::rosfi msg;
    bool ready_inject = false;
    bool pick_inject = false;
    std_msgs::Int32 start_msg;
    int add_random;
    srand(time(0));
    while (ros::ok()) {
        if(!sent){
            fcc_ptr->start_ros = ros::Time::now();
            mp_ptr->start_ros = ros::Time::now();
            drone.start_ros = ros::Time::now();
            msg.pid = getpid();
            msg.name = "occupancy_map_node";
            PID_pub.publish(msg);
            ROS_INFO("publish %s's id: %d", msg.name.c_str(), msg.pid);
            sent = true;
        }
        //sleep(1);
        if(ready){
            //mp.FI_PID = FI_PID;
            //mp.ready = true;
            ready_msg.data = 1;
            traced_process_pub.publish(ready_msg);
            ready_inject = true;
            ready = false;
        }
        ros::Time start_octo;
        ros::Time end_octo;

        start = true;
        add_random = rand()%10;
        /*if(ready_inject && pick_inject && !once && add_random == 0){
            start_msg.data = 1;
            Start_pub.publish(start_msg);
            ROS_INFO("try to wake %d", FI_PID);
            once = true;
            ready_inject = false;
            start_octo = ros::Time::now();
            ros::spinOnce();
            end_octo = ros::Time::now();
            std::cout << "octomap takes " << (end_octo - start_octo).toSec() << "\n";
        }
        else{
            //ROS_INFO("Start octo");
            start_octo = ros::Time::now();
            ros::spinOnce();
            end_octo = ros::Time::now();
        }*/
        start_octo = ros::Time::now();
        ros::spinOnce();
        end_octo = ros::Time::now();
        if((end_octo - start_octo).toSec() > 0.01 && total_octo > 20 ){
            pick_inject = true;
            
            //ROS_INFO("End octo");
            //std::cout << "octomap takes " << (end_octo - start_octo).toSec() << "\n";
        }
        //start_FI = false;

        
        count_octo++;
        total_octo += (end_octo - start_octo).toSec();

        ros::Time start_fcc = ros::Time::now();
        if(fcc.recompute || start){
            fcc.spinOnce();
            count_fcc ++;
            start = false;
        }
        ros::Time end_fcc = ros::Time::now();
        //std::cout << "fcc takes " << (end_fcc - start_fcc).toSec() << "\n";
        total_fcc += (end_fcc - start_fcc).toSec();

        //ROS_INFO("Start mp");
        ros::Time start_mp = ros::Time::now();
        mp.spinOnce();
        //ROS_INFO("End mp");
        ros::Time end_mp = ros::Time::now();
        count_mp++;
        //std::cout << "mp takes " << (end_mp - start_mp).toSec() << "\n";
        total_mp += (end_mp - start_mp).toSec();
    }
}

