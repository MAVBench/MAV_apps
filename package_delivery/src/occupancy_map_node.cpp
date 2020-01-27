#include <Drone.h>
#include <future_collision/future_collision.h>
#include <octomap_server/OctomapServer.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <rosconsole/macros_generated.h>
#include <signal.h>
#include <XmlRpcValue.h>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <std_msgs/Bool.h>

#include "motion_planner.h"

octomap_server::OctomapServer* server_ptr;


void log_data_before_shutting_down() {
	profile_manager::profiling_data_srv profiling_data_srv_inst;
    profile_manager::profiling_data_verbose_srv profiling_data_verbose_srv_inst;
    server_ptr->profiling_container.setStatsAndClear();
    profiling_data_verbose_srv_inst.request.key = ros::this_node::getName()+"_verbose_data";
    profiling_data_verbose_srv_inst.request.value = "\n" + server_ptr->profiling_container.getStatsInString();
    server_ptr->my_profile_manager.verboseClientCall(profiling_data_verbose_srv_inst);
}


void sigIntHandlerPrivate(int signo) {
    if (signo == SIGINT) {
    	std_msgs::Bool msg;
    	msg.data = true;
    	server_ptr->inform_pc_done_pub.publish(msg); // -- informing point cloud to publish its profiling results. This is a hack, because I can't get the nodelet to register the sigInt
    	log_data_before_shutting_down();
    	ros::shutdown();
    }
    exit(0);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_map_node");
    ros::NodeHandle nh("~");
    std::string mapFilename(""), mapFilenameParam("");
    signal(SIGINT, sigIntHandlerPrivate);

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
    //octomap::OcTree * octree = server.tree_ptr();


    if (nh.getParam("/occupancy_map_node/map_file", mapFilenameParam)) {
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
    server_ptr = &server;
    ros::Duration(1).sleep(); // -- need this to prevent octomap from running before imgPublisher/point cloud
    while (ros::ok()) {
    	server.spinOnce();
//    	ros::spinOnce();
    }
}

