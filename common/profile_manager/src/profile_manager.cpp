#include <boost/smart_ptr/shared_ptr.hpp>
#include <common.h>
#include <coord.h>
#include <ros/duration.h>
#include <Drone.h>
#include <ros/init.h>
#include <ros/param.h>
#include "profile_manager.h"
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/profiling_data_verbose_srv.h>
#include <ros/service.h>
#include <ros/this_node.h>
#include <cmath>
#include <deque>


ProfileManager::ProfileManager(std::string mode, std::string channel_name, std::string verbose_channel_name):nh("~"),
    channel_name(channel_name), verbose_channel_name(verbose_channel_name)
{
    // Create a new callback queue
    //nh.setCallbackQueue(&callback_queue);
    // Topics
    if (mode != "client") {
        std::cout<<"mode: "<< mode << " is not supported yet"<<endl;
        throw;
    }
}

void ProfileManager::verboseClientCall (profile_manager::profiling_data_verbose_srv data){
	if (ros::service::waitForService(verbose_channel_name, 5)){ // @suppress("Invalid arguments")
		if(!ros::service::call(verbose_channel_name, data)){
			ROS_ERROR_STREAM("could not probe data using stats manager");
			ros::shutdown();
		}
	}
}

void ProfileManager::clientCall(profile_manager::profiling_data_srv data) {
	if (ros::service::waitForService(channel_name, 5)){ // @suppress("Invalid arguments")
		if(!ros::service::call(channel_name, data)){
			ros::shutdown();
		}
	}else {
		ROS_INFO_STREAM("waited but didn't got a response back"<<data.request.key);
	}
}
