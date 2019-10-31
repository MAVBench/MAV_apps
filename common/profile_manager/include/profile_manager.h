#ifndef PROFILE_MANAGER_H
#define PROFILE_MANAGER_H

#include "ros/ros.h"

// Standard headers
#include <string>
#include <tuple>

// MAVBench headers
#include "timer.h"
#include <iostream>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/profiling_data_verbose_srv.h>

class ProfileManager {
	public:
		ProfileManager(std::string channel_name, std::string verbose_channel_name,
                std::string mode);
		void verboseClientCall(profile_manager::profiling_data_verbose_srv data);
		void clientCall(profile_manager::profiling_data_srv prof_data);
	private:
		ros::NodeHandle nh;
        std::string channel_name, verbose_channel_name;  // the channel to communicate the
                                                     // the results with
};

#endif

