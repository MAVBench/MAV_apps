#include <boost/smart_ptr/shared_ptr.hpp>
#include <common.h>
#include <coord.h>
#include <ros/duration.h>
#include <Drone.h>
#include <future_collision/future_collision.h>
#include <ros/init.h>
#include <mavbench_msgs/future_collision.h>
#include <mavbench_msgs/multiDOFtrajectory.h>
/*
#include <octomap_server/OctomapServer.h>
#include <octomap/octomap_types.h>
#include <octomap/OccupancyOcTreeBase.hxx>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeIterator.hxx>
*/
#include <ros/param.h>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/profiling_data_verbose_srv.h>
#include <ros/publisher.h>
#include <rosconsole/macros_generated.h>
#include <ros/service.h>
#include <ros/this_node.h>
#include <cmath>
#include <deque>


bool FutureCollisionChecker::collision(const octomap::OcTree * octree, const multiDOFpoint& n1, const multiDOFpoint& n2) const
{
    const double height = drone_height__global; 
    const double radius = drone_radius__global; 

    // Create a bounding box representing the drone
    octomap::point3d min(n1.x-radius, n1.y-radius, n1.z-height/2);
    octomap::point3d max(n1.x+radius, n1.y+radius, n1.z+height/2);

    // Create a direction vector over which to check for collisions
    double dx = n2.x - n1.x;
    double dy = n2.y - n1.y;
    double dz = n2.z - n1.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    octomap::point3d direction(dx, dy, dz);

    for (auto it = octree->begin_leafs_bbx(min, max),
            end = octree->end_leafs_bbx(); it != end; ++it)
    {
        octomap::point3d start_point (it.getCoordinate());
        octomap::point3d end_point;
        if (octree->castRay(start_point, direction, end_point, true, distance))
            return true;
    }

    return false;
}

void FutureCollisionChecker::pull_traj(const mavbench_msgs::multiDOFtrajectory::ConstPtr& msg)
{
    traj_future_collision_seq_id = msg->future_collision_seq;

    if (msg->points.size() > 0) {
        auto pos = drone->position();
        const auto& traj_front = msg->points.front();
        double x_offset = pos.x - traj_front.x;
        double y_offset = pos.y - traj_front.y;
        double z_offset = pos.z - traj_front.z;

        traj = create_trajectory_from_msg(*msg);
        for (auto& point : traj) {
            point.x += x_offset;
            point.y += y_offset;
            point.z += z_offset;
        }
    } else {
        traj = trajectory_t();
    }
}

//
std::tuple <bool, double> FutureCollisionChecker::check_for_collisions(Drone& drone)
{

	if (g_micro_benchmark){ //short circuit the execution and always return 1
		return std::make_tuple(true, 1.0);
	}

	start_hook_chk_col_t = ros::Time::now();

    if (octree == nullptr || traj.empty())
        return std::make_tuple(false, 0.0);

    bool col = false;
    double time_to_collision = 0.0;

    for (int i = 0; i < traj.size() - 1; ++i) {
        const auto& pos1 = traj[i];
        const auto& pos2 = traj[i+1];

        if (collision(octree, pos1, pos2)) {
            col = true;

            if(CLCT_DATA) {
                // g_distance_to_collision_first_realized = dist_to_collision(drone, pos1);
                g_distance_to_collision_first_realized = time_to_collision;
            }

            break;
        }

        time_to_collision += pos1.duration;
    }

    end_hook_chk_col_t = ros::Time::now(); 
    g_checking_collision_t = end_hook_chk_col_t;
    g_checking_collision_kernel_acc += ((end_hook_chk_col_t - start_hook_chk_col_t).toSec()*1e9);
    g_check_collision_ctr++;

    return std::make_tuple(col, time_to_collision);
}


void FutureCollisionChecker::future_collision_initialize_params()
{
	ros::param::get("/future_col_drone_radius", drone_radius__global);
    ros::param::get("/future_col_drone_height", drone_height__global);

    if(!ros::param::get("/ip_addr", ip_addr__global)) {
        ROS_FATAL("Could not start future_collision. IP address parameter missing!");
        return;
    }
    if(!ros::param::get("/localization_method", localization_method)) {
        ROS_FATAL("Could not start future_collision. Localization parameter missing!");
        return;
    }
	if(!ros::param::get("/CLCT_DATA",CLCT_DATA)) {
        ROS_FATAL("Could not start future_collision. CLCT_DATA parameter missing!");
        return;
    }

	if(!ros::param::get("/micro_benchmark",g_micro_benchmark)) {
        ROS_FATAL("Could not start future_collision. micro_benchmark parameter missing!");
        return;
    }

	if(!ros::param::get("/DEBUG",DEBUG)) {
        ROS_FATAL("Could not start future_collision. DEBUG parameter missing!");
        return; 
    }

    if(!ros::param::get("/follow_trajectory/grace_period", grace_period__global)) {
        ROS_FATAL("Could not start future_collision. grace_period parameter missing!");
        return;
    }

}



void FutureCollisionChecker::log_data_before_shutting_down()
{
	std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    profile_manager::profiling_data_verbose_srv profiling_data_verbose_srv_inst;
    server->profiling_container.setStatsAndClear();

	for (auto &data: server->profiling_container.container){
    	profiling_data_srv_inst.request.key = data.data_key_name + " last window's avg: ";
		vector<double>* avg_stat = data.getStat("avg");
		if (avg_stat) {
			profiling_data_srv_inst.request.value = avg_stat->back();

		}
		else{
			profiling_data_srv_inst.request.value = nan("");
		}
		profile_manager.clientCall(profiling_data_srv_inst);
	}

    profiling_data_verbose_srv_inst.request.key = ros::this_node::getName()+"_verbose_data";
	profiling_data_verbose_srv_inst.request.value = "\n" + server->profiling_container.getStatsInString();
    profile_manager.verboseClientCall(profiling_data_verbose_srv_inst);


/*
    profiling_data_srv_inst.request.key = "future_collision_kernel";
    profiling_data_srv_inst.request.value = (((double)g_checking_collision_kernel_acc)/1e9)/g_check_collision_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "future_collision_main_loop";
    profiling_data_srv_inst.request.value = (((double)g_future_collision_main_loop)/1e9)/g_check_collision_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "img_to_octomap_commun_t";
    profiling_data_srv_inst.request.value = ((double)g_pt_cld_to_octomap_commun_olverhead_acc/1e9)/octomap_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager using octomap");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "octomap_integration";
    profiling_data_srv_inst.request.value = (((double)octomap_integration_acc)/1e9)/octomap_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager using octomap");
            ros::shutdown();
        }
    }

    ROS_ERROR_STREAM("done with the octomap profiles");
	*/
}


void FutureCollisionChecker::stop_drone()
{
    while (ros::ok() &&
            (traj_future_collision_seq_id < future_collision_seq_id
             || (traj_future_collision_seq_id == future_collision_seq_id
                 && !traj.empty())))
    {
        callback_queue.callAvailable(ros::WallDuration());
        drone->fly_velocity(0,0,0);
    }
}

void FutureCollisionChecker::spinOnceDummy()
{
	mavbench_msgs::future_collision col_coming_msg;
	col_coming_msg.header.stamp = g_pt_cloud_header;
	col_coming_msg.future_collision_seq = future_collision_seq_id;
    future_collision_seq_id++;
	col_coming_msg.collision = true;
	col_coming_msg.time_to_collision = 10;
	col_coming_pub.publish(col_coming_msg);
}

void FutureCollisionChecker::spinOnce()
{
	if (g_micro_benchmark){
		spinOnceDummy();
		return;
	}

	static ros::Time main_loop_start_hook_t;
    static ros::Time main_loop_end_hook_t;

    main_loop_start_hook_t = ros::Time::now();

    callback_queue.callAvailable(ros::WallDuration());

    if (CLCT_DATA){ 
        g_pt_cloud_header = server->rcvd_point_cld_time_stamp; 
        octomap_ctr = server->octomap_ctr;
        octomap_integration_acc = server->octomap_integration_acc; 
        g_pt_cld_to_octomap_commun_olverhead_acc = server->pt_cld_octomap_commun_overhead_acc;
    }

    if (traj_future_collision_seq_id >= future_collision_seq_id) {
        bool collision_coming;
        double time_to_collision;

        std::tie(collision_coming, time_to_collision)
            = check_for_collisions(*drone);

        if (collision_coming) {
            future_collision_seq_id++;

            mavbench_msgs::future_collision col_coming_msg;
            col_coming_msg.header.stamp = g_pt_cloud_header;
            col_coming_msg.future_collision_seq = future_collision_seq_id;
            col_coming_msg.collision = collision_coming;
            col_coming_msg.time_to_collision = time_to_collision;

            col_coming_pub.publish(col_coming_msg);

            ROS_WARN("future_collision: Collision on trajectory!");

            if (grace_period__global == 0) {
                // If the drone is supposed to immediately stop, then stop it
                // here until follow_trajectory catches up
                ROS_INFO("Stopping the drone immediately inside FutureCollision");
                stop_drone();
            }

            // Profiling
            if(CLCT_DATA)
                g_pt_cloud_to_future_collision_t = start_hook_chk_col_t - g_pt_cloud_header;
            if(DEBUG)
                ROS_INFO_STREAM("pt cloud to start of checking collision in future collision"<< g_pt_cloud_to_future_collision_t);
        }
    } else if (grace_period__global == 0) {
        // If the drone is supposed to immediately stop, then stop it here so
        // we're faster
        ROS_INFO("Stopping the drone immediately inside FutureCollision");
        stop_drone();
    } else {
        ROS_WARN("Future collision will ignore old trajectory");
    }

    main_loop_end_hook_t = ros::Time::now();
    g_future_collision_main_loop += (main_loop_end_hook_t - main_loop_start_hook_t).toSec()*1e9; 

}

