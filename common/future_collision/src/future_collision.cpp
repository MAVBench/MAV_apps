#include <future_collision/future_collision.h>

// Standard headers
#include <cmath>

// MAVBench headers
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>

#include <fstream>
#include <typeinfo>
#include <vector>
#include <list>
using namespace std;
// for saving results
std::list<float> col_timetocol_list;
std::list<int> col_future_col_seq_list;
std::list<float> recompute_time;
int num_recompute = 0;

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


std::tuple <bool, double> FutureCollisionChecker::check_for_collisions(Drone& drone)
{
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
    if(!ros::param::get("/DEBUG",DEBUG)) {
        ROS_FATAL("Could not start future_collision. DEBUG parameter missing!");
        return; 
    }
    if(!ros::param::get("/follow_trajectory/grace_period", grace_period__global)) {
        ROS_FATAL("Could not start future_collision. grace_period parameter missing!");
        return;
    }
    if(!ros::param::get("/follow_trajectory/num_fault", num_fault)) {
        ROS_FATAL("Could not start noise injection to future_collision. num_fault parameter missing!");
        return;
    }
    if(!ros::param::get("/follow_trajectory/var_choose_col", var_choose_col)) {
        ROS_FATAL("Could not start noise injection to future_collision. var_choose_col parameter missing!");
        return;
    }
    if(!ros::param::get("/follow_trajectory/range_select_col", range_select_col)) {
        ROS_FATAL("Could not start noise injection to future_collision. range_select_col parameter missing!");
        return;
    }
    if(!ros::param::get("/follow_trajectory/num_sigma_col", num_sigma)) {
        ROS_FATAL("Could not start noise injection to future_collision. num_sigma parameter missing!");
        return;
    }
    if(!ros::param::get("/follow_trajectory/detect", detect)) {
        ROS_FATAL("Could not start noise injection to future_collision. detect parameter missing!");
        return;
    }
    if(!ros::param::get("/follow_trajectory/detect_percentage", detect_percentage)) {
        ROS_FATAL("Could not start noise injection to future_collision. detect_percentage parameter missing!");
        return;
    }
}

void FutureCollisionChecker::log_data_before_shutting_down()
{
    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    //write the variable vaules into .txt files
    /*std::ofstream outfile;
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/detect_perception_summary.txt", std::ios_base::app);
    outfile << injected_detected << ", " << injected_not_detected << ", " << not_injected_detected << ", " << not_injected_not_detected << "\n";
    outfile.close();

    std::cout << "collision stage:" << "\n";
    std::cout << injected_detected << ", " << injected_not_detected << ", " << not_injected_detected << ", " << not_injected_not_detected << "\n";

    // write the location date into a .txt file
    //std::ofstream outfile;
    if(is_float){
        outfile.open("/home/nvidia/MAVBench_base/src/MAV_apps/package_delivery/src/collision_float64_var_" + std::to_string(var_choose_col)
            + "_range_" + std::to_string(range_select_col) + ".txt", std::ios_base::app);
        outfile << original_float << ", " << after_float << ", " <<  difference_float << "\n";
    }
    else {
        outfile.open("/home/nvidia/MAVBench_base/src/MAV_apps/package_delivery/src/collision_int_var_" + std::to_string(var_choose_col)
            + "_range_" + std::to_string(range_select_col) + ".txt", std::ios_base::app);
        outfile << original_int << ", " << after_int << ", " <<  difference_int << "\n";
    }
    outfile.close();*/
    //write the variable vaules into .txt files
    ofstream outfile;
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/col_time_to_col.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:col_timetocol_list){
        outfile << v << "\n";}
    outfile.close();
    
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/col_future_col_seq.txt", std::ios_base::app);
    outfile << -10000 << "\n";
    for (auto v:col_future_col_seq_list){
        outfile << v << "\n";}
    outfile.close();

    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/recompute.txt", std::ios_base::app);
    outfile << "future_collision: "<< num_recompute << "\n";
    outfile.close();
    outfile.open("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/data/package_delivery/variable/recompute_time.txt", std::ios_base::app);
    outfile << 1 << "\n";
    for (auto v:recompute_time){
         outfile << v << "\n";}
    outfile.close();

    profiling_data_srv_inst.request.key = "future_collision_kernel";
    profiling_data_srv_inst.request.value = (((double)g_checking_collision_kernel_acc)/1e9)/g_check_collision_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "future_collision_main_loop";
    profiling_data_srv_inst.request.value = (((double)g_future_collision_main_loop)/1e9)/g_check_collision_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "img_to_octomap_commun_t";
    profiling_data_srv_inst.request.value = ((double)g_pt_cld_to_octomap_commun_olverhead_acc/1e9)/octomap_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager using octomap");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    profiling_data_srv_inst.request.key = "octomap_integration";
    profiling_data_srv_inst.request.value = (((double)octomap_integration_acc)/1e9)/octomap_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            //ROS_ERROR_STREAM("could not probe data using stats manager using octomap");
            ROS_INFO("could not probe data using stats manager");
            ros::shutdown();
        }
    }

    ROS_INFO_STREAM("done with the octomap profiles");
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

void FutureCollisionChecker::fault_injection_double(double &original){
    int noise; 
    int pick;
    int select;
    original_float = original;
    bool success = false;
    std::vector<int> record;
    is_float = true;

   // write inputted data into the file.
    std::cout << "print float64: " << original << std::endl;
    for(int i = 0; i < num_fault; i++){
        while(!success){
            if(range_select_col == 1){
                select = 63;
            }
            else if(range_select_col == 2){
                select = rand() % 11 + 52;
            }
            else if(range_select_col == 3){
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
    after_float = original;
    record.clear();
    difference_float = after_float - original_float;
    std::cout << "after injection print float64: " << original << std::endl;
    std::cout << "difference: " << difference_float << std::endl;

}

void FutureCollisionChecker::fault_injection_int(int &original){
    int noise; 
    int pick;
    int select;
    original_int = original;
    is_float = false;

    bool success = false;
    std::vector<int> record;
    std::cout << "print int32: " << original << std::endl;
    for(int i = 0; i < num_fault; i++){
        while(!success){
            if(range_select_col == 1){
                select = 31;
            }
            else if(range_select_col == 2){
                select = rand() % 31;
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
    after_int = original;
    record.clear();
    difference_int = after_int - original_int;
    std::cout << "after injection print int32: " << original << std::endl;
    std::cout << "difference: " << difference_int << std::endl;
}
void FutureCollisionChecker::fault_injection_bool(bool &original){
    if(original){
        original = false;
    }
    else{
        original = true;
    }
}
void FutureCollisionChecker::detect_double(std::vector<double> &points, double mean, double stddev){
    double upper_b = mean + num_sigma * stddev;
    double lower_b = mean - num_sigma * stddev;
    double delta;
    bool current_recompute = false;

    /*for(int i = 0; i < points.size(); i++){
        std::cout << i << ": " << points[i] << " ";
    }*/
    delta = points[1] - points[0];
    if(delta > upper_b || delta < lower_b){
        //std::cout << " upper_b: " << upper_b << " lower_b: " << lower_b << "\n";
        std::cout << "collision double delta: " << delta << "\n";
        recompute = true;
        current_recompute = true;
    } 


}
void FutureCollisionChecker::detect_int(std::vector<int> &points, double mean, double stddev){
    float upper_b = mean + num_sigma * stddev;
    float lower_b = mean - num_sigma * stddev;
    int delta;
    bool current_recompute = false;


    delta = points[1] - points[0];
    if(delta > upper_b || delta < lower_b){
        std::cout << "collision int delta: " << delta << "\n";
        recompute = true;
        current_recompute = true;
    } 

}
void FutureCollisionChecker::detect_all(){
    // Starting from index = 1
    std::vector<double> mean = {-0.0366667, 0.481667};
    std::vector<double> stddev = {0.272438, 0.552974};
    //std::vector<double> mean = {-0.714444, 1};
    //std::vector<double> stddev = {5.3363, 0.358207};
    int num_detect = 2.0 * detect_percentage;
    int detect_run = 0;
    int select;
    bool success = false;
    if(record_detect.empty()){
        if(detect_percentage == 1){
            for(int i = 0; i< 2; i++){
                record_detect.push_back(i);
            }
        }
        else{
            while(detect_run < num_detect){
                success = false;
                while(!success){
                    select = rand() % 2;
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
        std::cout << "perception: ";
        for(int i = 0; i < record_detect.size(); i++){
            std::cout << record_detect[i] << ", ";
        }
        std::cout << "\n";
    }

    for(int i = 0; i < record_detect.size(); i++){
        select = record_detect[i];
        if(select == 0){
            detect_double(g_time_to_collision, mean[0], stddev[0]);
        }
        else if(select == 1){
            detect_int(g_future_collision_seq_id, mean[1], stddev[1]);
        }
    }

    if(injected && one_time_injection == false){
        one_time_injection = true;
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

    if(recompute){
        end_ros = ros::Time::now();
        recompute_time.push_back((end_ros - start_ros).toSec());
        num_recompute++;
        std::cout << "recompute: " << recompute << "\n";
        std::cout << "\n";
    }

}
void FutureCollisionChecker::spinOnce()
{
    static ros::Time main_loop_start_hook_t;
    static ros::Time main_loop_end_hook_t;
    // For noise injection
    srand(time(0));


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
            bool success = false;
            std::vector<int> record;
            // noise injection
            if(!injected && var_choose_col != 0){
                if(var_choose_col == 1){
                    fault_injection_double(time_to_collision);
                }
                else if(var_choose_col == 2){
                    fault_injection_int(future_collision_seq_id);
                }
                else if(var_choose_col == 3){
                    fault_injection_bool(collision_coming);
                }
                injected = true;
            }

            g_time_to_collision[1] = time_to_collision;
            g_future_collision_seq_id[1] = future_collision_seq_id;
            recompute = false;
            if(detect == 1){
                detect_all();
            }

            g_time_to_collision[0] = g_time_to_collision[1];
            g_future_collision_seq_id[0] = g_future_collision_seq_id[1];

            col_coming_msg.future_collision_seq = future_collision_seq_id;
            col_coming_msg.collision = collision_coming;
            col_coming_msg.time_to_collision = time_to_collision;
    
            // saving values in lists
            col_timetocol_list.push_back(col_coming_msg.time_to_collision);
            col_future_col_seq_list.push_back(col_coming_msg.future_collision_seq);
            
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

