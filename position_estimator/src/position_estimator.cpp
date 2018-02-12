#include <ros/ros.h>

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "position_estimator");
    ros::NodeHandle nh;
    ns = ros::this_node::getName();
    
    //----------------------------------------------------------------- 
	// *** F:DN variables	
	//----------------------------------------------------------------- 	
    //uint16_t port = 41451;
    //Drone drone(ip_addr__global.c_str(), port, localization_method);
                                              //pkg and successfully returned to origin
    // *** F:DN subscribers,publishers,servers,clients
    ros::ServiceClient SLAM1_client = 
        nh.serviceClient<std_msgs::Bool>("/SLAM1/start_stop");
    ros::Subscriber error_sub =  
		nh.subscribe<std_msgs::Bool>("error", 1, error_callback);
	ros::spin();
}


void error_callback(msg){
	
	msg.imu0
	msg.imu1
	msg.gps
	msg.camera0
	msg.camera1
	
	enum slam_positioner {SLAM1, SLAM2, IMU}

	if(msg.gps){
		//GPS + SLAM
	}
	if(msg.camera1 && msg.camera0){
		positioner=SLAM1
	}
	else if(msg.camera1 && !msg.camera0 || !msg.camera1 && msg.camera0){
		positioner=SLAM2
	}
	else if(!msg.camera1 && !msg.camera0){
		positioner=IMU
		//Maybe use IMU / optical flow
	}

}