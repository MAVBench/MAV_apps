#include <ros/ros.h>
#include <phoenix_msg/error.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
enum estimator_t {GPS, SLAM_All, SLAM_MI, IMU}; // SLAM_All: stereo+IMU. SLAM_MI: monocular+IMU

void error_callback(const phoenix_msg::error::ConstPtr& msg);

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "position_estimator");
    ros::NodeHandle nh;
    
    //-----------------------------------------------------------------
	// *** F:DN variables	
	//-----------------------------------------------------------------
   
    // ros::ServiceClient SLAM_All_start_stop_client = 
    //     nh.serviceClient<std_msgs::Bool>("/SLAM_All/start_stop");
    // ros::ServiceClient SLAM_All_inputs_client = 
    //     nh.serviceClient<phoenix_tb_msgs::error>("/SLAM_All/good_sensors");

    // ros::ServiceClient SLAM_MI_start_stop_client = 
    //     nh.serviceClient<std_msgs::Bool>("/SLAM_MI/start_stop");
    // ros::ServiceClient SLAM_MI_inputs_client = 
    //     nh.serviceClient<phoenix_tb_msgs::error>("/SLAM_MI/good_sensors");
    
    ros::Subscriber error_sub =  
		nh.subscribe<phoenix_msg::error>("error", 1, error_callback);

	ros::spin();
}


void error_callback(const phoenix_msg::error::ConstPtr& msg) {
	/*
	msg.imu0
	msg.imu1
	msg.gps
	msg.camera0
	msg.camera1
	*/

	// For now, assume we only have one IMU

	estimator_t estimator;

	static phoenix_msg::error last_msg;

	if(msg->gps) {
		estimator = GPS;
	}
	else if (msg->camera_left && msg->camera_right && msg->imu_0) {
		estimator = SLAM_All;
	}
	else if ((!msg->camera_left || !msg->camera_right) && msg->imu_0){
		if(!msg->camera_left && msg->camera_right || msg->camera_left && !msg->camera_right) { // only one camera is working
			estimator = SLAM_MI;
		}
		else{ // only IMU no camera
			estimator = IMU;
		}
	}

    static tf::TransformListener tfListen;
    static tf::TransformBroadcaster tfBroadcast;
    try {
        tf::StampedTransform transform;

        if (estimator == GPS) {
            // broadcast GPS as true transform
            tfListen.lookupTransform("world", "gps", ros::Time(0), transform);
        } else if (estimator == SLAM_All) {
            // broacast SLAM_All as true transform
            tfListen.lookupTransform("world", "orb_slam2_rgbd", ros::Time(0), transform);
        } else if (estimator == SLAM_MI) {
            // broadcast SLAM_MI as true transform
            tfListen.lookupTransform("world", "orb_slam2_rgbd", ros::Time(0), transform);
        }
        /*
        else if (estimator == IMU) {
            // broadcast IMU as true transform
        }
        */

        transform.child_frame_id_ = "drone";
        tfBroadcast.sendTransform(transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }

    /*
	if (msg != last_msg) {
		if (estimator == GPS) {
			SLAM_All_start_stop_client.call("stop");
			SLAM_MI_start_stop_client.call("stop");
		} else if (estimator == SLAM_All) {
			SLAM_All_start_stop_client.call("start");
			SLAM_All_inputs_client.call(msg);

			SLAM_MI_start_stop_client.call("stop");
		} else if (estimator == SLAM_MI) {
			SLAM_MI_start_stop_client.call("start");
			SLAM_MI_inputs_client.call(msg);

			SLAM_All_start_stop_client.call("stop");
		} else if (estimator == IMU) {
			SLAM_All_start_stop_client.call("stop");
			SLAM_MI_start_stop_client.call("stop");
		}
	}
    */

	last_msg = *msg;
}

