#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <phoenix_msg/error.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
enum estimator_t {GPS, SLAM_LEFT, SLAM_RIGHT, IMU}; // SLAM_LEFT: stereo+IMU. SLAM_RIGHT: monocular+IMU
estimator_t estimator;

bool slam_lost = false;
bool slam_lost2 = false;

void error_callback(const phoenix_msg::error::ConstPtr& msg);

void slam_loss_callback (const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        ROS_ERROR("SLAM Lost!");
        slam_lost = true;
    }
}

void slam_loss2_callback (const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        ROS_ERROR("SLAM2 Lost!");
        slam_lost2 = true;
    }
}

// *** F:DN main function
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "position_estimator");
    ros::NodeHandle nh;
    
    //-----------------------------------------------------------------
	// *** F:DN variables	
	//-----------------------------------------------------------------
   
    // ros::ServiceClient SLAM_LEFT_start_stop_client = 
    //     nh.serviceClient<std_msgs::Bool>("/SLAM_LEFT/start_stop");
    // ros::ServiceClient SLAM_LEFT_inputs_client = 
    //     nh.serviceClient<phoenix_tb_msgs::error>("/SLAM_LEFT/good_sensors");

    // ros::ServiceClient SLAM_RIGHT_start_stop_client = 
    //     nh.serviceClient<std_msgs::Bool>("/SLAM_RIGHT/start_stop");
    // ros::ServiceClient SLAM_RIGHT_inputs_client = 
    //     nh.serviceClient<phoenix_tb_msgs::error>("/SLAM_RIGHT/good_sensors");
    
    ros::Subscriber error_sub =  
		nh.subscribe<phoenix_msg::error>("error", 1, error_callback);

	ros::Subscriber slam_lost_sub = 
		nh.subscribe<std_msgs::Bool>("/slam_lost", 1, slam_loss_callback);
	ros::Subscriber slam_lost2_sub = 
		nh.subscribe<std_msgs::Bool>("/slam_lost2", 1, slam_loss2_callback);

    ros::Publisher land_pub = nh.advertise<std_msgs::Bool>("/land_now", 1);

    tf::TransformListener tfListen;
    tf::TransformBroadcaster tfBroadcast;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        try {
            tf::StampedTransform transform;

            if (estimator == GPS) {
                // broadcast GPS as true transform
                tfListen.lookupTransform("world", "gps", ros::Time(0), transform);
            } else if (estimator == SLAM_LEFT) {
                // broacast SLAM_LEFT as true transform
                tfListen.lookupTransform("world", "vins_mono2", ros::Time(0), transform);
                // tfListen.lookupTransform("world", "ground_truth", ros::Time(0), transform);
            } else if (estimator == SLAM_RIGHT) {
                // broadcast SLAM_RIGHT as true transform
                tfListen.lookupTransform("world", "vins_mono", ros::Time(0), transform);
                // tfListen.lookupTransform("world", "ground_truth", ros::Time(0), transform);
            }
            else if (estimator == IMU) {
                // broadcast IMU as true transform
                ROS_ERROR("IMU estimator not available");
                tfListen.lookupTransform("world", "ground_truth", ros::Time(0), transform);

                std_msgs::Bool msg;
                msg.data = true;
                land_pub.publish(msg);
            }

            if (transform.frame_id_ == "world") {
                transform.child_frame_id_ = "drone";
                tfBroadcast.sendTransform(transform);
            } else {
                // ROS_ERROR("world not frame");
            }
        } catch (tf::TransformException& ex) {
            // ROS_ERROR("%s", ex.what());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}


void error_callback(const phoenix_msg::error::ConstPtr& msg) {
	/*
	msg.imu_0
	msg.imu_1
	msg.gps
	msg.camera0
	msg.camera1
	*/

	// For now, assume we only have one IMU

	static phoenix_msg::error last_msg;

    /*if (!msg->imu_0 && !msg->imu_1) {
        estimator = IMU;
    } else*/ if (msg->gps) {
		estimator = GPS;
	} else if (msg->camera_left && !slam_lost2) {
        estimator = SLAM_LEFT;
    } else if (msg->camera_right && !slam_lost) {
        estimator = SLAM_RIGHT;
    } else {
        estimator = IMU;
    }

    /*
	else if (msg->camera_left && msg->camera_right && msg->imu_0) {
		estimator = SLAM_LEFT;
	}
	else if ((!msg->camera_left || !msg->camera_right) && msg->imu_0){
		if(!msg->camera_left && msg->camera_right || msg->camera_left && !msg->camera_right) { // only one camera is working
			estimator = SLAM_RIGHT;
		}
		else{ // only IMU no camera
			estimator = IMU;
		}
	}
    */

    /*
	if (msg != last_msg) {
		if (estimator == GPS) {
			SLAM_LEFT_start_stop_client.call("stop");
			SLAM_RIGHT_start_stop_client.call("stop");
		} else if (estimator == SLAM_LEFT) {
			SLAM_LEFT_start_stop_client.call("start");
			SLAM_LEFT_inputs_client.call(msg);

			SLAM_RIGHT_start_stop_client.call("stop");
		} else if (estimator == SLAM_RIGHT) {
			SLAM_RIGHT_start_stop_client.call("start");
			SLAM_RIGHT_inputs_client.call(msg);

			SLAM_LEFT_start_stop_client.call("stop");
		} else if (estimator == IMU) {
			SLAM_LEFT_start_stop_client.call("stop");
			SLAM_RIGHT_start_stop_client.call("stop");
		}
	}
    */

	last_msg = *msg;
}

