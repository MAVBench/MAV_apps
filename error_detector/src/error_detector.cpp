#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <phoenix_msg/error.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "cam-feat.hpp"
#include <thread>

//monitors IMU, camera, gps for errors, alerts subscribers

static ros::Publisher error_pub;
static phoenix_msg::error current_msg;
static tf::StampedTransform last_tr;
static Vector3 last_imu_accel;

int calc_max_dist(void);

void camera_l_timer_callback(const boost::system::error_code& e);
void camera_r_timer_callback(const boost::system::error_code& e);
void imu_0_timer_callback(const boost::system::error_code& e);
void gps_timer_callback(const boost::system::error_code& e);

void imu_0_sub_callback(const sensor_msgs::Imu::ConstPtr&);
void camera_l_sub_callback(const sensor_msgs::ImageConstPtr&);
void camera_r_sub_callback(const sensor_msgs::ImageConstPtr&);
void gps_sub_callback(const sensor_msgs::GPS::ConstPtr&

void monitor_transform(void);

//timer init
boost::asio::io_service io;
boost::asio::deadline_timer imu_0_timer(io, boost::posix_time::seconds(.5));
boost::asio::deadline_timer camera_l_timer(io, boost::posix_time::seconds(.5));
boost::asio::deadline_timer camera_r_timer(io, boost::posix_time::seconds(.5));
boost::asio::deadline_timer gps_timer(io, boost::posix_time::seconds(.5));



int main(int argc, char **argv)
{
    //ros node init
    ros::init(argc, argv, "error_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    std::cout << "Error Detector Node" << std::endl;


    ros::Subscriber imu_0_sub = nh.subscribe("imu_topic", 1, imu_0_sub_callback);
    image_transport::Subscriber camera_r_sub = it.subscribe("/Airsim/right/image_raw", 1, camera_r_sub_callback);

    error_pub = nh.advertise<phoenix_msg::error>("error", 1000);

    //start timers

    io.run();
    ros::Rate r(10);

    current_msg.gps          = 1;
    current_msg.imu_0        = 1;
    current_msg.camera_right = 1;

    // feature detection based camera fault detector
    
    //thread to monitor gps
    std::thread transformThread(monitor_transform);
    
    while(ros::ok()){
        error_pub.publish(current_msg);
        ros::spinOnce();
        r.sleep();
    }
    transformThread.join();
}

void monitor_transform(){
  
    tf::TransformListener tfListen;
    while(ros::ok()){
        ros::Time now = ros::Time::now();
        tf::StampedTransform transform;

        if (tfListen.waitForTransform("/world", "/gps", ros::Time::now(), ros::Duration(1.0))) {
            if (current_msg.gps == 0){
                tfListen.LookUpTransform("/world", "/gps", ros:Time::now(), transform);
                std::cout << "GPS data found" << std::endl;
                //compare old coordinates to new  coordinates
                //need velocity from  IMU
                int maxDist = calc_max_dist();
                int traveledDist = transform.getOrigin().distance();
                if(traveledDist>maxDist){
                     std::cout<<"GPS position not consistent with IMU acceleration"<<std::endl;
                }else{
                    //valid gps data
                    current_msg.gps =1;
                }
            }
        } else {
            if (current_msg.gps == 1)
                
                std::cout << "GPS data timed out" << std::endl;
            current_msg.gps = 0;
        }

        error_pub.publish(current_msg);
        ros::spinOnce();
        r.sleep();
    }
}

int calc_max_dist(){
   //TODO: check units of numbers
   if(last_imu_accel == NULL)
      //can't use imu accel to calc max dist
      return INT_MAX;
   int max_accel = last_imu_accel.distance();
   double time = .5; //picked 5 seconds, cause that is the max time btwn
                 //valid Imu msgs
   return max_accel * time * time; 
}

void camera_l_timer_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.camera_left = 0;
    std::cout << "Left camera timed out" << std::endl;
    camera_l_timer.async_wait(camera_l_timer_callback);
}

void camera_r_timer_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.camera_right = 0;
    std::cout << "Right camera timed out" << std::endl;
    camera_r_timer.async_wait(camera_r_timer_callback);
}

void gps_timer_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.gps = 0;
    gps_timer.async_wait(gps_timer_callback);
}

void imu_0_timer_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.imu_0 = 0;
    std::cout << "IMU 0 timed out" << std::endl;
    imu_0_timer.async_wait(imu_0_timer_callback);
}


uint8_t test_frame(cam_feat & cf, const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg);
    cv::Mat resized;
    cv::resize(cv_ptr->image, resized, cv::Size(80, 45));
    if (!cf.test_frame(resized))
        return 0;
    return 1;
}

cam_feat cf_l(20); // tentative 20 frames before consider camera to be faulty
void camera_l_sub_callback(const sensor_msgs::ImageConstPtr& msg){
    camera_l_timer.cancel();
    if (current_msg.camera_left == 0)
        std::cout << "Left camera data found" << std::endl;
    current_msg.camera_left = 1;
    camera_l_timer.async_wait(camera_l_timer_callback);
    
    current_msg.camera_left = test_frame(cf_l, msg);
}

cam_feat cf_r(20); // tentative 20 frames before consider camera to be faulty
void camera_r_sub_callback(const sensor_msgs::ImageConstPtr& msg){
    camera_r_timer.cancel();
    if (current_msg.camera_right == 0)
        std::cout << "Right camera data found" << std::endl;
    current_msg.camera_right = 1;
    camera_r_timer.async_wait(camera_r_timer_callback);

    current_msg.camera_right = test_frame(cf_r, msg);
}

void imu_0_sub_callback(const sensor_msgs::Imu::ConstPtr& msg){
    imu_0_timer.cancel();
    if (current_msg.imu_0 == 0)
        std::cout << "IMU data found" << std::endl;
    current_msg.imu_0 = 1;
    last_imu_accel = msg.linear_acceleration;
    imu_0_timer.async_wait(imu_0_timer_callback);
}
