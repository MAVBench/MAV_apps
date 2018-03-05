#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <phoenix_msg/error.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>

//monitors IMU, camera, gps for errors, alerts subscribers

static tf::TransformListener listener;
static ros::Publisher error_pub;
static phoenix_msg::error current_msg;

void camera_l_timer_callback(const boost::system::error_code& e);
void camera_r_timer_callback(const boost::system::error_code& e);
void imu_0_timer_callback(const boost::system::error_code& e);
void gps_timer_callback(const boost::system::error_code& e);

void imu_0_sub_callback(const sensor_msgs::Imu::ConstPtr&);
void camera_l_sub_callback(const sensor_msgs::ImageConstPtr&);
void camera_r_sub_callback(const sensor_msgs::ImageConstPtr&);


//timer init
boost::asio::io_service io;
boost::asio::deadline_timer imu_0_timer(io, boost::posix_time::seconds(5));
boost::asio::deadline_timer camera_l_timer(io, boost::posix_time::seconds(5));
boost::asio::deadline_timer camera_r_timer(io, boost::posix_time::seconds(5));
boost::asio::deadline_timer gps_timer(io, boost::posix_time::seconds(5));

int main(int argc, char **argv)
{
    //ros node init
    ros::init(argc, argv, "error_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    tf::TransformListener tfListen;
        
    ros::Subscriber imu_0_sub = nh.subscribe("imu_topic", 1, imu_0_sub_callback);
    image_transport::Subscriber camera_r_sub = it.subscribe("/Airsim/right/image_raw", 1, camera_r_sub_callback);

    error_pub = nh.advertise<phoenix_msg::error>("error", 1000);
    
    //start timers
    //gps_timer.async_wait(gps_timer_callback);
    camera_l_timer.async_wait(camera_l_timer_callback);
    camera_r_timer.async_wait(camera_r_timer_callback);
    imu_0_timer.async_wait(imu_0_timer_callback);

    io.run();
    ros::Rate r(10);
    while(ros::ok()){
        ros::Time now = ros::Time::now();
        tf::StampedTransform transform;

        try {
            tfListen.waitForTransform("/world", "/gps", ros::Time::now(), ros::Duration(2.0));
            current_msg.gps = 1;
        } catch (tf::TransformException ex) {
            current_msg.gps = 0;
        }
        
        error_pub.publish(current_msg);
        ros::spinOnce();
    }
}

void camera_l_timer_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.camera_left = 0;
    camera_l_timer.async_wait(camera_l_timer_callback);
}

void camera_r_timer_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.camera_right = 0;
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
    imu_0_timer.async_wait(imu_0_timer_callback);
}

void camera_l_sub_callback(const sensor_msgs::ImageConstPtr& msg){
    camera_l_timer.cancel();
    camera_l_timer.async_wait(camera_l_timer_callback);
}

void camera_r_sub_callback(const sensor_msgs::ImageConstPtr& msg){
    camera_r_timer.cancel();
    camera_r_timer.async_wait(camera_r_timer_callback);
}

void imu_0_sub_callback(const sensor_msgs::Imu::ConstPtr& msg){
    imu_0_timer.cancel();
    imu_0_timer.async_wait(imu_0_timer_callback);
}
