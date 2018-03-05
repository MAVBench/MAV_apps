#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <phoenix_msg/error.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


//monitors IMU, camera, gps for errors, alerts subscribers

static tf::TransformListener listener;
static ros::Publisher errorPub;

void camera_timer_callback(void);
void imu_timer_callback(void);
void gps_timer_callback(void);

int main(int argc, char **argv)
{
    //ros node init
    ros::init(argc, argv, "error_detector");
    ros::NodeHandle nh;
    
    //timer init
    boost::asio::io_context io;
    boost::asio::deadline_timer gpsTimer(io, boost::posix_time::seconds(5));
    boost::asio::deadline_timer imuTimer(io, boost::posix_time::seconds(5));
    boost::asio::deadline_timer cameraTimer(io, boost::posix_time::seconds(5));    
    
    
    
    errorPub = nh.advertise<phonix_msg::error("error", 1000);
    //start timers
    gpsTimer.async_wait(gps_timer_callback());
    imuTimer.async_wait(im_timer_callback());
    cameraTimer.async_wait(camera_timer_callback());

    

    while(node.ok()){
        //look up
        io_run();
        ros::spin(1);
    }
}

void camera_timer_callback(){
    phoenix_msg::error error_msg;
    error_msg.camera = 1;
    errorPub.publish(msg);
    gpsTimer.async_wait(gps_timer_callback());
}

void imu_timer_callback(){
    phoenix_msg::error error_msg;
    error_msg.imu = 1;
    errorPub.publish(msg);
    imuTimer.async_wait(im_timer_callback());
}

void gps_timer_callback(){
    
    tf::StampedTransform transform;
    try{
        tfListen.lookupTransform("/world", "/gps", ros::Time(0), transform);
    }catch(tf::TransformException ex){
        phoenix_msg::error error_msg;
        error_msg.gps = 1;
        errorPub.publish(msg);
    }
    cameraTimer.async_wait(camera_timer_callback());
}

