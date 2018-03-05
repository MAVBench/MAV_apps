#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <phoenix_msg/error.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


//monitors IMU, camera, gps for errors, alerts subscribers

static tf::TransformListener listener;
static ros::Publisher errorPub;
static phoenix_msg::error current_msg;

void camera_timer_callback(const boost::system::error_code& e);
void imu_timer_callback(const boost::system::error_code& e);
void imu_callback(void);
void gps_timer_callback(const boost::system::error_code& e);

int main(int argc, char **argv)
{
    //ros node init
    ros::init(argc, argv, "error_detector");
    ros::NodeHandle nh;
    
    //timer init
    boost::asio::io_context io;
    boost::asio::deadline_timer imuTimer(io, boost::posix_time::seconds(5));
    boost::asio::deadline_timer cameraTimer0(io, boost::posix_time::seconds(5));    
    boost::asio::deadline_timer cameraTimer1(io, boost::posix_time::seconds(5));     
    
    ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>("imu_topic", 1, imu_callback);

    errorPub = nh.advertise<phoenix_msg::error>("error", 1000);
    //start timers
    gpsTimer.async_wait(gps_timer_callback);
    
    ros::Rate r(10);
    while(ros::ok()){
        io_run();
        ros::Time now = ros::Time:now();
        tf::StampedTransform transform;
        if(!tfListen.waitForTransform("/world", "/gps", ros::Time(0), transform)){ 
            phoenix_msg::error error_msg;
            current_msg.gps = 1;i
        }
        
        errorPub.publish(current_msg);
        ros::spinOnce();
    }
}

void camera_timer0_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.camera0 = 1;
    cameraTimer.async_wait(camera_timer0_callback);
}

void camera_timer1_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.camera1 = 1;
    cameraTimer1.async_wait(camera_timer1_callback);
}

void imu_callback(){
    imuTimer.cancel();
    imuTimer.async_wait(imu_timer_callback);
}

void imu_timer_callback(const boost::system::error_code& e){
    if(e) return; //timer canceled
    current_msg.imu = 1;
    imuTimer.async_wait(imu_timer_callback);
}


