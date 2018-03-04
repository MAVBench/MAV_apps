#include <ros/ros.h>


void Start(){

}

void Stop(){

}

void startStop(phoenix::start_stop::Request &req){
	//start the SLAM algorithm running
	if(*req == "start"){
		Start();
	}
	else if(*req =="stop"){
		Stop();
	}
	else{
		// ??? invalid request
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "start_stop");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("start_stop", startStop);

	//need to read in the "start" or "stop" message and write those functions

	return 0;
}