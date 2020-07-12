#include "ros/ros.h"
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "template_library.hpp"
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include <chrono>
#include <thread>
#include <fstream>
#include "Drone.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <time.h>
#include "common.h"
#include <cstring>
#include <string>

using namespace std;

void output_steps_taken_to_a_file(Drone *drone){
	ofstream recorded_steps_file;
	recorded_steps_file.open("recorded_steps.txt");
	recorded_steps_file <<"#!/bin/bash"<<endl;
	for (auto el: drone->all_steps_taken) {
		if (el.vx == 0 && el.vy == 0 && el.vz == 0 && el.yaw != 0){
			recorded_steps_file <<"echo y"<<" "<< el.yaw<<endl;
		} else{
			recorded_steps_file <<"echo f"<<" "<< el.vx << " " << el.vy<< " " << el.vz << " " << el.duration <<std::endl;
		}
		recorded_steps_file <<"sleep"<< " " << el.duration<<endl;
	}
	recorded_steps_file.close();
}



geometry_msgs::Twist drone_vel;
double drone_vel_mag;
ros::Time start_deceleration_time;



bool control_drone_for_motion_models(Drone& drone)
{
	cout << "Initialize drone:\n";
	cout << "\ta: arm\n";
	cout << "\td: disarm\n";
	cout << "\tt h: takeoff to h m\n";
	cout << "\tl: land\n";
	cout << "\tf x y z d: fly at (x,y,z) m/s for d s\n";
	cout << "\tfz x y z d: fly at (x,y) m/s for d s, holding height z m\n";
	cout << "\ty x: set yaw to x\n";
	cout << "\tyz x z: set yaw to x degrees, holding height z m\n";
	cout << "\tp: print pitch, roll, yaw, height\n";
	cout << "\tb: print battery stats\n";
	cout << "\tc: complete drone setup and continue\n";
	cout << "\ts: sleep for 5 seconds\n";
	cout << "\tr: rotate slowly\n";
    cout << "\tCtrl-c/q: quit\n";

	std::string cmd("");

	while(cmd != "c") {
		cin >> cmd;
        if (cmd == "q") {
          //LOG_TIME(package_delivery);
          cout << "bye~" << endl;
          ros::shutdown();
          exit(0);
          return true;
        }


        if (cmd == "a") {
            drone.arm();
        }
        else if (cmd == "d") {
			drone.disarm();
		} else if (cmd == "t") {
			double height;
			cin >> height;
			if (!drone.takeoff(height))
                return false; // Take-off has failed
		} else if (cmd == "l") {
			drone.land();
		} else if (cmd == "f") {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity(x, y, z, YAW_UNCHANGED, d);
        } else if (cmd == "fz") {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity_at_z(x, y, z, YAW_UNCHANGED, d);
		} else if (cmd == "y") {
			double x;
			cin >> x;
			drone.set_yaw(x);
		} else if (cmd == "yz") {
			double y, z;
			cin >> y >> z;
			drone.set_yaw_at_z(y, z);
		} else if (cmd == "s"){
			std::vector<double> vel_mag_vec;
			vel_mag_vec.reserve(10000);
			bool vel_small = false;
			bool vel_neg = false;
			drone_vel = drone.velocity();
			   drone_vel_mag= (double) calc_vec_magnitude(drone_vel.linear.x, drone_vel.linear.y, drone_vel.linear.z);
			   drone.fly_velocity(0, -10, 0, YAW_UNCHANGED, 3);
			   start_deceleration_time = ros::Time::now();
			   	while(true){
			   		drone_vel = drone.velocity();
			   		double cur_vel_mag = (double) calc_vec_magnitude(drone_vel.linear.x, drone_vel.linear.y, drone_vel.linear.z);
			   		vel_mag_vec.push_back(cur_vel_mag);
			   		if (cur_vel_mag < .1){
			   			drone.fly_velocity(0, 0, 0, YAW_UNCHANGED, 10);
			   			ROS_INFO_STREAM("----------------------------deceleration time "<<(ros::Time::now() - start_deceleration_time).toSec()<< " for velocity:"  << drone_vel_mag);
			   			vel_small = true;
			   		}
			   		if (drone_vel.linear.y<0){
			   			drone.fly_velocity(0, 0, 0, YAW_UNCHANGED, 10);
			   			ROS_INFO_STREAM("----------------------------deceleration time based onvel "<<(ros::Time::now() - start_deceleration_time).toSec()<< " for velocity:"  << drone_vel_mag);
			   			vel_neg = true;
			   		}
			   		if (vel_small || vel_neg){
			   			break;
			   		}
			   	}
			   	for (auto el: vel_mag_vec){
			   		cout<<el<<" -- ";
			   	}
			   	exit(0);
		}else if(cmd == "p"){
			while(true){
				drone_vel = drone.velocity();
				double cur_vel_mag = (double) calc_vec_magnitude(drone_vel.linear.x, drone_vel.linear.y, drone_vel.linear.z);
				if (cur_vel_mag < .1){
					break;
				}
			}
			ROS_INFO_STREAM("deceleration time "<<(ros::Time::now() - start_deceleration_time).toSec()<< " for velocity:"  << drone_vel_mag);
			exit(0);
		}
       else if (cmd != "c") {
			cout << "Unknown command" << endl;
            // ros::shutdown();
            // exit(0);
		}
	}
    return true;
}



bool control_drone(Drone& drone)
{
	cout << "Initialize drone:\n";
	cout << "\ta: arm\n";
	cout << "\td: disarm\n";
	cout << "\tt h: takeoff to h m\n";
	cout << "\tl: land\n";
	cout << "\tf x y z d: fly at (x,y,z) m/s for d s\n";
	cout << "\tfz x y z d: fly at (x,y) m/s for d s, holding height z m\n";
	cout << "\ty x: set yaw to x\n";
	cout << "\tyz x z: set yaw to x degrees, holding height z m\n";
	cout << "\tp: print pitch, roll, yaw, height\n";
	cout << "\tb: print battery stats\n";
	cout << "\tc: complete drone setup and continue\n";
	cout << "\ts: sleep for 5 seconds\n";
	cout << "\tr: rotate slowly\n";
    cout << "\tCtrl-c/q: quit\n";

	std::string cmd("");

	while(cmd != "c") {
		cin >> cmd;
        if (cmd == "q") {
          //LOG_TIME(package_delivery);
          cout << "bye~" << endl;
          ros::shutdown();
          exit(0);
          return true;
        }

	    
        if (cmd == "a") {
            drone.arm();
        } else if (cmd == "s") {
			double t;
			cin >> t;
            sleep(t);
		} else if (cmd == "d") {
			drone.disarm();
		} else if (cmd == "t") {
			double height;
			cin >> height;
			if (!drone.takeoff(height))
                return false; // Take-off has failed
		} else if (cmd == "l") {
			drone.land();
		} else if (cmd == "f") {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity(x, y, z, YAW_UNCHANGED, d);
        } else if (cmd == "fz") {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity_at_z(x, y, z, YAW_UNCHANGED, d);
		} else if (cmd == "y") {
			double x;
			cin >> x;
			drone.set_yaw(x);
		} else if (cmd == "yz") {
			double y, z;
			cin >> y >> z;
			drone.set_yaw_at_z(y, z);
		} else if (cmd == "p") {
			auto pos = drone.pose().position;
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " pos: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
        } else if (cmd == "b") {
            auto flight_stats = drone.getFlightStats();
            cout << "energy consumed: " << flight_stats.energy_consumed << endl;
        } else if (cmd == "r") {
            spin_around(drone); 
        } else if (cmd == "r"){
        	output_steps_taken_to_a_file(&drone);
        }
        else if (cmd != "c") {
			cout << "Unknown command" << endl;
            // ros::shutdown();
            // exit(0);
		}
	}

    // Print flight summary at start of execution
    std::string fname;
    if (ros::param::get("/stats_file_addr", fname)) {
        //update_stats_file(fname, "\nFlightSummaryStart: ");
        //output_flight_summary(drone, fname);
    } else {
        ROS_ERROR("No stats_file found");
    }

    return true;
}


/*
void control_drone(Drone& drone)
{
	cout << "Initialize drone:\n";
	cout << "\ta: arm\n";
	cout << "\td: disarm\n";
	cout << "\tt h: takeoff to h m\n";
	cout << "\tl: land\n";
	cout << "\tf x y z d: fly at (x,y,z) m/s for d s\n";
	cout << "\ty x: set yaw to x\n";
	cout << "\tp: print pitch, roll, yaw, height\n";
	cout << "\tc: complete drone setup and continue\n";
	cout << "\tr: rotate slowlyd\n";
    cout << "\tCtrl-c: quit\n";

	std::string cmd("");

	while(cmd != "c") {
		cin >> cmd;

	    if (cmd == "a") {
	        drone.arm();
		} else if (cmd == "d") {
			drone.disarm();
		} else if (cmd == "t") {
			double height;
			cin >> height;
			cin.ignore(numeric_limits<streamsize>::max(), '\n');
			drone.takeoff(height);
		} else if (cmd == "l") {
			drone.land();
		} else if (cmd == "f") {
			double x,y,z,d;
			cin >> x >> y >> z >> d;
			drone.fly_velocity(x, y, z, d);
		} else if (cmd == "y") {
			double x;
			cin >> x;
			drone.set_yaw(x);
		} else if (cmd == "p") {
			auto pos = drone.gps();
			cout << "pitch: " << drone.get_pitch() << " roll: " << drone.get_roll() << " yaw: " << drone.get_yaw() << " pos: " << pos.x << ", " << pos.y << ", " << pos.z << endl;
        } else if (cmd == "r") {
            spin_around(drone); 
        }else if (cmd != "c") {
			cout << "Unknown command" << endl;
		}
	}
}
*/

