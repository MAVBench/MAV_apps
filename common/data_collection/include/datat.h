/*
 * datat.h
 *
 *  Created on: Oct 19, 2019
 *      Author: reddi-rtx
 */

#ifndef DATAT_H_
#define DATAT_H_

/*
 * Datacollection.h
 *
 *  Created on: Oct 17, 2019
 *      Author: reddi-rtx
 */

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>

#ifdef ROS
#include<ros/ros.h>
#endif


class Data {
public:
	//function members
	Data(std::string my_name , double max_duration = 300, int sample_size_per_window = 300);
	~Data();
	// setters
	void capture(double Time, std::string mode);
#ifdef ROS
	void capture(ros::Time Time, std::string mode);
#endif
#ifdef ROS
	void capture_end(ros::Time Time);
#endif
	void capture_end(double Time);

	double calcAvg(std::vector<double>);
	double calcStd(std::vector<double>);
	void incr_counter();
	void setSingle(double time);

#ifdef ROS
	void setSingle(ros::Time time);
#endif
	void setStatsAndClear();

	//getters
	double calcStat(std::string stat_name);
	std::string getStatsInString(std::vector<std::string> data_type = {"avg", "std", "sample_size"}, std::string leading_spaces="\t\t");
	std::string getStatInString(std::string stat_name);
	std::vector<double>* getStat(std::string statName);
	int assign_next_index();

	// streams
	friend std::ostream& operator<< (std::ostream &out, const Data &data);

    // variabels
	std::string data_key_name;
	std::vector<double> values;
	std::vector< std::tuple<std::string, std::vector<double> > > value_statistics; //vector of statistics
	std::string mode;
	int sample_size_per_window;
	double current_start_time_double; //the time that the start time was captured
	double current_end_time_double; //the time that the end time was captured

	#ifdef ROS
	ros::Time current_start_time_time; //the time that the start time was captured
	ros::Time current_end_time_time; //the time that the end time was captured
#endif
	int current_index_to_use; //current index to push_back the new value in
	double std, avg;
}
;

//#include "datat.cpp"
#endif /* DATACOLLECTION_H_ */
