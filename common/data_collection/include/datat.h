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


template <class T, class S>  //T is the input data that is captured, and S is the difference
class Data {
public:
	//function members
	Data(std::string my_name , float max_duration = 300, int sample_size_per_window = 300);
	~Data();
	// setters
	void capture(T Time, std::string mode);
	void capture_end(T Time);
	double calcAvg(std::vector<double>);
	double calcStd(std::vector<double>);
	double calcAvg(std::vector<float>);
	double calcStd(std::vector<float>);
#ifdef ROS
	double calcAvg(std::vector<ros::Duration>);
	double calcStd(std::vector<ros::Duration>);
#endif
	void setStatsAndClear();

	//getters
	double calcStat(std::string stat_name);
	std::string getStatsInString(std::vector<std::string> data_type = {"avg", "std", "sample_size"}, std::string leading_spaces="\t\t");
	std::string getStatInString(std::string stat_name);
	std::vector<double>* getStat(std::string statName);

    // getters
	int assign_next_index();

	// streams
	friend std::ostream& operator<< (std::ostream &out, const Data<float, float> &data);

#ifdef ROS
	friend std::ostream& operator<< (std::ostream &out, const Data<ros::Time, ros::Duration> &data);
#endif

    // variabels
	std::string data_key_name;
	std::vector<S> values;
	std::vector< std::tuple<std::string, std::vector<double> > > value_statistics; //vector of statistics


	int sample_size_per_window;
	T current_start_time; //the time that the start time was captured
	T current_end_time; //the time that the end time was captured
	int current_index_to_use; //current index to push_back the new value in
	float std, avg;
	//vector<float> values(sample_size_per_window);
}
;

//#include "datat.cpp"
#endif /* DATACOLLECTION_H_ */
