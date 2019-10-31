/*
 * datacontainer.h
 *
 *  Created on: Oct 20, 2019
 *      Author: reddi-rtx
 */


#ifndef DATACONTAINER_H_
#define DATACONTAINER_H_

#include "datat.h"
#include <iostream>
//using namespace std;

class DataContainer {
public:
	DataContainer();
	virtual ~DataContainer();


	std::vector<Data>::iterator find_if_name_equal(std::string name);
	void capture(std::string name, std::string mode, double data_value); //mode : {start, end}

	bool static data_name_equal(Data& data1);

#ifdef ROS
	void capture(std::string name, std::string mode, ros::Time data_value); //mode : {start, end}
#endif

	void findDataByName(std::string name, Data **data);
	Data *findDataByName(std::string name);
	void setStatsAndClear(); //setting all the statistical parameters
	std::string getStatsInString(std::vector <std::string> stats_name = {"avg", "std", "sample_size"}, std::string leading_space="\t\t");
	//void printDataLastValue(string name);
	std::vector<Data> container;
};

#endif /* DATACONTAINER_H_ */
