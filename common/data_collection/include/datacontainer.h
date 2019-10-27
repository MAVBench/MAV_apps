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

template  <class T, class S> //T is the single data point and S is the difference
class DataContainer {
public:
	DataContainer();
	virtual ~DataContainer();
	void capture(std::string name, std::string mode, T data_value); //mode : {start, end}
	void findDataByName(std::string name, Data<T, S> **data);
	Data<T, S> *findDataByName(std::string name);
	void setStatsAndClear(); //setting all the statistical parameters
	std::string getStatsInString(std::vector <std::string> stats_name = {"avg", "std", "sample_size"}, std::string leading_space="\t\t");
	//void printDataLastValue(string name);
	std::vector<Data<T, S> > container;
};

#endif /* DATACONTAINER_H_ */
