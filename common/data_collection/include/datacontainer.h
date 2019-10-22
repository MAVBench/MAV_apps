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
using namespace std;
template  <class T, class S> //T is the single data point and S is the difference
class DataContainer {
public:
	DataContainer();
	virtual ~DataContainer();
	void capture(string name, string mode, T data_value); //mode : {start, end}
	void findDataByName(string name, Data<T, S> **data);
	Data<T, S> *findDataByName(string name);
	void setStats(); //setting all the statistical parameters
	//void printDataLastValue(string name);
	vector<Data<T, S> > container;
};

#endif /* DATACONTAINER_H_ */
