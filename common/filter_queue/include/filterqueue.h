/*
 * datacontainer.h
 *
 *  Created on: Oct 20, 2019
 *      Author: reddi-rtx
 */


#ifndef FILTERQUEUE_H
#define FILTERQUEUE_H
#include <list>
#include <iostream>
using namespace std;

class FilterQueue {
public:
	FilterQueue(int max_size);
	~FilterQueue();

    void push(double value);
    double reduce(string mode);
	std::list<double> storage;
    int max_size;	
};

#endif /* DATACONTAINER_H_ */
