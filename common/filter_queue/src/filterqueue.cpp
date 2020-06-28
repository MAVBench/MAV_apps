/*
 * datacontainer.h
 *
 *  Created on: Oct 20, 2019
 *      Author: reddi-rtx
 */


#include <iostream>
#include <algorithm>
#include "filterqueue.h"
//using namespace std;

FilterQueue::FilterQueue(int max_size) {
       this->max_size = max_size;
}

void FilterQueue::push(double value) {
    if (this->storage.size() > this->max_size) {
        this->storage.pop_front();
        this->storage.push_back(value);
    }else{
        this->storage.push_back(value); 
    }
}

double FilterQueue::reduce (string mode) {
    if (mode == "min") {
        return *std::min_element(this->storage.begin(), this->storage.end());
    }
    if (mode == "max") {
        return *std::max_element(this->storage.begin(), this->storage.end());
    }else if (mode =="avg") {
        double total = 0; 
        for (auto el : this->storage) {
            total += el;
        }
        return total/this->storage.size();
    }else{
        std::cout<<"mode" << mode<< " is not defined"<<endl; 
    }
}
