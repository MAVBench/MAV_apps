/*
 * datat.cpp
 *
 *  Created on: Oct 19, 2019
 *      Author: reddi-rtx
 */

#include "datat.h"
#include <cstdint>
#include <cmath>

template <class T, class S>
Data<T, S>::Data(string name, float max_duration , int max_samples_to_collect){
	// TODO Auto-generated constructor stub
	this->data_key_name = name;
	this->max_samples_to_collect = max_samples_to_collect;
	this->current_index_to_use = 0;
	this->avg = nan("");
	this->std = nan("");
	//this->values.reserve(max_samples_to_collect);
}

// setters
// self explanatory
template <class T, class S>
void Data<T, S>::set_avg(){
	float total = 0;
	for (auto it : this->values){
//			auto it = this->values.begin(); it != this->values.end(); ++it){
			total += it;
	}
	this->avg = float(total)/this->values.size();
}

//template specialization
template <>
void Data<ros::Time, ros::Duration>::set_avg(){
	double total = 0;
	for (auto it : this->values){
		total +=  it.toSec();
	}
	this->avg = double(total)/this->values.size();
}


// self explanatory
template <class T, class S>
void Data<T, S>::set_std(){
	double var_sqrd = 0;
	for (auto it: this->values){
		var_sqrd += pow(it - this->get_avg(), 2);
	}
	this->std = sqrt(var_sqrd/(this->values.size()-1));
}

template <>
void Data<ros::Time, ros::Duration>::set_std(){
	double var_sqrd = 0;
	for (auto it: this->values){
		var_sqrd += (double) pow(it.toSec() - this->get_avg(), 2);

	}
	this->std = sqrt(var_sqrd/(this->values.size()-1));
}



template <class T, class S>
void Data<T, S>::setStats(){
	this->set_avg();
	this->set_std();
}


//getters
// self explanatory
template <class T, class S>
float Data<T, S>::get_avg(){
	return this->avg;
}

// self explanatory
template <class T, class S>
float Data<T, S>::get_std(){
	return this->std;
}

// capture time
template <class T, class S>
void Data<T, S>::capture(T time, string mode){
	if (mode == "start") {
		this->current_start_time = time;
	}
	else {
		this->capture_end(time);
	}
}

template <class T, class S>
int Data<T, S>::assign_next_index(){
		if(this->values.size() >= this->max_samples_to_collect){
			return this->values.size() - 1 ;
		}else {
			return this->values.size();
		}
}

template <class T, class S>
void Data<T, S>::capture_end(T time){
	this->current_end_time = time;
	S diff = (this->current_end_time) - (this->current_start_time);

	if (this->current_index_to_use == this->values.size()){
		this->values.push_back(diff);
	}else{ //o.w rewrite
		this->values.at(this->current_index_to_use) = diff;
	}
	this->current_index_to_use = this->assign_next_index();
}



template <class T, class S>
Data<T, S>::~Data() {
	// TODO Auto-generated destructor stub
}

template class Data<float, float>;
template class Data<ros::Time, ros::Duration>;
