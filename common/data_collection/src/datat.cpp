/*
 * datat.cpp
 *
 *  Created on: Oct 19, 2019
 *      Author: reddi-rtx
 */

#include "datat.h"
#include <cstdint>
#include <cmath>
using namespace std;

template <class T, class S>
Data<T, S>::Data(string name, float max_duration , int sample_size_per_window){
	// TODO Auto-generated constructor stub
	this->data_key_name = name;
	this->sample_size_per_window = sample_size_per_window;
	//this->current_index_to_use = 0;
	//this->avg = nan("");
	//this->std = nan("");

	this->value_statistics.push_back(std::make_tuple("std", vector<double> {}));
	this->value_statistics.push_back(std::make_tuple("avg", vector<double> {}));
	this->value_statistics.push_back(std::make_tuple("sample_size", vector<double> {}));

	//this->values.reserve(sample_size_per_window);
}

// setters
// self explanatory

template <class T, class S>
double Data<T, S>::calcAvg(vector<double> data){
	double total = 0;
	for (auto &it : data){
			total += it;
	}
	double avg = double (total)/this->values.size();
	return avg;
}

template <class T, class S>
double Data<T, S>::calcAvg(vector<float> data){
	double total = 0;
	for (auto &it : data){
			total += it;
	}
	double avg = double (total)/this->values.size();
	return avg;
}


#ifdef ROS
//template specialization
template <>
double Data<ros::Time, ros::Duration>::calcAvg(vector<ros::Duration>){
	double total = 0;
	for (auto &it : this->values){
		total +=  it.toSec();
	}
	double avg= double(total)/this->values.size();
	return avg;
}
#endif


std::ostream& operator<< (std::ostream &out, const Data<float, float> &data){
		out<< data.data_key_name<<" values: ";
		for_each(data.values.begin(), data.values.end(), [&](float data_value){out<<data_value<<",";});
		return out;
}

#ifdef ROS
	std::ostream& operator<< (std::ostream &out, const Data<ros::Time, ros::Duration> &data){//, const Data<ros::Time, ros::Duration> &data) {
		out<< data.data_key_name<<" values: ";
		for_each(data.values.begin(), data.values.end(), [&](ros::Duration data_value){out<<data_value.toSec()<<",";});
		return out;
	}
#endif



//
template <class T, class S>
double Data<T, S>::calcStd(vector<double> data){
	double var_sqrd = 0;
	for (auto &it: data){
		var_sqrd += pow(it - this->calcAvg(data), 2);
	}
	return sqrt(var_sqrd/(this->values.size()-1));
}

template <class T, class S>
double Data<T, S>::calcStd(vector<float> data){
	double var_sqrd = 0;
	for (auto &it: data){
		var_sqrd += pow(it - this->calcAvg(data), 2);
	}
	return sqrt(var_sqrd/(this->values.size()-1));
}
#ifdef ROS
template <>
double Data<ros::Time, ros::Duration>::calcStd(vector<ros::Duration> data){
	double var_sqrd = 0;
	for (auto &it: data){
		var_sqrd += (double) pow(it.toSec() - this->calcAvg(data), 2);

	}
	return sqrt(var_sqrd/(this->values.size()-1));
}
#endif


template <class T, class S>
double Data<T, S>::calcStat(string stat_name){
	if (stat_name == "avg"){
		return this->calcAvg(this->values);
	}
	if (stat_name == "std"){
		return this->calcStd(this->values);
	}
	if (stat_name == "sample_size"){
		return (this->values.size());
	}
}

//to avoid double counting we clear the contents, once we set the stats (avg, std)
template <class T, class S>
void Data<T, S>::setStatsAndClear(){
	if (this->values.size() < 1) { //making sure that we don't add stats if there is no data
		return;
	}
	for (auto &el: this->value_statistics){
		double value = this->calcStat(std::get<0>(el));
		std::get<1>(el).push_back(value);
	}

#ifdef PRINT_VALS
	cout<<*this<<endl;
#endif
	this->values.clear();
}

/*
template <class T, class S>
calcStat Data<T, S>::return_func(string name){
	if (name == "std") {
		return calcStd;
	}else if (name == "avg") {
		return calcAvg;
	}else if (name =="sample_size"){

	}
}
*/

template <class T, class S>
string Data<T, S>::getStatInString(string stat_name){
	string results = "\"" + stat_name+ + "\"" + ":";
	vector<double>* stat_values = this->getStat(stat_name);
	if (!stat_values) { return string("no_value_captured");}
	for (auto &el: *stat_values){
		results += std::to_string(el) + ",";
	}
	return results;
}

//to avoid double counting we clear the contents, once we set the stats (avg, std)
template <class T, class S>
string Data<T, S>::getStatsInString(vector<string> stats_name, string leading_spaces){
	string results = "" + leading_spaces;
	for (const auto &el: stats_name){
		results +=  getStatInString(el) + "		";
	}
	return results;
}


/*
template <class T, class S>
void Data<T, S>::setStats(){
	if (this->values.size() < 2) { //making sure that we don't end up with inf std and avg with no value
		return;
	}

	this->std_of_values.push_back(this->calcStd()); //update
	this->avg_of_values.push_back(this->calcAvg());
	this->sample_size.push_back(this->values.size());
}
*/
//getters
// self explanatory
/*
template <class T, class S>
float Data<T, S>::get_avg(){
	return this->avg;
}

// self explanatory
template <class T, class S>
float Data<T, S>::get_std(){
	return this->std;
}
*/


template <class T, class S>
vector<double>* Data<T, S>::getStat(string stat_name){
	for (auto &el: this->value_statistics){
		if (std::get<0>(el) == stat_name){
			auto *stat_values= &std::get<1>(el);
			if (stat_values->size() == 0) {
				return nullptr;
			}
			return stat_values;
		}
	}
	return nullptr;
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
	return this->values.size();
	/*
	if(this->values.size() >= this->sample_size_per_window){
			return this->values.size() - 1 ;
		}else {
			return this->values.size();
		}
	*/
}

template <class T, class S>
void Data<T, S>::capture_end(T time){
	this->current_end_time = time;
	S diff = (this->current_end_time) - (this->current_start_time);

	if (this->values.size() == this->sample_size_per_window){
		this->setStatsAndClear(); //calculate stats;
	}

	this->values.push_back(diff);
	//this->current_index_to_use = this->assign_next_index();
}





template <class T, class S>
Data<T, S>::~Data() {
	// TODO Auto-generated destructor stub
}

template class Data<float, float>;
#ifdef ROS
template class Data<ros::Time, ros::Duration>;
#endif

