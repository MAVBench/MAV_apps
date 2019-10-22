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
#include<ros/ros.h>
using namespace std;

template <class T, class S>  //T is the input data that is captured, and S is the difference
class Data {
public:
	//function members
	Data(string my_name , float max_duration = 600, int max_samples_to_collect = 600);
	~Data();
	// setters
	void capture(T Time, string mode);
	void capture_end(T Time);

	void set_avg();
	void set_std();
	void setStats();

    // getters
	float get_avg();
	float get_std();
	int assign_next_index();

	//vars
	string data_key_name;
	vector<S> values;
	int max_samples_to_collect;
	T current_start_time; //the time that the start time was captured
	T current_end_time; //the time that the end time was captured
	int current_index_to_use; //current index to push_back the new value in
	float std, avg;
	//vector<float> values(max_samples_to_collect);
}
;

#endif /* DATACOLLECTION_H_ */
