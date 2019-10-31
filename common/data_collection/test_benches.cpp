/*
 * datatestbench.cpp
 *
 *  Created on: Oct 20, 2019
 *      Author: reddi-rtx
 */

#include <datatestbench.h>
#include <iostream>
#include "datat.h"
#include "datacontainer.h"
#include <algorithm>
#include <vector>
using namespace std;


data_test_bench::data_test_bench() {
	// TODO Auto-generated constructor stub

}

// populating the data with values from 1 to vec_size
int data_test_bench::data_test_deterministic(int input_cnt, int vec_size){
	string my_data_name = "my_data";
	Data my_data(my_data_name, 100, vec_size);

	double rand_val;
	for (int i = 0; i < input_cnt; i++){
		my_data.capture(0, "start");
		rand_val = i+1;
		my_data.capture(rand_val, "end");
	}
	my_data.setStatsAndClear();
	cout<<"data name" << my_data.data_key_name << my_data.getStatsInString(vector<string> {"avg", "std"}) << endl;
	return (int) my_data.getStat("avg")->back();
}


// populating the data with random values
void data_test_bench::data_test_random(int input_cnt, int vec_size){
	string my_data_name = "my_data";
	Data my_data(my_data_name, 100, vec_size);

	for (int i = 0; i < input_cnt; i++){
		double rand_val = rand() % 100;
		my_data.capture(rand_val, "start");
		rand_val = rand() % 100;
		my_data.capture(rand_val, "end");
	}
	my_data.setStatsAndClear();

	cout<<"data name" << my_data.data_key_name << my_data.getStatsInString(vector<string> {"avg", "std"}) << endl;
	my_data.setStatsAndClear();
	cout<<"data name" << my_data.data_key_name << my_data.getStatsInString(vector<string> {"avg", "std"}) << endl;
}


void data_test_bench::datacontainer_test_2() {
	// test 3: capturing end before start
	DataContainer myContainer;
	try {
		myContainer.capture("my_data_3", "end", 10);
	}catch(...) {
			//std::exception &e){

	std::cerr<<"fine";
	///std::cerr<<e.what();
	}
}

void data_test_bench::datacontainer_test_1(){

	// test 1 : instantiaing a container and writing data into it
	DataContainer myContainer;
	Data *data;
	myContainer.capture("my_data", "start", 10);
	myContainer.capture("my_data", "end", 25);
	myContainer.findDataByName("my_data", &data);
	data->setStatsAndClear();
	cout<< data->getStatsInString(vector<string> {"avg"});
	cout<< myContainer.getStatsInString(vector<string> {"std"})<< endl;

	// test 2 : writing two different data and checking their stats
	DataContainer  myContainer_2;
	int input_cnt = 10;
	double rand_val = 0;
	for (int i = 0; i < input_cnt; i++){
		rand_val = rand() % 100;
		myContainer_2.capture("first_data", "start", rand_val);
		rand_val = rand() % 100;
		myContainer_2.capture("first_data", "end", rand_val);
	}

	for (int i = 0; i < input_cnt; i++){
		rand_val = rand() % 100;
		myContainer_2.capture("second_data", "start", rand_val);
		rand_val = rand() % 100;
		myContainer_2.capture("second_data", "end", rand_val);
	}
	//for (auto el: myContainer_2.container) { cout<< el<<endl;}
	myContainer_2.setStatsAndClear();
	Data * my_data;
	myContainer_2.findDataByName("first_data", &my_data);
	cout<<myContainer_2.getStatsInString()<<endl;;

	//test 3: finding a data
//	myContainer_2.findDataByName("second_data", &my_data);
//	cout<<"second_data_avg"<< my_data->getStatsInString(vector<string>{"avg"});
}

// no end
void data_test_bench::datacontainer_test_3(){

	// test 1 : instantiaing a container and writing data into it
	DataContainer myContainer;
	Data *data;
	myContainer.capture("my_data", "start", 10);
	myContainer.capture("my_data", "start", 15);
	myContainer.findDataByName("my_data", &data);
	data->setStatsAndClear();
	if (data->getStat("avg")) {
		double blah = data->getStat("avg")->back();//vector<string> {"avg"});
	}
	cout<< data->getStatsInString();//vector<string> {"avg"});
	//test 3: finding a data
//	myContainer_2.findDataByName("second_data", &my_data);
//	cout<<"second_data_avg"<< my_data->getStatsInString(vector<string>{"avg"});
}

// testing the counters
void data_test_bench::datacontainer_test_4(){

	// test 1 : instantiaing a container and writing data into it
	DataContainer myContainer;
	for (int i=0; i < 10; i++){
		myContainer.capture("my_data", "counter", 0);
	}
	myContainer.setStatsAndClear();
	cout<<myContainer.getStatsInString()<<endl;;
}

// testing single values
void data_test_bench::datacontainer_test_5(){
	// test 1 : instantiaing a container and writing data into it
	DataContainer myContainer;
	Data *data;
	for (int i=0; i < 10; i++){
		myContainer.capture("my_data", "single", i*4);
	}
	myContainer.setStatsAndClear();
	cout<<myContainer.getStatsInString()<<endl;;
}





data_test_bench::~data_test_bench() {
	// TODO Auto-generated destructor stub
}

