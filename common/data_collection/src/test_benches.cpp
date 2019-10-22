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
using namespace std;


data_test_bench::data_test_bench() {
	// TODO Auto-generated constructor stub

}

void data_test_bench::data_test_1(int input_cnt, int vec_size){
	string my_data_name = "my_data";
	Data<float, float> my_data(my_data_name, 100, vec_size);

	for (int i = 0; i < input_cnt; i++){
		float rand_val = rand() % 100;
		my_data.capture(rand_val, "start");
		rand_val = rand() % 100;
		my_data.capture(rand_val, "end");
	}
	my_data.setStats();
	for (auto el: my_data.values){
		cout<<el<< " ";
	}
	cout<<"my data's name"<< my_data.data_key_name << "avg:"<<my_data.get_avg() << "std:"<<my_data.get_std()<<endl;
}


void data_test_bench::datacontainer_test_2() {
	// test 3: capturing end before start
	DataContainer<float, float> myContainer;
	try {
		myContainer.capture(10, "my_data_3", "end");
	}catch(...) {
			//std::exception &e){

	std::cerr<<"fine";
	///std::cerr<<e.what();
	}
}

void data_test_bench::datacontainer_test_1(){

	// test 1 : instantiaing a container and writing data into it
	DataContainer<float, float> myContainer;
	Data<float, float> *data;
	myContainer.capture(10, "my_data", "start");
	myContainer.capture(20, "my_data", "end");
	myContainer.findDataByName("my_data", &data);
	data->setStats();
	cout<< data->get_avg() << endl;

	// test 2 :
	DataContainer<float, float>  myContainer_2;
	int input_cnt = 10;
	float rand_val = 0;
	for (int i = 0; i < input_cnt; i++){
		rand_val = rand() % 100;
		myContainer_2.capture(rand_val, "first_data", "start");
		rand_val = rand() % 100;
		myContainer_2.capture(rand_val, "first_data", "end");
	}

	for (int i = 0; i < input_cnt; i++){
		rand_val = rand() % 100;
		myContainer_2.capture(rand_val, "second_data", "start");
		rand_val = rand() % 100;
		myContainer_2.capture(rand_val, "second_data", "end");
	}

	myContainer_2.setStats();
	for (auto el: myContainer_2.container) {
		cout<<el.data_key_name<<" avg is: " << el.get_avg()<<endl;
	}

	//test 3: finding a data
	Data<float, float> * my_data;
	myContainer_2.findDataByName("second_data", &my_data);
	cout<<"second_data avg is :"<< my_data->get_avg();
}

data_test_bench::~data_test_bench() {
	// TODO Auto-generated destructor stub
}

