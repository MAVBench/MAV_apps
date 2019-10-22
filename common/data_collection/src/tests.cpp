//============================================================================
// Name        : data_collection_2.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <datat.h>
#include "datatestbench.h"

using namespace std;



int main() {
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	int input_cnt = 15;
	int vec_size = 20;
	data_test_bench test_bench_1;
	test_bench_1.data_test_1(input_cnt, vec_size);

	vec_size = 9;
	test_bench_1.data_test_1(input_cnt, vec_size);

	test_bench_1.datacontainer_test_1();

	return 0;
}
