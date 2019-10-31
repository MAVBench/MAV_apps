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
#include <assert.h>
using namespace std;
#define PRINT_VALS 1
int main() {
	cout << "!!s!Hello World!!!" << endl; // prints !!!Hello World!!!
	int input_cnt = 15;
	int vec_size = 20;
	data_test_bench test_bench_1;

	// simply writing into the data with deterministic values
	float result = test_bench_1.data_test_deterministic(input_cnt, vec_size);
	assert(result == (float)input_cnt*(input_cnt+1)/(2*input_cnt));

	// simply writing into the data with random values
	test_bench_1.data_test_random(input_cnt, vec_size);

	// overflowing the values
	vec_size = 9;
	test_bench_1.data_test_random(input_cnt, vec_size);

	test_bench_1.datacontainer_test_1();

	test_bench_1.datacontainer_test_3();

	test_bench_1.datacontainer_test_4();

	test_bench_1.datacontainer_test_5();

	return 0;
}
