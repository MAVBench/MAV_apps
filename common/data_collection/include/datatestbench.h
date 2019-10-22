/*
 * datatestbench.h
 *
 *  Created on: Oct 20, 2019
 *      Author: reddi-rtx
 */

#ifndef DATATESTBENCH_H_
#define DATATESTBENCH_H_

class data_test_bench {
public:
	data_test_bench();
	virtual ~data_test_bench();
	void data_test_1(int input_cnt, int vec_size);  //writing into the vectors, and getting avg and std
	void datacontainer_test_1();
	void datacontainer_test_2();
};

#endif /* DATATESTBENCH_H_ */
