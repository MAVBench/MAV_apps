/*
 * test_bench.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: reddi-rtx
 */
#include <iostream>
#include <vector>
#include "TimeBudgetter.h"
#include "data_types.h"
#include "data.h"
using namespace std;

void copy_velocity_data_to_traj(double velocity [], int dataSize, std::deque<multiDOFpoint> &traj){
//	for (auto &velocity_el: velocity) {
	for (int i =0; i < dataSize ; i++) {
		double each_val = sqrt(pow(velocity[i], 2.0)/3);
		multiDOFpoint point;
		point.vx = each_val;
		point.vy = each_val;
		point.vz = each_val;
		traj.push_back(point);
	}
}

void data(vector<double> &velocity_vec){
	for (auto &el: velocity_vec) {
		velocity_vec.push_back(el);
	}
}

// calc next sample
void test_1(){

	std::deque<multiDOFpoint> traj;
	vector<double> velocity_vec;
	int dataSize = sizeof(velocity_data)/sizeof(velocity_data[0]);
	copy_velocity_data_to_traj(velocity_data, dataSize, traj);

	double maxSensorRange = 25;
	vector<double> accelerationCoeffs = {.1439,.8016};
	double TimeIncr = .05;
	double maxVelocity = 10;
	TimeBudgetter timeBudgetter(maxSensorRange, maxVelocity, accelerationCoeffs, TimeIncr);
	double latency = 1.55;

	vector<double> timingSamples = timeBudgetter.calcSamplingTime(traj, latency);
	for (auto &sample: timingSamples) {
		cout<<sample<<", ";
	}
}

int main(){
	test_1();
}



