// Copyright 2016, Tobias Hermann.
// https://github.com/Dobiasd/frugally-deep
// Distributed under the MIT License.
// (See accompanying LICENSE file or at
//  https://opensource.org/licenses/MIT)
#ifndef MAVBENCH_AUTOENCODER_H
#define MAVBENCH_AUTOENCODER_H

#include "fdeep.hpp"


// Standard headers
#include <string>
#include <tuple>



// MAVBench headers
//#include <mavbench_msgs/multiDOFtrajectory.h>
//#include <mavbench_msgs/future_collision.h>


#include <bitset>
#include <vector>

#include <fstream>
#include <iostream>

class Autoencoder {
public:
	const fdeep::model model_planning;
	const fdeep::model model_control;
	const fdeep::model model_all;
	const fdeep::model model_envgen;
	Autoencoder():
		model_planning(fdeep::load_model("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/common/autoencoder_planning.json")),
		model_control(fdeep::load_model("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/common/autoencoder_control.json")),
		model_envgen(fdeep::load_model("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/common/autoencoder_envgen.json")),
		model_all(fdeep::load_model("/home/yushun/Desktop/MAVBench_original/MAVBench_base/src/MAV_apps/common/autoencoder_sparse.json"))
	{}
	std::vector<float> inference(std::vector<double>&, int);



};

#endif


