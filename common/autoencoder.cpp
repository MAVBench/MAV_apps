// Copyright 2016, Tobias Hermann.
// https://github.com/Dobiasd/frugally-deep
// Distributed under the MIT License.
// (See accompanying LICENSE file or at
//  https://opensource.org/licenses/MIT)



#include "autoencoder.h"
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

std::vector<float> Autoencoder::inference(std::vector<double>& tmp, int detect){
    /*auto result = model_all.predict(
        {fdeep::tensor(fdeep::tensor_shape(15),
        fdeep::float_vec{tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], tmp[9], tmp[10], tmp[11], tmp[12], tmp[13], tmp[14]})});
    return result[0].to_vector();*/
    /*auto result = model_all.predict(
        {fdeep::tensor(fdeep::tensor_shape(14),
        fdeep::float_vec{tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], tmp[9], tmp[10], tmp[11], tmp[12], tmp[13]})});
    return result[0].to_vector();*/
    //detect = 2 (only planning), 3(all), 4(only control), 5(planning and control) 6(envgen_part of planning and control)
    if(detect == 6){
        auto result = model_envgen.predict(
            {fdeep::tensor(fdeep::tensor_shape(11),
            fdeep::float_vec{tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], tmp[9], tmp[10]})});
        return result[0].to_vector();
    }
    else if(detect == 3){
        auto result = model_all.predict(
            {fdeep::tensor(fdeep::tensor_shape(13),
            fdeep::float_vec{tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], tmp[9], tmp[10], tmp[11], tmp[12]})});
        return result[0].to_vector();
    }
    else if(detect == 4 || detect == 5){
        auto result = model_control.predict(
            {fdeep::tensor(fdeep::tensor_shape(4),
            fdeep::float_vec{tmp[9], tmp[10], tmp[11], tmp[12]})});
        return result[0].to_vector();
    }
    
}
		
/*bool Autoencoder::inference(){
    const auto result = model.predict(
        {fdeep::tensor(fdeep::tensor_shape(static_cast<std::size_t>(11)),
        fdeep::float_vec{6.4160e-01,  6.3130e-01,  2.1730e-02, -3.5170e-02, -3.4610e-02,
       -1.1916e-03, -4.3577e-03, -4.2886e-03, -1.4767e-04,  0.0000e+00,
        0.0000e+00})});
    //std::cout << fdeep::show_tensors(result) << std::endl;
    return true;
}*/





