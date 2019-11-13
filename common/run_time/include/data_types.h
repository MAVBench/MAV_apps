/*
 * data_types.h
 *
 *  Created on: Nov 12, 2019
 *      Author: reddi-rtx
 */

#ifndef DATA_TYPES_H_
#define DATA_TYPES_H_
#include <iostream>
#include <algorithm>
#include <deque>

struct multiDOFpoint {
    double x, y, z;
    double vx, vy, vz;
    double ax, ay, az; // Currently, the acceleration values are ignored
    double yaw;
    bool blocking_yaw;
    double duration;
};
typedef std::deque<multiDOFpoint> trajectory_t;

#endif /* DATA_TYPES_H_ */
