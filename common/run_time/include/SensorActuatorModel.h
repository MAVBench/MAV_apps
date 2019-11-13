/*
 * SensorActuatorModel.h
 *
 *  Created on: Nov 12, 2019
 *      Author: reddi-rtx
 */


#ifndef SENSORACTUATORMODEL_H_
#define SENSORACTUATORMODEL_H_
#include <iostream>
#include <cmath>
#include <math.h> //Not sure about including math.h
#include <vector>

class SensorActuatorModel {
public:
	SensorActuatorModel(double sensor_max_range, double max_velocity, std::vector<double> acceleration_coeffs);

	double worseCaseResponeTime(double velocity, double sensorRange);
	double worseCaseResponeTime(double velocity, double sensorRange,
			std::vector<double> acceleration_coeffs);

	double maxSensorRange();
	double maxVelocity();

	std::vector<double> accelerationCoeffs();


	virtual ~SensorActuatorModel();

private:
	double maxSensorRange_;
	std::vector<double> accelerationCoeffs_;
	double maxVelocity_;

};

#endif /* SENSORACTUATORMODELS_H_ */
