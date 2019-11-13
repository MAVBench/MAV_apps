/*
 * SensorActuatorModel.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: reddi-rtx
 */

#include "SensorActuatorModel.h"
using namespace std;

SensorActuatorModel::SensorActuatorModel(double max_sensor_range, double max_velocity, vector<double >acceleration_coeffs){
	maxSensorRange_ = max_sensor_range;
	maxVelocity_ = max_velocity;
	accelerationCoeffs_ = acceleration_coeffs;
}


double SensorActuatorModel::worseCaseResponeTime(double velocityMag, double sensorRange){
	return worseCaseResponeTime(velocityMag, sensorRange, this->accelerationCoeffs_);
}


double SensorActuatorModel::worseCaseResponeTime(double velocityMag,  double sensorRange , vector<double> accelerationCoeffs) {
	double time_budget;
	if (accelerationCoeffs.size() == 2) { // linear model
		double m = accelerationCoeffs[0]; // m: line slope
		double b = accelerationCoeffs[1]; //b: line offset
		time_budget = (pow(velocityMag, 2.0) - 2*m*sensorRange*velocityMag - 2*b*sensorRange)/(-2*m*pow(velocityMag, 2.0) - 2*b*velocityMag);
	}else{
		cout<<"does not support non-linear acceleation models"<<endl;
		throw;
	}

	return time_budget;
}

double SensorActuatorModel::maxSensorRange(){
	return maxSensorRange_;
}

double SensorActuatorModel::maxVelocity(){
	return maxVelocity_;
}


std::vector<double> SensorActuatorModel::accelerationCoeffs() {
	return accelerationCoeffs_;
}

SensorActuatorModel::~SensorActuatorModel() {
	// TODO Auto-generated destructor stub
}

