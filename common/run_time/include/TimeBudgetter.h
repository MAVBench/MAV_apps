#ifndef TIMEBUDGETTER_H_
#define TIMEBUDGETTER_H_
#include "SensorActuatorModel.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <common.h>
//#include "data_types.h"

class TimeBudgetter {
public:
	TimeBudgetter(double maxSensorRange, double maxVelocity, std::vector<double> accelerationCoeffs, double timeIncr, double max_time_budget, double drone_radius);


	double calc_budget_till_closest_unknown(multiDOFpoint cur_point, multiDOFpoint closest_unknown_point);
	double calc_budget_till_closest_unknown(trajectory_t traj, multiDOFpoint closest_unknown_point, coord drone_position);

	double calcSamplingTimeFixV(double velocityMag , double sensorRange, std::vector<double> acceleartionCoeffs, double latency);
	//double calcSamplingTimeFixV(double velocityMag, double sensorRange, double latency);
	double calcSamplingTimeFixV(double velocityMag, double latency, string mode, double closest_obs);

	void calcSamplingTimeHelper(std::deque<multiDOFpoint>::iterator trajBegin,
			std::deque<multiDOFpoint>::iterator trajEnd , std::deque<multiDOFpoint>::iterator &trajItr,
			double &nextSamplingTime, double latency, multiDOFpoint closest_unknown_point, double velocity_error);
	std::vector<double> calcSamplingTime(trajectory_t traj, double latency, multiDOFpoint closest_unknown_point, coord position);
	double calc_magnitude(double x, double y, double z);
	double inline get_velocity_projection_mag(multiDOFpoint cur_point, multiDOFpoint closest_unknown);


	virtual ~TimeBudgetter();
    double calc_dist(multiDOFpoint point1, multiDOFpoint point2);
private:
	SensorActuatorModel sensorActuatorModel_;
	std::vector<double> SamplingTimes_;
	double timeIncr_;
	double max_time_budget;
	double drone_radius;
};

#endif /* TIMEBUDGETTER_H_ */
