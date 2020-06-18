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
	TimeBudgetter(double maxSensorRange, double maxVelocity, std::vector<double> accelerationCoeffs, double timeIncr, double max_time_budget);


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
	double get_velocity_projection_mag(multiDOFpoint cur_point, multiDOFpoint closest_unknown);


	virtual ~TimeBudgetter();
    double calc_dist(multiDOFpoint point1, multiDOFpoint point2);
private:
	SensorActuatorModel sensorActuatorModel_;
	std::vector<double> SamplingTimes_;
	double timeIncr_;
	double max_time_budget;
};

// linker is not happy if the definition of a member inline function is
// outside its header file.
// see: https://isocpp.org/wiki/faq/inline-functions#inline-member-fns
inline double TimeBudgetter::get_velocity_projection_mag(multiDOFpoint cur_point, multiDOFpoint closest_unknown){
	multiDOFpoint vector_to_project_on;

	// projection of a over b is (a.b/|a|**2)*a where a and b are both vectors
	// find the vector to project it on. This is calculated using the current point and the closest unknown point
	// this is out b vector, where as our a vector is cur_point's velocity
	vector_to_project_on.x= closest_unknown.x - cur_point.x;
	vector_to_project_on.y= closest_unknown.y - cur_point.y;
	vector_to_project_on.z= closest_unknown.z - cur_point.z;

	double dot_product = vector_to_project_on.x*cur_point.vx +
			vector_to_project_on.y*cur_point.vy + vector_to_project_on.z*cur_point.vz;
	double vector_to_project_on_mag_sqr = pow(calc_magnitude(vector_to_project_on.x,vector_to_project_on.y, vector_to_project_on.z), 2);

	multiDOFpoint projected_vector;
	projected_vector.x = (dot_product/vector_to_project_on_mag_sqr)*vector_to_project_on.x;
	projected_vector.y = (dot_product/vector_to_project_on_mag_sqr)*vector_to_project_on.y;
	projected_vector.z = (dot_product/vector_to_project_on_mag_sqr)*vector_to_project_on.z;
	return calc_magnitude(projected_vector.x, projected_vector.y, projected_vector.z);
}

#endif /* TIMEBUDGETTER_H_ */
