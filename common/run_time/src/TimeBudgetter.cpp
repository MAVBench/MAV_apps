/*
 * TimeBudgetter.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: reddi-rtx
 */

#include "TimeBudgetter.h"
bool first_itr = true; // only for debugging
TimeBudgetter::TimeBudgetter(double maxSensorRange, double maxVelocity, std::vector<double> accelerationCoeffs, double timeIncr, double max_time_budget)
			:sensorActuatorModel_(maxSensorRange, maxVelocity, accelerationCoeffs),
			timeIncr_(timeIncr),
			max_time_budget(max_time_budget){}



// simple vector magnitude calculation
double TimeBudgetter::calc_magnitude(double x, double y, double z) {
  return std::sqrt(x*x + y*y + z*z);
}

double TimeBudgetter::calc_dist(multiDOFpoint point1, multiDOFpoint point2){
	double dx = point1.x - point2.x;
	double dy = point1.y - point2.y;
	double dz = point1.z - point2.z;
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}


// for a fixed V (i.e., if the drone's v doesn't change), how much budget
double TimeBudgetter::calcSamplingTimeFixV(double velocityMag, double sensorRange, std::vector<double> acceleartionCoeffs, double latency){
	double budget = this->sensorActuatorModel_.worseCaseResponeTime(velocityMag, sensorRange, acceleartionCoeffs )- latency;
	if (isnan(budget) || isinf(budget))  { // occurs when velocity is 0
		budget = max_time_budget;
	}
	return budget;
}

/*
double TimeBudgetter::calcSamplingTimeFixV(double velocityMag, double sensorRange, double latency){
	double budget = this->sensorActuatorModel_.worseCaseResponeTime(velocityMag, sensorRange , this->sensorActuatorModel_.accelerationCoeffs()) - latency;
	if (isnan(budget) || isinf(budget))  { // occurs when velocity is 0
		budget = max_time_budget;
	}

}
*/

double TimeBudgetter::calcSamplingTimeFixV(double velocityMag, double latency, string mode="no_pipelining", double sensor_range=25){
	//double response_time = this->sensorActuatorModel_.worseCaseResponeTime(velocityMag, this->sensorActuatorModel_.maxSensorRange(), this->sensorActuatorModel_.accelerationCoeffs());
	double response_time = this->sensorActuatorModel_.worseCaseResponeTime(velocityMag, sensor_range, this->sensorActuatorModel_.accelerationCoeffs());

	double next_sampling_time;
	if (mode == "no_pipelining"){ // this assumes that latency is equal to 1/throughput
		next_sampling_time = response_time;  // at this I belive, we just have the entire time
											 // to dedicate for budget (but remember to remove decision making time
											 // of this iteration and next iteration from it)
	}else{
		next_sampling_time = response_time - latency;
	}

	if (isnan(next_sampling_time) || isinf(next_sampling_time))  { // occurs when velocity is 0
		next_sampling_time= max_time_budget;
	}

	return next_sampling_time;
}


double TimeBudgetter::calc_budget_till_closest_unknown(multiDOFpoint cur_point, multiDOFpoint closest_unknown_point){
	double velocity_magnitude = get_velocity_projection_mag(cur_point, closest_unknown_point);
	double sensor_range = calc_dist(cur_point, closest_unknown_point);
	double latency = 0;
	return calcSamplingTimeFixV(velocity_magnitude, latency, "no_pipelineing", sensor_range);
}


double TimeBudgetter::calc_budget_till_closest_unknown(trajectory_t traj, multiDOFpoint closest_unknown_point, coord drone_position){
	auto macro_time_budgets = calcSamplingTime(traj, 0.0, closest_unknown_point, drone_position); // not really working well with cur_vel_mag
	return min(max_time_budget, macro_time_budgets[1]);
}


// calcSamplingTime helper (called recursively)
void TimeBudgetter::calcSamplingTimeHelper(std::deque<multiDOFpoint>::iterator trajBegin, std::deque<multiDOFpoint>::iterator trajEnd,
		std::deque<multiDOFpoint>::iterator &trajItr, double &nextSamplingTime, double latency, multiDOFpoint closest_unknown_point, double distance_error){
	multiDOFpoint point = *(trajBegin);
	multiDOFpoint projection;

	//velocity_project_mag = get_velocity_projection_mag(point, closest_unknown_point);
	//double velocity_magnitude = calc_magnitude(point.vx, point.vy, point.vz);
	double velocity_magnitude = get_velocity_projection_mag(point, closest_unknown_point);
	//velocity_magnitude -= velocity_error;

	double sensor_range = calc_dist(point, closest_unknown_point) + distance_error;
	// blah change the sensor_Range value after data collection
	//sensor_range = 25;
	/*
	if (first_itr){ // for debugging
		//ROS_INFO_STREAM("------------ first unknown point distance from first way point"<<sensor_range);
	}
	*/
	/*
	if (isnan(velocity_magnitude)){
		ROS_INFO_STREAM("fucked up");
	}
	*/
	double BudgetTillNextSample = calcSamplingTimeFixV(velocity_magnitude, latency, "no_pipelineing", sensor_range);
	double potentialBudgetTillNextSample;  // a place holder that gets updated
	std::deque<multiDOFpoint>::iterator trajItrTemp =  trajBegin;  //pointing to the sample point we are considering at the moment

	// corener case
	if (BudgetTillNextSample <= 0) {
		//std::cout<<"shoudn't get sample time less than zero; probaly went over the v limit"<<std::endl;
		nextSamplingTime = this->timeIncr_;
		trajItr += 1;
		return;
	}

	double nextSamplingTimeTemp = 0;

	while (BudgetTillNextSample > 0 && trajItrTemp != trajEnd){
		BudgetTillNextSample -= this->timeIncr_;
		nextSamplingTimeTemp += this->timeIncr_;
		point = *(trajItrTemp);
		//velocity_magnitude = calc_magnitude(point.vx, point.vy, point.vz);
		velocity_magnitude = get_velocity_projection_mag(point, closest_unknown_point);
		//velocity_magnitude -= velocity_error;
		sensor_range = calc_dist(point, closest_unknown_point);
		potentialBudgetTillNextSample = calcSamplingTimeFixV(velocity_magnitude, latency, "no_pipelining", sensor_range);
		if (potentialBudgetTillNextSample <= 0) {
			//std::cout<<"-- shoudn't get sample time less than zero; probaly went over the v limit"<<std::endl;
			trajItrTemp +=1;
			nextSamplingTimeTemp +=  this->timeIncr_;
			break;
		}
		BudgetTillNextSample = std::min(potentialBudgetTillNextSample, BudgetTillNextSample);
		trajItrTemp +=1;
	}
	trajItr = trajItrTemp;
	if (trajItrTemp == trajEnd){ // add the left overs
		nextSamplingTime = nextSamplingTimeTemp + BudgetTillNextSample;
	}else{
		nextSamplingTime = nextSamplingTimeTemp;
	}
}


// iteratively going through all the points in the trajectory and calculating the time budget by
// calling its Helper
std::vector<double> TimeBudgetter::calcSamplingTime(trajectory_t traj, double latency, multiDOFpoint closest_unknown_point, coord position){
	double thisSampleTime = 0;
	double nextSampleTime = 0;
	this->SamplingTimes_.clear();
	this->SamplingTimes_.push_back(thisSampleTime);

	std::deque<multiDOFpoint>::iterator trajRollingItrBegin = traj.begin();
	std::deque<multiDOFpoint>::iterator trajItrEnd = traj.end();
	std::deque<multiDOFpoint>::iterator trajItr = traj.begin();

	first_itr = true; // this variable is only for debugging.

	// first point
	multiDOFpoint point = *(trajItr);
	//double velocity_magnitude = calc_magnitude(point.vx, point.vy, point.vz);
	//double velocity_error;
	double distance_error = calc_magnitude(position.x - point.x , position.y - point.y, position.z - point.z);
	// use velocity error to correct for the difference between current velocity and the desired velocity
	// this will help us to avoid being conservative
	/*
	if (cur_velocity_mag == -1){
		velocity_error = 0;
	}else{
		velocity_error = velocity_magnitude - cur_velocity_mag;
	}
	*/

	while (trajRollingItrBegin < trajItrEnd){
		calcSamplingTimeHelper(trajRollingItrBegin, trajItrEnd, trajItr, nextSampleTime, latency, closest_unknown_point, distance_error);
		thisSampleTime += nextSampleTime;
		this->SamplingTimes_.push_back(thisSampleTime);
		trajRollingItrBegin = trajItr;
		first_itr = false;
	}
	return this->SamplingTimes_;
}


// destructor
TimeBudgetter::~TimeBudgetter() {
	// TODO Auto-generated destructor stub
}
