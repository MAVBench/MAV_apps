#include "pid.h"
#include "ros/ros.h"

PID::PID(double Kp, double Ki, double Kd, double max, double min) :
    _Kp(Kp), _Ki(Ki), _Kd(Kd), _max(max), _min(min), _integral(0), _prev_error(0)
{
}

double PID::calculate(double target, double sensor, double dt)
{
    double result, prop, integ, deriv;
    double error = target - sensor;

    prop = _Kp * error;

    _integral += error * dt;
    integ = _Ki * _integral;

    deriv = _Kd * (error - _prev_error) / dt;
    _prev_error = error;

    result = prop + integ + deriv;
        
    if (result >= _max){
        return _max;
    }
    else if (result <= _min){
        return _min;
    }
    else
        return result;
}

void PID::reset()
{
    _prev_error = 0;
    _integral = 0;
}
