from data import *
import numpy as np

def calc_quadrative(a,b,c):
    return (-b + sqrt((b**2) - 4*a*c))/2*a
def calc_total_t_budget(v, visibility, m, b_):
    return (v**2 - 2*m*visibility*v - 2*b_*visibility)/(-2*m*(v**2) - 2*b_*v) 

def calc_next_sample_time_fix_v(v, visibility, m, b_, latency):
    return calc_total_t_budget(v, visibility, m, b_) - latency

def fit_func(x, y, deg):
    m, b = np.polyfit(x, y, deg)
    return m,b


def subsample(my_l, subsample_rate = 5):
    cntr = 0 
    result = [] 
    for i in range(len(my_l)):
        if cntr % subsample_rate == 0:
            result.append(my_l[cntr])
        cntr +=1
    return result


def calc_next_sample_time(velocity_data_, visibility, m, b_, min_latency, delta_t):
    v = velocity_data_[0]
    budget_till_next_sample = calc_next_sample_time_fix_v(v, visibility, m, b_, min_latency)
    if budget_till_next_sample <= 0:
        print("shouldn't get sample time less than zero, probably went over the v limit but still didn't hit anything")
        return 1, delta_t
    next_sample_time = 0
    idx = 0 
    while budget_till_next_sample > 0 and idx < len(velocity_data_):
        budget_till_next_sample -= delta_t
        next_sample_time += delta_t
        v = velocity_data_[idx]
        potential_budget_till_next_sample = calc_next_sample_time_fix_v(v, visibility, m, b_, min_latency)
        if (potential_budget_till_next_sample <= 0):
            print("shouldn't get sample time less than zero, probably went over the v limit but still didn't hit anything")
            return idx+1, next_sample_time + delta_t
        budget_till_next_sample = min(potential_budget_till_next_sample, budget_till_next_sample)
        idx +=1
    return idx, next_sample_time


def iteratively_calc_next_sample_time(velocity_data_, visibility, m, b_, min_latency, delta_t):
    idx_base = 0
    idx = 0
    this_sample_time = 0
    next_sample_time_l = [this_sample_time]
    while idx_base < len(velocity_data_):
        idx, next_sample_time = calc_next_sample_time(velocity_data_[idx_base:], visibility, m, b_, min_latency, delta_t)
        idx_base += idx
        this_sample_time += next_sample_time
        next_sample_time_l.append(this_sample_time)
    return  next_sample_time_l


def main():
    delta_t = .05
    v_max_l = []
    velocity_vals = np.array([25.7, 18.5, 9.6, 7.7, 5.8, 3.9, 2.0])
    acceleration_vals = np.array([4.3, 3.55, 2.46, 2.11, 1.73, 1.25, .75])
    deg = 1
    m, b = fit_func(velocity_vals, acceleration_vals, deg)
    visibility = 25
    velocity_data_ = velocity_data[:]
    min_latency = 1.55  # gotten from the stats.json

    iteratively_calc_next_sample_time(velocity_data_, visibility, m, b, min_latency, delta_t)


#main()
