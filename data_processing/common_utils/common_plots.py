from utils import *
velocity_vals = np.array([25.7, 18.5, 9.6, 7.7, 5.8, 3.9, 2.0])
acceleration_vals = np.array([4.3, 3.55, 2.46, 2.11, 1.73, 1.25, .75])
deg = 1
m, b_ = fit_func(velocity_vals, acceleration_vals, deg)
visibility = 25
response_time = 3.4  # gotten from the stats.json


print calc_v_max(response_time, visibility, m, b_)
