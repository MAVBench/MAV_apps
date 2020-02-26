import numpy as np
import matplotlib.patches as patches
import sys


def calc_total_t_budget(v, visibility, m, b_):
    return (v**2 - 2*m*visibility*v - 2*b_*visibility)/(-2*m*(v**2) - 2*b_*v) 

def calc_next_stample_time(v, visibility, m, b_, latency):
    return calc_total_t_budget(v, visibility, m, b_) - latency

def subsample(my_l, subsample_rate = 5):
    cntr = 0 
    result = [] 
    for i in range(len(my_l)):
        if cntr % subsample_rate == 0:
            result.append(my_l[cntr])
        cntr +=1
    return result

def calc_poly_deg_2(a, b, c):
    return (-b + np.sqrt(b**2 - 4*a*c))/(2*a)

"""
def calc_v_max(response_time, visibility, m, b_):
    a = 1 + 2*response_time*m
    b = -2*m*visibility + 2*response_time*b_
    c = -2*b_*visibility 
    return calc_poly_deg_2(a, b, c) 
"""
def calc_v_max(response_time, visibility, m, b_):
    a = 1 + 2*response_time*m
    b = -2*m*visibility + 2*response_time*b_
    c = -2*b_*visibility 
    return calc_poly_deg_2(a, b, c)


def fit_func(x, y, deg):
    m, b = np.polyfit(x, y, deg)
    return m,b


def fit_func_2(x, y, deg):
    a, b, c = np.polyfit(x, y, deg)
    return a,b,c




