import matplotlib.pyplot as plt
import numpy as np
import math

plt.ylabel('max velocity')
plt.xlabel('Process time')
d_t_list = np.arange(0, 2.3, 0.1) #process time

a_max = 6 #max acceleration (note that the assumption is that we can
          # achieve this immediately, so in realiy we should use some avg
          # a
d_s = 10 #sensor max range
drone_radius = .5
v_max = map(lambda d_t: a_max*(math.sqrt(math.pow(d_t,2) + 2*((d_s-drone_radius))/a_max) - d_t), d_t_list)
line_1, = plt.plot(d_t_list, v_max, label="sensor_max_range = 10");

a_max = 6 
d_s = 7 #sensor max range
drone_radius = .5
v_max = map(lambda d_t: a_max*(math.sqrt(math.pow(d_t,2) + 2*((d_s-drone_radius))/a_max) - d_t), d_t_list)
line_2, = plt.plot(d_t_list, v_max, label="sensor_max_range = 7");

a_max = 6 
drone_radius = .5
d_s = 5 
v_max_2 = map(lambda d_t: a_max*(math.sqrt(math.pow(d_t,2) + 2*((d_s-drone_radius))/a_max) - d_t), d_t_list)
line_3, = plt.plot(d_t_list, v_max_2, label="sensor_max_range = 5");

a_max = 6 
drone_radius = .5
d_s = 3 
v_max_2 = map(lambda d_t: a_max*(math.sqrt(math.pow(d_t,2) + 2*((d_s-drone_radius))/a_max) - d_t), d_t_list)
line_4, = plt.plot(d_t_list, v_max_2, label="sensor_max_range = 3");


plt.legend(handles=[line_1, line_2, line_3, line_4])
plt.show()
