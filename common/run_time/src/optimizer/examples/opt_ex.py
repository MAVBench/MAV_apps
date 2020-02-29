import numpy as np
import sys
sys.path.append('..')
from opt import Opt

if __name__ == '__main__':

	# Desired response times to test #
	num_tests = 1
	rt_d_list = np.array([10.0, 5.0, 2.0, 1.0, 0.5, 0.25])
	rt_d_list = np.repeat(rt_d_list, num_tests)

	# Set important variables #
	r_gap = 0.5
	v_sensor_max = 10000

	r_min = 0.15 # minimal octomap res
	r_max = 3.0

	v_min = 0
	v_max = np.inf

	r_gap_hat = 1 / r_gap
	r_max_hat = 1 / r_min
	r_min_hat = 1 / r_max

	#r1 = 0.15
	#r_hat1 = 1/ r1

	#r0 = 0.15
	#r_hat0 = 1 / r0

	# Coefficient matrix #
	Q = np.array([[-2.16196038e-05, -2.78515364e-03,  2.86859999e-05],
				  [ 2.00720436e-04,  4.60333360e-02, -1.05093373e-05],
				  [ 1.34399197e-04,  4.64316885e-02,  1.24233987e-05],
				  [ 1.00483609e-01,  1.80366135e-05,  4.71434480e-03]])

	# Constraint matrices #
	G = np.array([[-1,1,0,0,0], [0,0,1,-1,0], [0,0,1,0,0], [-1,0,0,0,0]])
	d = np.array([0, 0, v_sensor_max, -r_gap_hat])

	# Create list of decision variable bounds #
	r_min_list = [r_min_hat] * 2
	r_max_list = [r_max_hat] * 2
	v_min_list = [v_min] * 3
	v_max_list = [v_max] * 3

	# Create optimizer instance #
	opt = Opt(Q=Q, 
	          r_min=r_min_list,
	          r_max=r_max_list,
	          v_min=v_min_list,
	          v_max=v_max_list,
	          G=G,
	          d=d)

	# Optimization parameters #
	x0 = np.array([1/0.5, 1/0.5, 5000, 5000, 5000])
	profile = True
	tol = 1e-12

	# Run opt for each desired response time #
	results = []
	for rt_d in rt_d_list:
	    results.append(opt.opt(profile=profile, rt_d=rt_d, x0=x0, tol=tol, verbose=False))

	# Optimization results returned in a dictionary #
	for (result, rt_d) in zip(results, rt_d_list):
		print('Desired response time: ' + str(rt_d) + '\n')
		for key in result:
			print(str(key) + ': ' + str(result[key]) + '\n')
		print('\n')