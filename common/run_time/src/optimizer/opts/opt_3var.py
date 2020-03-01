import numpy as np
from .opt_3var_base import opt_3var_base
from .opt_base_r import opt_base_r

# x = [v0 v1 v2]^T
# r0, r1 fixed

class opt_3var(opt_base_r, opt_3var_base):

	# only have this bc im not sure if multi-inheritance works without it
	def __init__(self, r0, r1, Q, r_min, r_max, v_min, v_max,
                    G=None, d=None, A=None, b=None):
		super(opt_3var, self).__init__(r0=r0,
									   r1=r1,
									   Q=Q, 
									   r_min=r_min, 
									   r_max=r_max, 
									   v_min=v_min, 
									   v_max=v_max,
									   G=G, 
									   d=d, 
									   A=A, 
									   b=b)