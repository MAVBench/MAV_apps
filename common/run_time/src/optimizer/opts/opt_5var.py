import numpy as np
from .opt_base_r import opt_base_r
from .opt_5var_base import opt_5var_base

# x = [r0 r1 v0 v1 v2]^T

class opt_5var(opt_base_r, opt_5var_base):

	# only have this bc im not sure if multi-inheritance works without it
	def __init__(self, Q, r_min, r_max, v_min, v_max,
                    G=None, d=None, A=None, b=None):
		super(opt_5var, self).__init__(Q=Q, 
									   r_min=r_min, 
									   r_max=r_max, 
									   v_min=v_min, 
									   v_max=v_max, 
									   G=G, 
									   d=d, 
									   A=A, 
									   b=b)