import numpy as np
from .opt_4var_base import opt_4var_base
from .opt_base_r import opt_base_r

# x = [r0 v0 v1 v2]^T
# r1 is fixed

class opt_4var(opt_base_r, opt_4var_base):

	# only have this bc im not sure if multi-inheritance works without it
	def __init__(self, r1, Q, r_min, r_max, v_min, v_max,
                    G=None, d=None, A=None, b=None):
		super().__init__(r1=r1,
						 Q=Q, 
						 r_min=r_min, 
						 r_max=r_max, 
						 v_min=v_min, 
						 v_max=v_max,
						 G=G, 
						 d=d, 
						 A=A, 
						 b=b)