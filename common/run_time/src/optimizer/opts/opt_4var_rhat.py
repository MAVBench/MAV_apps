import numpy as np
from .opt_4var import opt_4var_base
from .opt_base_rhat import opt_base_rhat

# x = [r_hat0 v0 v1 v2]^T
# r_hat1 is fixed

class opt_4var_rhat(opt_base_rhat, opt_4var_base):

	# only have this bc im not sure if multi-inheritance works without it
	def __init__(self, r1, Q, r_min, r_max, v_min, v_max,
                    G=None, d=None, A=None, b=None):
		super(opt_4var_rhat, self).__init__(r1=r1,
											Q=Q, 
											r_min=r_min, 
											r_max=r_max, 
											v_min=v_min, 
											v_max=v_max,
											G=G, 
											d=d, 
											A=A, 
											b=b)