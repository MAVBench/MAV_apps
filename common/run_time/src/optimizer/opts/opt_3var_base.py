import numpy as np
from .opt_base import opt_base

# x = [v0 v1 v2]^T
# r0, r1 fixed

class opt_3var_base(opt_base):

	def __init__(self, r0, r1, Q, r_min, r_max, v_min, v_max,
                    G=None, d=None, A=None, b=None):
		super().__init__(Q=Q, 
						 r_min=r_min, 
						 r_max=r_max, 
						 v_min=v_min, 
						 v_max=v_max,
						 G=G, 
						 d=d, 
						 A=A, 
						 b=b)
		self._r0 = r0
		self._r1 = r1

	def _x0_fun(self, x):
		return np.array([[self._r0, x[0]]])

	def _x1_fun(self, x):
		return np.array([[self._r1, x[1]]])

	def _x2_fun(self, x):
		return np.array([[self._r1, x[2]]])

	def _model(self, x):
		Q = self._Q
		r0 = self._r0
		r1 = self._r1

		x0 = self._x0_fun(x)
		model0 = self.task_model(x0, Q[0,:])
	    
		x1 = self._x1_fun(x)
		model1 = self.task_model(x1, Q[1,:])
	    
		x2 = self._x2_fun(x)
		model2 = self.task_model(x2, Q[2,:])
	    
		return np.hstack((model0, model1, model2))


	def _obj_J(self, x):
		x0 = self._x0_fun(x)
		x1 = self._x1_fun(x)
		x2 = self._x2_fun(x)
		Q = self._Q

		return np.hstack((-self.partial_v(x0, Q[0,:]), 
		                  -self.partial_v(x1, Q[1,:]), 
		                  -self.partial_v(x2, Q[2,:])))


	def _obj_H(self, x):
		x0 = self._x0_fun(x)
		x1 = self._x1_fun(x)
		x2 = self._x2_fun(x)
		Q = self._Q
	    
		return np.vstack((np.hstack((0, 0, 0)),
	                      np.hstack((0, 0, 0)),
	                      np.hstack((0, 0, 0))))