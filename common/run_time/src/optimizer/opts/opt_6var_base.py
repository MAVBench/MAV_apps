import numpy as np
from .opt_base import opt_base

# x = [r0 r1 r2 v0 v1 v2]^T

class opt_6var_base(opt_base):

	def _x0_fun(self, x):
		return np.array([[x[0], x[3]]])

	def _x1_fun(self, x):
		return np.array([[x[1], x[4]]])

	def _x2_fun(self, x):
		return np.array([[x[2], x[5]]])

	def _model(self, x):
		Q = self._Q

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

		return np.hstack((-self.partial_r(x0, Q[0,:]), 
		                  -self.partial_r(x1, Q[1,:]), 
		                  -self.partial_r(x2, Q[2,:]), 
		                  -self.partial_v(x0, Q[0,:]), 
		                  -self.partial_v(x1, Q[1,:]), 
		                  -self.partial_v(x2, Q[2,:])))


	def _obj_H(self, x):
		x0 = self._x0_fun(x)
		x1 = self._x1_fun(x)
		x2 = self._x2_fun(x)
		Q = self._Q
	   
		return np.vstack((np.hstack((-self.partial_r_r(x0, Q[0,:]), 0, 0, -self.partial_r_v(x0, Q[0,:]), 0, 0)),
						  np.hstack((0, -self.partial_r_r(x1, Q[1,:]), 0, 0, -self.partial_r_v(x1, Q[1,:]), 0)),
						  np.hstack((0, 0, -self.partial_r_r(x2, Q[2,:]), 0, 0, -self.partial_r_v(x2, Q[2,:]))),
						  np.hstack((-self.partial_r_v(x0, Q[0,:]), 0, 0, 0, 0, 0)),
						  np.hstack((0, -self.partial_r_v(x1, Q[1,:]), 0, 0, 0 ,0)),
						  np.hstack((0, 0, -self.partial_r_v(x2, Q[2,:]), 0, 0, 0))))