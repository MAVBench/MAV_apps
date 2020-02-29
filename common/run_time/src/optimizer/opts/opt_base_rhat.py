import numpy as np
from .opt_base import opt_base

# x = [r_hat0 ... v0 ...]^T
# r_hat0 = 1 / r0

class opt_base_rhat(opt_base):


	def _task_model(self, res, vol, q):
		return (q[0]*res**(3) + q[1]*np.square(res) + q[2]*res) * (q[3]*vol)

	# x = [r_hat_i v_i]^T
	def _partial_r(self, res, vol, q):
		return (3*q[0]*np.square(res) + 2*q[1]*res + q[2]) * (q[3]*vol)

	def _partial_v(self, res, vol, q):
		return q[3] * (q[0]*res**(3) + q[1]*np.square(res) + q[2]*res)

	def _partial_r_r(self, res, vol, q):
		return (6*q[0]*res + 2*q[1]) * (q[3]*vol)

	def _partial_r_v(self, res, vol, q):
		return q[3] * (3*q[0]*np.square(res) + 2*q[1]*res + q[2])