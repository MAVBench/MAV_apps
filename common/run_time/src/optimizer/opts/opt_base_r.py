import numpy as np
from .opt_base import opt_base

# x = [r0 ... v0 ...]^T

class opt_base_r(opt_base):


	def _task_model(self, res, vol, q):
		return (q[0]*res**(-3) + q[1]*res**(-2) + q[2]*res**(-1)) * (q[3]*vol)

	# x = [r_i v_i]^T
	def _partial_r(self, res, vol, q):
		return -(3*q[0]*res**(-4) + 2*q[1]*res**(-3) + q[2]*res**(-2)) * (q[3]*vol)

	def _partial_v(self, res, vol, q):
		return q[3] * (q[0]*res**(-3) + q[1]*res**(-2) + q[2]*res**(-1))

	def _partial_r_r(self, res, vol, q):
		return (12*q[0]*res**(-5) + 6*q[1]*res**(-4) + 2*q[2]*res**(-3)) * (q[3]*vol)

	def _partial_r_v(self, res, vol, q):
		return -q[3] * (3*q[0]*res**(-4) + 2*q[1]*res**(-3) + q[2]*res**(-2))