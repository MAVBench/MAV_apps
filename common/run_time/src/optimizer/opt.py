import numpy as np
import time

from opts.opt_6var import opt_6var
from opts.opt_6var_rhat import opt_6var_rhat
from opts.opt_5var import opt_5var
from opts.opt_5var_rhat import opt_5var_rhat
from opts.opt_4var import opt_4var
from opts.opt_4var_rhat import opt_4var_rhat
from opts.opt_3var import opt_3var
from opts.opt_3var_rhat import opt_3var_rhat


class Opt:

	def __init__(self, method='var5_rhat', *args, **kwargs):

		self.method = method
		self.init_args = args
		self.init_kwargs = kwargs

		self._init_optimizer()


	def _init_optimizer(self):

		if (self.method == 'var6'):
			self.optimizer = opt_6var(*self.init_args, **self.init_kwargs)
		elif (self.method == 'var6_rhat'):
			self.optimizer = opt_6var_rhat(*self.init_args, **self.init_kwargs)
		elif (self.method == 'var5'):
			self.optimizer = opt_5var(*self.init_args, **self.init_kwargs)
		elif (self.method == 'var5_rhat'):
			self.optimizer = opt_5var_rhat(*self.init_args, **self.init_kwargs)
		elif (self.method == 'var4'):
			self.optimizer = opt_4var(*self.init_args, **self.init_kwargs)
		elif (self.method == 'var4_rhat'):
			self.optimizer = opt_4var_rhat(*self.init_args, **self.init_kwargs)
		elif (self.method == 'var3'):
			self.optimizer = opt_3var(*self.init_args, **self.init_kwargs)
		elif (self.method == 'var3_rhat'):
			self.optimizer = opt_3var_rhat(*self.init_args, **self.init_kwargs)
		else:
			raise ValueError('Unknown method %s' % self.method)


	def opt(self, profile=False, *args, **kwargs):
		if profile:
			t0 = time.time()

		result = self.optimizer.opt(*args, **kwargs)

		elapsed = None
		if profile:
			t1 = time.time()
			elapsed = t1 - t0

		# extend OptimizeResult object with two more attributes
		result.elapsed = elapsed
		result.rt_err = self.optimizer.obj(result.x)
		return result


