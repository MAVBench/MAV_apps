import numpy as np
from scipy.optimize import minimize
from .opt_base_rhat import opt_base_rhat
from .opt_5var_base import opt_5var_base

### Main difference between this and 5var_rhat solver is that this one has extra ###
### -v_0 term in objective to also maximize v_0                                  ###


### Decided to add new functions to allow easier insight into previous portions ###
### of objective function.                                                      ###


# x = [r_hat0 r_hat1 v0 v1 v2]^T
# r_hat = 1 / r

class opt_5var_rhat_volmax(opt_base_rhat, opt_5var_base):

    # only have this bc im not sure if multi-inheritance works without it
    def __init__(self, Q, r_min, r_max, v_min, v_max, 
                    G=None, d=None, A=None, b=None):
        super(opt_5var_rhat_volmax, self).__init__(Q=Q, 
                                                   r_min=r_min, 
                                                   r_max=r_max, 
                                                   v_min=v_min, 
                                                   v_max=v_max,
                                                   G=G, 
                                                   d=d, 
                                                   A=A,
                                                   b=b)

    def obj_volmax(self, x):
        v0 = self._x0_fun(x).flatten()[1] # second element
        return self.obj_norm(x) - v0

    def obj_volmax_J(self, x):
        J_adj = np.array([0, 0, -1, 0, 0]) # adjustment from volume term
        return self.obj_norm_J(x) + J_adj

    def opt(self, rt_d, x0, tol=1e-9, maxiter=10000, verbose=False):
        # only handles SLSQP for now
        method = 'SLSQP'
        self.rt_d = rt_d
        return minimize(self.obj_volmax,
                        x0, 
                        method=method, 
                        jac=self.obj_volmax_J,
                        constraints=self.cons,
                        bounds=self.bounds,
                        options={'ftol': tol, 
                                 'maxiter': maxiter,
                                 'disp': verbose})