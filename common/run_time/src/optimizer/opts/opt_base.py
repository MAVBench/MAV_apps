import numpy as np
from scipy.optimize import Bounds, LinearConstraint, NonlinearConstraint, curve_fit, minimize



class opt_base(object):

    # Constraints defined as such:
    # Gx <= d
    # Ax = b
    # Q is a 4x3 coefficient matrix, column major also
    def __init__(self, Q, r_min, r_max, v_min, v_max, 
                    G=None, d=None, A=None, b=None):
        self.set_Q(Q)
        self.G = G
        self.d = d
        self.A = A
        self.b = b

        # these are lists of mins and maxes, in numerical order
        self.r_min = r_min
        self.r_max = r_max
        self.v_min = v_min
        self.v_max = v_max

        self.ineq_cons = None
        if (G is not None and d is not None):
            self.ineq_cons = {'type': 'ineq',
                              'fun' : lambda x: np.matmul(-G, x) + d,
                              'jac' : lambda x: -G}

        self.eq_cons = None
        if (A is not None and b is not None):
            self.eq_cons = {'type' : 'eq',
                            'fun'  : lambda x: np.matmul(A, x) + b,
                            'jac'  : lambda x: A}

        self.cons = list(filter(None, [self.ineq_cons, self.eq_cons]))

        self.set_bounds()


    # use this to change Q so as to not forget to transpose
    def set_Q(self, Q):
        self._Q = Q.T

    def set_bounds(self):
        self.lb = self.r_min + self.v_min
        self.ub = self.r_max + self.v_max
        self.bounds = Bounds(self.lb, self.ub)


    def opt(self, rt_d, x0, tol=1e-9, maxiter=100, verbose=False):
        # only handles SLSQP for now
        method = 'SLSQP'
        self.rt_d = rt_d
        return minimize(self.obj_norm,
                        x0, 
                        method=method, 
                        jac=self.obj_norm_J,
                        constraints=self.cons,
                        bounds=self.bounds,
                        options={'ftol': tol, 
                                 'maxiter': maxiter,
                                 'disp': verbose})


    def _task_model(self, res, vol, q):
        pass

    def task_model(self, x, q):
    	res = x[:,0]
    	vol = x[:,1]
    	return self._task_model(res, vol, q)


    # also uses Q!
    def _model(self, x):
        pass

    def model(self, x):
    	return self._model(x)


    def obj(self, x):
        return self.rt_d - np.sum(self.model(x))

    def obj_norm(self, x):
        return np.square(self.obj(x))


    def _obj_J(self, x):
        pass

    def obj_J(self, x):
        return self._obj_J(x)

    def obj_norm_J(self, x):
        return 2 * self.obj(x) * self.obj_J(x)


    def _obj_H(self, x):
        pass

    def obj_H(self, x, v=None):
        return self._obj_H(x)

    def obj_norm_H(self, x, v=None):
        J_g = self.obj_J(x).reshape((1,-1))
        return 2 * (np.matmul(J_g.T, J_g) + self.obj(x) * self.obj_H(x))


    def _partial_r(self, res, vol, q):
        pass

    def partial_r(self, x, q):
        res = x[:,0]
        vol = x[:,1]
        return self._partial_r(res, vol, q)


    def _partial_v(self, res, vol, q):
        pass

    def partial_v(self, x, q):
        res = x[:,0]
        vol = x[:,1]
        return self._partial_v(res, vol, q)


    def _partial_r_r(self, res, vol, q):
        pass

    def partial_r_r(self, x, q):
        res = x[:,0]
        vol = x[:,1]
        return self._partial_r_r(res, vol, q)


    def _partial_r_v(self, res, vol, q):
        pass

    def partial_r_v(self, x, q):
        res = x[:,0]
        vol = x[:,1]
        return self._partial_r_v(res, vol, q)