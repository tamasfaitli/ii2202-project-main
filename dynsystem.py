import numpy as np

class DynSystem:
    def __init__(self, dof, dt=0.01):
        self.dof = dof
        self.reset()
        self.dt = dt

    def step(self, action):
        raise NotImplementedError("Dynamics of system are not implemented!")

    def observe_system(self):
        return self.state

    def reset(self):
        self.state = np.zeros(self.dof)

    def get_model(self, linearized=False):
        raise NotImplementedError("Get model of system is not implemented!")

    def get_dof(self):
        return self.dof

    def set_initial_state(self, init_state):
        self.state = init_state


# 2 degree of freedom driven van der pol oscillator
# https://en.wikipedia.org/wiki/Van_der_Pol_oscillator
class VanDerPolOscillator2DOF(DynSystem):
    def __init__(self, mu, m, w0, alpha, l, dt=0.01):
        super().__init__(3, dt)
        self.mu = mu
        self.m = m
        self.w0 = w0
        self.alpha = alpha
        self.l = l


    def step(self, action):
        # dyn block calculates second derivate
        d_state = self.dyn_block(action)

        # update state using euler integration
        upd_state = self.integrate(d_state)
        self.state = upd_state

    def observe_system(self):
        return self.state

    def get_state(self):
        return self.state

    def integrate(self, d_state):
        upd_state = np.zeros(self.dof)

        upd_state[2] = d_state[1]
        upd_state[1] = self.state[1] + d_state[1]*self.dt
        upd_state[0] = self.state[0] + d_state[0]*self.dt

        return upd_state

    def dyn_block(self, action):
        d_state = np.zeros(self.dof-1)

        d_state[0] = self.state[2]
        d_state[1] = (-self.mu*(1-self.state[0]**2)*self.state[1]+self.w0**2*self.state[0]+self.alpha*self.state[0]**3+self.l*self.state[0]**5-action)/self.m

        return d_state


