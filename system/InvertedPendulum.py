from system.system import System

import numpy as np

class InvertedPendulum4DOF(System):
    def __init__(self, params, dt=0.01):
        super().__init__(4, 1, 6, dt)

        self.w = 0.0

        self.g = params['g']
        self.m = params['m']
        self.M = params['M']
        self.I = params['I']
        self.l = params['l']
        self.bc = params['bc']
        self.bp = params['bp']

        self.approx_g = params['approx_g']
        self.approx_m = params['approx_m']
        self.approx_M = params['approx_M']
        self.approx_I = params['approx_I']
        self.approx_l = params['approx_l']
        self.approx_bc = params['approx_bc']
        self.approx_bp = params['approx_bp']

    def _System__dynamics(self, state, action):
        x1 = state[0]
        x2 = state[1]
        x3 = state[2]
        x4 = state[3]
        u = action

        # dot(x2)
        f1 = (1.0 / (self.M + self.m - (self.m ** 2 * self.l ** 2 * np.cos(x3) ** 2) / (
                    self.I + self.m * self.l ** 2))) * \
             (u + ((self.m * self.l ** 2 * np.cos(x3) ** 2) / (self.I + self.m * self.l ** 2) - 1) * self.w
              - self.bc * x2
              - self.m * self.l * x4 ** 2 * np.sin(x3)
              + (self.m ** 2 * self.l ** 2 * self.g * np.sin(x3) * np.cos(x3)) / (self.I + self.m * self.l ** 2)
              - (self.m * self.l * self.bp * x4 * np.cos(x3)) / (self.I + self.m * self.l ** 2)
              )

        # dot(x4)
        f2 = (1.0 / (self.I + self.m * self.l ** 2 - (self.m ** 2 * self.l ** 2 * np.cos(x3) ** 2) / (
                    self.m + self.M))) * \
             (
                     u * (self.m * self.l * np.cos(x3)) / (self.m + self.M)
                     + self.w * (self.M * self.l * np.cos(x3)) / (self.m + self.M)
                     - self.bp * x4
                     + self.m * self.l * self.g * np.sin(x3)
                     - (self.m ** 2 * self.l ** 2 * x4 ** 2 * np.sin(x3) * np.cos(x3)) / (self.m + self.M)
                     - (self.m * self.l * self.bc * x2 * np.cos(x3)) / (self.m + self.M)
             )

        dot_state = [x2, f1, x4, f2]

        return dot_state

    def __model(self, desired_effect):
        v_1 = (self.approx_m+self.approx_M)/(self.approx_I*(self.approx_m+self.approx_M)+
                                             self.approx_m*self.approx_M*(self.approx_l**2))

        A_d_omega = np.array([
            0,
            -(self.approx_m*self.approx_l*self.approx_bc*v_1)/(self.approx_m+self.approx_M),
            self.approx_m*self.approx_g*self.approx_l*v_1,
            -self.approx_bp*v_1
        ])

        B_d_omega = (self.approx_m*self.approx_l*v_1)/(self.approx_m+self.approx_M)

        A_s_d_omega = np.matmul(A_d_omega, self.state)

        u = (1/B_d_omega)*(desired_effect-A_s_d_omega)

        return u

    def get_model(self, linearized=False):
        return self.__model