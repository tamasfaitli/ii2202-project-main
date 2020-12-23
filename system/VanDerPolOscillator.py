from system.system import System
import numpy as np

# Dynamics are taken from paper below - Equation [1]
# Van Dooren, René. "Numerical study of the controlled Van der Pol oscillator in Chebyshev series."
# Zeitschrift für angewandte Mathematik und Physik ZAMP 38.6 (1987): 934-939.
class VanDerPolOscillator2DOF(System):
    def __init__(self, params, dt=0.01):
        super().__init__(2, dt)
        self.epsilon    = params['epsilon']
        self.omega      = params['omega']


    def _System__dynamics(self, state, action):
        ''' Execute the dynamical equations of the Van der Pol oscillator

        :param state:  np.array state[0]:x, state[1]:x_dot
        :param action: np.array/scalar control action
        :return: np.array dot_state[0]: x_dot, dot_state[1]: x_ddot
        '''
        assert len(state)  == 2, "State vector should contain 2 elements!"
        # assert len(action) == 1, "Action should be a scalar!"

        dot_state = np.zeros(self.dof)

        dot_state[0] = state[1]
        dot_state[1] = self.epsilon*self.omega*(1-state[0]**2)*state[1]-self.omega**2*state[0]-action

        return dot_state

    def get_model(self, linearized=False):
        #TODO implement
        return None