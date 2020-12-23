import numpy as np
import abc

class System(metaclass=abc.ABCMeta):
    def __init__(self, dof, doc, dt=0.01):
        self.n_state    = dof
        self.n_control  = doc
        self.dt = dt


    @abc.abstractmethod
    def __dynamics(self, state, action):
        ''' Function implements the dynamics of the system.

        :param state:
        :param action:
        :return:
        '''
        raise NotImplementedError("Dynamics are not implemented!")


    #TODO implement process noise option
    def step(self, state, action):
        ''' Update the system state based on current state, control action, and the
        dynamics of the system.

        :param state:   np.array the current state of the system
        :param action:  np.array the control action value
        :return:
        '''
        # calculating the rate of change of the system
        dot_state = self.__dynamics(state, action)
        # euler integration
        next_state   = state + self.dt * dot_state

        return next_state


    #TODO implement option for disturbance
    def observe_system(self, state):
        '''

        :return:
        '''
        return state

    @abc.abstractmethod
    def get_model(self, linearized=False):
        raise NotImplementedError("Get model of system is not implemented!")

    def get_state_dim(self):
        return self.n_state

    def get_action_dim(self):
        return self.n_control

