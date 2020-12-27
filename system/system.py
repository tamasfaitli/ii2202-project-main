import numpy as np
import abc

class System(metaclass=abc.ABCMeta):
    def __init__(self, dof, doc, dos, dt=0.01):
        self.n_state    = dof
        self.n_control  = doc
        self.store_dim  = dos
        self.dt = dt

        self.state = np.zeros(self.n_state)
        self.dot_state = np.zeros(self.n_state)


    @abc.abstractmethod
    def __dynamics(self, state, action):
        ''' Function implements the dynamics of the system.

        :param state:
        :param action:
        :return:
        '''
        raise NotImplementedError("Dynamics are not implemented!")


    #TODO implement process noise option
    def step(self, action):
        ''' Update the system state based on current state, control action, and the
        dynamics of the system.

        :param state:   np.array the current state of the system
        :param action:  np.array the control action value
        :return:
        '''
        # calculating the rate of change of the system
        self.dot_state = self.__dynamics(self.state, action)
        # euler integration
        self.state   = self.state + self.dt * self.dot_state


    #TODO implement option for disturbance
    def observe_system(self, acceleration=False):
        '''

        :return:
        '''
        if not acceleration:
            return self.state
        else:
            return np.append(self.state, self.dot_state[-(self.store_dim-self.n_state):])

    @abc.abstractmethod
    def get_model(self, linearized=False):
        raise NotImplementedError("Get model of system is not implemented!")

    def get_state_dim(self):
        return self.n_state

    def get_action_dim(self):
        return self.n_control

    def get_store_dim(self):
        return self.store_dim

    def set_initial_state(self, state):
        self.state = state

