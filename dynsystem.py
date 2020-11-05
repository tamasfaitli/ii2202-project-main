import numpy as np

class System:
    def __init__(self, dof):
        self.state = np.zeros(dof,1)


    def step(self, action):
        raise NotImplementedError("Dynamics of system are not implemented!")

    def observe_system(self):
        return self.state

    def get_model(self, linearized=False):
        raise NotImplementedError("Get model of system is not implemented!")



class VanDerPolOscillator(System):
    