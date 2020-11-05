import numpy as np

class Data:
    def __init__(self, t_end, dt, dof_state, dof_control, aux_signals=None):
        self.N = int((t_end/dt))+1
        self.time = np.linspace(0, t_end, self.N)
        self.reference = np.zeros((dof_state, self.N))
        self.state = np.zeros((dof_state, self.N))
        self.control_signal = np.zeros((dof_control, self.N))

        if aux_signals is not None:
            self.aux = np.zeros((aux_signals, self.N))
        else:
            self.aux = None

        self.idx = 0

    def push_datapoint(self, ref, state, control, aux=0.0):
        self.reference[:,self.idx] = ref
        self.state[:,self.idx] = state
        self.control_signal[:,self.idx] = control

        if self.aux:
            self.aux[:,self.idx] = aux
        self.idx += 1

    def get_time(self):
        return self.time