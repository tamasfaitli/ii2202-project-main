import numpy as np
import time
import matplotlib.pyplot as plt

class Data:
    def __init__(self, T, dt, dof_state, dof_control, aux_signals=None):
        self.N              = int((T/dt))+1
        self.dof_state      = dof_state
        self.dof_control    = dof_control
        self.time           = np.linspace(0, T, self.N)
        self.reference      = np.zeros((dof_state, self.N))
        self.state          = np.zeros((dof_state, self.N))
        self.control_signal = np.zeros((dof_control, self.N))

        if aux_signals is not None:
            self.aux = np.zeros((aux_signals, self.N))
        else:
            self.aux = None

        self.idx = 0

    def push_datapoint(self, ref, state, control, aux=0.0):
        self.reference[:,self.idx] = ref[0:self.dof_state]
        self.state[:,self.idx] = state
        self.control_signal[:,self.idx] = control

        if self.aux:
            self.aux[:,self.idx] = aux
        self.idx += 1

    def get_time_vector(self):
        return self.time

    def get_N_steps(self):
        return self.N

    # figures:
    # states + ref
    # derivatives
    # control signal
    def plot_data(self, save=False, figname=None, **kwargs):
        if save and figname==None:
            t = time.localtime()
            figname = '-'.join([''+str(t[x]) for x in range(5)])+'-'

        # TODO with kwargs decide what figures to plot
        # for key, value in kwargs.items():
        #     print("{0} = {1}".format(key, value))

        for i in range(self.dof_state):
            plt.figure()
            plt.plot(self.time, self.reference[i,:])
            plt.plot(self.time, self.state[i,:])

            plt.legend(["ref"+str(i+1), "state"+str(i+1)])

            plt.xlabel("Time [s]")
            plt.ylabel("Position [m]")

        for i in range(self.dof_control):
            plt.figure()
            plt.plot(self.time, self.control_signal[i,:])

            plt.legend(["control"+str(i+1)])

            plt.xlabel("Time [s]")
            plt.ylabel("Force [N]")

        plt.show()


    def write_to_file(self, filename):
        # header
        # self.N = int((t_end/dt))+1
        # self.time = np.linspace(0, t_end, self.N)
        # self.reference = np.zeros((dof_state, self.N))
        # self.state = np.zeros((dof_state, self.N))
        # self.control_signal = np.zeros((dof_control, self.N))
        with open(filename, "w") as fout:
            header_line = "time"

        fout.close()









