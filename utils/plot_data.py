

import numpy as np
import matplotlib.pyplot as plt


def subplot_results(t, q, dq, ddq, ref, Q):
    fig, axs = plt.subplots(4)

    axs[0].plot(t, q[0,:])
    axs[0].plot(t, q[1,:])
    axs[0].plot(t, ref[0,:])
    axs[0].plot(t, ref[1,:])

    axs[1].plot(t, dq[0,:])
    axs[1].plot(t, dq[1,:])

    axs[2].plot(t, ddq[0,:])
    axs[2].plot(t, ddq[1,:])

    axs[3].plot(t, Q)

    plt.show()

def subplot_pos_force(t, q, ref, Q):
    fix, axs = plt.subplots(3)

    axs[0].plot(t[:-1], q[0,:-1])
    axs[0].plot(t[:-1], ref[0,:-1])
    axs[0].legend(["x[t]", "x_ref[t]"])

    axs[1].plot(t[:-1], q[1,:-1])
    axs[1].plot(t[:-1], ref[1,:-1])
    axs[1].legend(["theta[t]", "theta_ref[t]"])

    axs[2].plot(t[:-1], Q[:-1])
    axs[2].legend(["Action force"])

    plt.show()


def plot_ddq(t, ddq_real, ddq_des, ddq_def):
    fig, axs = plt.subplots(2)

    axs[0].plot(t[:-1], ddq_real[0,:-1])
    axs[0].plot(t[:-1], ddq_des[:-1])
    axs[0].plot(t[:-1], ddq_def[:-1])
    axs[0].legend(["x_ddq_real", "x_ddq_des", "x_ddq_def"])

    axs[1].plot(t[:-1], ddq_real[0,:-1])
    axs[1].plot(t[:-1], ddq_des[:-1])
    axs[1].plot(t[:-1], ddq_def[:-1])
    axs[1].legend(["th_ddq_real", "th_ddq_des", "th_ddq_def"])

    plt.show()

