import numpy as np

from dynsystem import VanDerPolOscillator2DOF
from controller import PIDController
from data import Data

dt = 0.01
length = 3

# main
if __name__ == "__main__":

    # values from Possible Adaptive Control by Tangent Hyperbolic Fixed Point
    # Transformations Used for Controlling the Φ6-Type Van der Pol Oscillator
    vdp_system = VanDerPolOscillator2DOF(0.4, 1.0, 0.46, 1.0, 0.1, dt)

    # initializing PID controller
    controller = PIDController(10, 1, 3, dt)

    # data holder
    vdp_data = Data(length, dt, vdp_system.get_dof(), 1)


    # init
    vdp_system.set_initial_state(np.array([2.5,0.0,0.0]))
    ref = np.zeros(vdp_system.get_dof())
    vdp_data.push_datapoint(ref, vdp_system.get_state(), 0.0)

    T = vdp_data.get_time_vector()
    for t in T[1:]:
        measured_state = vdp_system.observe_system()
        action = controller.calculate_action(measured_state, ref)

        vdp_system.step(action)

        vdp_data.push_datapoint(ref, vdp_system.get_state(), action)

    vdp_data.plot_data()

