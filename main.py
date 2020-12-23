from utils.data import Data
from utils import config as conf

import sys
import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) > 1:
    config_file = sys.argv[1]
else:
    config_file = None

epsilon = 0.15
omega = 1
vdp_params = {'epsilon': 0.15, 'omega': 1}
dt = 0.01
length = 3
init_state = [2.5, 0.0]
pid_params = {'proportional': 10, 'integral': 1, 'derivative': 3}



# main
if __name__ == "__main__":

    config = conf.read_config(config_file)

    system, controller = conf.init_entities(config)

    # data holder
    vdp_data = Data(length, dt, system.get_dof(), 1)

    # init
    state = np.array(init_state)
    ref = np.zeros(system.get_dof())
    vdp_data.push_datapoint(ref, state, 0.0)

    T = vdp_data.get_time_vector()
    for t in T[1:]:
        # measure system state
        obs_state = system.observe_system(state)

        # evaluate controller
        action = controller.calculate_action(obs_state, ref)

        vdp_data.push_datapoint(ref, obs_state, action)

        # feed control signal into the system
        state = system.step(state, action)


    vdp_data.plot_data()

