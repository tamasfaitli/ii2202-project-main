from utils.data import Data
from utils import config as conf

import sys
import numpy as np


if len(sys.argv) > 1:
    config_file = sys.argv[1]
else:
    config_file = None


def simulation(system, controller, reference, init_state, data):
    T = data.get_N_steps()
    state = init_state
    dot_state = np.zeros(len(state))

    for t in range(T):
        ref_signal = reference[:,t]
        # measure system state
        obs_state = system.observe_system(state)

        # evaluate controller
        if controller.get_name() == "PID":
            action = controller.calculate_action(obs_state, ref_signal)
        elif controller.get_name() == "RFPT":
            action = controller.calculate_action(np.append(obs_state,dot_state[1]), ref_signal)
        else:
            action = 0.0

        data.push_datapoint(ref_signal, obs_state, action)

        # feed control signal into the system
        state, dot_state = system.step(state, action)

    return data

# main
if __name__ == "__main__":
    #read config
    config = conf.read_config(config_file)

    # init entities based on config
    T, dt, system, controller, reference, init_state = conf.init_entities(config)

    # init data storage
    data = Data(T, dt, system.get_state_dim(), system.get_action_dim())

    # execute simulation
    simulation_data = simulation(system, controller, reference, init_state, data)

    # evaluate results

    # save/plot results
    data.plot_data()

