from utils.data import Data
from utils import config as conf

import sys

if len(sys.argv) > 1:
    config_file = sys.argv[1]
else:
    config_file = None


def simulation(system, controller, reference, init_state, data):
    T = data.get_time_vector()
    state = init_state

    for t in T[1:]:
        # measure system state
        obs_state = system.observe_system(state)

        # evaluate controller
        action = controller.calculate_action(obs_state, reference)

        data.push_datapoint(reference, obs_state, action)

        # feed control signal into the system
        state = system.step(state, action)

    return data

# main
if __name__ == "__main__":
    #read config
    config = conf.read_config(config_file)

    # init entities based on config
    system, controller, init_state, reference = conf.init_entities(config)

    # init data storage
    data = Data(config["simulation"], system.get_state_dim(), system.get_action_dim())

    # execute simulation
    simulation_data = simulation(system, controller, reference, init_state, data)

    # evaluate results

    # save/plot results
    data.plot_data()

