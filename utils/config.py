from system.VanDerPolOscillator import VanDerPolOscillator2DOF
from system.InvertedPendulum import InvertedPendulum4DOF
from controller.PIDController import PIDController
from controller.MPCController import MPCController
from controller.RFPTController import RFPTController

import json
import numpy as np

def read_config(file):
    ''' Read config file.

    :param file:
    :return:
    '''
    config = {}

    if file == None:
        file = input("Please insert config file!")
    try:
        with open(file, 'r') as f:
            config = json.load(f)

    except:
        print("Config file cannot be read!")
        exit(1)

    return config

def init_entities(config):
    system = None
    controller = None

    dt = config["simulation"]["dt"]

    system_config = config["system"]
    controller_config = config["controller"]

    # init system
    if system_config["name"] == "VDP":
        system = VanDerPolOscillator2DOF(system_config["params"], dt)
    elif system_config["name"] == "IP":
        system = InvertedPendulum4DOF(dt)
    else:
        print("System configuration is not correct!")
        exit(1)

    # init controller
    if controller_config["name"] == "PID":
        controller = PIDController(controller_config["params"], dt)
    elif controller_config["name"] == "MPC":
        controller = MPCController(controller_config["params"], dt)
    elif controller_config["name"] == "RFPT":
        controller = RFPTController(controller_config["params"], dt)

    assert system != None, "System configuration is not correct!"
    assert controller != None, "Controller configuration is not correct!"

    init_state = np.array(system_config["init_state"])

    #TODO implement different options for reference parametrized through json
    reference = np.zeros(system.get_state_dim())

    return system, controller, reference, init_state