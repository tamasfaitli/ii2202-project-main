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

def initiate_reference(sim_config):
    dt = sim_config["dt"]
    T = sim_config["length"]
    dof = sim_config["dof"]

    length = int((T/dt))+1

    reference = np.ones((dof*3, length))

    ref_config = sim_config["reference"]

    if ref_config["type"] == "constant":
        value = np.transpose(np.array(ref_config["params"]["value"]))
        reference *= value
    if ref_config["type"] == "sinusoidal":
        params = ref_config["params"]
        gain   = params["gain"]
        omega  = params["frequency"]
        offset = params["v_offset"]
        phase  = params["h_offset"]

        for t in range(length):
            reference[0,t] = gain*np.sin(omega*(t-phase)) + offset
            reference[1,t] = gain*omega*np.cos(omega*(t-phase))
            reference[2,t] = -gain*(omega**2)*np.sin(omega*(t-phase))

    return T, dt, reference

def init_entities(config):
    system = None
    controller = None

    system_config = config["system"]
    controller_config = config["controller"]

    # parse reference and simulation data
    T, dt, reference = initiate_reference(config["simulation"])

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
        controller = RFPTController(controller_config["params"], system.get_model(), dt)

    assert system != None, "System configuration is not correct!"
    assert controller != None, "Controller configuration is not correct!"

    init_state = np.array(system_config["init_state"])

    return T, dt, system, controller, reference, init_state