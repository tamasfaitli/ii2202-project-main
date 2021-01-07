# system imports
import sys
import os
from shutil import copyfile
import timeit
from datetime import datetime

from dataclasses import dataclass

# json for reading config file
import json

# scientific libraries
import numpy as np
import matplotlib.pyplot as plt

import utils.plot_data as mplt


# debug switch, if its true then debug mode is ON
#   e.g. don't save files unnecessarily
SWITCH_DBG          = True

CONST_OUT_FOLDER    = "output-data"
CONST_NAME          = "rfpt-ip"

@dataclass
class RFPTController:
    _l: float
    _D: float
    _A: float
    x_star: float = 0.0

@dataclass
class InvertedPendulum:
    _g: float
    _m: float
    _M: float
    _I: float
    _l: float
    _bc: float
    _bp: float

@dataclass
class SimulationTime:
    _dt: float
    _T: float


def read_config():
    ''' Read config file passed as system argument or user input.

    :param file:
    :return:
    '''
    # config file is passed as first argument
    if len(sys.argv) > 1:
        file = sys.argv[1]
    else:
        file = input("Please insert config file!")

    config = {}

    try:
        with open(file, 'r') as f:
            config = json.load(f)

    except:
        print("Config file cannot be read!")
        exit(1)

    return file, config

def get_time_stamp():
    ''' Generate time stamp

    :return: string : time stamp
    '''
    time = datetime.now()
    time_stamp_data = [str(x) for x in [time.year, time.month, time.day, time.hour, time.minute, time.second]]
    time_stamp = "-".join(time_stamp_data)

    return time_stamp


def create_output_folder(conf_file):
    # generate path using timestamp
    time_stamp = get_time_stamp()
    path = os.path.join(CONST_OUT_FOLDER, CONST_NAME, time_stamp)

    # create folder
    if not os.path.exists(path):
        os.makedirs(path)

    # copy config file
    copyfile(conf_file, os.path.join(path,"config.json"))

    return path

def init_main(config):
    controller = RFPTController(*list(config["controller"].values()))
    system = InvertedPendulum(*list(config["system"].values()))
    approx_system = InvertedPendulum(*list(config["approx_system"].values()))
    sim_time = SimulationTime(*list(config["simulation"]["time"].values()))

    init_state = config["simulation"]["init_state"]
    reference = np.loadtxt(config["simulation"]["reference"])


    return controller, system, approx_system, sim_time, init_state, reference

def kinematic_block(ctrl, err, derr, ierr, des):
    return  des + ((ctrl._l**3)*ierr) + (3*(ctrl._l**2)*err) + (3*ctrl._l*derr)


def def_inner_function(ctrl, value):
    return (value / 2) + ctrl._D

def deformation_block(ctrl, ddq_real, ddq_des, ddq_def_prev, q_star):
    h = ddq_real-ddq_des
    h_norm = np.linalg.norm(h)
    if h_norm > 1e-10:
        return (def_inner_function(ctrl, ctrl._A*h_norm+q_star)-q_star)*(h/h_norm)+ddq_def_prev
    else:
        return ddq_def_prev

def inverse_dynamics_model(model, q, dq, ddq_des):
    '''

    :param model:   The approximated system parameters.
    :param ddq_des:
    :return:
    '''
    return (model._M+model._m)*ddq_des[0]-model._m*model._l*ddq_des[1]*np.cos(q[1])+ \
        model._m*model._l*(dq[1]**2)*np.sin(q[1])+model._bc*dq[0]


def real_dynamics(sys, q, dq, Q):
    '''

    :param sys:
    :param q:
    :param dq:
    :param Q:
    :return:
    '''

    ddq = np.zeros(2)

    # dist term would be: (1/(sys._M+sys._m-((sys._m**2)*(sys._l**2)*(np.cos(q[1])**2)/(sys._I+sys._m*(sys._l**2)))))* \
    #              (Q + (((sys._m*(sys._l**2)*(np.cos(q[1])**2)*q[1])/(sys._I+sys._m*(sys._l**2)))-1))

    ddq[0] = (1/(sys._M+sys._m-((sys._m**2)*(sys._l**2)*(np.cos(q[1])**2)/(sys._I+sys._m*(sys._l**2))))) * \
             (Q-sys._bc*dq[0] - sys._m*sys._l*(dq[1]**2)*np.sin(q[1]) +
             (((sys._m**2)*(sys._l**2)*sys._g*np.sin(q[1])*np.cos(q[1]))/(sys._I+(sys._m*(sys._l**2)))) -
             ((sys._m*sys._l*sys._bp*dq[1]*np.cos(q[1]))/(sys._I+(sys._m*(sys._l**2)))))

    ddq[1] = ((1)/(sys._I+sys._m*sys._l**2+(sys._m**2*sys._l**2*np.cos(q[1])**2)/(sys._M+sys._m)))* \
             ((sys._m*sys._l*np.cos(q[1])/(sys._M+sys._m))*Q - sys._bp*dq[1] + sys._m*sys._g*sys._l*np.sin(q[1]) -
              (sys._m**2*sys._l**2*dq[1]**2*np.sin(q[1])*np.cos(q[1])/(sys._M+sys._m))-
              (sys._m*sys._l*sys._bc*dq[0]*np.cos(q[1])/(sys._M+sys._m)))

    return ddq


def exec_simulation(ctrl, sys, model, sim_time, init_state, reference):
    ''' Execute simulation for the given time with given delta time.
        For simplicity reference is a constant position value, with zero velocity.
        1. Initialize memory to save results.
        2. For t in T:
            - observe system
            - execute controller
            - update system
            - store data

    :param ctrl:
    :param sys:
    :param model:
    :param sim_time:
    :param init_state:
    :param ref:
    :return:
    '''

    # calculate the fixed point of the inner function by iteration
    q_star = 0.0
    for i in range(200):
        q_star = def_inner_function(ctrl, q_star)


    N   = int((sim_time._T/sim_time._dt)+1)

    # initialize memory
    q       = np.zeros((2,N))   # state
    dq      = np.zeros((2,N))   # \dot state
    ddq     = np.zeros((2,N))   # \ddot state
    def_mem = np.zeros((2,N))
    des_mem = np.zeros((2,N))
    ref     = np.zeros((2,N))   # reference
    dref    = np.zeros((2,N))
    ddref   = np.zeros((2,N))
    Q       = np.zeros(N)       # control (action)
    time    = np.zeros(N)       # simulation time
    T       = np.zeros(N)       # computation time

    # set init state
    q[:,0]  = np.array((init_state[0],init_state[2]))
    dq[:,0] = np.array((init_state[1],init_state[3]))

    ierror = 0.0
    ddq_def = 0.0

    for t in range(1, N-1):
        time[t] = time[t-1] + sim_time._dt
        # constant reference signal
        ref[0,t]    = reference[t,1]
        dref[0,t]   = reference[t,2]
        ddref[0,t]  = reference[t,3]

        # start timer for controller evaluation
        start = timeit.default_timer()

        # calculate error terms
        error   = q[:,t] - ref[:,t]
        derror  = dq[:,t] - dref[:,t]
        ierror  += error * sim_time._dt

        # calculate the necessary effect we need to achieve on the system
        ddq_des = kinematic_block(ctrl, error, derror, ierror, ddref[:,t])
        des_mem[:,t] = ddq_des

        # apply the adaptive deformation block
        ddq_def = deformation_block(ctrl, ddq[:,t-1], ddq_des, ddq_def, q_star)
        def_mem[:,t] = ddq_def

        # calculate required forces based on model
        Q[t] = inverse_dynamics_model(model, q[:,t], dq[:,t], ddq_def)

        # stop timer for controller evaluation
        T[t] = timeit.default_timer()-start

        # apply the control action on the system and calculate its realised
        # response using the real system parameters
        ddq[:, t] = real_dynamics(sys, q[:,t], dq[:,t], Q[t])

        # euler integration
        dq[:, t+1] = dq[:, t] + sim_time._dt * ddq[:, t]
        q[:, t+1] = q[:,t] + sim_time._dt * dq[:,t]

    # time[-1] = time[-2] + sim_time._dt


    simulation_data = {'q': q, 'dq': dq, 'ddq': ddq, 'ref': ref, 'dref': dref, 'ddref': ddref, 'def_mem': def_mem,
                       'des_mem': des_mem, 'Qaction': Q, 'time': time, 'T': T}

    return simulation_data

def write_out_data(path, data):
    for d in data:
        try:
            file_path = os.path.join(path, "data_"+str(d))
            np.save(file_path, data[d])
        except Exception:
            print("Could not write data " + str(d) + " !!!")


def main():
    ''' Main function, it executes the following steps:
        1. Read config file (contains controller parameters, system parameters etc..)
        2. Create output folder based on timestamp, and copy config file there for traceability
        3. Initiate controller, system, reference trajectory
        4. Execute simulation
        5. Export data into output folder
    :return: None
    '''
    # read config file
    conf_file, config = read_config()

    # init controller, system, reference trajectory
    controller, real_system, system_model, sim_time, init_state, reference \
        = init_main(config)

    data = exec_simulation(controller, real_system, system_model, sim_time, init_state, reference)

    if not SWITCH_DBG:
        folder = create_output_folder(conf_file)
        write_out_data(folder, data)


    if SWITCH_DBG:
        # mplt.subplot_results(data['time'], data['q'], data['dq'], data['ddq'], data['ref'], data['Qaction'])
        mplt.subplot_pos_force(data['time'], data['q'], data['ref'], data['Qaction'])
        mplt.plot_ddq(data['time'], data['ddq'], data['des_mem'], data['def_mem'])

if __name__ == "__main__":
    # Entry point of script executed on its own
    main()