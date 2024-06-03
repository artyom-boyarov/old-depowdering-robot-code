"""
Code written based on servoj example from: https://github.com/davizinho5/RTDE_control_example
Edited by Nosa Edoimioya
10/2/2022
"""
import sys
sys.path.append('')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

import time
import copy
import numpy as np
import scipy.io as sio
from control.matlab import *
import csv
from matplotlib import pyplot as plt
from generate_sine_sweep_var_amp import TableTrajectoryVarying


# -------- functions -------------
def setp_to_list(setp):
    temp = []
    for i in range(0, 6):
        temp.append(setp.__dict__["input_double_register_%i" % i])
    return temp


def list_to_setp(setp, list):
    for i in range(0, 6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

FREQUENCY = 500  # send data in 500 Hz instead of default 125Hz
dt = 1/FREQUENCY  # 500 Hz    # frequency


filename_load = 'input_shaping_trouble_shooting_test/Desired_trajectories/simulink_trajec_ZVD_FBS_FBS_V_5dot625_R_241dot667_vel_2dot4_a11_exp_parameters_04_09_2023.mat'
test = sio.loadmat(filename_load)


# ------------- robot communication stuff -----------------
ROBOT_HOST = '169.254.250.178'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'  # specify xml file for data synchronization

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')  # Define recipe for access to robot output ex. joints,tcp etc.
setp_names, setp_types = conf.get_recipe('setp')  # Define recipe for access to robot input
watchdog_names, watchdog_types= conf.get_recipe('watchdog')

# -------------------- Establish connection--------------------
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
connection_state = con.connect()

# check if connection has been established
while connection_state != 0:
    time.sleep(0.5)
    connection_state = con.connect()
print("---------------Successfully connected to the robot-------------\n")

# get controller version
con.get_controller_version()

# ------------------- setup recipes ----------------------------
con.send_output_setup(state_names, state_types, FREQUENCY)
setp = con.send_input_setup(setp_names, setp_types)  # Configure an input package that the external application will send to the robot controller
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

q_0d = test['q_0d']
q_1d = test['q_1d']
q_2d = test['q_2d']
q_3d = test['q_3d']
q_4d = test['q_4d']
q_5d = test['q_5d']

# print(test.shape)
start_q = [q_0d[0,0], q_1d[0,0], q_2d[0,0], q_3d[0,0], q_4d[0,0], q_5d[0,0]] # starting joint positions - pos_Q0

setp.input_double_register_0 = start_q[0]
setp.input_double_register_1 = start_q[1]
setp.input_double_register_2 = start_q[2]
setp.input_double_register_3 = start_q[3]
setp.input_double_register_4 = start_q[4]
setp.input_double_register_5 = start_q[5]

# Setting the configurable DIO pins
setp.input_int_register_1 = 0

watchdog.input_int_register_0 = 0

# start data synchronization
if not con.send_start():
    sys.exit()

Ts = 1/FREQUENCY

q_0d_out = np.squeeze( q_0d ).tolist()
q_1d_out = np.squeeze( q_1d ).tolist()
q_2d_out = np.squeeze( q_2d ).tolist()
q_3d_out = np.squeeze( q_3d ).tolist()
q_4d_out = np.squeeze( q_4d ).tolist()
q_5d_out = np.squeeze( q_5d ).tolist()

command_idx = 0 # index of joint we're commanding

state = con.receive()
tcp1 = state.actual_TCP_pose
print(tcp1)

#   ------------  mode = 1 (Connection) -----------
while True:
    print('Boolean 1 is False, please click CONTINUE on the Polyscope')
    state = con.receive()
    con.send(watchdog)
    # print(f"runtime state is {state.runtime_state}")
    if state.output_bit_registers0_to_31 == True:
        print('Boolean 1 is True, Robot Program can proceed to mode 1\n')
        break

print("-------Executing moveJ -----------\n")


watchdog.input_int_register_0 = 1
con.send(watchdog)  # sending mode == 1
list_to_setp(setp, start_q)  # changing initial joint positions to setp
con.send(setp) # sending initial joint positions

while True:
    print('Waiting for movej() to finish')
    state = con.receive()
    con.send(watchdog)
    if state.output_bit_registers0_to_31 == False:
        print('Proceeding to mode 2\n')
        break


print("-------Executing servoJ  -----------\n")
watchdog.input_int_register_0 = 3
con.send(watchdog)  # sending mode == 2