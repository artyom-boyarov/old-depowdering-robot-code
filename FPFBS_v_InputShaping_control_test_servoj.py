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


# start_q = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0] # starting joint positions - pos_Q1
# start_q = [np.pi/2, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0] # starting joint positions - pos_Q2
# start_q = [np.pi/2, -np.pi/2, (5*np.pi)/6, -np.pi/2, 0.0, 0.0] # starting joint positions - pos_Q3
Ts = 1/FREQUENCY

# q_0d_out = np.squeeze( q_0d.T ).tolist()
# q_1d_out = np.squeeze( q_1d.T ).tolist()
# q_2d_out = np.squeeze( q_2d.T ).tolist()
# q_3d_out = np.squeeze( q_3d.T ).tolist()
# q_4d_out = np.squeeze( q_4d.T ).tolist()
# q_5d_out = np.squeeze( q_5d.T ).tolist()

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
watchdog.input_int_register_0 = 2
con.send(watchdog)  # sending mode == 2


dt = 1/FREQUENCY  # 500 Hz    # frequency
plotter = True

# ------------------ Control loop initialization -------------------------

t_out = np.arange(0,len(q_0d_out))*dt
trajectory_time = t_out[-1]  # time of min_jerk trajectory
# ----------- minimum jerk preparation -----------------------


if plotter:
    time_plot = []

    base = []
    shoulder = []
    elbow = []

    wrist_1 = []
    wrist_2 = []
    wrist_3 = []

    base_des = []
    shoulder_des = []
    elbow_des = []

    wrist_1_des = []
    wrist_2_des = []
    wrist_3_des = []

    x = []
    y = []
    z = []

    ax = []
    ay = []
    az = []

#   -------------------------Control loop --------------------
state = con.receive()
tcp = state.actual_TCP_pose
t_current = 0
counter = 0
t_start = time.time()
q_ref = start_q # vector (list) we use to store signals before they get sent out

while time.time() - t_start < trajectory_time:
    t_init = time.time()
    state = con.receive()
    t_prev = t_current
    t_current = time.time() - t_start

    # print(f"dt:{t_current-t_prev}")
    # read state from the robot
    if state.runtime_state > 1:
        #   ----------- minimum_jerk trajectory --------------
        if counter <= len(q_0d_out)-1:
            q_ref[0] = q_0d_out[counter] # add the initial position of the **commanded joint** to the reference trajectory
            q_ref[1] = q_1d_out[counter]
            q_ref[2] = q_2d_out[counter]
            q_ref[3] = q_3d_out[counter]
            q_ref[4] = q_4d_out[counter]
            q_ref[5] = q_5d_out[counter]

        # ------------------ impedance -----------------------
        current_pose = state.actual_TCP_pose
        current_speed = state.actual_TCP_speed
        current_joints_position = state.actual_q
        current_tool_acc = state.actual_tool_accelerometer

        joints = q_ref

        list_to_setp(setp, joints)
        con.send(setp)

        if plotter:
            time_plot.append(time.time() - t_start)

            base.append(current_joints_position[0])
            shoulder.append(current_joints_position[1])
            elbow.append(current_joints_position[2])

            wrist_1.append(current_joints_position[3])
            wrist_2.append(current_joints_position[4])
            wrist_3.append(current_joints_position[5])

            base_des.append(q_ref[0])
            shoulder_des.append(q_ref[1])
            elbow_des.append(q_ref[2])

            wrist_1_des.append(q_ref[3])
            wrist_2_des.append(q_ref[4])
            wrist_3_des.append(q_ref[5])

            tcp1 = state.actual_TCP_pose
            x.append(tcp1[0])
            y.append(tcp1[1])
            z.append(tcp1[2])

            ax.append(current_tool_acc[0])
            ay.append(current_tool_acc[1])
            az.append(current_tool_acc[2])
        counter += 1

print(f"It took {time.time()-t_start}s to execute the servoJ")
print(f"time needed for min_jerk {trajectory_time}s\n")

state = con.receive()
print('--------------------\n')
print(state.actual_TCP_pose)

# ====================mode 3===================
watchdog.input_int_register_0 = 3
con.send(watchdog)


con.send_pause()
con.disconnect()

# saving to csv
field = ['time', 'desired_base_joint_pos', 'base_joint_pos', 'desired_shoulder_joint_pos', 'shoulder_joint_pos', \
    'desired_elbow_joint_pos', 'elbow_joint_pos', 'x_out', 'y_out', 'z_out', 'tool_acc_x', 'tool_acc_y', 'tool_acc_z']

filename = 'input_shaping_trouble_shooting_test/Experimental_results/' + filename_load.split('/')[-1]
filename = filename.replace('.','_')
with open(filename+'_measurements.csv', 'w', newline='') as f:
    write = csv.writer(f)

    write.writerow(field)

    for i in range(0,len(time_plot)):
        write.writerow([time_plot[i], base_des[i], base[i], shoulder_des[i], shoulder[i], elbow_des[i], elbow[i], x[i], y[i], z[i], ax[i], ay[i], az[i]])

if plotter:
    # ----------- position -------------
    plt.figure()
    plt.plot(time_plot, base_des, label="commanded")
    plt.plot(time_plot, base, label="output")
    plt.legend()
    plt.grid()
    plt.ylabel('Base Joint Position [rad]')
    plt.xlabel('Time [sec]')

    plt.figure()
    plt.plot(time_plot, shoulder_des, label="commanded")
    plt.plot(time_plot, shoulder, label="output")
    plt.legend()
    plt.grid()
    plt.ylabel('Shoulder Joint Position [rad]')
    plt.xlabel('Time [sec]')

    plt.figure()
    plt.plot(time_plot, elbow_des, label="commanded")
    plt.plot(time_plot, elbow, label="output")
    plt.legend()
    plt.grid()
    plt.ylabel('Elbow Joint Position [rad]')
    plt.xlabel('Time [sec]')

    # plt.figure()
    # plt.plot(time_plot, shoulder_des, label="desired")
    # plt.plot(time_plot, shoulder, label="output")
    # plt.legend()
    # plt.grid()
    # plt.ylabel('Shoulder Joint Position [rad]')
    # plt.xlabel('Time [sec]')

    # plt.figure()
    # plt.plot(time_plot, elbow_des, label="desired")
    # plt.plot(time_plot, elbow, label="output")
    # plt.legend()
    # plt.grid()
    # plt.ylabel('Elbow Joint Position [rad]')
    # plt.xlabel('Time [sec]')

    # plt.figure()
    # plt.plot(time_plot, wrist_1_des, label="desired")
    # plt.plot(time_plot, wrist_1, label="output")
    # plt.legend()
    # plt.grid()
    # plt.ylabel('Wrist 1 Joint Position [rad]')
    # plt.xlabel('Time [sec]')

    # plt.figure()
    # plt.plot(time_plot, wrist_2_des, label="desired")
    # plt.plot(time_plot, wrist_2, label="output")
    # plt.legend()
    # plt.grid()
    # plt.ylabel('Wrist 2 Joint Position [rad]')
    # plt.xlabel('Time [sec]')

    # plt.figure()
    # plt.plot(time_plot, wrist_3_des, label="desired")
    # plt.plot(time_plot, wrist_3, label="output")
    # plt.legend()
    # plt.grid()
    # plt.ylabel('Wrist 3 Joint Position [rad]')
    # plt.xlabel('Time [sec]')

    # ----------- tool acceleration -------------
    plt.figure()
    plt.plot(time_plot, ax)
    plt.grid()
    plt.ylabel('Tool Acceleration (X) [m/s^2]')
    plt.xlabel('Time [sec]')

    plt.figure()
    plt.plot(time_plot, ay)
    plt.grid()
    plt.ylabel('Tool Acceleration (Y) [m/s^2]')
    plt.xlabel('Time [sec]')

    plt.figure()
    plt.plot(time_plot, az)
    plt.grid()
    plt.ylabel('Tool Acceleration (Z) [m/s^2]')
    plt.xlabel('Time [sec]')
    plt.show()