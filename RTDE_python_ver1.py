#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# All rights reserved.

import sys

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import numpy as np

# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = "169.254.160.217"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

keep_running = True

#function of converting setp to list
def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list

#function of converting list to setp
def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# Setpoints to move the robot to
# original test for robot moving
# setp1 = [0.4, 0.3, 0.5, np.pi, 0, np.pi/2]
# setp2 = [0.4, -0.3, 0.5, np.pi, 0, np.pi/2]
# setp3 = [0.3, -0.3, 0.5, np.pi, 0, np.pi/2]
# setp4 = [0.3, 0.3, 0.5, np.pi, 0, np.pi/2]    

#test1 rectangular within the box
setp1 = [-0.424, -0.287, 0.121, 3.068, 0.549, -0.008]
setp2 = [-0.552, -0.287, 0.121, 3.068, 0.549, -0.008]
setp3 = [-0.552, -0.181, 0.121, 3.068, 0.549, -0.008]
setp4 = [-0.426, -0.181, 0.121, 3.068, 0.549, -0.008]


setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0

# start data synchronization
if not con.send_start():
    sys.exit()

# control loop
move_completed = True
move_loop_num = 2
move_loop_flag = 0
print("waiting")
while keep_running:
    # receive the current state
    state = con.receive()

    if state is None:
        break
    

    # do something...
    if move_completed and state.output_int_register_0 == 1:
        move_completed = False
        #control the robot move according to the sequence: setp1->setp2->setp3->setp4->setp1->......
        if setp_to_list(setp) == setp1:
            new_setp = setp2
        elif setp_to_list(setp) == setp2:
            new_setp = setp3
        elif setp_to_list(setp) == setp3:
            move_loop_flag += 1
            new_setp = setp4
        else:
            new_setp = setp1

        list_to_setp(setp, new_setp)
        print("New pose = " + str(new_setp))
        # send new setpoint
        con.send(setp)
        watchdog.input_int_register_0 = 1
    elif not move_completed and state.output_int_register_0 == 0:
        print("Move to confirmed pose!")
        move_completed = True
        watchdog.input_int_register_0 = 0


    # kick watchdog
    con.send(watchdog)
    #check if the robot finishes the num of loop we set
    if move_loop_flag == move_loop_num and move_completed == True:
        #watchdog.input_int_register_0 = 2
        #con.send(watchdog)
        break

    #setp = setp_to_list(list1)
    #con.send(watchdog)

con.send_pause()

con.disconnect()