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

ROBOT_HOST = "169.254.52.225"
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

#generate path for cube
def cube_dp_path(center,length,loop):
    path_list = []
    #point1 = center
    #point1[2] = 0.39
    point0 = [center[0],center[1],0.39,center[3],center[4],center[5]]#0.39 is to make sure the robot will get into the box from upside
    path_list.append(point0)
    #path_list
    for i in range(loop):

        point1 = [center[0],center[1],center[2],center[3],center[4],center[5]]
        path_list.append(point1)

        point2 = [center[0] - length[0]/2.0,center[1],center[2],center[3],center[4],center[5]]
        path_list.append(point2)

        point3 = [center[0] - length[0]/2.0,center[1] + length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point3)

        point4 = [center[0],center[1] + length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point4)

        point5 = [center[0] + length[0]/2.0,center[1] + length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point5)

        point6 = [center[0] + length[0]/2.0,center[1],center[2],center[3],center[4],center[5]]
        path_list.append(point6)

        point7 = [center[0] + length[0]/2.0,center[1] - length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point7)

        point8 = [center[0],center[1] - length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point8)

        point9 = [center[0] - length[0]/2.0,center[1] - length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point9)

        path_list.append(point2)
        path_list.append(point1)
    
    path_list.append(point0)

    return path_list
    
def cube_dp_path_ver2(center,length,loop):
    path_list = []
    #point1 = center
    #point1[2] = 0.39
    #point0 = [center[0],center[1],0.39,center[3],center[4],center[5]]
    for i in range(loop):
        #point2 = center
        #print(point2)
        # point2[0] = center[0] - length/2.0
        # point2[1] = center[1] + length/2.0
        point1 = [center[0],center[1],center[2],center[3],center[4],center[5]]
        path_list.append(point1)

        point2 = [center[0] - length[0]/2.0,center[1],center[2],center[3],center[4],center[5]]
        path_list.append(point2)

        point3 = [center[0] - length[0]/2.0,center[1] + length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point3)

        point4 = [center[0],center[1] + length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point4)

        point5 = [center[0] + length[0]/2.0,center[1] + length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point5)

        point6 = [center[0] + length[0]/2.0,center[1],center[2],center[3],center[4],center[5]]
        path_list.append(point6)

        point7 = [center[0] + length[0]/2.0,center[1] - length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point7)

        point8 = [center[0],center[1] - length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point8)

        point9 = [center[0] - length[0]/2.0,center[1] - length[1]/2.0,center[2],center[3],center[4],center[5]]
        path_list.append(point9)

        path_list.append(point2)
        path_list.append(point1)
    
    #path_list.append(point0)

    return path_list


def get_whole_path(centers,lens,loops):
    septs = []
    for i in range(len(centers)):
        septs.extend(cube_dp_path_ver2(centers[i],lens[i],loops[i]))
    return septs


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

#septs = [[-0.438, -0.091, 0.39, 3.029, 1.111, 0.0]] #version 1(without enclosures)
septs = [[-0.449, -0.05146, 0.440, 3.029, 1.111, 0.0]]
dheight = 0.060
# objects_center0 = [[-0.426, -0.246, dheight, 3.029, 1.111, 0.051]]#suppose we fix the height and change the air flow
# lens0 = [[0.102,0.022]]#change the range to a bigger one ##height 60cm
# loops = [1]#set 1 is not enough, 2-3 is good, 3-4 will depowder too much

# objects_center1 = [[-0.378, -0.237, dheight, 3.029, 1.111, 0.051]]#suppose we fix the height and change the air flow
# lens1 = [[0.018,0.018]]#change the range to a bigger one ##height 60cm
# loops = [1]#set 1 is not enough, 2-3 is good, 3-4 will depowder too much

objects_center2 = [[-0.427, -0.246, dheight, 3.029, 1.111, 0.051]]#suppose we fix the height and change the air flow
lens2 = [[0.018,0.018]]#change the range to a bigger one ##height 60cm
loops = [1]#set 1 is not enough, 2-3 is good, 3-4 will depowder too much

# objects_center3 = [[-0.474, -0.243, dheight, 3.029, 1.111, 0.051]]#suppose we fix the height and change the air flow
# lens3 = [[0.018,0.018]]#change the range to a bigger one ##height 60cm
# loops = [1]#set 1 is not enough, 2-3 is good, 3-4 will depowder too much
# septs_medium0 = get_whole_path(objects_center0,lens0,loops)
# septs_medium1 = get_whole_path(objects_center1,lens1,loops)
septs_medium2 = get_whole_path(objects_center2,lens2,loops)
# septs_medium3 = get_whole_path(objects_center3,lens3,loops)
# septs.extend(septs_medium0)
# septs.extend(septs_medium1)
septs.extend(septs_medium2)
# septs.extend(septs_medium3)

#septs.extend([[-0.246, -0.056, 0.380, 3.029, 1.111, 0.0]]) #version 1(without enclosure)

#septs.extend([[-0.449, -0.05146, 0.440, 3.029, 1.111, 0.0]])# with enclosure
septs.extend([[-0.50671, -0.05830, 0.440, 3.029, 1.111, 0.0]])# with enclosure

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
move_loop_flag = 0
point_num = len(septs)
i_index = 0
print("waiting")
while keep_running:
    # receive the current state
    state = con.receive()

    if state is None:
        break
    
    

    # do something...
    if move_completed and state.output_int_register_0 == 1:
        if i_index >= point_num:
            break
        move_completed = False
        
        new_setp = septs[i_index]

        list_to_setp(setp, new_setp)
        print("New pose = " + str(new_setp))
        # send new setpoint
        con.send(setp)
        i_index += 1
        watchdog.input_int_register_0 = 1
    elif not move_completed and state.output_int_register_0 == 0:
        print("Move to confirmed pose!")
        move_completed = True
        watchdog.input_int_register_0 = 0


    # kick watchdog
    con.send(watchdog)
    #check if the robot finishes the num of loop we set
    # if move_loop_flag == move_loop_num and move_completed == True:
    #     #watchdog.input_int_register_0 = 2
    #     #con.send(watchdog)
    #     break

con.send_pause()

con.disconnect()