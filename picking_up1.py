#!/usr/bin/env python

"""basics6_gripper_control.py

This tutorial does position and force control (if available) for grippers supported by Flexiv.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
from utility import quat2eulerZYX
from utility import parse_pt_states
from utility import list2str
import matplotlib.pyplot as plt
import numpy as np
import math
# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on

# Global flag: whether the gripper control tasks are finished
g_is_done = False
global states_logs
states_logs = []
global pull_out_force_logs
pull_out_force_logs = []

def print_description():
    """
    Print tutorial description.

    """
    print("This tutorial does position and force control (if available) for grippers "
          "supported by Flexiv.")
    print()


def print_gripper_states(robot, gripper, log):
    """
    Print gripper states data @ 1Hz.

    """
    # Data struct storing gripper states
    gripper_states = flexivrdk.GripperStates()
    robot_states = flexivrdk.RobotStates()


    while (not g_is_done):
        # Get the latest gripper states
        gripper.getGripperStates(gripper_states)
        robot.getRobotStates(robot_states)
        # Print all gripper states, round all float values to 2 decimals
        log.info("Current gripper states:")
        print("width: ", round(gripper_states.width, 2))
        print("force: ", round(gripper_states.force, 2))
        print("max_width: ", round(gripper_states.maxWidth, 2))
        print("is_moving: ", gripper_states.isMoving)
        log.info("Current TCP force:")
        # fmt: off
        print("F_ext_tcp_frame: ", ['%.2f' % i for i in robot_states.extWrenchInTcp])
        print("F_ext_base_frame: ", ['%.2f' % i for i in robot_states.extWrenchInBase])
        states_logs.append(round(gripper_states.force, 2))
        x_axis = float(robot_states.extWrenchInBase[0])
        y_axis = float(robot_states.extWrenchInBase[1])
        z_axis = float(robot_states.extWrenchInBase[2])
        total_force = math.sqrt(x_axis**2+y_axis**2+(z_axis)**2)-0.2
        pull_out_force_logs.append(round(total_force, 2))

        time.sleep(1)


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument('robot_ip', help='IP address of the robot server')
    argparser.add_argument('local_ip', help='IP address of this PC')
    args = argparser.parse_args()

    # Define alias
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    # Print description
    log.info("Tutorial description:")
    print_description()

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            robot.clearFault()
            time.sleep(2)
            # Check again
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Wait for the robot to become operational
        seconds_waited = 0
        while not robot.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please check that the robot 1) "
                    "has no fault, 2) is in [Auto (remote)] mode")

        log.info("Robot is now operational")

        # Gripper Control
        # ==========================================================================================
        # Gripper control is not available if the robot is in IDLE mode, so switch to some mode
        # other than IDLE
        robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)
        #robot.executePlan("PLAN-Home")
        time.sleep(1)

        # Instantiate gripper control interface
        gripper = flexivrdk.Gripper(robot)

        # Thread for printing gripper states
        print_thread = threading.Thread(
            target=print_gripper_states, args=[robot, gripper, log])
        print_thread.start()

        # print_thread2 = threading.Thread(
        #     target=print_robot_states, args=[robot, log])

        # gripper.grasp(2.0)
        # time.sleep(10)
        #gripper.move(0.02, 0.1, 3)##(grip_Width, grip_Velocity, grip_Force)

        # alltargets = ["0.5588 0.0388 0.084 -7.75 -180 6.13 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.08"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        
        alltargets = ["0.5588 0.0488 0.0760 -7.75 -180 6.13 WORLD WORLD_ORIGIN"]
        allwaypoints = ["0.2721 -0.2734 0.4416 -179.36 0 142.61 WORLD WORLD_ORIGIN 0.4944 0.023 0.3872 -3.59 180 5.45 WORLD WORLD_ORIGIN"]
        allmaxVel = ["0.1"]
        log.info("Executing primitive: MoveL")
        for i in range(len(alltargets)):
            robot.executePrimitive(
                "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
            while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
                time.sleep(1)


        ### Close gripper to grab the object
        # log.info("Closing gripper") close gripper to grab
        # gripper.move(0.0066, 0.1, 4)##(grip_Width, grip_Velocity, grip_Force)
        # time.sleep(2)
        
        #np.savetxt('/home/phantom/Pictures/force_control_experiments/fragile_metal_1_0mm/set_force='+str(force_in)+'(with_depowder).txt', states_logs)
        
        #object_name = 'fragile_metal_1_0mm'
        #force_in = np.loadtxt('/home/phantom/Pictures/force_control_experiments/'+object_name+'/grasp_force/grasp_force.txt')
        force_in = 5.0

        gripper.grasp(force_in)

        #gripper.move(0.016, 0.009, 4)
        time.sleep(3)
        #gripper.grasp(force_in)

        alltargets = ["0.4944 0.023 0.3872 -3.59 180 5.45 WORLD WORLD_ORIGIN"]
        allmaxVel = ["0.1"]
        log.info("Executing primitive: MoveL")
        for i in range(len(alltargets)):

            #gripper.grasp(force_in)
            

            robot.executePrimitive(
                "MoveL(target="+alltargets[i]+", maxVel="+ allmaxVel[i] +")")
            while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
                time.sleep(1)

        time.sleep(3)
        
        # if len(num_logs) != len(states_logs):
        #     del num_logs[-1]
        
        ##########record gripper force
        states_len = len(states_logs)
        num_logs = list(range(0,states_len))
        print(len(num_logs))
        print(len(states_logs))
        agv_force = (states_logs[-1]+states_logs[-2]+states_logs[-3]+states_logs[-4]+states_logs[-5])/5.0
        print("average picking up force:",agv_force)
        #if len(num_logs) != len(states_logs):
            #del states_logs[-1]
        states_logs1 = states_logs[:len(num_logs)]
        plt.figure(1)
        plt.plot(num_logs,states_logs1,marker='o')
        plt.xlabel("Time(s)")
        plt.ylabel("Force(N)")
        plt.title("force vs time(set force="+str(force_in)+")")
        plt.xlabel('average picking up force')
        plt.savefig('/home/phantom/Pictures/force_control_experiments/fragile_metal_1_0mm/break_object/set_force='+str(force_in)+'(with_depowder).png')
        np.savetxt('/home/phantom/Pictures/force_control_experiments/fragile_metal_1_0mm/break_object/set_force='+str(force_in)+'(with_depowder).txt', states_logs)
        
        
        ##########record pulling out force
        pull_out_force_logs_len = len(pull_out_force_logs)
        num_logs = list(range(0,pull_out_force_logs_len))
        print(len(num_logs))
        print(len(pull_out_force_logs))
        fig = plt.figure()
        #if len(num_logs) != len(pull_out_force_logs):
            #del pull_out_force_logs[-1]
        pull_out_force_logs1 = pull_out_force_logs[:len(num_logs)]
        plt.figure(2)
        plt.plot(num_logs,pull_out_force_logs1,marker='o')
        plt.xlabel("Time(s)")
        plt.ylabel("Force(N)")
        plt.title("External force on TCP(set force="+str(force_in)+")")
        plt.savefig('/home/phantom/Pictures/force_control_experiments/fragile_metal_1_0mm/break_object/pull_out_force/set_force='+str(force_in)+'(with_depowder).png')
        np.savetxt('/home/phantom/Pictures/force_control_experiments/fragile_metal_1_0mm/break_object/pull_out_force/set_force='+str(force_in)+'(with_depowder).txt', pull_out_force_logs)
        ##plt.savefig('/home/phantom/Pictures/force_control_experiments/fragile_metal_1_0mm/set_force='+str(force_in)+'(with_depowder)3.png')
        ##np.savetxt('/home/phantom/Pictures/force_control_experiments/fragile_metal_1_0mm/set_force='+str(force_in)+'(with_depowder)3.txt', pull_out_force_logs)
        alltargets = ["0.3329 -0.4695 0.0689 -176.39 0 124.29 WORLD WORLD_ORIGIN"]
        allwaypoints = ["0.4944 0.023 0.3872 -3.59 180 3.45 WORLD WORLD_ORIGIN 0.2721 -0.2734 0.4416 -179.36 0 142.61 WORLD WORLD_ORIGIN"]
        allmaxVel = ["0.1"]
        log.info("Executing primitive: MoveL")
        for i in range(len(alltargets)):
            robot.executePrimitive(
                "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
            while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
                time.sleep(1)
        
        # ###########  Check if force is enough:
        # gripper_states_1 = flexivrdk.GripperStates()
        # gripper.getGripperStates(gripper_states_1)
        # if round(gripper_states_1.width, 2) < 0.005:
        #     print("force_in is:",force_in)
        #     force_in += 0.5
        #     force_in_list = [force_in]
        #     np.savetxt('/home/phantom/Pictures/force_control_experiments/'+object_name+'/grasp_force/grasp_force.txt', force_in_list)

        # #Go for blowing
        # alltargets = ["0.6859 -0.1261 0.3881 85.66 94.31 82.05 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.4944 0.023 0.3872 -3.59 180 3.45 WORLD WORLD_ORIGIN 0.6259 -0.0923 0.2981 10.49 116.68 12.90 WORLD WORLD_ORIGIN "]
        # allmaxVel = ["0.04"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)

        # #start rotating
        # ####first half 
        # alltargets = ["0.6859 -0.1261 0.3881 91.65 -78.65 84.75 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6859 -0.1261 0.3881 90.33 0.36 86.38 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.1"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        # time.sleep(3)
        # ####second half 
        # alltargets = ["0.6859 -0.1261 0.3881 85.66 94.31 82.05 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6859 -0.1261 0.3881 90.33 0.36 86.38 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.1"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        # time.sleep(3)
        # ####third half 
        # alltargets = ["0.6859 -0.1261 0.3881 91.65 -98.65 84.75 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6859 -0.1261 0.3881 89.67 -177.0 86.39 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.1"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        # time.sleep(3)

        # ####fourth half 
        # alltargets = ["0.6859 -0.1261 0.3881 85.66 94.31 82.05 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6859 -0.1261 0.3881 89.67 -177.0 86.39 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.1"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        # time.sleep(3)

        # alltargets = ["0.6859 -0.1261 0.3881 91.65 -78.65 84.75 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6859 -0.1261 0.3881 90.33 0.36 86.38 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.1"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        # time.sleep(3)
        # ####second half 
        # alltargets = ["0.6859 -0.1261 0.3881 85.66 94.31 82.05 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6859 -0.1261 0.3881 90.33 0.36 86.38 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.1"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        # time.sleep(3)
        # ####third half 
        # alltargets = ["0.6859 -0.1261 0.3881 91.65 -98.65 84.75 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6859 -0.1261 0.3881 89.67 -177.0 86.39 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.1"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        # time.sleep(3)

        # ####fourth half 
        # alltargets = ["0.6859 -0.1261 0.3881 85.66 94.31 82.05 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6859 -0.1261 0.3881 89.67 -177.0 86.39 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.1"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)
        # time.sleep(3)


        # alltargets = ["0.3329 -0.4695 0.0689 -176.39 0 124.29 WORLD WORLD_ORIGIN"]
        # allwaypoints = ["0.6259 -0.0923 0.2981 10.49 116.68 12.90 WORLD WORLD_ORIGIN 0.4944 0.023 0.3872 -3.59 180 3.45 WORLD WORLD_ORIGIN 0.2721 -0.2734 0.4416 -179.36 0 142.61 WORLD WORLD_ORIGIN"]
        # allmaxVel = ["0.05"]
        # log.info("Executing primitive: MoveL")
        # for i in range(len(alltargets)):
        #     robot.executePrimitive(
        #         "MoveL(target="+alltargets[i]+", waypoints="+allwaypoints[i]+", maxVel="+ allmaxVel[i] +")")
        #     while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
        #         time.sleep(1)



        ### Open gripper to grab the object
        force_in = -0.8
        gripper.grasp(force_in)
        time.sleep(5)

        gripper.move(0.03, 0.1, 4)##(grip_Width, grip_Velocity, grip_Force)
        time.sleep(2)
        
        # ### Open gripper to grab the object
        # log.info("Opening gripper")
        # gripper.move(0.02, 0.1, 5)
        # time.sleep(2)
        # Finished, exit all threads
        np.savetxt('/home/phantom/Pictures/force_control_experiments/fragile_metal_1_0mm/1_5.txt', states_logs)
        gripper.stop()
        global g_is_done
        g_is_done = True
        log.info("Program finished")
        print_thread.join()
        #robot.stop()
    except Exception as e:
        log.error(str(e))

    

if __name__ == "__main__":
    main()