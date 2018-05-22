#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 10 19:58:44 2018

@author: greg
"""

#%% Import modules

import vrep
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math
from subprocess import Popen, PIPE


#%% Function to start VREP

def initialise_vrep(scene_name: 'str', 
                    vrep_port: 'int', 
                    headless: 'bool',
                    start_sim: 'bool',
                    sim_time: 'int', 
                    quit_vrep_after_sim: 'bool'):
    
    # Defines path to VREP folder
    path_to_vrep = '/home/greg/Programs/V-REP_PRO_EDU_V3_5_0_Linux'
    path_to_scenes = '../vrep_scenes/test_scenes'
    
    # Converts vrep initialisation settings to arguments recognised by vrep
    head_call = ''
    if headless:
        head_call = '-h'
        
    start_sim_call = ''
    if start_sim:
        if sim_time > 0:
            sim_time_milli = int(sim_time*1000)
            start_sim_call = f'-s{sim_time_milli}'
        elif sim_time == 0:
            start_sim_call = '-s'
        else:
            raise ValueError('''sim_time (simulation duration) must be a 
                             positive number, or zero for no time limit''')
           
    quit_after_sim = ''
    if quit_vrep_after_sim:
        quit_after_sim = '-q'
    
    # close all opened connections to vrep
    vrep.simxFinish(-1) 
    
    # Command-line call to initialise vrep
    Popen(["nice", "-n", "-20", f"{path_to_vrep}/vrep.sh", head_call, start_sim_call, quit_after_sim,
           f"-gREMOTEAPISERVERSERVICE_{vrep_port}_FALSE_FALSE", 
           f"{path_to_scenes}/{scene_name}.ttt"], 
            stdout=PIPE, stderr=PIPE)
    
    # Allow vrep to boot before initialising comms
    time.sleep(5)
    
    # Sets up communication between python and vrep
    clientID = vrep.simxStart('127.0.0.1',vrep_port,True,True,5000,5)
    
    if clientID!=-1:
        print('Connected to remote API server')
    else:
        print('Connection not successful')
        sys.exit('Could not connect')
        
    return clientID


#%% Set ePuck position, orientation and velocity

def set_pos_ang(clientID: 'int',
                handle_robot: 'int',
                position: '[x y z] metres',
                angle: 'rad'):
    
    # Set position
    returnCode = vrep.simxSetObjectPosition(
            clientID, handle_robot, -1, position, vrep.simx_opmode_oneshot)
    
    # Convert angle to euler angle
    eulerAngles = [0, 0, angle]
    
    # Set orientation
    returnCode = vrep.simxSetObjectOrientation(
            clientID, handle_robot, -1, eulerAngles, vrep.simx_opmode_oneshot)
    
    
#%% Set wheel velocities
    
def set_vel(clientID: 'int',
            handle: 'int',
            handle_leftJoint: 'int', 
            handle_rightJoint: 'int',
            linear_velocity: 'm/s',
            angular_velocity: 'rad/sec'):
    
    # ePuck geometric parameters
    wheel_sep = 0.053   # distance between wheels (m)
    wheel_rad = 0.02    # radius of wheels (m)
    
    # Convert to wheel velocities
    vel_r = linear_velocity + wheel_sep*angular_velocity/2
    vel_l = linear_velocity - wheel_sep*angular_velocity/2
    
    # Convert to wheel angular veolocities
    omega_r = vel_r/wheel_rad
    omega_l = vel_l/wheel_rad
    
    returnCode = vrep.simxSetJointTargetVelocity(
                clientID, handle_leftJoint, omega_l, vrep.simx_opmode_streaming)
    
    returnCode = vrep.simxSetJointTargetVelocity(
                clientID, handle_rightJoint, omega_r, vrep.simx_opmode_streaming)
    
#%% Function: move to target
    
def move_to_target(clientID,
                   handle_robot,
                   handle_force_sensor,
                   handle_leftJoint,
                   handle_rightJoint,
                   linear_velocity,
                   target: '[x y] metres'):

#    cmdTimeAll = []
#    forceMagLinkAll = []
    
    returnCode, position = vrep.simxGetObjectPosition(
            clientID, handle_robot, -1, vrep.simx_opmode_buffer)
    
    pos_start_2d = np.array([position[0], position[1]])
    pos_target_2d = np.array([target[0], target[1]])
    dist_to_target = np.linalg.norm(pos_target_2d - pos_start_2d)
    
    time_step = 0.02

    while dist_to_target > 0.02:

        time.sleep(time_step)
        
        returnCode, state, forceVectorLink, torqueVector = vrep.simxReadForceSensor(
                clientID, handle_force_sensor, vrep.simx_opmode_buffer)
        forceMagLink = np.linalg.norm(forceVectorLink)
        
        returnCode, eulerAngles = vrep.simxGetObjectOrientation(
                clientID, handle_robot, -1, vrep.simx_opmode_buffer)
        
        returnCode, position = vrep.simxGetObjectPosition(
                clientID, handle_robot, -1, vrep.simx_opmode_buffer)
        
        xCurrent = position[0]
        yCurrent = position[1]
        
        pos_current_2d = np.array([xCurrent, yCurrent])
        
        xDelta = target[0] - xCurrent
        yDelta = target[1] - yCurrent
        
        phi_current = eulerAngles[2]
        phi_target = math.atan2(yDelta, xDelta)
        phi_delta = phi_current - phi_target
        
        cmdTime = vrep.simxGetLastCmdTime(clientID)
        
        if forceMagLink < 100:
#            forceMagLinkAll.append(forceMagLink)
#            cmdTimeAll.append(cmdTime/1000)
    
            if forceMagLink > 0.4:
                print('collision!')
                return 1
            elif cmdTime >= sim_time*1000:
                ts2 = time.time()
                real_time_lapsed = ts2-ts1
                print('timeout')
                print(f'Simulation Time: {cmdTime}')
                print(f'Real Time: {real_time_lapsed}')
                return 2
            else:
                set_vel(clientID, handle_robot, handle_leftJoint, handle_rightJoint, 
                        linear_velocity, 2*-phi_delta)
                
        dist_to_target = np.linalg.norm(pos_target_2d - pos_current_2d)
        
    return 0
        
    # Plot force sensor graph
#    fig1 = plt.figure()
#    ax1 = fig1.add_subplot(1,1,1)
#    ax1.plot(cmdTimeAll,forceMagLinkAll)
#    plt.show()
    
#%% function: turn to target
    
def turn_to_target(clientID,
                   handle_robot,
                   handle_leftJoint,
                   handle_rightJoint,
                   target: '[x y] metres'):
    
    returnCode, eulerAngles = vrep.simxGetObjectOrientation(
            clientID, handle_robot, -1, vrep.simx_opmode_buffer)
    
    returnCode, position = vrep.simxGetObjectPosition(
            clientID, handle_robot, -1, vrep.simx_opmode_buffer)
    
    xDelta = target[0] - position[0]
    yDelta = target[1] - position[0]
        
    phi_current = eulerAngles[2]
    phi_target = math.atan2(yDelta, xDelta)
    phi_delta = phi_current - phi_target

    while abs(phi_delta) > 0.1:
        
        returnCode, eulerAngles = vrep.simxGetObjectOrientation(
                clientID, handle_robot, -1, vrep.simx_opmode_buffer)
        
        returnCode, position = vrep.simxGetObjectPosition(
                clientID, handle_robot, -1, vrep.simx_opmode_buffer)
    
        xDelta = target[0] - position[0]
        yDelta = target[1] - position[0]
        
        phi_current = eulerAngles[2]
        phi_target = math.atan2(yDelta, xDelta)
        phi_delta = phi_current - phi_target

        set_vel(clientID, handle_robot, handle_leftJoint, handle_rightJoint, 
                0, 2)
    
    
    
#%% Main script
    
    

answer = 0
while answer == 0:
    
    start_sim = False
    start_vrep = input("Start a new vrep session, y/n? ")
    if start_vrep == 'y':
        # initialise vrep
        headless = False
        quit_after_sim = False
        sim_time = 0
        
        headless_yn = input("Headless? y/n ")
        if headless_yn == 'y':
            headless = True
            start_sim_yn = input("Start the simulation immediately? y/n ")
            if start_sim_yn == 'y':
                start_sim = True
            quit_after_sim_yn = input("Quit V-REP after the simulation? y/n ")
            if quit_after_sim_yn == 'y':
                quit_after_sim = True
                
        sim_time = int(input('Enter the duration of the simulation (seconds): '))
        
        clientID = initialise_vrep(
                    'epuck_arena_ramps2', 19999, headless, start_sim, 0, quit_after_sim)
        answer = 1
    elif start_vrep == 'n':
        sim_time = int(input('Enter the duration of the simulation (seconds): '))
        clientID = 0
        answer = 1
    else:
        print("please type 'y' to start vrep or 'n' to use open vrep program\n")            
            
#%% Get all object handles

ts1 = time.time()

returnCode, handles, intData, floatData, stringData = vrep.simxGetObjectGroupData(
        clientID, vrep.sim_appobj_object_type, 0, vrep.simx_opmode_blocking)

for i in range(len(handles)):
    exec(stringData[i] + f"={handles[i]}")
    
    
#%% Set initial posiition, orientation and velocity parameters of the ePuck
    
xStart = 1
yStart = 0.5

xTarget = 1
yTarget = 1.1

pos_init = [xStart, yStart, 0.21915] # initial position of ePuck
ang_init = 0 # angle of ePuck from the x-axis (rad) 
    
set_pos_ang(clientID, ePuck, pos_init, ang_init)

#%% Setup streaming calls

returnCode, linearVelocity, angularVelocity = vrep.simxGetObjectVelocity(
        clientID, ePuck, vrep.simx_opmode_streaming)

returnCode, state, forceVectorLink, torqueVector = vrep.simxReadForceSensor(
        clientID, ePuck_link, vrep.simx_opmode_streaming)

returnCode, eulerAngles = vrep.simxGetObjectOrientation(
        clientID, ePuck, -1, vrep.simx_opmode_streaming)

returnCode, position = vrep.simxGetObjectPosition(
        clientID, ePuck, -1, vrep.simx_opmode_streaming)


#%% Run simulation

if start_sim != True:
    start_sim = input("To start the simulation, press Enter: ")
    returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    
turn_to_target(clientID, ePuck, ePuck_leftJoint, ePuck_rightJoint, 
               [xTarget, yTarget])

outcome = move_to_target(clientID, ePuck, ePuck_link, ePuck_leftJoint, 
                         ePuck_rightJoint, 0.1, [xTarget, yTarget])

outcome = move_to_target(clientID, ePuck, ePuck_link, ePuck_leftJoint, 
                         ePuck_rightJoint, 0.1, [1.5, 1.1])

print(f'outcome: {outcome}')
    
set_vel(clientID, ePuck, ePuck_leftJoint, ePuck_rightJoint, 0, 0)
    
print('made it here')


returnCode = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)


