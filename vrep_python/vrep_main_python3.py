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
import matplotlib.pyplot as mlp
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
    path_to_vrep = '/home/greg/V-REP_PRO_EDU_V3_5_0_Linux'
    path_to_scenes = '../vrep_scenes/greg_scenes'
    
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
           f"{path_to_scenes}/{scene_name}.ttt",
           '--display:=0'], 
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

def set_pos_ang_vel(clientID: 'int',
                    handle: 'int',
                    handle_leftJoint: 'int', 
                    handle_rightJoint: 'int',
                    position: '[x y z] metres',
                    angle: 'degrees',
                    linear_velocity: 'm/s',
                    angular_velocity: 'deg/sec'):
    
    # Convert deg to rad
    ang_vel_rad = angular_velocity*math.pi/180
    
    # ePuck geometric parameters
    wheel_sep = 0.053   # distance between wheels (m)
    wheel_rad = 0.02    # radius of wheels (m)
    
    # Set position
    returnCode = vrep.simxSetObjectPosition(
            clientID, handle, -1, position, vrep.simx_opmode_oneshot)
    
    # Convert angle to euler angle
    eulerAngles = [0, 0, angle*math.pi/180]
    
    # Set orientation
    returnCode = vrep.simxSetObjectOrientation(
            clientID, handle, -1, eulerAngles, vrep.simx_opmode_oneshot)
    
    # Convert to wheel velocities
    vel_r = linear_velocity - wheel_sep*ang_vel_rad/2
    vel_l = linear_velocity + wheel_sep*ang_vel_rad/2
    
    # Convert to wheel angular veolocities
    omega_r = vel_r/wheel_rad
    omega_l = vel_l/wheel_rad
    
    vrep.simxSetJointTargetVelocity(
            clientID, handle_leftJoint, omega_r, vrep.simx_opmode_streaming)
    
    vrep.simxSetJointTargetVelocity(
            clientID, handle_rightJoint, omega_l, vrep.simx_opmode_streaming)

    
#%% Main script

answer = 0
while answer == 0:
    
    start_vrep = input("Start a new vrep session, y/n? ")
    if start_vrep == 'y':
        # initialise vrep
        headless = False
        quit_after_sim = False
        start_sim = False
        sim_time = 0
        
        headless_yn = input("Headless? y/n ")
        if headless_yn == 'y':
            headless = True
            start_sim_yn = input("Start the simulation immediately? y/n ")
            if start_sim_yn == 'y':
                start_sim = True
            quit_after_sim_yn = input("Quit V-REP after the simulation? y/n ")
            sim_time = ('Enter the duration of the simulation (seconds): ')
            if quit_after_sim_yn == 'y':
                quit_after_sim = True
        
        clientID = initialise_vrep(
                'epuck_arena_dr12', 19999, headless, start_sim, sim_time, quit_after_sim)
        answer = 1
    elif start_vrep == 'n':
        clientID = 0
        answer = 1
    else:
        print("please type 'y' to start vrep or 'n' to use open vrep program\n")            
            
#%% Get all object handles

returnCode, handles, intData, floatData, stringData = vrep.simxGetObjectGroupData(
        clientID, vrep.sim_appobj_object_type, 0, vrep.simx_opmode_blocking)

for i in range(len(handles)):
    exec(stringData[i] + f"={handles[i]}")
    
#%% Get ePuck position

# initialise position and orientation readings
returnCode, pos = vrep.simxGetObjectPosition(
        clientID, ePuck, -1, vrep.simx_opmode_streaming)
True
returnCode, eulerAngles = vrep.simxGetObjectOrientation(
        clientID, ePuck, -1, vrep.simx_opmode_streaming)

# time required before second call otherwise it returns [0,0,0]
time.sleep(1)

returnCode, pos = vrep.simxGetObjectPosition(
        clientID, ePuck, -1, vrep.simx_opmode_buffer)

returnCode, eulerAngles = vrep.simxGetObjectOrientation(
        clientID, ePuck, -1, vrep.simx_opmode_buffer)

# check that position is notTrue still [0,0,0]
print(pos)
print(eulerAngles)

#%% Set initial posiition, orientation and velocity parameters of the ePuck

pos_init = [0.5, 1, 0.01915] # initial position of ePuck
ang_init = 0 # angle of ePuck from the x-axis (deg)
vel_init = 0.2 # initial linear velocity (m/s)
ang_vel_init = 0 # initial angular velocity (deg/sec)
    
set_pos_ang_vel(clientID, ePuck, ePuck_leftJoint, ePuck_rightJoint, pos_init, ang_init, vel_init, ang_vel_init)

#%% Run simulation

returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

returnCode, linearVelocity, angularVelocity = vrep.simxGetObjectVelocity(
        clientID, ePuck, vrep.simx_opmode_streaming)

returnCode, force = vrep.simxGetJointForce(
        clientID, ePuck_link, vrep.simx_opmode_streaming)

returnCode, linearVelocity, angularVelocity = vrep.simxGetObjectVelocity(
        clientID, ePuck, vrep.simx_opmode_buffer)

for i in range(1,60):
    returnCode, force = vrep.simxGetJointForce(
            clientID, ePuck_link, vrep.simx_opmode_buffer)
    cmdTime = vrep.simxGetLastCmdTime(clientID)
    print(f'{cmdTime} - {force}')
    time.sleep(1)
    
print(linearVelocity)
print(angularVelocity)
    
returnCode = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

