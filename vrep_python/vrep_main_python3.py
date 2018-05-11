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
    Popen([f"{path_to_vrep}/vrep.sh", head_call, start_sim_call, quit_after_sim,
           f"-gREMOTEAPISERVERSERVICE_{vrep_port}_FALSE_FALSE", 
           f"{path_to_vrep}/scenes/greg_scenes/{scene_name}.ttt",
           '--display:=1'], 
            stdout=PIPE, stderr=PIPE)
    
    # Allow vrep to boot before initialising comms
    time.sleep(5)
    
    # Sets up communication between python and vrep
    clientID = vrep.simxStart('127.0.0.1',vrep_port,True,True,8000,5)
    
    if clientID!=-1:
        print('Connected to remote API server')
    else:
        print('Connection not successful')
        sys.exit('Could not connect')
        
    return clientID
        
    
#%% Main script
    
    # initialise vrep
    clientID = initialise_vrep('epuck_arena_test2', 19999, False, False, 0, False)

#%% Get all object handles

    returnCode, handles, intData, floatData, stringData = vrep.simxGetObjectGroupData(
            clientID, vrep.sim_appobj_object_type, 0, vrep.simx_opmode_blocking)
    
    for i in range(len(handles)):
        exec(stringData[i] + f"={handles[i]}")
    
#%% Get ePuck position

    # initialise position and orientation readings
    returnCode, pos = vrep.simxGetObjectPosition(
            clientID, ePuck, -1, vrep.simx_opmode_streaming)
    
    returnCode, eulerAngles = vrep.simxGetObjectOrientation(
            clientID, ePuck, -1, vrep.simx_opmode_streaming)
    
    # time required before second call otherwise it returns [0,0,0]
    time.sleep(1)
    
    returnCode, pos = vrep.simxGetObjectPosition(
            clientID, ePuck, -1, vrep.simx_opmode_buffer)
    
    returnCode, eulerAngles = vrep.simxGetObjectOrientation(
            clientID, ePuck, -1, vrep.simx_opmode_buffer)
    
    # check that position is not still [0,0,0]
    print(pos)
    print(eulerAngles)
    
#%% Set ePuck position, orientation and velocity

    pos_init = [1, 0.5, pos[2]] # initial position of ePuck
    
    ang_init = 90 # angle of ePuck from the x-axis (deg)
    
    v_des = 2 # initial velocity
    
    # Set position
    returnCode = vrep.simxSetObjectPosition(
            clientID, ePuck, -1, pos_init, vrep.simx_opmode_oneshot)
    
    # Convert angle to euler angle
    eulerAngles = [0, 0, ang_init*math.pi/180]
    
    # Set orientation
    returnCode = vrep.simxSetObjectOrientation(
            clientID, ePuck, -1, eulerAngles, vrep.simx_opmode_oneshot)
    
    vrep.simxSetJointTargetVelocity(
            clientID, ePuck_leftJoint, v_des, vrep.simx_opmode_streaming)

    vrep.simxSetJointTargetVelocity(
            clientID, ePuck_rightJoint, v_des, vrep.simx_opmode_streaming)
    
    