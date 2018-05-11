#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 10 13:06:51 2018

@author: greg
"""

#%% Import modules

import vrep
import sys
import numpy as np
import matplotlib.pyplot as mlp
import time
from subprocess import Popen, PIPE

#%% Settings

scene_name = 'epuck_arena_test2'
headless = False
start_sim = False
quit_vrep_after_sim = False
sim_time = 20 # duration of simulation (seconds). 0 = no time limit.

#%% Paths and set variables

path_to_vrep = '/home/greg/V-REP_PRO_EDU_V3_5_0_Linux'
vrep_port = 19999

#%% Call to vrep

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
        raise ValueError('sim_time (simulation duration) must be a positive number, or zero for no time limit')
        
quit_after_sim = ''
if quit_vrep_after_sim:
    quit_after_sim = '-q'
    

vrep.simxFinish(-1) # just in case, close all opened connections

process = Popen([f"{path_to_vrep}/vrep.sh", head_call, start_sim_call, quit_after_sim,
                 f"-gREMOTEAPISERVERSERVICE_{vrep_port}_FALSE_FALSE", 
                 f"{path_to_vrep}/scenes/greg_scenes/{scene_name}.ttt"], stdout=PIPE, stderr=PIPE)
    
#%% Initiate communication to vrep (requires running simulation)
    
if start_sim:
    time.sleep(10)
else:
    cont_sim = input("Press 'Enter' if simulation is running, or 'n' to cancel")
    if cont_sim=='n':
        exit()

clientID=vrep.simxStart('127.0.0.1',vrep_port,True,True,5000,5)

if clientID!=-1:
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')
    
#%% Get object handles
    
errorCode, ePuck = vrep.simxGetObjectHandle(
        clientID, 'ePuck', vrep.simx_opmode_oneshot_wait)

errorCode, leftMotor = vrep.simxGetObjectHandle(
        clientID, 'ePuck_leftJoint', vrep.simx_opmode_oneshot_wait)

errorCode, rightMotor = vrep.simxGetObjectHandle(
        clientID, 'ePuck_rightJoint', vrep.simx_opmode_oneshot_wait)


#%%
    
    
returnCode, start_position = vrep.simxGetObjectPosition(
        clientID, ePuck, -1, vrep.simx_opmode_streaming)

time.sleep(1)

returnCode, start_position = vrep.simxGetObjectPosition(
        clientID, ePuck, -1, vrep.simx_opmode_buffer)

print(start_position)

#%% Set ePuck position, orientation and velocity

position = [0.5,0.5,start_position[2]]
v_des = 2

returnCode = vrep.simxSetObjectPosition(
        clientID, ePuck, -1, position, vrep.simx_opmode_oneshot)




returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)



vrep.simxSetJointTargetVelocity(
        clientID,leftMotor,v_des,vrep.simx_opmode_streaming)

vrep.simxSetJointTargetVelocity(
        clientID,rightMotor,v_des,vrep.simx_opmode_streaming)
    
    
returnCode = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
print('Simulation terminated')
    

    
