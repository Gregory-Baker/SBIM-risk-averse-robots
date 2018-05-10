#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun May  6 23:54:59 2018

@author: greg
"""
#%%
import vrep
import sys
import numpy as np
import matplotlib.pyplot as mlp
import time
from subprocess import Popen, PIPE

#%%

scene_name = 'epuck_UI_edit3'
headless = True
start_sim = True

head_call = ''
if headless:
    head_call = '-h'
    
start_sim_call = ''
if start_sim:
    start_sim_call = '-s'

vrep.simxFinish(-1) # just in case, close all opened connections

process = Popen(["../V-REP_PRO_EDU_V3_5_0_Linux/vrep.sh", f"{head_call}", f"{start_sim_call}", "-q",
                 "-gREMOTEAPISERVERSERVICE_19999_FALSE_FALSE", 
                 f"../V-REP_PRO_EDU_V3_5_0_Linux/scenes/greg_scenes/{scene_name}.ttt"], stdout=PIPE, stderr=PIPE)
#stdout, stderr = process.communicate()
#print(stdout)
    
#%%
if start_sim:
    time.sleep(10)
else:
    input("Continue? (Press Enter when simulation is running)")
    

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')
    
#%%
    
#returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

returnCode, pingTime = vrep.simxGetPingTime(clientID)

print(pingTime)

#%%
    
errorCode, leftMotor = vrep.simxGetObjectHandle(clientID, 'ePuck_leftJoint', vrep.simx_opmode_oneshot_wait)
errorCode, rightMotor = vrep.simxGetObjectHandle(clientID, 'ePuck_rightJoint', vrep.simx_opmode_oneshot_wait)

errorCode, camHandle = vrep.simxGetObjectHandle(clientID, 'ePuck_camera', vrep.simx_opmode_oneshot_wait)
returnCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, camHandle, 0, vrep.simx_opmode_streaming)

proxSens=[-1,-1,-1,-1,-1,-1,-1,-1]
sensName=[-1,-1,-1,-1,-1,-1,-1,-1]
for i in range(0,8):
    sensName[i] = f'ePuck_proxSensor{i+1}'
    errorCode, proxSens[i] = vrep.simxGetObjectHandle(clientID, sensName[i], vrep.simx_opmode_oneshot_wait)
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, proxSens[i], vrep.simx_opmode_streaming)

#%%
    
v_des = 0.5

vrep.simxSetJointTargetVelocity(clientID,leftMotor,v_des,vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID,rightMotor,-v_des,vrep.simx_opmode_streaming)


returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, proxSens[4], vrep.simx_opmode_buffer)

for j in range(1,30):
    returnCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, camHandle, 0, vrep.simx_opmode_buffer)
    im = np.array(image, dtype=np.uint8)
    im.resize([resolution[0],resolution[1],3])
    #except IndexError:
        #break        
    mlp.imshow(im, origin='lower')
    mlp.show()
    print('ePuck Camera View')

#if headless:
returnCode = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
print('Simulation terminated')
    
#returnCode = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    

