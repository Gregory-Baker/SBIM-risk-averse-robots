#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 15:52:32 2018

@author: greg
"""

#%% Import modules

import vrep
import sys
import numpy as np
import time
import math
from subprocess import Popen, PIPE
from matplotlib import path
import threading
from queue import Queue

#%% ePuck class

class ePuck:
    
    # ePuck geometric parameters
    wheel_sep = 0.053   # distance between wheels (m)
    wheel_rad = 0.02    # radius of wheels (m)
    
    # Braitenberg weights for obstacle avoidance
    brait_weights_leftMotor = [-1,1,2,-2,-1,0,0,0]
    noDetectionDistance = 0.04
    
#----------------------------------------------------------------------------
        
    def __init__(self, index):
        self.i = str(index)
        self.name = 'ePuck#' + self.i
        
        # Get object handles
        returnCode, self.handle = vrep.simxGetObjectHandle(clientID, self.name, vrep.simx_opmode_blocking)
        
        returnCode, self.leftJoint = vrep.simxGetObjectHandle(clientID, 'ePuck_leftJoint#' + self.i, vrep.simx_opmode_blocking)
                                                              
        returnCode, self.rightJoint = vrep.simxGetObjectHandle(clientID, 'ePuck_rightJoint#' + self.i, vrep.simx_opmode_blocking)
        
        returnCode, self.link = vrep.simxGetObjectHandle(clientID, 'ePuck_link' + self.i, vrep.simx_opmode_blocking)
        
        self.proxSensorHandle = [None]*8
        for j in range(0,8):
            returnCode, self.proxSensorHandle[j] = vrep.simxGetObjectHandle(clientID, 'ePuck_proxSensor' + str(j+1) + '#' + self.i, vrep.simx_opmode_blocking)
    
        # Setup object streaming calls       
        returnCode, linearVelocity, angularVelocity = vrep.simxGetObjectVelocity(clientID, self.handle, vrep.simx_opmode_streaming)
        
        returnCode, state, forceVectorLink, torqueVector = vrep.simxReadForceSensor(clientID, self.link, vrep.simx_opmode_streaming)
        
        returnCode, eulerAngles = vrep.simxGetObjectOrientation(clientID, self.handle, -1, vrep.simx_opmode_streaming)
        
        returnCode, position = vrep.simxGetObjectPosition(clientID, self.handle, -1, vrep.simx_opmode_streaming)
        
        self.proxSensor = [None]*8
        self.proxPoint = [None]*8
        self.proxDist = [None]*8
        for i in range(0,8):
           returnCode, self.proxSensor[i], self.proxPoint[i], _, _ = vrep.simxReadProximitySensor(clientID, self.proxSensorHandle[i], vrep.simx_opmode_streaming) 
        
#----------------------------------------------------------------------------
# Setters
        
    def set_pos(self, 
                position: '[x y] metres'):
        
        position = np.array(position)
        position = position.tolist()
        
        if len(position)==2:
            position.append(0.01915)
        
        vrep.simxSetObjectPosition(clientID, self.handle, -1, position, vrep.simx_opmode_oneshot)

    def set_ang(self, 
                angle: 'rad'):
        
        # Convert angle to euler angle
        eulerAngles = [0, 0, angle]
        
        # Set orientation
        vrep.simxSetObjectOrientation(clientID, self.handle, -1, eulerAngles, vrep.simx_opmode_oneshot)
    
    def set_vel(self, 
                linear_velocity: 'm/s', 
                angular_velocity: 'rad/sec'):
        
        # Convert to wheel velocities
        vel_r = linear_velocity + self.wheel_sep*angular_velocity/2
        vel_l = linear_velocity - self.wheel_sep*angular_velocity/2
        
        # Convert to wheel angular veolocities
        omega_r = vel_r/self.wheel_rad
        omega_l = vel_l/self.wheel_rad
        
        # Set wheel velocities
        vrep.simxSetJointTargetVelocity(clientID, self.leftJoint, omega_l, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, self.rightJoint, omega_r, vrep.simx_opmode_streaming)

#----------------------------------------------------------------------------
# Getters
        
    def get_pos(self):
        
        returnCode, position = vrep.simxGetObjectPosition(
                clientID, self.handle, -1, vrep.simx_opmode_buffer)
        
        self.position = [position[0], position[1]]
        return self.position
        
    def get_ang(self):
        
        returnCode, eulerAngles = vrep.simxGetObjectOrientation(
                clientID, self.handle, -1, vrep.simx_opmode_buffer)
        
        self.angle = eulerAngles[2]
        return self.angle
        
    def get_vel(self):
        
        returnCode, linearVelocity, angularVelocity =                   vrep.simxGetObjectVelocity(clientID, self.handle, vrep.simx_opmode_buffer)
        
#----------------------------------------------------------------------------
# Other functions
        
    def read_prox_sens(self):
        
        for i in range(0,8):
            returnCode, self.proxSensor[i], self.proxPoint[i], _, _ = vrep.simxReadProximitySensor(clientID, self.proxSensorHandle[i], vrep.simx_opmode_buffer)
        
            self.proxDist[i] = self.proxSensor[i]*np.linalg.norm(np.array(self.proxPoint[i]))
        
    def move_to_target(self, 
                       target: '[x y] metres', 
                       linear_velocity: 'm/s', 
                       proximity_to_target = 0.05):

        self.get_pos()
        
        target_np = np.array(target)
        position_np = np.array(self.position)      
        
        dist_to_target = np.linalg.norm(target_np - position_np) 

        while dist_to_target > proximity_to_target:
            
            _, sim_run = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state)
            if(sim_run == 0):
                break
            
            self.get_pos()
            self.get_ang()
            self.read_prox_sens()
            
            # Check if front and side prox sens are activated
            if(sum(self.proxDist[1:5])>0):
                velRight = 2
                velLeft = 2
                for i in range(1,5):
                    velLeft = velLeft + 2*self.brait_weights_leftMotor[i]*(1-(self.proxDist[i]/self.noDetectionDistance))
                    velRight = velRight + 2*self.brait_weights_leftMotor[5-i]*(1-(self.proxDist[i]/self.noDetectionDistance))

                vrep.simxSetJointTargetVelocity(clientID, self.leftJoint, velRight, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, self.rightJoint, velLeft, vrep.simx_opmode_streaming)
            
            else:
    
                position_np = np.array(self.position) 
                
                xDelta = target[0] - self.position[0]
                yDelta = target[1] - self.position[1]
                
                phi_target = math.atan2(yDelta, xDelta)
                phi_delta = phi_target - self.angle
                
                if phi_delta < -math.pi:
                    phi_delta = phi_delta + 2*math.pi
                elif phi_delta > math.pi:
                    phi_delta = phi_delta - 2*math.pi
                    
                angular_velocity = 2*phi_delta

                self.set_vel(linear_velocity, angular_velocity)
                    
            dist_to_target = np.linalg.norm(target_np - position_np)
        
    
#%% Function to start VREP

def initialise_vrep(scene_name: 'str', 
                    vrep_port: 'int', 
                    headless: 'bool'):
    
    # Defines path to VREP folder
    path_to_vrep = '/home/greg/Programs/V-REP_PRO_EDU_V3_5_0_Linux'
    path_to_scenes = '../../vrep_scenes/test_scenes'
    
    # Converts vrep initialisation settings to arguments recognised by vrep
    head_call = ''
    if headless:
        head_call = '-h'  
    
    # close all opened connections to vrep
    vrep.simxFinish(-1) 
    
    # Command-line call to initialise vrep
    Popen(["nice", "-n", "-20", f"{path_to_vrep}/vrep.sh", head_call, '', '',
           f"-gREMOTEAPISERVERSERVICE_{vrep_port}_FALSE_FALSE", 
           f"{path_to_scenes}/{scene_name}.ttt"], 
            stdout=PIPE, stderr=PIPE)
    
    # Allow vrep to boot before initialising comms
    time.sleep(5)
    
    # Sets up communication between python and vrep
    clientID = vrep.simxStart('127.0.0.1',vrep_port,True,True,10000,5)
    
    if clientID!=-1:
        print('Connected to remote API server')
    else:
        print('Connection not successful')
        sys.exit('Could not connect')
        
    return clientID

#%%
    
def calculate_positions(map_buffer: 'np.array of points',
                number_of_points: 'number of points to be calculated',
                min_spacing: 'minimum euc distance between points'):

    arena_dims = np.array([max(map_buffer[:,0]), max(map_buffer[:,1])])
    
    map_path = path.Path(map_buffer)
    
    positions = np.empty([number_of_points,2])
    
    for i in range(number_of_points):
        position_added = False
        
        while not position_added:
            position_candidate = np.random.rand(1,2)
            position_candidate = np.multiply(position_candidate, arena_dims)
         
            if map_path.contains_point(position_candidate[0]):
                if i > 0:
                    for j in range(0,i):
                        if(np.linalg.norm(position_candidate - positions[j,:]) < min_spacing):
                            break
                        if(j == i-1):
                            positions[i,:] = position_candidate
                            position_added = True
                else:
                    positions[i,:] = position_candidate
                    position_added = True
        
    return positions

#%%--------------------------------------------------------------- 
# Main script
    
# Prompt user for vrep startup options    
answer = 0
while answer == 0:
    
    start_sim = False
    start_vrep = input("Start a new vrep session, y/n? ")
    if start_vrep == 'y':
        # initialise vrep
        headless = False        
        headless_yn = input("Headless? y/n ")
        if headless_yn == 'y':
            headless = True
                
        clientID = initialise_vrep(
                    'epuck_arena_multi', 19999, headless)
        answer = 1
    elif start_vrep == 'n':
        clientID = 0
        answer = 1
    else:
        print("please type 'y' to start vrep or 'n' to use open vrep program\n")            

#%% Setup ePuck objects

number_epucks = 5

returnCode = []
epuck = []

for i in range(number_epucks):
    robot = ePuck(i)
    epuck.append(robot)
    
    
#%% Size of arena (map)
    
map_points = np.array([[0,0],[0,1.5],[2,1.5],[2,0]])

buffer = 0.1
map_buffer = np.array([[0 + buffer, 0 + buffer],
                       [0 + buffer, 1.5 - buffer],
                       [2 - buffer, 1.5 - buffer],
                       [2 - buffer, 0 + buffer]])
        

#%% Randomise ePuck starting positions


# Minimum initial spacing between robots (m)
epuck_min_spacing = 0.2     

# Randomised starting position with minimum spacing and buffer dist from arena walls
start_positions = calculate_positions(map_buffer, number_epucks, epuck_min_spacing)
                
target_positions = calculate_positions(map_buffer, number_epucks, epuck_min_spacing)

# Set epuck starting parameters
for robot in epuck:
    robot.set_vel(0,0)
    robot.set_pos(start_positions[int(robot.i),:])
    robot.set_ang(np.random.rand()*2*math.pi)
    
#%%

start_sim = input("Start the simulation, press Enter (or n to cancel): ")
if start_sim == 'n':
    sys.exit()

returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

time.sleep(3)

epuck[2].move_to_target(target_positions[2,:], 0.05)
epuck[2].set_vel(0,0)

