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
import time
import math
from subprocess import Popen, PIPE
from matplotlib import path
import threading
import matplotlib.pyplot as plt
import epuck


#%%
        
def create_obstacle(obstacle_type_int: '0 = cuboid, 1 = cylinder',
                    mean = 0.2,
                    std_dev = 0.03):
    
    # changes int to 0 for cuboid, 2 for cylinder
    obstacle_type_int *= 2
    
    # obstacle dimensions
    obstacle_dimensions = [None]*3
    obstacle_dimensions[0] = np.absolute(np.random.normal(mean,std_dev))
    obstacle_dimensions[1] = np.absolute(np.random.normal(mean,std_dev))
    obstacle_dimensions[2] = np.absolute(np.random.normal(mean,std_dev))
    
    # Call vrep function that creates pure shapes
    res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_customizationscript, 'createObstacle', [obstacle_type_int], obstacle_dimensions, [], emptyBuff, vrep.simx_opmode_blocking)
    
    obstacle_handle = retInts[0]
    
    return obstacle_handle, obstacle_dimensions

    
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
    clientID = vrep.simxStart('127.0.0.1',vrep_port,True,True,15000,5)
    
    if clientID!=-1:
        print('Connected to remote API server')
    else:
        print('Connection not successful')
        sys.exit('Could not connect')
        
    return clientID

#%%
    
def calculate_positions(map_points: 'np.array of points',
                number_of_points: 'number of points to be calculated',
                min_spacing: 'minimum euc distance between points'):

    map_buffer = buffer_map(map_points, min_spacing)
    
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


def extra_position(map_points: 'np.array of points',
                 positions: 'positions',
                 radii: 'array of radii of existing objects',
                 radius: 'radius of new object'):
    
    map_buffer = buffer_map(map_points, radius)
    
    arena_dims = np.array([max(map_buffer[:,0]), max(map_buffer[:,1])])
    
    map_path = path.Path(map_buffer)
    
    position_added = False
    
    while not position_added:
        position_candidate = np.random.rand(1,2)
        position_candidate = np.multiply(position_candidate, arena_dims)
        print(f'Position Candidate: {position_candidate}')
        
        if map_path.contains_point(position_candidate[0]):
            for j in range(0,positions.shape[0]):
                min_spacing = radii[j] + radius
                if(np.linalg.norm(position_candidate - positions[j,:]) < min_spacing):
                    print(np.linalg.norm(position_candidate - positions[j,:]))
                    break
                if(j == positions.shape[0]-1):
                    position_added = True
        
    return position_candidate[0]


def delete_epucks():
    
    for robot in epuck:
        if robot.i != '1':
            vrep.simxRemoveModel(clientID, robot.handle, vrep.simx_opmode_oneshot)
            
            
def delete_objects(object_array):
    
    for obj in object_array:
        vrep.simxRemoveObject(clientID, obj.handle, vrep.simx_opmode_oneshot)
        
        
def buffer_map(map_points: 'n x 2 numpy array',
               buffer = 0.1):
    
    map_buffer = map_points.flatten()
    
    for i in range(map_buffer.size):
        if map_buffer[i] == 0:
            map_buffer[i] += buffer
        else:
            map_buffer[i] -= buffer
    
    map_buffer = map_buffer.reshape(-1,2)
    
    return map_buffer

def clone_sensor(sensor_handle: 'handle of sensor to be cloned',
                 turn_angle):
    
    _, newObjectHandles = vrep.simxCopyPasteObjects(clientID, [sensor_handle], vrep.simx_opmode_blocking)
    
    new_sensor_handle = newObjectHandles[0]
    
    _, parent_handle = vrep.simxGetObjectParent(clientID, sensor_handle, vrep.simx_opmode_blocking)
    
    vrep.simxSetObjectParent(clientID, new_sensor_handle, parent_handle, True, vrep.simx_opmode_oneshot)
    
    vrep.simxSetObjectOrientation(clientID, new_sensor_handle, sensor_handle, [0,(turn_angle*math.pi)/180,1], vrep.simx_opmode_oneshot)
    
    return new_sensor_handle


def create_sensor_array(sensor_name: 'name of sensor to be cloned',
                        number_sensors: 'number to be added',
                        angle_between_sensors: 'deg'):
    
    _, sensor_handle = vrep.simxGetObjectHandle(clientID, sensor_name, vrep.simx_opmode_blocking)

    laser_sensor_handles = []
    
    for i in range(number_sensors):
        turn_angle = (i+1)*angle_between_sensors
        new_sensor_handle = clone_sensor(sensor_handle, turn_angle)
        laser_sensor_handles.append(new_sensor_handle)
        
    return laser_sensor_handles

        

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
                    'epuck_arena_multi_spawn', 19998, headless)
        answer = 1
    elif start_vrep == 'n':
        clientID = 0
        answer = 1
    else:
        print("please type 'y' to start vrep or 'n' to use open vrep program\n")            

#%% Setup ePuck objects

returnCode = []

ego_puck = ePuck(0, True)

number_epucks = 6
epuck = []
epuck.append(ePuck(1))

for i in range(2, number_epucks+1):
    returnCode, newObjectHandles = vrep.simxCopyPasteObjects( clientID, [epuck[0].handle], vrep.simx_opmode_blocking)
    robot = ePuck(i)
    epuck.append(robot)

    
#%% Size of arena (map)
    
map_points = np.array([[0,0],[0,1.5],[2,1.5],[2,0]])

        
#%% Create obstacles
obstacle = []
number_obstacles = 5

for i in range(number_obstacles):
    obs_type = np.random.randint(2)
    obstacle_handle, obstacle_dimensions = create_obstacle(obs_type)
    if obs_type == 0:
        obstacle.append(cuboid(obstacle_handle, obstacle_dimensions))
    else:
        obstacle.append(cylinder(obstacle_handle, obstacle_dimensions))

#%% Randomise ePuck starting positions


# Minimum initial spacing between robots (m)
epuck_min_spacing = 0.2     

# Randomised starting position with minimum spacing and buffer dist from arena walls
start_positions = calculate_positions(map_points, number_epucks+2, epuck_min_spacing)

# array of robot and obstacle radii, used to avoid placing obstacles on top of robots
radii = [0.05]

# Set epuck starting parameters
for robot in epuck:
    robot.set_vel(0,0)
    robot.set_pos(start_positions[int(robot.i),:])
    robot.set_ang(np.random.rand()*2*math.pi)
    radii.append(0.05)

# keep track of obstacle positions and radii separately
obstacle_positions = []
obstacle_radii = []

# Caluclate positions for obstacles that dont impact robot positions
for obs in obstacle:
    radii.append(obs.radius)
    obs.position = extra_position(map_points, start_positions, radii, obs.radius)
    obstacle_positions.append(obs.position)
    obstacle_radii.append(obs.radius)
    obs.set_pos(obs.position)

#%%%%%%%%%%%%%%%%%%%%%%%% Need to make zero objects possible
# target position avoiding obstacles in map   
# target_positions = extra_position(map_points, np.array(obstacle_positions), obstacle_radii, 0.05)
#%%%%%%%%%%%%%%%%%%%%%%%

# Set ego_puck starting parameters
ego_puck.set_vel(0,0)
ego_puck.set_pos(start_positions[int(ego_puck.i),:])
#ego_puck.set_pos([0.25,0.75])
ego_puck.set_ang(np.random.rand()*2*math.pi)
ego_puck.scan_dist = [None]*(number_epucks + number_obstacles)
ego_puck.scan_ang = [None]*(number_epucks + number_obstacles)
ego_puck.velocity_towards = [None]*(number_epucks + number_obstacles)
ego_puck.velocity_perpendicular = [None]*(number_epucks + number_obstacles)
ego_puck.object_radii = [None]*(number_epucks + number_obstacles)
    
#%%

start_sim = input("Start the simulation, press Enter (or n to cancel): ")
if start_sim == 'n':
    delete_epucks()
    delete_objects(obstacle)
    sys.exit()

returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

time.sleep(1)

t = [None]*(number_epucks+1)

stop_event = threading.Event()

for i in range(number_epucks):
    robot_velocity = np.absolute(np.random.normal(0.1,0.02))
    print(f'Velocity of ePuck#{i+1}: {robot_velocity}')
    
    t[i] = threading.Thread(target = epuck[i].random_walk, args = (robot_velocity,))
    
t[number_epucks] = threading.Thread(target = ego_puck.move_to_target, args = ([1.75,0.75],0.1,True,True,False,True))

for i in range(number_epucks+1):
    t[i].start()

for i in range(number_epucks+1):
    t[i].join()

for i in range(number_epucks):
    epuck[i].set_vel(0,0)


#%% Delete extra epucks
    
delete_epucks()
delete_objects(obstacle)
        
plt.plot(ego_puck.forceMagList)


