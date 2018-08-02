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
import matplotlib.pyplot as plt
#from queue import Queue

#%% Misc. Parameters

emptyBuff = bytearray()

#%% ePuck class

class ePuck:
    
    # ePuck geometric parameters
    wheel_sep = 0.053   # distance between wheels (m)
    wheel_rad = 0.02    # radius of wheels (m)
    
    # Braitenberg weights for obstacle avoidance
    brait_weights_leftMotor = [-1,1,2,-2,-1,0,0,0]
    noDetectionDistance = 0.04
    
    # Special Parameters for ego_puck
    number_laser_sensors = 181
    
#----------------------------------------------------------------------------
        
    def __init__(self, 
                 index,
                 ego_puck = False):

        self.i = str(index)
        self.name = 'ePuck#' + self.i
        
        # Get object handles
        returnCode, self.handle = vrep.simxGetObjectHandle(clientID, self.name, vrep.simx_opmode_blocking)
        
        returnCode, self.leftJoint = vrep.simxGetObjectHandle(clientID, 'ePuck_leftJoint#' + self.i, vrep.simx_opmode_blocking)
                                                              
        returnCode, self.rightJoint = vrep.simxGetObjectHandle(clientID, 'ePuck_rightJoint#' + self.i, vrep.simx_opmode_blocking)
        
        returnCode, self.link = vrep.simxGetObjectHandle(clientID, 'ePuck_link#' + self.i, vrep.simx_opmode_blocking)
        
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
        for i in range(8):
            returnCode, self.proxSensor[i], self.proxPoint[i], _, _ = vrep.simxReadProximitySensor(clientID, self.proxSensorHandle[i], vrep.simx_opmode_streaming)
        
        if ego_puck:
            self.laserSensor = [None]*self.number_laser_sensors
            self.laserPoint = [None]*self.number_laser_sensors
            self.laserDist = [None]*self.number_laser_sensors
            
            _, self.laserSensorHandle, _, _, _ = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_customizationscript, 'laserSensorHandles', [self.number_laser_sensors], [], [], emptyBuff, vrep.simx_opmode_blocking)

            for i in range(self.number_laser_sensors):
                _, self.laserSensor[i], self.laserPoint[i], _, _ = vrep.simxReadProximitySensor(clientID, self.laserSensorHandle[i], vrep.simx_opmode_streaming)
            
            
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
        
        returnCode, linearVelocity, angularVelocity = vrep.simxGetObjectVelocity(clientID, self.handle, vrep.simx_opmode_buffer)
        
#----------------------------------------------------------------------------
# Read Sensors      
        
    def read_prox_sens(self):
        
        for i in range(8):
            returnCode, self.proxSensor[i], self.proxPoint[i], _, _ = vrep.simxReadProximitySensor(clientID, self.proxSensorHandle[i], vrep.simx_opmode_buffer)
        
            self.proxDist[i] = self.proxSensor[i]*np.linalg.norm(np.array(self.proxPoint[i]))
            
    
    def read_force_sens(self):
        
        _, state, forceVector, _ = vrep.simxReadForceSensor(clientID, self.link, vrep.simx_opmode_buffer)
        
        self.forceMag = np.linalg.norm(forceVector)
        
    def read_laser_sens(self):
        
        for i in range(self.number_laser_sensors):
            
            returnCode, self.laserSensor[i], self.laserPoint[i], _, _ = vrep.simxReadProximitySensor(clientID, self.laserSensorHandle[i], vrep.simx_opmode_buffer)
            
            self.laserDist[i] = self.laserSensor[i]*self.laserPoint[i][2]
        
#----------------------------------------------------------------------------
# Other functions
        
    def move_to_target(self, 
                       target: '[x y] metres', 
                       linear_velocity: 'm/s',
                       stop_at_target: 'bool',
                       force_sens = False,
                       proximity_to_target = 0.05):

        self.get_pos()
        self.forceMagList = []
        
        target_np = np.array(target)
        position_np = np.array(self.position)      
        
        dist_to_target = np.linalg.norm(target_np - position_np)
        print(dist_to_target)

        while dist_to_target > proximity_to_target:
            
            _, sim_run = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state)
            if(sim_run == 0):
                break
            
            self.read_laser_sens()
            
            if force_sens:
                self.read_force_sens()
                if self.forceMag < 100:
                    self.forceMagList.append(self.forceMag)
            
            self.get_pos()
            self.get_ang()
            self.read_prox_sens()
                
            position_np = np.array(self.position) 
            
            xDelta = target_np[0] - self.position[0]
            yDelta = target_np[1] - self.position[1]
            
            phi_target = math.atan2(yDelta, xDelta)
            phi_delta = phi_target - self.angle
            
            if phi_delta < -math.pi:
                phi_delta = phi_delta + 2*math.pi
            elif phi_delta > math.pi:
                phi_delta = phi_delta - 2*math.pi
                
            angular_velocity = 2*phi_delta

            self.set_vel(linear_velocity, angular_velocity)
                    
            dist_to_target = np.linalg.norm(target_np - position_np)
            print(f'Distance to Target: {dist_to_target}')
            
            time.sleep(0.05)
        
        if stop_at_target:
            self.set_vel(0,0)
            
    def avoid_obstacles(self,
                        wheel_velocity):

        velRight = wheel_velocity
        velLeft = wheel_velocity
        for i in range(1,5):
            
            velLeft = velLeft + wheel_velocity*self.brait_weights_leftMotor[5-i]*(1-(self.proxDist[i]/self.noDetectionDistance))
            
            velRight = velRight + wheel_velocity*self.brait_weights_leftMotor[i]*(1-(self.proxDist[i]/self.noDetectionDistance))
        
        proxDist_np = np.array(self.proxDist[1:5])
        proxDist_nonzero_ind = np.nonzero(proxDist_np)[0]
        proxDist_nonzero = proxDist_np[proxDist_nonzero_ind]
        proxDist_min = np.min(proxDist_nonzero)
        proxDist_argmin = np.argmin(proxDist_nonzero)
        
        if proxDist_min < 0.02:
            
            if proxDist_argmin <= 1:
                self.set_vel(0, -2)
            else:
                self.set_vel(0, 2)
            
        else:
            vrep.simxSetJointTargetVelocity(clientID, self.leftJoint, velLeft, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, self.rightJoint, velRight, vrep.simx_opmode_streaming)
        
        
    def random_walk(self,
                    linear_velocity: 'm/s',
                    angular_velocity_std_dev = 0.1,
                    epsilon_straight = 0.02):
        
        _, sim_run = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state)
        angular_velocity = 0.0
        
        while sim_run != 0:
            
            self.read_prox_sens()
            
            if(sum(self.proxDist[1:5])>0):
                
                self.avoid_obstacles(2)
            else:
                
                if np.random.rand() < epsilon_straight:
                    angular_velocity = 0.0
                
                angular_velocity = np.random.normal(angular_velocity, angular_velocity_std_dev)
                self.set_vel(linear_velocity, angular_velocity)
                
            _, sim_run = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state)
            
            time.sleep(0.05)
            
#%%
        
class obstacle:
    
    def __init__(self, handle, dimensions):
        self.handle = handle
        self.dimensions = dimensions
        self.width = dimensions[0]
        self.depth = dimensions[1]
        self.height = dimensions[2]
        self.radius = np.hypot(dimensions[0],dimensions[1])
        
        
    def set_pos(self, 
            position: '[x y] metres'):
    
        position = np.array(position)
        position = position.tolist()
        
        if len(position)==2:
            position.append(self.height/2)
        
        vrep.simxSetObjectPosition(clientID, self.handle, -1, position, vrep.simx_opmode_oneshot)
        
    def set_ang(self, 
        angle: 'rad'):
        
        # Convert angle to euler angle
        eulerAngles = [0, 0, angle]
        
        # Set orientation
        vrep.simxSetObjectOrientation(clientID, self.handle, -1, eulerAngles, vrep.simx_opmode_oneshot)
        
        
def create_obstacle(obstacle_type_int: '0 = cuboid, 1 = cylinder',
                    mean = 0.2,
                    std_dev = 0.03):
    
    obstacle_dimensions = [None]*3
    obstacle_dimensions[0] = np.absolute(np.random.normal(mean,std_dev))
    obstacle_dimensions[1] = np.absolute(np.random.normal(mean,std_dev))
    obstacle_dimensions[2] = np.absolute(np.random.normal(mean,std_dev))
    
    res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_customizationscript, 'createCuboid', [obstacle_type_int], obstacle_dimensions, [], emptyBuff, vrep.simx_opmode_blocking)
    
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
    clientID = vrep.simxStart('127.0.0.1',vrep_port,True,True,10000,5)
    
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

number_epucks = 3
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
number_obstacles = 2

for i in range(number_obstacles):
    obstacle_handle, obstacle_dimensions = create_cuboid_obstacle()
    obstacle.append(obstacle_cuboid(obstacle_handle, obstacle_dimensions))

#%% Randomise ePuck starting positions


# Minimum initial spacing between robots (m)
epuck_min_spacing = 0.2     

# Randomised starting position with minimum spacing and buffer dist from arena walls
start_positions = calculate_positions(map_points, number_epucks+1, epuck_min_spacing)

# array of robot and obstacle radii, used to avoid placing obstacles on top of robots
radii = [0.05]*number_epucks

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

# target position avoiding obstacles in map   
target_positions = extra_position(map_points, np.array(obstacle_positions), obstacle_radii, 0.05)


# Set ego_puck starting parameters
ego_puck.set_vel(0,0)
# ego_puck.set_pos(start_positions[int(ego_puck.i),:])
ego_puck.set_pos([0.25,0.75])
ego_puck.set_ang(np.random.rand()*2*math.pi)

# Set epuck starting parameters
for robot in epuck:
    robot.set_vel(0,0)
    robot.set_pos(start_positions[int(robot.i),:])
    robot.set_ang(np.random.rand()*2*math.pi)

    
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
    
t[number_epucks] = threading.Thread(target = ego_puck.move_to_target, args = ([1.75,0.75],0.1,True,True))

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