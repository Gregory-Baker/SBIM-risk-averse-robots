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
# import matplotlib.pyplot as plt

#%% MIsc parameters
clientID = 0
emptyBuff = bytearray()

#%% ePuck class

class epuck:
    
    # ePuck geometric parameters
    wheel_sep = 0.053   # distance between wheels (m)
    wheel_rad = 0.02    # radius of wheels (m)
    radius = 0.05       # radius of robot with buffer distance (m)
    
    # Braitenberg weights for obstacle avoidance
    brait_weights_leftMotor = [-1,1,2,-2,-1,0,0,0]
    noDetectionDistance = 0.04
    
    # Special Parameters for ego_puck
    number_laser_sensors = 181
    sensor_angle = [-math.pi/2, math.pi/2]    # min and max angle of sensor
    sensor_distance = 1         # max distance of sensor
    
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
            
        self.get_pos()
        self.get_ang()
        self.get_vel()
        
        self.laserSensor = [None]*self.number_laser_sensors
        self.laserPoint = [None]*self.number_laser_sensors
        self.laserDist = [None]*self.number_laser_sensors
        
        if ego_puck:
            
            _, self.laserSensorHandle, _, _, _ = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_customizationscript, 'laserSensorHandles', [self.number_laser_sensors], [], [], emptyBuff, vrep.simx_opmode_blocking)

            for i in range(self.number_laser_sensors):
                _, _, _, _, _ = vrep.simxReadProximitySensor(clientID, self.laserSensorHandle[i], vrep.simx_opmode_streaming)
        
            self.distance_to_wall = [0]*4
            self.velocity_towards_wall = [0]*4
            self.velocity = [0]*3
            
            self.cumulative_reward = 0

#----------------------------------------------------------------------------
# Setters
        
    def set_pos(self, 
                position):
        
        self.position = position
        
        position = np.array(position)
        position = position.tolist()
        
        if len(position)==2:
            position.append(0.01915)
        
        vrep.simxSetObjectPosition(clientID, self.handle, -1, position, vrep.simx_opmode_oneshot)

    def set_ang(self, 
                angle):
        
        # Convert angle to euler angle
        eulerAngles = [0, 0, angle]
        
        # Set orientation
        vrep.simxSetObjectOrientation(clientID, self.handle, -1, eulerAngles, vrep.simx_opmode_oneshot)
    
    def set_vel(self, 
                linear_velocity, 
                angular_velocity):
        
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
        
        returnCode, self.linearVelocity, self.angularVelocity = vrep.simxGetObjectVelocity(clientID, self.handle, vrep.simx_opmode_buffer)
        
    def get_all(self):
        
        self.get_pos()
        self.get_ang()
        self.get_vel()
        
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
            
    def dist_ang_to_target(self,
                           target):
        
        position_np = np.array(self.position) 
        target_np = np.array(target)
        
        distance_to_target = np.linalg.norm(target_np - position_np)
        
        xDelta = target_np[0] - position_np[0]
        yDelta = target_np[1] - position_np[1]
        
        # angle between robot position and target position in world coordinate frame
        angle_to_target_world = math.atan2(yDelta, xDelta)
        
        # angle to target in robot coordinate frame
        angle_to_target = angle_to_target_world - self.angle
        
        if angle_to_target < -math.pi:
            angle_to_target = angle_to_target + 2*math.pi
        elif angle_to_target > math.pi:
            angle_to_target = angle_to_target - 2*math.pi
            
        return distance_to_target, angle_to_target
    
    def relative_velocity(self,
                          v_object,
                          angle):
        
        # translate to world coordinate frame
        alpha = self.angle + angle 
        
        velocity_towards = (self.linearVelocity[0] - v_object[0])*math.cos(alpha) + (self.linearVelocity[1] - v_object[1])*math.sin(alpha)
        velocity_perpendicular = (self.linearVelocity[1] - v_object[1])*math.cos(alpha) + (v_object[0] - self.linearVelocity[0])*math.sin(alpha)
        
        return velocity_towards, velocity_perpendicular
        
    
    def dist_ang_to_objects(self,
                            epucks,
                            obstacles):
        
        i = 0
        
        for robot in epucks:
            distance, angle = self.dist_ang_to_target(robot.position)
            if (distance < self.sensor_distance) and (self.sensor_angle[0] < angle < self.sensor_angle[1]):
                self.scan_dist[i] = distance
                self.scan_ang[i] = angle
                self.velocity_towards[i], self.velocity_perpendicular[i] = self.relative_velocity(robot.linearVelocity, angle)
                self.object_radii[i] = robot.radius
            else:
                self.scan_dist[i] = 0
                self.scan_ang[i] = 0
                self.velocity_towards[i] = 0
                self.velocity_perpendicular[i] = 0
                self.object_radii[i] = 0
            i += 1
            
        for obs in obstacles:
            distance, angle = self.dist_ang_to_target(obs.position)
            if (distance < self.sensor_distance) and (self.sensor_angle[0] < angle < self.sensor_angle[1]):
                self.scan_dist[i] = distance
                self.scan_ang[i] = angle
                self.velocity_towards[i], self.velocity_perpendicular[i] = self.relative_velocity([0,0], angle)
                self.object_radii[i] = obs.radius
            else:
                self.scan_dist[i] = 0
                self.scan_ang[i] = 0
                self.velocity_towards[i] = 0
                self.velocity_perpendicular[i] = 0
                self.object_radii[i] = 0
            i += 1
            
        scan_matrix_trans = np.stack((self.scan_dist, 
                                      self.scan_ang,
                                      self.velocity_towards,
                                      self.velocity_perpendicular,
                                      self.object_radii))
        
        scan_matrix = scan_matrix_trans.T
        scan_matrix_nonzero = scan_matrix[scan_matrix[:,0].nonzero()[0]]
        scan_matrix_nonzero_sorted = scan_matrix_nonzero[scan_matrix_nonzero[:,0].argsort()]
        num_zero_cols = scan_matrix.shape[0] - scan_matrix_nonzero.shape[0]
        scan_matrix_pad = np.zeros(scan_matrix.shape)
        scan_matrix_pad[:-num_zero_cols, :] = scan_matrix_nonzero_sorted
        
        self.scan_matrix = scan_matrix_pad
    
    
    def dist_to_walls(self,
                      map_points):
        
        arena_dims = [max(map_points[:,0]), max(map_points[:,1])]
        
        self.distance_to_wall[0] = self.position[0]
        self.distance_to_wall[1] = arena_dims[1] - self.position[1]
        self.distance_to_wall[2] = arena_dims[0] - self.position[0]
        self.distance_to_wall[3] = self.position[1]
        
    def velocity_inputs(self):
        
        self.velocity[0] = self.linearVelocity[0]
        self.velocity[1] = self.linearVelocity[1]
        self.velocity[2] = self.angularVelocity[2]
        
    def radar_observation(self):
        
        self.dist_to_target, self.ang_to_target = self.dist_ang_to_target(self.target_position)
        self.dist_ang_to_objects(epucks, obstacles)
        self.dist_to_walls(map_points)
        self.velocity_inputs()
        self.nn_input = np.concatenate((self.scan_matrix.flatten(), np.array(self.distance_to_wall), np.array(self.velocity), [self.dist_to_target], [self.ang_to_target]))
        
    def sensor_sweep(self):
        
        self.get_all()
        self.read_force_sens()
        self.radar_observation()
        
#-----------------------------------------------------------------------------
# Reward Functions
        
    def calc_distance_reward(self,
                             drop_off_exponent = 1):
        
        self.distance_reward = (1 - np.tanh(self.dist_to_target))**drop_off_exponent
        
    def calc_completion_reward(self,
                               proximity_to_target = 0.05):
        
        if self.dist_to_target < proximity_to_target:
            self.completion_reward = 20
        else:
            self.completion_reward = 0
            
    def calc_crash_reward(self):
        
        if 0.4 < self.forceMag < 100:
            self.crash_reward = -10*self.forceMag
        else:
            self.crash_reward = 0
        
    def calc_step_reward(self,
                          step_reward = -0.5):
        
        self.calc_completion_reward()
        self.calc_crash_reward()
        self.calc_distance_reward()
        
        self.total_reward = self.distance_reward + self.completion_reward + self.crash_reward + step_reward
        
    def calc_cumulative_reward(self):
        
        self.calc_step_reward()
        self.cumulative_reward += self.total_reward

#-----------------------------------------------------------------------------

    def move_to_target(self, 
                       target, 
                       linear_velocity,
                       stop_at_target,
                       force_sens = False,
                       laser_sens = False,
                       radar_sens = False,
                       proximity_to_target = 0.05):

        self.get_pos()
        
        target_np = np.array(target)
        position_np = np.array(self.position)      
        
        self.dist_to_target = np.linalg.norm(target_np - position_np)
        self.read_force_sens()

        while (self.dist_to_target > proximity_to_target) and not (0.4 < self.forceMag < 100):
            
            _, sim_run = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state)
            if(sim_run == 0):
                break
            
            self.get_all()
            self.read_force_sens()
            
            self.dist_to_target, self.ang_to_target = self.dist_ang_to_target(target_np)
            
            if laser_sens:
                self.read_laser_sens()
                
            if radar_sens:
                self.radar_observation()
            
            self.calc_cumulative_reward()

            angular_velocity = 2*self.ang_to_target
            self.set_vel(linear_velocity, angular_velocity)
                    
#            print(f'Cumulative Reward: {self.cumulative_reward}')
            
            time.sleep(0.05)
        
        stop_sim()
        
    def step(self,
             action,
             proximity_to_target = 0.05,
             step_penalty = -0.5):
        
        done = False
        
        _, sim_run = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state)
        if(sim_run == 0):
            done = True
            
        self.set_vel(action[0][1], action[0][0])
        
        self.sensor_sweep()
        
        self.calculate_distance_reward()
        
        if 0.4 < self.forceMag < 100:
            self.crash_reward = -20*self.forceMag
            done = True
        else:
            self.crash_reward = 0
        
        if self.dist_to_target < proximity_to_target:
            self.completion_reward = 20
            done = True
        else:
            self.completion_reward = 0
        
        self.step_reward = self.distance_reward + self.completion_reward + self.crash_reward + step_penalty
        
        return done
            
            
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
                    linear_velocity,
                    angular_velocity_std_dev = 0.1,
                    epsilon_straight = 0.02):
        
        _, sim_run = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state)
        angular_velocity = 0.0
        
        while sim_run != 0:
            
            self.get_pos()
            self.get_ang()
            self.get_vel()
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
            
#%% Obstacle class
            
class obstacle:
    
    def __init__(self, handle, dimensions):
        self.handle = handle
        self.dimensions = dimensions
        
        
    def set_pos(self, 
            position):
        
        self.position = position
    
        position = np.array(position)
        position = position.tolist()
        
        if len(position)==2:
            position.append(self.height/2)
        
        vrep.simxSetObjectPosition(clientID, self.handle, -1, position, vrep.simx_opmode_oneshot)
        
    def set_ang(self, 
                angle):
        
        # Convert angle to euler angle
        eulerAngles = [0, 0, angle]
        
        # Set orientation
        vrep.simxSetObjectOrientation(clientID, self.handle, -1, eulerAngles, vrep.simx_opmode_oneshot)
        
class cuboid (obstacle):
    
    def __init__ (self, handle, dimensions):
        super().__init__(handle, dimensions)
        self.width = dimensions[0]
        self.depth = dimensions[1]
        self.height = dimensions[2]
        self.radius = np.hypot(dimensions[0],dimensions[1])
        self.type = 0
        

class cylinder (obstacle):
    
    def __init__ (self, handle, dimensions):
        super().__init__(handle, dimensions)
        self.radius = dimensions[0]
        self.height = dimensions[2]
        self.type = 1
        
class dummy (obstacle):
    
    def __init__ (self, handle, dimensions):
        super().__init__(handle, dimensions)
        self.height = dimensions
        

#%%
        
def create_obstacle(obstacle_type_int,
                    mean = 0.18,
                    std_dev = 0.05):
    
    # changes int to 0 for cuboid, 2 for cylinder
    obstacle_type_int *= 2
    
    # obstacle dimensions
    obstacle_dimensions = [None]*3
    obstacle_dimensions[0] = np.absolute(np.random.normal(mean,std_dev))
    obstacle_dimensions[1] = np.absolute(np.random.normal(mean,std_dev))
    obstacle_dimensions[2] = 0.2
    
    # Call vrep function that creates pure shapes
    res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_customizationscript, 'createObstacle', [obstacle_type_int], obstacle_dimensions, [], emptyBuff, vrep.simx_opmode_blocking)
    
    obstacle_handle = retInts[0]
    
    return obstacle_handle, obstacle_dimensions

def create_obstacles(number_obstacles,
                     default_position = [-1.2,0.5]):
    
    obstacles = []

    for i in range(number_obstacles):
        obs_type = np.random.randint(2)
        obstacle_handle, obstacle_dimensions = create_obstacle(obs_type)
        if obs_type == 0:
            obstacles.append(cuboid(obstacle_handle, obstacle_dimensions))
        else:
            obstacles.append(cylinder(obstacle_handle, obstacle_dimensions))
        obstacles[i].set_pos(default_position)
        
    return obstacles

def create_dummy(dummy_size = 0.05,
                 colors = None):
    
    returnCode, dummyHandle = vrep.simxCreateDummy(clientID, dummy_size, colors, vrep.simx_opmode_blocking)
    
    return dummyHandle

def get_dummy_handle():
    
    _, dummyHandle = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)
    
    return dummyHandle

def place_dummy(position):
    dummy_handle = get_dummy_handle()
    if dummy_handle == 0:
        dummy_handle = create_dummy()
    target_dummy = dummy(dummy_handle,0.05)
    target_dummy.set_pos(position)
                                     
#%% Function to start VREP

def initialise_vrep(scene_name, 
                    vrep_port, 
                    headless):
    
    # Defines path to VREP folder
    path_to_vrep = '/home/greg/Programs/V-REP_PRO_EDU_V3_5_0_Linux'
    path_to_scenes = '../../vrep_scenes/test_scenes'
    
    # Converts vrep initialisation settings to arguments recognised by vrep
    head_call = ''
    if headless:
        head_call = '-h'  
    
    # close all opened connections to vrep_input
    vrep.simxFinish(-1) 
    
    # Command-line call to initialise vrep
    Popen(["nice", "-n", "-20", "/home/greg/Programs/V-REP_PRO_EDU_V3_5_0_Linux/vrep.sh", head_call, '', '', "-gREMOTEAPISERVERSERVICE_19998_FALSE_FALSE", "../../vrep_scenes/test_scenes/epuck_arena_multi_spawn.ttt"], stdout=PIPE, stderr=PIPE)
    
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


def start_sim():
    
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

def stop_sim():
    
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

#%%
    
def calculate_positions(map_points,
                        number_of_points,
                        min_spacing):

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


def extra_position(map_points,
                   positions,
                   radii,
                   radius):
    
    map_buffer = buffer_map(map_points, radius)
    
    arena_dims = np.array([max(map_buffer[:,0]), max(map_buffer[:,1])])
    
    map_path = path.Path(map_buffer)
    
    position_added = False
    
    while not position_added:
        position_candidate = np.random.rand(1,2)
        position_candidate = np.multiply(position_candidate, arena_dims)
        
        if map_path.contains_point(position_candidate[0]):
            for j in range(0,positions.shape[0]):
                min_spacing = radii[j] + radius
                if(np.linalg.norm(position_candidate - positions[j,:]) < min_spacing):
                    break
                if(j == positions.shape[0]-1):
                    position_added = True
        
    return position_candidate[0]


def delete_epucks(epucks):
    
    for robot in epucks:
        if robot.i != '1':
            vrep.simxRemoveModel(clientID, robot.handle, vrep.simx_opmode_oneshot)
            
            
def delete_objects(object_array):
    
    for obj in object_array:
        vrep.simxRemoveObject(clientID, obj.handle, vrep.simx_opmode_oneshot)
        
        
def buffer_map(map_points,
               buffer = 0.1):
    
    map_buffer = map_points.flatten()
    
    for i in range(map_buffer.size):
        if map_buffer[i] == 0:
            map_buffer[i] += buffer
        else:
            map_buffer[i] -= buffer
    
    map_buffer = map_buffer.reshape(-1,2)
    
    return map_buffer

def clone_sensor(sensor_handle,
                 turn_angle):
    
    _, newObjectHandles = vrep.simxCopyPasteObjects(clientID, [sensor_handle], vrep.simx_opmode_blocking)
    
    new_sensor_handle = newObjectHandles[0]
    
    _, parent_handle = vrep.simxGetObjectParent(clientID, sensor_handle, vrep.simx_opmode_blocking)
    
    vrep.simxSetObjectParent(clientID, new_sensor_handle, parent_handle, True, vrep.simx_opmode_oneshot)
    
    vrep.simxSetObjectOrientation(clientID, new_sensor_handle, sensor_handle, [0,(turn_angle*math.pi)/180,1], vrep.simx_opmode_oneshot)
    
    return new_sensor_handle


def create_sensor_array(sensor_name,
                        number_sensors,
                        angle_between_sensors):
    
    _, sensor_handle = vrep.simxGetObjectHandle(clientID, sensor_name, vrep.simx_opmode_blocking)

    laser_sensor_handles = []
    
    for i in range(number_sensors):
        turn_angle = (i+1)*angle_between_sensors
        new_sensor_handle = clone_sensor(sensor_handle, turn_angle)
        laser_sensor_handles.append(new_sensor_handle)
        
    return laser_sensor_handles

def create_epucks(number_epucks,
                  copy_object = False):
    
    epuck_array = []
    epuck_array.append(epuck(1))
    
    for i in range(2, number_epucks+1):
        if copy_object:
            returnCode, newObjectHandles = vrep.simxCopyPasteObjects(clientID, [epuck_array[0].handle], vrep.simx_opmode_blocking)
        robot = epuck(i)
        epuck_array.append(robot)
        
    return epuck_array
        
def reset_objects(objects, position):
    
    for obj in objects:
        obj.set_pos(position)
        
def reset_epucks(epucks,
                 number_active_epucks,
                 start_positions):
    
    for robot in epucks:
        x_pos = -1 -int(robot.i)/10
        robot.set_pos([x_pos, 1])
        robot.set_vel(0,0)
        
    for i in range(number_active_epucks):
        epucks[i].set_pos(start_positions[i,:])
        epucks[i].set_ang(np.random.rand()*2*math.pi)
        
def reset_obstacles(obstacles,
                    number_active_obstacles,
                    start_positions,
                    map_points,
                    radii):
    
    for i in range(number_active_obstacles):
        obstacles[i].position = extra_position(map_points, start_positions, radii, obstacles[i].radius)
        obstacles[i].set_pos(obstacles[i].position)
        obstacles[i].set_ang(np.random.rand()*2*math.pi)
        
def reset_ego_puck(ego_puck,
                   start_positions,
                   number_epucks,
                   number_obstacles,
                   number_active_epucks):
    
    # Set ego_puck starting parameters
    ego_puck.set_vel(0,0)
    ego_puck.set_pos(start_positions[number_active_epucks,:])
    ego_puck.set_ang(np.random.rand()*2*math.pi)
    
    # Set up arrays for NN input parameters recorded by ego_puck
    ego_puck.scan_dist = [None]*(number_epucks + number_obstacles)
    ego_puck.scan_ang = [None]*(number_epucks + number_obstacles)
    ego_puck.velocity_towards = [None]*(number_epucks + number_obstacles)
    ego_puck.velocity_perpendicular = [None]*(number_epucks + number_obstacles)
    ego_puck.object_radii = [None]*(number_epucks + number_obstacles)
        
        
#%% Main Script
        
import random
import argparse
from keras.models import model_from_json, Model
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.optimizers import Adam
import tensorflow as tf
#from keras.engine.training import collect_trainable_weights
import json
        
from ReplayBuffer import ReplayBuffer
from ActorNetwork import ActorNetwork
from CriticNetwork import CriticNetwork
    
from OU import OU
import timeit

OU = OU()       #Ornstein-Uhlenbeck Process

#def playGame(train_indicator=1):    #1 means Train, 0 means simply Run
train_indicator = 1
BUFFER_SIZE = 100000
BATCH_SIZE = 32
GAMMA = 0.99
TAU = 0.001     #Target Network HyperParameters
LRA = 0.0001    #Learning rate for Actor
LRC = 0.001     #Lerning rate for Critic

action_dim = 2  # Angluar_Velocity| Linear_Velocity
state_dim = 64  #of sensors input

#np.random.seed(1337)

EXPLORE = 100000.
episode_count = 2000
max_steps = 100000
reward = 0
done = False
step = 0
epsilon = 1
indicator = 0

clientID = 0
open_vrep = True
headless = False
number_epucks = 6
number_obstacles = 5
map_points = np.array([[0,0],[0,1.5],[2,1.5],[2,0]])

#Tensorflow GPU optimization
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
from keras import backend as K
K.set_session(sess)

actor = ActorNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRA)
critic = CriticNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRC)
buff = ReplayBuffer(BUFFER_SIZE)    #Create replay buffer

# Generate a Torcs environment
if open_vrep:
    clientID = initialise_vrep('epuck_arena_multi_spawn', 19998, headless)
    epucks = create_epucks(number_epucks, False)
    ego_puck = epuck(0, True)

#    #Now load the weight
#    print("Now we load the weight")
#    try:
#        actor.model.load_weights("actormodel.h5")
#        critic.model.load_weights("criticmodel.h5")
#        actor.target_model.load_weights("actormodel.h5")
#        critic.target_model.load_weights("criticmodel.h5")
#        print("Weight load successfully")
#    except:
#        print("Cannot find the weight")
    
    

print("Experiment Start.")
for i in range(episode_count):

#        print("Episode : " + str(i) + " Replay Buffer " + str(buff.count()))
    
    obstacles = create_obstacles(number_obstacles)

    number_active_epucks = np.random.randint(number_epucks+1)
    number_active_obstacles = np.random.randint(number_obstacles+1)
    
    start_positions = calculate_positions(map_points, number_active_epucks+2, 0.2)
    target_position = start_positions[len(start_positions)-1,:]
    ego_puck.target_position = target_position

    reset_epucks(epucks, number_active_epucks, start_positions)
    place_dummy(target_position)
    
    radii = [0.05]*(number_epucks+2) 
    for obs in obstacles:
        radii.append(obs.radius)
        
    reset_obstacles(obstacles, number_active_obstacles, start_positions, map_points, radii)
    
    reset_ego_puck(ego_puck, start_positions, number_epucks, number_obstacles, number_active_epucks)
    
    t = [None]*(number_active_epucks)

    start_sim()
    
    time.sleep(1)
    
    for i in range(number_active_epucks):
        robot_velocity = np.random.gumbel(0.1,0.02)
        t[i] = threading.Thread(target = epucks[i].random_walk, args = (robot_velocity,))
        
    for i in range(number_active_epucks):
        t[i].start()
        
    ego_puck.sensor_sweep()
    
    s_t = ego_puck.nn_input

#    stop_sim()
#    
#    delete_objects(obstacles)
#    sys.exit()
 
    total_reward = 0.
    for j in range(max_steps):
        loss = 0 
        epsilon -= 1.0 / EXPLORE
        a_t = np.zeros([1,action_dim])
        noise_t = np.zeros([1,action_dim])
        
        a_t_original = actor.model.predict(s_t.reshape(1, s_t.shape[0]))
        noise_t[0][0] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][0],  0.0 , 1.00, 0.5)
        noise_t[0][1] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][1],  0.15 , 1.00, 0.05)

        a_t[0][0] = a_t_original[0][0] + noise_t[0][0]
        a_t[0][1] = a_t_original[0][1] + noise_t[0][1]

        done = ego_puck.step(a_t[0])
        s_t1 = ego_puck.nn_input
        r_t = ego_puck.step_reward
    
        buff.add(s_t, a_t[0], r_t, s_t1, done)      #Add replay buffer
        
        #Do the batch update
        batch = buff.getBatch(BATCH_SIZE)
        states = np.asarray([e[0] for e in batch])
        actions = np.asarray([e[1] for e in batch])
        rewards = np.asarray([e[2] for e in batch])
        new_states = np.asarray([e[3] for e in batch])
        dones = np.asarray([e[4] for e in batch])
        y_t = np.asarray([e[1] for e in batch])

        target_q_values = critic.target_model.predict([new_states, actor.target_model.predict(new_states)])  
       
        for k in range(len(batch)):
            if dones[k]:
                y_t[k] = rewards[k]
            else:
                y_t[k] = rewards[k] + GAMMA*target_q_values[k]
   
        if (train_indicator):
            loss += critic.model.train_on_batch([states,actions], y_t) 
            a_for_grad = actor.model.predict(states)
            grads = critic.gradients(states, a_for_grad)
            actor.train(states, grads)
            actor.target_train()
            critic.target_train()

        total_reward += r_t
        s_t = s_t1
    
        print("Episode", i, "Step", step, "Action", a_t, "Reward", r_t, "Loss", loss)
    
        step += 1
        if done:
            stop_sim()
            delete_objects(obstacles)
            break

    if np.mod(i, 10) == 0:
        if (train_indicator):
            print("Now we save model")
            actor.model.save_weights("actormodel.h5", overwrite=True)
            with open("actormodel.json", "w") as outfile:
                json.dump(actor.model.to_json(), outfile)

            critic.model.save_weights("criticmodel.h5", overwrite=True)
            with open("criticmodel.json", "w") as outfile:
                json.dump(critic.model.to_json(), outfile)

    print("TOTAL REWARD @ " + str(i) +"-th Episode  : Reward " + str(total_reward))
    print("Total Step: " + str(step))
    print("")

print("Finish.")

#if __name__ == "__main__":
#    playGame()
        
    

##%%--------------------------------------------------------------- 
## Main script
#    
#number_epucks = 6
#    
## Prompt user for vrep startup options    
#answer = 0
#while answer == 0:
#    
#    start_sim = False
#    start_vrep = input("Start a new vrep session, y/n? ")
#    if start_vrep == 'y':
#        # initialise vrep
#        headless = False        
#        headless_yn = input("Headless? y/n ")
#        if headless_yn == 'y':
#            headless = True
#                
#        clientID = initialise_vrep('epuck_arena_multi_spawn', 19998, headless)
#        
#        epucks = create_epucks(number_epucks, False)
#        answer = 1
#    elif start_vrep == 'n':
#        clientID = 0
#        answer = 1
#    else:
#        print("please type 'y' to start vrep or 'n' to use open vrep program\n")        
#
#ego_puck = epuck(0, True)
#
#
##%% Size of arena (map)
#    
#map_points = np.array([[0,0],[0,1.5],[2,1.5],[2,0]])
#
#        
##%% Create obstacles
#
#number_obstacles = 5
#obstacles = []
#
#for i in range(number_obstacles):
#    obs_type = np.random.randint(2)
#    obstacle_handle, obstacle_dimensions = create_obstacle(obs_type)
#    if obs_type == 0:
#        obstacles.append(cuboid(obstacle_handle, obstacle_dimensions))
#    else:
#        obstacles.append(cylinder(obstacle_handle, obstacle_dimensions))
#    obstacles[i].set_pos([-1.2,0.5])

##%% Randomise ePuck starting positions
#    
## reset ePuck positions
#for robot in epucks:
#    x_pos = -1 -int(robot.i)/10
#    robot.set_pos([x_pos, 1])
#
## Minimum initial spacing between robots (m)
#epuck_min_spacing = 0.2

## array of robot and obstacle radii, used to avoid placing obstacles on top of robots
#radii = [0.05]
#
## number of epucks in arena
#number_active_epucks = np.random.randint(number_epucks+1)
#print('Number of active ePucks: ' + str(number_active_epucks))
#
## Randomised starting position with minimum spacing and buffer dist from arena walls
#start_positions = calculate_positions(map_points, number_active_epucks+2, epuck_min_spacing)
#target_position = start_positions[len(start_positions)-1,:]
#
## Move dummy to highlight target position
#dummy_handle = get_dummy_handle()
#if dummy_handle == 0:
#    dummy_handle = create_dummy()
#target_dummy = dummy(dummy_handle,0.05)
#target_dummy.set_pos(target_position)

## Set epuck starting parameters
#for robot in epucks:
#    robot.set_vel(0,0)
#    radii.append(0.05)
#
#for i in range(number_active_epucks):
#    epucks[i].set_pos(start_positions[i,:])
#    epucks[i].set_ang(np.random.rand()*2*math.pi)

## number of obstacles in arena
#number_active_obstacles = np.random.randint(number_obstacles+1)
#print(f'Number of obstacles: {number_active_obstacles}')

## Caluclate positions for obstacles that dont impact robot positions
#for obs in obstacles:
#    radii.append(obs.radius)
        
# HEEEREEEEEEEEEEEEEEEEE

#for i in range(number_active_obstacles):
#    obstacles[i].position = extra_position(map_points, start_positions, radii, obstacles[i].radius)
#    obstacles[i].set_pos(obstacles[i].position)
#    obstacles[i].set_ang(np.random.rand()*2*math.pi)
#    
#
## Set ego_puck starting parameters
#ego_puck.set_vel(0,0)
#ego_puck.set_pos(start_positions[number_active_epucks,:])
#ego_puck.set_ang(np.random.rand()*2*math.pi)
#
## Set up arrays for NN input parameters recorded by ego_puck
#ego_puck.scan_dist = [None]*(number_epucks + number_obstacles)
#ego_puck.scan_ang = [None]*(number_epucks + number_obstacles)
#ego_puck.velocity_towards = [None]*(number_epucks + number_obstacles)
#ego_puck.velocity_perpendicular = [None]*(number_epucks + number_obstacles)
#ego_puck.object_radii = [None]*(number_epucks + number_obstacles)
#    
##%%
#
#start_sim = input("Start the simulation, press Enter (or n to cancel): ")
#if start_sim == 'n':
#    delete_objects(obstacles)
#    sys.exit()
#
#returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
#
#time.sleep(1)
#
#t = [None]*(number_epucks+1)
#
#stop_event = threading.Event()
#
#for i in range(number_active_epucks):
#    #robot_velocity = np.absolute(np.random.normal(0.1,0.02))
#    robot_velocity = np.random.gumbel(0.1,0.02)
#    print(f'Velocity of ePuck#{i+1}: {robot_velocity}')
#    
#    t[i] = threading.Thread(target = epucks[i].random_walk, args = (robot_velocity,))
#    
#t[number_active_epucks] = threading.Thread(target = ego_puck.move_to_target, args = (target_position,0.15,True,True,False,True))
#
#for i in range(number_active_epucks+1):
#    t[i].start()
#
#for i in range(number_active_epucks+1):
#    t[i].join()
#
#for i in range(number_epucks):
#    epucks[i].set_vel(0,0)
#
#
##%% Delete obstacles
#    
#delete_objects(obstacles)



