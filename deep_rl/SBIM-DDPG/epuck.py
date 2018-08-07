#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  6 17:32:14 2018

@author: greg
"""
import vrep
import numpy as np
import time
import math

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
        
        
        if ego_puck:
            self.laserSensor = [None]*self.number_laser_sensors
            self.laserPoint = [None]*self.number_laser_sensors
            self.laserDist = [None]*self.number_laser_sensors
            
            _, self.laserSensorHandle, _, _, _ = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_customizationscript, 'laserSensorHandles', [self.number_laser_sensors], [], [], emptyBuff, vrep.simx_opmode_blocking)

            for i in range(self.number_laser_sensors):
                _, self.laserSensor[i], self.laserPoint[i], _, _ = vrep.simxReadProximitySensor(clientID, self.laserSensorHandle[i], vrep.simx_opmode_streaming)
        
            self.distance_to_wall = [0]*4
            self.velocity_towards_wall[0]*4
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
        
        returnCode, self.linearVelocity, self.angularVelocity = vrep.simxGetObjectVelocity(clientID, self.handle, vrep.simx_opmode_buffer)
        
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
                          v_object: 'linear velocity of other object',
                          angle: 'angle to other object in coordinate system of ego_bot'
                          ):
        
        # translate to world coordinate frame
        alpha = self.angle + angle 
        
        velocity_towards = (self.linearVelocity[0] - v_object[0])*math.cos(alpha) + (self.linearVelocity[1] - v_object[1])*math.sin(alpha)
        velocity_perpendicular = (self.linearVelocity[1] - v_object[1])*math.cos(alpha) + (v_object[0] - self.linearVelocity[0])*math.sin(alpha)
        
        return velocity_towards, velocity_perpendicular
        
    
    def dist_ang_to_objects(self,
                            epucks: 'array containing other ePuck objects',
                            obstacles: 'array of obstacles'):
        
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
    
    
    def dist_to_walls(self,
                      map_points: 'works on square maps only'):
        
        arena_dims = [max(map_points[:,0]), max(map_points[:,1])]
        
        self.distance_to_wall[0] = self.position[0]
        self.distance_to_wall[1] = arena_dims[1] - self.position[1]
        self.distance_to_wall[2] = arena_dims[0] - self.position[0]
        self.distance_to_wall[3] = self.position[1]
        
    def velocity_inputs(self):
        
        self.velocity_inputs[0] = self.linearVelocity[0]
        self.velocity_inputs[1] = self.linearVelocity[1]
        self.velocity_inputs[2] = self.angularVelocity[2]
        

#-----------------------------------------------------------------------------

    def move_to_target(self, 
                       target: '[x y] metres', 
                       linear_velocity: 'm/s',
                       stop_at_target: 'bool',
                       force_sens = False,
                       laser_sens = False,
                       radar_sens = False,
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
            
            self.get_pos()
            self.get_ang()
            self.get_vel()
            
            if laser_sens:
                self.read_laser_sens()
                
            if radar_sens:
                self.dist_ang_to_objects(epucks, obstacles)
                self.dist_to_walls(map_points)
            
            if force_sens:
                self.read_force_sens()
                if self.forceMag < 100:
                    self.forceMagList.append(self.forceMag)
            
            dist_to_target, ang_to_target = self.dist_ang_to_target(target_np)
            
            angular_velocity = 2*ang_to_target

            self.set_vel(linear_velocity, angular_velocity)
                    
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
        
        