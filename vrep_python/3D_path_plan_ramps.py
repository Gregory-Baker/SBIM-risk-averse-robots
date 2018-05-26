#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 25 10:08:10 2018

@author: greg
"""
import math
import numpy as np
import matplotlib.pyplot as plt

node_spacing = 0.1

target = [0.3, 0.3]
target_index = [int(round(target[0]/node_spacing)), int(round(target[1]/node_spacing))]

start = [1.0, 0.5]
start_index = [int(round(start[0]/node_spacing)), int(round(start[1]/node_spacing))]

arena_length_x = 2
arena_length_y = 1.5

x_num = int(round(arena_length_x/node_spacing-1))
y_num = int(round(arena_length_y/node_spacing-1))

x_coords = np.linspace(0, arena_length_x, x_num+2)

y_coords = np.linspace(0, arena_length_y, y_num+2)

coords = np.zeros((x_num+2, y_num+2, 2))
euc_dist_to_targ = np.zeros((x_num+2, y_num+2))

fig1 = plt.figure()
ax1 = fig1.add_subplot(1,1,1)

fig2 = plt.figure()
ax2 = fig2.add_subplot(1,1,1)

for i in range(0, x_num+2):
    for j in range(0, y_num+2):
        coords[i,j,0] = x_coords[i]
        coords[i,j,1] = y_coords[j]
        ax1.scatter(coords[i,j,0], coords[i,j,1], c='k', marker='.', s=10)
        ax2.scatter(coords[i,j,0], coords[i,j,1], c='k', marker='.', s=10)
        euc_dist_to_targ[i,j] = math.sqrt((x_coords[i]-target[0])**2 + (y_coords[j]-target[1])**2)

arena = np.array([[0,0],[0, arena_length_y],[arena_length_x, arena_length_y],[arena_length_x, 0],[0,0]])

ax1.scatter(start[0], start[1], c='g', marker='8', s=200)
ax1.scatter(target[0], target[1], c='g', marker='X', s=200)
ax1.plot(arena[:,0], arena[:,1])

ax2.scatter(start[0], start[1], c='g', marker='8', s=200)
ax2.scatter(target[0], target[1], c='g', marker='X', s=200)
ax2.plot(arena[:,0], arena[:,1])

weights = np.zeros((x_num+2,y_num+2,9))


for i in range(1,x_num+1):
    for j in range(1,y_num+1):
        k = 0
        weight_sum = 0
        for x_delta in range(-1,2):
            for y_delta in range(-1,2):
                euc_x_index = i+x_delta
                euc_y_index = j+y_delta
                if ((euc_x_index in (0, x_num+1)) or (euc_y_index in (0, y_num+1)) or (x_delta == 0 and y_delta == 0)):
                    weights[i,j,k] = 0
                else:
                    weights[i,j,k] = 1 / (euc_dist_to_targ[euc_x_index, euc_y_index])**3
                    weight_sum += 1 / (euc_dist_to_targ[euc_x_index, euc_y_index])**3
                k += 1
        for k in range(0,9):
            weights[i,j,k] = weights[i,j,k] / weight_sum
                

x_int_start_ind = start_index[0]
y_int_start_ind = start_index[1]

path = [start_index]

while x_int_start_ind != target_index[0] or y_int_start_ind != target_index[1]:
    
    rand_num = np.random.rand()
    
    weights_cum = 0
    for k in range(0,9):
        weights_cum += weights[x_int_start_ind, y_int_start_ind, k]
        if rand_num < weights_cum:
            direction_index = k
            break
    
    k=0
    for x_delta in range(-1,2):
        for y_delta in range(-1,2):
            if k == direction_index:
                x_int_targ_ind = x_int_start_ind + x_delta
                y_int_targ_ind = y_int_start_ind + y_delta
            k += 1

    path.append([x_int_targ_ind, y_int_targ_ind])
    
    x_int_start_ind = x_int_targ_ind
    y_int_start_ind = y_int_targ_ind
    
path_np = np.array(path)
path_coords_orig = np.zeros((len(path_np),2))

for i in range(0,len(path_np)):
    path_coords_orig[i,0] = coords[path_np[i,0], path_np[i,1],0]
    path_coords_orig[i,1] = coords[path_np[i,0], path_np[i,1],1]

ax1.plot(path_coords_orig[:,0], path_coords_orig[:,1])

print(len(path))

for i in range(0, len(path_np)-2):
    for j in range(len(path_np)-1, i+1,-1):
        if j >= len(path_np)-1:
            continue
        if np.logical_and(*np.equal(path_np[i],path_np[j])):
            print(f'rows {i} and {j} are the same')
            for k in range(i, j):
                path_np = np.delete(path_np, (i), axis = 0)
                
path_coords = np.zeros((len(path_np),2))

for i in range(0,len(path_np)):
    path_coords[i,0] = coords[path_np[i,0], path_np[i,1],0]
    path_coords[i,1] = coords[path_np[i,0], path_np[i,1],1]
    
ax2.plot(path_coords[:,0], path_coords[:,1])

for i in range(0, len(path_np)-3):
    for j in range(len(path_np)-1, i+1,-1):
        if j >= len(path_np)-1:
            continue
        x_i = path_np[i,0]
        y_i = path_np[i,1]
        
        x_j = path_np[j,0]
        y_j = path_np[j,1]
        
        k = 0
        for x_delta in range(-1,2):
            for y_delta in range(-1,2):
                x_i_neigh = x_i + x_delta
                y_i_neigh = y_i + y_delta
                if x_i_neigh == x_j and y_i_neigh == y_j:
                    print(f'path point {i} and {j} are neighbours')
                    if weights[x_i, y_i, k] > 0.1:
                        print(f'taking a shortcut from point {i} to point {j}')
                        for k in range(i+1, j):
                            path_np = np.delete(path_np, (i+1), axis = 0)
                k += 1
                    
                
path_coords = np.zeros((len(path_np),2))

for i in range(0,len(path_np)):
    path_coords[i,0] = coords[path_np[i,0], path_np[i,1],0]
    path_coords[i,1] = coords[path_np[i,0], path_np[i,1],1]
    
ax2.plot(path_coords[:,0], path_coords[:,1])

plt.show()
            
for i in range(1,len(path_np)):
    int_target = path_coords[i]
    
    
        

    



