#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 11 14:32:38 2018

@author: greg
"""
import csv
import matplotlib.pyplot as plt

episode = []
success_ratio_radar = []
success_ratio_laser = []

with open('radar_dense-reward.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        episode.append(int(row[0]))
        success_ratio_radar.append(float(row[1]))
        
with open('laser_dense-reward.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        success_ratio_laser.append(float(row[1]))
        
plt.plot(episode, success_ratio_radar, 'r')
plt.plot(episode, success_ratio_laser, 'b')
  
        
        

    

