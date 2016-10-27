# -*- coding: utf-8 -*-
"""
Created on Thu Oct 27 16:36:09 2016

@author: lvanhulle
"""

vMax = 0.3183
vMin = 3.183e-4
acc = 1.1406e-4


steps = 0;
currStep = 0
vel = vMin
numcycles = 0
lastStep = 0
while(steps < 200):
    numcycles += 1
    vel += acc
    if(vel > vMax): vel = vMax
    currStep += vel
    if(currStep >= 1):
        currStep -= 1
        steps += 1
        print('Step: ', steps, ' Vel: ', vel, 'NumCycles: ', numcycles, ' diff: ', numcycles - lastStep)
        print('\tPrint Vel mm/min: ', 1/((numcycles - lastStep)/10000/60*38.197))
        lastStep = numcycles
        
    