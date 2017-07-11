# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 13:32:57 2016

@author: lvanhulle
"""
import numpy as np
from itertools import cycle
import math

rotMat = 0
W = 0
X = 1
Y = 2
Z = 3

x = 'x'
y = 'y'
z = 'z'

CW = -1
CCW = 1

MOVEL = 'MoveL'
MOVEJ = 'MoveJ'

NOZ_DIA = 0.5/(np.sqrt(2)/2)

class Quat:
    def __init__(self, w, x=None, y=None, z=None):
        self.PRECISION = 7
        try:
            self.w, self.x, self.y, self.z = w
        except Exception:
            self.w = w
            self.x = x
            self.y = y
            self.z = z

    def _key(self):
        return self.w, self.x, self.y, self.z

    def __iter__(self):
        return (i for i in self._key())
            
    def rotate_deg(self, axis, deg):
        rad = deg/360.0*2*np.pi
        return self.rotate_rad(axis, rad)

    def rotate_rad(self, axis, rad):
        """
        Rotation quaternion creation from here:
        http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-17-quaternions/
        """
#        rad *= -1
        axis = 'wxyz'.index(axis)
        quat = [0]*4
        quat[W] = np.cos(rad/2)
        quat[axis] = np.sin(rad/2)
        return Quat(quat)*self
        
    def __getitem__(self, index):
        return self._key()[index]
    
    def __mul__(self, other):
        """
        Multiplication from here:
        http://www.cprogramming.com/tutorial/3d/quaternions.html
        """
        quat = [0]*4
        quat[W] = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        quat[X] = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        quat[Y] = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        quat[Z] = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quat(quat)
        
    def __eq__(self, other):
        tolerance = 0.001
        sArray = np.array(list(self))
        oArray = np.array(list(other))
        res = sArray @ oArray
        if abs(res) - 1 < tolerance:
            return True
        return False
        
    
    def __str__(self):
        return '[' + ', '.join('{:0.7g}'.format(round(i,self.PRECISION)) for i in self) + ']'
    
    def __repr__(self):
        return 'Quat(' + ', '.join('{:0.7g}'.format(i) for i in self) + ')'

def move(type_, point, quat, config, speed):
    tool = 'tNozzle'
    workObj = 'wobjFlatPlat'

    return ('\t\t' + type_ + ' [[' +
            ', '.join(['{:0.3f}'.format(i) for i in point]) + '], ' + # X, Y, Z
            str(quat) + ', ' + # Quaternion
            str(list(config)) + # Configuration
            ', [' + ', '.join(['9E+09']*6) + ']], ' + # List of 9E+09 which are robot default for ignore
            'v{:.0f}, z0, '.format(speed) + # Speed
            tool + ', \\Wobj := ' + workObj + ';\n') # Tool and work object 
        
def moveL(point, quat, config, speed):
    return move(MOVEL, point, quat, config, speed)
    
def moveJ(point, quat, config, speed):
    return move(MOVEJ, point, quat, config, speed)

def outsideCylinder(*, centerX=0, centerY=0, centerZ=15, dia=16.8, length=None, endZ=None, stepOver=NOZ_DIA, helixAngleDeg=45, vel=30):
    if endZ is None and length is None:
        raise Exception('Must enter length or endZ')
    if endZ is not None and length is not None:
        raise Exception('Must enter only one length or endZ')
    if length is None:
        length = endZ - centerZ
    
    helixAngleRad = (helixAngleDeg/360)*2*np.pi
    stepOver = abs(stepOver)
    config = np.array([-1,1,1,1])
    startQuat = Quat(0.6532815, -0.2705981, -0.6532815, 0.2705981)
    maxOneMoveRotation = np.pi/2
    maxBeadOverlap = NOZ_DIA*0.01
    
    circumf = np.pi*dia
    rad = dia/2
    
    numRadialPoints = int(round(circumf*np.sin(helixAngleRad)/stepOver)) # Rounds to nearest integer, may over/under fill
    if numRadialPoints == 0:
        raise Exception('Helix Angle too flat')
    actStepOver = np.sin(helixAngleRad)*circumf/numRadialPoints
    beadOverlap = NOZ_DIA-actStepOver
    if beadOverlap > maxBeadOverlap:
        numRadialPoints -= int(numRadialPoints/abs(numRadialPoints)) # numRadialPoints can be positive or negative so use this
                                                                # trick to move it closer to zero if beadOverlap is too great
        if numRadialPoints == 0:
            raise Exception('Reduced numRadialPoints by 1 and now Helix Angle too flat')
    
    stepOverRotAngle = 2*np.pi/numRadialPoints
    
    idealStepHeight = np.tan(helixAngleRad)*maxOneMoveRotation*circumf/(2*np.pi)
    numHeightPoints = abs(int(length//idealStepHeight))+1
    heightStep = length/numHeightPoints
    mainRotAngle = heightStep/(circumf*np.tan(helixAngleRad))*2*np.pi
    
    
    print('Main Angle:', mainRotAngle, 'Stepover Angle:', stepOverRotAngle)
    print('Rad Points:', numRadialPoints, 'Height Points:', numHeightPoints)
    print('Bead Overlap:', beadOverlap)
    angle = 0
    currHeight = centerZ
    
    yield moveJ((rad+10,0,currHeight), startQuat, config, vel)
    yield moveJ((rad,0,currHeight), startQuat, config, vel)
    yield ('\t\tWaitRob \InPos;\n' +
            '\t\tSetDO DO6_Between_Layer_Retract, 0;\n' + 
            '\t\tSetDO DO5_Program_Feed, 1;\n')
    for i in range(abs(numRadialPoints)):
        # if i is odd we are moving down so subtract
        dir_ = (-1 if i%2 else 1)
        for j in range(numHeightPoints):
            angle += dir_ * mainRotAngle
            currHeight += dir_ * heightStep
            x = rad * np.cos(angle) + centerX
            y = rad * np.sin(angle) + centerY
            quat = startQuat.rotate_rad('z', angle)
            yield moveJ((x,y,currHeight), quat, config-[0,0,int(angle/(np.pi/2)),0], vel)
        angle += stepOverRotAngle
        x = rad * np.cos(angle) + centerX
        y = rad * np.sin(angle) + centerY
        quat = startQuat.rotate_rad('z', angle)
        yield moveJ((x,y,currHeight), quat, config-[0,0,int(angle/(np.pi/2)),0], vel)
        
    yield ('\t\tWaitRob \InPos;\n'
            + '\t\tSetDO DO6_Between_Layer_Retract, 1;\n'
            + '\t\tSetDO DO5_Program_Feed, 0;\n')    
    yield moveJ((x+10*np.cos(angle),y+10*np.sin(angle),currHeight), quat, config-[0,0,int(angle/(np.pi/2)),0], vel*2)    
    yield moveJ((rad+10,0,currHeight), startQuat, config, 100)
    
def mat2quat(m):
    q1 = np.sqrt(m[0,0] + m[1,1] + m[2,2] + 1)/2
    q2 = (np.sqrt(m[0,0] - m[1,1] - m[2,2] + 1)/2) * np.sign(m[2,1] - m[1,2])
    q3 = (np.sqrt(m[1,1] - m[0,0] - m[2,2] + 1)/2) * np.sign(m[0,2] - m[2,0])
    q4 = (np.sqrt(m[2,2] - m[0,0] - m[1,1] + 1)/2) * np.sign(m[1,0] - m[0,1])
    q = (q1, q2, q3, q4)
    r = sum(i**2 for i in q)
    return q, r
    
def quat2mat(q):
    m1 = np.array([[q[W], q[Z], -q[Y], q[X]],
                   [-q[Z], q[W], q[X], q[Y]],
                   [q[Y], -q[X], q[W], q[Z]],
                   [-q[X], -q[Y], -q[Z], q[W]]])

    m2 = np.array([[q[W], q[Z], -q[Y], -q[X]],
                   [-q[Z], q[W], q[X], -q[Y]],
                   [q[Y], -q[X], q[W], -q[Z]],
                   [q[X], q[Y], q[Z], q[W]]])

    return np.dot(m1, m2)[:3,:3]
    
def rotate(axis, deg):
    global rotMat
    rad = deg/360.0*2*np.pi
    cos = np.cos(rad)
    sin = np.sin(rad)
    if axis is 'x':
        matrix = np.array([[1, 0, 0],
                           [0, cos, -sin],
                           [0, sin, cos]])
    elif axis is 'y':
        matrix = np.array([[cos, 0, sin],
                           [0, 1, 0],
                           [-sin, 0, cos]])
    elif axis is 'z':
        matrix = np.array([[cos, -sin, 0],
                           [sin, cos, 0],
                           [0, 0, 1]])
    else:
        print('Invalid axis: ', axis)
    
    rotMat = matrix    
    origin = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
    return mat2quat(matrix.dot(origin))
    
    
def rotation_quat(axis, deg):
    axis = 'wxyz'.index(axis)
    fRad = deg/360.0*np.pi
    quat = [0]*4
    quat[W] = np.cos(fRad)
    quat[axis] = np.sin(fRad)
    return Quat(quat)

def circleHeightReduction_gen(layerHeight, circleRad):    
    for i in range(int(circleRad//layerHeight)):
        x = circleRad-layerHeight*i
        t = np.arccos(x/circleRad)
        z = circleRad*np.sin(t)
        yield z

def grips(startZbottom, startZtop, startDia, gripLength=25, layerHeight=0.2, radialThickness=5, filletRadius=25):
    numLayers = int(radialThickness//layerHeight)
    # base grip - grip closest to platform
    for zReduction, layerNumber in zip(circleHeightReduction_gen(layerHeight, filletRadius),
                                       range(numLayers)):
        yield '\n\t\t! Base Grip layer number ' + str(layerNumber + 1) + ' of ' + str(numLayers+1) + '\n'
        yield from outsideCylinder(centerZ=startZbottom,
                                   dia = startDia+layerNumber*layerHeight*2,
                                   stepOver=0.6,
                                   helixAngleDeg = 45 if layerNumber % 2 else -45,
                                   endZ = startZbottom + gripLength - zReduction,                                   
                                   )
        
    # Top grip - grip farthest from platform
    for zReduction, layerNumber in zip(circleHeightReduction_gen(layerHeight, filletRadius),
                                       range(numLayers)):
        yield '\n\t\t! Top Grip layer number ' + str(layerNumber + 1) + ' of ' + str(numLayers+1) + '\n'
        yield from outsideCylinder(centerZ=startZtop + zReduction,
                                   dia = startDia+layerNumber*layerHeight*2,
                                   stepOver=0.6,
                                   helixAngleDeg = 45 if layerNumber % 2 else -45,
                                   endZ = startZtop + gripLength,
                                   )

def multiLayer(*, angles = None, centerX=0, centerY=0, centerZ=10,
               initialDia=15.5, height=60, layerHeight = 0.2, vel=30, numLayers=1):
    if angles is None:
        angles = [45]
    try:
        list(angles)
    except Exception:
        angles = [angles]
    angles = cycle(np.array(angles))
    for layerNum, angle in zip(range(numLayers), angles):
        yield '\n\n\t\t!Layer number ' + str(layerNum+1) + ' of ' + str(numLayers) + '\n'
        yield from outsideCylinder(centerX=centerX, centerY=centerY, centerZ=centerZ,
                                   dia = initialDia + 2*layerHeight*(layerNum+1),
                                    stepOver=0.6,
                                    helixAngleDeg = angle,
                                    length=height,
                                    vel=vel)
    
    
def helix(diameter = 7.0, stepsPerRev = 4.0, height= 95.0, layerHeight = .2, 
    vel = 10.0, baseLayers = 2.0, centerX = 0.0, centerY = 0.0, centerZ = 0.0):
    
    if diameter < 5:
        raise Exception('Diameter must be >=5')
    if stepsPerRev < 2:
        raise Exception('Steps per revolution must be >=2')
    if stepsPerRev % 1 != 0:
        raise Exception('Must have an integer number of steps per revolution')
    if height < layerHeight*baseLayers:
        raise Exception('Height must be >= layerHeight * baseLayers')
    if layerHeight < .1:
        raise Exception('Minimum layer height is .1 mm')
    if vel < 10:
        raise Exception('Speed too slow')
    if vel > 100:
        raise Exception('Unsafe speed')
    if baseLayers < 0:
        raise Exception('Number of base layers cannot be negative')
    if baseLayers % 1 != 0 :
        raise Exception('Must have an integer number of base layers')

    nozzleWidth = .5        
    radius = diameter/2
    pathDiameter = diameter - (nozzleWidth)
    pathRadius = pathDiameter/2
    zHeight = centerZ + 5 + layerHeight                                        #+5 for clearance plane
    
    thetaStep = 0
    thetaTotal = 0
    
#    Not sure if these are needed       
#    config = np.array([-1,1,1,1])
#    startQuat = Quat(0, 0, -1, 0)

    if baseLayers > 0:                                                         #If there are base layers, make them


##########################################################################################################################################################
        

        if baseLayers % 2 == 1:                                                #If odd number of base layers: make one layer, then use even sets   
            numCircles = math.floor(radius/nozzleWidth)
            stepRadius = pathRadius                                            #stepRadius is used to track the tool poition in the following loop, and starts at the outermost tool position pathRadius

            #This block calculates the angluar change for one layer as theta total
            for l in range(numCircles):
                thetaStep = 2 * math.asin(nozzleWidth/(2*stepRadius))          #Finds the angular change per circle
                thetaTotal += thetaStep                                        #Total angular change for one layer

                stepRadius = stepRadius - nozzleWidth

            thetaStart = np.pi - thetaTotal                                    #Helix start position is always at 180 degrees, so taking the final- travel gives starting posiiton
            radiusCurrent = pathRadius - (nozzleWidth * (numCircles - 1))            #Starting from innermost circle

            yield ('\t\tMoveL Offs(pZero, ' +                                  #Move to clearance plane
            str(np.cos(thetaStart) * radiusCurrent) + ', ' +                      #X
            str(np.sin(thetaStart) * radiusCurrent) + ', ' +                      #Y 
            str(zHeight) +                                                     #Z
            '), v100, z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')         #velocity, zone, tool, and work object
            
            zHeight -= 5                                                       #Z value for starting position
            
            yield ('\t\tMoveL Offs(pZero, ' +                                      #move to start position
            str(np.cos(thetaStart) * radiusCurrent) + ', ' +                      #X
            str(np.sin(thetaStart) * radiusCurrent) + ', ' +                      #Y
            str(zHeight) +                                                     #Z
            '), v30, z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')          #velocity, zone, tool, and work object
            
            thetaCurrent = thetaStart

            for h in range(numCircles):                                        #Layer 1
                thetaStep = 2 * math.asin(nozzleWidth/(2*radiusCurrent))       #Finds the angular change per circle                
                thetaTravel = (2 * np.pi) - thetaStep
                
                #next two yields are the moveC command, split to update thetaCurrent
                yield ('\t\tMoveC Offs(pZero, ' + 
                str(np.cos(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #X
                str(np.sin(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #Y
                str(zHeight) + '), ' +                                     #Z
                'Offs(pZero, ' + 
                str(np.cos(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #X
                str(np.sin(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #Y
                str(zHeight) +                                             #Z
                '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool

                yield ('\t\tMoveC Offs(pZero, ' + 
                str(np.cos(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #X
                str(np.sin(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #Y
                str(zHeight) + '), ' +                                     #Z
                'Offs(pZero, ' + 
                str(np.cos(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #X
                str(np.sin(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #Y
                str(zHeight) +                                             #Z
                '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool


                thetaCurrent = thetaCurrent + thetaStep                    #Move theta current to find end theta, and strting theta for next circle 
    
                
                if h != (numCircles - 1):
                    radiusCurrent += nozzleWidth                               #Increase radius by one nozzle width for next smallest circle
                    yield ('\t\tMoveL Offs(pZero, ' + 
                    str(np.cos(thetaCurrent) * radiusCurrent) + ', ' +         #X
                    str(np.sin(thetaCurrent) * radiusCurrent) + ', ' +         #Y
                    str(zHeight) +                                             #Z
                    '), v' + str(int(vel)) +                                        #velcoity 
                    ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')        #Tool and work object

            zHeight += layerHeight

            if baseLayers > 2:
                for b in range(np.floor(baseLayers/2)):                        #Make pairs of base layers
                    thetaCurrent = np.pi - (2 * thetaTotal)                    #Helix start position is always at 180 degrees, so taking the final- travel gives starting posiiton
                    for l in range(numCircles):                                #Layer 2, 4, 6...
                        thetaStep = 2*math.asin(nozzleWidth/(2*radiusCurrent)) #Finds the angular change per circle                
                        thetaTravel = (2 * np.pi) - thetaStep
                
                        #next two yields are the moveC command, split to update thetaCurrent
                        yield ('\t\tMoveC Offs(pZero, ' + 
                        str(np.cos(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #X
                        str(np.sin(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #Y
                        str(zHeight) + '), ' +                                     #Z
                        'Offs(pZero, ' + 
                        str(np.cos(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #X
                        str(np.sin(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #Y
                        str(zHeight) +                                             #Z
                        '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool

                        yield ('\t\tMoveC Offs(pZero, ' + 
                        str(np.cos(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #X
                        str(np.sin(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #Y
                        str(zHeight) + '), ' +                                     #Z
                        'Offs(pZero, ' + 
                        str(np.cos(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #X
                        str(np.sin(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #Y
                        str(zHeight) +                                             #Z
                        '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool

                        thetaCurrent = thetaCurrent + thetaStep                    #Move theta current to find end theta, and strting theta for next circle 
                    
                    
                        if l != (numCircles - 1):                                    #dont do the moveL if this is the last circle
                            radiusCurrent -= nozzleWidth                       #Reduce radius by one nozzle width for next smallest circle
 
                            yield ('\t\tMoveL Offs(pZero, ' + 
                            str(np.cos(thetaCurrent) * radiusCurrent) + ', ' + #X
                            str(np.sin(thetaCurrent) * radiusCurrent) + ', ' + #Y
                            str(zHeight) +                                     #Z
                            '), v' + str(int(vel)) +                                #velcoity 
                            ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')#Tool and work object
                
                    zHeight += layerHeight
            
                    yield ('\t\tMoveL Offs(pZero, ' +                              #Move up one layer
                    str(np.cos(thetaCurrent) * radiusCurrent) + ', ' +         #X
                    str(np.sin(thetaCurrent) * radiusCurrent) + ', ' +         #Y
                    str(zHeight) +                                             #Z
                    '), v' + str(int(vel)) +                                        #velcoity 
                    ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')        #Tool and work object
            
                    for k in range(numCircles):                                #Layer 3, 5, 7...
                        thetaStep = 2*math.asin(nozzleWidth/(2*radiusCurrent)) #Finds the angular change per circle                
                        thetaTravel = (2 * np.pi) - thetaStep
                
                        #next two yields are the moveC command, split to update thetaCurrent
                        yield ('\t\tMoveC Offs(pZero, ' + 
                        str(np.cos(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #X
                        str(np.sin(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #Y
                        str(zHeight) + '), ' +                                     #Z
                        'Offs(pZero, ' + 
                        str(np.cos(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #X
                        str(np.sin(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #Y
                        str(zHeight) +                                             #Z
                        '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool

                        yield ('\t\tMoveC Offs(pZero, ' + 
                        str(np.cos(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #X
                        str(np.sin(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #Y
                        str(zHeight) + '), ' +                                     #Z
                        'Offs(pZero, ' + 
                        str(np.cos(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #X
                        str(np.sin(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #Y
                        str(zHeight) +                                             #Z
                        '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool


                        thetaCurrent = thetaCurrent + thetaStep                    #Move theta current to find end theta, and strting theta for next circle 
                                
                        if k != (numCircles - 1):
                            radiusCurrent += nozzleWidth                       #Increase radius by one nozzle width for next smallest circle
                            yield ('\t\tMoveL Offs(pZero, ' + 
                            str(np.cos(thetaCurrent) * radiusCurrent) + ', ' + #X
                            str(np.sin(thetaCurrent) * radiusCurrent) + ', ' + #Y
                            str(zHeight) +                                     #Z
                            '), v' + str(int(vel)) +                                #velcoity 
                            ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')#Tool and work object
            
                    zHeight += layerHeight
                    
                yield ('\t\tMoveL Offs(pZero, ' +                                  #Move up one layer
                str(np.cos(thetaCurrent) * radiusCurrent) + ', ' +             #X
                str(np.sin(thetaCurrent) * radiusCurrent) + ', ' +             #Y
                str(zHeight) +                                                 #Z
                '), v' + str(int(vel)) +                                            #velcoity 
                ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')            #Tool and work object

        
        
########################################################################################################################################################       
        
        if baseLayers % 2 == 0:                                                #If even number of base layers: use even sets   
            numCircles = math.floor(radius/nozzleWidth)
            stepRadius = pathRadius                                            #stepRadius is used to track the tool poition in the following loop, and starts at the outermost tool position pathRadius
        
            #This block calculates the angluar change for one layer as theta total
            for l in range(numCircles):

                
                thetaStep = 2 * math.asin(nozzleWidth/(2*stepRadius))          #Finds the angular change per circle
                thetaTotal += thetaStep                                        #Total angular change for one layer
                
                stepRadius = stepRadius - nozzleWidth

            thetaStart = np.pi - (2 * thetaTotal)                              #Helix start position is always at 180 degrees, so taking the final- travel gives starting posiiton

            yield ('\t\tMoveL Offs(pZero, ' +                                      #Move to clearance plane
            str(np.cos(thetaStart) * pathRadius) + ', ' +                      #X
            str(np.sin(thetaStart) * pathRadius) + ', ' +                      #Y 
            str(zHeight) +                                                     #Z
            '), v100, z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')         #velocity, zone, tool, and work object
            
            zHeight -= 5                                                       #Z value for starting position
            
            yield ('\t\tMoveL Offs(pZero, ' +                                      #move to start position
            str(np.cos(thetaStart) * pathRadius) + ', ' +                      #X
            str(np.sin(thetaStart) * pathRadius) + ', ' +                      #Y
            str(zHeight) +                                                     #Z
            '), v' + str(int(vel)) +                                                #velocity
            ' , z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')               #zone, tool, and work object
            
            thetaCurrent = thetaStart
            radiusCurrent = pathRadius

            for n in range(np.int(baseLayers/2)):                                      #Make pairs of layers
            
                for l in range(numCircles):                                    #Layer 1, 3, 5...
                    thetaStep = 2 * math.asin(nozzleWidth/(2*radiusCurrent))   #Finds the angular change per circle  
                    thetaTravel = (2 * np.pi) - thetaStep
                
                    #next two yields are the moveC command, split to update thetaCurrent
                    yield ('\t\tMoveC Offs(pZero, ' + 
                    str(np.cos(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #X
                    str(np.sin(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #Y
                    str(zHeight) + '), ' +                                     #Z
                    'Offs(pZero, ' + 
                    str(np.cos(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #X
                    str(np.sin(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #Y
                    str(zHeight) +                                             #Z
                    '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool

                    yield ('\t\tMoveC Offs(pZero, ' + 
                    str(np.cos(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #X
                    str(np.sin(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #Y
                    str(zHeight) + '), ' +                                     #Z
                    'Offs(pZero, ' + 
                    str(np.cos(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #X
                    str(np.sin(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #Y
                    str(zHeight) +                                             #Z
                    '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool


                    thetaCurrent = thetaCurrent + thetaStep                    #Move theta current to find end theta, and strting theta for next circle 
                                 
                                
                    if l != (numCircles - 1):                                        #dont do the moveL if this is the last circle
                        radiusCurrent -= nozzleWidth                           #Reduce radius by one nozzle width for next smallest circle
 
                        yield ('\t\tMoveL Offs(pZero, ' + 
                        str(np.cos(thetaCurrent) * radiusCurrent) + ', ' +     #X
                        str(np.sin(thetaCurrent) * radiusCurrent) + ', ' +     #Y
                        str(zHeight) +                                         #Z
                        '), v' + str(int(vel)) +                                    #velcoity 
                        ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')    #Tool and work object
                
                zHeight += layerHeight
            
                yield ('\t\tMoveL Offs(pZero, ' +                                  #Move up one layer
                str(np.cos(thetaCurrent) * radiusCurrent) + ', ' +             #X
                str(np.sin(thetaCurrent) * radiusCurrent) + ', ' +             #Y
                str(zHeight) +                                                 #Z
                '), v' + str(int(vel)) +                                            #velcoity 
                ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')            #Tool and work object
            
                for k in range(numCircles):                                    #Layer 2, 4, 6...
                    thetaStep = 2 * math.asin(nozzleWidth/(2*radiusCurrent))   #Finds the angular change per circle                
                    thetaTravel = (2 * np.pi) - thetaStep
                
                    #next two yields are the moveC command, split to update thetaCurrent
                    yield ('\t\tMoveC Offs(pZero, ' + 
                    str(np.cos(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #X
                    str(np.sin(thetaCurrent - .25 * thetaTravel) * radiusCurrent) + ', ' + #Y
                    str(zHeight) + '), ' +                                     #Z
                    'Offs(pZero, ' + 
                    str(np.cos(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #X
                    str(np.sin(thetaCurrent - .5 * thetaTravel) * radiusCurrent) + ', ' +         #Y
                    str(zHeight) +                                             #Z
                    '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool

                    yield ('\t\tMoveC Offs(pZero, ' + 
                    str(np.cos(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #X
                    str(np.sin(thetaCurrent - .75 * thetaTravel) * radiusCurrent) + ', ' + #Y
                    str(zHeight) + '), ' +                                     #Z
                    'Offs(pZero, ' + 
                    str(np.cos(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #X
                    str(np.sin(thetaCurrent - thetaTravel) * radiusCurrent) + ', ' +         #Y
                    str(zHeight) +                                             #Z
                    '), v' + str(int(vel)) + ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')                #Speed, zone, and tool

                    thetaCurrent = thetaCurrent + thetaStep                    #Move theta current to find end theta, and strting theta for next circle 
    
                            
                    if k != (numCircles - 1):
                        radiusCurrent += nozzleWidth                           #Increase radius by one nozzle width for next smallest circle
                        yield ('\t\tMoveL Offs(pZero, ' + 
                        str(np.cos(thetaCurrent) * radiusCurrent) + ', ' +     #X
                        str(np.sin(thetaCurrent) * radiusCurrent) + ', ' +     #Y
                        str(zHeight) +                                         #Z
                        '), v' + str(int(vel)) +                                    #velcoity 
                        ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')    #Tool and work object
                        
                zHeight += layerHeight
                
                yield ('\t\tMoveL Offs(pZero, ' +                                  #Move up one layer
                str(np.cos(thetaCurrent) * radiusCurrent) + ', ' +             #X
                str(np.sin(thetaCurrent) * radiusCurrent) + ', ' +             #Y
                str(zHeight) +                                                 #Z
                '), v' + str(int(vel)) +                                            #velcoity 
                ', z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')            #Tool and work object

#############################################################################################################################


    if(baseLayers) == 0:
        thetaStart = np.pi
        yield ('\t\tMoveL Offs(pZero, ' +                                      #Move to clearance plane
        str(np.cos(thetaStart) * pathRadius) + ', ' +                          #X
        str(np.sin(thetaStart) * pathRadius) + ', ' +                          #Y 
        str(zHeight) +                                                         #Z
        '), v100, z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')             #velocity, zone, tool, and work object
            
        zHeight -= 5                                                           #Z value for starting position
            
        yield ('\t\tMoveL Offs(pZero, ' +                                      #move to start position
        str(np.cos(thetaStart) * pathRadius) + ', ' +                          #X
        str(np.sin(thetaStart) * pathRadius) + ', ' +                          #Y
        str(zHeight) +                                                         #Z
        '), v' + str(int(vel)) + ' , z0, tNozzleAlCal, \Wobj := wobjAlWithButton;\n')    #velocity, zone, tool, and work object
 

    numLayers = np.round((height/layerHeight) - baseLayers)
    
    if numLayers > 0:
        angleIncrement = 360/(2*stepsPerRev)                                       #steps times 2 because with n arcs there are 2n points used to define it
        angleIncrementRad = (angleIncrement/360) * 2 * np.pi
    
        startingAngle = 180
        startingAngleRad = (startingAngle/360) * 2 * np.pi
        
        yield ('zHeight = ' + str(zHeight))
    
        yield ('\t\tFOR i FROM 1 TO ' + str(numLayers) + ' DO \n'+
        '\t\t\tTPWRITE "Layer " \\' + 'NUM:=i;\n')
    
    
        currentAngleRad = startingAngleRad
        for j in range(int(stepsPerRev)):
            yield ('\t\t\tMoveC Offs(pZero, '+     
            str(pathRadius * np.cos(currentAngleRad - angleIncrementRad)) + ',' +
            str(pathRadius * np.sin(currentAngleRad - angleIncrementRad)) + ',' + 
            'zHeight+' + str((layerHeight/(2*stepsPerRev)) * (2 * j)) + '), ' +
            
            'Offs(pZero, ' +  
            str(pathRadius * np.cos(currentAngleRad - 2 * angleIncrementRad)) + ',' +
            str(pathRadius * np.sin(currentAngleRad - 2 * angleIncrementRad)) + ',' + 
            'zHeight+' + str((layerHeight/(2*stepsPerRev)) * (2*j + 1)) + '), ' + 
                 
            'v' + str(int(vel)) +', ' + 
            'z0, ' +
            'tNozzleCorrected, ' + 
            '\Wobj := wobjAlWithButton;\n')
        
            zHeight += (layerHeight/stepsPerRev)
            currentAngleRad -= (2 * angleIncrementRad)

        yield '\t\t\tzHeight:= zHeight + ' + str(layerHeight) + ';\n'
        yield '\t\tENDFOR\n'
    
    yield '\t\tWaitRob \InPos;\n'
    yield '\t\tSetDO DO5_Program_Feed, 0;\n'
    yield '\t\tSetDO DO6_Between_Layer_Retract, 1;\n'
    yield '\t\tzHeight:= zHeight + 3;\n'
    yield ('\t\tMoveL Offs(pZero, 0, 0, zHeight), v' + str(int(vel)) + 
           ' , z0, tNozzleCorrected, \Wobj := wobjAlWithButton;\n')

    
        
        
        
def writePoints(points):
    points = list(points)
    with open('path.mod', 'w') as f:
        f.write('MODULE MainModule\n')
        f.write('\tVAR num zHeight;\n\n')
        f.write('\tPROC main()\n')
        f.write('\t\tSetDO DO4_Heat_Nozzle, 1;\n')
        f.write('\t\tWaitDI DI3_Nozzle_At_Temp, 1;\n')
        f.write('\t\tSetDO DO1_Auto_Mode, 1;\n')
        for line in points:
            f.write(line)
        f.write('\n\t\t! End Program codes\n' +
                '\t\tALL_STOP;\n' +
                '\tENDPROC\n\n'
                )
        f.write('\tPROC ALL_STOP()\n' +
                '\t\tSetDO DO1_Auto_Mode, 0;\n' +
                '\t\tSetDO DO5_Program_Feed, 0;\n' +
                '\t\tSetDO DO3_Heat_Bed, 0;\n' +
                '\t\tSetDO DO4_Heat_Nozzle, 0;\n' +
                '\t\tSetDO DO6_Between_Layer_Retract, 0;\n' +
                '\tENDPROC\n'
                )
        f.write('ENDMODULE'
                )
       
        
        
        
        
        
        