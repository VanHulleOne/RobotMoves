# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 13:32:57 2016

@author: lvanhulle
"""
import numpy as np

rotMat = 0
W = 0
X = 1
Y = 2
Z = 3

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

def outsideCylinder(*, centerX=0, centerY=0, centerZ=15, dia=16.8, height=55, vel=30):
    config = np.array([-1,1,1,1])
    startQuat = Quat(0.6532815, -0.2705981, -0.6532815, 0.2705981)
    
    circumf = np.pi*dia
    numRadialPoints = int(round(circumf/NOZ_DIA)) # Rounds to nearest integer, may over/under fill
    numHeightPoints = int((height//(circumf/4)))+1
    heightStep = height/numHeightPoints
    mainRotAngle = heightStep/circumf*2*np.pi
    stepOverRotAngle = NOZ_DIA/circumf*2*np.pi
    rad = dia/2
    print('Main:', mainRotAngle, 'Step:', stepOverRotAngle)
    print('Rad Points:', numRadialPoints, 'Height Points:', numHeightPoints)
    beadError = (numRadialPoints*NOZ_DIA - circumf)/numRadialPoints
    print('BeadError:', beadError)
    angle = 0
    currHeight = centerZ
    
    yield moveJ((rad+10,0,currHeight), startQuat, config, vel)
    yield moveJ((rad,0,currHeight), startQuat, config, vel)
    
    for i in range(numRadialPoints):
        # if i is odd we are moving down so subtract
        dir_ = (-1 if i%2 else 1)
        for j in range(numHeightPoints):
            angle += dir_ * mainRotAngle
            currHeight += dir_ * heightStep
            x = rad * np.cos(angle)
            y = rad * np.sin(angle)
            quat = startQuat.rotate_rad('z', angle)
            yield moveJ((x,y,currHeight), quat, config-[0,0,int(angle/(np.pi/2)),0], vel)
        angle += stepOverRotAngle
        x = rad * np.cos(angle)
        y = rad * np.sin(angle)
        quat = startQuat.rotate_rad('z', angle)
        yield moveJ((x,y,currHeight), quat, config-[0,0,int(angle/(np.pi/2)),0], vel)
    yield moveJ((x+10,y,currHeight), quat, config-[0,0,int(angle/(np.pi/2)),0], vel)
    yield moveJ((rad+10,0,currHeight), startQuat, config, vel)
    
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

    
def writePoints(points):
    points = list(points)
    with open('path.mod', 'w') as f:
        f.write('MODULE MainModule\n\tPROC main()\n')
        f.write('\t\tSetDO DO4_Heat_Nozzle, 1;\n')
        f.write('\t\tSetDO DO1_Auto_Mode, 1;\n')
        f.write(points[0])
        f.write(points[1])
        f.write('\t\tWaitRob \InPos;\n' +
                '\t\tSetDO DO5_Program_Feed, 1;\n')
        for line in points[2:-1]:
            f.write(line)
        f.write('\t\tWaitRob \InPos;\n'
                + '\t\tSetDO DO6_Between_Layer_Retract, 1;\n'
                + '\t\tSetDO DO5_Program_Feed, 0;\n')
        f.write(points[-1])
        f.write('\n\t\t! End Program codes\n' +
                '\t\tSetDO DO1_Auto_Mode, 0;\n' +
                '\t\tSetDO DO5_Program_Feed, 0;\n' +
                '\t\tSetDO DO3_Heat_Bed, 0;\n' +
                '\t\tSetDO DO4_Heat_Nozzle, 0;\n' +
                '\t\tSetDO DO6_Between_Layer_Retract, 0;\n'
                )
        f.write('\tENDPROC\nENDMODULE')
       
        
        
        
        
        
        